"""
heatpump_fsm.py - Waermepumpen-FSM auf Basis von FSMBase
========================================================
Konfiguriert fuer: LG ThermaV mit Modbus, 4-Sensor-Pufferspeicher,
                  3-Wege-Mischer mit Motorlaufzeit-Steuerung (PI-Regler),
                  Grundfos Heizkreispumpe (Kreis 2)

═══════════════════════════════════════════════════════════════════
PUFFERSPEICHER-ZONENKONZEPT
═══════════════════════════════════════════════════════════════════
  Oben  ┌-----------------------------┐  ← sensor_buffer_top   (oben)
        │  WW-Zone                    │    WW-Ziel: ww_target (z.B. 52°C), AUS wenn ≥ Ziel
        │                             │
  3/4   ├ - - - - - - - - - - - - - - │  ← sensor_buffer_mid_high (3/4)
        │                             │    PV-Ladung: Einlass < buffer_charge_inlet_temp
        │                             │               (40002=1: WP regelt auf Einlass 30003)
        │                             │    Boost:     Einlass < boost_inlet_temp
  1/2   ├=============================│  ← sensor_buffer_mid   (1/2)  ← Vorlauf-Stutzen
        │  Heizzone (einfach)         │    Heizung EIN  wenn 1/2 < Sollwert + buffer_drain_margin
        │  leichte Durchmischung      │    Heizung AUS  wenn 1/2 >= Sollwert + margin + hyst
  1/4   ├ - - - - - - - - - - - - - - │  ← sensor_buffer_bottom (1/4)
        │                             │    buffer_drain EIN wenn 1/2 >= Sollwert + margin + hyst
        │                             │    (1/4 = Ruecklaufzone, nur Vorcheck bei PV-Ladung)
  Unten └-----------------------------┘

  Heizzone ist UNGETEILT (1/4 bis 1/2). Vorlauf-Stutzen auf Hoehe 1/2.
  EIN- und AUS-Entscheidung beide am 1/2-Sensor (Vorlauf-Hoehe, direkter
  Bezug zum Heizkreis). 1/4 (Ruecklaufzone) bleibt waehrend Heizkreislauf
  naturgemaess kalt und taugt nicht als Schaltpunkt - dient nur als
  pumpenunabhaengiger Vorcheck in _should_charge_buffer.
  WW-Zone ist OBEN - WP laedt von unten hoch, Schichtung bleibt erhalten.

═══════════════════════════════════════════════════════════════════
MISCHER (3-Wege, ESPHome time_based cover)
═══════════════════════════════════════════════════════════════════
  cover.lg_thermav_circuit_2_1e6134_rl_mischer
  100% = Vorlauf kommt vollstaendig aus Pufferspeicher (heiss)
  0%   = Vorlauf ist vollstaendig Ruecklauf (kalt)
  Laufzeit: 123s (+ 2s Ueberfahren bei Endpositionen)

═══════════════════════════════════════════════════════════════════
GRUNDFOS PUMPE (Heizkreis 2)
═══════════════════════════════════════════════════════════════════
  EIN:  heating, heating_forced, buffer_drain
        buffer_charge / buffer_charge_boost: EIN nur wenn AT < heating_threshold
        (Parallelheizung im Winter, AUS im Sommer via _update_charge_heating)
  AUS:  idle, standby, hot_water
        buffer_charge / buffer_charge_boost bei AT >= heating_threshold

═══════════════════════════════════════════════════════════════════
MISCHER PI-REGLER (velocity-form)
═══════════════════════════════════════════════════════════════════
  Aktiv in: heating, heating_forced, buffer_drain,
            buffer_charge / buffer_charge_boost (wenn AT < heating_threshold)
  Formel:   delta = kp*(e - e_prev) + ki*e   (velocity-form, kein Integral-Akkumulator)
  Step-Limit: delta wird auf ±mixer_max_step_pct (~4%) geclampt
              (verhindert Ueberschwingen bei 123s-Stellantrieb, 5s-Takt)
  Anti-Windup: implizit durch Position-Clamp (0–100%) und Step-Limit
  Sensor-Cache: letzter gueltiger VL-Wert wird gehalten bei Sensorausfall;
                PI pausiert nur wenn noch kein Cache vorhanden (Startup).

═══════════════════════════════════════════════════════════════════
ZUSTAENDE
═══════════════════════════════════════════════════════════════════
  idle                -> WP aus, Mischer geschlossen, Pumpe AUS
                         Nur wenn AT >= heating_threshold (keine Heizanforderung)

  heating             -> Heizbetrieb nach Heizkurve, WP EIN, Mischer PI-Regler, Pumpe EIN
                         EIN:  AT < heating_threshold UND Puffer 1/2 < threshold_on
                         AUS:  Puffer 1/2 >= threshold_off (= threshold_on + buffer_drain_hyst)
                               ODER AT >= heating_threshold

  heating_forced      -> Verdichterschutz-Mindestlaufzeit, WP EIN, Mischer PI-Regler, Pumpe EIN
                         Einstieg: heating beendet, Kompressor lief < min_runtime_minutes
                                   UND Kompressor war tatsaechlich an (_compressor_on_since != None)
                         Kreis 1 (40003): EWMA(WP-Ausgang) + forced_setpoint_offset + Kreis1-Offset
                                          Hard-Limit: heat_curve_vl_low (Max-Auslegung WP)
                                          Ratchet: steigt nur, faellt nicht waehrend Mindestlaufzeit
                         Kreis 2 (40006): normaler Heizkurven-Sollwert (kein Einfluss auf WP-Regelung)
                         Austritt (Zeit abgelaufen + Heizbedarf):    -> heating (Kreis1 Ramp-Down -1°C/min)
                         Austritt (Zeit abgelaufen + kein Heizbedarf): -> standby
                         Neustart-Restore: Timer sofort abgelaufen (_forced_end_time = now())

  hot_water           -> Warmwasser-Vorrang, WP EIN (WW-Modus), Pumpe AUS

  buffer_charge       -> PV-Ueberschuss laedt Puffer, WP EIN, 40002=1 (Einlass-Regelung)
                         Pumpe/Mischer: AT-abhaengig via _update_charge_heating
                         Einlass-Ziel: buffer_charge_inlet_temp (50°C) auf Sensor 30003
                         Exit: Einlass >= 50°C (Puffer voll) ODER PV zu schwach

  buffer_charge_boost -> PV + Heizstab (40010=boost_energy_state), 40002=1 bleibt
                         Pumpe/Mischer: AT-abhaengig via _update_charge_heating
                         Einlass-Ziel: boost_inlet_temp (55°C)
                         Startet wenn: PV >= boost_pv_min_w UND Einlass >= boost_inlet_min_c (45°C)
                         Exit: Einlass >= boost_inlet_temp ODER PV zu schwach

  buffer_drain        -> Puffer heizt Haus direkt, WP AUS, Mischer PI-Regler, Pumpe EIN
                         EIN:  AT < heating_threshold UND Puffer 1/2 >= threshold_off
                               (threshold_off = threshold_on + buffer_drain_hyst)
                         AUS:  Puffer 1/2 < threshold_on     -> heating
                               AT >= heating_threshold + heating_hyst -> idle
                               WW / PV-Vorrang

  standby             -> Verdichterschutz-Pause nach WP-Lauf, Pumpe AUS
                         Dauer: standby_minutes

  buffer_drain/heating Schwellen:
    threshold_on  = _calc_flow_setpoint(ohne PV-Korrektur) + buffer_drain_margin
    threshold_off = threshold_on + buffer_drain_hyst
    Kein PV-Anteil in den Schwellen (verhindert PV-Jitter-Takt)

═══════════════════════════════════════════════════════════════════
ZUSTANDSUEBERGAENGE (vereinfacht)
═══════════════════════════════════════════════════════════════════

  AT kalt + Puffer warm:   idle / standby --> buffer_drain --> heating
                                                                   │
  AT kalt + Puffer kalt:   idle / standby ----------------------> heating
                                                                   │
  Puffer 1/2 voll:         heating -----------------------------> heating_forced / standby
  Komp. intern gestoppt:   heating -----------------------------> buffer_drain (wenn Puffer warm genug)
  WW-Fenster:              (jeder Zustand) ----------------------> hot_water
  PV-Ueberschuss:           idle / standby / buffer_drain --------> buffer_charge

  buffer_drain -> idle:
    Einstieg bei AT < heating_threshold UND Puffer 1/2 >= threshold_off
    Austritt  bei AT >= heating_threshold UND Puffer 1/2 < threshold_off

═══════════════════════════════════════════════════════════════════
SCHUTZMECHANISMEN
═══════════════════════════════════════════════════════════════════
  E-Stop (fsm_base):
    > estop_max_transitions Wechsel in estop_window_seconds ->
    -> Dry-Run + HA persistent_notification

  Verdichter-Takt-Schutz (_check_compressor_cycling):
    > cycling_max_starts Starts in cycling_window_minutes ->
    -> Dry-Run + HA persistent_notification
    Frühwarnung bei max_starts - 2

  Neustart-Retention:
    FSM-Zustand aus input_select wiederhergestellt.
    heating_forced: _forced_end_time = now() (Timer sofort abgelaufen nach Neustart)
    _initial_enter_done Guard verhindert doppelten on_enter-Aufruf.

  Sensor-Fallback:
    AT: letzter gueltiger Wert gecacht; Startup-Default 30°C (kein Heizbedarf)
    VL: letzter gueltiger Wert gecacht; PI pausiert bei Startup ohne Cache
    _compressor_on_since = None -> kein heating_forced (kein Verdichter zu schuetzen)

═══════════════════════════════════════════════════════════════════
DRY-RUN MODUS (Simulation)
═══════════════════════════════════════════════════════════════════
  input_boolean.hp_dry_run = EIN -> FSM rechnet, kein Modbus/Cover-Write
"""

from fsm_base import FSMBase, Transition
import collections
import datetime
import os


class HeatpumpFSM(FSMBase):

    # -- Fallback-Standardwerte ------------------------------------------------
    _DEFAULTS = {
        "heat_curve_at_high": 16.0,
        "heat_curve_vl_high": 27.0,
        "heat_curve_at_low": -15.0,
        "heat_curve_vl_low": 40.0,
        "pv_correction_per_kw": 0.2,
        "buffer_drain_margin": 3.0,
        "buffer_drain_hyst": 2.0,
        "ww_target_temp": 51.0,
        "pv_buffer_min_kwh": 5.0,
        "hw_morning_weekday_h": 5,
        "hw_morning_weekday_m": 30,
        "hw_morning_weekend_h": 6,
        "hw_morning_weekend_m": 30,
        "hw_evening_h": 21,
        "hw_evening_m": 0,
        "hw_duration_minutes": 60,
        "mixer_kp": 0.5,
        "mixer_ki": 0.1,
        "mixer_interval_s": 5.0,
        "mixer_max_step_pct": 4.0,
        "mixer_warmstart_position": 20.0,
        # velocity-form PI: kein Integral-Akkumulator noetig
        "min_runtime_minutes": 45.0,
        "standby_minutes": 3.0,
        "forced_setpoint_offset": 1.0,
        "forced_lp_alpha": 0.2,
        "cycling_window_minutes": 60.0,
        "cycling_max_starts": 6,
        "circuit1_offset": 1.0,
        "boost_energy_state": 5,
        "boost_pv_min_w": 3500.0,
        "boost_pv_off_w": 1500.0,
        "pv_buffer_max_kwh": 15.0,
        "heating_hyst": 1.0,  # Hysterese buffer_drain -> idle (°C ueber Heizgrenze)
        # Einlass-Regelung (40002=1): Sollwerte fuer Modbus-Register 40003
        "buffer_charge_inlet_temp": 50.0,  # Einlass-Ziel buffer_charge (ganzer Puffer >= 50°C)
        "boost_inlet_temp": 55.0,  # Einlass-Ziel buffer_charge_boost (+ Heizstab)
        "boost_inlet_min_c": 45.0,  # Boost startet erst wenn Einlass >= X°C
        # Hysterese nach erfolgreicher Pufferladung:
        # Neustart erst wenn Einlass unter (buffer_charge_inlet_temp - buffer_charge_hyst) faellt.
        # Verhindert Takt wenn Einlass nach Ladungsstopp knapp unter Zieltemperatur bleibt.
        "buffer_charge_hyst": 5.0,
        # Mindestlaufzeit in buffer_charge vor temperaturbasiertem Ausstieg.
        # Verhindert Fehlausstieg durch heisses Rohrrestwasser (Sommer, Grundfos AUS):
        # WP-interne Pumpe startet, liest ~52°C Restwasser -> sofortiger Exit wäre falsch.
        "buffer_charge_min_minutes": 5.0,
        "setpoint_ewma_alpha": 0.1,
    }

    @property
    def initial_state(self) -> str:
        return "idle"

    def define_states(self) -> list[str]:
        return [
            "idle",
            "heating",
            "heating_forced",
            "hot_water",
            "buffer_charge",
            "buffer_charge_boost",
            "buffer_drain",  # Puffer heizt Haus, WP aus - Ueberbrueckung bis WP noetig
            "standby",
        ]

    def initialize(self):
        try:
            mtime = os.path.getmtime(__file__)
            mtime_str = datetime.datetime.fromtimestamp(mtime).strftime(
                "%Y-%m-%d %H:%M:%S"
            )
        except Exception:
            mtime_str = "unbekannt"
        self.log(f"[HP] Code-Datei: {mtime_str}", level="WARNING")

        # -- Sensoren ----------------------------------------------------------
        self._sensor_outdoor_temp = self.args["sensor_outdoor_temp"]
        self._sensor_outdoor_temp_1h = self.args.get(
            "sensor_outdoor_temp_1h", "sensor.aussentemperatur_mittelwert_1h"
        )
        self._sensor_pv_power_w = self.args["sensor_pv_power_w"]
        self._sensor_pv_energy_kwh = self.args["sensor_pv_energy_kwh"]
        self._sensor_flow_temp = self.args["sensor_flow_temp"]
        self._sensor_return_temp = self.args.get("sensor_return_temp", "")
        self._sensor_wp_leaving_temp = self.args.get("sensor_wp_leaving_temp", "")
        self._sensor_buffer_bottom = self.args["sensor_buffer_bottom"]
        self._sensor_buffer_mid = self.args["sensor_buffer_mid"]
        self._sensor_buffer_mid_high = self.args["sensor_buffer_mid_high"]
        self._sensor_buffer_top = self.args["sensor_buffer_top"]
        # Wassereinlasstemperatur WP (Modbus 30003) - Regelgroesse bei 40002=1
        self._sensor_wp_inlet_temp = self.args.get(
            "sensor_wp_inlet_temp", "sensor.wassereinlasstemperatur_30003"
        )

        # -- Steuerung ---------------------------------------------------------
        self._switch_heating = self.args.get(
            "switch_heating", "switch.heizungsschalter_10001"
        )
        self._switch_hot_water = self.args.get(
            "switch_hot_water", "switch.warmwasserschalter_10002"
        )
        self._input_setpoint_circuit1 = self.args.get(
            "input_setpoint_circuit1", "input_number.thermav_40003"
        )
        self._input_setpoint_circuit2 = self.args.get(
            "input_setpoint_circuit2", "input_number.thermav_40006"
        )
        self._input_setpoint_ww = self.args.get(
            "input_setpoint_ww", "input_number.thermav_40009"
        )
        self._binary_compressor = self.args.get(
            "binary_compressor", "binary_sensor.kompressor_20004"
        )
        self._binary_wp_pump = self.args.get(
            "binary_wp_pump", "binary_sensor.umwalzpumpe_20002"
        )
        self._cover_mixer = self.args.get("cover_mixer", "")
        self._mixer_full_travel_s = float(self.args.get("mixer_full_travel_s", 123))
        # Modbus-Verbindung fuer direkten Write (40002 Steuermethode)
        self._modbus_hub = self.args.get("modbus_hub", "modbus_waveshare")
        self._modbus_slave = int(self.args.get("modbus_slave", 1))

        # -- Grundfos Pumpe ----------------------------------------------------
        self._switch_circuit2_pump = self.args.get("switch_circuit2_pump", "")

        # -- Silent Mode -------------------------------------------------------
        self._switch_silent_mode = self.args.get(
            "switch_silent_mode", "switch.silent_mode_10003"
        )
        self._silent_mode_before_boost: bool | None = (
            None  # gespeicherter Zustand vor Boost
        )

        # -- Dry-Run -----------------------------------------------------------
        self._dry_run_entity = self.args.get(
            "dry_run_entity", "input_boolean.hp_dry_run"
        )

        # -- Dashboard-Visualisierung ------------------------------------------
        self._input_pi_position = self.args.get(
            "input_pi_position", "input_number.hp_mischer_pi_position"
        )

        # -- Heizstab-Boost ----------------------------------------------------
        self._input_energy_state = self.args.get(
            "input_energy_state", "input_number.thermav_40010"
        )
        self._binary_defrost = self.args.get(
            "binary_defrost", "binary_sensor.abtauen_20005"
        )
        self._binary_aux_heater_1 = self.args.get(
            "binary_aux_heater_1", "binary_sensor.zusatzheizung_1_20011"
        )
        self._binary_aux_heater_2 = self.args.get(
            "binary_aux_heater_2", "binary_sensor.zusatzheizung_2_20012"
        )
        self._binary_werktag = self.args.get("binary_werktag", "binary_sensor.werktag")
        self._boost_window_start = self.args.get("boost_window_start", "10:00")
        self._boost_window_end = self.args.get("boost_window_end", "18:00")

        # -- HA-Parameter Prefix -----------------------------------------------
        self._ha_param_prefix = self.args.get("ha_param_prefix", "input_number.hp_")

        # -- Interne Zustaende --------------------------------------------------
        self._pi_prev_error = 0.0  # velocity-form PI: vorheriger Fehler
        self._pi_position = (
            50.0  # wird in on_enter_heating/buffer_drain aus Cover gelesen
        )
        self._mixer_handle = None
        self._forced_lp_value = None
        self._decay_start_time = None   # Zeitstempel: Beginn Phase-2-Absenkung
        self._decay_lp_start = None     # LP-Wert beim Start der Absenkung
        self._compressor_on_since = None
        self._forced_end_time = None  # absoluter Zeitstempel: Ende der Mindestlaufzeit
        self._hw_window_active = False
        self._ramp_c1_current = (
            None  # aktuelle Kreis1-Position waehrend Ramp-Down (None = inaktiv)
        )
        self._ramp_c1_handle = None  # AppDaemon-Timer-Handle fuer Kreis1-Ramp-Down
        self._buffer_fully_charged = (
            False  # Hysterese: True nach erfolgreicher Pufferladung
        )
        self._setpoint_ewma: float | None = None  # EWMA des fertigen Heizkurven-Sollwerts
        self._last_forced_c1: float | None = (
            None  # Ratchet: nur steigende Werte in Mindestlaufzeit
        )
        self._outdoor_temp_cache: float | None = None  # letzter gueltiger AT-EMA-Wert
        self._outdoor_temp_1h_cache: float | None = None  # letzter gueltiger AT-1h-Wert
        self._flow_temp_cache: float | None = None  # letzter gueltiger VL-Wert
        self._compressor_starts: collections.deque = collections.deque()  # Takt-Schutz
        self._last_wp_mode_log: str | None = None  # Deduplizierung: letzter Modbus-Log
        self._last_pi_position: float | None = None  # letzte PI-Gleichgewichtsposition vor Schliessen
        self._mixer_was_closed: bool = False  # _mixer_full_close() wurde aufgerufen; Cover evtl. noch in Bewegung

        super().initialize()

        # Energiezustand beim Start auf Normal setzen
        self._set_energy_state(2)

        # Puffer-Hysterese nach Neustart wiederherstellen:
        # Wenn Puffer 1/4 noch warm genug, war Puffer kuerzlich geladen.
        restart_temp = self._buffer_charge_inlet_temp - self._get_param("buffer_charge_hyst")
        if self._buffer_bottom() >= restart_temp:
            self._buffer_fully_charged = True
            self.log(
                f"[HP] Puffer-Hysterese wiederhergestellt: Puffer 1/4 {self._buffer_bottom():.1f}C "
                f">= {restart_temp:.1f}C",
                level="INFO",
            )

        # WW-Fenster nach Neustart wiederherstellen falls gerade aktiv
        self._restore_hw_window_if_active()

        # Periodischer Heartbeat-Log alle 10 Minuten
        self.run_every(self._log_state_heartbeat, "now", 600)

    # -------------------------------------------------------------------------
    # Parameter-Lesehilfen
    # -------------------------------------------------------------------------

    def _get_param(self, key: str) -> float:
        entity_id = f"{self._ha_param_prefix}{key}"
        try:
            raw = self.get_state(entity_id)
            if raw not in (None, "unavailable", "unknown", ""):
                return float(raw)
        except Exception:
            pass
        apps_key = self._defaults_to_apps_key(key)
        if apps_key in self.args:
            return float(self.args[apps_key])
        return float(self._DEFAULTS.get(key, 0.0))

    def _get_time_param(self, key_h: str, key_m: str) -> str:
        h = int(self._get_param(key_h))
        m = int(self._get_param(key_m))
        return f"{h:02d}:{m:02d}"

    # apps.yaml verwendet bis auf wenige Ausnahmen denselben Schluesselnamen wie
    # die internen _DEFAULTS-Keys. Nur abweichende Namen hier auflisten.
    _APPS_KEY_EXCEPTIONS: dict[str, str] = {}

    @classmethod
    def _defaults_to_apps_key(cls, key: str) -> str:
        return cls._APPS_KEY_EXCEPTIONS.get(key, key)

    # -- Convenience-Properties ------------------------------------------------

    @property
    def _heating_threshold(self) -> float:
        return self._get_param("heat_curve_at_high")

    @property
    def _heating_hyst(self) -> float:
        """Hysterese-Band fuer buffer_drain -> idle. Austritt erst bei AT >= threshold + hyst."""
        return self._get_param("heating_hyst")

    def _buffer_drain_margin(self) -> float:
        return self._get_param("buffer_drain_margin")

    def _buffer_drain_threshold_on(self) -> float:
        """WP-Start-Schwelle (heating EIN): AT-Heizkurve ohne PV-Korrektur + Margin.
        PV-Korrektur bewusst ausgelassen: Pufferkapazitaet haengt nicht von PV ab,
        PV-Jitter wuerde sonst die Schwelle zittern lassen und Taktbetrieb riskieren."""
        return (
            self._calc_flow_setpoint(with_pv_correction=False)
            + self._buffer_drain_margin()
        )

    def _buffer_drain_threshold_off(self) -> float:
        """buffer_drain-Schwelle (Entladen statt WP): on-Schwelle + Hysterese.
        Erzeugt totes Band zwischen EIN und AUS - verhindert Takt an der Grenze."""
        return self._buffer_drain_threshold_on() + self._get_param("buffer_drain_hyst")

    @property
    def _ww_target(self) -> float:
        return self._get_param("ww_target_temp")

    @property
    def _buffer_charge_inlet_temp(self) -> float:
        """Einlass-Sollwert (40002=1) fuer buffer_charge. Ganzer Puffer >= X°C."""
        return self._get_param("buffer_charge_inlet_temp")

    @property
    def _boost_inlet_temp(self) -> float:
        """Einlass-Sollwert (40002=1) fuer buffer_charge_boost (WP + Heizstab)."""
        return self._get_param("boost_inlet_temp")

    @property
    def _boost_inlet_min_c(self) -> float:
        """Mindest-Einlass fuer Boost-Aktivierung (Boost startet erst wenn Puffer bereits warm)."""
        return self._get_param("boost_inlet_min_c")

    @property
    def _buffer_charge_hyst(self) -> float:
        """Hysterese nach erfolgreicher Pufferladung.
        Neustart erst wenn Einlass < (buffer_charge_inlet_temp - buffer_charge_hyst).
        """
        return self._get_param("buffer_charge_hyst")

    @property
    def _pv_buffer_min_kwh(self) -> float:
        return self._get_param("pv_buffer_min_kwh")

    @property
    def _pv_buffer_max_kwh(self) -> float:
        return self._get_param("pv_buffer_max_kwh")

    @property
    def _pv_correction_per_kw(self) -> float:
        return self._get_param("pv_correction_per_kw")

    @property
    def _standby_minutes(self) -> float:
        return self._get_param("standby_minutes")

    @property
    def _min_runtime_minutes(self) -> float:
        return self._get_param("min_runtime_minutes")

    @property
    def _forced_setpoint_offset(self) -> float:
        return self._get_param("forced_setpoint_offset")

    @property
    def _forced_lp_alpha(self) -> float:
        return self._get_param("forced_lp_alpha")

    @property
    def _circuit1_offset(self) -> float:
        return self._get_param("circuit1_offset")

    @property
    def _mixer_kp(self) -> float:
        return self._get_param("mixer_kp")

    @property
    def _mixer_ki(self) -> float:
        return self._get_param("mixer_ki")

    @property
    def _mixer_interval_s(self) -> float:
        return self._get_param("mixer_interval_s")

    @property
    def _hw_duration_minutes(self) -> int:
        return int(self._get_param("hw_duration_minutes"))

    @property
    def _boost_energy_state(self) -> int:
        try:
            raw = self.get_state("input_select.hp_boost_energy_state")
            if raw and raw not in ("unavailable", "unknown"):
                return int(raw.split(" ")[0])
        except Exception:
            pass
        return int(self._DEFAULTS.get("boost_energy_state", 5))

    @property
    def _boost_pv_min_w(self) -> float:
        return self._get_param("boost_pv_min_w")

    @property
    def _boost_pv_off_w(self) -> float:
        return self._get_param("boost_pv_off_w")

    @property
    def _hw_morning_weekday(self) -> str:
        return self._get_datetime_time("input_datetime.hp_hw_morning_weekday", "05:30")

    @property
    def _hw_morning_weekend(self) -> str:
        return self._get_datetime_time("input_datetime.hp_hw_morning_weekend", "07:30")

    @property
    def _hw_evening(self) -> str:
        return self._get_datetime_time("input_datetime.hp_hw_evening", "21:00")

    def _get_datetime_time(self, entity_id: str, default: str) -> str:
        try:
            raw = self.get_state(entity_id)
            if raw and raw not in ("unavailable", "unknown"):
                return raw[:5]
        except Exception:
            pass
        return default

    # -------------------------------------------------------------------------
    # Dry-Run
    # -------------------------------------------------------------------------

    def _is_dry_run(self) -> bool:
        try:
            return self.get_state(self._dry_run_entity) == "on"
        except Exception:
            return False

    # -------------------------------------------------------------------------
    # Uebergaenge
    # -------------------------------------------------------------------------

    def define_transitions(self) -> list[Transition]:
        return [
            # WW hat Vorrang (aus allen aktiven Zustaenden)
            Transition(
                "idle", "hot_water", self._need_hot_water, label="WW-Fenster geoeffnet"
            ),
            Transition(
                "heating",
                "hot_water",
                self._need_hot_water,
                label="WW verdraengt Heizung",
            ),
            Transition(
                "buffer_charge",
                "hot_water",
                self._need_hot_water,
                label="WW verdraengt PV-Ladung",
            ),
            Transition(
                "buffer_charge_boost",
                "hot_water",
                self._need_hot_water,
                label="WW verdraengt Boost",
            ),
            Transition(
                "buffer_drain",
                "hot_water",
                self._need_hot_water,
                label="WW verdraengt Puffer-Entladen",
            ),
            # WW fertig -> Standby
            Transition(
                "hot_water",
                "standby",
                lambda: not self._hw_window_active,
                label="WW-Fenster geschlossen",
            ),
            # Heizung EIN (Puffer 1/2 kalt -> WP muss ran)
            Transition(
                "idle",
                "heating",
                self._need_heating,
                label="Heizbedarf (1/2-Sensor kalt)",
            ),
            Transition(
                "standby",
                "heating",
                lambda: self._standby_done() and self._need_heating(),
                label="Standby vorbei, Heizung noetig",
            ),
            Transition(
                "buffer_drain",
                "heating",
                self._need_heating,
                label="Puffer erschoepft - WP startet",
            ),
            # Heizung AUS -> Mindestlaufzeit oder Standby
            # Exit: AT 1h (schnell) statt EMA - reagiert sofort wenn es warm wird.
            # Entry (idle/standby -> heating) bleibt EMA-basiert: verhindert Einschalten
            # bei kurzem Temperaturtief ohne echten Heizbedarf.
            Transition(
                "heating",
                "heating_forced",
                lambda: (
                    self._at_above_threshold()
                    and self._compressor_on_since is not None
                    and self._compressor_runtime_minutes() < self._min_runtime_minutes
                ),
                label="Kein Heizbedarf (EMA oder AT 1h warm), Mindestlaufzeit laeuft noch",
            ),
            Transition(
                "heating",
                "standby",
                lambda: (
                    self._at_above_threshold()
                    and (
                        self._compressor_on_since is None
                        or self._compressor_runtime_minutes() >= self._min_runtime_minutes
                    )
                ),
                label="Kein Heizbedarf (EMA oder AT 1h warm), Mindestlaufzeit abgelaufen oder Kompressor nie gestartet",
            ),
            Transition(
                "heating",
                "buffer_drain",
                lambda: self._need_buffer_drain() and self._compressor_on_since is None,
                label="Puffer aufgeladen, Kompressor intern gestoppt - Puffer entladen",
            ),
            # heating_forced
            Transition(
                "heating_forced",
                "hot_water",
                self._need_hot_water,
                label="WW-Vorrang unterbricht Mindestlaufzeit",
            ),
            Transition(
                "heating_forced",
                "heating",
                lambda: self._forced_time_elapsed() and self._need_heating(),
                label="Mindestlaufzeit abgelaufen, Heizbedarf weiterhin vorhanden",
            ),
            Transition(
                "heating_forced",
                "standby",
                lambda: self._forced_time_elapsed() and self._at_above_threshold(),
                label="Mindestlaufzeit abgelaufen, kein Heizbedarf (EMA oder AT 1h warm)",
            ),
            # PV-Ladung EIN (Vorrang vor buffer_drain)
            Transition(
                "idle",
                "buffer_charge",
                self._should_charge_buffer,
                label="PV-Ueberschuss verfuegbar",
            ),
            Transition(
                "standby",
                "buffer_charge",
                lambda: self._standby_done() and self._should_charge_buffer(),
                label="Standby vorbei, PV-Laden",
            ),
            Transition(
                "buffer_drain",
                "buffer_charge",
                self._should_charge_buffer,
                label="PV-Ueberschuss - Laden statt Entladen",
            ),
            # PV-Boost
            Transition(
                "buffer_charge",
                "buffer_charge_boost",
                self._should_boost,
                label="PV-Boost: Heizstab zuschaltbar",
            ),
            Transition(
                "buffer_charge_boost",
                "buffer_charge",
                self._should_stop_boost,
                label="Boost-Bedingung entfallen",
            ),
            # PV-Ladung AUS
            Transition(
                "buffer_charge",
                "standby",
                lambda: (
                    self._wp_inlet_temp() >= self._buffer_charge_inlet_temp
                    and self.minutes_in_state()
                    >= self._get_param("buffer_charge_min_minutes")
                ),
                label="Einlass-Ziel erreicht - Puffer vollstaendig geladen (40002=1)",
            ),
            Transition(
                "buffer_charge",
                "standby",
                lambda: self._pv_energy_kwh() < self._pv_buffer_min_kwh * 0.5,
                label="PV-Ertrag zu niedrig",
            ),
            Transition(
                "buffer_charge_boost",
                "standby",
                lambda: (
                    self._wp_inlet_temp() >= self._boost_inlet_temp
                    or self._pv_energy_kwh() < self._pv_buffer_min_kwh * 0.5
                ),
                label="Boost-Einlass-Ziel erreicht oder PV weg",
            ),
            # -- Puffer-Entladen (buffer_drain) -----------------------------------
            # Puffer warm + AT kalt -> Pumpe + Mischer liefern Pufferwaerme, WP bleibt aus.
            # Uebergang zu heating wenn Puffer 1/2 unter Schwelle (WP noetig).
            Transition(
                "idle",
                "buffer_drain",
                self._need_buffer_drain,
                label="AT kalt, Puffer warm - Entladen ohne WP",
            ),
            Transition(
                "standby",
                "buffer_drain",
                lambda: self._standby_done() and self._need_buffer_drain(),
                label="Standby vorbei, Puffer liefert ans Haus",
            ),
            Transition(
                "buffer_drain",
                "idle",
                self._at_above_threshold,
                label="Kein Heizbedarf (EMA oder AT 1h warm) - Entladen beendet",
            ),
            # Standby -> Idle: Catch-all wenn heating/buffer_drain/buffer_charge nicht greifen.
            # Kein AT-Check hier: bei kaltem AT aber Puffer in Totzone (zwischen threshold_on
            # und threshold_off) wuerde die FSM sonst ewig in standby haengen.
            # idle -> heating / buffer_drain greifen danach wieder mit ihren eigenen Bedingungen.
            Transition(
                "standby",
                "idle",
                lambda: (
                    self._standby_done()
                    and not self._need_hot_water()
                    and not self._should_charge_buffer()
                ),
                label="Standby abgelaufen, kein anderer Bedarf",
            ),
        ]

    # -------------------------------------------------------------------------
    # Trigger
    # -------------------------------------------------------------------------

    def define_triggers(self):
        for s in [
            self._sensor_outdoor_temp,
            self._sensor_pv_power_w,
            self._sensor_pv_energy_kwh,
            self._sensor_buffer_bottom,
            self._sensor_buffer_mid,
            self._sensor_buffer_mid_high,
            self._sensor_buffer_top,
        ]:
            self.listen_state(self._on_sensor_change, s)

        if self._sensor_wp_leaving_temp:
            self.listen_state(
                self._on_wp_leaving_temp_change, self._sensor_wp_leaving_temp
            )

        if self._sensor_return_temp:
            self.listen_state(self._on_sensor_change, self._sensor_return_temp)

        if self._binary_aux_heater_1:
            self.listen_state(self._on_aux_heater_change, self._binary_aux_heater_1)
        if self._binary_aux_heater_2:
            self.listen_state(self._on_aux_heater_change, self._binary_aux_heater_2)

        # Inlet-Sensor abhoeren (fuer buffer_charge/boost Exit-Bedingung)
        self.listen_state(self._on_sensor_change, self._sensor_wp_inlet_temp)

        for key in [
            "heat_curve_at_high",
            "buffer_drain_margin",
            "buffer_drain_hyst",
            "ww_target_temp",
            "pv_buffer_min_kwh",
            "pv_buffer_max_kwh",
            "min_runtime_minutes",
            "standby_minutes",
            "buffer_charge_inlet_temp",
            "buffer_charge_min_minutes",
            "boost_inlet_temp",
            "cycling_window_minutes",
            "cycling_max_starts",
        ]:
            entity_id = f"{self._ha_param_prefix}{key}"
            self.listen_state(self._on_param_change, entity_id, attribute="state")

        if self._dry_run_entity:
            self.listen_state(self._on_dry_run_change, self._dry_run_entity)

        self.run_every(self.evaluate_transitions, self.datetime(), 60)
        self.run_every(self._sync_modbus_setpoints, self.datetime(), 60)
        self._schedule_hot_water_windows()

        prefix = self.args.get("event_prefix", "fsm_heatpump")
        self.listen_event(self._on_force_event, f"{prefix}_force_state")
        self.listen_state(self._on_compressor_change, self._binary_compressor)

    def _schedule_hot_water_windows(self):
        h, m = self._parse_time(self._hw_morning_weekday)
        self.run_daily(self._open_hw_window_if_werktag, f"{h:02d}:{m:02d}:00")
        h, m = self._parse_time(self._hw_morning_weekend)
        self.run_daily(self._open_hw_window_if_wochenende, f"{h:02d}:{m:02d}:00")
        h, m = self._parse_time(self._hw_evening)
        self.run_daily(self._open_hw_window, f"{h:02d}:{m:02d}:00")

    # -------------------------------------------------------------------------
    # Grundfos Pumpe
    # -------------------------------------------------------------------------

    def _pump_on(self):
        """Grundfos Heizkreispumpe einschalten."""
        if not self._switch_circuit2_pump:
            return
        if self._is_dry_run():
            self.log("[DRY-RUN] _pump_on() -> kein Switch-Write", level="DEBUG")
            return
        try:
            self.call_service("switch/turn_on", entity_id=self._switch_circuit2_pump)
            self.log("[HP] Grundfos Pumpe EIN")
        except Exception as e:
            self.log(f"[HP] Pumpe EIN fehlgeschlagen: {e}", level="WARNING")

    def _pump_off(self):
        """Grundfos Heizkreispumpe ausschalten."""
        if not self._switch_circuit2_pump:
            return
        if self._is_dry_run():
            self.log("[DRY-RUN] _pump_off() -> kein Switch-Write", level="DEBUG")
            return
        try:
            self.call_service("switch/turn_off", entity_id=self._switch_circuit2_pump)
            self.log("[HP] Grundfos Pumpe AUS")
        except Exception as e:
            self.log(f"[HP] Pumpe AUS fehlgeschlagen: {e}", level="WARNING")

    def _update_charge_heating(self):
        """Grundfos + Mischer waehrend buffer_charge / buffer_charge_boost AT-abhaengig steuern.

        Kalt (AT < Heizgrenze): Pumpe EIN + Mischer PI-Regler aktiv.
          WP laedt Puffer (40002=1), Heizkreis wird parallel mit Pufferwaerme versorgt.
          Der PI-Regler stellt den Mischer so, dass Heizkurven-Sollwert erreicht wird.
        Warm (AT >= Heizgrenze): Pumpe AUS + Mischer geschlossen (Sommerbetrieb).

        Idempotent: Pump-State und _mixer_handle werden geprueft, um unnoetige
        Schaltaktionen und Timer-Neustarts bei jedem Sensor-Update zu vermeiden.
        """
        if not self._at_above_threshold():
            # Pumpe: nur einschalten wenn aktuell aus
            if self._switch_circuit2_pump:
                pump_state = self.get_state(self._switch_circuit2_pump)
                if pump_state != "on":
                    self._pump_on()
            # Mischer PI: nur starten wenn noch nicht aktiv
            if self._mixer_handle is None:
                sp = self._calc_flow_setpoint()
                self._pi_prev_error = sp - self._flow_temp()
                self._start_mixer_controller()
        else:
            # Pumpe: nur ausschalten wenn aktuell ein
            if self._switch_circuit2_pump:
                pump_state = self.get_state(self._switch_circuit2_pump)
                if pump_state != "off":
                    self._pump_off()
            # Mischer: nur stoppen und schliessen wenn aktiv
            if self._mixer_handle is not None:
                self._stop_mixer_controller()
                self._mixer_full_close()

    # -------------------------------------------------------------------------
    # State Hooks
    # -------------------------------------------------------------------------

    def on_enter_heating(self):
        self._pump_on()
        sp = self._calc_flow_setpoint()
        self._pi_prev_error = sp - self._flow_temp()
        self._pi_position = self._pi_start_position()
        self._start_mixer_controller()
        if (
            self.previous_state == "heating_forced"
            and self._forced_lp_value is not None
        ):
            # Ramp-Down: 40003 bleibt auf forced-Wert, wird -1 deg/min zur Kurve abgesenkt.
            # 40006 sofort auf Heizkurven-Sollwert (via _set_forced_setpoint).
            forced_c1 = round(
                self._forced_lp_value
                + self._forced_setpoint_offset
                + self._circuit1_offset,
                1,
            )
            self._set_forced_setpoint(forced_c1)
            self._start_c1_ramp(forced_c1)
            self._decay_start_time = None
        else:
            # WP einschalten (Schalter/WW-aus); Kreis2 = Heizkurve.
            self._set_wp_mode("heat", sp)
            # Kreis1 (40003) folgt ab jetzt dynamisch dem WP-Auslass (LP-Tracking).
            # _forced_lp_value frisch initialisieren (kein Ueberhang aus altem State).
            self._forced_lp_value = None
            self._decay_start_time = None
            self._update_heating_setpoint()
        self.log(
            f"[HP] Heizung AN | Vorlauf-Soll: {sp:.1f}C | "
            f"Puffer: 1/4={self._buffer_bottom():.1f} 1/2={self._buffer_mid():.1f} "
            f"3/4={self._buffer_mid_high():.1f} oben={self._buffer_top():.1f}C"
        )

    def on_exit_heating(self):
        self._cancel_c1_ramp()

    def on_enter_hot_water(self):
        self._pump_off()  # WW laeuft ueber Frischwasser-Waermetauscher, Heizkreis nicht noetig
        # WW-Fenster sind morgens/abends - PV laeuft mittags -> kein Dual-Temperatur noetig.
        # Puffer wurde tagsuebers per buffer_charge geladen; WW profitiert indirekt davon.
        self._set_wp_mode("hot_water", self._ww_target)
        self._stop_mixer_controller()
        if self._pi_position > 1.0:
            self._last_pi_position = self._pi_position  # Position merken, nicht fahren
        self.log(
            f"[HP] Warmwasser AN | Oben: {self._buffer_top():.1f}C / Ziel: {self._ww_target:.1f}C"
        )

    def on_exit_hot_water(self):
        pass  # Mischer bleibt auf eingefrorener Position

    def on_enter_buffer_charge(self):
        self._set_energy_state(3)
        self._set_control_method(1)  # 40002=1: WP regelt auf Wassereinlass (30003)
        # Sollwert = Einlass-Zieltemperatur: WP laeuft bis Puffer-Ruecklauf X°C erreicht.
        # Wenn der Einlass (Ruecklauf vom Puffer-Unten) = 50°C ist, ist der gesamte Puffer >= 50°C.
        self._set_wp_mode("buffer_charge", self._buffer_charge_inlet_temp)
        # Grundfos + Mischer: AT-abhaengig (kalt -> parallel heizen, warm -> AUS)
        self._update_charge_heating()
        self.log(
            f"[HP] PV-Pufferladung AN | 40002=1 (Einlass-Regelung) | "
            f"Einlass-Soll: {self._buffer_charge_inlet_temp:.1f}C | "
            f"Einlass-Ist: {self._raw_inlet_temp():.1f}C | PV: {self._pv_power_w()/1000:.1f} kW | "
            f"AT: EMA={self._outdoor_temp():.1f}C 1h={self._outdoor_temp_1h():.1f}C ({'Pumpe EIN' if not self._at_above_threshold() else 'Pumpe AUS'})"
        )

    def on_exit_buffer_charge(self):
        self._set_energy_state(2)
        self._set_control_method(0)  # 40002=0: zurueck auf Wasserauslass-Regelung
        # Puffer 1/4 (tank_1_4) statt Einlass: Einlass kühlt via Rohrverluste ab wenn Pumpe stoppt
        restart_temp = self._buffer_charge_inlet_temp - self._get_param("buffer_charge_hyst")
        bottom = self._buffer_bottom()
        if bottom >= restart_temp:
            self._buffer_fully_charged = True
        self.log(
            f"[HP] PV-Ladung AUS | Puffer 1/4={bottom:.1f}C restart_temp={restart_temp:.1f}C "
            f"→ vollgeladen={'JA' if self._buffer_fully_charged else 'NEIN'}"
        )

    def on_enter_buffer_charge_boost(self):
        self._set_energy_state(self._boost_energy_state)
        self._set_control_method(1)  # 40002=1 bleibt aktiv (Einlass-Regelung)
        # Boost hebt Einlass-Ziel von buffer_charge_inlet_temp auf boost_inlet_temp.
        # Der hoehere Sollwert zwingt WP+Heizstab laenger zu laufen.
        self._set_wp_mode("buffer_charge", self._boost_inlet_temp)
        # Grundfos + Mischer: AT-abhaengig (kalt -> parallel heizen, warm -> AUS)
        self._update_charge_heating()
        self._set_silent_mode(False)
        self.log(
            f"[HP] Heizstab-Boost AN | Energiezustand={self._boost_energy_state} | "
            f"Einlass-Soll={self._boost_inlet_temp}C | "
            f"Einlass-Ist: {self._raw_inlet_temp():.1f}C | PV: {self._pv_power_w()/1000:.1f} kW | "
            f"AT: EMA={self._outdoor_temp():.1f}C 1h={self._outdoor_temp_1h():.1f}C ({'Pumpe EIN' if not self._at_above_threshold() else 'Pumpe AUS'})"
        )

    def on_exit_buffer_charge_boost(self):
        self._set_energy_state(2)
        self._set_control_method(0)  # 40002=0: zurueck auf Wasserauslass-Regelung
        # Boost-Hysterese: selbe Schwelle wie buffer_charge (1/4 Tank, kein Pumpen-Guard)
        restart_temp = self._buffer_charge_inlet_temp - self._get_param("buffer_charge_hyst")
        bottom = self._buffer_bottom()
        if bottom >= restart_temp:
            self._buffer_fully_charged = True
        self.log(
            f"[HP] Heizstab-Boost AUS | Puffer 1/4={bottom:.1f}C restart_temp={restart_temp:.1f}C "
            f"→ vollgeladen={'JA' if self._buffer_fully_charged else 'NEIN'}"
        )
        self._restore_silent_mode()

    def on_enter_heating_forced(self):
        self._pump_on()
        current = self._wp_leaving_temp()
        self._forced_lp_value = current
        self._last_forced_c1 = None  # Ratchet-Reset beim Eintritt
        # Kreis 1 (40003): LP-Wert + Offset + Kreis1-Offset (haelt WP am Laufen)
        # Kreis 2 (40006): normaler Heizkurven-Sollwert (kein Einfluss auf WP-Regelung)
        forced_c1 = round(
            min(
                self._get_param("heat_curve_vl_low"),
                current + self._forced_setpoint_offset + self._circuit1_offset,
            ),
            1,
        )
        self._last_forced_c1 = forced_c1
        self._set_forced_setpoint(forced_c1)
        # Mischer PI-Regler weiterlaufen lassen (Integral aus heating erhalten).
        # _start_mixer_controller() stoppt ggf. alten Timer bevor neuer gestartet wird -
        # noetig falls heating_forced per force_state direkt betreten wird.
        self._start_mixer_controller()
        if getattr(self, "_is_restart_enter", False):
            # Neustart-Restore: WP lief bereits, Restlaufzeit unbekannt.
            # Timer sofort ablaufen lassen - FSM kann direkt in heating/standby wechseln.
            self._forced_end_time = datetime.datetime.now()
            self.log(
                f"[HP] Mindestlaufzeit (Neustart-Restore) | WP-Ausgang: {current:.1f}C "
                f"-> Kreis1={forced_c1:.1f}C | Timer sofort abgelaufen"
            )
        else:
            # Absoluten End-Timestamp berechnen: verbleibende Mindestlaufzeit ab jetzt.
            already_run = self._compressor_runtime_minutes()
            remaining = max(0.0, self._min_runtime_minutes - already_run)
            self._forced_end_time = datetime.datetime.now() + datetime.timedelta(
                minutes=remaining
            )
            self.log(
                f"[HP] Mindestlaufzeit aktiv | WP-Ausgang: {current:.1f}C "
                f"-> Kreis1={forced_c1:.1f}C | Puffer 1/2: {self._buffer_mid():.1f}C | "
                f"Kompressor lief schon {already_run:.0f} min, "
                f"Ende um {self._forced_end_time.strftime('%H:%M:%S')}"
            )

    def on_exit_heating_forced(self):
        self._stop_mixer_controller()

    def _enter_passive_state(self, close_mixer: bool = True) -> float:
        """Gemeinsame Aktionen fuer idle/standby: WP+Pumpe aus,
        Heizkurven-Sollwerte auf beide Kreise schreiben. Gibt den Sollwert zurueck.
        close_mixer=False einfrieren (z.B. Standby nach heating_forced).
        """
        self._pump_off()
        self._set_energy_state(2)
        self._set_wp_mode("off", 0)
        self._stop_mixer_controller()
        if close_mixer:
            self._mixer_full_close()
        sp = self._calc_flow_setpoint_smoothed()
        self._write_c2_direct(sp)
        self._write_c1_direct(round(sp + self._circuit1_offset, 1))
        return sp

    def on_enter_standby(self):
        # Mischer immer einfrieren: PI startet beim naechsten heating/buffer_drain
        # von der letzten bekannten Position statt von 0% (vermeidet unnoetige Fahrt).
        if self._pi_position > 1.0:
            self._last_pi_position = self._pi_position
        self._enter_passive_state(close_mixer=False)
        self.log(f"[HP] Standby | Verdichterschutz {self._standby_minutes:.0f} min")

    def on_enter_buffer_drain(self):
        """Pufferwaerme ans Haus liefern, WP bleibt aus.
        PI-Regler positioniert Mischer so, dass Heizkurven-Sollwert erreicht wird.
        Wenn Puffer 1/2 unter threshold_on faellt -> Uebergang zu heating (WP startet).
        """
        self._pump_on()
        self._set_energy_state(2)
        self._set_wp_mode("off", 0)  # WP bleibt AUS
        self._pi_position = self._pi_start_position()
        sp = self._calc_flow_setpoint()
        self._pi_prev_error = sp - self._flow_temp()
        self._start_mixer_controller()
        sp_s = self._calc_flow_setpoint_smoothed()
        self._write_c2_direct(sp_s)
        self._write_c1_direct(round(sp_s + self._circuit1_offset, 1))
        self.log(
            f"[HP] Puffer-Entladen | Kreis-2-Pumpe EIN, WP AUS | "
            f"Vorlauf-Soll: {sp:.1f}C | "
            f"Puffer 1/2: {self._buffer_mid():.1f}C 1/4: {self._buffer_bottom():.1f}C "
            f"(Drain-Grenze 1/2: EIN>={self._buffer_drain_threshold_off():.1f}C / AUS<{self._buffer_drain_threshold_on():.1f}C)"
        )

    def on_exit_buffer_drain(self):
        self._stop_mixer_controller()

    def on_enter_idle(self):
        # Idle = echter Ruhezustand: Mischer schliesst. Position wird via _mixer_full_close
        # in _last_pi_position gesichert, damit naechster heating/buffer_drain korrekt startet.
        self._enter_passive_state(close_mixer=True)
        self.log("[HP] Idle")

    # -------------------------------------------------------------------------
    # PI-Regler Mischer
    # -------------------------------------------------------------------------

    def _start_mixer_controller(self):
        self._stop_mixer_controller()
        self._mixer_handle = self.run_every(
            self._mixer_pi_step, self.datetime(), self._mixer_interval_s
        )

    def _stop_mixer_controller(self):
        if self._mixer_handle:
            try:
                self.cancel_timer(self._mixer_handle)
            except Exception:
                pass
            self._mixer_handle = None

    def _mixer_pi_step(self, kwargs):
        if self.state not in (
            "heating",
            "buffer_drain",
            "heating_forced",
            "buffer_charge",
            "buffer_charge_boost",
        ):
            return
        if self._binary_defrost and self.get_state(self._binary_defrost) == "on":
            self.log("[Mischer] Abtauen aktiv - PI-Regler pausiert", level="DEBUG")
            return
        raw_flow = self.get_state(self._sensor_flow_temp)
        try:
            float(raw_flow)
        except (TypeError, ValueError):
            if self._flow_temp_cache is None:
                self.log(
                    "[Mischer] Vorlauf-Sensor unavailable, kein Cache - PI pausiert",
                    level="DEBUG",
                )
                return
        setpoint = self._calc_flow_setpoint_smoothed()
        actual = self._flow_temp()
        e = setpoint - actual
        # velocity-form PI: delta = kp*(e - e_prev) + ki*e
        # Step-Limit: verhindert Ueberschwingen durch Stellantrieb-Verzoegerung (123s Vollhub).
        de = e - self._pi_prev_error
        delta = self._mixer_kp * de + self._mixer_ki * e
        max_step = self._get_param("mixer_max_step_pct")
        delta = max(-max_step, min(max_step, delta))
        self._pi_prev_error = e
        old_pos = self._pi_position
        new_pos = max(0.0, min(100.0, old_pos + delta))
        # Sub-1%-Akkumulation: _pi_position immer aktualisieren,
        # Cover-Befehl nur bei Aenderung des ganzzahligen Wertes.
        self._pi_position = new_pos
        self.log(
            f"[Mischer] Soll={setpoint:.1f}C Ist={actual:.1f}C "
            f"e={e:+.1f}C de={de:+.2f} delta={delta:+.2f} "
            f"Pos: {old_pos:.1f}%->{new_pos:.1f}%",
            level="DEBUG",
        )
        if round(new_pos) != round(old_pos):
            self._set_cover_position(new_pos)
            self._write_pi_position(new_pos)

    def _set_cover_position(self, position: float):
        if not self._cover_mixer:
            return
        if self._is_dry_run():
            self.log(
                f"[DRY-RUN] _set_cover_position({position:.0f}%) -> kein cover-Write",
                level="DEBUG",
            )
            return
        try:
            self.call_service(
                "cover/set_cover_position",
                entity_id=self._cover_mixer,
                position=round(position),
            )
        except Exception as e:
            self.log(f"[Mischer] cover setzen fehlgeschlagen: {e}", level="WARNING")

    def _cover_position_actual(self) -> float:
        """Aktuelle physische Mischer-Position aus HA lesen (Fallback: 50%)."""
        if not self._cover_mixer:
            return 50.0
        try:
            pos = self.get_state(self._cover_mixer, attribute="current_position")
            if pos is not None:
                return float(pos)
        except Exception:
            pass
        return 50.0

    def _write_pi_position(self, position: float):
        """PI-Soll-Position in HA schreiben (Dashboard-Visualisierung)."""
        if not self._input_pi_position:
            return
        try:
            self.call_service(
                "input_number/set_value",
                entity_id=self._input_pi_position,
                value=round(position, 1),
            )
        except Exception as e:
            self.log(
                f"[Mischer] PI-Position schreiben fehlgeschlagen: {e}", level="WARNING"
            )

    def _pi_start_position(self) -> float:
        """PI-Startposition: actual wenn Mischer offen und kein aktiver Close-Vorgang,
        sonst letzte bekannte Gleichgewichtsposition, sonst Warmstart-Default.
        _mixer_was_closed verhindert, dass eine noch laufende close-Bewegung (time_based Cover
        braucht bis zu 125 s) als valide Startposition gewertet wird."""
        actual = self._cover_position_actual()
        use_last = actual < 1.0 or self._mixer_was_closed
        self._mixer_was_closed = False  # einmalig konsumieren
        if use_last:
            start = (
                self._last_pi_position
                if self._last_pi_position is not None
                else self._get_param("mixer_warmstart_position")
            )
            self._set_cover_position(start)
            self._write_pi_position(start)
            self.log(
                f"[Mischer] PI-Start: actual={actual:.0f}% last_pi="
                f"{'%.0f%%' % self._last_pi_position if self._last_pi_position is not None else 'None'}"
                f" was_closed={use_last} → start={start:.0f}%",
                level="DEBUG",
            )
            return start
        self.log(
            f"[Mischer] PI-Start: actual={actual:.0f}% (Cover offen, kein Close-Flag) → direkt uebernommen",
            level="DEBUG",
        )
        return actual

    def _mixer_full_open(self):
        """Mischer vollstaendig oeffnen mit 2s Ueberfahren zum Positions-Reset."""
        self._pi_position = 100.0
        self._write_pi_position(100.0)
        self._set_cover_position(100.0)
        if self._cover_mixer and not self._is_dry_run():
            self.run_in(
                lambda _: self.call_service(
                    "cover/open_cover", entity_id=self._cover_mixer
                ),
                self._mixer_full_travel_s + 2,
            )
        else:
            self.log(
                "[DRY-RUN] _mixer_full_open() Ueberfahren -> kein cover-Write",
                level="DEBUG",
            )

    def _mixer_full_close(self):
        """Mischer vollstaendig schliessen mit 2s Ueberfahren zum Positions-Reset."""
        if self._pi_position > 1.0:
            self._last_pi_position = self._pi_position
        self._mixer_was_closed = True  # Signal fuer _pi_start_position: Cover evtl. noch in Bewegung
        self._pi_position = 0.0
        self._pi_prev_error = 0.0
        self._write_pi_position(0.0)
        self._set_cover_position(0.0)
        if self._cover_mixer and not self._is_dry_run():
            self.run_in(
                lambda _: self.call_service(
                    "cover/close_cover", entity_id=self._cover_mixer
                ),
                self._mixer_full_travel_s + 2,
            )
        else:
            self.log(
                "[DRY-RUN] _mixer_full_close() Ueberfahren -> kein cover-Write",
                level="DEBUG",
            )

    # -------------------------------------------------------------------------
    # Heartbeat-Log
    # -------------------------------------------------------------------------

    def _log_state_heartbeat(self, kwargs):
        """Alle 10 min: aktueller Zustand + warum kein Zustandswechsel (Debugging)."""
        state = self.state
        at_ema = self._outdoor_temp()
        at_1h = self._outdoor_temp_1h()
        thr = self._heating_threshold
        mid = self._buffer_mid()
        thr_on = self._buffer_drain_threshold_on()
        thr_off = self._buffer_drain_threshold_off()
        mins = self.minutes_in_state()

        def yn(cond: bool) -> str:
            return "JA" if cond else "nein"

        msg = f"[HP] HB {state} ({mins:.0f}min)"

        if state in ("buffer_drain", "idle", "standby", "heating", "heating_forced"):
            heat = self._need_heating()
            drain = self._need_buffer_drain()
            # Entry: EMA; Exit: 1h-Mittelwert
            msg += (
                f" | AT_EMA={at_ema:.1f}C AT_1h={at_1h:.1f}C(Grenze={thr:.1f})"
                f" | Puf1/2={mid:.1f}C(DrainAN>={thr_off:.1f} AUS<{thr_on:.1f})"
            )
            above = self._at_above_threshold()
            if state == "buffer_drain":
                msg += (
                    f" | ->heat:{yn(heat)}"
                    f"(EMA<{thr:.1f}:{yn(at_ema<thr)} 1h<{thr:.1f}:{yn(at_1h<thr)} Puf<{thr_on:.1f}:{yn(mid<thr_on)})"
                    f" | ->idle:{yn(above)}"
                    f"(EMA>={thr:.1f}:{yn(at_ema>=thr)} oder 1h>={thr:.1f}:{yn(at_1h>=thr)})"
                )
            elif state in ("idle", "standby"):
                charge = self._should_charge_buffer()
                msg += (
                    f" | ->heat:{yn(heat)} ->drain:{yn(drain)} ->charge:{yn(charge)}"
                )
            elif state in ("heating", "heating_forced"):
                comp = self._compressor_on_since is not None
                rt = self._compressor_runtime_minutes()
                min_rt = self._min_runtime_minutes
                forced_ok = state == "heating_forced" and self._forced_time_elapsed()
                msg += (
                    f" | Komp={'EIN' if comp else 'AUS'} {rt:.0f}/{min_rt:.0f}min"
                    f" | ->standby:{yn(above and (not comp or rt >= min_rt))}"
                    f"(EMA>={thr:.1f}:{yn(at_ema>=thr)} oder 1h>={thr:.1f}:{yn(at_1h>=thr)})"
                )
                if state == "heating_forced":
                    msg += f" | Mindestlaufzeit-Ende: {yn(forced_ok)}"

        elif state in ("buffer_charge", "buffer_charge_boost"):
            inlet = self._raw_inlet_temp()
            bottom = self._buffer_bottom()
            pv_rest = self._pv_energy_kwh()
            charge_ok = self._should_charge_buffer()
            restart_temp = self._buffer_charge_inlet_temp - self._get_param("buffer_charge_hyst")
            target = self._boost_inlet_temp if state == "buffer_charge_boost" else self._buffer_charge_inlet_temp
            msg += (
                f" | Einlass={inlet:.1f}C(Ziel={target:.1f})"
                f" | Puf1/4={bottom:.1f}C(restart>={restart_temp:.1f})"
                f" | PV-Rest={pv_rest:.1f}kWh"
                f" | vollgeladen={yn(self._buffer_fully_charged)}"
                f" | Ladebedingung:{yn(charge_ok)}"
            )

        elif state == "hot_water":
            msg += f" | WW-Fenster={'aktiv' if self._hw_window_active else 'inaktiv'}"

        self.log(msg)

    # -------------------------------------------------------------------------
    # Bedingungsfunktionen
    # -------------------------------------------------------------------------

    def _at_above_threshold(self) -> bool:
        """Exit-Bedingung: kein Heizbedarf wenn EMA *oder* AT_1h ueber Heizgrenze.
        Pendant zu _need_heating(): dort EMA UND 1h unter Grenze (Entry); hier EMA ODER 1h drueber (Exit)."""
        return (
            self._outdoor_temp() >= self._heating_threshold
            or self._outdoor_temp_1h() >= self._heating_threshold
        )

    def _need_heating(self) -> bool:
        # buffer_mid (1/2) als Schaltpunkt: zeigt ob genuegend Waerme fuer Drain/Haus vorhanden.
        # In heating: off-Schwelle (on + hyst) verhindert Pendeln.
        # Aus idle/standby/drain: on-Schwelle triggert WP-Start.
        # Entry-Bedingung prueft BEIDE AT-Sensoren: EMA (traege, Entry-Schutz) UND 1h (schnell).
        # Verhindert standby->heating->standby-Bounce wenn AT_1h bereits warm aber EMA noch kalt.
        threshold = (
            self._buffer_drain_threshold_off()
            if self.state == "heating"
            else self._buffer_drain_threshold_on()
        )
        return (
            self._outdoor_temp() < self._heating_threshold
            and self._outdoor_temp_1h() < self._heating_threshold
            and self._buffer_mid() < threshold
        )

    def _need_hot_water(self) -> bool:
        return self._hw_window_active

    def _need_buffer_drain(self) -> bool:
        """Puffer hat noch genug Waerme, AT kalt - Pumpe liefert Pufferwaerme ans Haus.
        WP bleibt aus. Uebergang zu heating wenn Puffer 1/2 < Sollwert + buffer_drain_margin.

        Verwendet buffer_mid (1/2) statt buffer_bottom (1/4): Nach PV-Ladung oder
        Heizzyklen ist der Pufferboden oft kalt (Ruecklauf), waehrend die oberen Zonen
        noch ausreichend Waerme fuer den Heizkreis haben.
        """
        return (
            self._outdoor_temp() < self._heating_threshold
            and self._outdoor_temp_1h() < self._heating_threshold
            and self._buffer_mid() >= self._buffer_drain_threshold_off()
        )

    def _should_charge_buffer(self) -> bool:
        """Puffer laden wenn PV verfuegbar und Einlass noch unter Zieltemperatur.
        Mit 40002=1 (Wassereinlass-Regelung) zeigt der Einlass-Sensor (30003) ob der
        gesamte Puffer bereits geladen ist - kein separater Tank-Sensor noetig.

        Hysterese nach erfolgreicher Ladung: _buffer_fully_charged verhindert Sofort-Neustart
        wenn der Einlass nach WP-Stopp knapp unter die Zieltemperatur faellt.
        Neustart erst wenn Einlass < (buffer_charge_inlet_temp - buffer_charge_hyst).

        Benutzt _raw_inlet_temp() (ohne WP-Pumpen-Guard): WP laeuft hier noch nicht,
        der Sensor zeigt zuverlaessig die Puffertemperatur. _wp_inlet_temp() wuerde 0 liefern
        und die Hysterese sofort loeschen.
        """
        if self._buffer_fully_charged:
            restart_temp = self._buffer_charge_inlet_temp - self._get_param(
                "buffer_charge_hyst"
            )
            bottom = self._buffer_bottom()
            if bottom >= restart_temp:
                return False  # Puffer noch warm genug, kein Neustart
            # Puffer 1/4 weit genug abgekuehlt - Hysterese zuruecksetzen
            # (1/4-Fühler direkt im Tank, kühlt nicht durch Rohrverluste wie 30003)
            self._buffer_fully_charged = False
            self.log(
                f"[HP] Puffer-Hysterese: Puffer 1/4 {bottom:.1f}C < {restart_temp:.1f}C "
                f"-> Ladung wieder moeglich",
                level="INFO",
            )

        inlet = self._raw_inlet_temp()
        pv_rest = self._pv_energy_kwh()
        return (
            self._pv_in_solar_window()
            and pv_rest >= self._pv_buffer_min_kwh  # genug Restmenge vorhanden
            and pv_rest < self._pv_buffer_max_kwh  # Nachmittags-Fenster: nicht zu frueh
            and inlet
            < self._buffer_charge_inlet_temp  # Puffer noch nicht voll (Einlass)
            and self._buffer_bottom()
            < self._buffer_charge_inlet_temp  # Tank 1/4 als Vorcheck (pumpenunabhaengig)
        )

    def _should_boost(self) -> bool:
        """Boost aktivieren wenn PV stark genug und Einlass-Temp noch unter Boost-Ziel.
        boost_inlet_min_c: Mindest-Einlass bevor Boost startet (Puffer muss schon warm sein).
        Mit 40002=1: Boost hebt Einlass-Ziel von buffer_charge_inlet_temp auf boost_inlet_temp.
        """
        if self._binary_defrost:
            if self.get_state(self._binary_defrost) == "on":
                return False
        if not self._in_boost_window():
            return False
        if self._pv_power_w() < self._boost_pv_min_w:
            return False
        if (
            self._wp_inlet_temp() < self._boost_inlet_min_c
        ):  # Puffer noch zu kalt fuer Boost
            return False
        if (
            self._wp_inlet_temp() >= self._boost_inlet_temp
        ):  # Boost-Ziel bereits erreicht
            return False
        return True

    def _should_stop_boost(self) -> bool:
        if self._binary_defrost:
            if self.get_state(self._binary_defrost) == "on":
                return True
        if self._pv_power_w() < self._boost_pv_off_w:
            return True
        if not self._in_boost_window():
            return True
        return False

    def _in_boost_window(self) -> bool:
        now = datetime.datetime.now().time()
        try:
            start_h, start_m = self._parse_time(self._boost_window_start)
            end_h, end_m = self._parse_time(self._boost_window_end)
            start = datetime.time(start_h, start_m)
            end = datetime.time(end_h, end_m)
            return start <= now < end
        except Exception:
            return self._pv_in_solar_window()

    def _compressor_runtime_minutes(self) -> float:
        if self._compressor_on_since is None:
            return 0.0
        return (
            datetime.datetime.now() - self._compressor_on_since
        ).total_seconds() / 60

    def _standby_done(self) -> bool:
        return self.minutes_in_state() >= self._standby_minutes

    def _forced_time_elapsed(self) -> bool:
        """True wenn die Verdichter-Mindestlaufzeit in heating_forced abgelaufen ist.
        Bevorzugt den absoluten End-Zeitstempel (_forced_end_time), faellt auf die
        Verweildauer im Zustand zurueck falls dieser nicht gesetzt wurde.
        """
        if self._forced_end_time is not None:
            return datetime.datetime.now() >= self._forced_end_time
        return self.minutes_in_state() >= self._min_runtime_minutes

    def _pv_in_solar_window(self) -> bool:
        """PV-Fenster fuer Pufferladung und WW-Temperaturwahl.
        Nutzt dasselbe Zeitfenster wie boost_window (apps.yaml: boost_window_start/end).
        """
        return self._in_boost_window()

    # -------------------------------------------------------------------------
    # Heizkurve
    # -------------------------------------------------------------------------

    def _calc_flow_setpoint(self, with_pv_correction: bool = True) -> float:
        """Momentaner (ungedaempfter) Heizkurven-Sollwert.
        Wird fuer Schwellen-Berechnungen (buffer_drain) genutzt - kein EWMA, instant response."""
        at_high = self._get_param("heat_curve_at_high")
        vl_high = self._get_param("heat_curve_vl_high")
        at_low = self._get_param("heat_curve_at_low")
        vl_low = self._get_param("heat_curve_vl_low")
        at = self._outdoor_temp()
        slope = (vl_low - vl_high) / (at_high - at_low)
        base = vl_high + (at_high - at) * slope
        if with_pv_correction:
            pv_kw = self._pv_power_w() / 1000.0
            base -= pv_kw * self._pv_correction_per_kw
        return max(18.0, min(60.0, base))

    def _calc_flow_setpoint_smoothed(self) -> float:
        """EWMA-gedaempfter Sollwert fuer PI-Regler und 40006-Writes.
        Verhindert sprunghafte Stellbefehle bei AT/PV-Schwankungen.
        Schwellen-Berechnungen nutzen _calc_flow_setpoint() (ohne Daempfung)."""
        raw = self._calc_flow_setpoint()
        alpha = self._get_param("setpoint_ewma_alpha")
        if self._setpoint_ewma is None:
            self._setpoint_ewma = raw
        else:
            self._setpoint_ewma = alpha * raw + (1.0 - alpha) * self._setpoint_ewma
        return self._setpoint_ewma

    # -------------------------------------------------------------------------
    # WP-Steuerung
    # -------------------------------------------------------------------------

    def _set_wp_mode(self, mode: str, temp: float = 0.0):
        if self._is_dry_run():
            self.log(
                f"[DRY-RUN] _set_wp_mode(mode={mode!r}, temp={temp}C) -> kein Modbus-Write",
                level="INFO",
            )
            return
        try:
            if mode == "off":
                self.call_service("switch/turn_off", entity_id=self._switch_heating)
                self.call_service("switch/turn_off", entity_id=self._switch_hot_water)
                self._log_wp_mode("[HP] Modbus -> Heizung OFF, WW OFF")

            elif mode == "heat":
                c1 = round(temp + self._circuit1_offset, 1)
                self.call_service("switch/turn_off", entity_id=self._switch_hot_water)
                self.call_service(
                    "input_number/set_value",
                    entity_id=self._input_setpoint_circuit2,
                    value=round(temp, 1),
                )
                self.call_service(
                    "input_number/set_value",
                    entity_id=self._input_setpoint_circuit1,
                    value=c1,
                )
                self.call_service("switch/turn_on", entity_id=self._switch_heating)
                self._log_wp_mode(
                    f"[HP] Modbus -> Heizung ON, Kreis2={round(temp,1)}C Kreis1={c1}C"
                )

            elif mode == "buffer_charge":
                self.call_service("switch/turn_off", entity_id=self._switch_hot_water)
                self.call_service(
                    "input_number/set_value",
                    entity_id=self._input_setpoint_circuit1,
                    value=round(temp, 1),
                )
                self.call_service("switch/turn_on", entity_id=self._switch_heating)
                self._log_wp_mode(f"[HP] Modbus -> PV-Ladung ON, Kreis1={round(temp,1)}C")

            elif mode == "hot_water":
                self.call_service("switch/turn_off", entity_id=self._switch_heating)
                self.call_service(
                    "input_number/set_value",
                    entity_id=self._input_setpoint_ww,
                    value=round(temp, 1),
                )
                self.call_service("switch/turn_on", entity_id=self._switch_hot_water)
                self._log_wp_mode(f"[HP] Modbus -> WW ON, WW-Soll={round(temp,1)}C")

        except Exception as e:
            self.log(f"[HP] Modbus-Steuerung fehlgeschlagen: {e}", level="WARNING")

    def _log_wp_mode(self, msg: str):
        if msg != self._last_wp_mode_log:
            self.log(msg)
            self._last_wp_mode_log = msg

    def _write_c1_direct(self, c1: float):
        """Schreibt nur Kreis 1 (40003), laesst Kreis 2 (40006) unveraendert."""
        if self._is_dry_run():
            self.log(f"[DRY-RUN] _write_c1_direct({c1:.1f}C)", level="INFO")
            return
        try:
            self.call_service(
                "input_number/set_value",
                entity_id=self._input_setpoint_circuit1,
                value=round(c1, 1),
            )
        except Exception as e:
            self.log(f"[HP] Kreis1-Write fehlgeschlagen: {e}", level="WARNING")

    def _write_c2_direct(self, c2: float):
        """Schreibt nur Kreis 2 (40006), laesst Kreis 1 (40003) unveraendert."""
        if self._is_dry_run():
            self.log(f"[DRY-RUN] _write_c2_direct({c2:.1f}C)", level="INFO")
            return
        try:
            self.call_service(
                "input_number/set_value",
                entity_id=self._input_setpoint_circuit2,
                value=round(c2, 1),
            )
        except Exception as e:
            self.log(f"[HP] Kreis2-Write fehlgeschlagen: {e}", level="WARNING")

    def _start_c1_ramp(self, from_c1: float):
        """Startet Ramp-Down fuer Kreis 1 (40003): -1 deg/min bis Heizkurven-Sollwert."""
        self._cancel_c1_ramp()
        target_c1 = round(self._calc_flow_setpoint_smoothed() + self._circuit1_offset, 1)
        if from_c1 <= target_c1:
            return  # bereits am/unter Ziel
        self._ramp_c1_current = from_c1
        self._ramp_c1_handle = self.run_every(self._ramp_c1_step, self.datetime(), 60)
        self.log(
            f"[HP] Kreis1 Ramp-Down: {from_c1:.1f}C -> {target_c1:.1f}C (-1 deg/min)"
        )

    def _ramp_c1_step(self, kwargs):
        """Timer-Callback: 40003 um 1 deg/min absenken bis Heizkurven-Sollwert erreicht."""
        del kwargs  # AppDaemon-Pflichtparameter, inhaltlich nicht benoetigt
        if self.state != "heating" or self._ramp_c1_current is None:
            self._cancel_c1_ramp()
            return
        target_c1 = round(self._calc_flow_setpoint_smoothed() + self._circuit1_offset, 1)
        self._ramp_c1_current -= 1.0
        if self._ramp_c1_current <= target_c1:
            self._cancel_c1_ramp()
            # Ramp-Down beendet: ab jetzt dynamisches LP-Tracking (wie normales heating).
            # _forced_lp_value und Decay-State frisch initialisieren.
            self._forced_lp_value = None
            self._decay_start_time = None
            self._update_heating_setpoint()
            self.log(
                f"[HP] Kreis1 Ramp-Down abgeschlossen -> dynamisches Tracking aktiv"
            )
        else:
            self._write_c1_direct(self._ramp_c1_current)
            self.log(
                f"[HP] Kreis1 Ramp-Down: {self._ramp_c1_current:.1f}C (Ziel: {target_c1:.1f}C)",
                level="DEBUG",
            )

    def _cancel_c1_ramp(self):
        """Kreis-1-Ramp-Down abbrechen und Zustand zuruecksetzen."""
        if self._ramp_c1_handle:
            try:
                self.cancel_timer(self._ramp_c1_handle)
            except Exception:
                pass
            self._ramp_c1_handle = None
        self._ramp_c1_current = None

    def _update_heating_setpoint(self, raw: float | None = None):
        """LP-Tracking fuer Kreis 1 (40003) im heating-Zustand.

        Phase 1 (<min_runtime): LP folgt WP-Auslass bidirektional; Untergrenze
        Kreis1 = Heizkurven-Sollwert (Kreis2), damit Kreis1 nie darunter faellt.
        Phase 2 (>=min_runtime): Kreis1 faellt linear 1 C/5 min zurueck auf den
        Heizkurven-Sollwert -> WP stoppt intern (Auslass > Kreis1 + 4 C).
        Obergrenze in beiden Phasen: heat_curve_vl_low.
        Kreis 2 (40006) bleibt der Heizkurven-Sollwert.
        """
        if raw is None:
            raw = self._wp_leaving_temp()
        curve_sp = self._calc_flow_setpoint_smoothed()
        min_elapsed = (
            self._compressor_runtime_minutes() >= self._min_runtime_minutes
        )
        # LP-Untergrenze so dass Kreis1 = curve_sp + circuit1_offset wenn LP = lp_floor:
        # raw_c1 = lp_floor + forced_setpoint_offset + circuit1_offset = curve_sp + circuit1_offset
        lp_floor = curve_sp - self._forced_setpoint_offset

        if self._forced_lp_value is None:
            # Init: LP mindestens so hoch dass Kreis1 >= Kurve
            self._forced_lp_value = max(raw, lp_floor)
            self._decay_start_time = None
        elif min_elapsed:
            # Phase 2: linearer Rueckgang 1 C / 5 min bis Heizkurven-Sollwert
            if self._decay_start_time is None:
                self._decay_start_time = datetime.datetime.now()
                self._decay_lp_start = self._forced_lp_value
            elapsed_min = (
                datetime.datetime.now() - self._decay_start_time
            ).total_seconds() / 60.0
            lp_decayed = self._decay_lp_start - elapsed_min / 5.0
            self._forced_lp_value = max(lp_floor, lp_decayed)
        else:
            # Phase 1: bidirektionaler LP-Filter; Untergrenze = Heizkurve
            self._decay_start_time = None
            lp = (
                self._forced_lp_alpha * raw
                + (1 - self._forced_lp_alpha) * self._forced_lp_value
            )
            self._forced_lp_value = max(lp_floor, lp)

        raw_c1 = (
            self._forced_lp_value + self._forced_setpoint_offset + self._circuit1_offset
        )
        # Hard-Limit: Mitkopplungs-Hochlauf verhindern (max. Auslegungstemperatur)
        c1 = round(min(self._get_param("heat_curve_vl_low"), raw_c1), 1)
        self._write_c2_direct(curve_sp)
        self._write_c1_direct(c1)
        if min_elapsed and self._decay_start_time is not None:
            elapsed = (
                datetime.datetime.now() - self._decay_start_time
            ).total_seconds() / 60.0
            mode = f"decay({elapsed:.1f}min)"
        else:
            mode = "track"
        self.log(
            f"[HP] heating Sollwert({mode}): LP={self._forced_lp_value:.2f}C "
            f"-> Kreis1={c1:.1f}C (raw={raw:.1f}C, Kurve={curve_sp:.1f}C, "
            f"cap={self._get_param('heat_curve_vl_low'):.0f}C)",
            level="DEBUG",
        )

    def _set_forced_setpoint(self, forced_c1: float):
        """Schreibt heating_forced Sollwerte:
        Kreis 1 (40003): forced_c1 (LP-Wert + Offset, haelt WP am Laufen)
        Kreis 2 (40006): aktueller Heizkurven-Sollwert (wie heating, kein Einfluss auf WP-Regelung)
        """
        curve_sp = self._calc_flow_setpoint_smoothed()
        if self._is_dry_run():
            self.log(
                f"[DRY-RUN] _set_forced_setpoint: Kreis1={forced_c1:.1f}C Kreis2={curve_sp:.1f}C",
                level="INFO",
            )
            return
        try:
            self.call_service("switch/turn_off", entity_id=self._switch_hot_water)
            self.call_service(
                "input_number/set_value",
                entity_id=self._input_setpoint_circuit2,
                value=round(curve_sp, 1),
            )
            self.call_service(
                "input_number/set_value",
                entity_id=self._input_setpoint_circuit1,
                value=round(forced_c1, 1),
            )
            self.call_service("switch/turn_on", entity_id=self._switch_heating)
            self.log(
                f"[HP] Modbus -> Heizung ON (forced) | "
                f"Kreis1={forced_c1:.1f}C Kreis2={curve_sp:.1f}C (Kurve)"
            )
        except Exception as e:
            self.log(f"[HP] Modbus-Steuerung fehlgeschlagen: {e}", level="WARNING")

    def _set_energy_state(self, state: int):
        if not self._input_energy_state:
            return
        if self._is_dry_run():
            self.log(
                f"[DRY-RUN] _set_energy_state({state}) -> kein Modbus-Write",
                level="INFO",
            )
            return
        try:
            self.call_service(
                "input_number/set_value",
                entity_id=self._input_energy_state,
                value=state,
            )
            self.log(f"[HP] Energiezustand -> {state}", level="DEBUG")
        except Exception as e:
            self.log(f"[HP] Energiezustand setzen fehlgeschlagen: {e}", level="WARNING")

    def _set_control_method(self, method: int):
        """Steuermethode Register 40002 direkt per Modbus schreiben.
        0 = Wasserauslass-Regelung (Default, normaler Heizbetrieb)
        1 = Wassereinlass-Regelung (buffer_charge/boost: WP regelt auf Ruecklauf-Temp)
        Direkter Write ohne input_number Bridge, da kein Dashboard-Helper noetig.
        """
        label = "Wassereinlass (1)" if method == 1 else "Wasserauslass (0)"
        if self._is_dry_run():
            self.log(
                f"[DRY-RUN] _set_control_method({method} / {label}) -> kein Modbus-Write",
                level="INFO",
            )
            return
        try:
            self.call_service(
                "modbus/write_register",
                hub=self._modbus_hub,
                slave=self._modbus_slave,
                address=1,  # 40002 = Holding-Register Index 1 (0-basiert von 40001)
                value=method,
            )
            self.log(f"[HP] Modbus 40002 -> {label}")
        except Exception as e:
            self.log(f"[HP] Steuermethode setzen fehlgeschlagen: {e}", level="WARNING")

    def _set_silent_mode(self, enable: bool):
        if not self._switch_silent_mode:
            return
        if self._is_dry_run():
            self.log(
                f"[DRY-RUN] Silent Mode -> {'EIN' if enable else 'AUS'}", level="INFO"
            )
            return
        try:
            self._silent_mode_before_boost = (
                self.get_state(self._switch_silent_mode) == "on"
            )
            service = "switch/turn_on" if enable else "switch/turn_off"
            self.call_service(service, entity_id=self._switch_silent_mode)
            self.log(f"[HP] Silent Mode -> {'EIN' if enable else 'AUS'}")
        except Exception as e:
            self.log(f"[HP] Silent Mode setzen fehlgeschlagen: {e}", level="WARNING")

    def _restore_silent_mode(self):
        if not self._switch_silent_mode or self._silent_mode_before_boost is None:
            return
        if self._is_dry_run():
            self.log("[DRY-RUN] Silent Mode -> Wiederherstellung", level="INFO")
            return
        try:
            service = (
                "switch/turn_on"
                if self._silent_mode_before_boost
                else "switch/turn_off"
            )
            self.call_service(service, entity_id=self._switch_silent_mode)
            self.log(
                f"[HP] Silent Mode -> wiederhergestellt ({'EIN' if self._silent_mode_before_boost else 'AUS'})"
            )
            self._silent_mode_before_boost = None
        except Exception as e:
            self.log(
                f"[HP] Silent Mode wiederherstellen fehlgeschlagen: {e}",
                level="WARNING",
            )

    def _sync_modbus_setpoints(self, kwargs=None):
        """Zyklischer Modbus-Write fuer 40003/40006.
        Stellt sicher, dass die Register nach FSM-Neustart korrekt stehen, auch wenn
        input_number sich nicht geaendert hat (HA-Automation triggert nur bei State-Change).
        """
        if self._is_dry_run():
            return
        try:
            for entity_id, address in (
                (self._input_setpoint_circuit1, 2),  # 40003
                (self._input_setpoint_circuit2, 5),  # 40006
            ):
                raw = self.get_state(entity_id)
                if raw in (None, "unavailable", "unknown"):
                    continue
                self.call_service(
                    "modbus/write_register",
                    hub=self._modbus_hub,
                    slave=self._modbus_slave,
                    address=address,
                    value=int(float(raw) * 10),
                )
            self.log("[HP] Modbus-Sync 40003/40006 OK", level="DEBUG")
        except Exception as e:
            self.log(f"[HP] Modbus-Sync fehlgeschlagen: {e}", level="WARNING")

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _on_sensor_change(self, entity, attribute, old, new, kwargs):
        self.evaluate_transitions()
        # Sollwerte haengen nur von AT und PV ab - nur bei diesen Sensoren neu schreiben.
        setpoint_relevant = entity in (
            self._sensor_outdoor_temp,
            self._sensor_pv_power_w,
        )
        if self.state == "heating":
            if self._ramp_c1_current is None and setpoint_relevant:
                # Kreis2 (40006) folgt der Heizkurve; Kreis1 (40003) bleibt
                # dynamisches LP-Tracking (Ratchet erhalten, nicht statisch ueberschreiben).
                self._update_heating_setpoint()
            # else: Ramp-Down aktiv - 40003 wird von _ramp_c1_step gesteuert
        elif self.state == "heating_forced" and setpoint_relevant:
            # Kreis2 (40006) auf aktuelle Heizkurve halten; Kreis1 via _on_wp_leaving_temp_change
            self._write_c2_direct(self._calc_flow_setpoint_smoothed())
        elif self.state == "buffer_drain" and setpoint_relevant:
            sp = self._calc_flow_setpoint_smoothed()
            self._write_c2_direct(sp)
            self._write_c1_direct(round(sp + self._circuit1_offset, 1))
        elif self.state in ("buffer_charge", "buffer_charge_boost"):
            self._update_charge_heating()

    def _on_param_change(self, entity, attribute, old, new, kwargs):
        self.log(f"[HP] Parameter geaendert: {entity} = {new}", level="INFO")
        self.evaluate_transitions()
        if self.state == "heating" and self._ramp_c1_current is None:
            self._update_heating_setpoint()
        elif self.state == "heating_forced":
            self._write_c2_direct(self._calc_flow_setpoint_smoothed())
        elif self.state == "buffer_drain":
            sp = self._calc_flow_setpoint_smoothed()
            self._write_c2_direct(sp)
            self._write_c1_direct(round(sp + self._circuit1_offset, 1))

    def _on_wp_leaving_temp_change(self, entity, attribute, old, new, kwargs):
        if self.state not in ("heating", "heating_forced"):
            return
        try:
            raw = float(new)
        except (TypeError, ValueError):
            return
        if self.state == "heating":
            # Im heating: Kreis1 dynamisch tracken, ABER nur wenn kein Ramp-Down
            # laeuft (Ramp-Down steuert 40003 selbst via _ramp_c1_step).
            if self._ramp_c1_current is None:
                self._update_heating_setpoint(raw)
            return
        if self._forced_lp_value is None:
            self._forced_lp_value = raw
        else:
            self._forced_lp_value = (
                self._forced_lp_alpha * raw
                + (1 - self._forced_lp_alpha) * self._forced_lp_value
            )
        raw_c1 = (
            self._forced_lp_value + self._forced_setpoint_offset + self._circuit1_offset
        )
        # Hard-Limit: Mitkopplungs-Hochlauf verhindern
        capped_c1 = min(self._get_param("heat_curve_vl_low"), raw_c1)
        # Ratchet: waehrend Mindestlaufzeit nur steigende Werte zulassen (Anti-Abschalt)
        if self._last_forced_c1 is not None:
            forced_c1 = round(max(capped_c1, self._last_forced_c1), 1)
        else:
            forced_c1 = round(capped_c1, 1)
        self._last_forced_c1 = forced_c1
        self._write_c1_direct(
            forced_c1
        )  # nur Kreis1 tracken; Kreis2 via _on_sensor_change
        self.log(
            f"[HP] heating_forced Sollwert: LP={self._forced_lp_value:.2f}C "
            f"-> Kreis1={forced_c1:.1f}C (raw={raw:.1f}C, cap={self._get_param('heat_curve_vl_low'):.0f}C)",
            level="DEBUG",
        )

    def _on_compressor_change(self, entity, attribute, old, new, kwargs):
        if new == "on":
            self._compressor_on_since = datetime.datetime.now()
            self._compressor_starts.append(datetime.datetime.now())
            self._check_compressor_cycling()
            if self.state == "heating":
                self.log("[HP] Kompressor EIN - Mindestlaufzeit-Timer laeuft")
        elif new == "off":
            self._compressor_on_since = None
        if new == "off" and self.state in ("heating", "heating_forced"):
            self.log(
                f"[HP] WARNUNG: Kompressor AUS im Zustand '{self.state}' - "
                "WP hat intern abgeschaltet (Stoerung / Abtauen?)",
                level="WARNING",
            )
            if self.state == "heating":
                # Kreis1 sofort zurueck auf berechneten Sollwert: sauberer Start fuer naechsten Zyklus
                sp = self._calc_flow_setpoint_smoothed()
                self._write_c1_direct(round(sp + self._circuit1_offset, 1))
                self._write_c2_direct(sp)
            # Transitions sofort pruefen: forced_end_time koennte bereits ueberschritten sein
            self.evaluate_transitions()

    def _check_compressor_cycling(self):
        """Verdichter-Takt-Schutz: warnt wenn zu viele Starts in kurzer Zeit.
        Fenster: cycling_window_minutes, Limit: cycling_max_starts.
        Loest Dry-Run aus wenn Limit ueberschritten (schont Verdichter)."""
        window_min = self._get_param("cycling_window_minutes")
        max_starts = int(self._get_param("cycling_max_starts"))
        cutoff = datetime.datetime.now() - datetime.timedelta(minutes=window_min)
        while self._compressor_starts and self._compressor_starts[0] < cutoff:
            self._compressor_starts.popleft()
        count = len(self._compressor_starts)
        if count >= max_starts:
            self.log(
                f"[HP] ! TAKT-SCHUTZ: {count} Verdichter-Starts in {window_min:.0f} min "
                f"(Limit: {max_starts}). Dry-Run aktiviert.",
                level="ERROR",
            )
            self._compressor_starts.clear()
            if self._dry_run_entity:
                try:
                    self.call_service(
                        "input_boolean/turn_on", entity_id=self._dry_run_entity
                    )
                    self.call_service(
                        "persistent_notification/create",
                        title="! WP Takt-Schutz ausgeloest",
                        message=(
                            f"{count} Verdichter-Starts in {window_min:.0f} min. "
                            "Dry-Run aktiviert. Ursache pruefen, dann Dry-Run manuell deaktivieren."
                        ),
                        notification_id="fsm_cycling",
                    )
                except Exception as e:
                    self.log(
                        f"[HP] Takt-Schutz aktivieren fehlgeschlagen: {e}",
                        level="ERROR",
                    )
        elif count >= max(2, max_starts - 2):
            self.log(
                f"[HP] WARNUNG: {count}/{max_starts} Verdichter-Starts in {window_min:.0f} min",
                level="WARNING",
            )

    def _on_aux_heater_change(self, entity, attribute, old, new, kwargs):
        state_name = "EIN" if new == "on" else "AUS"
        heater_num = "1" if entity == self._binary_aux_heater_1 else "2"
        self.log(
            f"[HP] Heizstab Stufe {heater_num} {state_name} | FSM-Zustand: {self.state}",
            level="INFO",
        )
        if new == "on" and self.state not in ("buffer_charge_boost", "hot_water"):
            self.log(
                f"[HP] INFO: Heizstab im Zustand '{self.state}' aktiv "
                f"(typisch beim Abtauen oder WW-Boost - DIP begrenzt auf 3 kW)",
                level="INFO",
            )

    def _on_force_event(self, event_name, data, kwargs):
        target = data.get("state")
        if target:
            self.force_state(target, reason=f"Event: {event_name}")

    def _on_dry_run_change(self, entity, attribute, old, new, kwargs):
        if new == "on":
            self.log(
                "[DRY-RUN] *** SIMULATIONSMODUS AKTIV *** FSM rechnet, aber kein Modbus-Write!",
                level="WARNING",
            )
        else:
            self.log(
                f"[DRY-RUN] *** SIMULATIONSMODUS BEENDET *** FSM-Zustand: {self.state} - Modbus-Writes aktiv",
                level="WARNING",
            )
            self._apply_current_state()

    def _apply_current_state(self):
        s = self.state
        self.log(f"[HP] Dry-Run beendet - synchronisiere Zustand '{s}' auf Modbus")
        if s in ("idle", "standby"):
            self._pump_off()
            self._set_wp_mode("off", 0)
            self._mixer_full_close()
        elif s == "heating":
            self._pump_on()
            sp = self._calc_flow_setpoint_smoothed()
            self._set_wp_mode("heat", sp)
            if self._ramp_c1_current is None:
                # Dynamisches LP-Tracking fortsetzen (vorhandenen _forced_lp_value
                # erhalten falls gesetzt, sonst aus aktuellem Auslass/Kurve initialisieren).
                self._update_heating_setpoint()
        elif s == "heating_forced":
            self._pump_on()
            lp = (
                self._forced_lp_value
                if self._forced_lp_value is not None
                else self._wp_leaving_temp()
            )
            forced_c1 = round(
                min(
                    self._get_param("heat_curve_vl_low"),
                    lp + self._forced_setpoint_offset + self._circuit1_offset,
                ),
                1,
            )
            self._set_forced_setpoint(forced_c1)
            self._start_mixer_controller()
        elif s == "hot_water":
            self._pump_off()
            self._set_wp_mode("hot_water", self._ww_target)
            self._mixer_full_close()
        elif s == "buffer_charge":
            self._set_energy_state(3)  # korrekt: wie on_enter_buffer_charge
            self._set_control_method(1)
            self._set_wp_mode("buffer_charge", self._buffer_charge_inlet_temp)
            self._update_charge_heating()
        elif s == "buffer_charge_boost":
            self._set_energy_state(self._boost_energy_state)
            self._set_control_method(1)
            self._set_wp_mode("buffer_charge", self._boost_inlet_temp)
            self._update_charge_heating()
        elif s == "buffer_drain":
            self._pump_on()
            self._set_energy_state(2)
            self._set_wp_mode("off", 0)
            sp = self._calc_flow_setpoint()
            self._start_mixer_controller()
            self.log(f"[HP] Dry-Run beendet (buffer_drain) - Vorlauf-Soll: {sp:.1f}C")

    def _is_werktag(self) -> bool:
        try:
            state = self.get_state(self._binary_werktag)
            if state not in (None, "unavailable", "unknown"):
                return state == "on"
        except Exception:
            pass
        return datetime.datetime.now().weekday() < 5

    def _restore_hw_window_if_active(self):
        """Nach Neustart: WW-Fenster wiederherstellen wenn gerade aktiv.
        _hw_window_active und der Close-Timer gehen beim Neustart verloren.
        """
        now = datetime.datetime.now()
        duration = datetime.timedelta(minutes=self._hw_duration_minutes)
        is_werktag = self._is_werktag()

        candidates = [self._hw_evening]  # Abend-Fenster immer pruefen
        candidates.append(
            self._hw_morning_weekday if is_werktag else self._hw_morning_weekend
        )

        for time_str in candidates:
            h, m = self._parse_time(time_str)
            window_start = now.replace(hour=h, minute=m, second=0, microsecond=0)
            window_end = window_start + duration
            if window_start <= now < window_end:
                remaining_s = (window_end - now).total_seconds()
                self._hw_window_active = True
                self.run_in(self._close_hw_window, remaining_s)
                self.evaluate_transitions()
                self.log(
                    f"[HP] WW-Fenster ({time_str}) nach Neustart wiederhergestellt "
                    f"- noch {remaining_s / 60:.0f} min"
                )
                return

    def _open_hw_window_if_werktag(self, kwargs):
        if self._is_werktag():
            self._open_hw_window(kwargs)

    def _open_hw_window_if_wochenende(self, kwargs):
        if not self._is_werktag():
            self._open_hw_window(kwargs)

    def _open_hw_window(self, kwargs):
        self.log(f"[HP] WW-Fenster oeffnet fuer {self._hw_duration_minutes} min")
        self._hw_window_active = True
        self.evaluate_transitions()
        self.run_in(self._close_hw_window, self._hw_duration_minutes * 60)

    def _close_hw_window(self, kwargs):
        self.log("[HP] WW-Fenster schliesst")
        self._hw_window_active = False
        self.evaluate_transitions()

    # -------------------------------------------------------------------------
    # Sensor-Lesehilfen
    # -------------------------------------------------------------------------

    def _outdoor_temp(self) -> float:
        """Letzten gueltigen AT-Wert cachen. Bei Sensorausfall wird der Cache gehalten
        statt auf 30°C zu springen, was den Sollwert schlagartig auf 18°C fallen lassen wuerde.
        Startup-Default 30°C (sicher warm) bis erster gueltiger Wert empfangen wird."""
        raw = self.get_state(self._sensor_outdoor_temp)
        try:
            val = float(raw)
            self._outdoor_temp_cache = val
            return val
        except (TypeError, ValueError):
            return (
                self._outdoor_temp_cache
                if self._outdoor_temp_cache is not None
                else 30.0
            )

    def _outdoor_temp_1h(self) -> float:
        """1h-Mittelwert AT fuer Exit-Entscheidungen (schneller als EMA).
        Fallback-Cache wie _outdoor_temp(); Startup-Default 30°C (sicher warm)."""
        raw = self.get_state(self._sensor_outdoor_temp_1h)
        try:
            val = float(raw)
            self._outdoor_temp_1h_cache = val
            return val
        except (TypeError, ValueError):
            return (
                self._outdoor_temp_1h_cache
                if self._outdoor_temp_1h_cache is not None
                else 30.0
            )

    def _flow_temp(self) -> float:
        """Letzten gueltigen VL-Wert cachen. Gibt None zurueck wenn nie ein gueltiger
        Wert empfangen wurde (Startup), sonst den Cache bei unavailable."""
        raw = self.get_state(self._sensor_flow_temp)
        try:
            val = float(raw)
            self._flow_temp_cache = val
            return val
        except (TypeError, ValueError):
            return self._flow_temp_cache if self._flow_temp_cache is not None else 30.0

    def _raw_inlet_temp(self) -> float:
        """Roher Einlass-Sensor (30003) ohne WP-Pumpen-Guard.
        Fuer Checks bei nicht laufender WP (Hysterese, Ladeentscheidung).
        """
        return self.get_numeric_state(self._sensor_wp_inlet_temp, default=0.0)

    def _wp_inlet_temp(self) -> float:
        """Wassereinlasstemperatur WP (Modbus 30003) - Regelgroesse bei 40002=1.
        Wenn Einlass = X°C, dann ist der gesamte Pufferspeicher >= X°C (Schichtung).
        Fallback 0.0°C wenn WP-Umwaelzpumpe nicht laeuft: veralteter Sensorwert koennte
        sonst faelschlicherweise einen fruehzeitigen Ausstieg aus buffer_charge ausloesen.
        """
        try:
            if self._binary_wp_pump and self.get_state(self._binary_wp_pump) != "on":
                return 0.0
        except Exception:
            pass
        return self.get_numeric_state(self._sensor_wp_inlet_temp, default=0.0)

    def _buffer_bottom(self) -> float:
        return self.get_numeric_state(self._sensor_buffer_bottom, default=35.0)

    def _buffer_mid(self) -> float:
        return self.get_numeric_state(self._sensor_buffer_mid, default=40.0)

    def _buffer_mid_high(self) -> float:
        return self.get_numeric_state(self._sensor_buffer_mid_high, default=44.0)

    def _buffer_top(self) -> float:
        return self.get_numeric_state(self._sensor_buffer_top, default=48.0)

    def _wp_leaving_temp(self) -> float:
        if not self._sensor_wp_leaving_temp:
            return self._flow_temp()
        return self.get_numeric_state(
            self._sensor_wp_leaving_temp, default=self._flow_temp()
        )

    def _pv_power_w(self) -> float:
        return self.get_numeric_state(self._sensor_pv_power_w, default=0.0)

    def _pv_energy_kwh(self) -> float:
        return self.get_numeric_state(self._sensor_pv_energy_kwh, default=0.0)

    def _parse_time(self, time_str: str) -> tuple[int, int]:
        h, m = time_str.split(":")
        return int(h), int(m)
