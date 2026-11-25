# LG ThermaV Heat Pump Control — AppDaemon FSM

> **Deutsche Beschreibung** · [English description below](#lg-thermav-heat-pump-control--appdaemon-fsm-english)

---

## LG ThermaV Wärmepumpensteuerung — AppDaemon FSM (Deutsch)

Ereignisgesteuerte **Finite State Machine (FSM)** für eine LG ThermaV Wärmepumpe mit Pufferspeicher.
Die Steuerung läuft als AppDaemon-App (Docker) und spricht die Wärmepumpe über Modbus TCP in Home Assistant an.

### Funktionsumfang

- **Heizen** nach Heizkurve mit gedämpfter Witterungsführung (Außentemperatur-EMA, einstellbare Zeitkonstante) und 3-Wege-Mischer mit Feedforward (invertierte Mischgleichung) und PI-Trim
- **Puffer-Entladung:** Der Puffer versorgt das Haus ohne Verdichterlauf, solange er warm genug ist.
- **PV-Überschussladung** des Puffers per Einlassregelung (Modbus 40002 = 1), optional mit **Heizstab-Boost** auf 55 °C
- **Warmwasser-Vorrang** in konfigurierbaren Zeitfenstern (Werktag/Wochenende), nur wenn der LG-WW-Fühler den Einschaltpunkt der Wärmepumpe unterschreitet
- **Silent Mode** dauerhaft aktiv; ausgenommen sind die Pufferladung und ein Erholungsfenster nach dem Abtauen
- **Urlaubsmodus:** kein Warmwasser, keine PV-Ladung, abgesenkte Heizkurve
- **Schutzfunktionen:** Verdichter-Mindestlaufzeit, Takt-Schutz, E-Stop, Dry-Run, Sensor-Fallbacks, Stillstands-Watchdog

Alle Parameter sind zur Laufzeit über das HA-Dashboard (`input_number.hp_*`) einstellbar.

### Voraussetzungen

| Komponente | Details |
|---|---|
| Home Assistant | mit `packages`-Setup; AppDaemon als Add-on oder Container |
| Modbus TCP | Adapter an der Wärmepumpe (z. B. Waveshare RS485-zu-TCP) |
| ESPHome | `time_based`-Cover für den 3-Wege-Mischer |
| PV-Sensoren | Momentanleistung (W) und verbleibender Tagesertrag (kWh) in HA |

### Hydraulikkonzept

Die Anlage hat zwei Heizkreise, die an einem Pufferspeicher hängen.

**Kreis 1** (Modbus 40003) ist der wärmepumpeninterne Regelkreis. Die LG schaltet den Verdichter ab, sobald der Auslass (30004) den Kreis-1-Sollwert um 4 K überschreitet. Die FSM führt deshalb in `heating` und `heating_forced` den Kreis-1-Sollwert dem gefilterten Auslass nach (Offset `forced_setpoint_offset`, Default −3 °C). So greift die interne Abschaltung nicht vor Ablauf der Mindestlaufzeit. Die Summe aus `forced_setpoint_offset` und `circuit1_offset` muss ≤ 0 bleiben, sonst entsteht Mitkopplung. Die FSM warnt beim Start, falls das nicht gilt. `heating` durchläuft drei Phasen:

1. **Startmodus** (Verdichter aus, `heating_restart_pause_minutes` seit dem letzten AUS vergangen): Kreis 1 = Puffer 1/4 + `heating_start_margin_c`. Der Auslassfühler ist im Stillstand veraltet, die Marge über dem Puffer lässt die LG sicher anlaufen.
2. **Tracking** (Verdichter läuft, bis `min_runtime_minutes`): Kreis 1 folgt dem Auslass in beide Richtungen, Untergrenze ist die Heizkurve.
3. **Absenkung** (ab Mindestlaufzeit und Puffer 1/2 ≥ `buffer_drain`-AUS-Schwelle): Kreis 1 sinkt mit 1 °C/10 min zur Heizkurve, die LG stoppt von selbst.

In `buffer_drain`, `idle` und `standby` folgt Kreis 1 der Heizkurve, in `buffer_charge` ist der Einlass-Zielwert der Sollwert.

**Kreis 2** (Modbus 40006) versorgt die Heizkörper. Die FSM schreibt die Heizkurve als Sollwert, die Vorlauftemperatur regelt der 3-Wege-Mischer.

### Pufferspeicher-Zonenkonzept

```
Oben   ┌─────────────────────────┐  buffer_top        WW-Zone
3/4    ├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┤  buffer_mid_high
1/2    ├═════════════════════════┤  buffer_mid        Höhe Vorlauf-Stutzen: Schaltpunkt Heizen/Entladen,
                                                      Neustart-Hysterese PV-Ladung
1/4    ├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┤  buffer_bottom     Rücklaufzone: Vorcheck PV-Ladung, Startmodus heating
Unten  └─────────────────────────┘
```

Alle Heizen-EIN/AUS-Entscheidungen und die Neustart-Hysterese nach PV-Ladung beziehen sich auf den 1/2-Fühler. Der 1/4-Fühler liegt in der Rücklaufzone, bleibt im Heizbetrieb kalt und dient als pumpenunabhängiger Vorcheck der PV-Ladung sowie als Bezug für den Startmodus. Warmwasser-Ein- und -Austritt richten sich nach dem LG-WW-Fühler (Modbus 30006, `sensor_ww_temp`), nicht nach dem Puffer.

### FSM-Zustände

| Zustand | WP | Pumpe | Mischer | Energiezustand |
|---|---|---|---|---|
| `idle` | AUS | AUS | **zu** | 2 |
| `heating` | EIN | EIN | PI | 2 |
| `heating_forced` | EIN | EIN | PI | 2 |
| `hot_water` | EIN (WW) | AUS | eingefroren | 2 |
| `buffer_charge` | EIN (40002 = 1) | AT-abh. | AT-abh. | 3 |
| `buffer_charge_boost` | EIN + Heizstab | AT-abh. | AT-abh. | 5 |
| `buffer_drain` | AUS | EIN | PI | 2 |
| `standby` | AUS | AUS | eingefroren | 2 |

*Eingefroren* heißt: Position wird gespeichert, es gibt keinen Fahrbefehl. Nur `idle` schließt den Mischer aktiv; der PI-Regler setzt danach von der gespeicherten Position fort. `heating_forced` hält den Verdichter bis zur Mindestlaufzeit am Laufen, wenn `heating` vorzeitig endet.

**AT-abhängig** (`buffer_charge`, `buffer_charge_boost`): Liegen EMA **und** 1-h-Mittel unter der Heizgrenze, laufen Pumpe und Mischer-PI (das Haus wird parallel beheizt). Meldet einer der beiden Sensoren ≥ Heizgrenze, bleibt die Pumpe aus (Sommerbetrieb).

```
AT kalt, Puffer warm:   idle → buffer_drain → heating → buffer_drain → …
AT kalt, Puffer kalt:   idle → heating → …
Verdichter intern aus:  heating → buffer_drain (Puffer warm) → heating (Puffer kalt)
Puffer 1/2 voll:        heating → heating_forced / standby
WW-Fenster, 30006 niedrig:  (beliebig) → hot_water → standby
PV-Überschuss:          idle / standby / buffer_drain → buffer_charge → [boost] → standby
```

**Warmwasser-Vorrang** ist konditional. Im WW-Fenster wechselt die FSM nur nach `hot_water`, wenn 30006 unter `ww_target − ww_lg_hysteresis` fällt, dem Einschaltpunkt der LG. Darüber lädt die LG ohnehin nicht, und der Heizkreis wird nicht unnötig unterbrochen. Fällt der Fühler erst während einer `standby`-Pause unter die Schwelle, startet WW nach der Pause. Der Austritt erfolgt bei 30006 ≥ `ww_target` oder nach dem Zeitlimit `hw_window_max_min`; bei ausgefallenem Fühler endet WW mit dem Fenster. Manuell erzwungenes WW (`force_state`, auch außerhalb des Fensters) endet bei 30006 ≥ `ww_target`, nach `hw_manual_max_min` oder durch manuelles Verlassen. Das Flag geht bei einem AppDaemon-Neustart verloren.

#### Schwellen für Entladen und Heizen

```
threshold_on  = Heizkurve(AT, ohne PV-Korrektur) + buffer_drain_margin
threshold_off = threshold_on + buffer_drain_hyst
```

Die Schwellen enthalten keine PV-Korrektur, damit PV-Schwankungen nicht zum Takten führen. Der Einstieg in `buffer_drain` ist zustandsabhängig: Aus `idle`/`standby` genügt `threshold_on`, aus `heating` gilt weiter `threshold_off`. Ohne diese Unterscheidung bliebe die FSM bei Heizbedarf in `idle`, solange der Puffer zwischen beiden Schwellen liegt. Die Hysterese beim Verlassen von `heating` verhindert dort das Takten.

### Regelungskonzept

#### Heizkurve

```
slope   = (vl_low − vl_high) / (at_high − at_low)
base    = vl_high + (at_high − AT_EMA) × slope
VL_roh  = clamp(base − pv_correction_per_kw × pv_kw_geglättet, 18, 60) °C
VL_Soll = EWMA(VL_roh, τ = setpoint_tau_s)
```

- **Außentemperatur:** `sensor.aussentemperatur_ema` ist exponentiell geglättet (τ = `hp_at_ema_tau_hours`, Default 24 h, einstellbar 1–72 h). Eingeschaltet wird nur, wenn EMA **und** 1-h-Mittel unter der Heizgrenze liegen. Ausgeschaltet wird, sobald einer der beiden Sensoren ≥ Heizgrenze meldet. So schalten widersprüchliche Sensoren nicht ein, und die Anlage taktet nicht an der Heizgrenze.
- **PV-Korrektur:** zusätzlich zeitbasiert geglättet (τ = `pv_smooth_minutes`). Wolkendurchzüge verschieben den Sollwert dadurch nicht und der Mischer fährt nicht unnötig.
- **Sollwert-Glättung:** zeitbasiert (τ = `setpoint_tau_s`) statt „Alpha pro Aufruf“, damit die Aufrufhäufigkeit die Zeitkonstante nicht verändert. Die Schwellenberechnungen nutzen den ungeglätteten Wert, Zustandswechsel erfahren also keine Verzögerung.
- **Urlaubsmodus:** senkt den Kreis-2-Sollwert um `vacation_setback_k`.

#### Mischer: Feedforward + PI-Trim

```
FF    = 100 × (VL_Soll − T_RL) / max(T_Tank − T_RL, dt_min, 0.1)     invertierte Mischgleichung
e     = VL_Soll − VL_Ist
e_eff = 0 innerhalb ±mixer_error_db_c, sonst um das Totband verkürzt
trim  = clamp(kp × e_eff + trim_i, −trim_limit, +trim_limit)         langsamer PI auf das Residuum
ideal = clamp(FF + trim, 0, 100) %, Slew-Limit ±max_step
```

Der Feedforward-Anteil berechnet die Ventilstellung direkt aus Puffer-Mitte `T_Tank` und Heizkreis-Rücklauf `T_RL`. Beide Temperaturen sind tiefpassgefiltert (τ = `mixer_sensor_tau_s`). `dt_min` regularisiert nur den Nenner, die Kennlinie bleibt stetig. Der PI-Trim korrigiert das Residuum außerhalb des Totbands und ist gegen Sättigung (Anti-Windup) geschützt. Mit `input_boolean.hp_mixer_ff = AUS` arbeitet der Regler als reiner Trim-Regler auf den vollen Fehler (A/B-Test).

**Stellantrieb-Schonung:** Ein Fahrbefehl geht nur raus, wenn |ideal − zuletzt kommandiert| ≥ `mixer_move_db_pct` und seit dem letzten Befehl `mixer_min_move_s` vergangen sind. Bei Abweichungen ≥ `mixer_big_move_pct` und an den Endanschlägen entfällt der Mindestabstand. Nach längerem Pumpenstillstand zeigt der Rücklauffühler einige Minuten veraltete Werte. Die Position wird deshalb `mixer_start_hold_s` gehalten, danach startet der Trim bei 0. Bei kurzen Unterbrechungen (< 2 min, Pumpe lief durch) bleibt der Trim-Integrator erhalten. Andernfalls ist die Vorbelegung auf `mixer_trim_init_max_pct` begrenzt, damit ein unplausibler Startwert keinen großen, lange stehenden Offset einbrennt.

#### PV-Pufferladung

Mit `40002 = 1` regelt die Wärmepumpe auf den **Einlass** (Rücklauf aus dem Puffer-Unten). Einlass ≥ Ziel bedeutet bei Schichtung: Der gesamte Puffer ist geladen. Das ist eine natürliche „Puffer voll“-Bedingung, ohne Takten durch zu frühe Auslasserkennung.

- **Start:** PV-Momentanleistung ≥ `pv_charge_min_w` (Default 2000 W). Die Prognose allein (`pv_buffer_min_kwh`) reicht nicht, sonst würde bei geringer Momentanleistung mit Netzstrom geladen.
- **Mindestlaufzeit** (`buffer_charge_min_minutes`, Default 5 min): Die interne Pumpe liest beim Start heißes Rohrrestwasser. Ohne Mindestlaufzeit würde der Temperaturausstieg sofort fälschlich auslösen.
- **Neustart-Hysterese** (`buffer_charge_hyst`, Default 5 °C): Nach „voll“ startet die nächste Ladung erst, wenn Puffer 1/2 unter `buffer_charge_inlet_temp − buffer_charge_hyst` fällt. Der 1/4-Fühler ist ungeeignet, er kühlt beim Entladen über den Rücklauf deutlich schneller aus als der übrige Puffer.
- **Heizstab-Boost:** PV muss `boost_pv_stable_minutes` (15 min) ununterbrochen ≥ `boost_pv_min_w` liegen. Nach dem Start läuft der Boost mindestens `boost_min_minutes` (30 min) bis zum weichen Ausstieg (PV zu schwach). Harte Gründe wie Abtauen oder Fensterende beenden ihn sofort.

#### Kreis-2-Pause bei PV-Deckung

In `heating` pausiert Kreis 2 (Pumpe aus, Mischer eingefroren), wenn der Heizkurven-Sollwert inklusive PV-Korrektur für `circuit2_pv_off_delay_s` (Default 1 h) unter `vl_high − circuit2_pv_off_hyst_c` liegt. Kreis 1 und Verdichter (Puffer/WW) bleiben unberührt. Wieder eingeschaltet wird bei Sollwert ≥ `vl_high`, die Hysterese wirkt nur auf der Abschaltseite. Fällt der AT-Sensor ohne Cache aus oder läuft die Startup-Grace, wird nie pausiert und eine laufende Pause sofort beendet.

### Architektur

```
FSMBase (fsm_base.py)
│  Wiederverwendbare Basisklasse: Zustandsverwaltung, Transitions,
│  HA-Integration (input_select bidirektional, Events, Logbuch),
│  E-Stop, Neustart-Retention
│
└── HeatpumpFSM (heatpump_fsm.py)
      ├── Heizkurve (2-Punkt-linear + PV-Korrektur, geglätteter Sollwert)
      ├── Mischer-Regelung (Feedforward + PI-Trim, Slew-Limit, Anti-Windup, Bumpless-Init)
      ├── Pufferspeicher-Zonenlogik (4 Temperaturfühler)
      ├── PV-Überschussladung (40002 = 1, Einlassregelung)
      ├── WW-Zeitplanung (3 Fenster, Werktag/Wochenende)
      ├── Verdichterschutz (heating_forced, Ramp-Down)
      └── Takt-Schutz (Starts pro Stunde)
```

Parameter-Lesekette: `HA input_number` → `apps.yaml` → `_DEFAULTS`.

### Wichtigste Parameter

#### Heizkurve
| Parameter | Default | Beschreibung |
|---|---|---|
| `hp_heat_curve_at_high` | 16 °C | Heizgrenze **und** oberer AT-Stützpunkt |
| `hp_heat_curve_vl_high` | 27 °C | VL-Soll bei AT hoch |
| `hp_heat_curve_at_low` | −15 °C | Unterer AT-Stützpunkt (Auslegungspunkt) |
| `hp_heat_curve_vl_low` | 40 °C | VL-Soll bei AT niedrig |
| `hp_pv_correction_per_kw` | 0.2 °C/kW | VL-Absenkung pro kW PV |
| `hp_circuit1_offset` | 1 °C | Kreis-1-Aufschlag über Kreis 2 |
| `hp_at_ema_tau_hours` | 24 h | Zeitkonstante der AT-EMA (1–72 h) |
| `pv_smooth_minutes` | 60 min | Zeitkonstante der PV-Glättung |
| `setpoint_tau_s` | 45 s | Sollwert-Glättung vor PI-Regler und 40006 |
| `vacation_setback_k` | 1 K | Absenkung Kreis 2 im Urlaubsmodus |

#### Puffer & Schwellen
| Parameter | Default | Beschreibung |
|---|---|---|
| `hp_buffer_drain_margin` | 1.5 °C | Abstand der EIN-Schwelle über der Heizkurve |
| `hp_buffer_drain_hyst` | 2 °C | Totband EIN/AUS |
| `pv_charge_min_w` | 2000 W | PV-Ladung startet erst ab dieser Momentanleistung |
| `buffer_charge_hyst` | 5 °C | Neustart-Hysterese nach Ladung (Bezug Puffer 1/2) |

#### Mischer
| Parameter | Default | Beschreibung |
|---|---|---|
| `hp_mixer_kp` / `hp_mixer_ki` | 0.4 / 0.01 %/°C | Trim-PI (T_i ≈ 200 s) |
| `hp_mixer_dt_min` | 2 °C | Untergrenze von (T_Tank − T_RL) im FF-Nenner |
| `hp_mixer_trim_limit_pct` | 15 % | Max. \|Trim\| um den FF-Arbeitspunkt |
| `hp_mixer_interval_s` | 5 s | Regelzyklus |
| `hp_mixer_max_step_pct` | 4 % | Max. Schritt pro Zyklus |
| `hp_mixer_warmstart_position` | 20 % | Startposition, wenn der Mischer zu war |
| `mixer_error_db_c` | 0.5 °C | Totband Soll-Ist für den Trim |
| `mixer_move_db_pct` | 3 %pt | Fahrbefehl erst ab \|Ideal − Ist\| ≥ x |
| `mixer_min_move_s` | 60 s | Mindestabstand zwischen Fahrbefehlen |
| `mixer_big_move_pct` | 10 %pt | Ab dieser Abweichung entfällt der Mindestabstand |
| `mixer_sensor_tau_s` | 60 s | Tiefpass Tank-/RL-Fühler im FF (0 = aus) |
| `mixer_start_hold_s` | 180 s | Positionshalten nach Pumpenstart |
| `mixer_trim_init_max_pct` | 8 %pt | Max. Trim-Vorbelegung beim Start |
| `input_boolean.hp_mixer_ff` | AN | Feedforward-Schalter (AUS = reiner Trim-Regler) |

#### Warmwasser & Silent Mode
| Parameter | Default | Beschreibung |
|---|---|---|
| `ww_lg_hysteresis` | 1 °C | WW-Modus erst bei 30006 < `ww_target` − x (LG-Einschaltpunkt) |
| `hw_window_max_min` | 90 min | Zeitlimit `hot_water` aus dem Fenster |
| `hw_manual_max_min` | 60 min | Zeitlimit bei manuell erzwungenem WW |
| `silent_mode_defrost_hold_h` | 0.5 h | Silent Mode nach Abtauende deaktiviert |

#### Heizstab-Boost
| Parameter | Default | Beschreibung |
|---|---|---|
| `hp_boost_pv_stable_minutes` | 15 min | PV muss so lange ununterbrochen ≥ `boost_pv_min_w` liegen |
| `hp_boost_min_minutes` | 30 min | Mindestlaufzeit bis zum weichen Ausstieg |

#### Kreis-2-Pause
| Parameter | Default | Beschreibung |
|---|---|---|
| `circuit2_pv_off_delay_s` | 3600 s | Mindestdauer unter `vl_high − hyst` bis Pumpe/Mischer aus |
| `circuit2_pv_off_hyst_c` | 1 °C | Hysterese auf der Abschaltseite |

#### Verdichterschutz
| Parameter | Default | Beschreibung |
|---|---|---|
| `hp_min_runtime_minutes` | 45 min | Mindestlaufzeit Verdichter |
| `hp_standby_minutes` | 3 min | Schutzpause zwischen Zyklen |
| `hp_forced_setpoint_offset` | −3 °C | Kreis-1-Offset zum gefilterten Auslass (30004); Summe mit `circuit1_offset` ≤ 0 |
| `hp_forced_lp_alpha` | 0.2 | LP-Filter-Alpha des Auslasses |
| `heating_start_margin_c` | 3 °C | Startmodus: Kreis 1 = Puffer 1/4 + Marge |
| `heating_restart_pause_minutes` | 10 min | Nach internem Verdichter-AUS kein Startmodus |
| `defrost_lp_hold_minutes` | 10 min | Kreis-1-Tracking während Abtauen und danach eingefroren |
| `hp_cycling_window_minutes` | 60 min | Takt-Schutz: Beobachtungsfenster |
| `hp_cycling_max_starts` | 6 | Takt-Schutz: Max. Starts bis Dry-Run |

### Schutzmechanismen

| Mechanismus | Wirkung |
|---|---|
| **E-Stop** | > 10 FSM-Wechsel in 5 s → Dry-Run + HA-Benachrichtigung |
| **Takt-Schutz** | ≥ 6 Verdichter-Starts in 60 min → Dry-Run + Benachrichtigung |
| **Dry-Run** | FSM rechnet voll, schreibt aber weder Modbus noch Cover; beim Deaktivieren Hardware-Sync |
| **AT-Fallback** | Asymmetrisch: Einschalt-Checks nutzen einen gecachten Wert bzw. den Startup-Default 30 °C (kein Heizbedarf). Abschalt-Checks liefern bei `unavailable` `None`, die WP wird nie auf einem Defaultwert abgeschaltet. |
| **VL-Fallback** | Letzter gültiger Vorlaufwert wird gehalten; ohne Cache pausiert der PI-Regler |
| **Einlass-Guard** | Bei stehender WP-Pumpe zählt der Einlass als 0 (kein Ausstieg auf veraltetem Wert); Ladeentscheidung und Hysterese nutzen den Rohwert |
| **Neustart-Restore** | FSM-Zustand aus `input_select`, „Puffer geladen“-Flag aus Puffer 1/2 (unbekannt zählt als geladen), Verdichterlaufzeit aus `binary_sensor.kompressor_20004` |
| **Startup-Grace** | Nach AppDaemon-Neustart (z. B. nach Backup) sind Sensoren kurz `unavailable`. `startup_grace_seconds` (180 s) unterdrückt alle WP-Abschalt-Transitionen; `initial_enter_delay_s` (30 s) gibt den Sensoren Zeit. |
| **Puffer-Gültigkeit** | Pufferfühler werden gecacht; bis zum ersten Messwert nach Neustart hält die FSM den Zustand, statt auf einem Default zu entscheiden |
| **AT-EMA-Persistenz** | `hp_at_ema_value` hat kein `initial:` (sonst überschreibt HA den gespeicherten Wert); Template-Sensor liefert bei unbekanntem Wert `none` |
| **Mischer-Positionserhalt** | `standby` und `hot_water` frieren die Position ein; nur `idle` schließt aktiv |
| **Silent Mode** | Dauerhaft aktiv, außer in `buffer_charge`/`buffer_charge_boost` und für `silent_mode_defrost_hold_h` nach dem Abtauen. Der Abgleich läuft zentral nach jedem Zustandswechsel und beim Start. |
| **Stillstands-Watchdog** | Bleibt der Verdichter in `heating`/`heating_forced` länger als `heating_stall_minutes` (90 min) aus, bei VL-Fehler ≥ `heating_stall_error_c` (2 °C), liegt die Ursache außerhalb der FSM (z. B. Kreis 1 am Gerät deaktiviert, Fehlercode). Die FSM meldet das einmalig per HA-Benachrichtigung. |
| **Schalter-Nachschreiben** | Liest der Poll für 10001/10002 einen anderen Wert als zuletzt geschrieben, schreibt die FSM nach `wp_switch_reassert_delay_s` erneut, höchstens `wp_switch_reassert_max`-mal je Fenster, danach Alarm |
| **Abtauzähler** | Jede Flanke `off→on` von `binary_sensor.abtauen_20005` erhöht zwei HA-Counter: einen Tageszähler (00:00 zurückgesetzt) und einen Gesamtzähler. Beide überleben Neustarts. |

### Modbus-Register (Auszug)

| Register | Typ | R/W | Beschreibung |
|---|---|---|---|
| 30003 | Input | R | Wassereinlass-Temperatur (Regelgröße bei 40002 = 1) |
| 30004 | Input | R | Wasserauslass-Temperatur (Bezug für Kreis 1 in `heating`/`heating_forced`) |
| 30006 | Input | R | WW-Fühler (Ein-/Austritt `hot_water`) |
| 40002 | Holding | RW | Steuermethode: 0 = Auslass, 1 = Einlass |
| 40003 | Holding | RW | Kreis-1-Sollwert (steuert den Verdichter) |
| 40006 | Holding | RW | Kreis-2-Sollwert (Heizkörper-Vorlauf, per Mischer geregelt) |
| 40009 | Holding | RW | WW-Solltemperatur |
| 40010 | Holding | RW | Energiezustand (2 = Normal, 3 = Empfohlen+, 5 = Heizstab¹) |
| 10001 / 10002 / 10003 | Coil | W | Heizung / WW / Silent Mode |
| 20004 / 20005 | Discrete | R | Verdichter / Abtauen |

¹ Energiezustand 5 erlaubt der WP den Heizstab (SG-Ready-ähnlich). Er springt nur bei kalter AT (< Heizgrenze) an, da die WP bei warmem Wetter intern keinen ausreichenden Bedarf erkennt.

### Dateistruktur

```
appdaemon/apps/
  heatpump_fsm.py               WP-spezifische FSM-Logik
  fsm_base.py                   Wiederverwendbare FSM-Basisklasse
  apps.yaml                     AppDaemon-Konfiguration (Entity-IDs, Schwellwerte)

homeassistant/my-config/packages/systems/
  lgthermav.yaml                Modbus-Konfiguration, Sensoren, Schalter, COP
  fsm_heatpump.yaml             hp_*-Parameter, input_select, Modbus-Bridge

dashboard.yaml                  Lovelace-Dashboard (importierbar)
```

### Installation

1. `appdaemon/apps/*.py` und `apps.yaml` in das AppDaemon-`apps/`-Verzeichnis kopieren.
2. HA-Pakete (`lgthermav.yaml`, `fsm_heatpump.yaml`) ins Packages-Verzeichnis kopieren.
3. Entity-IDs in `apps.yaml` an die eigene Installation anpassen (Modbus-Hub, Cover, Sensoren).
4. `dashboard.yaml` als Lovelace-Dashboard importieren.
5. AppDaemon und HA neu starten.
6. **Erster Test:** `input_boolean.hp_dry_run = EIN`. Die FSM rechnet, schreibt aber nicht. Sensorwerte im Dashboard prüfen, dann Dry-Run deaktivieren.
7. **Optional, Ventilkennlinie kalibrieren:** `input_boolean.hp_mixer_calibration_run` startet einen FOPDT-Stufentest (Positionen aus `input_text.hp_mixer_calibration_steps`, Haltezeit `input_number.hp_mixer_calibration_hold_min`) zur Ermittlung von `_VALVE_CAL` für die eigene Hydraulik.

### Lizenz

[PolyForm Noncommercial 1.0.0](LICENSE): kostenlose Nutzung, Modifikation und Weitergabe für nichtkommerzielle Zwecke. Kommerzielle Nutzung oder kostenpflichtiges Anbieten des Codes ist nicht gestattet.

---
---

## LG ThermaV Heat Pump Control — AppDaemon FSM (English)

Event-driven **finite state machine (FSM)** for an LG ThermaV heat pump with a buffer tank.
It runs as an AppDaemon app (Docker) and controls the heat pump via Modbus TCP in Home Assistant.

### Features

- **Space heating** following a heating curve with damped weather compensation (outdoor-temperature EMA, adjustable time constant) and a 3-way mixer with feedforward (inverted mixing equation) plus PI trim
- **Buffer drain:** the buffer supplies the house without compressor operation while it is warm enough.
- **PV surplus charging** of the buffer via inlet control (Modbus 40002 = 1), optionally with a **heating-rod boost** to 55 °C
- **DHW priority** in configurable time windows (weekday/weekend), only when the LG DHW sensor falls below the heat pump's own switch-on point
- **Silent mode** on permanently, except during buffer charging and a recovery window after defrost
- **Vacation mode:** no DHW, no PV charging, lowered heating curve
- **Protection:** minimum compressor runtime, cycling protection, E-stop, dry-run, sensor fallbacks, stall watchdog

All parameters can be adjusted at runtime through the HA dashboard (`input_number.hp_*`).

### Requirements

| Component | Details |
|---|---|
| Home Assistant | with `packages` setup; AppDaemon as add-on or container |
| Modbus TCP | adapter on the heat pump (e.g. Waveshare RS485-to-TCP) |
| ESPHome | `time_based` cover for the 3-way mixer |
| PV sensors | instantaneous power (W) and remaining daily yield (kWh) in HA |

### Hydraulic Concept

The system has two heating circuits connected to one buffer tank.

**Circuit 1** (Modbus 40003) is the heat pump's internal control loop. The LG stops the compressor once the outlet (30004) exceeds the Circuit 1 setpoint by 4 K. In `heating` and `heating_forced` the FSM therefore tracks the Circuit 1 setpoint to the filtered outlet (offset `forced_setpoint_offset`, default −3 °C), so the internal cut-off does not trigger before the minimum runtime has elapsed. The sum of `forced_setpoint_offset` and `circuit1_offset` must stay ≤ 0, otherwise positive feedback results; the FSM logs a warning at startup if it does not. `heating` has three phases:

1. **Start mode** (compressor off, `heating_restart_pause_minutes` since the last OFF): Circuit 1 = buffer 1/4 + `heating_start_margin_c`. The outlet sensor is stale at standstill, and the margin above the buffer lets the LG start reliably.
2. **Tracking** (compressor running, up to `min_runtime_minutes`): Circuit 1 follows the outlet in both directions, with the heating curve as lower bound.
3. **Ramp-down** (after the minimum runtime and buffer 1/2 ≥ `buffer_drain` OFF threshold): Circuit 1 falls at 1 °C/10 min toward the heating curve and the LG stops on its own.

In `buffer_drain`, `idle` and `standby` Circuit 1 follows the heating curve; in `buffer_charge` the inlet target is the setpoint.

**Circuit 2** (Modbus 40006) supplies the radiators. The FSM writes the heating curve as setpoint, and the 3-way mixer regulates the supply temperature.

### Buffer Storage Zone Concept

```
Top    ┌─────────────────────────┐  buffer_top        DHW zone
3/4    ├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┤  buffer_mid_high
1/2    ├═════════════════════════┤  buffer_mid        Supply connection height: heating/drain switching point,
                                                      PV-charge restart hysteresis
1/4    ├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┤  buffer_bottom     Return zone: PV-charge pre-check, heating start mode
Bottom └─────────────────────────┘
```

All heating ON/OFF decisions and the restart hysteresis after PV charging refer to the 1/2 sensor. The 1/4 sensor sits in the return zone, stays cold during heating, and serves as a pump-independent pre-check for PV charging and as reference for the start mode. DHW entry and exit follow the LG DHW sensor (Modbus 30006, `sensor_ww_temp`), not the buffer.

### FSM States

| State | HP | Pump | Mixer | Energy state |
|---|---|---|---|---|
| `idle` | OFF | OFF | **closed** | 2 |
| `heating` | ON | ON | PI | 2 |
| `heating_forced` | ON | ON | PI | 2 |
| `hot_water` | ON (DHW) | OFF | frozen | 2 |
| `buffer_charge` | ON (40002 = 1) | OAT-dep. | OAT-dep. | 3 |
| `buffer_charge_boost` | ON + rod | OAT-dep. | OAT-dep. | 5 |
| `buffer_drain` | OFF | ON | PI | 2 |
| `standby` | OFF | OFF | frozen | 2 |

*Frozen* means the position is saved and no travel command is issued. Only `idle` actively closes the mixer; the PI controller resumes from the saved position afterwards. `heating_forced` keeps the compressor running until the minimum runtime has elapsed when `heating` ends early.

**OAT-dependent** (`buffer_charge`, `buffer_charge_boost`): if the EMA **and** the 1-h average are below the heating limit, pump and mixer PI run (the house is heated in parallel). If either sensor reads ≥ the heating limit, the pump stays off (summer mode).

```
OAT cold, buffer warm:   idle → buffer_drain → heating → buffer_drain → …
OAT cold, buffer cold:   idle → heating → …
Compressor stops itself: heating → buffer_drain (buffer warm) → heating (buffer cold)
Buffer 1/2 full:         heating → heating_forced / standby
DHW window, 30006 low:   (any) → hot_water → standby
PV surplus:              idle / standby / buffer_drain → buffer_charge → [boost] → standby
```

**DHW priority** is conditional. Within the DHW window the FSM only enters `hot_water` when 30006 drops below `ww_target − ww_lg_hysteresis`, the LG's own switch-on point. Above it the LG does not charge anyway, and the heating circuit is not interrupted needlessly. If the sensor only drops below the threshold during a `standby` pause, DHW starts after the pause. Exit occurs at 30006 ≥ `ww_target` or after the time limit `hw_window_max_min`; with a failed sensor, DHW ends with the window. Manually forced DHW (`force_state`, also outside the window) ends at 30006 ≥ `ww_target`, after `hw_manual_max_min`, or when the state is left manually. The flag is lost on an AppDaemon restart.

#### Thresholds for Drain and Heating

```
threshold_on  = heating_curve(OAT, no PV correction) + buffer_drain_margin
threshold_off = threshold_on + buffer_drain_hyst
```

The thresholds contain no PV correction, so PV fluctuations do not cause cycling. Entry into `buffer_drain` is state-dependent: from `idle`/`standby`, `threshold_on` suffices; from `heating`, `threshold_off` still applies. Without this distinction the FSM would stay in `idle` despite heating demand while the buffer sits between the two thresholds. The hysteresis on leaving `heating` prevents cycling there.

### Control Concept

#### Heating Curve

```
slope    = (vl_low − vl_high) / (at_high − at_low)
base     = vl_high + (at_high − OAT_EMA) × slope
SP_raw   = clamp(base − pv_correction_per_kw × pv_kw_smoothed, 18, 60) °C
SP_flow  = EWMA(SP_raw, τ = setpoint_tau_s)
```

- **Outdoor temperature:** `sensor.aussentemperatur_ema` is exponentially smoothed (τ = `hp_at_ema_tau_hours`, default 24 h, adjustable 1–72 h). Heating switches on only if the EMA **and** the 1-h average are below the heating limit, and off as soon as either reads ≥ the limit. Conflicting sensors therefore do not switch on, and the system does not cycle at the heating limit.
- **PV correction:** additionally time-smoothed (τ = `pv_smooth_minutes`), so passing clouds neither shift the setpoint nor make the mixer move unnecessarily.
- **Setpoint smoothing:** time-based (τ = `setpoint_tau_s`) rather than "alpha per call", so call frequency does not alter the time constant. Threshold calculations use the unsmoothed value, so state transitions see no lag.
- **Vacation mode:** lowers the Circuit 2 setpoint by `vacation_setback_k`.

#### Mixer: Feedforward + PI Trim

```
FF    = 100 × (SP_flow − T_return) / max(T_tank − T_return, dt_min, 0.1)   inverted mixing equation
e     = SP_flow − T_flow_actual
e_eff = 0 within ±mixer_error_db_c, otherwise shortened by the dead band
trim  = clamp(kp × e_eff + trim_i, −trim_limit, +trim_limit)               slow PI on the residual
ideal = clamp(FF + trim, 0, 100) %, slew limit ±max_step
```

The feedforward term computes the valve position directly from buffer-mid `T_tank` and heating-circuit return `T_return`, both low-pass filtered (τ = `mixer_sensor_tau_s`). `dt_min` only regularizes the denominator, so the characteristic stays continuous. The PI trim corrects the residual outside the dead band and is protected against saturation (anti-windup). With `input_boolean.hp_mixer_ff = OFF` the controller acts as a plain trim controller on the full error (A/B test).

**Actuator protection:** a move command is only issued if |ideal − last commanded| ≥ `mixer_move_db_pct` and `mixer_min_move_s` has elapsed since the last command. The minimum gap does not apply for deviations ≥ `mixer_big_move_pct` or at the end stops. After a long pump stop the return sensor shows stale values for a few minutes, so the position is held for `mixer_start_hold_s` and the trim then starts at 0. On short interruptions (< 2 min, pump kept running) the trim integrator is retained. Otherwise the pre-load is capped at `mixer_trim_init_max_pct`, so an implausible start value cannot bake in a large, long-lasting offset.

#### PV Buffer Charging

With `40002 = 1` the heat pump regulates to the **inlet** temperature (return from the buffer bottom). With stratification, inlet ≥ target means the whole buffer is charged. This is a natural "buffer full" condition without cycling from premature outlet detection.

- **Start:** instantaneous PV power ≥ `pv_charge_min_w` (default 2000 W). The forecast alone (`pv_buffer_min_kwh`) is not sufficient, otherwise charging would draw grid power at low instantaneous PV.
- **Minimum runtime** (`buffer_charge_min_minutes`, default 5 min): the internal pump reads hot residual pipe water at start. Without a minimum runtime the temperature exit would fire immediately and wrongly.
- **Restart hysteresis** (`buffer_charge_hyst`, default 5 °C): after "full", the next charge starts only once buffer 1/2 drops below `buffer_charge_inlet_temp − buffer_charge_hyst`. The 1/4 sensor is unsuitable, since it cools much faster than the rest of the buffer while draining through the return.
- **Heating-rod boost:** PV must stay ≥ `boost_pv_min_w` continuously for `boost_pv_stable_minutes` (15 min). Once started, boost runs for at least `boost_min_minutes` (30 min) before the soft exit (PV too weak) applies. Hard reasons such as defrost or window end stop it immediately.

#### Circuit 2 Pause under PV Coverage

In `heating`, Circuit 2 pauses (pump off, mixer frozen) when the heating-curve setpoint including PV correction stays below `vl_high − circuit2_pv_off_hyst_c` for `circuit2_pv_off_delay_s` (default 1 h). Circuit 1 and compressor (buffer/DHW) are unaffected. It resumes at setpoint ≥ `vl_high`; the hysteresis acts only on the shut-off side. If the OAT sensor fails without a cache, or startup grace is active, it never pauses and ends an active pause immediately.

### Architecture

```
FSMBase (fsm_base.py)
│  Reusable base class: state management, transitions,
│  HA integration (bidirectional input_select, events, logbook),
│  E-stop, restart retention
│
└── HeatpumpFSM (heatpump_fsm.py)
      ├── Heating curve (2-point linear + PV correction, smoothed setpoint)
      ├── Mixer control (feedforward + PI trim, slew limit, anti-windup, bumpless init)
      ├── Buffer zone logic (4 temperature sensors)
      ├── PV surplus charging (40002 = 1, inlet control)
      ├── DHW scheduling (3 windows, weekday/weekend)
      ├── Compressor protection (heating_forced, ramp-down)
      └── Cycling protection (starts per hour)
```

Parameter read chain: `HA input_number` → `apps.yaml` → `_DEFAULTS`.

### Key Parameters

#### Heating curve
| Parameter | Default | Description |
|---|---|---|
| `hp_heat_curve_at_high` | 16 °C | Heating limit **and** upper OAT reference point |
| `hp_heat_curve_vl_high` | 27 °C | Supply setpoint at high OAT |
| `hp_heat_curve_at_low` | −15 °C | Lower OAT reference point (design point) |
| `hp_heat_curve_vl_low` | 40 °C | Supply setpoint at low OAT |
| `hp_pv_correction_per_kw` | 0.2 °C/kW | Supply reduction per kW PV |
| `hp_circuit1_offset` | 1 °C | Circuit 1 offset above Circuit 2 |
| `hp_at_ema_tau_hours` | 24 h | OAT EMA time constant (1–72 h) |
| `pv_smooth_minutes` | 60 min | PV smoothing time constant |
| `setpoint_tau_s` | 45 s | Setpoint smoothing before PI controller and 40006 |
| `vacation_setback_k` | 1 K | Circuit 2 setback in vacation mode |

#### Buffer & thresholds
| Parameter | Default | Description |
|---|---|---|
| `hp_buffer_drain_margin` | 1.5 °C | ON-threshold margin above the heating curve |
| `hp_buffer_drain_hyst` | 2 °C | Dead band between ON and OFF |
| `pv_charge_min_w` | 2000 W | PV charging starts only above this instantaneous power |
| `buffer_charge_hyst` | 5 °C | Restart hysteresis after charging (reference: buffer 1/2) |

#### Mixer
| Parameter | Default | Description |
|---|---|---|
| `hp_mixer_kp` / `hp_mixer_ki` | 0.4 / 0.01 %/°C | Trim PI (T_i ≈ 200 s) |
| `hp_mixer_dt_min` | 2 °C | Lower bound of (T_tank − T_return) in the FF denominator |
| `hp_mixer_trim_limit_pct` | 15 % | Max. \|trim\| around the FF operating point |
| `hp_mixer_interval_s` | 5 s | Control cycle |
| `hp_mixer_max_step_pct` | 4 % | Max. step per cycle |
| `hp_mixer_warmstart_position` | 20 % | Start position when the mixer was closed |
| `mixer_error_db_c` | 0.5 °C | Setpoint/actual dead band for the trim |
| `mixer_move_db_pct` | 3 %pt | Move only once \|ideal − actual\| ≥ x |
| `mixer_min_move_s` | 60 s | Minimum gap between move commands |
| `mixer_big_move_pct` | 10 %pt | Above this deviation the minimum gap no longer applies |
| `mixer_sensor_tau_s` | 60 s | Low-pass on tank/return sensors in FF (0 = off) |
| `mixer_start_hold_s` | 180 s | Position hold after pump start |
| `mixer_trim_init_max_pct` | 8 %pt | Max. trim pre-load at start |
| `input_boolean.hp_mixer_ff` | ON | Feedforward switch (OFF = plain trim controller) |

#### DHW & silent mode
| Parameter | Default | Description |
|---|---|---|
| `ww_lg_hysteresis` | 1 °C | DHW mode only at 30006 < `ww_target` − x (LG switch-on point) |
| `hw_window_max_min` | 90 min | Time limit for `hot_water` entered from the window |
| `hw_manual_max_min` | 60 min | Time limit for manually forced DHW |
| `silent_mode_defrost_hold_h` | 0.5 h | Silent mode disabled after defrost ends |

#### Heating-rod boost
| Parameter | Default | Description |
|---|---|---|
| `hp_boost_pv_stable_minutes` | 15 min | PV must stay ≥ `boost_pv_min_w` this long without interruption |
| `hp_boost_min_minutes` | 30 min | Minimum runtime before the soft exit applies |

#### Circuit 2 pause
| Parameter | Default | Description |
|---|---|---|
| `circuit2_pv_off_delay_s` | 3600 s | Minimum time below `vl_high − hyst` before pump/mixer shut off |
| `circuit2_pv_off_hyst_c` | 1 °C | Hysteresis on the shut-off side |

#### Compressor protection
| Parameter | Default | Description |
|---|---|---|
| `hp_min_runtime_minutes` | 45 min | Minimum compressor runtime |
| `hp_standby_minutes` | 3 min | Protective pause between cycles |
| `hp_forced_setpoint_offset` | −3 °C | Circuit 1 offset to the filtered outlet (30004); sum with `circuit1_offset` ≤ 0 |
| `hp_forced_lp_alpha` | 0.2 | LP filter alpha of the outlet |
| `heating_start_margin_c` | 3 °C | Start mode: Circuit 1 = buffer 1/4 + margin |
| `heating_restart_pause_minutes` | 10 min | No start mode after an internal compressor OFF |
| `defrost_lp_hold_minutes` | 10 min | Circuit 1 tracking frozen during and after defrost |
| `hp_cycling_window_minutes` | 60 min | Cycling protection: observation window |
| `hp_cycling_max_starts` | 6 | Cycling protection: max. starts before dry-run |

### Protection Mechanisms

| Mechanism | Effect |
|---|---|
| **E-stop** | > 10 FSM transitions in 5 s → dry-run + HA notification |
| **Cycling protection** | ≥ 6 compressor starts in 60 min → dry-run + notification |
| **Dry-run** | FSM computes fully but writes neither Modbus nor cover; hardware sync on deactivation |
| **OAT fallback** | Asymmetric: turn-on checks use a cached value or the startup default 30 °C (no heating demand). Turn-off checks return `None` when `unavailable`, so the heat pump is never switched off on a default value. |
| **Flow-temp fallback** | Last valid flow temperature is held; without a cache the PI controller pauses |
| **Inlet guard** | With the HP pump stopped the inlet counts as 0 (no exit on a stale value); charge decision and hysteresis use the raw value |
| **Restart restore** | FSM state from `input_select`, "buffer charged" flag from buffer 1/2 (unknown counts as charged), compressor runtime from `binary_sensor.kompressor_20004` |
| **Startup grace** | After an AppDaemon restart (e.g. after a backup) sensors are briefly `unavailable`. `startup_grace_seconds` (180 s) suppresses all heat-pump-off transitions; `initial_enter_delay_s` (30 s) gives sensors time. |
| **Buffer validity** | Buffer sensors are cached; until the first reading after a restart the FSM holds its state instead of deciding on a default |
| **OAT EMA persistence** | `hp_at_ema_value` has no `initial:` (otherwise HA overrides the stored value); the template sensor returns `none` when unknown |
| **Mixer position retention** | `standby` and `hot_water` freeze the position; only `idle` closes actively |
| **Silent mode** | On permanently, except in `buffer_charge`/`buffer_charge_boost` and for `silent_mode_defrost_hold_h` after defrost. Reconciled centrally after every state change and at start. |
| **Stall watchdog** | If the compressor stays off in `heating`/`heating_forced` for longer than `heating_stall_minutes` (90 min) with a flow error ≥ `heating_stall_error_c` (2 °C), the cause lies outside the FSM (e.g. Circuit 1 disabled on the unit, error code). The FSM reports it once via an HA notification. |
| **Switch re-assertion** | If polling reads a different value for 10001/10002 than last written, the FSM rewrites after `wp_switch_reassert_delay_s`, at most `wp_switch_reassert_max` times per window, then raises an alarm |
| **Defrost counter** | Each `off→on` edge of `binary_sensor.abtauen_20005` increments two HA counters: a daily one (reset at 00:00) and a total. Both survive restarts. |

### Modbus Registers (excerpt)

| Register | Type | R/W | Description |
|---|---|---|---|
| 30003 | Input | R | Water inlet temperature (control variable at 40002 = 1) |
| 30004 | Input | R | Water outlet temperature (reference for Circuit 1 in `heating`/`heating_forced`) |
| 30006 | Input | R | DHW sensor (entry/exit of `hot_water`) |
| 40002 | Holding | RW | Control method: 0 = outlet, 1 = inlet |
| 40003 | Holding | RW | Circuit 1 setpoint (controls the compressor) |
| 40006 | Holding | RW | Circuit 2 setpoint (radiator supply, regulated by the mixer) |
| 40009 | Holding | RW | DHW setpoint |
| 40010 | Holding | RW | Energy state (2 = Normal, 3 = Recommended+, 5 = Heating rod¹) |
| 10001 / 10002 / 10003 | Coil | W | Heating / DHW / Silent mode |
| 20004 / 20005 | Discrete | R | Compressor / Defrost |

¹ Energy state 5 permits the heat pump to use the heating rod (SG-Ready style). It only engages at cold OAT (< heating limit), as the unit sees no sufficient demand internally in warm weather.

### File Structure

```
appdaemon/apps/
  heatpump_fsm.py               HP-specific FSM logic
  fsm_base.py                   Reusable FSM base class
  apps.yaml                     AppDaemon configuration (entity IDs, thresholds)

homeassistant/my-config/packages/systems/
  lgthermav.yaml                Modbus configuration, sensors, switches, COP
  fsm_heatpump.yaml             hp_* parameters, input_select, Modbus bridge

dashboard.yaml                  Lovelace dashboard (importable)
```

### Installation

1. Copy `appdaemon/apps/*.py` and `apps.yaml` to your AppDaemon `apps/` directory.
2. Copy the HA packages (`lgthermav.yaml`, `fsm_heatpump.yaml`) to your packages directory.
3. Adapt the entity IDs in `apps.yaml` to your installation (Modbus hub, cover, sensors).
4. Import `dashboard.yaml` as a Lovelace dashboard.
5. Restart AppDaemon and Home Assistant.
6. **First test:** set `input_boolean.hp_dry_run = ON`. The FSM computes but does not write. Verify sensor values in the dashboard, then deactivate dry-run.
7. **Optional, calibrate the valve characteristic:** `input_boolean.hp_mixer_calibration_run` starts a FOPDT step test (positions from `input_text.hp_mixer_calibration_steps`, hold time `input_number.hp_mixer_calibration_hold_min`) to determine `_VALVE_CAL` for your own hydraulics.

### License

[PolyForm Noncommercial 1.0.0](LICENSE): free to use, modify, and share for noncommercial purposes. Commercial use or selling the software is not permitted.
