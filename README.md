# LG ThermaV Heat Pump Control — AppDaemon FSM

> **Deutsche Beschreibung** · [English description below](#lg-thermav-heat-pump-control--appdaemon-fsm-english)

---

## LG ThermaV Wärmepumpensteuerung — AppDaemon FSM (Deutsch)

Ereignisgesteuerte **Finite State Machine (FSM)** zur intelligenten Steuerung einer
LG ThermaV Wärmepumpe über Modbus TCP in Home Assistant. Läuft als AppDaemon-App (Docker).

### Funktionsumfang

- **Heizbetrieb** nach adaptiver Heizkurve mit PI-geregeltem 3-Wege-Mischer
- **Puffer-Entladung** — Haus heizen ohne Verdichter, solange Puffer warm genug
- **PV-Überschussladung** — Pufferspeicher mit Photovoltaik-Überschuss laden (40002=1 Einlass-Regelung)
- **Heizstab-Boost** — bei hohem PV-Überschuss Puffer auf 55 °C laden (inkl. Heizstab)
- **Warmwasservorrang** — konfigurierbare Zeitfenster (Werktag / Wochenende)
- **Verdichterschutz** — Mindestlaufzeit, Ramp-Down, Takt-Schutz
- **Sicherheitsmechanismen** — E-Stop, Dry-Run-Simulation, Sensor-Fallbacks

Alle Parameter sind zur Laufzeit über das HA-Dashboard (`input_number.hp_*`) anpassbar.

---

### Voraussetzungen

| Komponente | Details |
|---|---|
| Home Assistant | Mit `packages`-Setup, AppDaemon Add-on oder Container |
| Modbus TCP | Adapter an der WP (z. B. Waveshare RS485-zu-TCP) |
| ESPHome | `time_based` Cover für den 3-Wege-Mischer (Laufzeit konfigurierbar) |
| PV-Sensoren | Momentanleistung (W) und Tagesrest (kWh) in HA |

**Hydraulikkonzept — zwei Kreise:**
- **Kreis 1** (Modbus 40003): WP-interner Regelkreis. Der FSM schreibt hier den Sollwert, um die WP am Laufen zu halten (`heating_forced`) oder die Puffer-Einlasstemperatur vorzugeben.
- **Kreis 2** (Modbus 40006): Heizkreis mit Heizkörpern im Haus. Vorlauftemperatur wird durch den 3-Wege-Mischer (PI-Regler) geregelt; der FSM schreibt den Heizkurven-Sollwert.

---

### Architektur

```
FSMBase (fsm_base.py)
│  Wiederverwendbare Basisklasse: State-Verwaltung, Transitions,
│  HA-Integration (input_select bidirektional, Events, Logbuch),
│  E-Stop, Neustart-Retention.
│
└── HeatpumpFSM (heatpump_fsm.py)
      ├── Heizkurve (2-Punkt-linear + PV-Korrektur, Sollwert EWMA-gedämpft)
      ├── Mischer-PI (velocity-form, Step-Limit, Sub-1%-Akkumulation)
      ├── Pufferspeicher-Zonenlogik (4 Temperatursensoren)
      ├── PV-Überschussladung (40002=1, Einlass-Regelung)
      ├── WW-Zeitplanung (3 Fenster, Werktag/Wochenende)
      ├── Verdichterschutz (heating_forced + Ramp-Down)
      └── Takt-Schutz (Starts pro Stunde überwacht)
```

Parameter-Lesekette: `HA input_number` → `apps.yaml` → `_DEFAULTS`.

---

### Pufferspeicher-Zonenkonzept

```
Oben   ┌─────────────────────────┐  ← buffer_top     WW-Zone (Ziel: ww_target)
3/4    ├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┤  ← buffer_mid_high
1/2    ├═════════════════════════┤  ← buffer_mid     Vorlauf-Stutzen · Heizung EIN/AUS + buffer_drain-Schwelle
1/4    ├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┤  ← buffer_bottom  Rücklaufzone · nur Vorcheck PV-Ladung
Unten  └─────────────────────────┘
```

EIN- und AUS-Entscheidung: 1/2-Fühler (`buffer_mid`, Höhe des Vorlauf-Stutzens) — einheitlicher Schaltpunkt.
Der 1/4-Fühler ist die Rücklaufzone (bleibt im Heizbetrieb kalt) und dient nur als pumpenunabhängiger Vorcheck bei der PV-Ladung.

---

### FSM-Zustände

| Zustand | WP | Pumpe | Mischer | Energiezustand |
|---|---|---|---|---|
| `idle` | AUS | AUS | zu | 2 |
| `heating` | EIN | EIN | PI | 2 |
| `heating_forced` | EIN | EIN | PI | 2 |
| `hot_water` | EIN (WW) | AUS | zu | 2 |
| `buffer_charge` | EIN (40002=1) | AT-abh. | AT-abh. | 3 |
| `buffer_charge_boost` | EIN + Heizstab | AT-abh. | AT-abh. | 5 |
| `buffer_drain` | AUS | EIN | PI | 2 |
| `standby` | AUS | AUS | zu | 2 |

**AT-abhängig** (buffer_charge/boost): bei AT < Heizgrenze läuft Pumpe + Mischer-PI
(Haus wird parallel beheizt); bei AT ≥ Heizgrenze Pumpe AUS (Sommerbetrieb).

```
AT kalt, Puffer warm:  idle → buffer_drain → heating → standby → ...
AT kalt, Puffer kalt:  idle → heating → ...
Puffer 1/2 voll:       heating → heating_forced / standby
WW-Fenster:            (beliebig) → hot_water → standby
PV-Überschuss:         idle/standby/buffer_drain → buffer_charge → [boost] → standby
```

#### buffer_drain / heating Schwellen (dynamisch)

```
threshold_on  = Heizkurve(AT, ohne PV-Korrektur) + buffer_drain_margin
threshold_off = threshold_on + buffer_drain_hyst
```

Keine PV-Korrektur in den Schwellen (verhindert PV-Jitter-Takt).
Hysterese zwischen EIN und AUS verhindert Takt an der Grenze.

---

### Regelungskonzept

#### Heizkurve

```
slope    = (vl_low − vl_high) / (at_high − at_low)
base     = vl_high + (at_high − AT_1h) × slope
VL_roh   = clamp(base − pv_correction_per_kw × pv_kw_15min, 18, 60) °C
VL_Soll  = EWMA(VL_roh, α = setpoint_ewma_alpha)          ← Sollwert-Glättung
```

- AT: 1h-Mittelwert (`sensor.aussentemperatur_mittelwert_1h`).
- PV: bereits ein 15-min-Mittelwert — kein zusätzliches EWMA auf PV nötig.
- **Sollwert-EWMA** (α = 0,1, Zeitkonstante ≈ 45 s): der fertige Sollwert wird gedämpft bevor er in PI-Regler und 40006 geht. Schwellen-Berechnungen nutzen den Roh-Sollwert (keine Glättungsverzögerung bei Zustandswechseln).

#### Mischer-PI (velocity-form)

```
e     = VL_Soll − VL_Ist
delta = kp × (e − e_prev) + ki × e          ← velocity-form, kein Integral-Akkumulator
delta = clamp(delta, −max_step, +max_step)   ← Step-Limit gegen Überschwingen
pos   = clamp(pos + delta, 0, 100) %
```

Cover-Befehl nur bei Änderung des ganzzahligen Werts (Sub-1%-Akkumulation).
Step-Limit ≈ 4 % (= 100 % / Stellantrieb-Laufzeit × Regelzyklus).

#### PV-Pufferladung (40002=1)

Mit `40002=1` (Einlass-Regelung) regelt die WP auf den **Einlass** (Rücklauf von Puffer-Unten).
Einlass ≥ Ziel ⇒ gesamter Puffer ≥ Ziel (Schichtung). Natürliche „Puffer voll"-Bedingung,
kein Takt durch frühzeitige Auslass-Erkennung.

Eine **Mindestlaufzeit** (`buffer_charge_min_minutes`, Default 5 min) schützt vor Fehlausstieg im
Sommerbetrieb: Die WP-interne Pumpe liest beim Start sofort heißes Rohrrestwasser aus dem
vorherigen Zyklus — ohne Mindestlaufzeit würde der Ausstieg nach Sekunden fälschlicherweise feuern.

---

### Wichtigste Parameter

#### Heizkurve
| Parameter | Default | Beschreibung |
|---|---|---|
| `hp_heat_curve_at_high` | 16 °C | Oberer AT-Stützpunkt (Heizgrenze) |
| `hp_heat_curve_vl_high` | 27 °C | VL-Soll bei AT hoch |
| `hp_heat_curve_at_low` | −15 °C | Unterer AT-Stützpunkt (Auslegungspunkt) |
| `hp_heat_curve_vl_low` | 40 °C | VL-Soll bei AT niedrig |
| `hp_pv_correction_per_kw` | 0,2 °C/kW | VL-Absenkung pro kW PV |
| `hp_circuit1_offset` | 1 °C | Kreis-1-Aufschlag über Kreis 2 (WP läuft etwas wärmer als Heizkörper-VL) |

#### Puffer & Schwellen
| Parameter | Default | Beschreibung |
|---|---|---|
| `hp_heating_threshold` | 16 °C | AT-Heizgrenze |
| `hp_buffer_drain_margin` | 3 °C | Abstand der EIN-Schwelle über Heizkurve |
| `hp_buffer_drain_hyst` | 2 °C | Totband EIN/AUS |

#### Mischer-PI
| Parameter | Default | Beschreibung |
|---|---|---|
| `hp_mixer_kp` | 0,5 %/°C | Proportionalanteil |
| `hp_mixer_ki` | 0,1 %/°C | Integralanteil |
| `hp_mixer_interval_s` | 5 s | Regelzyklus |
| `hp_mixer_max_step_pct` | 4 % | Max-Schritt pro Zyklus |
| `hp_mixer_warmstart_position` | 20 % | Startposition wenn Mischer war zu |

#### Verdichterschutz
| Parameter | Default | Beschreibung |
|---|---|---|
| `hp_min_runtime_minutes` | 45 min | Mindestlaufzeit |
| `hp_forced_setpoint_max_c` | 50 °C | Kreis-1-Obergrenze in heating_forced |
| `hp_cycling_window_minutes` | 60 min | Takt-Schutz: Beobachtungsfenster |
| `hp_cycling_max_starts` | 6 | Takt-Schutz: Max-Starts → Dry-Run |

---

### Schutzmechanismen

| Mechanismus | Wirkung |
|---|---|
| **E-Stop** | > 10 FSM-Wechsel in 5 s → Dry-Run + HA-Benachrichtigung |
| **Takt-Schutz** | ≥ 6 Verdichter-Starts / 60 min → Dry-Run + Benachrichtigung |
| **Dry-Run** | FSM aktiv, kein Modbus-/Cover-Write; Hardware-Sync beim Deaktivieren |
| **AT-Fallback** | Letzter gültiger AT gecacht; Startup-Default 30 °C (kein Heizbedarf) |
| **VL-Fallback** | Letzter gültiger Vorlaufwert gecacht; PI pausiert bei fehlendem Cache |
| **Einlass-Guard** | Einlass-Temp → 0 wenn Umwälzpumpe aus (kein falscher buffer_charge-Exit) |
| **Mindestlaufzeit buffer_charge** | Ausstieg per Temperatur erst nach 5 min; verhindert Fehlausstieg durch Rohrrestwasser (Sommerbetrieb) |
| **Neustart-Restore** | FSM-Zustand aus `input_select` nach AppDaemon-Neustart wiederhergestellt |

---

### Modbus-Register (Auszug)

| Register | Typ | R/W | Beschreibung |
|---|---|---|---|
| 30003 | Input | R | Wassereinlass-Temp (Regelgröße bei 40002=1) |
| 30004 | Input | R | Wasserauslass-Temp |
| 40002 | Holding | RW | Steuermethode: 0=Auslass, 1=Einlass |
| 40003 | Holding | RW | Kreis-1-Sollwert (steuert WP-Verdichter) |
| 40006 | Holding | RW | Kreis-2-Sollwert (Heizkörper-Vorlauf, via Mischer geregelt) |
| 40009 | Holding | RW | WW-Solltemperatur |
| 40010 | Holding | RW | Energiezustand (2=Normal, 3=Empf.+, 5=Heizstab¹) |
| 10001/10002/10003 | Coil | W | Heizung / WW / Silent Mode |
| 20004 / 20005 | Discrete | R | Verdichter / Abtauen |

¹ Energiezustand 5 erlaubt der WP den Einsatz des Heizstabs (SG-Ready-ähnlich). Der Heizstab aktiviert sich in der Praxis nur bei kalter AT (AT < Heizgrenze), da die WP intern bei warmem Wetter keinen ausreichenden Bedarf erkennt.

---

### Dateistruktur

```
appdaemon/apps/
  heatpump_fsm.py               WP-spezifische FSM-Logik
  fsm_base.py                   Wiederverwendbare FSM-Basisklasse
  apps.yaml                     AppDaemon-Konfig (Entity-IDs, Schwellwerte)

homeassistant/my-config/packages/systems/
  lgthermav.yaml                Modbus-Konfiguration, Sensoren, Switches, COP
  fsm_heatpump.yaml             hp_*-Parameter, input_select, Modbus-Bridge

dashboard.yaml                  Lovelace-Dashboard (importierbar)
```

---

### Installation (Kurzanleitung)

1. `appdaemon/apps/*.py` und `apps.yaml` in das AppDaemon-`apps/`-Verzeichnis kopieren.
2. HA-Pakete (`lgthermav.yaml`, `fsm_heatpump.yaml`) ins Packages-Verzeichnis kopieren.
3. Entity-IDs in `apps.yaml` an die eigene Installation anpassen (Modbus-Hub, Cover, Sensoren).
4. `dashboard.yaml` als Lovelace-Dashboard importieren.
5. AppDaemon und HA neu starten.
6. **Erster Test:** `input_boolean.hp_dry_run = EIN` — FSM rechnet, kein Hardware-Write.
   Sensorwerte im Dashboard prüfen, dann Dry-Run deaktivieren.

---

### Lizenz

[PolyForm Noncommercial 1.0.0](LICENSE) — kostenlose Nutzung, Modifikation und Weitergabe für nichtkommerzielle Zwecke. Kommerzielle Nutzung oder kostenpflichtiges Anbieten des Codes ist nicht gestattet.

---
---

## LG ThermaV Heat Pump Control — AppDaemon FSM (English)

Event-driven **Finite State Machine (FSM)** for intelligent control of an LG ThermaV
heat pump via Modbus TCP in Home Assistant. Runs as an AppDaemon app (Docker).

### Features

- **Heating mode** following an adaptive heating curve with PI-controlled 3-way mixer
- **Buffer drain** — heat the house without the compressor while the buffer is warm enough
- **PV surplus charging** — charge buffer storage with photovoltaic surplus (Modbus 40002=1 inlet control)
- **Heating element boost** — charge buffer to 55 °C with high PV surplus (including heating rod)
- **Domestic hot water priority** — configurable time windows (weekday / weekend)
- **Compressor protection** — minimum runtime, ramp-down, cycling protection
- **Safety mechanisms** — E-Stop, dry-run simulation, sensor fallbacks

All parameters are adjustable at runtime via the HA dashboard (`input_number.hp_*`).

---

### Requirements

| Component | Details |
|---|---|
| Home Assistant | With `packages` setup, AppDaemon add-on or container |
| Modbus TCP | Adapter on the heat pump (e.g. Waveshare RS485-to-TCP) |
| ESPHome | `time_based` cover for the 3-way mixer (configurable travel time) |
| PV sensors | Instantaneous power (W) and daily remaining energy (kWh) in HA |

**Hydraulic concept — two circuits:**
- **Circuit 1** (Modbus 40003): internal heat pump control loop. The FSM writes a setpoint here to keep the compressor running (`heating_forced`) or to set the buffer inlet target.
- **Circuit 2** (Modbus 40006): house heating circuit with radiators. Supply temperature is regulated by the 3-way mixer (PI controller); the FSM writes the heating curve setpoint.

---

### Architecture

```
FSMBase (fsm_base.py)
│  Reusable base class: state management, transitions,
│  HA integration (bidirectional input_select, events, logbook),
│  E-Stop, restart retention.
│
└── HeatpumpFSM (heatpump_fsm.py)
      ├── Heating curve (2-point linear + PV correction, setpoint EWMA-smoothed)
      ├── Mixer PI controller (velocity-form, step limit, sub-1% accumulation)
      ├── Buffer storage zone logic (4 temperature sensors)
      ├── PV surplus charging (40002=1, inlet control)
      ├── DHW scheduling (3 windows, weekday/weekend)
      ├── Compressor protection (heating_forced + ramp-down)
      └── Cycling protection (starts per hour monitored)
```

Parameter read chain: `HA input_number` → `apps.yaml` → `_DEFAULTS`.

---

### Buffer Storage Zone Concept

```
Top    ┌─────────────────────────┐  ← buffer_top     DHW zone (target: ww_target)
3/4    ├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┤  ← buffer_mid_high
1/2    ├═════════════════════════┤  ← buffer_mid     Supply connection · Heating ON/OFF + buffer_drain threshold
1/4    ├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┤  ← buffer_bottom  Return zone · PV-charge pre-check only
Bottom └─────────────────────────┘
```

ON and OFF decision: 1/2 sensor (`buffer_mid`, supply connection height) — single switching point.
The 1/4 sensor is the return zone (stays cold during heating) and only serves as a pump-independent pre-check for PV charging.

---

### FSM States

| State | HP | Pump | Mixer | Energy state |
|---|---|---|---|---|
| `idle` | OFF | OFF | closed | 2 |
| `heating` | ON | ON | PI | 2 |
| `heating_forced` | ON | ON | PI | 2 |
| `hot_water` | ON (DHW) | OFF | closed | 2 |
| `buffer_charge` | ON (40002=1) | OAT-dep. | OAT-dep. | 3 |
| `buffer_charge_boost` | ON + rod | OAT-dep. | OAT-dep. | 5 |
| `buffer_drain` | OFF | ON | PI | 2 |
| `standby` | OFF | OFF | closed | 2 |

**OAT-dependent** (buffer_charge/boost): if OAT < heating threshold, pump + mixer PI run
(house heated in parallel); if OAT ≥ threshold, pump OFF (summer mode).

```
OAT cold, buffer warm:  idle → buffer_drain → heating → standby → ...
OAT cold, buffer cold:  idle → heating → ...
Buffer 1/2 full:        heating → heating_forced / standby
DHW window:             (any state) → hot_water → standby
PV surplus:             idle/standby/buffer_drain → buffer_charge → [boost] → standby
```

#### buffer_drain / heating thresholds (dynamic)

```
threshold_on  = heating_curve(OAT, no PV correction) + buffer_drain_margin
threshold_off = threshold_on + buffer_drain_hyst
```

No PV correction in thresholds (prevents PV-jitter cycling).
Hysteresis between ON and OFF prevents cycling at the boundary.

---

### Control Concept

#### Heating Curve

```
slope    = (vl_low − vl_high) / (at_high − at_low)
base     = vl_high + (at_high − OAT_1h) × slope
SP_raw   = clamp(base − pv_correction_per_kw × pv_kw_15min, 18, 60) °C
SP_flow  = EWMA(SP_raw, α = setpoint_ewma_alpha)           ← setpoint smoothing
```

- OAT: 1h mean (`sensor.aussentemperatur_mittelwert_1h`).
- PV: already a 15-min average — no additional EWMA on PV needed.
- **Setpoint EWMA** (α = 0.1, time constant ≈ 45 s): final setpoint is smoothed before being sent to PI controller and register 40006. Threshold calculations use the raw setpoint (no lag for state transitions).

#### Mixer PI Controller (velocity-form)

```
e     = SP_flow − T_flow_actual
delta = kp × (e − e_prev) + ki × e          ← velocity-form, no integral accumulator
delta = clamp(delta, −max_step, +max_step)   ← step limit prevents overshoot
pos   = clamp(pos + delta, 0, 100) %
```

Cover command only on integer position change (sub-1% accumulation in float).
Step limit ≈ 4 % (= 100 % / actuator travel time × control cycle).

#### PV Buffer Charging (40002=1)

With `40002=1` (inlet control), the HP regulates to the **inlet** temperature
(return from buffer bottom). Inlet ≥ target ⇒ entire buffer ≥ target (stratification).
Natural "buffer full" condition — no cycling from premature outlet detection.

---

### Key Parameters

#### Heating curve
| Parameter | Default | Description |
|---|---|---|
| `hp_heat_curve_at_high` | 16 °C | Upper OAT setpoint (heating limit) |
| `hp_heat_curve_vl_high` | 27 °C | Supply setpoint at high OAT |
| `hp_heat_curve_at_low` | −15 °C | Lower OAT setpoint (design point) |
| `hp_heat_curve_vl_low` | 40 °C | Supply setpoint at low OAT |
| `hp_pv_correction_per_kw` | 0.2 °C/kW | Supply setpoint reduction per kW PV |
| `hp_circuit1_offset` | 1 °C | Circuit 1 offset above circuit 2 (HP runs slightly warmer than radiator supply) |

#### Buffer & thresholds
| Parameter | Default | Description |
|---|---|---|
| `hp_heating_threshold` | 16 °C | OAT heating limit |
| `hp_buffer_drain_margin` | 3 °C | ON-threshold margin above heating curve |
| `hp_buffer_drain_hyst` | 2 °C | Dead band between ON and OFF |

#### Mixer PI
| Parameter | Default | Description |
|---|---|---|
| `hp_mixer_kp` | 0.5 %/°C | Proportional gain |
| `hp_mixer_ki` | 0.1 %/°C | Integral gain |
| `hp_mixer_interval_s` | 5 s | Control cycle |
| `hp_mixer_max_step_pct` | 4 % | Max step per cycle |
| `hp_mixer_warmstart_position` | 20 % | Start position when mixer was closed |

#### Compressor protection
| Parameter | Default | Description |
|---|---|---|
| `hp_min_runtime_minutes` | 45 min | Minimum runtime |
| `hp_forced_setpoint_max_c` | 50 °C | Circuit 1 upper limit in heating_forced |
| `hp_cycling_window_minutes` | 60 min | Cycling protection: observation window |
| `hp_cycling_max_starts` | 6 | Cycling protection: max starts → dry-run |

---

### Protection Mechanisms

| Mechanism | Effect |
|---|---|
| **E-Stop** | > 10 FSM transitions in 5 s → dry-run + HA notification |
| **Cycling protection** | ≥ 6 compressor starts / 60 min → dry-run + notification |
| **Dry-run** | FSM fully active, no Modbus/cover write; hardware sync on deactivation |
| **OAT fallback** | Last valid OAT cached; startup default 30 °C (no heating demand) |
| **Flow temp fallback** | Last valid flow temp cached; PI pauses if no cache (startup) |
| **Inlet guard** | Inlet temp → 0 when circulation pump off (prevents false buffer_charge exit) |
| **Restart restore** | FSM state restored from `input_select` after AppDaemon restart |

---

### Modbus Registers (excerpt)

| Register | Type | R/W | Description |
|---|---|---|---|
| 30003 | Input | R | Water inlet temp (control variable for 40002=1) |
| 30004 | Input | R | Water outlet temp |
| 40002 | Holding | RW | Control method: 0=outlet, 1=inlet |
| 40003 | Holding | RW | Circuit 1 setpoint (controls compressor) |
| 40006 | Holding | RW | Circuit 2 setpoint (radiator supply, regulated by mixer) |
| 40009 | Holding | RW | DHW setpoint |
| 40010 | Holding | RW | Energy state (2=Normal, 3=Recommended+, 5=Heating rod¹) |
| 10001/10002/10003 | Coil | W | Heating / DHW / Silent mode |
| 20004 / 20005 | Discrete | R | Compressor / Defrost |

¹ Energy state 5 permits the heat pump to use the heating rod (SG-Ready-style). In practice the rod only activates in cold weather (OAT < heating threshold); at warm OAT the HP's internal control does not see sufficient demand to engage it.

---

### File Structure

```
appdaemon/apps/
  heatpump_fsm.py               HP-specific FSM logic
  fsm_base.py                   Reusable FSM base class
  apps.yaml                     AppDaemon config (entity IDs, thresholds)

homeassistant/my-config/packages/systems/
  lgthermav.yaml                Modbus config, sensors, switches, COP templates
  fsm_heatpump.yaml             hp_* parameters, input_select, Modbus bridge automations

dashboard.yaml                  Lovelace dashboard (importable)
```

---

### Installation (Quick Start)

1. Copy `appdaemon/apps/*.py` and `apps.yaml` to your AppDaemon `apps/` directory.
2. Copy HA packages (`lgthermav.yaml`, `fsm_heatpump.yaml`) to your packages directory.
3. Adapt entity IDs in `apps.yaml` to your installation (Modbus hub, cover, sensors).
4. Import `dashboard.yaml` as a Lovelace dashboard.
5. Restart AppDaemon and Home Assistant.
6. **First test:** Set `input_boolean.hp_dry_run = ON` — FSM runs, no hardware writes.
   Verify sensor values in the dashboard, then deactivate dry-run.

---

### License

[PolyForm Noncommercial 1.0.0](LICENSE) — free to use, modify, and share for noncommercial purposes. Commercial use or selling the software is not permitted.
