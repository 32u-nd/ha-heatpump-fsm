"""
fsm_base.py - Wiederverwendbare Finite State Machine Basisklasse fuer AppDaemon
===============================================================================
Kopiere diese Datei in dein AppDaemon apps/-Verzeichnis und importiere sie
in deinen konkreten Apps.

Struktur:
  - FSMBase       -> Basisklasse, die du ableitest
  - Transition    -> Datenklasse fuer einen Zustandsuebergang
  - Hilfsmethoden fuer HA-Integration (Events, Input Select, Logbuch)
"""

import appdaemon.plugins.hass.hassapi as hass
from dataclasses import dataclass, field
from typing import Callable, Optional, Any
import collections
import datetime

# -----------------------------------------------------------------------------
# Datenklasse: ein einzelner Uebergang
# -----------------------------------------------------------------------------


@dataclass
class Transition:
    """
    Beschreibt einen moeglichen Zustandsuebergang.

    Felder:
      from_state   - Quellzustand (None = gilt aus jedem Zustand)
      to_state     - Zielzustand
      condition    - Callable() -> bool, muss True liefern damit der Uebergang stattfindet
      action       - Optionales Callable(), wird *nach* dem Uebergang ausgefuehrt
      label        - Menschenlesbarer Name fuer Logs / Events
    """

    from_state: Optional[str]
    to_state: str
    condition: Callable[[], bool]
    action: Optional[Callable[[], None]] = None
    label: str = ""


# -----------------------------------------------------------------------------
# Basisklasse
# -----------------------------------------------------------------------------


class FSMBase(hass.Hass):
    """
    Abstrakte Basisklasse fuer AppDaemon-basierte Finite State Machines.

    Ableiten & implementieren:
      1. define_states()       -> Liste aller gueltigen State-Strings zurueckgeben
      2. define_transitions()  -> Liste von Transition-Objekten zurueckgeben
      3. define_triggers()     -> Listener / Scheduler registrieren, die
                                 self.evaluate_transitions() aufrufen
      4. initial_state         -> Property: Start-Zustand (String)

    Optional ueberschreiben:
      on_enter_<state>(self)   -> wird beim Betreten eines Zustands aufgerufen
      on_exit_<state>(self)    -> wird beim Verlassen eines Zustands aufgerufen

    Konfiguration via apps.yaml:
      input_select_entity  - HA input_select zum Spiegeln des aktuellen States
                             (optional, leer lassen um zu deaktivieren)
      fire_events          - true/false, ob HA-Events gefeuert werden sollen
      log_transitions      - true/false, Uebergaenge ins HA-Logbuch schreiben
      event_prefix         - Praefix fuer gefeuerte Events, z.B. "fsm_washer"
    """

    # -- Pflicht-Properties / -Methoden in der Unterklasse --------------------

    @property
    def initial_state(self) -> str:
        raise NotImplementedError(
            "initial_state muss in der Unterklasse definiert werden"
        )

    def define_states(self) -> list[str]:
        raise NotImplementedError(
            "define_states() muss in der Unterklasse implementiert werden"
        )

    def define_transitions(self) -> list[Transition]:
        raise NotImplementedError(
            "define_transitions() muss in der Unterklasse implementiert werden"
        )

    def define_triggers(self):
        """Hier Listener/Scheduler registrieren - wird nach initialize() aufgerufen."""
        pass

    # -- AppDaemon Entry Point -------------------------------------------------

    def initialize(self):
        # Konfiguration aus apps.yaml lesen
        self._input_select = self.args.get("input_select_entity", "")
        self._fire_events = self.args.get("fire_events", True)
        self._log_trans = self.args.get("log_transitions", True)
        self._event_prefix = self.args.get(
            "event_prefix", self.__class__.__name__.lower()
        )
        # Verzoegerung bis on_enter des Startzustands ausgefuehrt wird.
        # HA-Services sind direkt nach initialize() noch nicht zuverlaessig verfuegbar.
        self._initial_enter_delay_s = float(self.args.get("initial_enter_delay_s", 5))

        # Validierung
        self._valid_states = self.define_states()
        assert (
            self.initial_state in self._valid_states
        ), f"initial_state '{self.initial_state}' nicht in define_states()"

        # State initialisieren
        self._state: str = self.initial_state
        self._previous_state: Optional[str] = None
        self._state_entered_at: datetime.datetime = datetime.datetime.now()

        # Zustand aus HA input_select wiederherstellen (Neustart-Retention).
        # get_state() ist in initialize() verfuegbar; call_service noch nicht.
        if self._input_select:
            try:
                saved = self.get_state(self._input_select)
                if saved and saved in self.define_states():
                    if saved != self._state:
                        self.log(
                            f"[FSM] Neustart-Restore: Zustand '{saved}' aus input_select",
                            level="INFO",
                        )
                    self._state = saved
            except Exception as e:
                self.log(f"[FSM] Neustart-Restore fehlgeschlagen: {e}", level="WARNING")

        # Transitions laden
        self._transitions: list[Transition] = self.define_transitions()

        # Guard: verhindert doppelten on_enter-Aufruf wenn eine Transition
        # im 5s-Fenster vor _enter_initial_state feuert.
        self._initial_enter_done = False

        # Pending-Liste fuer input_select-Writes (Race-Condition-Schutz)
        # Enthaelt alle Werte die wir selbst geschrieben haben, aber noch kein
        # HA-Event dafuer erhalten haben.
        self._input_select_pending: list[str] = []

        # E-Stop: Takt-Schutz - aktiviert estop_entity wenn die FSM innerhalb
        # von estop_window_seconds mehr als estop_max_transitions macht.
        self._estop_entity = self.args.get("estop_entity", "")
        self._estop_max_transitions = int(self.args.get("estop_max_transitions", 10))
        self._estop_window_s = float(self.args.get("estop_window_seconds", 5.0))
        self._transition_timestamps: collections.deque = collections.deque()

        # HA Input Select synchronisieren
        self._sync_input_select()

        # Unterklasse registriert ihre Trigger
        self.define_triggers()

        # input_select bidirektional: manuelle Auswahl in HA -> force_state
        if self._input_select:
            self.listen_state(self._on_input_select_manual, self._input_select)

        self.log(
            f"[FSM] {self.__class__.__name__} initialisiert. Startzustand: {self._state}"
        )

        # on_enter des Startzustands mit Delay ausfuehren (HA-Services erst nach ~2s verfuegbar)
        self.run_in(self._enter_initial_state, self._initial_enter_delay_s)

    # -- Oeffentliche API -------------------------------------------------------

    @property
    def state(self) -> str:
        """Aktueller FSM-Zustand."""
        return self._state

    @property
    def previous_state(self) -> Optional[str]:
        """Vorheriger Zustand (None beim Start)."""
        return self._previous_state

    @property
    def time_in_state(self) -> datetime.timedelta:
        """Wie lange ist die FSM schon im aktuellen Zustand?"""
        return datetime.datetime.now() - self._state_entered_at

    def _enter_initial_state(self, kwargs=None):
        """Wird 5s nach initialize() aufgerufen - HA-Services sind dann verfuegbar."""
        if self._initial_enter_done:
            self.log(
                f"[FSM] Initial-Enter uebersprungen (Transition kam zuerst, Zustand: {self._state})"
            )
            return
        # Transitions zuerst pruefen: wenn eine sofort greift (z.B. Neustart in
        # idle obwohl System eigentlich in buffer_drain laufen sollte), wird
        # _initial_enter_done in _do_transition gesetzt und on_enter_idle
        # (inkl. _mixer_full_close) wird nie ausgefuehrt.
        self.evaluate_transitions()
        if self._initial_enter_done:
            return
        self._initial_enter_done = True
        self.log(f"[FSM] Fuehre on_enter_{self._state}() aus (Start-Sync)")
        self._is_restart_enter = True
        self._call_hook(f"on_enter_{self._state}")
        self._is_restart_enter = False

    def evaluate_transitions(self, kwargs=None):
        """
        Prueft alle definierten Transitionen und fuehrt die erste zutreffende aus.
        Wird von Triggern in define_triggers() aufgerufen.
        """
        for t in self._transitions:
            # from_state pruefen (None = aus jedem Zustand erlaubt)
            if t.from_state is not None and t.from_state != self._state:
                continue
            # Bedingung pruefen
            try:
                if not t.condition():
                    continue
            except Exception as e:
                self.log(
                    f"[FSM] Fehler in condition fuer '{t.label}': {e}", level="ERROR"
                )
                continue
            # Uebergang ausfuehren
            self._do_transition(t)
            return  # nur einen Uebergang pro Aufruf

    def force_state(self, new_state: str, reason: str = "manuell"):
        """
        Erzwingt einen bestimmten Zustand (z.B. per HA-Service-Aufruf).
        Umgeht Bedingungspruefungen, aber respektiert on_exit/on_enter.
        """
        assert new_state in self._valid_states, f"Ungueltiger Zustand: {new_state}"
        dummy = Transition(
            from_state=self._state,
            to_state=new_state,
            condition=lambda: True,
            label=f"force ({reason})",
        )
        self._do_transition(dummy)

    # -- Interne Uebergangsmechanik ---------------------------------------------

    def _do_transition(self, t: Transition):
        old_state = self._state
        new_state = t.to_state
        self._initial_enter_done = (
            True  # on_enter folgt sofort - kein zweiter Initial-Enter
        )

        self.log(f"[FSM] {old_state} -> {new_state}  [{t.label}]")

        # Verweildauer im alten Zustand JETZT festhalten - nach dem Reset von
        # _state_entered_at (unten) waere time_in_state ~0 und das HA-Event wuerde
        # faelschlich "0:00:00" als duration_in_old_state melden.
        duration_in_old_state = self.time_in_state

        # on_exit des alten Zustands
        self._call_hook(f"on_exit_{old_state}")

        # State wechseln
        self._previous_state = old_state
        self._state = new_state
        self._state_entered_at = datetime.datetime.now()

        # on_enter des neuen Zustands
        self._call_hook(f"on_enter_{new_state}")

        # Optionale Action der Transition
        if t.action:
            try:
                t.action()
            except Exception as e:
                self.log(f"[FSM] Fehler in action fuer '{t.label}': {e}", level="ERROR")

        # HA-Integrationen
        self._sync_input_select()
        if self._fire_events:
            self._fire_ha_event(old_state, new_state, t.label, duration_in_old_state)
        if self._log_trans:
            self._write_logbook(old_state, new_state, t.label)

        # E-Stop: Takt-Erkennung nach jedem Zustandswechsel
        self._check_transition_rate()

    def _call_hook(self, hook_name: str):
        """Ruft on_enter_<state> / on_exit_<state> auf, falls vorhanden."""
        hook = getattr(self, hook_name, None)
        if callable(hook):
            try:
                hook()
            except Exception as e:
                self.log(f"[FSM] Fehler in Hook '{hook_name}': {e}", level="ERROR")

    # -- HA-Integrationen ------------------------------------------------------

    def _sync_input_select(self):
        """Spiegelt den aktuellen Zustand in einen HA input_select.

        Jeder Write wird in _input_select_pending eingetragen, damit
        _on_input_select_manual das zurueckkommende Event als eigenen Write
        erkennt - auch wenn sich self._state bis dahin schon wieder geaendert hat
        (Race Condition bei schnellen Zustandswechseln).
        """
        if not self._input_select:
            return
        self._input_select_pending.append(self._state)
        try:
            self.call_service(
                "input_select/select_option",
                entity_id=self._input_select,
                option=self._state,
            )
        except Exception as e:
            self.log(f"[FSM] input_select sync fehlgeschlagen: {e}", level="WARNING")

    def _on_input_select_manual(self, entity, attribute, old, new, kwargs):
        """Manuelle Auswahl im HA input_select -> erzwingt den gewaehlten Zustand.

        Race-Condition-Schutz: Vergleich gegen _input_select_pending statt gegen
        self._state. Hintergrund: Die FSM kann zwischen dem Schreiben (in
        _sync_input_select) und dem Eintreffen des HA-Events bereits in einen
        anderen Zustand gewechselt sein. Ein reiner Vergleich new == self._state
        wuerde dann faelschlicherweise force_state() ausloesen und einen
        Takt-Loop verursachen.
        """
        if new not in self._valid_states:
            return
        if new in self._input_select_pending:
            self._input_select_pending.remove(new)  # ersten Treffer entfernen
            return  # FSM hat selbst geschrieben -> ignorieren
        self.log(
            f"[FSM] Manuell via input_select: {old} -> {new}",
            level="INFO",
        )
        self.force_state(new, reason="input_select manuell")

    def _fire_ha_event(
        self, old: str, new: str, label: str, duration_in_old_state: datetime.timedelta
    ):
        """Feuert ein HA-Event beim Zustandswechsel."""
        event_type = f"{self._event_prefix}_state_changed"
        self.fire_event(
            event_type,
            old_state=old,
            new_state=new,
            transition=label,
            duration_in_old_state=str(duration_in_old_state),
        )

    def _write_logbook(self, old: str, new: str, label: str):
        """Schreibt einen Eintrag ins HA-Logbuch."""
        try:
            self.call_service(
                "logbook/log",
                name=self.__class__.__name__,
                message=f"Zustandswechsel: {old} -> {new} ({label})",
            )
        except Exception as e:
            self.log(f"[FSM] Logbuch-Eintrag fehlgeschlagen: {e}", level="WARNING")

    # -- E-Stop: Takt-Schutz --------------------------------------------------

    def _check_transition_rate(self):
        """Zaehlt Zustandswechsel im Sliding-Window und loest E-Stop aus wenn zu viele."""
        now = datetime.datetime.now()
        self._transition_timestamps.append(now)
        # Timestamps ausserhalb des Fensters entfernen
        cutoff = now - datetime.timedelta(seconds=self._estop_window_s)
        while self._transition_timestamps and self._transition_timestamps[0] < cutoff:
            self._transition_timestamps.popleft()
        # Limit ueberschritten?
        if len(self._transition_timestamps) > self._estop_max_transitions:
            self._trigger_estop()

    def _trigger_estop(self):
        """E-Stop: aktiviert estop_entity (Dry-Run), schreibt HA-Benachrichtigung."""
        count = len(self._transition_timestamps)
        msg = (
            f"{count} Zustandswechsel in {self._estop_window_s:.0f}s erkannt. "
            f"Simulationsmodus automatisch aktiviert. "
            f"Bitte Ursache pruefen, dann Dry-Run manuell deaktivieren."
        )
        self.log(f"[FSM] ! E-STOP: {msg}", level="ERROR")
        # Timestamps loeschen - kein Wiederhol-Trigger im selben Burst
        self._transition_timestamps.clear()
        # Dry-Run / estop_entity einschalten
        if self._estop_entity:
            try:
                self.call_service(
                    "input_boolean/turn_on",
                    entity_id=self._estop_entity,
                )
            except Exception as e:
                self.log(f"[FSM] E-Stop aktivieren fehlgeschlagen: {e}", level="ERROR")
        # Persistente HA-Benachrichtigung (bleibt bis manuell quittiert)
        try:
            self.call_service(
                "persistent_notification/create",
                title="! WP FSM E-Stop ausgeloest",
                message=msg,
                notification_id="fsm_estop",
            )
        except Exception as e:
            self.log(f"[FSM] Notification fehlgeschlagen: {e}", level="WARNING")

    # -- Hilfsmethoden fuer Unterklassen ----------------------------------------

    def get_numeric_state(self, entity_id: str, default: float = 0.0) -> float:
        """Liest einen numerischen Sensor-Wert sicher aus."""
        try:
            return float(self.get_state(entity_id))
        except (TypeError, ValueError):
            return default

    def is_state(self, entity_id: str, expected: str) -> bool:
        """Prueft ob eine HA-Entity einen bestimmten State hat."""
        return self.get_state(entity_id) == expected

    def minutes_in_state(self) -> float:
        """Minuten im aktuellen Zustand."""
        return self.time_in_state.total_seconds() / 60
