"""Reusable CANoe Communication Setup model for third-party instruments.

Instrument-specific modules should only provide a driver factory, a panel,
and a small schema describing the relationship between panel variables and
driver state. CANoe callbacks, worker/request queues, polling, safe shutdown,
and system-variable updates remain here.

The driver adapter contract is synchronous and runs on one worker thread:
``find_instruments``, ``connect``, ``configure_safe_defaults``,
``validate_desired_state``, ``apply_configuration``, ``set_output``,
``read_measurements``, ``safe_disable_output``, and ``close``.
"""

from __future__ import annotations

import queue
import threading
import time

import vector.canoe


CONNECTION_DISCONNECTED = 0
CONNECTION_CONNECTING = 1
CONNECTION_CONNECTED = 2
CONNECTION_FAULT = 3
OUTPUT_OFF = 0
OUTPUT_ON = 1
OUTPUT_UNKNOWN = -1


def _read_value(variable):
    """Return a primitive value from a CANoe converted/system variable."""
    if hasattr(variable, "impl_value"):
        return variable.impl_value
    snapshot = variable.copy()
    if hasattr(snapshot, "impl_value"):
        return snapshot.impl_value
    if hasattr(snapshot, "raw_value"):
        return snapshot.raw_value
    return snapshot


def _make_converted_value(variable, value):
    """Build a converted value before handing it to the update queue."""
    snapshot = variable.copy()
    if hasattr(snapshot, "impl_value"):
        snapshot.impl_value = value
        return snapshot
    if hasattr(snapshot, "raw_value"):
        snapshot.raw_value = value
        return snapshot
    return value


def create_application_model(
    *,
    model_name,
    panel,
    driver_factory,
    default_resource="",
    desired_state=None,
    driver_options=None,
    setpoint_variables=(),
    output_enable_variable="",
    output_state_variable="OutputState",
    poll_interval_seconds=0.5,
    setpoint_debounce_ms=200,
):
    """Create a CANoe-decorated model bound to one panel and driver.

    ``desired_state`` and ``driver_options`` map driver field names to either
    a system-variable name or ``(system-variable name, converter)``. Example::

        desired_state={"voltage": ("VoltageSetpoint", float)}

    ``setpoint_variables`` are debounced and sent as an ``apply`` request.
    Panel command variable names use the conventional keys: ``find_command``,
    ``connect_command``, ``disconnect_command``, and ``apply_command``.
    """
    system_variables = panel.system_variables
    desired_state = dict(desired_state or {})
    driver_options = dict(driver_options or {})
    setpoint_variables = tuple(setpoint_variables)
    poll_interval_seconds = float(poll_interval_seconds)
    setpoint_debounce_ms = int(setpoint_debounce_ms)

    def read_mapping(mapping):
        values = {}
        for key, spec in mapping.items():
            variable_name, converter = spec if isinstance(spec, tuple) else (spec, None)
            value = _read_value(getattr(system_variables, variable_name))
            values[key] = converter(value) if converter else value
        return values

    class ApplicationModel:
        """Universal CANoe-facing wrapper; instrument I/O stays in its adapter."""

        def initialize(self):
            self._requests = queue.Queue()
            self._updates = queue.Queue()
            self._stop_event = threading.Event()
            self._worker = None
            self._driver = None
            self._session_connected = False
            self._desired_state = {}
            self._pending_desired_state = {}
            self._timer_running = False
            self._suppress_setpoint_events = False
            self._suppress_output_events = False
            self._output_enable_debounce_pending = False
            self._output_requests_outstanding = 0
            self._confirmed_output_state = OUTPUT_UNKNOWN
            self._pending_output_state = OUTPUT_UNKNOWN
            self._measurement_start = time.monotonic()

            self._panel_timer = vector.canoe.MsTimer(50, self._drain_updates)
            self._setpoint_timer = vector.canoe.MsTimer(
                setpoint_debounce_ms, self._submit_desired_update
            )
            self._output_enable_timer = vector.canoe.MsTimer(
                setpoint_debounce_ms, self._submit_output_update
            )
            self._output_state_timer = vector.canoe.MsTimer(
                setpoint_debounce_ms, self._sync_output_enable_from_state
            )
            self._connection_values = {
                value: _make_converted_value(
                    getattr(system_variables, "ConnectionState"), value
                )
                for value in (
                    CONNECTION_DISCONNECTED,
                    CONNECTION_CONNECTING,
                    CONNECTION_CONNECTED,
                    CONNECTION_FAULT,
                )
            }
            self._output_values = {
                value: _make_converted_value(
                    getattr(system_variables, output_state_variable), value
                )
                for value in (OUTPUT_OFF, OUTPUT_ON, OUTPUT_UNKNOWN)
            }

        def start(self):
            self._stop_event.clear()
            self._pending_desired_state = self._snapshot_desired_state()
            system_variables.Connected = 0
            system_variables.ConnectionState = self._connection_values[
                CONNECTION_DISCONNECTED
            ]
            setattr(
                system_variables,
                output_state_variable,
                self._output_values[OUTPUT_UNKNOWN],
            )
            self._confirmed_output_state = OUTPUT_UNKNOWN
            self._pending_output_state = OUTPUT_UNKNOWN
            self._output_enable_debounce_pending = False
            self._output_requests_outstanding = 0
            self._output_state_timer.cancel()
            system_variables.OperationStatus = (
                "Ready; instrument output state not yet verified"
            )
            resource = getattr(system_variables, "ResourceName")
            if not str(resource.copy()).strip():
                system_variables.ResourceName = default_resource

            self._worker = threading.Thread(
                target=self._run_worker,
                name=f"{model_name}-driver",
                daemon=False,
            )
            self._timer_running = True
            self._panel_timer.start()
            self._worker.start()

        def stop(self):
            self._stop_event.set()
            self._requests.put(("shutdown",))
            if self._worker is not None:
                self._worker.join(timeout=5.0)
            self._timer_running = False
            self._panel_timer.cancel()
            self._setpoint_timer.cancel()
            self._output_enable_timer.cancel()
            self._output_state_timer.cancel()
            self._drain_updates()

        def shutdown(self):
            self._stop_event.set()

        def _on_variable_changed(self, action):
            if action == "find_command":
                self._command("FindCommand", "find", read_mapping(driver_options))
            elif action == "connect_command":
                resource = str(system_variables.ResourceName.copy()).strip()
                self._command(
                    "ConnectCommand",
                    "connect",
                    resource,
                    read_mapping(driver_options),
                    dict(self._pending_desired_state),
                )
                self._queue_update("ConnectionState", CONNECTION_CONNECTING)
            elif action == "disconnect_command":
                self._command("DisconnectCommand", "disconnect")
            elif action == "apply_command":
                # self._command("ApplyCommand", "apply", self._snapshot_desired_state())
                self._setpoint_timer.cancel()
                self._advance_apply_sequence()
                setattr(system_variables, "ApplyCommand", 0)

            elif action == "setpoint":
                self._schedule_setpoint_update()

            elif action == "output_enable":
                if self._suppress_output_events:
                    return
                self._schedule_output_update()

            elif action == "apply_sequence":
                sequence = int(_read_value(system_variables.ApplySequence))
                if sequence == int(_read_value(system_variables.AppliedSequence)):
                    return
                self._setpoint_timer.cancel()
                desired = self._snapshot_desired_state()
                self._pending_desired_state = desired
                if int(_read_value(system_variables.Connected)) != 1:
                    system_variables.OperationStatus = (
                        "Desired configuration pending; connect instrument to apply"
                    )
                    return

                self._requests.put(("apply", sequence, desired))

        def _command(self, variable_name, operation, *args):
            variable = getattr(system_variables, variable_name)
            if int(_read_value(variable)) != 1:
                return
            
            if operation == "connect":
                system_variables.OperationStatus = "Connecting..."

            self._requests.put((operation, *args))
            setattr(system_variables, variable_name, 0)

        def _schedule_setpoint_update(self):
            if self._suppress_setpoint_events:
                return
            
            if int(_read_value(system_variables.Connected)) != 1:
                self._pending_desired_state = self._snapshot_desired_state()
                system_variables.OperationStatus = (
                    "Setpoint changed; configuration will apply after connection"
                )
                return
            
            self._setpoint_timer.cancel()
            self._setpoint_timer.start()

        def _submit_desired_update(self):
            if int(_read_value(system_variables.Connected)) != 1:
                return
            
            desired = self._snapshot_desired_state()
            self._pending_desired_state = desired
            self._requests.put(("apply", desired))

        def _schedule_output_update(self):
            """Debounce button changes and submit only its latest requested value."""
            self._output_enable_debounce_pending = True
            self._output_enable_timer.cancel()
            self._output_enable_timer.start()

        def _submit_output_update(self):
            if not self._output_enable_debounce_pending:
                return
            self._output_enable_debounce_pending = False
            enabled = bool(
                int(_read_value(getattr(system_variables, output_enable_variable)))
            )
            requested_state = OUTPUT_ON if enabled else OUTPUT_OFF

            # Ignore a delayed CANoe on-change echo if the confirmed state is
            # already the requested state. Do not skip while another request
            # is outstanding, since a newer click may need to supersede it.
            if (
                self._output_requests_outstanding == 0
                and requested_state == self._confirmed_output_state
            ):
                return

            self._output_requests_outstanding += 1
            try:
                self._requests.put(("output", enabled), block=False)
            except Exception:
                self._output_requests_outstanding -= 1
                raise

        def _sync_output_enable_from_state(self):
            """Mirror a settled, confirmed output state to the enable button."""
            state = self._pending_output_state
            if state not in (OUTPUT_OFF, OUTPUT_ON):
                return
            if self._output_enable_debounce_pending or self._output_requests_outstanding:
                return
            if not output_enable_variable:
                return

            current_state = int(
                _read_value(getattr(system_variables, output_state_variable))
            )
            if current_state != state:
                return

            enable_variable = getattr(system_variables, output_enable_variable)
            confirmed_enabled = state == OUTPUT_ON
            if bool(int(_read_value(enable_variable))) != confirmed_enabled:
                self._suppress_output_events = True
                try:
                    setattr(
                        system_variables,
                        output_enable_variable,
                        int(confirmed_enabled),
                    )
                finally:
                    self._suppress_output_events = False

        def _advance_apply_sequence(self, *args):
            sequence = int(_read_value(system_variables.ApplySequence))

            if sequence == int(_read_value(system_variables.AppliedSequence)):
                return;
        
            sequence = 1 if sequence >= 0x7FFFFFFF else sequence + 1
            system_variables.ApplySequence = sequence
            
        def _snapshot_desired_state(self):
            return read_mapping(desired_state)

        def _queue_update(self, name, value):
            self._updates.put((name, value), block=False)

        def _drain_updates(self):
            while True:
                try:
                    name, value = self._updates.get(block=False)
                except queue.Empty:
                    break
                if name == "ConnectionState":
                    system_variables.ConnectionState = self._connection_values[value]
                elif name == output_state_variable:
                    state = int(value)
                    self._confirmed_output_state = state
                    self._pending_output_state = state
                    current = int(_read_value(getattr(system_variables, output_state_variable)))
                    if current != state:
                        self._suppress_output_events = True
                        try:
                            setattr(
                                system_variables,
                                output_state_variable,
                                self._output_values[state],
                            )
                        finally:
                            self._suppress_output_events = False

                    self._output_state_timer.cancel()
                    if state in (OUTPUT_OFF, OUTPUT_ON):
                        if self._timer_running:
                            self._output_state_timer.start()
                        else:
                            self._sync_output_enable_from_state()
                elif name in setpoint_variables:
                    self._suppress_setpoint_events = True
                    try:
                        setattr(system_variables, name, value)
                    finally:
                        self._suppress_setpoint_events = False
                elif output_enable_variable and name == output_enable_variable:
                    self._suppress_output_events = True
                    try:
                        setattr(system_variables, name, int(bool(value)))
                    finally:
                        self._suppress_output_events = False
                else:
                    setattr(system_variables, name, value)
            if self._timer_running:
                self._panel_timer.start()

        def _run_worker(self):
            next_poll = time.monotonic()
            while not self._stop_event.is_set():
                timeout = max(0.05, min(0.25, next_poll - time.monotonic()))
                try:
                    request = self._requests.get(timeout=timeout)
                    if self._handle_request(request):
                        break
                except queue.Empty:
                    pass
                except Exception as error:
                    self._report_error(error, additional_text=" during handling request")

                if self._session_connected and time.monotonic() >= next_poll:
                    try:
                        self._refresh_measurements()
                    except Exception as error:
                        self._report_error(error, disconnect=True)
                    next_poll = time.monotonic() + poll_interval_seconds
            self._close_session(disable_output=True)

        def _handle_request(self, request):
            match request[0]:
                case"shutdown":
                    return True
                case "find":
                    self._find_instrument(request[1] if len(request) > 1 else None)
                case "connect":
                    self._connect(
                        request[1],
                        request[2] if len(request) > 2 else None,
                        request[3] if len(request) > 3 else {},
                    )
                case "disconnect":
                    self._close_session(disable_output=True)
                    self._set_disconnected("Disconnected; safe-off requested")
                case "apply":
                    self._desired_state = dict(request[-1])
                    self._apply_configuration(request[-1])
                    self._refresh_measurements()

                    if len(request) >= 3:
                        self._queue_update("AppliedSequence", request[1])
                        self._queue_update(
                            "OperationStatus",
                            f"Applied sequence {request[1]}: {request[-1]["voltage"]:g} V / {request[-1]["voltage"]:g} A; "
                            f"output {'ON' if request[-1]["output_enabled"] else 'OFF'}",
                        )

                case "output":
                    try:
                        self._set_output(bool(request[1]))
                    finally:
                        self._output_requests_outstanding = max(
                            0, self._output_requests_outstanding - 1
                        )

            return False

        def _find_instrument(self, options=None):
            if self._driver is None:
                self._driver = driver_factory()
            matches = self._driver.find_instruments(options)
            if len(matches) == 1:
                self._queue_update("ResourceName", matches[0][0])
                self._queue_update("OperationStatus", f"Found {matches[0][1]}")
            elif not matches:
                self._queue_update("OperationStatus", f"No {model_name} device found")
            else:
                resources = ", ".join(resource for resource, _ in matches)
                self._queue_update(
                    "OperationStatus", f"Multiple matching resources; select one: {resources}"
                )

        def _connect(self, resource_name, options=None, initial_desired=None):
            self._close_session(disable_output=True)
            if not resource_name:
                raise ValueError("Device resource/name is empty")
            if self._driver is None:
                self._driver = driver_factory()
            try:
                identity = self._driver.connect(resource_name, options)
                self._driver.configure_safe_defaults()
                self._session_connected = True
                desired = dict(initial_desired or {})
                self._apply_configuration(desired)
                self._desired_state = desired
                self._queue_update("Connected", 1)
                self._queue_update("ConnectionState", CONNECTION_CONNECTED)
                self._queue_update(
                    "OperationStatus",
                    f"Connected: {identity}; configuration applied; output remains OFF",
                )
                self._refresh_measurements()
            except Exception:
                self._session_connected = False
                try:
                    self._driver.close()
                except Exception:
                    pass
                self._driver = None
                self._queue_update("Connected", 0)
                self._queue_update("ConnectionState", CONNECTION_FAULT)
                raise

        def _apply_configuration(self, desired):
            self._require_instrument()
            self._driver.validate_desired_state(desired)
            self._driver.apply_configuration(desired)
            self._queue_update("OperationStatus", "Configuration applied")

        def _set_output(self, enabled):
            if not self._session_connected or self._driver is None:
                if enabled:
                    if output_enable_variable:
                        self._queue_update(output_enable_variable, 0)
                    self._queue_update(
                        "OperationStatus",
                        "Output enable request rejected; instrument is disconnected",
                    )
                return
            if enabled:
                if not self._desired_state:
                    raise RuntimeError("Apply a valid configuration before enabling output")
                self._driver.validate_desired_state(self._desired_state)
            confirmed = self._driver.set_output(enabled)
            if confirmed is enabled:
                self._queue_update(
                    output_state_variable, OUTPUT_ON if enabled else OUTPUT_OFF
                )
            else:
                self._queue_update(output_state_variable, OUTPUT_UNKNOWN)
                raise RuntimeError("Driver did not confirm the requested output state")

        def _refresh_measurements(self):
            self._require_instrument()
            for name, value in self._driver.read_measurements().items():
                self._queue_update(name, value)

            sample_time_ms = (time.monotonic() - self._measurement_start) * 1000.0
            self._queue_update("LastUpdateMs", sample_time_ms)

        def _close_session(self, disable_output):
            driver = self._driver
            self._driver = None
            self._session_connected = False
            off_confirmed = False
            if driver is not None:
                if disable_output:
                    try:
                        off_confirmed = driver.safe_disable_output() is True
                    except Exception as error:
                        self._queue_update(
                            "OperationStatus",
                            f"UNSAFE: output-off could not be confirmed: {error}",
                        )
                try:
                    driver.close()
                except Exception:
                    pass
            self._queue_update("Connected", 0)
            self._queue_update(
                output_state_variable,
                OUTPUT_OFF if off_confirmed else OUTPUT_UNKNOWN,
            )
            return off_confirmed

        def _require_instrument(self):
            if not self._session_connected or self._driver is None:
                raise RuntimeError("Instrument driver is not connected")

        def _set_disconnected(self, status):
            self._queue_update("Connected", 0)
            self._queue_update("ConnectionState", CONNECTION_DISCONNECTED)
            self._queue_update("OperationStatus", status)

        def _report_error(self, error, disconnect=False, additional_text=str()):
            vector.canoe.write(f"[{model_name}] {error}")
            self._queue_update("OperationStatus", str(error) + additional_text)
            if disconnect:
                off_confirmed = self._close_session(disable_output=True)
                self._queue_update("ConnectionState", CONNECTION_FAULT)
                if not off_confirmed:
                    self._queue_update(output_state_variable, OUTPUT_UNKNOWN)
                    self._queue_update(
                        "OperationStatus",
                        f"UNSAFE: communication fault; output OFF unconfirmed: {error}",
                    )

    ApplicationModel.__name__ = f"{model_name.replace(' ', '').replace('-', '')}ApplicationModel"
    ApplicationModel.__qualname__ = ApplicationModel.__name__

    event_bindings = []

    default_actions = {
        "find_command": "FindCommand",
        "connect_command": "ConnectCommand",
        "disconnect_command": "DisconnectCommand",
        "apply_command": "ApplyCommand",
        "apply_sequence": "ApplySequence"
    }
    for action, variable_name in default_actions.items():
        if hasattr(system_variables, variable_name):
            event_bindings.append((variable_name, action))

    event_bindings.extend((name, "setpoint") for name in setpoint_variables)

    if output_enable_variable:
        event_bindings.append((output_enable_variable, "output_enable"))

    for index, (variable_name, action) in enumerate(event_bindings):
        def callback(self, _action=action):
            self._on_variable_changed(_action)

        callback.__name__ = f"on_{variable_name.lower()}_changed_{index}"
        decorated = vector.canoe.on_change(getattr(system_variables, variable_name))(callback)
        setattr(ApplicationModel, callback.__name__, decorated)

    ApplicationModel = vector.canoe.measurement_script(ApplicationModel)
    return ApplicationModel
