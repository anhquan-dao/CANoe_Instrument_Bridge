"""Self-contained RIGOL DL3021 driver using the reusable adapter contract.

This is the primary RIGOL driver implementation for the template-based CANoe
binding. It owns VISA discovery, session configuration, SCPI, CC-mode limits,
safe-off, and measurements. It deliberately does not import
RigolDL3021_DriverAdapter.py or any CANoe module.
"""

from __future__ import annotations

import math
import logging
import tempfile
from enum import IntEnum
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any, Iterator, Mapping, Sequence

try:
    import pyvisa
except ImportError:  # Allows offline examples and contract checks without VISA installed.
    pyvisa = None

from Instrument_Driver_Adapter_Template import (
    ConfigurableInstrument,
    DeviceInfo,
    DiscoverableInstrument,
    InstrumentDriverAdapter,
    MeasurementPollingInstrument,
    OutputControlInstrument,
    SafeStateInstrument,
)


MAX_VOLTAGE = 48.0
MAX_CURRENT = 10.0
MAX_POWER = 480.0
INPUT_OFF = 0
INPUT_ON = 1
LOG_DIRECTORY = Path(tempfile.gettempdir()) / Path(__file__).stem
LOG_FILE = LOG_DIRECTORY / "driver.log"


class RigolOperatingMode(IntEnum):
    """Operating modes exposed by this adapter.

    IntEnum keeps the CANoe bridge's existing integer-based Mode variable
    compatible while giving driver code a descriptive mode identifier.
    Only constant-current mode is currently implemented.
    """

    CURR = 0


def _get_driver_logger() -> logging.Logger:
    """Return a shared rotating logger under %TEMP%/<driver-module-name>."""
    logger = logging.getLogger(Path(__file__).stem)
    logger.setLevel(logging.INFO)
    logger.propagate = False
    LOG_DIRECTORY.mkdir(parents=True, exist_ok=True)

    log_path = str(LOG_FILE.resolve())
    if not any(
        isinstance(handler, RotatingFileHandler)
        and handler.baseFilename == log_path
        for handler in logger.handlers
    ):
        handler = RotatingFileHandler(
            LOG_FILE,
            maxBytes=2_000_000,
            backupCount=3,
            encoding="utf-8",
        )
        handler.setFormatter(
            logging.Formatter("%(asctime)s %(levelname)s %(message)s")
        )
        logger.addHandler(handler)
    return logger


class _BridgeDeviceInfo(DeviceInfo):
    """DeviceInfo with pair-like access for the current CANoe bridge API."""

    def __iter__(self) -> Iterator[str]:
        yield self.resource_name
        yield self.identity

    def __getitem__(self, index: int) -> str:
        return (self.resource_name, self.identity)[index]

    def __len__(self) -> int:
        return 2

    def __str__(self) -> str:
        return self.identity


class RigolDL3021TemplateDriverAdapter(
    InstrumentDriverAdapter,
    DiscoverableInstrument,
    ConfigurableInstrument,
    OutputControlInstrument,
    MeasurementPollingInstrument,
    SafeStateInstrument,
):
    """RIGOL DL3021 VISA/SCPI implementation with CC-load safety logic."""

    INPUT_CHANNEL = "input"
    OPERATING_MODE = [RigolOperatingMode.CURR]
    MODE_BY_RESPONSE = {
        "CC": RigolOperatingMode.CURR,
    }
    MODE_COMMAND = {
        RigolOperatingMode.CURR: ("CURR", "A", "CC"),
    }

    # DL3021 static-mode reference table. This documents the mode-specific
    # setpoint, selectable ranges, and related settings for future adapter
    # work; it does not imply that CV/CR/CP are currently implemented here.
    # Range endpoints are from the RIGOL DL3000 User Guide. Use instrument
    # MIN/MAX queries for runtime validation, since the usable operating point
    # is also constrained by the DL3021's 150 V / 40 A / 200 W ratings.
    MODE_SETTING_LOOKUP = {
        "CC": {
            "function": "CURR",
            "setpoint": "Current",
            "configuration_key": "setpoint",
            "unit": "A",
            "set_command": ":SOUR:CURR:LEV:IMM",
            "query_command": ":SOUR:CURR:LEV:IMM?",
            "range_command": ":SOUR:CURR:RANG",
            "ranges": {
                "low": (0.0, 4.0),
                "high": (0.0, 40.0),
            },
            "additional_settings": (
                "Slew",
                "Von",
                "V_Limit",
                "C_Limit",
            ),
            "implemented": True,
        },
        "CV": {
            "function": "VOLT",
            "setpoint": "Voltage",
            "configuration_key": "setpoint",
            "unit": "V",
            "set_command": ":SOUR:VOLT:LEV:IMM",
            "query_command": ":SOUR:VOLT:LEV:IMM?",
            "range_command": ":SOUR:VOLT:RANG",
            "ranges": {
                "low": (0.0, 15.0),
                "high": (0.0, 150.0),
            },
            "additional_settings": ("V_Limit", "C_Limit"),
            "implemented": False,
        },
        "CR": {
            "function": "RES",
            "setpoint": "Resistance",
            "configuration_key": "setpoint",
            "unit": "ohm",
            "set_command": ":SOUR:RES:LEV:IMM",
            "query_command": ":SOUR:RES:LEV:IMM?",
            "range_command": ":SOUR:RES:RANG",
            "ranges": {
                "low": (0.08, 15.0),
                "high": (2.0, 15000.0),
            },
            "additional_settings": ("Von", "V_Limit", "C_Limit"),
            "notes": (
                "Low resistance range may not work normally above 10 V input.",
            ),
            "implemented": False,
        },
        "CP": {
            "function": "POW",
            "setpoint": "Power",
            "configuration_key": "setpoint",
            "unit": "W",
            "set_command": ":SOUR:POW:LEV:IMM",
            "query_command": ":SOUR:POW:LEV:IMM?",
            # CP has no selectable measurement range in the user guide.
            "range_command": None,
            "ranges": None,
            "additional_settings": ("V_Limit", "C_Limit"),
            "implemented": False,
        },
    }

    def __init__(self):
        self._resource_managers = {}
        self._instrument = None
        self._resource_name = ""
        self._active_backend = ""
        self._logger = _get_driver_logger()
        self._logger.info("Driver adapter initialized; log file: %s", LOG_FILE)

    def _resource_manager(self, backend: str):
        if backend not in self._resource_managers:
            if pyvisa is None:
                raise RuntimeError("PyVISA is required to discover or connect to the RIGOL instrument")
            self._resource_managers[backend] = (
                pyvisa.ResourceManager("@py")
                if backend == "py"
                else pyvisa.ResourceManager()
            )
        return self._resource_managers[backend]

    @staticmethod
    def _backend_order(resource_name: str = "") -> tuple[str, str]:
        return ("py", "native") if resource_name.upper().startswith("ASRL") else ("native", "py")

    def _open_and_identify(self, resource_name: str, timeout: int = 2000):
        errors = []
        for backend in self._backend_order(resource_name):
            instrument = None
            try:
                instrument = self._resource_manager(backend).open_resource(resource_name)
                instrument.timeout = timeout
                instrument.write_termination = "\n"
                instrument.read_termination = "\n"
                instrument.write("*CLS")
                identity = instrument.query("*IDN?").strip()
                if "RIGOL" not in identity.upper() or "DL30" not in identity.upper():
                    raise RuntimeError(f"Unexpected instrument identity: {identity}")
                return instrument, identity, backend
            except Exception as error:
                errors.append(f"{backend}: {error}")
                self._logger.debug(
                    "Could not identify RIGOL instrument using %s backend at %s",
                    backend,
                    resource_name,
                    exc_info=True,
                )
                if instrument is not None:
                    try:
                        instrument.close()
                    except Exception:
                        pass
        raise RuntimeError(
            f"Cannot identify RIGOL DL3021 at {resource_name}: " + "; ".join(errors)
        )

    def _list_resources(self) -> list[str]:
        resources = []
        for backend in ("native", "py"):
            try:
                resources.extend(self._resource_manager(backend).list_resources())
            except Exception:
                self._logger.debug(
                    "Could not enumerate VISA resources with %s backend",
                    backend,
                    exc_info=True,
                )
                pass
        return list(dict.fromkeys(resources))

    def find_instruments(self, options: Mapping[str, Any] | None = None) -> Sequence[DeviceInfo]:
        del options
        self._logger.info("Starting RIGOL DL3021 discovery")
        matches = []
        for resource_name in self._list_resources():
            candidate = None
            try:
                candidate, identity, _ = self._open_and_identify(resource_name, timeout=750)
                matches.append(_BridgeDeviceInfo(resource_name=resource_name, model=identity))
            except Exception:
                self._logger.debug(
                    "Ignoring non-matching/unreachable resource %s",
                    resource_name,
                    exc_info=True,
                )
                continue
            finally:
                if candidate is not None:
                    try:
                        candidate.close()
                    except Exception:
                        pass
        self._logger.info(
            "RIGOL DL3021 discovery complete; found %d matching instrument(s)",
            len(matches),
        )
        return tuple(matches)

    def connect(
        self,
        resource_name: str,
        options: Mapping[str, Any] | None = None,
    ) -> DeviceInfo:
        resource_name = str(resource_name or "").strip()
        if resource_name and "::" not in resource_name and not resource_name.upper().startswith("COM"):
            resource_name = f"TCPIP0::{resource_name}::INSTR"
        if not resource_name:
            matches = self.find_instruments(options)
            if len(matches) != 1:
                raise RuntimeError(f"Expected one RIGOL DL3021; found {len(matches)}")
            resource_name = matches[0].resource_name
        try:
            self._instrument, identity, self._active_backend = self._open_and_identify(resource_name)
            self._resource_name = resource_name
            self._logger.info(
                "Connected to %s at %s using %s backend",
                identity,
                resource_name,
                self._active_backend,
            )
            return _BridgeDeviceInfo(resource_name=resource_name, model=identity)
        except Exception:
            self._logger.exception("Failed to connect to RIGOL DL3021 at %s", resource_name)
            raise

    def prepare_safe_state(self) -> None:
        self.configure_safe_defaults()

    def configure_safe_defaults(self) -> None:
        self._require_instrument()
        self._instrument.write(":SOUR:INP:STAT OFF")
        if self._input_state() != INPUT_OFF:
            raise RuntimeError("RIGOL load input OFF was not confirmed")
        self._instrument.write(":SOUR:FUNC:MODE FIX")
        self._instrument.write(":SOUR:FUNC CURR")
        self._logger.info("Safe defaults established; load input confirmed OFF in CC mode")

    def validate_configuration(self, configuration: Mapping[str, Any]) -> None:
        """Validate a mode/setpoint against the mode lookup and live limits."""
        requested_mode = configuration.get("mode")
        try:
            mode = RigolOperatingMode(requested_mode)
            mode_name = self.MODE_COMMAND[mode][2]
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"Invalid RIGOL operating mode: {requested_mode!r}"
            ) from error

        mode_settings = self.MODE_SETTING_LOOKUP.get(mode_name)
        if mode_settings is None:
            raise ValueError(f"Invalid RIGOL operating mode: {requested_mode!r}")
        if not mode_settings["implemented"]:
            raise ValueError(
                f"{mode_name} mode is documented but not implemented in this adapter"
            )

        self._require_instrument()
        setpoint_key = mode_settings["configuration_key"]
        # TODO: Apply mode-specific defaults when optional keywords are absent
        # from the user's parameters; required setpoints must still be provided.
        try:
            setpoint = float(configuration[setpoint_key])
        except KeyError as error:
            raise ValueError(
                f"Missing {mode_settings['setpoint']} parameter "
                f"({setpoint_key!r}) for {mode_name} mode"
            ) from error
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"{mode_settings['setpoint']} must be numeric in {mode_name} mode"
            ) from error

        if not math.isfinite(setpoint):
            raise ValueError(f"{mode_settings['setpoint']} must be finite")

        documented_ranges = mode_settings["ranges"]
        if documented_ranges and not any(
            low <= setpoint <= high
            for low, high in documented_ranges.values()
        ):
            range_text = " or ".join(
                f"{low:g}..{high:g} {mode_settings['unit']}"
                for low, high in documented_ranges.values()
            )
            raise ValueError(
                f"{mode_settings['setpoint']} must be in {range_text}"
            )

        level_command = mode_settings["set_command"]
        minimum = float(self._instrument.query(f"{level_command}? MIN"))
        maximum = float(self._instrument.query(f"{level_command}? MAX"))
        if not minimum <= setpoint <= maximum:
            raise ValueError(
                f"{mode_settings['setpoint']} must be "
                f"{minimum:g}..{maximum:g} {mode_settings['unit']}"
            )

    def validate_desired_state(self, desired: Mapping[str, Any]) -> None:
        """Compatibility entry point for the current CANoe bridge."""
        self.validate_configuration(desired)

    def apply_configuration(self, configuration: Mapping[str, Any]) -> None:
        self._require_instrument()
        self.validate_configuration(configuration)
        if self._input_state() != INPUT_OFF:
            raise RuntimeError("Disable ELOAD input before changing the setpoint")

        mode = RigolOperatingMode(configuration.get("mode"))

        match mode:
            case RigolOperatingMode.CURR:
                setpoint = float(configuration["setpoint"])
                func, unit, label = self.MODE_COMMAND.get(RigolOperatingMode.CURR)
                self._instrument.write(":SOUR:FUNC:MODE FIX")
                self._instrument.write(f":SOUR:FUNC {func}")
                self._instrument.write(f":SOUR:{func}:LEV:IMM {setpoint:.12g}")

                actual = float(self._instrument.query(":SOUR:CURR:LEV:IMM?"))
                if not math.isfinite(actual) or abs(actual - setpoint) > max(1e-6, abs(setpoint) * 1e-4):
                    self._logger.error(
                        "CC setpoint readback mismatch: requested %.12g A, got %.12g A",
                        setpoint,
                        actual,
                    )
                    raise RuntimeError(
                        f"CC setpoint readback mismatch: requested {setpoint:g}, got {actual:g}"
                    )
                self._logger.info("Configuration applied and verified: mode=CC, setpoint=%.12g %s", actual, unit)

            case _:
                raise ValueError(f"{configuration.get('mode')!r} mode is not implemented in this adapter")
            
    def set_output(
        self,
        channel: str | bool,
        value: float | bool | None = None,
    ) -> bool | None:
        self._require_instrument()
        if value is None:
            enabled = bool(channel)  # Existing bridge calls set_output(enabled).
        else:
            if channel != self.INPUT_CHANNEL:
                raise ValueError(f"Unsupported RIGOL load input channel: {channel!r}")
            enabled = bool(value)

        # Support only CC mode at the moment
        # TODO: Look into other modes and implement
        if enabled:
            response = self._instrument.query(":SOUR:FUNC?").strip().upper()
            mode = self.MODE_BY_RESPONSE.get(response)
            if mode is None:
                raise RuntimeError(f"{response} mode is not implemeneted in this adapter")

            func, unit, label = self.MODE_COMMAND.get(mode)
            match mode:
                case RigolOperatingMode.CURR:
                    setpoint = float(self._instrument.query(f":SOUR:{func}:LEV:IMM?"))
                    self.validate_configuration({"mode": mode, "setpoint": setpoint})
                    voltage = float(self._instrument.query(":MEAS:VOLT?"))
                    self._validate_input_power(voltage, setpoint)
                case _:
                    raise RuntimeError(f"{mode} mode is not implemeneted in this adapter")
                        
        self._instrument.write(f":SOUR:INP:STAT {'ON' if enabled else 'OFF'}")

        # Noodle logic to handle unexpected ELoad response
        actual = self._input_state()
        if enabled and actual == INPUT_ON:
            voltage = float(self._instrument.query(":MEAS:VOLT?"))
            current = float(self._instrument.query(":MEAS:CURR?"))
            self._enforce_measured_power(voltage, current)

        confirmed = actual == INPUT_ON if actual in (INPUT_OFF, INPUT_ON) else None
        self._logger.info(
            "Load input command requested=%s, confirmed=%s",
            "ON" if enabled else "OFF",
            "ON" if confirmed is True else "OFF" if confirmed is False else "UNKNOWN",
        )
        return confirmed

    def read_measurements(self) -> Mapping[str, int | float | bool | str]:
        self._require_instrument()
        setpoint = float(self._instrument.query(":SOUR:CURR:LEV:IMM?"))
        voltage = float(self._instrument.query(":MEAS:VOLT?"))
        current = float(self._instrument.query(":MEAS:CURR?"))
        input_state = self._input_state()
        if input_state == INPUT_ON:
            self._enforce_measured_power(voltage, current)
        # These keys match the current bridge's system-variable update schema.
        measurements = {
            "ActualMode": int(RigolOperatingMode.CURR),
            "ActualSetpoint": setpoint,
            "SetpointUnit": "A",
            "MeasuredVoltage": voltage,
            "MeasuredCurrent": current,
            "MeasuredResistance": voltage / current if abs(current) > 1e-12 else 0.0,
            "MeasuredPower": voltage * current,
            "InputState": input_state if input_state in (INPUT_OFF, INPUT_ON) else 2,
        }
        self._logger.debug("Measurement snapshot: %s", measurements)
        return measurements

    def safe_disable_outputs(self) -> bool:
        if self._instrument is None:
            self._logger.warning("Safe-off requested while no instrument session is open")
            return False
        self._instrument.write(":SOUR:INP:STAT OFF")
        confirmed_off = self._input_state() == INPUT_OFF
        if confirmed_off:
            self._logger.info("Safe-off completed; load input confirmed OFF")
        else:
            self._logger.error("Safe-off failed; load input remains ON")
        return confirmed_off

    def safe_disable_output(self) -> bool:
        """Compatibility entry point for the current CANoe bridge."""
        return self.safe_disable_outputs()

    def enter_safe_state(self) -> bool:
        return self.safe_disable_outputs()

    def close(self) -> None:
        instrument, self._instrument = self._instrument, None
        resource_name = self._resource_name
        self._resource_name = ""
        self._active_backend = ""
        if instrument is not None:
            try:
                instrument.close()
                self._logger.info("Closed VISA session for %s", resource_name)
            except Exception:
                self._logger.exception("Error closing VISA session for %s", resource_name)
                raise

    def _input_state(self) -> int:
        self._require_instrument()
        state = int(float(self._instrument.query(":SOUR:INP:STAT?")))
        if state not in (INPUT_OFF, INPUT_ON):
            raise RuntimeError(f"Unexpected ELOAD input state: {state}")
        return state

    def _validate_input_power(self, voltage: float, current_setpoint: float) -> None:
        if not math.isfinite(voltage):
            raise RuntimeError("Input ON blocked: invalid voltage readback")
        if voltage < 0:
            raise RuntimeError(f"Input ON blocked: reverse voltage detected ({voltage:g} V)")
        requested_power = voltage * current_setpoint
        if requested_power > MAX_POWER:
            raise ValueError(
                f"Input ON blocked: requested {requested_power:g} W exceeds {MAX_POWER:g} W"
            )
        if voltage > MAX_VOLTAGE:
            raise ValueError(f"Input ON blocked: voltage exceeds {MAX_VOLTAGE:g} V")

    def _enforce_measured_power(self, voltage: float, current: float) -> None:
        '''
        Ensure at that invalid measurement is met with immediate shutdown
        '''
        if not math.isfinite(voltage) or not math.isfinite(current):
            self._emergency_input_off()
            raise RuntimeError("ELOAD input disabled: invalid voltage/current readback")
        power = abs(voltage * current)
        if power > MAX_POWER or voltage > MAX_VOLTAGE or abs(current) > MAX_CURRENT:
            self._emergency_input_off()
            raise RuntimeError(
                "ELOAD input disabled: measured limit exceeded "
                f"(V={voltage:g}, I={current:g}, P={power:g})"
            )

    def _emergency_input_off(self) -> None:
        self._logger.error("Attempting emergency load-input OFF after unsafe readback")
        self._instrument.write(":SOUR:INP:STAT OFF")
        if self._input_state() != INPUT_OFF:
            raise RuntimeError("Emergency ELOAD input-off was not confirmed")

    def _require_instrument(self) -> None:
        if self._instrument is None:
            raise RuntimeError("RIGOL DL3021 is not connected")


if __name__ == "__main__":
    class _SimulatedRigol:
        """Small SCPI stand-in so this example never opens physical hardware."""

        def __init__(self):
            self.timeout = None
            self.write_termination = None
            self.read_termination = None
            self.input_on = False
            self.mode = "CURR"
            self.setpoint = 2.0

        def query(self, command: str) -> str:
            responses = {
                "*IDN?": "RIGOL,DL3021,SIMULATED,1.0",
                ":SOUR:INP:STAT?": "1" if self.input_on else "0",
                ":SOUR:FUNC?": self.mode,
                ":SOUR:CURR:LEV:IMM? MIN": "0.0",
                ":SOUR:CURR:LEV:IMM? MAX": "10.0",
                ":SOUR:CURR:LEV:IMM?": str(self.setpoint),
                ":MEAS:VOLT?": "5.0",
                ":MEAS:CURR?": str(self.setpoint if self.input_on else 0.0),
            }
            try:
                return responses[command]
            except KeyError as error:
                raise ValueError(f"Unsupported simulated query: {command}") from error

        def write(self, command: str) -> None:
            parts = command.split()
            if command == "*CLS":
                return
            if command.startswith(":SOUR:INP:STAT "):
                self.input_on = parts[-1].upper() == "ON"
            elif command.startswith(":SOUR:FUNC "):
                self.mode = parts[-1].upper()
            elif command.startswith(":SOUR:FUNC:MODE "):
                return
            elif command.startswith(":SOUR:CURR:LEV:IMM "):
                self.setpoint = float(parts[-1])
            else:
                raise ValueError(f"Unsupported simulated command: {command}")

        def close(self) -> None:
            pass

    class _SimulatedRigolResourceManager:
        def __init__(self):
            self.instrument = _SimulatedRigol()

        def list_resources(self):
            return ("USB0::RIGOL::DL3021::SIMULATED::INSTR",)

        def open_resource(self, resource_name: str):
            del resource_name
            return self.instrument

    adapter = RigolDL3021TemplateDriverAdapter()
    simulated_rm = _SimulatedRigolResourceManager()
    adapter._resource_managers = {"py": simulated_rm, "native": simulated_rm}
    try:
        discovered = adapter.find_instruments()
        if len(discovered) != 1:
            raise RuntimeError(f"Expected one simulated RIGOL device, found {len(discovered)}")
        connected = adapter.connect(discovered[0].resource_name)
        print(f"Connected to simulated {connected}")
        adapter.configure_safe_defaults()
        adapter.apply_configuration({"mode": RigolOperatingMode.CURR, "setpoint": 2.0})
        adapter.set_output(True)
        print(f"Measurements: {dict(adapter.read_measurements())}")
    finally:
        adapter.safe_disable_output()
        adapter.close()
