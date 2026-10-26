"""Self-contained OWON SPE15105 driver using the reusable adapter contract.

This is the primary OWON driver implementation for the template-based CANoe
binding. It owns VISA discovery, session configuration, SCPI, limits, safe-off,
and measurements. It deliberately does not import OwonSPE15105_DriverAdapter.py
or any CANoe module.
"""

from __future__ import annotations

import math
import logging
import re
import tempfile
import time
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any, Iterator, Mapping, Sequence


import pyvisa


from Instrument_Driver_Adapter_Template import (
    ConfigurableInstrument,
    DeviceInfo,
    DiscoverableInstrument,
    InstrumentDriverAdapter,
    MeasurementPollingInstrument,
    OutputControlInstrument,
    SafeStateInstrument,
)


TARGET_MODEL = "SPE15105"
MAX_VOLTAGE = 60.0
MAX_CURRENT = 10.0
COMMAND_SETTLE_SECONDS = 0.1
LOG_DIRECTORY = Path(tempfile.gettempdir()) / Path(__file__).stem
LOG_FILE = LOG_DIRECTORY / "driver.log"




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


def _parse_output_state(response: str) -> bool:
    value = response.strip().upper()
    if value in ("1", "ON"):
        return True
    if value in ("0", "OFF"):
        return False
    raise RuntimeError(f"Unexpected OUTP? response: {response!r}")


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


class OwonSPE15105TemplateDriverAdapter(
    InstrumentDriverAdapter,
    DiscoverableInstrument,
    ConfigurableInstrument,
    OutputControlInstrument,
    MeasurementPollingInstrument,
    SafeStateInstrument,
):
    """OWON SPE15105 VISA/SCPI implementation and driver-level safety logic."""

    OUTPUT_CHANNEL = "output"
    OPERATING_MODE = ["STANDBY", "CV", "CC", "FAILURE"]

    def __init__(self):
        self._resource_managers = {}
        self._instrument = None
        self._resource_name = ""
        self._baud_rate = 9600
        self._logger = _get_driver_logger()
        self._logger.info("Driver adapter initialized; log file: %s", LOG_FILE)

    def _resource_manager(self, backend: str):
        if backend not in self._resource_managers:
            if pyvisa is None:
                raise RuntimeError("PyVISA is required to discover or connect to the OWON instrument")
            self._resource_managers[backend] = (
                pyvisa.ResourceManager("@py")
                if backend == "py"
                else pyvisa.ResourceManager()
            )
        return self._resource_managers[backend]

    @staticmethod
    def _normalize_resource_name(resource_name: str | None) -> str:
        name = str(resource_name or "").strip()
        if not name or "::" in name:
            return name
        match = re.fullmatch(r"COM(\d+)", name, re.IGNORECASE)
        if match:
            return f"ASRL{match.group(1)}::INSTR"
        return f"TCPIP0::{name}::INSTR"

    @staticmethod
    def _backend_order(resource_name: str) -> tuple[str, str]:
        if resource_name.upper().startswith("ASRL"):
            return ("py", "native")
        return ("native", "py")

    @staticmethod
    def _configure_session(instrument, resource_name: str, baud_rate: int) -> None:
        instrument.timeout = 2000
        instrument.write_termination = "\n"
        instrument.read_termination = "\n"
        if resource_name.upper().startswith("ASRL"):
            instrument.baud_rate = baud_rate
            instrument.data_bits = 8
            instrument.parity = pyvisa.constants.Parity.none
            instrument.stop_bits = pyvisa.constants.StopBits.one

    def _open_and_identify(self, resource_name: str, baud_rate: int):
        errors = []
        for backend in self._backend_order(resource_name):
            instrument = None
            try:
                instrument = self._resource_manager(backend).open_resource(resource_name)
                self._configure_session(instrument, resource_name, baud_rate)
                if resource_name.upper().startswith("ASRL"):
                    time.sleep(0.25)
                identity = instrument.query("*IDN?").strip()
                normalized = identity.upper().replace(" ", "")
                if "OWON" not in identity.upper() or TARGET_MODEL not in normalized:
                    raise RuntimeError(f"Unexpected instrument identity: {identity}")
                return instrument, identity
            except Exception as error:
                errors.append(f"{backend}: {error}")
                if instrument is not None:
                    try:
                        instrument.close()
                    except Exception:
                        pass
        raise RuntimeError("Unable to identify OWON SPE15105: " + "; ".join(errors))

    def find_instruments(self, options: Mapping[str, Any] | None = None) -> Sequence[DeviceInfo]:
        options = options or {}
        baud_rate = int(options.get("baud_rate", self._baud_rate))
        self._logger.info("Starting OWON SPE15105 discovery")
        resources = []
        for backend in ("py", "native"):
            try:
                resources.extend(self._resource_manager(backend).list_resources())
            except Exception:
                self._logger.debug("Could not enumerate VISA resources with %s backend", backend, exc_info=True)
                pass

        matches = []
        for resource_name in dict.fromkeys(resources):
            candidate = None
            try:
                candidate, identity = self._open_and_identify(resource_name, baud_rate)
                matches.append(_BridgeDeviceInfo(resource_name=resource_name, model=identity))
            except Exception:
                self._logger.debug("Ignoring non-matching/unreachable resource %s", resource_name, exc_info=True)
                continue
            finally:
                if candidate is not None:
                    try:
                        candidate.close()
                    except Exception:
                        pass
        self._logger.info(
            "OWON SPE15105 discovery complete; found %d matching instrument(s)",
            len(matches),
        )
        return tuple(matches)

    def connect(
        self,
        resource_name: str,
        options: Mapping[str, Any] | None = None,
    ) -> DeviceInfo:
        options = options or {}
        self._baud_rate = int(options.get("baud_rate", self._baud_rate))
        if self._baud_rate <= 0:
            raise ValueError("Baud rate must be greater than zero")
        resource_name = self._normalize_resource_name(resource_name)
        if not resource_name:
            matches = self.find_instruments(options)
            if len(matches) != 1:
                raise RuntimeError(f"Expected one OWON device; found {len(matches)}")
            resource_name = matches[0].resource_name

        try:
            self._instrument, identity = self._open_and_identify(resource_name, self._baud_rate)
            self._resource_name = resource_name
            self._logger.info("Connected to %s at %s", identity, resource_name)
            return _BridgeDeviceInfo(resource_name=resource_name, model=identity)
        except Exception:
            self._logger.exception("Failed to connect to OWON SPE15105 at %s", resource_name)
            raise

    def prepare_safe_state(self) -> None:
        self.configure_safe_defaults()

    def configure_safe_defaults(self) -> None:
        self._require_instrument()
        self._write("OUTP OFF")
        if self._output_enabled():
            self._logger.error("OWON output OFF was not confirmed")
            raise RuntimeError("OWON output OFF was not confirmed")
        self._logger.info("Safe defaults established; output confirmed OFF")

    def validate_configuration(self, configuration: Mapping[str, Any]) -> None:
        voltage = float(configuration["voltage"])
        current = float(configuration["current"])
        if not math.isfinite(voltage) or not 0.0 <= voltage <= MAX_VOLTAGE:
            raise ValueError(f"Voltage must be 0..{MAX_VOLTAGE:g} V")
        if not math.isfinite(current) or not 0.0 <= current <= MAX_CURRENT:
            raise ValueError(f"Current must be 0..{MAX_CURRENT:g} A")

    def validate_desired_state(self, desired: Mapping[str, Any]) -> None:
        """Compatibility entry point for the current CANoe bridge."""
        self.validate_configuration(desired)

    def apply_configuration(self, configuration: Mapping[str, Any]) -> None:
        self._require_instrument()
        self.validate_configuration(configuration)
        # if self._output_enabled():
        #     raise RuntimeError("Disable the PSU output before changing setpoints")
        # 22-10-2026 quan.dao: Use OCP setting instead of current setting
        # This ensures overcurrent will shutdown DUT instead of throttling the supply voltage
        
        # Current setting must be higher than OCP limit
        self._write(f"CURR {(float(configuration['current'])+0.1):.12g}")
        self._write(f"CURR:LIM {float(configuration['current']):.12g}")
        self._write(f"VOLT {float(configuration['voltage']):.12g}")
        
        actual_current = float(self._instrument.query("CURR:LIM?"))
        actual_voltage = float(self._instrument.query("VOLT?"))
        self.validate_configuration({"voltage": actual_voltage, "current": actual_current})
        self._logger.info(
            "Configuration applied and verified: voltage=%.12g V, current=%.12g A",
            actual_voltage,
            actual_current,
        )

    def set_output(
        self,
        channel: str | bool,
        value: float | bool | None = None,
    ) -> bool | None:
        self._require_instrument()
        if value is None:
            enabled = bool(channel)  # Existing bridge calls set_output(enabled).
        else:
            if channel != self.OUTPUT_CHANNEL:
                raise ValueError(f"Unsupported OWON output channel: {channel!r}")
            enabled = bool(value)
        if enabled:
            self.validate_configuration(
                {
                    "voltage": float(self._instrument.query("VOLT?")),
                    "current": float(self._instrument.query("CURR:LIM?")),
                }
            )
        self._write(f"OUTP {'ON' if enabled else 'OFF'}")
        confirmed = self._output_enabled()
        self._logger.info(
            "Output command requested=%s, confirmed=%s",
            "ON" if enabled else "OFF",
            "ON" if confirmed else "OFF",
        )
        return confirmed

    def read_measurements(self) -> Mapping[str, int | float | bool | str]:
        self._require_instrument()
        all_info = self._instrument.query("MEAS:ALL:INFO?").split(",")
        voltage = float(all_info[0])
        current = float(all_info[1])
        try:
            power = float(all_info[2])
        except Exception:
            power = voltage * current

        # These keys match the current bridge's system-variable update schema.
        measurements = {
            "ActualVoltageSetpoint": float(self._instrument.query("VOLT?")),
            "ActualCurrentSetpoint": float(self._instrument.query("CURR:LIM?")),
            "MeasuredVoltage": voltage,
            "MeasuredCurrent": current,
            "MeasuredPower": power,
            "OutputState": 1 if self._output_enabled() else 0,
            "OutputMode": int(all_info[6]),
        }
        self._logger.debug("Measurement snapshot: %s", measurements)
        return measurements

    def safe_disable_outputs(self) -> bool:
        if self._instrument is None:
            self._logger.warning("Safe-off requested while no instrument session is open")
            return False
        self._write("OUTP OFF")
        confirmed_off = not self._output_enabled()
        if confirmed_off:
            self._logger.info("Safe-off completed; output confirmed OFF")
        else:
            self._logger.error("Safe-off failed; output remains ON")
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
        if instrument is not None:
            try:
                instrument.close()
                self._logger.info("Closed VISA session for %s", resource_name)
            except Exception:
                self._logger.exception("Error closing VISA session for %s", resource_name)
                raise

    def _output_enabled(self) -> bool:
        self._require_instrument()
        return _parse_output_state(self._instrument.query("OUTP?"))

    def _write(self, command: str) -> None:
        self._require_instrument()
        self._instrument.write(command)
        time.sleep(COMMAND_SETTLE_SECONDS)

    def _require_instrument(self) -> None:
        if self._instrument is None:
            raise RuntimeError("OWON SPE15105 is not connected")


if __name__ == "__main__":
    class _SimulatedOwon:
        """Small SCPI stand-in so this example never opens physical hardware."""

        def __init__(self):
            self.timeout = None
            self.write_termination = None
            self.read_termination = None
            self.voltage = 5.0
            self.current = 0.2
            self.output_on = False

        def query(self, command: str) -> str:
            responses = {
                "*IDN?": "OWON,SPE15105,SIMULATED,1.0",
                "OUTP?": "1" if self.output_on else "0",
                "VOLT?": str(self.voltage),
                "CURR:LIM?": str(self.current),
                "MEAS:VOLT?": "5.0" if self.output_on else "0.0",
                "MEAS:CURR?": "0.1" if self.output_on else "0.0",
                "MEAS:POW?": "0.5" if self.output_on else "0.0",
            }
            try:
                return responses[command]
            except KeyError as error:
                raise ValueError(f"Unsupported simulated query: {command}") from error

        def write(self, command: str) -> None:
            parts = command.split()
            operation = parts[0].upper()
            if operation == "OUTP":
                self.output_on = parts[1].upper() == "ON"
            elif operation == "VOLT":
                self.voltage = float(parts[1])
            elif operation == "CURR:LIM":
                self.current = float(parts[1])
            else:
                raise ValueError(f"Unsupported simulated command: {command}")

        def close(self) -> None:
            pass

    class _SimulatedOwonResourceManager:
        def __init__(self):
            self.instrument = _SimulatedOwon()

        def list_resources(self):
            return ("USB0::OWON::SPE15105::SIMULATED::INSTR",)

        def open_resource(self, resource_name: str):
            del resource_name
            return self.instrument

    adapter = OwonSPE15105TemplateDriverAdapter()
    simulated_rm = _SimulatedOwonResourceManager()
    adapter._resource_managers = {"py": simulated_rm, "native": simulated_rm}
    try:
        discovered = adapter.find_instruments()
        if len(discovered) != 1:
            raise RuntimeError(f"Expected one simulated OWON device, found {len(discovered)}")
        connected = adapter.connect(discovered[0].resource_name)
        print(f"Connected to simulated {connected}")
        adapter.configure_safe_defaults()
        adapter.apply_configuration({"voltage": 5.0, "current": 0.2})
        adapter.set_output(True)
        print(f"Measurements: {dict(adapter.read_measurements())}")
    finally:
        adapter.safe_disable_output()
        adapter.close()
