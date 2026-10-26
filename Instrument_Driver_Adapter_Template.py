"""Base class and optional capabilities for instrument-driver adapters.

Keep vendor APIs, transport protocols, device limits, and instrument safety
logic in concrete driver adapters. Do not import CANoe from this module.

The CANoe bridge currently targets the PSU/e-load interaction it was designed
for. A future bridge revision can use these optional capabilities to select
appropriate actions for DAQ, scopes, meters, generators, and other devices.
Do not add placeholder methods to a driver for capabilities it does not have.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol, Sequence, runtime_checkable


@dataclass(frozen=True)
class DeviceInfo:
    """Normalized identity for a discovered or connected instrument."""

    resource_name: str
    manufacturer: str = ""
    model: str = ""
    serial_number: str = ""
    firmware_version: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @property
    def identity(self) -> str:
        """Human-readable identity suitable for a log or operator display."""
        parts = [part for part in (self.manufacturer, self.model) if part]
        if self.serial_number:
            parts.append(f"S/N {self.serial_number}")
        return " ".join(parts) or self.resource_name


class InstrumentDriverAdapter(ABC):
    """Minimal common lifecycle shared by all instrument drivers.

    Implementations own connection/session state and must keep blocking device
    I/O synchronous; the CANoe bridge is responsible for calling drivers from
    its worker thread. ``prepare_safe_state`` must leave outputs disabled or
    acquisition stopped, as appropriate for that device.
    """

    API_VERSION = 1

    @abstractmethod
    def connect(
        self,
        resource_name: str,
        options: Mapping[str, Any] | None = None,
    ) -> DeviceInfo:
        """Open a session, identify the device, and return its identity."""

    @abstractmethod
    def prepare_safe_state(self) -> None:
        """Put the device in a known, non-energizing/non-acquiring state.

        Raise if the safe state cannot be established or verified. This is
        called after connecting and before any requested test operation.
        """

    @abstractmethod
    def close(self) -> None:
        """Release sessions and driver resources, even after an I/O failure."""

    def health_check(self) -> bool:
        """Optionally override with a lightweight communication check."""
        return True


@runtime_checkable
class DiscoverableInstrument(Protocol):
    """Optional capability for drivers that can enumerate resources."""

    def find_instruments(
        self, options: Mapping[str, Any] | None = None
    ) -> Sequence[DeviceInfo]:
        """Discover matching devices; return an empty sequence if none found."""


@runtime_checkable
class ConfigurableInstrument(Protocol):
    """Optional capability for instruments with a validated configuration."""

    def validate_configuration(self, configuration: Mapping[str, Any]) -> None:
        """Raise ``ValueError`` when a requested setting is unsafe/unsupported."""

    def apply_configuration(self, configuration: Mapping[str, Any]) -> None:
        """Apply settings and verify readback where the instrument permits."""


@runtime_checkable
class OutputControlInstrument(Protocol):
    """Optional capability for one or more controllable physical outputs."""

    def set_output(self, channel: str, value: float | bool) -> float | bool | None:
        """Set a named output; return its confirmed state/value when available."""

    def safe_disable_outputs(self) -> bool:
        """Disable outputs and return True only when the safe state is confirmed."""


@runtime_checkable
class MeasurementPollingInstrument(Protocol):
    """Optional capability for scalar/latest-value measurements."""

    def read_measurements(self) -> Mapping[str, int | float | bool | str]:
        """Return instrument-level names and primitive values, not CANoe names."""


@runtime_checkable
class BufferedAcquisitionInstrument(Protocol):
    """Optional capability for DAQ/scope-style finite or streaming acquisition."""

    def configure_acquisition(self, configuration: Mapping[str, Any]) -> None:
        """Configure channels, sample rate, sample count, and acquisition mode."""

    def start_acquisition(self) -> None:
        """Arm/start acquisition; return promptly if the device supports it."""

    def read_samples(self, max_samples: int, timeout: float) -> "SampleBlock | None":
        """Read one available block, or return None when the timeout expires."""

    def stop_acquisition(self) -> None:
        """Stop or abort acquisition and leave the device safe for reconfiguration."""


@dataclass(frozen=True)
class SampleBlock:
    """A time-aligned batch returned by a buffered acquisition instrument.

    ``samples`` is channel-major: one sequence of samples per channel in
    ``channel_names``. Timestamps may be omitted when hardware timing is not
    available; sample rate and channel order must still be explicit.
    """

    channel_names: tuple[str, ...]
    samples: tuple[tuple[float, ...], ...]
    sample_rate_hz: float
    start_time_seconds: float | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


@runtime_checkable
class TriggeredInstrument(Protocol):
    """Optional capability for instruments supporting trigger coordination."""

    def configure_trigger(self, configuration: Mapping[str, Any]) -> None:
        """Configure source, edge/level, delay, and related trigger settings."""

    def arm_trigger(self) -> None:
        """Arm the instrument and wait for its trigger according to policy."""


@runtime_checkable
class SafeStateInstrument(Protocol):
    """Optional capability for explicit runtime safe-state confirmation."""

    def enter_safe_state(self) -> bool:
        """Enter/verify safe state; return False when confirmation is unavailable."""


def supports(adapter: InstrumentDriverAdapter, capability: type) -> bool:
    """Return whether an adapter structurally implements an optional capability."""
    return isinstance(adapter, capability)


__all__ = [
    "BufferedAcquisitionInstrument",
    "ConfigurableInstrument",
    "DeviceInfo",
    "DiscoverableInstrument",
    "InstrumentDriverAdapter",
    "MeasurementPollingInstrument",
    "OutputControlInstrument",
    "SafeStateInstrument",
    "SampleBlock",
    "TriggeredInstrument",
    "supports",
]


if __name__ == "__main__":
    class SimulatedNiDaqAdapter(
        InstrumentDriverAdapter,
        BufferedAcquisitionInstrument,
    ):
        """Runnable example only; this does not connect to physical NI hardware."""

        def __init__(self):
            self._connected = False
            self._configured = False
            self._acquiring = False

        def connect(
            self,
            resource_name: str,
            options: Mapping[str, Any] | None = None,
        ) -> DeviceInfo:
            self._connected = True
            return DeviceInfo(
                resource_name=resource_name,
                manufacturer="NI (simulated)",
                model="PXIe-6357",
            )

        def prepare_safe_state(self) -> None:
            self._acquiring = False
            print("Safe state prepared: acquisition stopped")

        def close(self) -> None:
            self._acquiring = False
            self._connected = False
            print("Driver session closed")

        def configure_acquisition(self, configuration: Mapping[str, Any]) -> None:
            if not self._connected:
                raise RuntimeError("Connect before configuring acquisition")
            self._channel_names = tuple(configuration["channels"])
            self._sample_rate_hz = float(configuration["sample_rate_hz"])
            self._samples_per_channel = int(configuration["samples_per_channel"])
            self._configured = True

        def start_acquisition(self) -> None:
            if not self._configured:
                raise RuntimeError("Configure acquisition before starting")
            self._acquiring = True

        def read_samples(self, max_samples: int, timeout: float) -> SampleBlock | None:
            del timeout  # The simulator has data immediately available.
            if not self._acquiring:
                return None
            count = min(max_samples, self._samples_per_channel)
            samples = tuple(
                tuple(0.1 * (channel_index + sample_index)
                      for sample_index in range(count))
                for channel_index, _ in enumerate(self._channel_names)
            )
            return SampleBlock(
                channel_names=self._channel_names,
                samples=samples,
                sample_rate_hz=self._sample_rate_hz,
            )

        def stop_acquisition(self) -> None:
            self._acquiring = False

    driver = SimulatedNiDaqAdapter()
    try:
        device = driver.connect("Dev1")
        print(f"Connected to: {device.identity}")
        driver.prepare_safe_state()

        if not supports(driver, BufferedAcquisitionInstrument):
            raise RuntimeError("This adapter does not support buffered acquisition")

        driver.configure_acquisition(
            {
                "channels": ("ai0", "ai1"),
                "sample_rate_hz": 1000.0,
                "samples_per_channel": 4,
            }
        )
        driver.start_acquisition()
        block = driver.read_samples(max_samples=4, timeout=1.0)
        print(f"Acquired block: {block}")
    finally:
        driver.stop_acquisition()
        driver.prepare_safe_state()
        driver.close()
