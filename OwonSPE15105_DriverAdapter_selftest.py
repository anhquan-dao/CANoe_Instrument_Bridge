"""Read-only live-connection self-test for the OWON SPE15105 driver.

Run from the workspace root, for example::

    py playground/CANoeAdaptation/OwonSPE15105_DriverAdapter_selftest.py
    py playground/CANoeAdaptation/OwonSPE15105_DriverAdapter_selftest.py --resource ASRL15::INSTR

The test opens a real VISA session, verifies the instrument identity, and reads
setpoints, measurements, and output state. It does not change setpoints or
enable/disable the output; an output that is already enabled is left unchanged.
"""

from __future__ import annotations

import argparse
import math
import sys

from OwonSPE15105_DriverAdapter import (
    MAX_CURRENT,
    MAX_VOLTAGE,
    OwonSPE15105TemplateDriverAdapter,
)


MEASUREMENT_FIELDS = (
    "ActualVoltageSetpoint",
    "ActualCurrentSetpoint",
    "MeasuredVoltage",
    "MeasuredCurrent",
    "MeasuredPower",
)


def run_self_test(resource_name: str | None, baud_rate: int) -> int:
    """Connect to the physical instrument and return a process exit code."""
    adapter = OwonSPE15105TemplateDriverAdapter()
    try:
        if resource_name:
            selected_resource = resource_name
        else:
            matches = adapter.find_instruments({"baud_rate": baud_rate})
            if not matches:
                raise RuntimeError(
                    "No OWON SPE15105 was discovered. Check VISA/serial setup or "
                    "specify its VISA resource with --resource."
                )
            if len(matches) > 1:
                resources = ", ".join(match.resource_name for match in matches)
                raise RuntimeError(
                    f"Found multiple OWON SPE15105 devices ({resources}); "
                    "select one with --resource."
                )
            selected_resource = matches[0].resource_name

        identity = adapter.connect(
            selected_resource,
            options={"baud_rate": baud_rate},
        )
        print(f"PASS: connected and identified {identity}")

        measurements = adapter.read_measurements()
        missing = [field for field in MEASUREMENT_FIELDS if field not in measurements]
        if missing:
            raise RuntimeError(f"Driver omitted measurement fields: {', '.join(missing)}")

        for field in MEASUREMENT_FIELDS:
            try:
                value = float(measurements[field])
            except (TypeError, ValueError) as error:
                raise RuntimeError(f"{field} is not numeric: {measurements[field]!r}") from error
            if not math.isfinite(value):
                raise RuntimeError(f"{field} is not finite: {value!r}")

        voltage_setpoint = float(measurements["ActualVoltageSetpoint"])
        current_setpoint = float(measurements["ActualCurrentSetpoint"])
        if not 0.0 <= voltage_setpoint <= MAX_VOLTAGE:
            raise RuntimeError(
                f"Voltage setpoint readback {voltage_setpoint:g} V is outside "
                f"0..{MAX_VOLTAGE:g} V"
            )
        if not 0.0 <= current_setpoint <= MAX_CURRENT:
            raise RuntimeError(
                f"Current setpoint readback {current_setpoint:g} A is outside "
                f"0..{MAX_CURRENT:g} A"
            )

        output_state = measurements.get("OutputState")
        if output_state not in (0, 1, False, True):
            raise RuntimeError(f"Unexpected output state readback: {output_state!r}")

        print(f"PASS: finite measurement and setpoint readbacks: {dict(measurements)}")
        if bool(output_state):
            print("WARNING: output is currently ON; the read-only self-test left it unchanged.")
        else:
            print("PASS: output is OFF.")
        print("PASS: read-only live-connection self-test completed; no settings were changed.")
        return 0
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    finally:
        try:
            adapter.close()
        except Exception as error:
            print(f"WARNING: error while closing the VISA session: {error}", file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--resource",
        help="VISA resource (for example ASRL15::INSTR); auto-discover if omitted",
    )
    parser.add_argument(
        "--baud-rate",
        type=int,
        default=9600,
        help="serial baud rate used for ASRL resources (default: 9600)",
    )
    args = parser.parse_args()
    if args.baud_rate <= 0:
        parser.error("--baud-rate must be greater than zero")
    return run_self_test(args.resource, args.baud_rate)


if __name__ == "__main__":
    raise SystemExit(main())
