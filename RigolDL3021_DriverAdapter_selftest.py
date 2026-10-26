"""Live-connection self-test for the RIGOL DL3021 driver.

Run from the workspace root, for example::

    py playground/CANoeAdaptation/RigolDL3021_DriverAdapter_selftest.py
    py playground/CANoeAdaptation/RigolDL3021_DriverAdapter_selftest.py --resource USB0::0x1AB1::0x0E11::DL3A280900256::INSTR

The test verifies instrument identity and reads the CC setpoint, measurements,
and load-input state. It does not change the setpoint or enable/disable the
load input. The driver's normal connection/discovery path sends ``*CLS`` to
clear the instrument status/event registers.
"""

from __future__ import annotations

import argparse
import math
import sys

from RigolDL3021_DriverAdapter import (
    MAX_CURRENT,
    RigolDL3021TemplateDriverAdapter,
)


MEASUREMENT_FIELDS = (
    "ActualSetpoint",
    "MeasuredVoltage",
    "MeasuredCurrent",
    "MeasuredResistance",
    "MeasuredPower",
)


def run_self_test(resource_name: str | None) -> int:
    """Connect to the physical instrument and return a process exit code."""
    adapter = RigolDL3021TemplateDriverAdapter()
    try:
        if resource_name:
            selected_resource = resource_name
        else:
            matches = adapter.find_instruments()
            if not matches:
                raise RuntimeError(
                    "No RIGOL DL3021 was discovered. Check VISA setup or specify "
                    "its VISA resource with --resource."
                )
            if len(matches) > 1:
                resources = ", ".join(match.resource_name for match in matches)
                raise RuntimeError(
                    f"Found multiple RIGOL DL3021 devices ({resources}); "
                    "select one with --resource."
                )
            selected_resource = matches[0].resource_name

        identity = adapter.connect(selected_resource)
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

        setpoint = float(measurements["ActualSetpoint"])
        if not 0.0 <= setpoint <= MAX_CURRENT:
            raise RuntimeError(
                f"CC setpoint readback {setpoint:g} A is outside 0..{MAX_CURRENT:g} A"
            )
        if measurements.get("SetpointUnit") != "A":
            raise RuntimeError(
                f"Unexpected CC setpoint unit: {measurements.get('SetpointUnit')!r}"
            )

        input_state = measurements.get("InputState")
        if input_state not in (0, 1, False, True):
            raise RuntimeError(f"Unexpected load-input state readback: {input_state!r}")

        print(f"PASS: finite readbacks: {dict(measurements)}")
        if bool(input_state):
            print("WARNING: load input is currently ON; the self-test left it unchanged.")
        else:
            print("PASS: load input is OFF.")
        print(
            "PASS: live-connection self-test completed; setpoint and load-input "
            "state were not changed."
        )
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
        help="VISA resource (for example USB0::...::INSTR); auto-discover if omitted",
    )
    args = parser.parse_args()
    return run_self_test(args.resource)


if __name__ == "__main__":
    raise SystemExit(main())
