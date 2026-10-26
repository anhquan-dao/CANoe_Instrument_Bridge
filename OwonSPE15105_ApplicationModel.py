"""OWON SPE15105 model using the driver-adapter template-based wrapper.

This is a review copy. The original OwonSPE15105_ApplicationModel.py remains
unchanged and continues to use its original driver adapter directly.
"""

from application_layer import PowerSupply_Panel
from OwonSPE15105_DriverAdapter import (
    OwonSPE15105TemplateDriverAdapter,
)
from CANoe_Instrument_Bridge import create_application_model


OwonSPE15105ApplicationModel = create_application_model(
    model_name="OWON SPE15105",
    panel=PowerSupply_Panel,
    driver_factory=OwonSPE15105TemplateDriverAdapter,
    default_resource="ASRL15::INSTR",
    desired_state={
        "voltage": ("VoltageSetpoint", float),
        "current": ("CurrentSetpoint", float),
        "output_enabled": ("OutputEnabled", lambda value: bool(int(value))),
    },
    driver_options={"baud_rate": ("BaudRate", int)},
    setpoint_variables=("VoltageSetpoint", "CurrentSetpoint"),
    output_enable_variable="OutputEnabled",
    output_state_variable="OutputState",
)
