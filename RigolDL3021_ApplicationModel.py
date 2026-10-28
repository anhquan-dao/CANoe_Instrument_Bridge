"""RIGOL DL3021 model using the driver-adapter template-based wrapper.

This is a review copy. The original RigolDL3021_ApplicationModel.py remains
unchanged and continues to use its original driver adapter directly.
"""

from application_layer import ELOAD_Panel
from RigolDL3021_DriverAdapter import (
    RigolDL3021TemplateDriverAdapter,
)
from CANoe_Instrument_Bridge import create_application_model


RigolDL3021ApplicationModel = create_application_model(
    model_name="RIGOL DL3021",
    panel=ELOAD_Panel,
    driver_factory=RigolDL3021TemplateDriverAdapter,
    default_resource="USB0::0x1AB1::0x0E11::DL3A281000261::INSTR",
    desired_state={
        "mode": ("Mode", int),
        "setpoint": ("Setpoint", float),
        "input_enabled": ("InputEnabled", lambda value: bool(int(value))),
    },
    setpoint_variables=("Setpoint",),
    output_enable_variable="InputEnabled",
    output_state_variable="InputState",
)
