from __future__ import annotations

import numpy as np

from .types import JointCalibration

# Your original behavior:
# - axes 18,25: deg = -deg + 70
# - axes 15,22,23,24,19,27,20,28: deg = -deg
CALIBRATIONS: dict[int, JointCalibration] = {
    18: JointCalibration(scale=-1.0, offset_deg=70.0),
    25: JointCalibration(scale=-1.0, offset_deg=70.0),

    15: JointCalibration(scale=-1.0, offset_deg=0.0),
    22: JointCalibration(scale=-1.0, offset_deg=0.0),
    23: JointCalibration(scale=-1.0, offset_deg=0.0),
    24: JointCalibration(scale=-1.0, offset_deg=0.0),
    19: JointCalibration(scale=-1.0, offset_deg=0.0),
    27: JointCalibration(scale=-1.0, offset_deg=0.0),
    20: JointCalibration(scale=-1.0, offset_deg=0.0),
    28: JointCalibration(scale=-1.0, offset_deg=0.0),
}


def get_calibration(axis: int) -> JointCalibration:
    return CALIBRATIONS.get(axis, JointCalibration())


def user_deg_to_motor_rad(axis: int, user_deg: float) -> float:
    cal = get_calibration(axis)
    motor_deg = cal.user_to_motor_deg(user_deg)
    return float(np.deg2rad(motor_deg))


def motor_rad_to_user_deg(axis: int, motor_rad: float) -> float:
    cal = get_calibration(axis)
    motor_deg = float(np.rad2deg(motor_rad))
    return float(cal.motor_to_user_deg(motor_deg))
