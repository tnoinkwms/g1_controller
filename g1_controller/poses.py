from __future__ import annotations

from typing import Dict

import numpy as np


def default_home_pose_motor_rad() -> Dict[int, float]:
    """
    Unitree example's target_pos mapped to joint indices:
      15 LShoulderPitch : 0
      16 LShoulderRoll  : +pi/2
      17 LShoulderYaw   : 0
      18 LElbow         : +pi/2
      19 LWristRoll     : 0
      22 RShoulderPitch : 0
      23 RShoulderRoll  : -pi/2
      24 RShoulderYaw   : 0
      25 RElbow         : +pi/2
      26 RWristRoll     : 0
      12 WaistYaw       : 0
      13 WaistRoll      : 0
      14 WaistPitch     : 0

    Other joints (20,21,27,28 etc.) are left to 0 by default here.
    """
    p2 = float(np.pi / 2.0)
    return {
        15: 0.0,
        16: 0.0,
        17: 0.0,
        18: +p2,
        19: 0.0,

        22: 0.0,
        23: 0.0,
        24: 0.0,
        25: +p2,
        26: 0.0,

        12: 0.0,
        13: 0.0,
        14: 0.0,

        # wrists pitch/yaw (if present on your model)
        20: 0.0,
        21: 0.0,
        27: 0.0,
        28: 0.0,
    }
