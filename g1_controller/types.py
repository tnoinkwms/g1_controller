from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class JointLimit:
    """Joint soft limits in radians."""
    min_rad: float
    max_rad: float


@dataclass(frozen=True)
class JointCalibration:
    """
    User-deg <-> motor-deg conversion.
    motor_deg = scale * user_deg + offset_deg
    """
    scale: float = 1.0
    offset_deg: float = 0.0

    def user_to_motor_deg(self, user_deg: float) -> float:
        return self.scale * user_deg + self.offset_deg

    def motor_to_user_deg(self, motor_deg: float) -> float:
        if self.scale == 0:
            raise ValueError("JointCalibration.scale must not be 0")
        return (motor_deg - self.offset_deg) / self.scale


@dataclass(frozen=True)
class Gains:
    kp: float = 60.0
    kd: float = 1.5


@dataclass
class ControllerConfig:
    network_interface: str = "eth0"
    dt: float = 0.02
    min_duration: float = 1.0  # safety: minimum interpolation duration

    # NOTE: dataclassでは default_factory が必要
    gains: Gains = field(default_factory=Gains)

    verbose: bool = True

    # Unitree examples: motor_cmd[29].q = 1 enable, 0 disable (Weight / NotUsedJoint)
    enable_axis: Optional[int] = 29
    enable_q: float = 1.0
