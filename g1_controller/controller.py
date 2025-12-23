from __future__ import annotations

import time
import threading
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

from .limits import LIMITS
from .kinematics import user_deg_to_motor_rad, motor_rad_to_user_deg
from .types import ControllerConfig


@dataclass
class _Interpolation:
    start_val: float
    end_val: float
    start_time: float
    duration: float


class G1ArmController:
    def __init__(
        self,
        config: ControllerConfig = ControllerConfig(),
        on_state: Optional[Callable[[LowState_], None]] = None,
    ) -> None:
        self.config = config
        self.dt = float(config.dt)
        self.crc = CRC()

        self._lock = threading.RLock()

        self.low_state: Optional[LowState_] = None
        self.initialized = False

        # Motor-space targets (radians)
        self.targets: dict[int, float] = {}
        self.interpolations: dict[int, _Interpolation] = {}

        # Optional callback for external user
        self._on_state = on_state

        ChannelFactoryInitialize(0, self.config.network_interface)

        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.publisher.Init()

        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._low_state_handler, 10)

        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.thread = RecurrentThread(
            interval=self.dt,
            target=self._control_step,
            name="g1_control",
        )

    # ---------- lifecycle ----------

    def start(self, wait: bool = True) -> None:
        if self.config.verbose:
            print("Waiting for robot state...")

        if wait:
            while not self.initialized:
                time.sleep(0.1)

        self.thread.Start()

        if self.config.verbose:
            print("Control loop started.")

    def stop(self) -> None:
        # Best-effort stop (depends on unitree RecurrentThread implementation)
        if hasattr(self.thread, "Stop"):
            try:
                self.thread.Stop()
            except Exception:
                pass

    def close(self) -> None:
        self.stop()

    def __enter__(self) -> "G1ArmController":
        self.start(wait=True)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ---------- state ----------

    def _low_state_handler(self, msg: LowState_) -> None:
        with self._lock:
            self.low_state = msg
            if self._on_state is not None:
                try:
                    self._on_state(msg)
                except Exception:
                    # Don't crash DDS callback.
                    pass

            if not self.initialized:
                # Initialize targets from current motor positions.
                for axis in LIMITS.keys():
                    try:
                        q = float(msg.motor_state[axis].q)
                    except Exception:
                        # If firmware/IDL differs, fail fast with a clear message.
                        raise RuntimeError(f"Failed to read motor_state[{axis}].q from LowState_")

                    self.targets[axis] = q
                    if self.config.verbose:
                        user_deg = motor_rad_to_user_deg(axis, q)
                        motor_deg = float(np.rad2deg(q))
                        print(
                            f"Axis {axis} initial motor={motor_deg:.2f} deg, user={user_deg:.2f} deg"
                        )

                self.initialized = True

    # ---------- motion API ----------

    def set_axes(self, axes_deg: dict[int, float], duration: float, blocking: bool = True) -> None:
        """
        Schedule multiple axes in user-degrees.
        If blocking=True, waits until duration elapses (like your original code).
        """
        for axis_num, deg_value in axes_deg.items():
            self.set_axis(axis_num, deg_value, duration)

        if blocking:
            time.sleep(float(duration) + 0.1)  # prevent overwrite, same intent as original

    def set_axis(self, axis_num: int, deg_value: float, duration: float) -> None:
        """
        Schedule one axis in user-degrees.
        Internally converted to motor radians with your original sign/offset rules.
        """
        if axis_num not in LIMITS:
            raise ValueError(f"Axis {axis_num} is not defined in limits.")

        with self._lock:
            if axis_num not in self.targets:
                # If called before initialization, we still allow scheduling by assuming start=0.
                # But it’s usually better to call start(wait=True) first.
                self.targets.setdefault(axis_num, 0.0)

            motor_rad = user_deg_to_motor_rad(axis_num, float(deg_value))
            lim = LIMITS[axis_num]
            safe_rad = float(np.clip(motor_rad, lim.min_rad, lim.max_rad))

            if motor_rad != safe_rad and self.config.verbose:
                print(f"Warning: Axis {axis_num} command clipped to safety limit.")

            start_val = float(self.targets[axis_num])
            self.interpolations[axis_num] = _Interpolation(
                start_val=start_val,
                end_val=safe_rad,
                start_time=time.time(),
                duration=max(float(duration), float(self.config.min_duration)),
            )

    def set_all_axes_to_zero(self, duration: float, blocking: bool = True) -> None:
        """Equivalent to your set_all_axes_to_zero(): command 0deg for all limited joints."""
        for axis in LIMITS.keys():
            self.set_axis(axis, 0.0, duration)
        if blocking:
            time.sleep(float(duration) + 0.1)

    # ---------- control loop ----------

    def _control_step(self) -> None:
        now = time.time()

        # Keep your original "enable" behavior
        if self.config.enable_axis is not None:
            try:
                self.low_cmd.motor_cmd[self.config.enable_axis].q = float(self.config.enable_q)
            except Exception:
                # If enable axis doesn't exist in this firmware, ignore.
                pass

        with self._lock:
            for axis in LIMITS.keys():
                # interpolation update
                if axis in self.interpolations:
                    interp = self.interpolations[axis]
                    elapsed = now - interp.start_time
                    ratio = float(np.clip(elapsed / interp.duration, 0.0, 1.0))
                    self.targets[axis] = interp.start_val + (interp.end_val - interp.start_val) * ratio

                    if ratio >= 1.0:
                        del self.interpolations[axis]

                # write motor command
                cmd = self.low_cmd.motor_cmd[axis]
                cmd.q = float(self.targets[axis])
                cmd.dq = 0.0
                cmd.kp = float(self.config.gains.kp)
                cmd.kd = float(self.config.gains.kd)
                cmd.tau = 0.0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)
