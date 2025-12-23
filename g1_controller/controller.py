from __future__ import annotations

import time
import threading
from dataclasses import dataclass
from typing import Callable, Optional, Dict

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
from .poses import default_home_pose_motor_rad


@dataclass
class _Interpolation:
    start_val: float
    end_val: float
    start_time: float
    duration: float


class G1ArmController:
    def __init__(
        self,
        config: Optional[ControllerConfig] = None,
        on_state: Optional[Callable[[LowState_], None]] = None,
    ) -> None:
        self.config = config if config is not None else ControllerConfig()
        self.dt = float(self.config.dt)
        self.crc = CRC()

        self._lock = threading.RLock()

        self.low_state: Optional[LowState_] = None
        self.initialized = False

        # Motor-space targets (radians) for joints
        self.targets: dict[int, float] = {}
        self.interpolations: dict[int, _Interpolation] = {}

        # arm_sdk enable scalar (motor_cmd[enable_axis].q)
        self._arm_sdk_q: float = float(self.config.enable_q)
        self._arm_sdk_interp: Optional[_Interpolation] = None

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

        # start with arm_sdk enabled by default
        self.enable_arm_sdk()

        self.thread.Start()
        if self.config.verbose:
            print("Control loop started.")

    def stop(self) -> None:
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

    # ---------- arm_sdk enable/disable ----------

    def enable_arm_sdk(self) -> None:
        """Immediately enable arm_sdk (motor_cmd[enable_axis].q = 1)."""
        with self._lock:
            self._arm_sdk_interp = None
            self._arm_sdk_q = float(self.config.enable_q)

    def disable_arm_sdk(self, duration: float = 1.0, blocking: bool = True) -> None:
        """
        Smoothly release arm_sdk (motor_cmd[enable_axis].q: 1 -> 0).
        Similar to Unitree example Stage4.
        """
        if self.config.enable_axis is None:
            return

        with self._lock:
            start = float(self._arm_sdk_q)
            self._arm_sdk_interp = _Interpolation(
                start_val=start,
                end_val=0.0,
                start_time=time.time(),
                duration=max(float(duration), 0.1),
            )

        if blocking:
            time.sleep(float(duration) + 0.1)

    # ---------- state ----------

    def _low_state_handler(self, msg: LowState_) -> None:
        with self._lock:
            self.low_state = msg

            if self._on_state is not None:
                try:
                    self._on_state(msg)
                except Exception:
                    pass

            if not self.initialized:
                for axis in LIMITS.keys():
                    try:
                        q = float(msg.motor_state[axis].q)
                    except Exception:
                        raise RuntimeError(f"Failed to read motor_state[{axis}].q from LowState_")

                    self.targets[axis] = q

                    if self.config.verbose:
                        user_deg = motor_rad_to_user_deg(axis, q)
                        motor_deg = float(np.rad2deg(q))
                        print(f"Axis {axis} initial motor={motor_deg:.2f} deg, user={user_deg:.2f} deg")

                self.initialized = True

    # ---------- motion API (user degrees) ----------

    def set_axes(self, axes_deg: dict[int, float], duration: float, blocking: bool = True) -> None:
        # Any commanded motion should ensure arm_sdk is enabled
        self.enable_arm_sdk()

        for axis_num, deg_value in axes_deg.items():
            self.set_axis(axis_num, deg_value, duration)

        if blocking:
            time.sleep(float(duration) + 0.1)

    def set_axis(self, axis_num: int, deg_value: float, duration: float) -> None:
        if axis_num not in LIMITS:
            raise ValueError(f"Axis {axis_num} is not defined in limits.")

        # Any commanded motion should ensure arm_sdk is enabled
        self.enable_arm_sdk()

        with self._lock:
            if axis_num not in self.targets:
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

    def set_all_axes_to_zero(
        self,
        duration: float,
        blocking: bool = True,
        release_arm_sdk: bool = True,
        release_duration: float = 1.0,
        pose_motor_rad: Optional[Dict[int, float]] = None,
    ) -> None:
        pose = pose_motor_rad if pose_motor_rad is not None else default_home_pose_motor_rad()
        self.set_axes_motor(pose, duration=duration, blocking=blocking)
        if release_arm_sdk:
            self.disable_arm_sdk(duration=release_duration, blocking=blocking)

    def _control_step(self) -> None:
        now = time.time()

        with self._lock:
            # Update arm_sdk enable scalar interpolation and write it
            if self.config.enable_axis is not None:
                if self._arm_sdk_interp is not None:
                    interp = self._arm_sdk_interp
                    elapsed = now - interp.start_time
                    ratio = float(np.clip(elapsed / interp.duration, 0.0, 1.0))
                    self._arm_sdk_q = interp.start_val + (interp.end_val - interp.start_val) * ratio
                    if ratio >= 1.0:
                        self._arm_sdk_interp = None

                try:
                    self.low_cmd.motor_cmd[self.config.enable_axis].q = float(self._arm_sdk_q)
                except Exception:
                    pass

            # Joint interpolation + command write
            for axis in LIMITS.keys():
                if axis in self.interpolations:
                    interp = self.interpolations[axis]
                    elapsed = now - interp.start_time
                    ratio = float(np.clip(elapsed / interp.duration, 0.0, 1.0))
                    self.targets[axis] = interp.start_val + (interp.end_val - interp.start_val) * ratio
                    if ratio >= 1.0:
                        del self.interpolations[axis]

                cmd = self.low_cmd.motor_cmd[axis]
                cmd.q = float(self.targets[axis])
                cmd.dq = 0.0
                cmd.kp = float(self.config.gains.kp)
                cmd.kd = float(self.config.gains.kd)
                cmd.tau = 0.0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)
