#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property
from typing import Any

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_franka import FrankaRobotConfig
from .constants import FRANKA_JOINT_POS_MAX, FRANKA_JOINT_POS_MIN, FRANKA_JOINTS
from .franka_client import FrankaClient

logger = logging.getLogger(__name__)


class FrankaRobot(Robot):
    """Franka Emika Panda (FCI) arm driven through a frankz-compatible ZMQ server.

    Unlike the Feetech/Dynamixel arms in LeRobot, this robot has no local motor bus: it talks
    to a server running on the realtime control PC next to the arm. A few things are specific to
    this robot and worth knowing:

    * Joint action/observation values are ABSOLUTE POSITIONS IN RADIANS.
    * The gripper is BINARY. `gripper.pos` is exposed in [0, 100] for consistency with other
      LeRobot arms, but any value >= 50 opens the hand and < 50 closes it.
    * The arm holds its last commanded joint position after `disconnect()`; there is no stop
      command in the protocol. Use the Franka Desk web UI or the hardware e-stop to release it.

    HIL-SERL is not yet supported (that pipeline expects a motor bus with sync_read/sync_write).
    """

    config_class = FrankaRobotConfig
    name = "franka"

    def __init__(self, config: FrankaRobotConfig):
        super().__init__(config)
        self.config = config
        self.client: FrankaClient | None = None
        self.cameras = make_cameras_from_configs(config.cameras)
        # Cache of the last observed joint positions, used to cap relative motion in send_action.
        self._last_qpos: dict[str, float] | None = None

    @property
    def _motors_ft(self) -> dict[str, type]:
        features = {f"{joint}.pos": float for joint in FRANKA_JOINTS}
        if self.config.gripper:
            features["gripper.pos"] = float
        return features

    @property
    def _velocity_ft(self) -> dict[str, type]:
        return {f"{joint}.vel": float for joint in FRANKA_JOINTS}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {**self._motors_ft}
        if self.config.with_joint_velocity:
            features.update(self._velocity_ft)
        features.update(self._cameras_ft)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.client is not None and all(cam.is_connected for cam in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        # Joint zeroing and limits are owned by the Franka control box / FCI, so there is no
        # LeRobot-side calibration to perform.
        return True

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        try:
            self.client = FrankaClient(
                server_address=f"tcp://{self.config.remote_ip}:{self.config.port}",
                dynamics_factor=self.config.dynamics_factor,
                home_joint_positions=self.config.home_joint_positions,
                connect_timeout_s=self.config.connect_timeout_s,
                request_timeout_s=self.config.request_timeout_s,
                home_timeout_s=self.config.home_timeout_s,
                server_control_mode_check=self.config.server_control_mode_check,
            )
        except Exception:
            # Never leak a half-open client / ZMQ context if construction fails partway.
            if self.client is not None:
                self.client.close()
                self.client = None
            raise

        for cam in self.cameras.values():
            cam.connect()

        if self.config.home_on_connect:
            logger.warning(f"{self} home_on_connect=True: commanding the arm to its home pose.")
            self.home()

        self.configure()
        logger.info(f"{self} connected.")

    def configure(self) -> None:
        pass

    def home(self) -> None:
        """Drive the arm to its home configuration (blocking). Commands physical motion."""
        if self.client is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        logger.warning(f"{self} homing: the arm will move to {self.config.home_joint_positions}.")
        self.client.reset()

    def calibrate(self) -> None:
        # No-op: calibration is owned by the Franka control box (see `is_calibrated`).
        pass

    def get_observation(self) -> dict[str, Any]:
        if self.client is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        obs_raw = self.client.get_obs()
        if "qpos" not in obs_raw or len(obs_raw["qpos"]) < len(FRANKA_JOINTS):
            raise RuntimeError(
                f"{self}: server observation is missing a valid 'qpos' (got {obs_raw.get('qpos')!r})."
            )

        obs: dict[str, Any] = {}
        for i, joint in enumerate(FRANKA_JOINTS):
            obs[f"{joint}.pos"] = float(obs_raw["qpos"][i])
        # Cache joint positions for relative-target safety capping in send_action.
        self._last_qpos = {joint: obs[f"{joint}.pos"] for joint in FRANKA_JOINTS}

        if self.config.with_joint_velocity:
            for i, joint in enumerate(FRANKA_JOINTS):
                obs[f"{joint}.vel"] = float(obs_raw["qvel"][i])

        if self.config.gripper:
            # gripper_state is a shape-(1,) array of the normalized opening in [0, 1]; index it
            # (float() on an ndim>0 array is deprecated) and scale to LeRobot's 0-100 convention.
            obs["gripper.pos"] = float(obs_raw["gripper_state"][0]) * 100.0

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command absolute joint targets (radians) and, optionally, the binary gripper.

        The per-joint change is capped by `config.max_relative_target` and the absolute target is
        clipped to the Panda joint limits. The returned dict reflects the values ACTUALLY sent
        (safe/clipped joints, and the quantized 0/100 gripper), so it replays exactly.
        """
        if self.client is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = {joint: float(action[f"{joint}.pos"]) for joint in FRANKA_JOINTS}

        # Clip to absolute joint limits (the server does not clip in joint_position mode).
        for i, joint in enumerate(FRANKA_JOINTS):
            goal_pos[joint] = float(
                np.clip(goal_pos[joint], FRANKA_JOINT_POS_MIN[i], FRANKA_JOINT_POS_MAX[i])
            )

        # Cap the per-step change relative to the current joint positions.
        if self.config.max_relative_target is not None:
            present = self._last_qpos
            if present is None:
                obs_raw = self.client.get_obs()
                present = {joint: float(obs_raw["qpos"][i]) for i, joint in enumerate(FRANKA_JOINTS)}
                self._last_qpos = present
            goal_present = {joint: (goal_pos[joint], present[joint]) for joint in FRANKA_JOINTS}
            goal_pos = ensure_safe_goal_position(goal_present, self.config.max_relative_target)

        joint_vec = np.array([goal_pos[joint] for joint in FRANKA_JOINTS], dtype=np.float64)

        sent: dict[str, Any] = {f"{joint}.pos": goal_pos[joint] for joint in FRANKA_JOINTS}

        if self.config.gripper:
            # Latch to the server's binary command so the semantics don't depend on its threshold.
            gripper_pos = float(action.get("gripper.pos", 100.0))
            gripper_wire = 1.0 if gripper_pos >= 50.0 else 0.0
            self.client.step(np.concatenate([joint_vec, [gripper_wire]]), blocking=self.config.blocking)
            sent["gripper.pos"] = 100.0 if gripper_wire >= 0.5 else 0.0
        else:
            # A length-7 action leaves the hand untouched (the server only actuates it when
            # len(action) > 7), so a disabled gripper is never commanded.
            self.client.step(joint_vec, blocking=self.config.blocking)

        return sent

    def disconnect(self) -> None:
        if self.client is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        for cam in self.cameras.values():
            cam.disconnect()
        self.client.close()
        self.client = None
        # NOTE: the arm holds its last commanded joint position after disconnect; there is no stop
        # command in the protocol. Use Franka Desk or the e-stop to release it.
        logger.info(f"{self} disconnected.")
