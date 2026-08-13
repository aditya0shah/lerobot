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
from typing import Any

import numpy as np

from lerobot.robots.franka.constants import FRANKA_JOINT_POS_MAX, FRANKA_JOINT_POS_MIN, FRANKA_JOINTS
from lerobot.utils.errors import DeviceNotConnectedError

from ..keyboard.teleop_keyboard import KeyboardTeleop
from .config_franka_keyboard import FrankaKeyboardTeleopConfig

logger = logging.getLogger(__name__)

# Number keys move a joint in the positive direction; the shifted glyph moves it negative.
# pynput reports the shifted character directly, so no shift-state bookkeeping is needed.
_POSITIVE_KEYS = {str(i + 1): i for i in range(len(FRANKA_JOINTS))}
_NEGATIVE_KEYS = dict(zip("!@#$%^&", range(len(FRANKA_JOINTS)), strict=False))
_GRIPPER_OPEN_KEY = "o"
_GRIPPER_CLOSE_KEY = "c"


class FrankaKeyboardTeleop(KeyboardTeleop):
    """Keyboard teleoperator for the Franka robot.

    Acts as a virtual leader arm: it holds its own joint/gripper targets and integrates keyboard
    input over time, emitting the exact `action_features` the `FrankaRobot` expects. It never
    reads the robot's observation, so the produced action stream is replayable.

    Controls:
        1-7            move joint 1-7 in the positive direction (hold to keep moving)
        ! @ # $ % ^ &  move joint 1-7 in the negative direction (shifted 1-7)
        o / c          open / close the gripper
        ESC            disconnect
    """

    config_class = FrankaKeyboardTeleopConfig
    name = "franka_keyboard"

    def __init__(self, config: FrankaKeyboardTeleopConfig):
        super().__init__(config)
        self.config = config
        self._q_target: list[float] = list(config.home_joint_positions)
        self._gripper_target: float = 100.0 if config.gripper_open_on_start else 0.0
        self._last_t: float | None = None

    @property
    def action_features(self) -> dict[str, type]:
        features = {f"{joint}.pos": float for joint in FRANKA_JOINTS}
        features["gripper.pos"] = float
        return features

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        super().connect()
        self._q_target = list(self.config.home_joint_positions)
        self._gripper_target = 100.0 if self.config.gripper_open_on_start else 0.0
        self._last_t = None

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self} is not connected. You need to run `connect()` before `get_action()`."
            )

        self._drain_pressed_keys()
        pressed = {key for key, is_down in self.current_pressed.items() if is_down}

        now = time.perf_counter()
        dt = 0.0 if self._last_t is None else min(now - self._last_t, self.config.max_dt_s)
        self._last_t = now
        step = self.config.max_joint_velocity_rad_s * dt

        for key in pressed:
            if key in _POSITIVE_KEYS:
                self._q_target[_POSITIVE_KEYS[key]] += step
            elif key in _NEGATIVE_KEYS:
                self._q_target[_NEGATIVE_KEYS[key]] -= step

        for i in range(len(FRANKA_JOINTS)):
            self._q_target[i] = float(
                np.clip(self._q_target[i], FRANKA_JOINT_POS_MIN[i], FRANKA_JOINT_POS_MAX[i])
            )

        # The gripper is binary; latch the target instead of stepping it.
        if _GRIPPER_OPEN_KEY in pressed:
            self._gripper_target = 100.0
        elif _GRIPPER_CLOSE_KEY in pressed:
            self._gripper_target = 0.0

        action = {f"{joint}.pos": self._q_target[i] for i, joint in enumerate(FRANKA_JOINTS)}
        action["gripper.pos"] = self._gripper_target
        return action

    def configure(self) -> None:
        pass

    def calibrate(self) -> None:
        pass

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError
