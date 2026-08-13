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

"""Hardware-free tests for the Franka keyboard teleoperator."""

import time

import pytest

from lerobot.robots.franka import (
    FRANKA_HOME_QPOS,
    FRANKA_JOINT_POS_MAX,
    FRANKA_JOINT_POS_MIN,
    FRANKA_JOINTS,
    FrankaRobot,
    FrankaRobotConfig,
)
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.teleoperators.franka_keyboard import FrankaKeyboardTeleop, FrankaKeyboardTeleopConfig


class _FakeTeleop(FrankaKeyboardTeleop):
    """Bypass the pynput listener so tests can drive `current_pressed` directly."""

    def connect(self, calibrate: bool = True) -> None:
        self._q_target = list(self.config.home_joint_positions)
        self._gripper_target = 100.0 if self.config.gripper_open_on_start else 0.0
        self._last_t = None
        self.current_pressed = {}
        self._is_connected_forced = True

    @property
    def is_connected(self) -> bool:  # type: ignore[override]
        return getattr(self, "_is_connected_forced", False)

    def _drain_pressed_keys(self) -> None:
        pass  # tests set `self.current_pressed` directly

    def disconnect(self) -> None:
        self._is_connected_forced = False


def test_config_type_is_registered():
    c = FrankaKeyboardTeleopConfig()
    assert c.type == "franka_keyboard"


def test_make_teleoperator_from_config_dispatches():
    t = make_teleoperator_from_config(FrankaKeyboardTeleopConfig())
    assert isinstance(t, FrankaKeyboardTeleop)


def test_action_features_are_all_floats_and_named_like_robot():
    t = FrankaKeyboardTeleop(FrankaKeyboardTeleopConfig())
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1"))
    assert set(t.action_features) == set(r.action_features)
    for v in t.action_features.values():
        assert v is float


def test_get_action_starts_at_home_pose():
    t = _FakeTeleop(FrankaKeyboardTeleopConfig(max_dt_s=0.05))
    t.connect()
    a = t.get_action()
    for i, joint in enumerate(FRANKA_JOINTS):
        assert a[f"{joint}.pos"] == pytest.approx(FRANKA_HOME_QPOS[i])
    assert a["gripper.pos"] == 100.0


def test_integrator_advances_by_velocity_times_dt():
    cfg = FrankaKeyboardTeleopConfig(max_joint_velocity_rad_s=1.0, max_dt_s=0.05)
    t = _FakeTeleop(cfg)
    t.connect()
    _ = t.get_action()  # prime the integrator (dt=0 on the first tick)
    time.sleep(0.03)
    t.current_pressed = {"1": True}
    a = t.get_action()
    delta = a["joint_1.pos"] - FRANKA_HOME_QPOS[0]
    assert 0 < delta <= cfg.max_dt_s * cfg.max_joint_velocity_rad_s + 1e-6


def test_integrator_clips_to_joint_limits():
    cfg = FrankaKeyboardTeleopConfig(max_joint_velocity_rad_s=100.0, max_dt_s=0.2)
    t = _FakeTeleop(cfg)
    t.connect()
    _ = t.get_action()  # prime last_t; this tick's step is 0
    # A single tick with dt clamped to max_dt_s and velocity 100 rad/s produces a step of 20 rad,
    # so the joint reaches its positive limit in one call.
    time.sleep(cfg.max_dt_s + 0.01)
    t.current_pressed = {"1": True}
    a = t.get_action()
    assert a["joint_1.pos"] == pytest.approx(FRANKA_JOINT_POS_MAX[0])

    # Release '1' and press '!' (both target joint 1) so the residual positive keypress does not
    # win. Sleep past max_dt_s so a single tick drives all the way to the negative limit.
    t.current_pressed = {"1": False, "!": True}
    time.sleep(cfg.max_dt_s + 0.01)
    a = t.get_action()
    assert a["joint_1.pos"] == pytest.approx(FRANKA_JOINT_POS_MIN[0])


def test_gripper_latches_binary():
    t = _FakeTeleop(FrankaKeyboardTeleopConfig())
    t.connect()
    _ = t.get_action()
    t.current_pressed = {"c": True}
    a = t.get_action()
    assert a["gripper.pos"] == 0.0
    t.current_pressed = {"o": True}
    a = t.get_action()
    assert a["gripper.pos"] == 100.0


def test_action_keys_match_robot_and_build_dataset_frame():
    from lerobot.datasets.utils import build_dataset_frame, hw_to_dataset_features

    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1"))
    t = _FakeTeleop(FrankaKeyboardTeleopConfig())
    t.connect()
    action = t.get_action()
    frame_features = hw_to_dataset_features(r.action_features, "action")
    # Must not KeyError: the invariant that keeps make_default_processors' identity pipeline working.
    frame = build_dataset_frame(frame_features, action, prefix="action")
    assert frame["action"].shape == (8,)
