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

"""Hardware-free tests for the Franka robot. The ZMQ client is patched with a fake so the
tests cover the LeRobot contract and every safety-critical behavior without a Panda arm."""

import dataclasses
from unittest.mock import MagicMock, patch

import draccus
import numpy as np
import pytest

from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.robots import make_robot_from_config
from lerobot.robots.franka import (
    FRANKA_HOME_QPOS,
    FRANKA_JOINT_POS_MAX,
    FRANKA_JOINT_POS_MIN,
    FRANKA_JOINTS,
    FrankaRobot,
    FrankaRobotConfig,
)
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError


def _make_client_mock(qpos=None, gripper_state=1.0):
    if qpos is None:
        qpos = np.zeros(7, dtype=np.float32)
    client = MagicMock(name="FrankaClientMock")
    client.get_obs.return_value = {
        "qpos": np.asarray(qpos, dtype=np.float32),
        "qvel": np.zeros(7, dtype=np.float32),
        "ee_pose": np.zeros(7, dtype=np.float32),
        "gripper_state": np.array([gripper_state], dtype=np.float32),
    }
    client.step.return_value = None
    client.reset.return_value = None
    return client


@pytest.fixture
def patch_client():
    with patch("lerobot.robots.franka.franka.FrankaClient") as ctor:
        client = _make_client_mock()
        ctor.return_value = client
        yield ctor, client


def test_config_type_is_registered():
    c = FrankaRobotConfig(remote_ip="127.0.0.1")
    assert c.type == "franka"


def test_config_fields_include_all_options():
    c = FrankaRobotConfig(remote_ip="127.0.0.1")
    fields = {f.name for f in dataclasses.fields(c)}
    for expected in {
        "remote_ip",
        "port",
        "connect_timeout_s",
        "request_timeout_s",
        "home_timeout_s",
        "dynamics_factor",
        "blocking",
        "max_relative_target",
        "gripper",
        "with_joint_velocity",
        "home_on_connect",
        "home_joint_positions",
        "server_control_mode_check",
        "cameras",
        "id",
        "calibration_dir",
    }:
        assert expected in fields, expected


def test_config_asdict_round_trips_all_fields():
    c = FrankaRobotConfig(remote_ip="127.0.0.1", id="panda", max_relative_target=0.05)
    d = dataclasses.asdict(c)
    assert d["remote_ip"] == "127.0.0.1"
    assert d["id"] == "panda"
    assert d["max_relative_target"] == 0.05
    assert d["gripper"] is True


def test_draccus_decode_from_dict():
    c = draccus.decode(FrankaRobotConfig, {"remote_ip": "172.16.0.1", "id": "panda"})
    assert isinstance(c, FrankaRobotConfig)
    assert c.remote_ip == "172.16.0.1"
    assert c.id == "panda"


def test_config_validation_rejects_bad_dynamics_factor():
    with pytest.raises(ValueError, match="dynamics_factor"):
        FrankaRobotConfig(remote_ip="127.0.0.1", dynamics_factor=1.5)


def test_config_validation_rejects_out_of_limit_home_pose():
    bad = list(FRANKA_HOME_QPOS)
    bad[3] = FRANKA_JOINT_POS_MAX[3] + 1.0
    with pytest.raises(ValueError, match="home_joint_positions"):
        FrankaRobotConfig(remote_ip="127.0.0.1", home_joint_positions=tuple(bad))


def test_make_robot_from_config_returns_franka():
    r = make_robot_from_config(FrankaRobotConfig(remote_ip="127.0.0.1", id="panda"))
    assert isinstance(r, FrankaRobot)
    assert r.calibration_fpath.name == "panda.json"


def test_features_shape_matches_action_by_default():
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1"))
    assert set(r.observation_features) == set(r.action_features)
    ds = hw_to_dataset_features(r.observation_features, "observation")
    assert ds["observation.state"]["shape"] == (8,), ds["observation.state"]["shape"]


def test_features_widen_with_joint_velocity():
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1", with_joint_velocity=True))
    ds = hw_to_dataset_features(r.observation_features, "observation")
    assert ds["observation.state"]["shape"] == (15,)


def test_features_shrink_without_gripper():
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1", gripper=False))
    ds_o = hw_to_dataset_features(r.observation_features, "observation")
    ds_a = hw_to_dataset_features(r.action_features, "action")
    assert ds_o["observation.state"]["shape"] == (7,)
    assert ds_a["action"]["shape"] == (7,)


def test_features_callable_when_disconnected():
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1"))
    assert not r.is_connected
    # Must not raise when disconnected (Robot contract).
    _ = r.observation_features
    _ = r.action_features


def test_get_observation_raises_when_disconnected():
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1"))
    with pytest.raises(DeviceNotConnectedError):
        r.get_observation()


def test_send_action_raises_when_disconnected():
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1"))
    with pytest.raises(DeviceNotConnectedError):
        r.send_action({f"{j}.pos": 0.0 for j in FRANKA_JOINTS} | {"gripper.pos": 100.0})


def test_disconnect_raises_when_disconnected():
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1"))
    with pytest.raises(DeviceNotConnectedError):
        r.disconnect()


def test_connect_does_not_command_motion(patch_client):
    _, client = patch_client
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1"))
    r.connect()
    try:
        assert not client.reset.called, "connect() must not call client.reset() by default"
        assert r.is_connected
    finally:
        r.disconnect()


def test_connect_homes_when_opted_in(patch_client):
    _, client = patch_client
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1", home_on_connect=True))
    r.connect()
    try:
        client.reset.assert_called_once()
    finally:
        r.disconnect()


def test_double_connect_raises(patch_client):
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1"))
    r.connect()
    try:
        with pytest.raises(DeviceAlreadyConnectedError):
            r.connect()
    finally:
        r.disconnect()


def test_observation_maps_gripper_from_normalized_to_zero_hundred(patch_client):
    ctor, client = patch_client
    client.get_obs.return_value = {
        "qpos": np.zeros(7, dtype=np.float32),
        "qvel": np.zeros(7, dtype=np.float32),
        "ee_pose": np.zeros(7, dtype=np.float32),
        "gripper_state": np.array([0.5], dtype=np.float32),
    }
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1"))
    r.connect()
    try:
        obs = r.get_observation()
        assert obs["gripper.pos"] == pytest.approx(50.0)
    finally:
        r.disconnect()


def test_send_action_latches_gripper_to_binary_wire(patch_client):
    ctor, client = patch_client
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1", max_relative_target=None))
    r.connect()
    try:
        action = {f"{j}.pos": 0.0 for j in FRANKA_JOINTS}
        action["gripper.pos"] = 70.0
        sent = r.send_action(action)
        assert sent["gripper.pos"] == 100.0
        args, _ = client.step.call_args
        wire = args[0]
        assert wire.shape == (8,)
        assert wire[-1] == 1.0

        action["gripper.pos"] = 20.0
        sent = r.send_action(action)
        assert sent["gripper.pos"] == 0.0
        args, _ = client.step.call_args
        assert args[0][-1] == 0.0
    finally:
        r.disconnect()


def test_send_action_sends_length_7_when_gripper_disabled(patch_client):
    ctor, client = patch_client
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1", gripper=False, max_relative_target=None))
    r.connect()
    try:
        action = {f"{j}.pos": 0.0 for j in FRANKA_JOINTS}
        sent = r.send_action(action)
        assert "gripper.pos" not in sent
        args, _ = client.step.call_args
        assert args[0].shape == (7,), args[0].shape
    finally:
        r.disconnect()


def test_send_action_caps_relative_target(patch_client):
    ctor, client = patch_client
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1", max_relative_target=0.1))
    r.connect()
    try:
        # Seed the cached qpos so the cap runs against a known starting point.
        _ = r.get_observation()
        action = {f"{j}.pos": 1.0 for j in FRANKA_JOINTS}
        action["gripper.pos"] = 100.0
        sent = r.send_action(action)
        # Every joint must have moved at most `max_relative_target` from 0.0.
        for i, j in enumerate(FRANKA_JOINTS):
            step = sent[f"{j}.pos"] - 0.0
            # The absolute limit for joint 4 forces the goal negative (max = -0.0698), then the
            # relative cap allows a step of up to 0.1 in either direction from 0.0.
            assert abs(step) <= 0.1 + 1e-9, (j, step)
            # The direction must respect the absolute limits — a joint's cap-clamped goal must be
            # within [MIN, MAX].
            assert FRANKA_JOINT_POS_MIN[i] <= sent[f"{j}.pos"] <= FRANKA_JOINT_POS_MAX[i]
    finally:
        r.disconnect()


def test_send_action_clips_to_joint_limits(patch_client):
    ctor, client = patch_client
    r = FrankaRobot(FrankaRobotConfig(remote_ip="127.0.0.1", max_relative_target=None))
    r.connect()
    try:
        action = {f"{j}.pos": 1000.0 for j in FRANKA_JOINTS}
        action["gripper.pos"] = 100.0
        sent = r.send_action(action)
        for i, j in enumerate(FRANKA_JOINTS):
            assert sent[f"{j}.pos"] == pytest.approx(FRANKA_JOINT_POS_MAX[i])
        action = {f"{j}.pos": -1000.0 for j in FRANKA_JOINTS}
        action["gripper.pos"] = 100.0
        sent = r.send_action(action)
        for i, j in enumerate(FRANKA_JOINTS):
            assert sent[f"{j}.pos"] == pytest.approx(FRANKA_JOINT_POS_MIN[i])
    finally:
        r.disconnect()
