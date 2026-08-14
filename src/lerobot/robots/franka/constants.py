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

"""Shared constants for the Franka Emika Panda robot and its keyboard teleoperator.

These values are the single source of truth used by the robot, the teleoperator, and the
tests, so that the dataset schema (joint names) and the safety limits stay consistent.
"""

# Joint names use the underscored `joint_<n>` convention from the LeRobot
# "Bring Your Own Hardware" tutorial (docs/source/integrate_hardware.mdx). This is a
# permanent commitment: it becomes part of the recorded dataset feature keys.
FRANKA_JOINTS: tuple[str, ...] = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
)

# Joint position limits in radians for the Franka Emika Panda (FCI).
# These match the clip ranges the frankz server applies server-side.
# TODO: The Franka Research 3 (FR3) has different limits; expose these per-model if FR3 is added.
FRANKA_JOINT_POS_MIN: tuple[float, ...] = (
    -2.8973,
    -1.7628,
    -2.8973,
    -3.0718,
    -2.8973,
    -0.0175,
    -2.8973,
)
FRANKA_JOINT_POS_MAX: tuple[float, ...] = (
    2.8973,
    1.7628,
    2.8973,
    -0.0698,
    2.8973,
    3.7525,
    2.8973,
)

# Default "home" joint configuration (radians), matching the frankz server's reset pose.
FRANKA_HOME_QPOS: tuple[float, ...] = (
    0.0854,
    -0.8716,
    -0.1131,
    -2.9228,
    -0.1388,
    2.0575,
    0.8828,
)

# Maximum gripper width in metres. The `gripper_state` value on the wire is the
# NORMALIZED opening `width / FRANKA_GRIPPER_MAX_WIDTH_M` in [0, 1] (NOT metres, despite
# what the frankz README states). LeRobot exposes it as `gripper.pos` in [0, 100].
FRANKA_GRIPPER_MAX_WIDTH_M: float = 0.08

# The only control mode this integration sends to the server. Joint values are always
# absolute radians. Delta and cartesian modes are intentionally not supported (see docs).
FRANKA_SERVER_CONTROL_MODE: str = "joint_position"
