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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig
from .constants import FRANKA_HOME_QPOS, FRANKA_JOINT_POS_MAX, FRANKA_JOINT_POS_MIN


@RobotConfig.register_subclass("franka")
@dataclass
class FrankaRobotConfig(RobotConfig):
    """Configuration for a Franka Emika Panda arm driven through a frankz-compatible ZMQ server.

    Joint targets are always absolute positions in radians (unlike most LeRobot arms, which use
    normalized or degree units). The gripper is binary.
    """

    # IP address of the control PC running the Franka server (no default: it is site-specific).
    remote_ip: str

    # ZMQ port the server binds.
    port: int = 5555

    # Timeout (s) for the initial connection/init handshake.
    connect_timeout_s: float = 5.0
    # Timeout (s) for regular step / get_obs requests.
    request_timeout_s: float = 5.0
    # Timeout (s) for the blocking home/reset motion.
    home_timeout_s: float = 30.0

    # Scales the server's velocity/acceleration limits, in [0, 1]. Higher is faster and riskier.
    dynamics_factor: float = 0.2

    # If True, `send_action` blocks until the motion completes. Requires a larger
    # `request_timeout_s`. Leave False for closed-loop teleop/policy control.
    blocking: bool = False

    # `max_relative_target` caps the per-step change of each joint target for safety, in radians.
    # Deliberately non-None by default (unlike SO-101) because this is a 7-DoF industrial arm.
    # Set to a scalar for all joints, a per-joint dict, or None to disable.
    max_relative_target: float | dict[str, float] | None = 0.05

    # Whether the arm has a gripper. When False, no gripper command is ever sent.
    gripper: bool = True

    # If True, also expose per-joint velocities (`<joint>.vel`) in the observation. Off by default
    # so the recorded `observation.state` matches the action dimensionality.
    with_joint_velocity: bool = False

    # If True, drive the arm to `home_joint_positions` during `connect()`. This commands physical
    # motion, so it is off by default; call `robot.home()` explicitly instead.
    home_on_connect: bool = False

    # Home joint configuration in radians, used by `home()` and sent to the server as its reset pose.
    home_joint_positions: tuple[float, ...] = FRANKA_HOME_QPOS

    # If True, refuse to connect unless the server reports it is in 'joint_position' mode. Guards
    # against the frankz sticky-init hazard (see franka_client.py). Only disable on a trusted server.
    server_control_mode_check: bool = True

    # Cameras keyed by name, added to the observation.
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self):
        if not 0.0 <= self.dynamics_factor <= 1.0:
            raise ValueError(f"dynamics_factor must be in [0, 1], got {self.dynamics_factor}")
        if len(self.home_joint_positions) != 7:
            raise ValueError(
                f"home_joint_positions must have 7 entries, got {len(self.home_joint_positions)}"
            )
        for i, q in enumerate(self.home_joint_positions):
            if not FRANKA_JOINT_POS_MIN[i] <= q <= FRANKA_JOINT_POS_MAX[i]:
                raise ValueError(
                    f"home_joint_positions[{i}]={q} is outside the joint limit "
                    f"[{FRANKA_JOINT_POS_MIN[i]}, {FRANKA_JOINT_POS_MAX[i]}]"
                )
        super().__post_init__()
