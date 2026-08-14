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

from dataclasses import dataclass

from lerobot.robots.franka.constants import FRANKA_HOME_QPOS

from ..config import TeleoperatorConfig
from ..keyboard.configuration_keyboard import KeyboardTeleopConfig


@TeleoperatorConfig.register_subclass("franka_keyboard")
@dataclass
class FrankaKeyboardTeleopConfig(KeyboardTeleopConfig):
    """Keyboard teleoperator that emits absolute Franka joint targets.

    The teleoperator owns its own target integrator; the joint step per tick is
    `max_joint_velocity_rad_s * dt`, so the motion rate is independent of the loop frequency.
    """

    # Angular rate applied while a joint key is held. Keep well under the Panda joint limits
    # (2.175 / 2.610 rad/s), and remember the server's `dynamics_factor` scales the effective
    # limit (at 0.2 the practical ceiling is ~0.435 rad/s).
    max_joint_velocity_rad_s: float = 0.4

    # Initial joint targets (radians) the integrator starts from on connect.
    home_joint_positions: tuple[float, ...] = FRANKA_HOME_QPOS

    # Whether the gripper target starts open.
    gripper_open_on_start: bool = True

    # Clamp the integration dt (s) so a stalled control loop cannot produce a huge single step.
    max_dt_s: float = 0.2
