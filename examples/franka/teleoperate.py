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

"""Teleoperate a Franka Emika Panda with the keyboard.

Prereq: a frankz-compatible ZMQ server is running on the control PC at ``remote_ip``.
See docs/source/franka.mdx for the safety notes (sticky-init hazard, binary gripper).
"""

import time

from lerobot.robots.franka import FrankaRobot, FrankaRobotConfig
from lerobot.teleoperators.franka_keyboard import FrankaKeyboardTeleop, FrankaKeyboardTeleopConfig
from lerobot.utils.robot_utils import busy_wait

FPS = 10

robot_config = FrankaRobotConfig(
    remote_ip="172.16.0.1",
    id="my_panda",
    # 0.05 rad per step cap prevents a bad action from becoming a fast slew.
    max_relative_target=0.05,
)
teleop_config = FrankaKeyboardTeleopConfig(
    id="my_laptop_keyboard",
    # Keep this well under the effective Panda joint limit (dynamics_factor scales it).
    max_joint_velocity_rad_s=0.2,
)

robot = FrankaRobot(robot_config)
teleop = FrankaKeyboardTeleop(teleop_config)

robot.connect()
teleop.connect()

if not robot.is_connected or not teleop.is_connected:
    raise ValueError("Robot or teleop is not connected!")

print("Starting teleop loop. Controls: 1-7/!@#$%^& move joints, o/c open/close, ESC exit.")
try:
    while True:
        t0 = time.perf_counter()

        _ = robot.get_observation()
        action = teleop.get_action()
        _ = robot.send_action(action)

        busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
finally:
    teleop.disconnect()
    robot.disconnect()
