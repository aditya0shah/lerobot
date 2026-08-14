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

"""Record a Franka Emika Panda teleoperation dataset with the keyboard.

Note: `--dataset.fps` must match the rate you drive the control loop at (the client no longer
sleeps internally). See docs/source/franka.mdx for safety notes.
"""

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.processor import make_default_processors
from lerobot.robots.franka import FrankaRobot, FrankaRobotConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.franka_keyboard import FrankaKeyboardTeleop, FrankaKeyboardTeleopConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun

NUM_EPISODES = 2
FPS = 10
EPISODE_TIME_SEC = 30
RESET_TIME_SEC = 10
TASK_DESCRIPTION = "My task description"
HF_REPO_ID = "<hf_username>/<dataset_repo_id>"

robot_config = FrankaRobotConfig(remote_ip="172.16.0.1", id="my_panda", max_relative_target=0.05)
teleop_config = FrankaKeyboardTeleopConfig(id="my_laptop_keyboard", max_joint_velocity_rad_s=0.2)

robot = FrankaRobot(robot_config)
teleop = FrankaKeyboardTeleop(teleop_config)

# TODO(Steven): Update this example to use pipelines
teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

action_features = hw_to_dataset_features(robot.action_features, ACTION)
obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR)
dataset_features = {**action_features, **obs_features}

dataset = LeRobotDataset.create(
    repo_id=HF_REPO_ID,
    fps=FPS,
    features=dataset_features,
    robot_type=robot.name,
    use_videos=True,
    image_writer_threads=4,
)

robot.connect()
teleop.connect()

listener, events = init_keyboard_listener()
init_rerun(session_name="franka_record")

if not robot.is_connected or not teleop.is_connected:
    raise ValueError("Robot or teleop is not connected!")

print("Starting record loop...")
recorded_episodes = 0
while recorded_episodes < NUM_EPISODES and not events["stop_recording"]:
    log_say(f"Recording episode {recorded_episodes}")

    record_loop(
        robot=robot,
        events=events,
        fps=FPS,
        dataset=dataset,
        teleop=[teleop],
        control_time_s=EPISODE_TIME_SEC,
        single_task=TASK_DESCRIPTION,
        display_data=True,
        teleop_action_processor=teleop_action_processor,
        robot_action_processor=robot_action_processor,
        robot_observation_processor=robot_observation_processor,
    )

    if not events["stop_recording"] and (
        (recorded_episodes < NUM_EPISODES - 1) or events["rerecord_episode"]
    ):
        log_say("Reset the environment")
        record_loop(
            robot=robot,
            events=events,
            fps=FPS,
            teleop=[teleop],
            control_time_s=RESET_TIME_SEC,
            single_task=TASK_DESCRIPTION,
            display_data=True,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
        )

    if events["rerecord_episode"]:
        log_say("Re-record episode")
        events["rerecord_episode"] = False
        events["exit_early"] = False
        dataset.clear_episode_buffer()
        continue

    dataset.save_episode()
    recorded_episodes += 1

log_say("Stop recording")
robot.disconnect()
teleop.disconnect()
listener.stop()

dataset.finalize()
dataset.push_to_hub()
