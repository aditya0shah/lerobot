# src/lerobot/robots/franka_remote.py
from lerobot.robots.robot import Robot
from lerobot.robots.config import RobotConfig
from typing import Any
import numpy as np
from functools import cached_property


from lerobot.robots.franka.franka_client import FrankaClient

class FrankaRemoteConfig(RobotConfig):
    def __init__(self, server_address: str = "tcp://172.16.0.1:5555", control_mode: str = "joint_delta", dynamics_factor: float = 0.2, gripper: bool = True):
        self.server_address = server_address
        self.control_mode = control_mode
        self.dynamics_factor = dynamics_factor
        self.gripper = gripper

class FrankaRemoteRobot(Robot):
    config_class = FrankaRemoteConfig
    name = "franka_remote"

    JOINT_NAMES = ["joint1","joint2","joint3","joint4","joint5","joint6","joint7"]

    def __init__(self, config: FrankaRemoteConfig):
        super().__init__(config)
        self.config = config
        self.client: FrankaClient | None = None
        self._connected = False
        # For HIL-SERL compatibility
        self.bus = type("MockBus", (), {"motors": {name: None for name in self.JOINT_NAMES}})()
        self.cameras = {}


    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features = {f"{j}.pos": float for j in self.JOINT_NAMES}
        features.update({f"{j}.vel": float for j in self.JOINT_NAMES})
        if self.config.gripper:
            features["gripper.width"] = float
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        features = {f"{j}.pos": float for j in self.JOINT_NAMES}
        if self.config.gripper:
            features["gripper.width"] = float
        return features

    @property
    def is_connected(self) -> bool:
        return self._connected and self.client is not None

    @property
    def is_calibrated(self) -> bool:
        return True  # Franka has built-in calibration; optional to implement


    def connect(self, calibrate: bool = True) -> None:
        self.client = FrankaClient(
            server_address=self.config.server_address,
            control_mode=self.config.control_mode,
            dynamics_factor=self.config.dynamics_factor
        )
        self.client.reset()
        self._connected = True
        self.configure()

    def configure(self) -> None:
        pass

    def disconnect(self) -> None:
        if self.client:
            self.client.close()
        self.client = None
        self._connected = False

    def get_observation(self) -> dict[str, Any]:
        assert self.client is not None, "Robot not connected"
        obs_raw = self.client.get_obs()
        obs = {}
        for i, j in enumerate(self.JOINT_NAMES):
            obs[f"{j}.pos"] = float(obs_raw["qpos"][i])
            obs[f"{j}.vel"] = float(obs_raw["qvel"][i])
        if self.config.gripper:
            obs["gripper.width"] = float(obs_raw.get("gripper_state", 0.0))
        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        assert self.client is not None, "Robot not connected"
        joint_action = np.array([action[f"{j}.pos"] for j in self.JOINT_NAMES], dtype=float)
        gripper = float(action.get("gripper.width", 1.0))
        full_action = np.concatenate([joint_action, [gripper]])
        self.client.step(full_action, blocking=True)
        return action

    def calibrate(self) -> None:
        pass
