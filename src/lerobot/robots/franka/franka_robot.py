from lerobot.robots.robot import Robot
from lerobot.robots.config import RobotConfig
import numpy as np

class FrankaConfig(RobotConfig):
    robot_ip: str = "172.16.0.2"
    gripper: bool = True

class FrankaRobot(Robot):
    config_class = FrankaConfig
    name = "franka"

    JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

    def __init__(self, config):
        super().__init__(config)
        from franky import FrankaArm
        self.arm = FrankaArm(config.robot_ip)
        self.bus = type("MockBus", (), {"motors": {name: None for name in self.JOINT_NAMES}})()
        self.cameras = {}
        self._connected = False
    
    @property
    def observation_features(self):
        features = {f"{j}.pos": float for j in self.JOINT_NAMES}
        features.update({f"{j}.vel": float for j in self.JOINT_NAMES})
        if self.config.gripper:
            features["gripper.width"] = float
            
        return features
    
    @property
    def action_features(self):
        features = {f"{j}.pos": float for j in self.JOINT_NAMES}
        if self.config.gripper:
            features["gripper.width"] = float
        return features
    
    @property
    def is_connected(self):
        return self._connected
    
    @property
    def is_calibrated(self):
        return True
    
    def connect(self, calibrate: bool = True):
        self._connected = True
        self.configure()
    
    def configure(self):
        # TODO: configure the robot
        pass

    def disconnect(self):
        self._connected = False
    
    def get_observation(self):
        obs = {}
        qpos = self.arm.current_joint_positions()
        qvel = self.arm.current_joint_velocities()
        for i, j in enumerate(self.JOINT_NAMES):
            obs[f"{j}.pos"] = float(qpos[i])
            obs[f"{j}.vel"] = float(qvel[i])

        if self.config.gripper:
            obs["gripper.width"] = float(self.arm.current_gripper_width())
        return obs
    
    def send_action(self, action):
        target_joints = [action[f"{j}.pos"] for j in self.JOINT_NAMES]
        self.arm.move_joints(target_joints)
        if self.config.gripper:
            self.arm.set_gripper_width(action["gripper.width"])
        return action
