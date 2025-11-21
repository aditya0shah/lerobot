#!/usr/bin/env python
"""Test script to verify Franka robot connection via Nook server."""

import sys
import os
import importlib.util
from pathlib import Path

import numpy as np

# Import directly from the files to avoid package dependencies
base_path = Path(__file__).parent / "src" / "lerobot" / "robots" / "franka"

franka_client_path = base_path / "franka_client.py"
spec_client = importlib.util.spec_from_file_location("franka_client", franka_client_path)
franka_client_module = importlib.util.module_from_spec(spec_client)
spec_client.loader.exec_module(franka_client_module)
FrankaClient = franka_client_module.FrankaClient

franka_robot_path = base_path / "franka_robot.py"
spec_robot = importlib.util.spec_from_file_location("franka_robot", franka_robot_path)
franka_robot_module = importlib.util.module_from_spec(spec_robot)
spec_robot.loader.exec_module(franka_robot_module)
FrankaRemoteConfig = franka_robot_module.FrankaRemoteConfig
FrankaRemoteRobot = franka_robot_module.FrankaRemoteRobot


def test_franka_connection():
    """Test basic connection, observation, and action sending."""
    
    # Create config - adjust server_address if needed
    config = FrankaRemoteConfig(
        server_address="tcp://172.16.0.2:5555",  # Update if your server is at different address
        control_mode="joint_delta",
        dynamics_factor=0.2,
        gripper=True,
    )
    
    # Create robot instance
    robot = FrankaRemoteRobot(config)
    
    print("=" * 60)
    print("Testing Franka Robot Connection")
    print("=" * 60)
    
    try:
        # Test 1: Connect
        print("\n[1/5] Connecting to robot...")
        robot.connect(calibrate=False)
        print(f"✓ Connected! is_connected: {robot.is_connected}")
        
        # Test 2: Get initial observation
        print("\n[2/5] Getting initial observation...")
        obs = robot.get_observation()
        print("✓ Observation received:")
        for key, value in obs.items():
            print(f"  {key}: {value:.4f}")
        
        # Test 3: Check observation features
        print("\n[3/5] Checking observation features...")
        obs_features = robot.observation_features
        print(f"✓ Observation features: {list(obs_features.keys())}")
        
        # Test 4: Send a small test action (small delta movement)
        print("\n[4/5] Sending test action (small delta on first joint)...")
        # Get current joint positions
        current_joints = {f"{j}.pos": obs[f"{j}.pos"] for j in robot.JOINT_NAMES}
        
        # Create a small delta action (very small movement for safety)
        action = current_joints.copy()
        action["joint1.pos"] += 0.01  # Small delta: 0.01 radians (~0.57 degrees)
        action["gripper.width"] = 0.04  # Keep gripper open
        
        print(f"  Current joint1.pos: {current_joints['joint1.pos']:.4f}")
        print(f"  Target joint1.pos: {action['joint1.pos']:.4f}")
        
        sent_action = robot.send_action(action)
        print("✓ Action sent successfully")
        
        # Test 5: Get observation after action
        print("\n[5/5] Getting observation after action...")
        import time
        time.sleep(0.1)  # Small delay to let action execute
        obs_after = robot.get_observation()
        print(f"  New joint1.pos: {obs_after['joint1.pos']:.4f}")
        print(f"  Movement: {obs_after['joint1.pos'] - current_joints['joint1.pos']:.4f} rad")
        print("✓ Observation after action received")
        
        print("\n" + "=" * 60)
        print("All tests passed! ✓")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n✗ Error during testing: {e}")
        import traceback
        traceback.print_exc()
        raise
    
    finally:
        # Always disconnect
        print("\nDisconnecting...")
        robot.disconnect()
        print(f"✓ Disconnected. is_connected: {robot.is_connected}")


if __name__ == "__main__":
    test_franka_connection()
