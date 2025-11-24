#!/usr/bin/env python
"""Simple keyboard teleoperation for Franka robot."""

import sys
import time

# Check for required dependencies
try:
    import numpy as np
except ImportError:
    print("Error: numpy is not installed. Install it with: pip install numpy")
    sys.exit(1)

try:
    from pynput import keyboard
except ImportError:
    print("Error: pynput is not installed. Install it with: pip install pynput")
    sys.exit(1)

# Import FrankaRemoteRobot
from lerobot.robots.franka import FrankaRemoteRobot, FrankaRemoteConfig

# Control parameters
JOINT_STEP = 0.4  # radians per keypress
GRIPPER_STEP = 0.02  # gripper width per keypress
FPS = 10  # Control frequency

# Current action state
pressed_keys = set()
shift_pressed = False

# Map shifted characters back to numbers
SHIFTED_TO_NUMBER = {
    '!': '1',  # Shift+1
    '@': '2',  # Shift+2
    '#': '3',  # Shift+3
    '$': '4',  # Shift+4
    '%': '5',  # Shift+5
    '^': '6',  # Shift+6
    '&': '7',  # Shift+7
}

def on_press(key):
    """Handle key press events."""
    global shift_pressed
    try:
        if key == keyboard.Key.shift or key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
            shift_pressed = True
        elif hasattr(key, 'char') and key.char:
            char = key.char
            # Check if it's a shifted number character
            if char in SHIFTED_TO_NUMBER:
                # Map shifted char to number and mark as negative
                base_char = SHIFTED_TO_NUMBER[char]
                pressed_keys.add(f'-{base_char}')  # Negative prefix
            else:
                pressed_keys.add(char.lower())
    except AttributeError:
        pass

def on_release(key):
    """Handle key release events."""
    global shift_pressed
    try:
        if key == keyboard.Key.shift or key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
            shift_pressed = False
        elif hasattr(key, 'char') and key.char:
            char = key.char
            # Check if it's a shifted number character
            if char in SHIFTED_TO_NUMBER:
                base_char = SHIFTED_TO_NUMBER[char]
                pressed_keys.discard(f'-{base_char}')
            else:
                pressed_keys.discard(char.lower())
        if key == keyboard.Key.esc:
            print("\nESC pressed. Exiting...")
            return False
    except AttributeError:
        pass

def compute_action_from_keyboard(current_obs):
    """
    Compute target action (absolute positions) based on keyboard input and current observation.
    Similar to how phone_to_so100 computes joint actions from teleop input.
    """
    # Start with current positions
    target_action = {}
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
    
    # Initialize target positions to current positions
    for joint_name in joint_names:
        target_action[f"{joint_name}.pos"] = current_obs.get(f"{joint_name}.pos", 0.0)
    
    # Initialize gripper to current width
    target_action["gripper.width"] = current_obs.get("gripper.width", 0.04)
    
    # Joint controls: number keys 1-7 for joints 1-7
    # Shift + number (or shifted chars like !@#) for negative direction
    joint_map = {
        '1': 'joint1',
        '2': 'joint2',
        '3': 'joint3',
        '4': 'joint4',
        '5': 'joint5',
        '6': 'joint6',
        '7': 'joint7',
    }
    
    # Apply deltas to current positions
    for key_char in pressed_keys:
        # Check for negative prefix (from shifted keys)
        is_negative = False
        if key_char.startswith('-'):
            is_negative = True
            key_char = key_char[1:]  # Remove '-' prefix
        # Also check if shift is currently pressed
        elif shift_pressed:
            is_negative = True
        
        if key_char in joint_map:
            joint_name = joint_map[key_char]
            current_pos = target_action[f"{joint_name}.pos"]
            if is_negative:
                target_action[f"{joint_name}.pos"] = current_pos - JOINT_STEP
            else:
                target_action[f"{joint_name}.pos"] = current_pos + JOINT_STEP
    
    # Gripper controls: 'g' to open, 'h' to close
    current_gripper = target_action["gripper.width"]
    if 'g' in pressed_keys:
        target_action["gripper.width"] = current_gripper + GRIPPER_STEP  # Open
    elif 'h' in pressed_keys:
        target_action["gripper.width"] = current_gripper - GRIPPER_STEP  # Close
    
    return target_action

def print_controls():
    """Print control instructions."""
    print("\n" + "=" * 60)
    print("Franka Keyboard Teleoperation")
    print("=" * 60)
    print("\nControls:")
    print("  Joints 1-7:    Press keys 1-7 (hold Shift for negative direction)")
    print("  Gripper Open:  Press G")
    print("  Gripper Close: Press H")
    print("  Exit:          Press ESC")
    print("\n" + "=" * 60 + "\n")

def main():
    """Main teleoperation loop."""
    server_address = "tcp://172.16.0.1:5555"  # Nook running frankz server
    
    print_controls()
    
    # Setup keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    try:
        # Create robot config - use joint_position mode like phone_to_so100 example
        config = FrankaRemoteConfig(
            server_address=server_address,
            control_mode="joint_position",  # Use position mode like so100 example
        )
        
        # Initialize robot
        robot = FrankaRemoteRobot(config)
        
        # Connect to robot
        print(f"Connecting to {config.server_address}...")
        robot.connect()
        print("✓ Connected!")
        
        print("\nStarting teleoperation loop. Use keyboard to control the robot.")
        print("Press ESC to exit.\n")
        
        if not robot.is_connected:
            raise ValueError("Robot is not connected!")
        
        # Main control loop - similar structure to phone_to_so100/teleoperate.py
        while True:
            t0 = time.perf_counter()
            
            # Get robot observation (like phone_to_so100 example)
            robot_obs = robot.get_observation()
            
            # Compute target action from keyboard input (like phone_to_so100 processes phone input)
            joint_action = compute_action_from_keyboard(robot_obs)
            
            # Send action to robot (exactly like phone_to_so100 example)
            _ = robot.send_action(joint_action)
            
            # Print status
            joint1_pos = robot_obs.get("joint1.pos", 0.0)
            joint1_target = joint_action.get("joint1.pos", 0.0)
            gripper_width = robot_obs.get("gripper.width", 0.0)
            gripper_target = joint_action.get("gripper.width", 0.0)
            print(f"\rjoint1: {joint1_pos:.3f} -> {joint1_target:.3f} | "
                  f"gripper: {gripper_width:.3f} -> {gripper_target:.3f}", 
                  end='', flush=True)
            
            # Maintain FPS (using busy_wait like phone_to_so100 would, but time.sleep is fine too)
            elapsed = time.perf_counter() - t0
            sleep_time = max(0, 1.0 / FPS - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        listener.stop()
        if 'robot' in locals():
            print("\nClosing connection...")
            robot.disconnect()
            print("✓ Disconnected")

if __name__ == "__main__":
    main()

