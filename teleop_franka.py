#!/usr/bin/env python
"""Simple keyboard teleoperation for Franka robot."""

import sys
import time
import importlib.util
from pathlib import Path

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

# Import FrankaClient
franka_client_path = Path(__file__).parent / "src" / "lerobot" / "robots" / "franka" / "franka_client.py"
spec = importlib.util.spec_from_file_location("franka_client", franka_client_path)
franka_client_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(franka_client_module)
FrankaClient = franka_client_module.FrankaClient

# Control parameters
JOINT_STEP = 0.2  # radians per keypress
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

def update_action():
    """Update action based on pressed keys."""
    # Reset action
    action = np.zeros(8)
    
    # Joint controls: number keys 1-7 for joints 1-7
    # Shift + number (or shifted chars like !@#) for negative direction
    joint_map = {
        '1': 0,  # Joint 1
        '2': 1,  # Joint 2
        '3': 2,  # Joint 3
        '4': 3,  # Joint 4
        '5': 4,  # Joint 5
        '6': 5,  # Joint 6
        '7': 6,  # Joint 7
    }
    
    # Check for joint movements
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
            joint_idx = joint_map[key_char]
            if is_negative:
                action[joint_idx] = -JOINT_STEP
            else:
                action[joint_idx] = JOINT_STEP
    
    # Gripper controls: 'g' to open, 'h' to close
    if 'g' in pressed_keys:
        action[7] = GRIPPER_STEP  # Open
    elif 'h' in pressed_keys:
        action[7] = -GRIPPER_STEP  # Close
    
    return action

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
        # Connect to robot
        print(f"Connecting to {server_address}...")
        client = FrankaClient(
            server_address=server_address,
            control_mode="joint_delta",
            dynamics_factor=0.2,
            control_hz=FPS,
        )
        print("✓ Connected!")
        
        # Reset robot
        print("Resetting robot...")
        client.reset()
        print("✓ Robot reset")
        
        print("\nStarting teleoperation loop. Use keyboard to control the robot.")
        print("Press ESC to exit.\n")
        
        # Main control loop
        while True:
            t0 = time.perf_counter()
            
            # Update action based on keyboard input
            action = update_action()
            
            # Send action if non-zero
            if np.any(action != 0):
                obs = client.step(action, blocking=True)
                # Print current joint positions
                print(f"\rAction: {action[:7]} | Gripper: {action[7]:.3f} | "
                      f"qpos[0]: {obs['qpos'][0]:.3f}", end='', flush=True)
            else:
                # Still send zero action to maintain connection
                client.step(np.zeros(8), blocking=True)
            
            # Maintain FPS
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
        if 'client' in locals():
            print("\nClosing connection...")
            client.close()
            print("✓ Disconnected")

if __name__ == "__main__":
    main()

