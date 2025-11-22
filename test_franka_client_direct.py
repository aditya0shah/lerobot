#!/usr/bin/env python
"""Direct test of FrankaClient - simpler test that doesn't require Robot wrapper."""

import sys
import importlib.util
from pathlib import Path

# Check for required dependencies
try:
    import numpy as np
except ImportError:
    print("Error: numpy is not installed. Install it with: pip install numpy")
    sys.exit(1)

try:
    import zmq
except ImportError:
    print("Error: pyzmq is not installed. Install it with: pip install pyzmq")
    print("Or if using conda: conda install -c conda-forge pyzmq")
    sys.exit(1)

# Import directly from the file to avoid package dependencies
franka_client_path = Path(__file__).parent / "src" / "lerobot" / "robots" / "franka" / "franka_client.py"
spec = importlib.util.spec_from_file_location("franka_client", franka_client_path)
franka_client_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(franka_client_module)
FrankaClient = franka_client_module.FrankaClient


def test_franka_client_direct():
    """Test FrankaClient directly without Robot wrapper."""
    
    print("=" * 60)
    print("Testing FrankaClient Direct Connection")
    print("=" * 60)
    
    # Update server address if needed
    server_address = "tcp://172.16.0.1:5555"   # Nook running frankz server
    try:
        # Test 1: Create client and connect
        print("\n[1/4] Creating client and connecting...")
        print(f"  Attempting to connect to {server_address}...")
        client = FrankaClient(
            server_address=server_address,
            control_mode="joint_delta",
            dynamics_factor=0.2,
            control_hz=10,
        )
        print(f"✓ Connected to server at {server_address}")
        
        # Test 2: Reset robot
        print("\n[2/4] Resetting robot...")
        client.reset()
        print("✓ Robot reset")
        
        # Test 3: Get observation
        print("\n[3/4] Getting observation...")
        obs = client.get_obs()
        print("✓ Observation received:")
        print(f"  qpos (joint positions): {obs['qpos']}")
        print(f"  qvel (joint velocities): {obs['qvel']}")
        if 'ee_pose' in obs:
            print(f"  ee_pose (end-effector pose): {obs['ee_pose']}")
        if 'gripper_state' in obs:
            print(f"  gripper_state: {obs['gripper_state']}")
        
        # Test 4: Send a small test action
        print("\n[4/4] Sending test action...")
        # Create a small delta action (8 values: 7 joints + 1 gripper)
        action = np.zeros(8)  # Keep gripper open
        
        print(f"  Sending action: {action}")
        obs_after = client.step(action, blocking=True)
        print(f"✓ Action sent. New qpos: {obs_after['qpos']}")
        
        print("\n" + "=" * 60)
        print("All tests passed! ✓")
        print("=" * 60)
        
    except TimeoutError as e:
        print(f"\n✗ Connection timeout: {e}")
        print("\nTroubleshooting:")
        print(f"  1. Is the Franka server running at {server_address}?")
        print("  2. Can you ping/connect to the server?")
        print("  3. Check firewall/network settings")
        print("  4. Verify the server is listening on port 5555")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error during testing: {e}")
        import traceback
        traceback.print_exc()
        print("\nTroubleshooting:")
        print(f"  1. Is the Franka server running at {server_address}?")
        print("  2. Can you ping/connect to the server?")
        print("  3. Check firewall/network settings")
        raise
    
    finally:
        # Always close client
        if 'client' in locals():
            print("\nClosing client...")
            client.close()
            print("✓ Client closed")


if __name__ == "__main__":
    test_franka_client_direct()

