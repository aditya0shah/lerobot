#!/usr/bin/env python
"""Simple ZMQ connection test to diagnose connection issues."""

import zmq
import pickle
import sys

def test_zmq_connection(server_address="tcp://172.16.0.2:5555", timeout_ms=5000):
    """Test basic ZMQ connection to the server."""
    
    print("=" * 60)
    print("ZMQ Connection Diagnostic Test")
    print("=" * 60)
    print(f"Server address: {server_address}")
    print(f"Timeout: {timeout_ms}ms")
    print()
    
    try:
        # Create context and socket
        print("[1/4] Creating ZMQ context and socket...")
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        
        # Set timeout
        print(f"[2/4] Setting receive timeout to {timeout_ms}ms...")
        socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        
        # Try to connect
        print(f"[3/4] Connecting to {server_address}...")
        socket.connect(server_address)
        print("✓ Socket connected (this doesn't mean server is ready)")
        
        # Try to send a simple init message
        print("[4/4] Sending test 'init' command...")
        message = {
            "command": "init",
            "data": {
                "control_mode": "joint_delta",
                "dynamics_factor": 0.2,
            }
        }
        
        print(f"  Sending: {message}")
        socket.send(pickle.dumps(message))
        print("  ✓ Message sent, waiting for response...")
        
        try:
            response_bytes = socket.recv()
            response = pickle.loads(response_bytes)
            print(f"  ✓ Response received: {response}")
            
            if response.get("error"):
                print(f"  ⚠ Server returned error: {response['error']}")
            else:
                print("  ✓ Server responded successfully!")
                
        except zmq.Again:
            print(f"  ✗ Timeout: No response from server after {timeout_ms}ms")
            print("\nTroubleshooting:")
            print("  1. Is the server actually running and listening?")
            print("  2. Can you reach the server? Try: ping 172.16.0.2")
            print("  3. Check firewall: sudo ufw status (if using ufw)")
            print("  4. Try telnet: telnet 172.16.0.2 5555")
            print("  5. Verify server is using zmq.REP socket type")
            return False
        
        # Clean up
        socket.close()
        context.term()
        
        print("\n" + "=" * 60)
        print("Connection test completed!")
        print("=" * 60)
        return True
        
    except zmq.ZMQError as e:
        print(f"\n✗ ZMQ Error: {e}")
        print("\nTroubleshooting:")
        print("  1. Check if pyzmq is installed: pip install pyzmq")
        print("  2. Verify server address format: tcp://IP:PORT")
        print("  3. Check network connectivity")
        return False
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    server_address = sys.argv[1] if len(sys.argv) > 1 else "tcp://172.16.0.2:5555"
    timeout_ms = int(sys.argv[2]) if len(sys.argv) > 2 else 5000
    
    success = test_zmq_connection(server_address, timeout_ms)
    sys.exit(0 if success else 1)

