import os
import zmq
import time
import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R

from pathlib import Path
import os

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

import mink

class MinkIK:
    """
    Inverse kinematics helper for MuJoCo + Mink on a Franka Panda model.

    Usage:
        ik = MinkIK(xml_path=..., ee_site_name="attachment_site")
        q = ik.solve(position, quaternion, dt=0.01)

    The pose must be in MuJoCo's convention: position (x, y, z) in meters and
    quaternion (w, x, y, z).
    """

    def __init__(
        self,
        xml_path: str | Path,
        ee_site_name: str = "attachment_site",
        target_mocap_name: str = "target",
        position_threshold: float = 1e-4,
        orientation_threshold: float = 1e-4,
        position_cost: float = 1.0,
        orientation_cost: float = 1.0,
        posture_cost: float = 1e-2,
        solver: str = "quadprog",
        max_iterations: int = 20,
        lm_damping: float = 1.0,
        dt: float = 0.002,
        max_joint_velocity: float = 2.5,  # rad/s
    ) -> None:

        self.xml_path = Path(xml_path)

        self.position_threshold = position_threshold
        self.orientation_threshold = orientation_threshold

        self.position_cost = position_cost
        self.orientation_cost = orientation_cost
        self.posture_cost = posture_cost

        self.solver = solver
        self.max_iterations = max_iterations
        self.lm_damping = lm_damping
        self.dt = dt

        # Load model and data
        self.model = mujoco.MjModel.from_xml_path(self.xml_path.as_posix())
        self.data = mujoco.MjData(self.model)

        self.ee_site_name = ee_site_name
        self.ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.ee_site_name)
        self.target_mocap_name = target_mocap_name

        # Configuration and tasks
        self.configuration = mink.Configuration(self.model)
        self.end_effector_task = mink.FrameTask(
            frame_name=self.ee_site_name,
            frame_type="site",
            position_cost=self.position_cost,
            orientation_cost=self.orientation_cost,
            lm_damping=self.lm_damping,
        )
        self.posture_task = mink.PostureTask(model=self.model, cost=self.posture_cost)
        self.tasks = [self.end_effector_task, self.posture_task]

        # Setup limits with velocity constraints
        self.limits = [
            mink.ConfigurationLimit(model=self.model),
            mink.VelocityLimit(
                model=self.model,
                velocities={f"joint{i}": max_joint_velocity for i in range(1, 8)}
            ),
        ]

        # Align mocap target with current end-effector pose
        mink.move_mocap_to_frame(
            self.model,
            self.data,
            self.target_mocap_name,
            self.ee_site_name,
            "site",
        )

    def _converge_ik(
        self, configuration, tasks, dt, solver, pos_threshold, ori_threshold, max_iters
    ):
        """
        Runs up to 'max_iters' of IK steps. Returns True if position and orientation
        are below thresholds, otherwise False.
        """
        for _ in range(max_iters):
            vel = mink.solve_ik(configuration, tasks, dt, solver, 1e-3, limits=self.limits)
            configuration.integrate_inplace(vel, dt)

            # Only checking the first FrameTask here (end_effector_task).
            # If you want to check multiple tasks, sum or combine their errors.
            err = tasks[0].compute_error(configuration)
            pos_achieved = np.linalg.norm(err[:3]) <= pos_threshold
            ori_achieved = np.linalg.norm(err[3:]) <= ori_threshold

            if pos_achieved and ori_achieved:
                return True
        return False

    def compute_fk(self, qpos: np.ndarray) -> np.ndarray:
        """
        Compute forward kinematics for the given joint positions.
        """
        # Add gripper dim
        qpos = np.concatenate([qpos[:7], [0.04, 0.04]])

        self.data.qpos = qpos
        mujoco.mj_forward(self.model, self.data)
        ee_pos = self.data.site_xpos[self.ee_site_id].astype(np.float32)
        ee_quat = R.from_matrix(self.data.site_xmat[self.ee_site_id].reshape(3, 3)).as_quat(scalar_first=True)
        return np.concatenate([ee_pos, ee_quat])
    
    def compute_ik(
        self,
        position: np.ndarray,
        quaternion: np.ndarray,
        q_init: np.ndarray,
    ) -> np.ndarray:
        """
        Solve IK to reach the provided end-effector pose.

        Args:
            position: (3,) world position in meters (MuJoCo convention)
            quaternion: (4,) world orientation quaternion [w, x, y, z]
            q_init: optional initial joint configuration to start from

        Returns:
            Joint positions as a NumPy array (same ordering/length as configuration.q)
        """

        # Add gripper dim
        q_init = np.concatenate([q_init[:7], [0.04, 0.04]])

        # # Keep posture task centered at the provided initial configuration
        self.configuration.update(q_init)
        self.posture_task.set_target_from_configuration(self.configuration)

        # Update mocap body to desired target pose (MuJoCo convention)
        self.data.mocap_pos[0] = np.asarray(position, dtype=float)
        self.data.mocap_quat[0] = np.asarray(quaternion, dtype=float)
        # mujoco.mj_forward(self.model, self.data)

        # Set the end-effector task target from the mocap body transform
        T_wt = mink.SE3.from_mocap_name(self.model, self.data, self.target_mocap_name)
        self.end_effector_task.set_target(T_wt)

        # Run IK
        success = self._converge_ik(
            configuration=self.configuration,
            tasks=self.tasks,
            dt=self.dt,
            solver=self.solver,
            pos_threshold=self.position_threshold,
            ori_threshold=self.orientation_threshold,
            max_iters=self.max_iterations,
        )
        # if not success:
        #     import IPython; IPython.embed()

        # assert success, "IK failed to converge"
        if not success:
            return None

        return self.configuration.q.copy()[:7]

class FrankaClient:
    def __init__(
        self,
        server_address="tcp://localhost:5555",
        control_mode="joint_position",
        dynamics_factor=0.2,
        control_hz=10,
    ):

        self.server_address = server_address

        self.control_mode = control_mode
        assert control_mode in ["joint_position", "joint_delta", "cartesian_position", "cartesian_delta"], "Invalid control mode"
        
        self.dynamics_factor = dynamics_factor
        assert dynamics_factor >= 0 and dynamics_factor <= 1, "Invalid dynamics factor"
        
        self.control_hz = control_hz
        assert control_hz > 0, "Invalid control hz"

        if control_mode == "cartesian_position" or control_mode == "cartesian_delta":
            relative_max_joint_delta = 0.2 # from DROID
            self.ik = MinkIK(
                xml_path=os.path.join("/home/tactile/robot_env/third_party/mink", "examples/franka_emika_panda/mjx_scene.xml"),
                dt=1/control_hz,
                max_iterations=20,
                position_threshold=np.inf,
                orientation_threshold=np.inf,
                max_joint_velocity=relative_max_joint_delta * control_hz
            )
            # run joint_position under the hood
            control_mode = "joint_position"
        
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(server_address)
        
        self._send_command("init", {
            "control_mode": control_mode,
            "dynamics_factor": dynamics_factor,
        })
        
        print(f"Connected to Franka server at {server_address}")
    
    def _send_command(self, command, data=None):
        
        assert command in ["step", "reset", "get_obs", "init"], "Invalid command"
        
        message = {
            "command": command,
            "data": data,
        }
    
        self.socket.send(pickle.dumps(message))
        response = pickle.loads(self.socket.recv())
        
        if response.get("error"):
            raise RuntimeError(f"Server error: {response['error']}")
        
        return response.get("result")
    
    def step(self, action, blocking=False):
        
        if self.control_mode == "cartesian_position":
            obs = self.get_obs()

            # mink/mujoco expects [w, x, y, z] convention
            action[3:7] = R.from_quat(action[3:7], scalar_first=False).as_quat(scalar_first=True)
            
            # compute target joint positions
            action = self.ik.compute_ik(action[:3], action[3:7], obs["qpos"])
            assert action is not None, "IK failed to converge"
        
        elif self.control_mode == "cartesian_delta":
            obs = self.get_obs()

            curr_pose = obs["ee_pose"]
            new_pose = curr_pose.copy()
            
            # add pos delta
            new_pose[:3] = curr_pose[:3] + action[:3]
            # add ori delta
            curr_ori = R.from_quat(curr_pose[3:7], scalar_first=False)
            delta_ori = R.from_quat(action[3:7], scalar_first=False)
            new_pose[3:7] = (delta_ori * curr_ori).as_quat(scalar_first=False)

            # mink/mujoco expects [w, x, y, z] convention
            new_pose[3:7] = R.from_quat(new_pose[3:7], scalar_first=False).as_quat(scalar_first=True)

            # compute target joint positions
            action = self.ik.compute_ik(new_pose[:3], new_pose[3:7], obs["qpos"])
            assert action is not None, "IK failed to converge"
        
        start_time = time.perf_counter()
        
        # send action to server
        obs = self._send_command("step", {
            "action": action,
            "blocking": blocking,
        })

        # ensure control loop runs at target hz
        end_time = time.perf_counter()
        sleep_time = 1.0 / self.control_hz - (end_time - start_time)
        if sleep_time < 0:
            print(f"Warning: Control loop running slower than target {self.control_hz}Hz (behind by {sleep_time:.4f}s)")
        time.sleep(max(0, sleep_time))

        return obs
    
    def reset(self):
        return self._send_command("reset")
    
    def get_obs(self):
        """
        Get the current observation from the robot.
        
        Returns:
            dict containing:
                - qpos: joint positions (7,)
                - qvel: joint velocities (7,)
                - ee_pose: end-effector pose as [x, y, z, qx, qy, qz, qw] (7,)
                - gripper_state: gripper state (1,)
        """
        return self._send_command("get_obs")
    
    def close(self):
        self.socket.close()
        self.context.term()
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


if __name__ == "__main__":
    from src.realsense import gather_realsense_cameras
    cameras = gather_realsense_cameras(rgb=True, depth=False, ir=False, high_res_rgb=False)

    img = cameras[0].read_camera()

    import matplotlib.pyplot as plt
    plt.imsave("rgb.png", img["rgb"])
    plt.show()

    with FrankaClient(
        server_address="tcp://localhost:5555",
        control_mode="joint_delta",
        dynamics_factor=0.1
    ) as client:
        # Reset robot
        obs = client.reset()
        print("Initial observation:", obs)
        
        # Get observation
        obs = client.get_obs()
        print("Current qpos:", obs["qpos"])
        
        # Send a small delta action
        action = np.zeros(8)
        action[0] = 0.1  # Small delta on first joint
        action[7] = 1.0   # Keep gripper open
        
        obs = client.step(action, blocking=True)
        print("After step:", obs["qpos"])
