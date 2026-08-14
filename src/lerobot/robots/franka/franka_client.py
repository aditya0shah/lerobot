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

"""Minimal ZMQ client for a frankz-compatible Franka control server.

This is a clean-room transport implementation written from the documented wire protocol; it
does not vendor any third-party source. The server (which runs on the realtime control PC
next to the arm) is the counterpart described at https://github.com/memmelma/frankz.

Wire protocol
-------------
Transport is ZMQ ``REQ``/``REP`` in strict lockstep. Each message is a ``pickle``-encoded
dict:

* request:  ``{"command": <str>, "data": <dict | None>}``
* reply:    ``{"result": <any>}`` on success, or ``{"error": <str>}`` on failure.

Four commands are supported: ``init``, ``reset``, ``step`` and ``get_obs``.

Security
--------
The payloads are serialized with ``pickle``, so a malicious server could execute arbitrary
code on the client. Only ever connect to a server you control over a trusted, isolated link
(the Franka FCI subnet). Do not expose this port to an untrusted network.
"""

import logging

# The FCI transport is trusted and isolated; see module docstring for the security note.
import pickle  # nosec B403
from typing import Any

import numpy as np

from lerobot.utils.errors import DeviceNotConnectedError

from .constants import FRANKA_SERVER_CONTROL_MODE

logger = logging.getLogger(__name__)

_VALID_COMMANDS = frozenset({"init", "reset", "step", "get_obs"})


class FrankaClient:
    """Thin request/reply client for the Franka control server.

    All communication is synchronous: every method sends one request and blocks until the
    matching reply arrives or the receive timeout elapses. On any transport error the socket
    is torn down and recreated (a ``REQ`` socket cannot recover its send/recv alternation
    after a failed transaction) and a :class:`DeviceNotConnectedError` is raised.
    """

    def __init__(
        self,
        server_address: str,
        dynamics_factor: float = 0.2,
        home_joint_positions: tuple[float, ...] | None = None,
        connect_timeout_s: float = 5.0,
        request_timeout_s: float = 5.0,
        home_timeout_s: float = 30.0,
        server_control_mode_check: bool = True,
    ) -> None:
        try:
            import zmq
        except ImportError as e:
            raise ImportError(
                'pyzmq is required to use the Franka robot. Install it with `pip install -e ".[franka]"`.'
            ) from e

        self._zmq = zmq
        self.server_address = server_address
        self.dynamics_factor = dynamics_factor
        self.home_joint_positions = home_joint_positions
        self.request_timeout_s = request_timeout_s
        self.home_timeout_s = home_timeout_s
        self.server_control_mode_check = server_control_mode_check

        self._context = zmq.Context()
        self._socket = self._make_socket(request_timeout_s)
        self._socket.connect(server_address)
        logger.info(f"Connecting to Franka server at {server_address}")

        # The init handshake may need to wait for the server to spin up its control loop.
        self._init(timeout_s=connect_timeout_s)

    def _make_socket(self, timeout_s: float):
        socket = self._context.socket(self._zmq.REQ)
        socket.setsockopt(self._zmq.LINGER, 0)
        socket.setsockopt(self._zmq.RCVTIMEO, int(timeout_s * 1000))
        socket.setsockopt(self._zmq.SNDTIMEO, int(timeout_s * 1000))
        return socket

    def _reset_socket(self, timeout_s: float) -> None:
        """Recreate the REQ socket after a failed transaction and reconnect."""
        self._socket.close()
        self._socket = self._make_socket(timeout_s)
        self._socket.connect(self.server_address)

    def _request(self, command: str, data: dict | None = None, timeout_s: float | None = None) -> Any:
        if command not in _VALID_COMMANDS:
            raise ValueError(f"Invalid command '{command}'. Must be one of {sorted(_VALID_COMMANDS)}.")

        timeout_s = self.request_timeout_s if timeout_s is None else timeout_s
        self._socket.setsockopt(self._zmq.RCVTIMEO, int(timeout_s * 1000))

        message = {"command": command, "data": data}
        try:
            self._socket.send(pickle.dumps(message))
            # See module docstring: trusted FCI link only.
            response = pickle.loads(self._socket.recv())  # nosec B301
        except self._zmq.Again as e:
            self._reset_socket(self.request_timeout_s)
            raise DeviceNotConnectedError(
                f"Timed out after {timeout_s:.1f}s waiting for the Franka server at {self.server_address} "
                f"on command '{command}'. Is the server running and reachable?"
            ) from e
        except self._zmq.ZMQError as e:
            self._reset_socket(self.request_timeout_s)
            raise DeviceNotConnectedError(
                f"ZMQ transport error talking to the Franka server on command '{command}': {e}"
            ) from e

        if response.get("error"):
            raise RuntimeError(f"Franka server error on '{command}': {response['error']}")

        return response.get("result")

    def _init(self, timeout_s: float) -> None:
        data = {
            "control_mode": FRANKA_SERVER_CONTROL_MODE,
            "dynamics_factor": self.dynamics_factor,
        }
        if self.home_joint_positions is not None:
            data["reset_qpos"] = list(self.home_joint_positions)

        result = self._request("init", data, timeout_s=timeout_s)

        if self.server_control_mode_check:
            effective_mode = result.get("control_mode") if isinstance(result, dict) else None
            if effective_mode is None:
                raise RuntimeError(
                    "The Franka server did not report its effective control mode, so this client cannot "
                    "verify it is running in the safe 'joint_position' mode. The frankz server SILENTLY "
                    "ignores the requested control mode if it was already initialized by an earlier "
                    "client, which can turn absolute joint targets into dangerous deltas.\n"
                    "Fix: restart the server before connecting, and patch its init handler to echo the "
                    'mode, e.g. `return {"result": {"status": "initialized", '
                    '"control_mode": self.control_mode}}`.\n'
                    "To bypass this check on a server you trust, set "
                    "`--robot.server_control_mode_check=false`."
                )
            if effective_mode != FRANKA_SERVER_CONTROL_MODE:
                raise RuntimeError(
                    f"The Franka server is running in '{effective_mode}' mode, not "
                    f"'{FRANKA_SERVER_CONTROL_MODE}'. Restart the server before connecting."
                )

    def reset(self) -> None:
        """Command the server to drive the arm to its home configuration (blocking)."""
        self._request("reset", timeout_s=self.home_timeout_s)

    def get_obs(self) -> dict[str, np.ndarray]:
        """Return the current robot observation.

        Returns:
            A dict with ``qpos`` (7,), ``qvel`` (7,), ``ee_pose`` (7,) and ``gripper_state``
            (1,) as numpy arrays. ``gripper_state`` is the normalized opening in [0, 1].
        """
        return self._request("get_obs")

    def step(self, action: np.ndarray, blocking: bool = False) -> None:
        """Send one action to the server.

        Args:
            action: A length-7 (joints only) or length-8 (joints + binary gripper) array of
                absolute joint positions in radians. A length-7 action leaves the gripper
                untouched.
            blocking: If True, the server waits for the motion to complete before replying.

        Returns:
            None. The server acknowledges the step but does not return an observation.
        """
        action = np.asarray(action, dtype=np.float64)
        self._request("step", {"action": action, "blocking": blocking})

    def close(self) -> None:
        self._socket.close()
        self._context.term()

    def __enter__(self) -> "FrankaClient":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
