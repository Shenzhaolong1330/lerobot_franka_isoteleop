"""ZeroRPC client for the local Franka bridge."""

import logging

import numpy as np
import zerorpc


logger = logging.getLogger(__name__)


class FrankaInterfaceClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 4242):
        self.endpoint = f"tcp://{host}:{int(port)}"
        self.server = zerorpc.Client(timeout=120, heartbeat=20)
        self.server.connect(self.endpoint)
        logger.info("Connected ZeroRPC client to %s", self.endpoint)

    def gripper_initialize(self) -> None:
        self.server.gripper_initialize()

    def gripper_goto(
        self,
        width: float,
        speed: float,
        force: float,
        epsilon_inner: float = -1.0,
        epsilon_outer: float = -1.0,
        blocking: bool = True,
    ) -> None:
        self.server.gripper_goto(
            float(width),
            float(speed),
            float(force),
            float(epsilon_inner),
            float(epsilon_outer),
            bool(blocking),
        )

    def gripper_get_state(self) -> dict:
        return self.server.gripper_get_state()

    def robot_get_joint_positions(self) -> np.ndarray:
        return np.asarray(self.server.robot_get_joint_positions(), dtype=float)

    def robot_get_joint_velocities(self) -> np.ndarray:
        return np.asarray(self.server.robot_get_joint_velocities(), dtype=float)

    def robot_get_ee_pose(self) -> np.ndarray:
        return np.asarray(self.server.robot_get_ee_pose(), dtype=float)

    def robot_get_ee_state(self) -> dict[str, np.ndarray]:
        state = self.server.robot_get_ee_state()
        return {
            "pose": np.asarray(state["pose"], dtype=float),
            "speed": np.asarray(state["speed"], dtype=float),
            "wrench": np.asarray(state["wrench"], dtype=float),
        }

    def robot_move_to_ee_pose(
        self,
        pose: np.ndarray,
        time_to_go: float | None = None,
        delta: bool = False,
        Kx: np.ndarray | None = None,
        Kxd: np.ndarray | None = None,
        op_space_interp: bool = True,
        blocking: bool = True,
    ) -> bool:
        return bool(self.server.robot_move_to_ee_pose(
            np.asarray(pose, dtype=float).tolist(),
            time_to_go,
            bool(delta),
            np.asarray(Kx, dtype=float).tolist() if Kx is not None else None,
            np.asarray(Kxd, dtype=float).tolist() if Kxd is not None else None,
            bool(op_space_interp),
            bool(blocking),
        ))

    def robot_start_cartesian_impedance_control(
        self,
        Kx: np.ndarray | None = None,
        Kxd: np.ndarray | None = None,
    ) -> None:
        self.server.robot_start_cartesian_impedance_control(
            np.asarray(Kx, dtype=float).tolist() if Kx is not None else None,
            np.asarray(Kxd, dtype=float).tolist() if Kxd is not None else None,
        )

    def robot_update_desired_ee_pose(
        self,
        pose: np.ndarray,
    ) -> int:
        return int(
            self.server.robot_update_desired_ee_pose(
                np.asarray(pose, dtype=float).tolist(),
            )
        )

    def robot_terminate_current_policy(self) -> None:
        self.server.robot_terminate_current_policy()

    def close(self) -> None:
        self.server.close()
