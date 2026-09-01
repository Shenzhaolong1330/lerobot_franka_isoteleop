"""Local ZeroRPC bridge between this project and Polymetis gRPC services."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import numpy as np
import torch
import yaml
import zerorpc
from polymetis import GripperInterface, RobotInterface
from scipy.spatial.transform import Rotation

try:
    from scripts.server.sign_invariant_cartesian_impedance import (
        SignInvariantCartesianImpedanceControl,
    )
except ModuleNotFoundError:
    # The runtime executes this file directly in the Polymetis environment.
    from sign_invariant_cartesian_impedance import (
        SignInvariantCartesianImpedanceControl,
    )


logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)


class FrankaInterfaceServer:
    def __init__(
        self,
        arm_host: str,
        arm_port: int,
        gripper_type: str,
        hand_host: str,
        hand_port: int,
        selection_to_base: np.ndarray,
        error_clip: np.ndarray,
        target_filter_alpha: float,
        nullspace_stiffness: float,
        joint1_nullspace_stiffness: float,
    ):
        self.robot = RobotInterface(
            ip_address=arm_host,
            port=int(arm_port),
            enforce_version=False,
        )
        self.gripper_type = gripper_type
        self.hand_host = hand_host
        self.hand_port = int(hand_port)
        self.gripper = None
        self.selection_to_base = torch.tensor(
            selection_to_base, dtype=torch.float32
        )
        self.error_clip = torch.tensor(error_clip, dtype=torch.float32)
        self.target_filter_alpha = float(target_filter_alpha)
        self.nullspace_stiffness = float(nullspace_stiffness)
        self.joint1_nullspace_stiffness = float(joint1_nullspace_stiffness)
        if not 0.0 < self.target_filter_alpha <= 1.0:
            raise ValueError("target_filter_alpha must be in (0, 1].")
        if self.nullspace_stiffness < 0.0:
            raise ValueError("nullspace_stiffness must be non-negative.")
        if self.joint1_nullspace_stiffness < 0.0:
            raise ValueError("joint1_nullspace_stiffness must be non-negative.")
        logger.info("Connected to Polymetis arm at %s:%d", arm_host, arm_port)
        logger.info(
            "Polymetis EE model: %s",
            self.robot.metadata.ee_link_name,
        )

    def _require_franka_hand(self):
        if self.gripper_type != "franka_hand":
            raise RuntimeError(
                "The selected PGI gripper is controlled directly over serial; "
                "Franka Hand RPC is unavailable."
            )
        if self.gripper is None:
            raise RuntimeError("Franka Hand is not initialized.")
        return self.gripper

    def gripper_initialize(self) -> None:
        if self.gripper_type != "franka_hand":
            self._require_franka_hand()
        if self.gripper is None:
            self.gripper = GripperInterface(
                ip_address=self.hand_host,
                port=self.hand_port,
            )
            logger.info(
                "Connected to Polymetis Franka Hand at %s:%d",
                self.hand_host,
                self.hand_port,
            )

    def gripper_goto(
        self,
        width: float,
        speed: float,
        force: float,
        epsilon_inner: float = -1.0,
        epsilon_outer: float = -1.0,
        blocking: bool = True,
    ) -> None:
        self._require_franka_hand().goto(
            width=width,
            speed=speed,
            force=force,
            epsilon_inner=epsilon_inner,
            epsilon_outer=epsilon_outer,
            blocking=blocking,
        )

    def gripper_get_state(self) -> dict:
        state = self._require_franka_hand().get_state()
        return {
            "width": state.width,
            "is_moving": state.is_moving,
            "is_grasped": state.is_grasped,
            "prev_command_successful": state.prev_command_successful,
            "error_code": state.error_code,
        }

    def robot_get_joint_positions(self) -> list:
        return self.robot.get_joint_positions().numpy().tolist()

    def robot_get_joint_velocities(self) -> list:
        return self.robot.get_joint_velocities().numpy().tolist()

    def robot_get_ee_pose(self) -> list:
        position, quaternion_xyzw = self.robot.get_ee_pose()
        rotvec = Rotation.from_quat(quaternion_xyzw.numpy()).as_rotvec()
        return np.concatenate([position.numpy(), rotvec]).tolist()

    def robot_get_ee_state(self) -> dict[str, list]:
        """Return TCP pose, speed, and external wrench."""
        state = self.robot.get_robot_state()
        joint_positions = torch.tensor(state.joint_positions, dtype=torch.float32)
        joint_velocities = torch.tensor(state.joint_velocities, dtype=torch.float32)
        position, quaternion_xyzw = self.robot.robot_model.forward_kinematics(
            joint_positions
        )
        jacobian = self.robot.robot_model.compute_jacobian(joint_positions)
        speed = jacobian @ joint_velocities
        external_joint_torque = np.asarray(
            state.motor_torques_external,
            dtype=np.float64,
        )
        wrench = np.linalg.lstsq(
            jacobian.numpy().T,
            external_joint_torque,
            rcond=1e-4,
        )[0]
        rotvec = Rotation.from_quat(quaternion_xyzw.numpy()).as_rotvec()
        return {
            "pose": np.concatenate([position.numpy(), rotvec]).tolist(),
            "speed": speed.numpy().tolist(),
            "wrench": wrench.tolist(),
        }

    def robot_move_to_ee_pose(
        self,
        pose: list,
        time_to_go: float | None = None,
        delta: bool = False,
        Kx: list | None = None,
        Kxd: list | None = None,
        op_space_interp: bool = True,
        blocking: bool = True,
    ) -> bool:
        position = torch.tensor(pose[:3], dtype=torch.float32)
        quaternion = torch.tensor(
            Rotation.from_rotvec(pose[3:]).as_quat(), dtype=torch.float32
        )
        result = self.robot.move_to_ee_pose(
            position=position,
            orientation=quaternion,
            time_to_go=time_to_go,
            delta=delta,
            Kx=torch.tensor(Kx, dtype=torch.float32) if Kx is not None else None,
            Kxd=torch.tensor(Kxd, dtype=torch.float32) if Kxd is not None else None,
            op_space_interp=op_space_interp,
            blocking=blocking,
        )
        return result is None if not blocking else bool(result)

    def robot_start_cartesian_impedance_control(
        self,
        Kx: list | None = None,
        Kxd: list | None = None,
    ) -> None:
        joint_pos_current = self.robot.get_joint_positions()
        stiffness = (
            self.robot.Kx_default
            if Kx is None
            else torch.tensor(Kx, dtype=torch.float32)
        )
        damping = (
            self.robot.Kxd_default
            if Kxd is None
            else torch.tensor(Kxd, dtype=torch.float32)
        )
        policy = SignInvariantCartesianImpedanceControl(
            joint_pos_current=joint_pos_current,
            Kp=stiffness,
            Kd=damping,
            robot_model=self.robot.robot_model,
            ignore_gravity=self.robot.use_grav_comp,
            selection_to_base=self.selection_to_base,
            error_clip=self.error_clip,
            target_filter_alpha=self.target_filter_alpha,
            nullspace_stiffness=self.nullspace_stiffness,
            joint1_nullspace_stiffness=self.joint1_nullspace_stiffness,
        )
        self.robot.send_torch_policy(torch_policy=policy, blocking=False)
        logger.info("Started Cartesian impedance policy.")

    def robot_update_desired_ee_pose(
        self,
        pose: list,
        action_delta: list | None = None,
    ) -> int:
        target_position = np.asarray(pose[:3], dtype=np.float32)
        target_quaternion = Rotation.from_rotvec(pose[3:]).as_quat().astype(
            np.float32
        )
        update_offset = self.robot.update_current_policy(
            {
                "ee_pos_desired": torch.from_numpy(target_position.copy()),
                "ee_quat_desired": torch.from_numpy(target_quaternion.copy()),
            }
        )
        return int(update_offset)

    def robot_terminate_current_policy(self) -> None:
        if self.robot.is_running_policy():
            self.robot.terminate_current_policy(return_log=False)


def _error_clip(values: list, name: str) -> np.ndarray:
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError(f"cartesian_impedance.{name} must contain six values.")
    result = np.asarray(
        [np.inf if value is None else float(value) for value in values],
        dtype=np.float32,
    )
    finite = result[np.isfinite(result)]
    if np.any(finite <= 0):
        raise ValueError(f"Finite values in {name} must be positive.")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    with args.config.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    services = config["services"]
    arm = services["arm"]
    hand = services["franka_hand"]
    zerorpc_config = services["zerorpc"]
    gripper_type = str(
        config["record"]["robot"].get("gripper", {}).get("type", "pgi")
    ).lower()
    robot_config = config["record"]["robot"]
    impedance_config = robot_config["cartesian_impedance"]
    teleop_config = config["record"]["teleop"]
    selection_to_base = Rotation.from_euler(
        "xyz",
        teleop_config.get("control_frame_euler_deg", [0.0, 0.0, 0.0]),
        degrees=True,
    ).as_matrix()
    error_clip = _error_clip(impedance_config["error_clip"], "error_clip")
    rpc = FrankaInterfaceServer(
        arm_host=arm["host"],
        arm_port=arm["port"],
        gripper_type=gripper_type,
        hand_host=hand["host"],
        hand_port=hand["port"],
        selection_to_base=selection_to_base,
        error_clip=error_clip,
        target_filter_alpha=float(impedance_config.get("target_filter_alpha", 0.005)),
        nullspace_stiffness=float(impedance_config.get("nullspace_stiffness", 0.2)),
        joint1_nullspace_stiffness=float(
            impedance_config.get("joint1_nullspace_stiffness", 100.0)
        ),
    )
    endpoint = f"tcp://{zerorpc_config['bind_host']}:{int(zerorpc_config['port'])}"
    server = zerorpc.Server(rpc)
    server.bind(endpoint)
    logger.info("ZeroRPC bridge listening at %s", endpoint)
    try:
        server.run()
    finally:
        try:
            rpc.robot_terminate_current_policy()
        except Exception:
            logger.warning(
                "Failed to terminate the Cartesian policy during server shutdown.",
                exc_info=True,
            )


if __name__ == "__main__":
    main()
