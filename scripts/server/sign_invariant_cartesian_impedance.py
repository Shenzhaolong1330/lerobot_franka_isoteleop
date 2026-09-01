"""Cartesian impedance policy with sign-invariant quaternion feedback."""

from __future__ import annotations

from typing import Dict

import torch
import torchcontrol as toco
from torchcontrol.transform import Rotation as R
from torchcontrol.utils.tensor_utils import diagonalize_gain, to_tensor


class SignInvariantCartesianSpacePDFast(toco.ControlModule):
    """Compute Cartesian feedback with per-axis error limits."""

    def __init__(
        self,
        Kp: torch.Tensor,
        Kd: torch.Tensor,
        selection_to_base: torch.Tensor,
        error_clip: torch.Tensor,
    ):
        super().__init__()
        Kp = diagonalize_gain(to_tensor(Kp))
        Kd = diagonalize_gain(to_tensor(Kd))
        assert Kp.shape == torch.Size([6, 6])
        assert Kd.shape == torch.Size([6, 6])
        assert selection_to_base.shape == torch.Size([3, 3])
        assert error_clip.shape == torch.Size([6])
        self.Kp = torch.nn.Parameter(Kp)
        self.Kd = torch.nn.Parameter(Kd)
        self.selection_to_base = torch.nn.Parameter(to_tensor(selection_to_base))
        self.error_clip = torch.nn.Parameter(to_tensor(error_clip))

    def forward(
        self,
        pos_current: torch.Tensor,
        quat_current: torch.Tensor,
        twist_current: torch.Tensor,
        pos_desired: torch.Tensor,
        quat_desired: torch.Tensor,
        twist_desired: torch.Tensor,
    ) -> torch.Tensor:
        pos_err = pos_desired - pos_current

        # Align equivalent quaternion signs before computing the error.
        if torch.dot(quat_current, quat_desired) < 0.0:
            quat_desired = -quat_desired
        quat_curr_inv = R.functional.invert_quaternion(quat_current)
        quat_err = R.functional.quaternion_multiply(quat_curr_inv, quat_desired)
        quat_err_n = R.functional.normalize_quaternion(quat_err)
        ori_err = R.functional.quat2matrix(quat_current) @ quat_err_n[0:3]

        # Apply gains and clips in the fixed control frame.
        base_to_selection = self.selection_to_base.T
        pose_err_selection = torch.cat(
            [base_to_selection @ pos_err, base_to_selection @ ori_err]
        )
        pose_err_selection = torch.minimum(
            torch.maximum(pose_err_selection, -self.error_clip),
            self.error_clip,
        )
        twist_err_selection = torch.cat(
            [
                base_to_selection @ twist_desired[:3]
                - base_to_selection @ twist_current[:3],
                base_to_selection @ twist_desired[3:]
                - base_to_selection @ twist_current[3:],
            ]
        )
        wrench_selection = (
            self.Kp @ pose_err_selection + self.Kd @ twist_err_selection
        )
        return torch.cat(
            [
                self.selection_to_base @ wrench_selection[:3],
                self.selection_to_base @ wrench_selection[3:],
            ]
        )


class SignInvariantCartesianImpedanceControl(toco.PolicyModule):
    """Apply Cartesian impedance torques with quaternion feedback."""

    def __init__(
        self,
        joint_pos_current: torch.Tensor,
        Kp: torch.Tensor,
        Kd: torch.Tensor,
        robot_model: torch.nn.Module,
        ignore_gravity: bool = True,
        selection_to_base: torch.Tensor = torch.eye(3),
        error_clip: torch.Tensor = torch.ones(6),
        target_filter_alpha: float = 0.005,
        nullspace_stiffness: float = 0.2,
        joint1_nullspace_stiffness: float = 100.0,
    ) -> None:
        super().__init__()
        self.robot_model = robot_model
        self.invdyn = toco.modules.feedforward.InverseDynamics(
            self.robot_model,
            ignore_gravity=ignore_gravity,
        )
        self.pose_pd = SignInvariantCartesianSpacePDFast(
            Kp,
            Kd,
            selection_to_base,
            error_clip,
        )

        joint_pos_current = to_tensor(joint_pos_current)
        ee_pos_current, ee_quat_current = self.robot_model.forward_kinematics(
            joint_pos_current
        )
        self.ee_pos_desired = torch.nn.Parameter(ee_pos_current)
        self.ee_quat_desired = torch.nn.Parameter(ee_quat_current)
        self.ee_pos_filtered = torch.nn.Parameter(ee_pos_current.clone())
        self.ee_quat_filtered = torch.nn.Parameter(ee_quat_current.clone())
        self.ee_vel_desired = torch.nn.Parameter(torch.zeros(3))
        self.ee_rvel_desired = torch.nn.Parameter(torch.zeros(3))
        self.target_filter_alpha = torch.nn.Parameter(
            torch.tensor(float(target_filter_alpha))
        )
        self.joint_pos_nullspace = torch.nn.Parameter(joint_pos_current.clone())

        # Set an independent nullspace gain for joint 1.
        nullspace_k = torch.full_like(joint_pos_current, float(nullspace_stiffness))
        nullspace_k[0] = float(nullspace_stiffness) * float(
            joint1_nullspace_stiffness
        )
        nullspace_d = torch.full_like(
            joint_pos_current,
            2.0 * float(nullspace_stiffness) ** 0.5,
        )
        nullspace_d[0] *= 2.0 * float(joint1_nullspace_stiffness) ** 0.5
        self.nullspace_k = torch.nn.Parameter(nullspace_k)
        self.nullspace_d = torch.nn.Parameter(nullspace_d)
        self.joint_identity = torch.nn.Parameter(
            torch.eye(joint_pos_current.numel())
        )

    def _update_filtered_target(self) -> None:
        alpha = self.target_filter_alpha
        filtered_position = (
            alpha * self.ee_pos_desired
            + (1.0 - alpha) * self.ee_pos_filtered
        )

        quaternion_target = self.ee_quat_desired
        if torch.dot(self.ee_quat_filtered, quaternion_target) < 0.0:
            quaternion_target = -quaternion_target
        dot = torch.clamp(
            torch.dot(self.ee_quat_filtered, quaternion_target),
            -1.0,
            1.0,
        )
        if dot > 0.9995:
            filtered_quaternion = (
                (1.0 - alpha) * self.ee_quat_filtered
                + alpha * quaternion_target
            )
        else:
            theta = torch.acos(dot)
            sin_theta = torch.sin(theta)
            filtered_quaternion = (
                torch.sin((1.0 - alpha) * theta) / sin_theta
            ) * self.ee_quat_filtered + (
                torch.sin(alpha * theta) / sin_theta
            ) * quaternion_target
        filtered_quaternion = R.functional.normalize_quaternion(
            filtered_quaternion
        )
        self.ee_pos_filtered.data.copy_(filtered_position)
        self.ee_quat_filtered.data.copy_(filtered_quaternion)

    def forward(
        self,
        state_dict: Dict[str, torch.Tensor],
    ) -> Dict[str, torch.Tensor]:
        joint_pos_current = state_dict["joint_positions"]
        joint_vel_current = state_dict["joint_velocities"]

        ee_pos_current, ee_quat_current = self.robot_model.forward_kinematics(
            joint_pos_current
        )
        jacobian = self.robot_model.compute_jacobian(joint_pos_current)
        ee_twist_current = jacobian @ joint_vel_current

        wrench_feedback = self.pose_pd(
            ee_pos_current,
            ee_quat_current,
            ee_twist_current,
            self.ee_pos_filtered,
            self.ee_quat_filtered,
            torch.cat([self.ee_vel_desired, self.ee_rvel_desired]),
        )
        torque_feedback = jacobian.T @ wrench_feedback
        nullspace_projector = (
            self.joint_identity
            - jacobian.T @ torch.pinverse(jacobian.T)
        )
        torque_nullspace = nullspace_projector @ (
            self.nullspace_k * (self.joint_pos_nullspace - joint_pos_current)
            - self.nullspace_d * joint_vel_current
        )
        torque_feedforward = self.invdyn(
            joint_pos_current,
            joint_vel_current,
            torch.zeros_like(joint_pos_current),
        )
        self._update_filtered_target()
        return {
            "joint_torques": (
                torque_feedback + torque_nullspace + torque_feedforward
            )
        }
