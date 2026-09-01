import unittest
from unittest.mock import MagicMock

import numpy as np
from scipy.spatial.transform import Rotation as R

from lerobot_robot_franka import Franka, FrankaConfig


ACTION_KEYS = ("delta_x", "delta_y", "delta_z", "delta_rx", "delta_ry", "delta_rz")


def make_action(values: list[float]) -> dict[str, float]:
    return dict(zip(ACTION_KEYS, values, strict=True))


class FrankaPoseHoldTest(unittest.TestCase):
    def make_robot(
        self,
        reference_frame: str = "base",
        select_vector: list[float] | None = None,
        control_frame_euler_deg: list[float] | None = None,
    ) -> Franka:
        return Franka(
            FrankaConfig(
                use_gripper=False,
                reference_frame=reference_frame,
                translation_axis_deadband=0.0005,
                rotation_axis_deadband=0.0005,
                select_vector=(select_vector if select_vector is not None else [1.0] * 6),
                control_frame_euler_deg=(
                    control_frame_euler_deg
                    if control_frame_euler_deg is not None
                    else [0.0, 0.0, 0.0]
                ),
            )
        )

    def test_tcp_projection_preserves_allowed_base_rz(self) -> None:
        robot = self.make_robot(
            reference_frame="tcp",
            select_vector=[1, 1, 1, 0, 0, 1],
            control_frame_euler_deg=[0, 0, 135],
        )
        current_rotation = R.from_euler("xyz", [35, -20, 70], degrees=True)
        delta_rotation_base = R.from_rotvec([0.0, 0.0, 0.01])
        delta_rotation_tcp = current_rotation.inv() * delta_rotation_base * current_rotation
        action = make_action([0.0, 0.0, 0.0, *delta_rotation_tcp.as_euler("xyz")])
        current = [0.4, 0.0, 0.5, *current_rotation.as_rotvec()]

        target = robot._target_pose_from_action(action, current)
        actual_delta = R.from_rotvec(target[3:]) * current_rotation.inv()

        np.testing.assert_allclose(actual_delta.as_rotvec(), [0.0, 0.0, 0.01], atol=1e-10)

    def test_disconnect_closes_rpc_when_gripper_cleanup_fails(self) -> None:
        robot = self.make_robot()
        rpc = MagicMock()
        gripper = MagicMock()
        gripper.close.side_effect = RuntimeError("test failure")
        robot._is_connected = True
        robot._robot = rpc
        robot._gripper_backend = gripper

        with self.assertLogs(robot.__class__.__module__, level="WARNING"):
            robot.disconnect()

        gripper.close.assert_called_once_with()
        rpc.close.assert_called_once_with()
        self.assertIsNone(robot._gripper_backend)
        self.assertIsNone(robot._robot)
        self.assertFalse(robot.is_connected)

    def test_tcp_projection_rejects_disabled_base_rx_ry(self) -> None:
        robot = self.make_robot(
            reference_frame="tcp",
            select_vector=[1, 1, 1, 0, 0, 1],
        )
        current_rotation = R.from_euler("xyz", [35, -20, 70], degrees=True)
        delta_rotation_base = R.from_rotvec([0.01, -0.02, 0.0])
        delta_rotation_tcp = current_rotation.inv() * delta_rotation_base * current_rotation
        action = make_action([0.0, 0.0, 0.0, *delta_rotation_tcp.as_euler("xyz")])
        current = [0.4, 0.0, 0.5, *current_rotation.as_rotvec()]

        target = robot._target_pose_from_action(action, current)

        np.testing.assert_allclose(target[3:], current[3:], atol=1e-10)

    def test_small_cross_axis_residual_is_not_active(self) -> None:
        robot = self.make_robot()
        current = [0.4, 0.0, 0.5, 0.0, 0.0, 0.0]
        action = make_action([0.006, 0.00003, -0.00002, 0.0, 0.0, 0.0])
        target = robot._target_pose_from_action(action, current)

        self.assertEqual(
            robot._motion_delta_mask(current, target),
            [True, False, False, False, False, False],
        )

    def test_tcp_state_read_rotation_error_does_not_enable_other_axes(self) -> None:
        robot = self.make_robot(reference_frame="tcp")
        action_rotation = R.from_euler("xyz", [2.8, -1.2, 0.7], degrees=True)
        send_rotation = R.from_euler("xyz", [2.9, -1.25, 0.75], degrees=True)
        action_tcp = action_rotation.inv().apply([0.006, 0.0, 0.0])
        current = [0.4, 0.0, 0.5, *send_rotation.as_rotvec()]
        action = make_action([*action_tcp, 0.0, 0.0, 0.0])
        target = robot._target_pose_from_action(action, current)

        self.assertEqual(
            robot._motion_delta_mask(current, target),
            [True, False, False, False, False, False],
        )

    def test_small_rotation_residual_is_not_active(self) -> None:
        robot = self.make_robot()
        current = [0.4, 0.0, 0.5, 0.1, -0.2, 0.3]
        action = make_action([0.0, 0.0, 0.0, 0.00001, -0.00002, 0.00003])
        target = robot._target_pose_from_action(action, current)

        self.assertEqual(
            robot._motion_delta_mask(current, target),
            [False, False, False, False, False, False],
        )

    def test_real_rotation_action_enables_orientation(self) -> None:
        robot = self.make_robot()
        current = [0.4, 0.0, 0.5, 0.1, -0.2, 0.3]
        action = make_action([0.0, 0.0, 0.0, 0.01, 0.0, 0.0])
        target = robot._target_pose_from_action(action, current)

        self.assertEqual(
            robot._motion_delta_mask(current, target),
            [False, False, False, True, True, True],
        )

    def test_inactive_position_and_orientation_hold_their_targets(self) -> None:
        robot = self.make_robot()
        held_rotation = R.from_euler("xyz", [0.1, -0.2, 0.3]).as_rotvec()
        robot._hold_pose = [0.4, 0.0, 0.5, *held_rotation]
        current_rotation = R.from_euler("xyz", [0.13, -0.18, 0.27]).as_rotvec()
        current = [0.41, 0.02, 0.49, *current_rotation]
        action = make_action([0.006, 0.00003, -0.00002, 0.0, 0.0, 0.0])

        target = robot._target_pose_from_impedance_action(action, current)

        self.assertAlmostEqual(target[0], current[0] + 0.006)
        self.assertAlmostEqual(target[1], 0.0)
        self.assertAlmostEqual(target[2], 0.5)
        np.testing.assert_allclose(target[3:], held_rotation, atol=1e-12)

    def test_active_translation_target_stays_one_step_ahead_of_current(self) -> None:
        robot = self.make_robot()
        current = [0.4, 0.0, 0.5, 0.0, 0.0, 0.0]
        action = make_action([0.006, 0.0, 0.0, 0.0, 0.0, 0.0])

        targets = [
            robot._target_pose_from_impedance_action(action, current)
            for _ in range(10)
        ]

        self.assertTrue(all(target[0] == 0.406 for target in targets))
        self.assertTrue(all(target[1] == 0.0 for target in targets))
        self.assertTrue(all(target[2] == 0.5 for target in targets))

    def test_releasing_axis_holds_measured_position(self) -> None:
        robot = self.make_robot()
        action = make_action([0.006, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot._target_pose_from_impedance_action(
            action,
            [0.4, 0.0, 0.5, 0.0, 0.0, 0.0],
        )

        target = robot._target_pose_from_impedance_action(
            make_action([0.0] * 6),
            [0.404, 0.002, 0.499, 0.01, 0.0, 0.0],
        )

        self.assertAlmostEqual(target[0], 0.404)
        self.assertAlmostEqual(target[1], 0.0)
        self.assertAlmostEqual(target[2], 0.5)
        np.testing.assert_allclose(target[3:], [0.0, 0.0, 0.0], atol=1e-12)

    def test_position_action_never_changes_held_orientation(self) -> None:
        robot = self.make_robot()
        held_rotation = R.from_euler("xyz", [0.2, -0.1, 0.4]).as_rotvec()
        robot._hold_pose = [0.4, 0.0, 0.5, *held_rotation]
        current_rotation = R.from_euler("xyz", [0.25, -0.15, 0.5]).as_rotvec()

        target = robot._target_pose_from_impedance_action(
            make_action([0.0, 0.0, 0.006, 0.0, 0.0, 0.0]),
            [0.4, 0.0, 0.5, *current_rotation],
        )

        np.testing.assert_allclose(target[3:], held_rotation, atol=1e-12)

    def test_rz_action_keeps_held_rx_ry(self) -> None:
        robot = self.make_robot(
            select_vector=[1, 1, 1, 0, 0, 1],
            control_frame_euler_deg=[0, 0, -45],
        )
        selection_to_base = R.from_euler("xyz", [0, 0, -45], degrees=True)
        held_rotation = selection_to_base * R.from_euler(
            "xyz", [-178.0, -2.0, 10.0], degrees=True
        )
        current_rotation = selection_to_base * R.from_euler(
            "xyz", [-176.0, 3.0, 12.0], degrees=True
        )
        robot._hold_pose = [0.4, 0.0, 0.5, *held_rotation.as_rotvec()]

        target = robot._target_pose_from_impedance_action(
            make_action([0.0, 0.0, 0.0, 0.0, 0.0, np.deg2rad(1.0)]),
            [0.4, 0.0, 0.5, *current_rotation.as_rotvec()],
        )

        target_euler = (
            selection_to_base.inv() * R.from_rotvec(target[3:])
        ).as_euler("xyz", degrees=True)
        np.testing.assert_allclose(target_euler, [-178.0, -2.0, 13.0], atol=1e-10)

    def test_releasing_rz_adopts_only_measured_rz(self) -> None:
        robot = self.make_robot(select_vector=[1, 1, 1, 0, 0, 1])
        held_rotation = R.from_euler("xyz", [-178.0, -2.0, 10.0], degrees=True)
        current_rotation = R.from_euler("xyz", [-176.0, 3.0, 15.0], degrees=True)
        robot._hold_pose = [0.4, 0.0, 0.5, *held_rotation.as_rotvec()]
        robot._previous_motion_mask = [False, False, False, True, True, True]

        target = robot._target_pose_from_impedance_action(
            make_action([0.0] * 6),
            [0.4, 0.0, 0.5, *current_rotation.as_rotvec()],
        )

        target_euler = R.from_rotvec(target[3:]).as_euler("xyz", degrees=True)
        np.testing.assert_allclose(target_euler, [-178.0, -2.0, 15.0], atol=1e-10)

    def test_real_two_axis_action_keeps_both_axes_active(self) -> None:
        robot = self.make_robot()
        current = [0.4, 0.0, 0.5, 0.0, 0.0, 0.0]
        action = make_action([0.002, -0.002, 0.00003, 0.0, 0.0, 0.0])
        target = robot._target_pose_from_action(action, current)

        self.assertEqual(
            robot._motion_delta_mask(current, target),
            [True, True, False, False, False, False],
        )

    def test_invalid_deadband_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "translation_axis_deadband"):
            Franka(
                FrankaConfig(
                    use_gripper=False,
                    translation_axis_deadband=0.0,
                )
            )

    def test_invalid_rotation_deadband_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "rotation_axis_deadband"):
            Franka(
                FrankaConfig(
                    use_gripper=False,
                    rotation_axis_deadband=0.0,
                )
            )

    def test_first_gripper_action_is_always_sent(self) -> None:
        robot = Franka(FrankaConfig(use_gripper=True, init_gripper=True))
        commands = []

        class Gripper:
            def command(self, position, blocking=False):
                commands.append((position, blocking))

        robot._gripper_backend = Gripper()
        robot._handle_gripper(1.0)

        self.assertEqual(commands, [(1.0, False)])

    def test_reset_pose_error_uses_physical_rotation_angle(self) -> None:
        current = np.array([0.4, 0.0, 0.5, *R.from_euler("z", 10, degrees=True).as_rotvec()])
        target = np.array([0.403, 0.004, 0.5, *R.from_euler("z", 13, degrees=True).as_rotvec()])

        position_error = np.linalg.norm(target[:3] - current[:3])
        orientation_error = (
            R.from_rotvec(target[3:]) * R.from_rotvec(current[3:]).inv()
        ).magnitude()

        self.assertAlmostEqual(position_error, 0.005)
        self.assertAlmostEqual(np.rad2deg(orientation_error), 3.0)


if __name__ == "__main__":
    unittest.main()
