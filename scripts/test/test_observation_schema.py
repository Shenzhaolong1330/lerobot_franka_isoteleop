import unittest
from unittest.mock import MagicMock

from lerobot_robot_franka import Franka, FrankaConfig, PgiGripper


class FrankaObservationSchemaTest(unittest.TestCase):
    def test_state_contains_nineteen_robot_values(self) -> None:
        robot = Franka(FrankaConfig(use_gripper=False))
        axes = ("x", "y", "z", "rx", "ry", "rz")
        expected = [
            *(f"tcp_pose.{axis}" for axis in axes),
            *(f"tcp_speed.{axis}" for axis in axes),
            *(f"tcp_force.{axis}" for axis in axes),
            "gripper_raw_position",
        ]

        self.assertEqual(list(robot._state_features), expected)
        self.assertEqual(len(robot._state_features), 19)

    def test_only_one_consecutive_observation_fallback_is_allowed(self) -> None:
        robot = Franka(FrankaConfig(use_gripper=False))
        robot._is_connected = True
        robot._robot = MagicMock()
        robot._robot.robot_get_ee_state.side_effect = RuntimeError("test failure")
        robot._prev_observation = {"tcp_pose.x": 0.0}

        with self.assertLogs(robot.__class__.__module__, level="WARNING"):
            observation = robot.get_observation()
        self.assertEqual(observation, robot._prev_observation)
        with self.assertRaisesRegex(RuntimeError, "test failure"):
            robot.get_observation()

    def test_pgi_reader_failure_is_not_returned_as_cached_state(self) -> None:
        gripper = PgiGripper(
            port="/dev/null",
            speed=100,
            force=100,
            min_position=0,
            max_position=1000,
            reverse=False,
        )
        gripper.gripper = object()
        gripper._reader_error = RuntimeError("test failure")

        with self.assertRaisesRegex(RuntimeError, "reader has stopped"):
            gripper.read_normalized()


if __name__ == "__main__":
    unittest.main()
