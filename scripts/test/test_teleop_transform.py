import unittest

import numpy as np
from scipy.spatial.transform import Rotation as R

from lerobot_teleoperator_franka import FrankaTeleop, FrankaTeleopConfig


ACTION_KEYS = ("delta_x", "delta_y", "delta_z", "delta_rx", "delta_ry", "delta_rz")


def make_action(values: list[float]) -> dict[str, float]:
    return dict(zip(ACTION_KEYS, values, strict=True))


class PoseRobot:
    def __init__(self, rotation: R):
        self.pose = [0.0, 0.0, 0.0, *rotation.as_rotvec()]

    def get_ee_pose(self) -> list[float]:
        return self.pose


class FrankaTeleopTransformTest(unittest.TestCase):
    def test_control_frame_and_axis_selection(self) -> None:
        teleop = FrankaTeleop(
            FrankaTeleopConfig(
                use_gripper=False,
                control_frame_euler_deg=[0.0, 0.0, -45.0],
                select_vector=[1, 0, 1, 0, 0, 1],
            )
        )
        actual = teleop._to_reference_delta(
            make_action([0.01, 0.02, 0.03, 0.04, 0.05, 0.06])
        )
        frame = R.from_euler("z", -45.0, degrees=True).as_matrix()

        np.testing.assert_allclose(
            [actual[key] for key in ACTION_KEYS[:3]],
            frame @ [0.01, 0.0, 0.03],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            [actual[key] for key in ACTION_KEYS[3:]],
            [0.0, 0.0, 0.06],
            atol=1e-12,
        )
        self.assertEqual(teleop.action_features["shape"], (6,))

    def test_base_delta_is_expressed_in_tcp_frame(self) -> None:
        current = R.from_euler("xyz", [20.0, -10.0, 35.0], degrees=True)
        teleop = FrankaTeleop(
            FrankaTeleopConfig(
                use_gripper=False,
                reference_frame="tcp",
            )
        )
        teleop.set_robot(PoseRobot(current))
        source = make_action([0.01, -0.02, 0.03, 0.01, 0.02, -0.03])
        actual = teleop._to_reference_delta(source)
        expected_rotation = (
            current.inv() * R.from_euler("xyz", [0.01, 0.02, -0.03]) * current
        ).as_euler("xyz")

        np.testing.assert_allclose(
            [actual[key] for key in ACTION_KEYS[:3]],
            current.inv().apply([0.01, -0.02, 0.03]),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            [actual[key] for key in ACTION_KEYS[3:]],
            expected_rotation,
            atol=1e-12,
        )


if __name__ == "__main__":
    unittest.main()
