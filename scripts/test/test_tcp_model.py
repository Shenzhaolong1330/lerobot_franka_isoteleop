import unittest

import numpy as np
import torch
from scipy.spatial.transform import Rotation

try:
    import torchcontrol as toco
except ModuleNotFoundError:
    toco = None

from scripts.server.robot_model_config import resolve_robot_description
from scripts.server.runtime import REPO_ROOT, _load_config


@unittest.skipIf(toco is None, "torchcontrol is installed in polymetis-local")
class TcpRobotModelTest(unittest.TestCase):
    Q = torch.tensor(
        [-0.79, 0.27, -0.19, -2.22, 0.09, 2.46, -0.38],
        dtype=torch.float32,
    )

    @staticmethod
    def _model(path, ee_link):
        return toco.models.RobotModelPinocchio(str(path), ee_link)

    def test_default_tcp_matches_link8(self) -> None:
        config = _load_config()["services"]["robot_model"]
        path = resolve_robot_description(config, REPO_ROOT)
        link8 = self._model(path, "fr3v2_1_link8")
        tcp = self._model(path, "fr3v2_1_tcp")

        link8_pose = link8.forward_kinematics(self.Q)
        tcp_pose = tcp.forward_kinematics(self.Q)
        torch.testing.assert_close(tcp_pose[0], link8_pose[0])
        self.assertAlmostEqual(
            abs(float(torch.dot(tcp_pose[1], link8_pose[1]))), 1.0, places=6
        )
        torch.testing.assert_close(
            tcp.compute_jacobian(self.Q), link8.compute_jacobian(self.Q)
        )

    def test_nonzero_tcp_changes_pose_and_linear_jacobian(self) -> None:
        config = dict(_load_config()["services"]["robot_model"])
        config["tcp_xyz_rpy"] = [0.03, -0.02, 0.1, 0.2, -0.1, 0.3]
        source = resolve_robot_description(
            {**config, "tcp_xyz_rpy": [0.0] * 6}, REPO_ROOT
        )
        generated = resolve_robot_description(config, REPO_ROOT)
        link8 = self._model(source, "fr3v2_1_link8")
        tcp = self._model(generated, "fr3v2_1_tcp")

        position_ee, quaternion_ee = link8.forward_kinematics(self.Q)
        position_tcp, quaternion_tcp = tcp.forward_kinematics(self.Q)
        rotation_ee = Rotation.from_quat(quaternion_ee.numpy()).as_matrix()
        rotation_offset = Rotation.from_euler(
            "xyz", config["tcp_xyz_rpy"][3:]
        ).as_matrix()
        offset_base = rotation_ee @ np.asarray(config["tcp_xyz_rpy"][:3])

        np.testing.assert_allclose(
            position_tcp.numpy(), position_ee.numpy() + offset_base, atol=1e-6
        )
        np.testing.assert_allclose(
            Rotation.from_quat(quaternion_tcp.numpy()).as_matrix(),
            rotation_ee @ rotation_offset,
            atol=1e-6,
        )

        jacobian_ee = link8.compute_jacobian(self.Q).numpy()
        jacobian_tcp = tcp.compute_jacobian(self.Q).numpy()
        expected_linear = (
            jacobian_ee[:3]
            - np.cross(offset_base, jacobian_ee[3:].T).T
        )
        np.testing.assert_allclose(jacobian_tcp[:3], expected_linear, atol=1e-6)
        np.testing.assert_allclose(jacobian_tcp[3:], jacobian_ee[3:], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
