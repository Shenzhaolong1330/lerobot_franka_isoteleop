import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from scripts.server.runtime import (
    REPO_ROOT,
    _cleanup_polymetis_port,
    _fr3_model_args,
    _load_config,
)
from scripts.core.run_record import (
    append_record_times,
    recording_time_info,
    run_record,
)
from scripts.server.robot_model_config import (
    TCP_JOINT_NAME,
    TCP_LINK_NAME,
    write_tcp_urdf,
)
from scripts.utils.dataset_utils import generate_dataset_name


class RuntimeConfigTest(unittest.TestCase):
    @patch("scripts.utils.dataset_utils.datetime")
    def test_generated_dataset_name_uses_date_and_version(self, datetime) -> None:
        datetime.today.return_value.strftime.return_value = "20260806"
        with tempfile.TemporaryDirectory() as directory:
            config = SimpleNamespace(
                resume=False,
                repo_id="scylearning/franka_dataset",
                dataset_path=Path(directory) / "scylearning" / "franka_dataset",
            )
            dataset_name, version = generate_dataset_name(config)

        self.assertEqual(dataset_name, "scylearning/franka_dataset_20260806_v01")
        self.assertEqual(version, "v01")

    @patch("scripts.core.run_record.time_module.perf_counter", return_value=130.0)
    def test_recording_times_are_kept_in_dataset_info(self, _perf_counter) -> None:
        timing = recording_time_info(
            record_start_time=100.0,
            record_loop_time_s=20.0,
            record_loop_count=2,
        )
        config = SimpleNamespace(user_info=None)
        append_record_times(config, timing)

        self.assertEqual(timing["total_time"], "30s")
        self.assertEqual(timing["reset_time"], "10s")
        self.assertEqual(timing["avg_record_time"], "10s")
        self.assertEqual(timing["avg_reset_time"], "5s")
        self.assertIn("total_time=30s", config.user_info)

    @patch("scripts.server.runtime.subprocess.run")
    @patch("scripts.server.runtime._find_run_server_pids", return_value=[])
    @patch("scripts.server.runtime._tcp_port_open", return_value=False)
    def test_closed_polymetis_port_needs_no_cleanup(
        self,
        _port_open,
        _run_server_pids,
        run,
    ) -> None:
        _cleanup_polymetis_port("127.0.0.1", 50051)
        run.assert_not_called()

    @patch("scripts.server.runtime.subprocess.run")
    @patch(
        "scripts.server.runtime._find_run_server_pids",
        side_effect=[[1234], [1234], [], [], []],
    )
    @patch("scripts.server.runtime._tcp_port_open", side_effect=[True, False])
    def test_polymetis_cleanup_targets_configured_port(
        self,
        _port_open,
        _run_server_pids,
        run,
    ) -> None:
        _cleanup_polymetis_port("127.0.0.1", 50051)
        run.assert_called_once_with(
            ["sudo", "kill", "-TERM", "1234"],
            check=False,
        )

    @patch("scripts.server.runtime.subprocess.run")
    @patch(
        "scripts.server.runtime._find_run_server_pids",
        side_effect=[[1234], [1234], [], [], []],
    )
    @patch("scripts.server.runtime._tcp_port_open", side_effect=[False, False])
    def test_polymetis_cleanup_stops_process_before_port_is_open(
        self,
        _port_open,
        _run_server_pids,
        run,
    ) -> None:
        _cleanup_polymetis_port("127.0.0.1", 50051)
        run.assert_called_once_with(
            ["sudo", "kill", "-TERM", "1234"],
            check=False,
        )

    @patch(
        "scripts.core.run_record.generate_dataset_name",
        side_effect=RuntimeError("test failure"),
    )
    def test_record_failure_exits_nonzero(self, _generate_dataset_name) -> None:
        with self.assertRaises(SystemExit) as context:
            run_record(SimpleNamespace(resume=False))
        self.assertEqual(context.exception.code, 1)

    @patch("scripts.core.run_record.handle_incomplete_dataset")
    @patch(
        "scripts.core.run_record.make_camera_configs",
        side_effect=RuntimeError("test failure"),
    )
    @patch(
        "scripts.core.run_record.generate_dataset_name",
        return_value=("scylearning/existing_v01", "v01"),
    )
    def test_resume_failure_never_offers_to_delete_existing_dataset(
        self,
        _generate_dataset_name,
        _make_camera_configs,
        handle_incomplete_dataset,
    ) -> None:
        with self.assertRaises(SystemExit) as context:
            run_record(SimpleNamespace(resume=True))
        self.assertEqual(context.exception.code, 1)
        handle_incomplete_dataset.assert_not_called()

    def test_fr3v2_1_limits_match_bundled_urdf(self) -> None:
        services = _load_config()["services"]
        model = services["robot_model"]
        urdf_path = REPO_ROOT / model["urdf_path"]
        root = ET.parse(urdf_path).getroot()
        joints = [
            root.find(f"./joint[@name='fr3v2_1_joint{index}']")
            for index in range(1, 8)
        ]

        self.assertTrue(all(joint is not None for joint in joints))
        lower = [float(joint.find("limit").attrib["lower"]) for joint in joints]
        upper = [float(joint.find("limit").attrib["upper"]) for joint in joints]
        velocity = [
            float(joint.find("limit").attrib["velocity"]) for joint in joints
        ]
        torque = [float(joint.find("limit").attrib["effort"]) for joint in joints]

        for urdf_values, configured_values in (
            (lower, model["joint_limits_low"]),
            (upper, model["joint_limits_high"]),
            (velocity, model["joint_velocity_limits"]),
            (torque, model["torque_limits"]),
        ):
            for urdf_value, configured_value in zip(
                urdf_values, configured_values
            ):
                self.assertAlmostEqual(urdf_value, configured_value, places=9)
        self.assertIsNotNone(root.find("./link[@name='fr3v2_1_link8']"))
        self.assertIsNotNone(root.find(f"./link[@name='{TCP_LINK_NAME}']"))
        tcp_joint = root.find(f"./joint[@name='{TCP_JOINT_NAME}']")
        self.assertIsNotNone(tcp_joint)
        self.assertEqual(tcp_joint.find("parent").attrib["link"], "fr3v2_1_link8")
        self.assertEqual(tcp_joint.find("child").attrib["link"], TCP_LINK_NAME)

    def test_arm_hydra_overrides_select_fr3v2_1(self) -> None:
        services = _load_config()["services"]
        args = _fr3_model_args(services)

        self.assertIn(f"robot_model.ee_link_name={TCP_LINK_NAME}", args)
        self.assertTrue(
            any(
                argument.startswith("robot_model.robot_description_path=")
                for argument in args
            )
        )
        self.assertTrue(
            any(
                argument.startswith(
                    "robot_client.executable_cfg.limits.joint_pos_lower="
                )
                for argument in args
            )
        )

    def test_tcp_xyz_rpy_overrides_fixed_joint_origin(self) -> None:
        model = _load_config()["services"]["robot_model"]
        source = REPO_ROOT / model["urdf_path"]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "tcp.urdf"
            write_tcp_urdf(source, output, [0.1, -0.2, 0.3, 0.4, -0.5, 0.6])
            origin = ET.parse(output).getroot().find(
                f"./joint[@name='{TCP_JOINT_NAME}']/origin"
            )
            self.assertEqual(
                origin.attrib["xyz"],
                "0.10000000000000001 -0.20000000000000001 0.29999999999999999",
            )
            self.assertEqual(
                origin.attrib["rpy"],
                "0.40000000000000002 -0.5 0.59999999999999998",
            )


if __name__ == "__main__":
    unittest.main()
