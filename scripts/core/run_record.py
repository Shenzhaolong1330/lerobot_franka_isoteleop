import logging
import sys
import termios
import time as time_module
from pathlib import Path
from typing import Any

import yaml
from send2trash import send2trash

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.processor import make_default_processors
from lerobot.utils.constants import HF_LEROBOT_HOME
from lerobot.utils.control_utils import (
    init_keyboard_listener,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.utils.visualization_utils import init_rerun
from lerobot_robot_franka import Franka, FrankaConfig
from lerobot_teleoperator_franka import FrankaTeleop, FrankaTeleopConfig
from scripts.core.record_loop import record_loop
from scripts.utils.dataset_utils import generate_dataset_name, update_dataset_info
from scripts.utils.gripper_config import franka_gripper_kwargs

logging.basicConfig(level=logging.INFO, format="%(message)s")


class RecordConfig:
    def __init__(self, cfg: dict[str, Any]):
        storage = cfg["storage"]
        task = cfg["task"]
        timing = cfg["time"]
        camera = cfg.get("cameras", {})
        robot = cfg["robot"]
        teleop = cfg.get("teleop", {})
        impedance = robot.get("cartesian_impedance", {})

        self.repo_id: str = cfg["repo_id"]
        self.fps: int = cfg.get("fps", 15)
        self.dataset_path = HF_LEROBOT_HOME / self.repo_id
        self.user_info: str | None = cfg.get("user_notes")

        self.server_host: str = robot.get("server_host", "127.0.0.1")
        self.server_port: int = robot.get("server_port", 4242)
        self.gripper = franka_gripper_kwargs(robot)
        self.reference_frame: str = robot.get("reference_frame", "base")
        self.translation_axis_deadband: float = impedance.get(
            "translation_axis_deadband", 0.0005
        )
        self.rotation_axis_deadband: float = impedance.get(
            "rotation_axis_deadband", 0.0005
        )
        self.select_vector: list = impedance.get(
            "select_vector", [1, 1, 1, 1, 1, 1]
        )
        self.cartesian_stiffness: list = impedance.get(
            "stiffness", [750, 750, 750, 250, 250, 250]
        )
        self.cartesian_damping: list = impedance.get(
            "damping", [37, 37, 37, 9, 9, 9]
        )
        self.init_pose: list[float] = robot["init_pose"]
        self.init_pose_range: list[float] = robot.get(
            "init_pose_range", [0, 0, 0, 0, 0, 0]
        )
        self.reset_time_to_go: float = robot.get("reset_time_to_go", 5.0)
        self.max_translation_step: float = robot.get("max_translation_step", 0.05)
        self.max_rotation_step: float = robot.get("max_rotation_step", 0.2)

        self.teleop_step_size: float = teleop.get("step_size", 0.006)
        self.teleop_rot_step_size: float = teleop.get("rot_step_size", 0.02)
        self.teleop_alternate_step_size: float | None = teleop.get("fine_step_size")
        self.teleop_alternate_rot_step_size: float | None = teleop.get(
            "fine_rot_step_size"
        )
        self.control_frame_euler_deg: list = teleop.get(
            "control_frame_euler_deg", [0.0, 0.0, 0.0]
        )

        self.num_episodes: int = task.get("num_episodes", 1)
        self.display: bool = task.get("display", True)
        self.task_description: str = task.get("description", "default task")
        self.resume: bool = task.get("resume", False)
        self.resume_dataset: str = task.get("resume_dataset", self.repo_id)

        self.episode_time_sec: int = timing.get("episode_time_sec", 60)
        self.reset_time_sec: int = timing.get("reset_time_sec", 60)
        self.save_meta_period: int = timing.get("save_meta_period", 1)

        camera_enabled = camera.get("enabled", True)
        if not isinstance(camera_enabled, bool):
            raise ValueError("cameras.enabled must be true or false.")
        self.cameras_enabled: bool = camera_enabled
        self.wrist_cam_serial: str | None = camera.get("wrist_cam_serial")
        self.exterior_cam_serial: str | None = camera.get("exterior_cam_serial")
        self.width: int = camera.get("width", 640)
        self.height: int = camera.get("height", 480)
        self.dummy_value: int = camera.get("dummy_value", 0)
        self.push_to_hub: bool = storage.get("push_to_hub", False)


def make_camera_configs(record_cfg: RecordConfig) -> dict:
    camera_names = ("wrist_image", "exterior_image")
    if not record_cfg.cameras_enabled:
        from lerobot_robot_franka import DummyCameraConfig

        logging.info(
            "====== [CAM] RealSense disabled; using dummy frames for %s ======",
            ", ".join(camera_names),
        )
        return {
            name: DummyCameraConfig(
                fps=record_cfg.fps,
                width=record_cfg.width,
                height=record_cfg.height,
                value=record_cfg.dummy_value,
            )
            for name in camera_names
        }

    if not record_cfg.wrist_cam_serial or not record_cfg.exterior_cam_serial:
        raise ValueError(
            "Both camera serial numbers are required when cameras.enabled is true."
        )

    # Import RealSense only when real cameras are enabled.
    from lerobot.cameras.configs import ColorMode, Cv2Rotation
    from lerobot.cameras.realsense.camera_realsense import RealSenseCameraConfig

    return {
        "wrist_image": RealSenseCameraConfig(
            serial_number_or_name=record_cfg.wrist_cam_serial,
            fps=record_cfg.fps,
            width=record_cfg.width,
            height=record_cfg.height,
            color_mode=ColorMode.RGB,
            use_depth=False,
            rotation=Cv2Rotation.NO_ROTATION,
        ),
        "exterior_image": RealSenseCameraConfig(
            serial_number_or_name=record_cfg.exterior_cam_serial,
            fps=record_cfg.fps,
            width=record_cfg.width,
            height=record_cfg.height,
            color_mode=ColorMode.RGB,
            use_depth=False,
            rotation=Cv2Rotation.NO_ROTATION,
        ),
    }


def handle_incomplete_dataset(dataset_path: Path) -> bool:
    if dataset_path.exists():
        print(
            "====== [WARNING] Detected an incomplete dataset folder: "
            f"{dataset_path} ======"
        )
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        ans = input("Do you want to delete it? (y/n): ").strip().lower()
        if ans == "y":
            print(f"====== [TRASH] Moving folder to trash: {dataset_path} ======")
            send2trash(str(dataset_path))
            print(
                "====== [DONE] Incomplete dataset folder moved to trash "
                "successfully. ======"
            )
            return False
        print(
            "====== [KEEP] Incomplete dataset folder retained; "
            "please check manually. ======"
        )
        return True
    return False


def format_duration(seconds: float) -> str:
    total_seconds = max(0, int(round(seconds)))
    hours, remainder = divmod(total_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    if hours:
        return f"{hours}h {minutes}m {seconds}s"
    if minutes:
        return f"{minutes}m {seconds}s"
    return f"{seconds}s"


def recording_time_info(
    record_start_time: float,
    record_loop_time_s: float,
    record_loop_count: int,
) -> dict[str, str]:
    total_time_s = time_module.perf_counter() - record_start_time
    reset_time_s = max(0.0, total_time_s - record_loop_time_s)
    denominator = max(record_loop_count, 1)
    return {
        "record_time": format_duration(record_loop_time_s),
        "reset_time": format_duration(reset_time_s),
        "total_time": format_duration(total_time_s),
        "avg_record_time": format_duration(record_loop_time_s / denominator),
        "avg_reset_time": format_duration(reset_time_s / denominator),
    }


def log_record_times(timing: dict[str, str]) -> None:
    labels = {
        "record_time": "Record loop time",
        "reset_time": "Reset/non-record time",
        "total_time": "Total recording time",
        "avg_record_time": "Average record loop time",
        "avg_reset_time": "Average reset/non-record time",
    }
    for key, label in labels.items():
        logging.info("====== [INFO] %s: %s ======", label, timing[key])


def append_record_times(record_cfg: RecordConfig, timing: dict[str, str]) -> None:
    duration_info = (
        f"record_time={timing['record_time']}, reset_time={timing['reset_time']}, "
        f"total_time={timing['total_time']}, "
        f"avg_record_time={timing['avg_record_time']}, "
        f"avg_reset_time={timing['avg_reset_time']}"
    )
    if record_cfg.user_info:
        if duration_info not in str(record_cfg.user_info):
            record_cfg.user_info = f"{record_cfg.user_info}; {duration_info}"
    else:
        record_cfg.user_info = duration_info


def get_episode_buffer_size(dataset: LeRobotDataset | None) -> int:
    if dataset is None:
        return 0
    episode_buffer = getattr(dataset, "episode_buffer", None)
    if not episode_buffer:
        return 0
    return int(episode_buffer.get("size", 0))


def discard_unsaved_episode(dataset: LeRobotDataset | None) -> None:
    if dataset is None:
        return

    if get_episode_buffer_size(dataset) <= 0:
        return

    try:
        dataset.clear_episode_buffer(delete_images=len(dataset.meta.image_keys) > 0)
        logging.info("====== [INFO] Discarded unsaved episode buffer. ======")
    except Exception as cleanup_error:
        logging.info(
            "====== [WARNING] Failed to discard unsaved episode buffer: %s ======",
            cleanup_error,
        )


def finalize_dataset_safely(dataset: LeRobotDataset | None) -> None:
    if dataset is None:
        return

    try:
        dataset.finalize()
    except Exception as finalize_error:
        logging.info(
            "====== [WARNING] Failed to finalize dataset cleanly: %s ======",
            finalize_error,
        )


def disconnect_safely(robot: Franka | None, teleop: FrankaTeleop | None) -> None:
    for name, device in (("Robot", robot), ("Keyboard", teleop)):
        if device is None or not device.is_connected:
            continue
        try:
            device.disconnect()
        except Exception as error:
            logging.warning(
                "====== [WARNING] %s cleanup failed: %s ======",
                name,
                error,
            )


def wait_for_enter(prompt: str) -> None:
    while True:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        user_input = input(prompt)
        if user_input == "":
            return
        logging.info("====== [WARNING] Please press only Enter to continue ======")


def reset_to_init_pose(
    record_cfg: RecordConfig,
    robot: Franka,
    teleop: FrankaTeleop,
    events: dict,
    teleop_action_processor,
    robot_action_processor,
    robot_observation_processor,
) -> None:
    logging.info(
        "====== [RESET] Use the keyboard to move to a safe pre-reset pose ======"
    )
    logging.info(
        "\033[32m====== [RESET] Press Right Arrow to finish manual reset ======\033[0m"
    )
    record_loop(
        robot=robot,
        events=events,
        fps=record_cfg.fps,
        teleop=teleop,
        teleop_action_processor=teleop_action_processor,
        robot_action_processor=robot_action_processor,
        robot_observation_processor=robot_observation_processor,
        control_time_s=record_cfg.reset_time_sec,
        single_task=record_cfg.task_description,
        display_data=record_cfg.display,
    )

    events["exit_early"] = False
    events["rerecord_episode"] = False
    if events["stop_recording"]:
        return

    logging.info("====== [RESET] Moving to the configured initial EE pose ======")
    robot.reset_to_init_pose(record_cfg.init_pose, record_cfg.init_pose_range)


def run_record(record_cfg: RecordConfig) -> None:
    robot = None
    teleop = None
    dataset = None
    dataset_name = None
    data_version = None
    record_start_time = None
    record_loop_time_s = 0.0
    record_loop_count = 0
    dataset_info_updated = False

    try:
        dataset_name, data_version = generate_dataset_name(record_cfg)

        camera_config = make_camera_configs(record_cfg)
        teleop_config = FrankaTeleopConfig(
            use_gripper=record_cfg.gripper["use_gripper"],
            init_gripper=record_cfg.gripper["init_gripper"],
            step_size=record_cfg.teleop_step_size,
            rot_step_size=record_cfg.teleop_rot_step_size,
            alternate_step_size=record_cfg.teleop_alternate_step_size,
            alternate_rot_step_size=record_cfg.teleop_alternate_rot_step_size,
            reference_frame=record_cfg.reference_frame,
            control_frame_euler_deg=record_cfg.control_frame_euler_deg,
            select_vector=record_cfg.select_vector,
        )
        robot_config = FrankaConfig(
            server_host=record_cfg.server_host,
            server_port=record_cfg.server_port,
            cameras=camera_config,
            **record_cfg.gripper,
            reference_frame=record_cfg.reference_frame,
            translation_axis_deadband=record_cfg.translation_axis_deadband,
            rotation_axis_deadband=record_cfg.rotation_axis_deadband,
            select_vector=record_cfg.select_vector,
            control_frame_euler_deg=record_cfg.control_frame_euler_deg,
            cartesian_stiffness=record_cfg.cartesian_stiffness,
            cartesian_damping=record_cfg.cartesian_damping,
            init_pose=record_cfg.init_pose,
            init_pose_range=record_cfg.init_pose_range,
            reset_time_to_go=record_cfg.reset_time_to_go,
            max_translation_step=record_cfg.max_translation_step,
            max_rotation_step=record_cfg.max_rotation_step,
        )
        robot = Franka(robot_config)
        teleop = FrankaTeleop(teleop_config)
        teleop.set_robot(robot)

        action_features = hw_to_dataset_features(robot.action_features, "action")
        obs_features = hw_to_dataset_features(
            robot.observation_features,
            "observation",
            use_video=True,
        )
        dataset_features = {**action_features, **obs_features}

        if record_cfg.resume:
            dataset = LeRobotDataset(
                dataset_name,
            )

            if hasattr(robot, "cameras") and len(robot.cameras) > 0:
                dataset.start_image_writer()
            sanity_check_dataset_robot_compatibility(
                dataset,
                robot,
                record_cfg.fps,
                dataset_features,
            )
        else:
            dataset = LeRobotDataset.create(
                repo_id=dataset_name,
                fps=record_cfg.fps,
                features=dataset_features,
                robot_type=robot.name,
                use_videos=True,
                image_writer_threads=4,
            )

        dataset.meta.metadata_buffer_size = record_cfg.save_meta_period

        _, events = init_keyboard_listener()
        init_rerun(session_name="recording")

        (
            teleop_action_processor,
            robot_action_processor,
            robot_observation_processor,
        ) = make_default_processors()

        robot.connect()
        robot.reset_to_init_pose(record_cfg.init_pose, record_cfg.init_pose_range)
        teleop.connect()

        episode_idx = 0
        record_start_time = time_module.perf_counter()
        while episode_idx < record_cfg.num_episodes and not events["stop_recording"]:
            events["exit_early"] = False
            events["rerecord_episode"] = False
            teleop.reset_step_size()
            robot.set_episode_reference_pose()
            logging.info(
                "====== [RECORD] Recording episode %d of %d ======",
                episode_idx + 1,
                record_cfg.num_episodes,
            )
            episode_record_start = time_module.perf_counter()
            try:
                record_loop(
                    robot=robot,
                    events=events,
                    fps=record_cfg.fps,
                    teleop=teleop,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    dataset=dataset,
                    control_time_s=record_cfg.episode_time_sec,
                    single_task=record_cfg.task_description,
                    display_data=record_cfg.display,
                )
            finally:
                record_loop_time_s += time_module.perf_counter() - episode_record_start
                record_loop_count += 1

            if events["rerecord_episode"]:
                logging.info("Re-recording episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                reset_to_init_pose(
                    record_cfg,
                    robot,
                    teleop,
                    events,
                    teleop_action_processor,
                    robot_action_processor,
                    robot_observation_processor,
                )
                continue
            robot.stop_control()

            if get_episode_buffer_size(dataset) <= 0:
                logging.info(
                    "====== [WARNING] No frames recorded; skipping episode. ======"
                )
                events["exit_early"] = False
                events["rerecord_episode"] = False
                if events["stop_recording"]:
                    break
                continue

            dataset.save_episode()

            has_next_episode = episode_idx < record_cfg.num_episodes - 1
            if not events["stop_recording"] and has_next_episode:
                wait_for_enter("====== [WAIT] Press Enter to reset the robot ======")
                reset_to_init_pose(
                    record_cfg,
                    robot,
                    teleop,
                    events,
                    teleop_action_processor,
                    robot_action_processor,
                    robot_observation_processor,
                )
                if not events["stop_recording"]:
                    wait_for_enter(
                        "====== [WAIT] Press Enter to start the next episode ======"
                    )

            episode_idx += 1

        logging.info("Stop recording")
        disconnect_safely(robot, teleop)
        dataset.finalize()

        timing = recording_time_info(
            record_start_time, record_loop_time_s, record_loop_count
        )
        log_record_times(timing)
        append_record_times(record_cfg, timing)
        update_dataset_info(record_cfg, dataset_name, data_version)
        dataset_info_updated = True
        if record_cfg.push_to_hub:
            dataset.push_to_hub()

    except (Exception, KeyboardInterrupt) as error:
        message = (
            f"====== [ERROR] {error} ======"
            if isinstance(error, Exception)
            else "\n====== [INFO] Ctrl+C detected ======"
        )
        logging.info(message)
        if record_start_time is not None:
            timing = recording_time_info(
                record_start_time, record_loop_time_s, record_loop_count
            )
            log_record_times(timing)
            append_record_times(record_cfg, timing)
        disconnect_safely(robot, teleop)
        discard_unsaved_episode(dataset)
        finalize_dataset_safely(dataset)
        if dataset_name is not None and record_cfg.resume:
            logging.info(
                "====== [INFO] Existing resume dataset was kept. ======"
            )
            if (
                record_start_time is not None
                and data_version is not None
                and not dataset_info_updated
            ):
                update_dataset_info(record_cfg, dataset_name, data_version)
        elif dataset_name is not None and dataset_info_updated:
            logging.info(
                "====== [INFO] Complete local dataset was kept. ======"
            )
        elif dataset_name is not None:
            dataset_path = Path(HF_LEROBOT_HOME) / dataset_name
            keep_dataset = handle_incomplete_dataset(dataset_path)
            if (
                keep_dataset
                and data_version is not None
                and not dataset_info_updated
            ):
                update_dataset_info(record_cfg, dataset_name, data_version)
        raise SystemExit(1) from error


def main() -> None:
    config_path = Path(__file__).parents[1] / "config" / "cfg.yaml"
    with config_path.open("r", encoding="utf-8") as file:
        cfg = yaml.safe_load(file)

    record_cfg = RecordConfig(cfg["record"])
    run_record(record_cfg)


if __name__ == "__main__":
    main()
