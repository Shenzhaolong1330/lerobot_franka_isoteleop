import time
from pathlib import Path
from typing import Any

import yaml

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import log_say
from lerobot_robot_franka import Franka, FrankaConfig
from scripts.utils.gripper_config import franka_gripper_kwargs


def feature_names(feature: dict[str, Any]) -> list[str]:
    names = feature["names"]
    if isinstance(names, dict):
        if all(isinstance(key, int) for key in names):
            return [names[index] for index in sorted(names)]
        return [name for name, _ in sorted(names.items(), key=lambda item: item[1])]
    return list(names)


class ReplayConfig:
    def __init__(self, cfg: dict[str, Any]):
        robot = cfg["robot"]
        impedance = robot.get("cartesian_impedance", {})
        self.repo_id: str = cfg["repo_id"]
        self.episode_idx: int = cfg.get("episode_idx", 0)
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
        self.control_frame_euler_deg: list = robot.get(
            "control_frame_euler_deg", [0.0, 0.0, 0.0]
        )
        self.cartesian_stiffness: list = impedance.get(
            "stiffness", [750, 750, 750, 250, 250, 250]
        )
        self.cartesian_damping: list = impedance.get(
            "damping", [37, 37, 37, 9, 9, 9]
        )


def run_replay(replay_cfg: ReplayConfig) -> None:
    robot = Franka(
        FrankaConfig(
            server_host=replay_cfg.server_host,
            server_port=replay_cfg.server_port,
            **replay_cfg.gripper,
            reference_frame=replay_cfg.reference_frame,
            translation_axis_deadband=replay_cfg.translation_axis_deadband,
            rotation_axis_deadband=replay_cfg.rotation_axis_deadband,
            select_vector=replay_cfg.select_vector,
            control_frame_euler_deg=replay_cfg.control_frame_euler_deg,
            cartesian_stiffness=replay_cfg.cartesian_stiffness,
            cartesian_damping=replay_cfg.cartesian_damping,
        )
    )
    robot.connect()
    try:
        dataset = LeRobotDataset(
            replay_cfg.repo_id,
            episodes=[replay_cfg.episode_idx],
        )
        actions = dataset.hf_dataset.select_columns("action")
        names = feature_names(dataset.features["action"])
        log_say(f"Replaying episode {replay_cfg.episode_idx}")

        for index in range(dataset.num_frames):
            start = time.perf_counter()
            action = {
                name: float(actions[index]["action"][offset])
                for offset, name in enumerate(names)
            }
            robot.send_action(action)
            busy_wait(1.0 / dataset.fps - (time.perf_counter() - start))
    finally:
        robot.disconnect()


def main() -> None:
    with open(Path(__file__).parents[1] / "config" / "cfg.yaml", "r") as file:
        config = yaml.safe_load(file)
    run_replay(ReplayConfig(config["replay"]))


if __name__ == "__main__":
    main()
