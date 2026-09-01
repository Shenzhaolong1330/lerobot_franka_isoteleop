"""Keyboard recording loop."""

import time

from lerobot.datasets.image_writer import safe_stop_image_writer
from lerobot.datasets.utils import build_dataset_frame
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import log_rerun_data


@safe_stop_image_writer
def record_loop(
    robot,
    events: dict,
    fps: int,
    teleop_action_processor,
    robot_action_processor,
    robot_observation_processor,
    dataset=None,
    teleop=None,
    control_time_s: int | float | None = None,
    single_task: str | None = None,
    display_data: bool = False,
) -> None:
    """Run keyboard control and optionally append dataset frames."""
    if dataset is not None and dataset.fps != fps:
        raise ValueError(f"Dataset FPS mismatch: {dataset.fps} != {fps}.")
    if teleop is None:
        raise ValueError("The Franka keyboard teleoperator is required.")
    if control_time_s is None:
        raise ValueError("control_time_s is required.")

    start_episode = time.perf_counter()
    while time.perf_counter() - start_episode < control_time_s:
        loop_start = time.perf_counter()
        if events["exit_early"]:
            events["exit_early"] = False
            break

        observation = robot.get_observation()
        processed_observation = robot_observation_processor(observation)
        action = teleop.get_action()
        processed_action = teleop_action_processor((action, observation))
        robot_action = robot_action_processor((processed_action, observation))
        robot.send_action(robot_action)

        if dataset is not None:
            observation_frame = build_dataset_frame(
                dataset.features,
                processed_observation,
                prefix=OBS_STR,
            )
            action_frame = build_dataset_frame(
                dataset.features,
                processed_action,
                prefix=ACTION,
            )
            dataset.add_frame(
                {**observation_frame, **action_frame, "task": single_task}
            )

        if display_data:
            log_rerun_data(
                observation=processed_observation,
                action=processed_action,
            )

        busy_wait(1.0 / fps - (time.perf_counter() - loop_start))
