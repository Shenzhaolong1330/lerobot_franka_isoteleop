import logging
import time
from pathlib import Path
from typing import Any

import yaml
from scipy.spatial.transform import Rotation as R

from lerobot_robot_franka import FrankaInterfaceClient
from scripts.utils.gripper_config import configured_gripper


logging.basicConfig(level=logging.INFO, format="%(message)s")


def load_robot_config() -> dict:
    with (Path(__file__).parents[1] / "config" / "cfg.yaml").open(
        "r", encoding="utf-8"
    ) as file:
        return yaml.safe_load(file)["record"]["robot"]


def format_vector(values, precision: int = 6) -> str:
    return "[" + ", ".join(f"{float(value):.{precision}f}" for value in values) + "]"


def read_robot_state(
    client: FrankaInterfaceClient,
    gripper_backend=None,
) -> dict[str, Any]:
    ee_pose = client.robot_get_ee_pose().tolist()
    state: dict[str, Any] = {
        "joint_positions_rad": client.robot_get_joint_positions().tolist(),
        "joint_velocities_rad_s": client.robot_get_joint_velocities().tolist(),
        "ee_pose_rotvec_m_rad": ee_pose,
        "ee_pose_euler_m_rad": [
            *ee_pose[:3],
            *R.from_rotvec(ee_pose[3:]).as_euler("xyz").tolist(),
        ],
    }
    if gripper_backend is not None:
        state["gripper"] = gripper_backend.read_state()
    return state


def print_robot_state(state: dict[str, Any]) -> None:
    logging.info("")
    logging.info("Franka state at %s", time.strftime("%Y-%m-%d %H:%M:%S"))
    logging.info(
        "  Joint positions [rad]:    %s",
        format_vector(state["joint_positions_rad"]),
    )
    logging.info(
        "  Joint velocities [rad/s]: %s",
        format_vector(state["joint_velocities_rad_s"]),
    )
    logging.info(
        "  EE pose rotvec [m, rad]:  %s",
        format_vector(state["ee_pose_rotvec_m_rad"]),
    )
    logging.info(
        "  EE pose Euler XYZ [m, rad]: %s",
        format_vector(state["ee_pose_euler_m_rad"]),
    )
    if "gripper" in state:
        logging.info("  Gripper: %s", state["gripper"])


def main() -> None:
    config = load_robot_config()
    server_host = config["server_host"]
    server_port = int(config["server_port"])
    logging.info(
        "Connecting to Franka ZeroRPC server at %s:%d...",
        server_host,
        server_port,
    )
    client = FrankaInterfaceClient(server_host, server_port)
    gripper_backend = None
    try:
        if config.get("use_gripper", True):
            gripper_backend = configured_gripper(config, client)
            gripper_backend.connect(config.get("gripper", {}).get("init", True))
        print_robot_state(read_robot_state(client, gripper_backend))
    finally:
        if gripper_backend is not None:
            gripper_backend.close()
        client.close()
        logging.info("Disconnected.")


if __name__ == "__main__":
    main()
