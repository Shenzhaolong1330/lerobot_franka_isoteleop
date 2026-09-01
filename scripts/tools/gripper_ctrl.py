from pathlib import Path

import yaml

from lerobot_robot_franka import FrankaInterfaceClient
from scripts.utils.gripper_config import configured_gripper


def load_config() -> dict:
    with (Path(__file__).parents[1] / "config" / "cfg.yaml").open(
        "r", encoding="utf-8"
    ) as file:
        return yaml.safe_load(file)["record"]["robot"]


def main() -> None:
    config = load_config()
    gripper_type = str(config.get("gripper", {}).get("type", "pgi")).lower()
    client = None
    if gripper_type == "franka_hand":
        client = FrankaInterfaceClient(
            config["server_host"],
            int(config["server_port"]),
        )
    gripper = configured_gripper(config, client)
    try:
        gripper.connect(config.get("gripper", {}).get("init", True))
        print("Enter 1 to open, 0 to close, or q to quit.")
        while True:
            value = input("Gripper command [0/1/q]: ").strip().lower()
            if value == "q":
                return
            if value not in {"0", "1"}:
                print("Please enter 0, 1, or q.")
                continue
            gripper.command(float(value), blocking=True)
            print(gripper.read_state())
    finally:
        gripper.close()
        if client is not None:
            client.close()


if __name__ == "__main__":
    main()
