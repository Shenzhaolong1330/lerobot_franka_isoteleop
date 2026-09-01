"""Build gripper settings and backends from the shared YAML config."""

from types import SimpleNamespace

from lerobot_robot_franka import make_gripper_backend


def franka_gripper_kwargs(robot: dict) -> dict:
    """Return FrankaConfig keyword arguments for the selected gripper."""
    gripper = robot.get("gripper", {})
    pgi = gripper.get("pgi", {})
    hand = gripper.get("franka_hand", {})
    gripper_type = str(gripper.get("type", "pgi")).lower()
    if gripper_type not in ("pgi", "franka_hand"):
        raise ValueError("gripper.type must be 'pgi' or 'franka_hand'.")
    device = pgi if gripper_type == "pgi" else hand
    return {
        "use_gripper": bool(robot.get("use_gripper", True)),
        "init_gripper": bool(gripper.get("init", True)),
        "gripper_type": gripper_type,
        "gripper_port": pgi.get("port", "/dev/franka_pgi_gripper"),
        "gripper_reverse": bool(gripper.get("reverse", False)),
        "close_threshold": float(gripper.get("close_threshold", 0.7)),
        "gripper_force": device.get("force", 70 if gripper_type == "pgi" else 20.0),
        "gripper_speed": device.get("speed", 60 if gripper_type == "pgi" else 0.2),
        "gripper_min_position": int(pgi.get("min_position", 0)),
        "gripper_max_position": int(pgi.get("max_position", 1000)),
        "gripper_max_open": float(hand.get("max_open", 0.0801)),
    }


def configured_gripper(robot: dict, client=None):
    config = SimpleNamespace(**franka_gripper_kwargs(robot))
    return make_gripper_backend(config, client)
