import sys

from .lerobot_robot_franka import (
    Franka,
    FrankaConfig,
    DummyCamera,
    DummyCameraConfig,
    FrankaHandGripper,
    FrankaInterfaceClient,
    PgiGripper,
    make_gripper_backend,
)
from .lerobot_robot_franka import config_franka as _config_franka
from .lerobot_robot_franka import dummy_camera as _dummy_camera
from .lerobot_robot_franka import franka as _franka
from .lerobot_robot_franka import franka_interface_client as _franka_interface_client
from .lerobot_robot_franka import grippers as _grippers

sys.modules[__name__ + ".config_franka"] = _config_franka
sys.modules[__name__ + ".dummy_camera"] = _dummy_camera
sys.modules[__name__ + ".franka"] = _franka
sys.modules[__name__ + ".franka_interface_client"] = _franka_interface_client
sys.modules[__name__ + ".grippers"] = _grippers

__all__ = [
    "Franka",
    "FrankaConfig",
    "DummyCamera",
    "DummyCameraConfig",
    "FrankaHandGripper",
    "FrankaInterfaceClient",
    "PgiGripper",
    "make_gripper_backend",
]
