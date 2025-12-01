from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from lerobot.robots.config import RobotConfig

@RobotConfig.register_subclass("franka_robot")
@dataclass
class FrankaConfig(RobotConfig):
    use_gripper: bool = True
    gripper_reverse: bool = True
    robot_ip: str = "172.16.0.2"
    gripper_bin_threshold: float = 0.98
    debug: bool = True
    close_threshold: float = 0.7
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
