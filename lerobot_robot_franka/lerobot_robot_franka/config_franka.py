from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("franka_robot")
@dataclass
class FrankaConfig(RobotConfig):
    use_gripper: bool = True
    init_gripper: bool = True
    gripper_type: str = "pgi"  # "pgi" or "franka_hand"
    gripper_port: str = "/dev/franka_pgi_gripper"
    gripper_reverse: bool = False
    server_host: str = "127.0.0.1"
    server_port: int = 4242
    gripper_force: float = 70.0
    gripper_speed: float = 60.0
    gripper_min_position: int = 0
    gripper_max_position: int = 1000
    gripper_max_open: float = 0.0801
    close_threshold: float = 0.7
    reference_frame: str = "base"  # "base" or "tcp"
    translation_axis_deadband: float = 0.0005
    rotation_axis_deadband: float = 0.0005
    select_vector: list = field(default_factory=lambda: [1, 1, 1, 1, 1, 1])
    control_frame_euler_deg: list = field(
        default_factory=lambda: [0.0, 0.0, 0.0]
    )
    cartesian_stiffness: list = field(
        default_factory=lambda: [750.0, 750.0, 750.0, 250.0, 250.0, 250.0]
    )
    cartesian_damping: list = field(
        default_factory=lambda: [37.0, 37.0, 37.0, 9.0, 9.0, 9.0]
    )
    init_pose: list = field(
        default_factory=lambda: [0.405813, 0.0, 0.441117, -3.107935, -0.032626, 1.539665]
    )
    init_pose_range: list = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    reset_time_to_go: float = 5.0
    max_translation_step: float = 0.05
    max_rotation_step: float = 0.2
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
