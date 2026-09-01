from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("lerobot_teleoperator_franka")
@dataclass
class FrankaTeleopConfig(TeleoperatorConfig):
    use_gripper: bool = True
    init_gripper: bool = True
    step_size: float = 0.006
    rot_step_size: float = 0.02
    alternate_step_size: float | None = None
    alternate_rot_step_size: float | None = None
    reference_frame: str = "base"
    control_frame_euler_deg: list = field(
        default_factory=lambda: [0.0, 0.0, 0.0]
    )
    select_vector: list | None = None
