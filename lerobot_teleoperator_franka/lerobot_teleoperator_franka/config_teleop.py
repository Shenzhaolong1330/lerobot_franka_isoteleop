from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("lerobot_teleoperator_franka")
@dataclass
class FrankaTeleopConfig(TeleoperatorConfig):
    port: str
    use_gripper: bool
    hardware_offsets: list[float]
    joint_ids: list[int]
    joint_offsets: list[float]
    joint_signs: list[int]
    gripper_config: tuple[int, float, float]
    control_mode: str = "isoteleop"
