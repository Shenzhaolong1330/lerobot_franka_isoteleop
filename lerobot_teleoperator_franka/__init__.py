import sys

from .lerobot_teleoperator_franka import FrankaTeleop, FrankaTeleopConfig
from .lerobot_teleoperator_franka import config_teleop as _config_teleop
from .lerobot_teleoperator_franka import teleop as _teleop

sys.modules[__name__ + ".config_teleop"] = _config_teleop
sys.modules[__name__ + ".teleop"] = _teleop

__all__ = ["FrankaTeleop", "FrankaTeleopConfig"]
