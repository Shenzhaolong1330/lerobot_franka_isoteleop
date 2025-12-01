from typing import Dict, Optional, Sequence, Tuple

import numpy as np
from .robot import Robot


class DynamixelRobot(Robot):
    """A class representing a Franka robot."""

    def __init__(
        self,
        port: str,
        hardware_offsets: Sequence[float],
        joint_ids: Sequence[int],
        joint_offsets: Sequence[float],
        joint_signs: Sequence[int],
        real: bool = False,
        baudrate: int = 57600,
        use_gripper: bool = True,
        gripper_config: Optional[Tuple[int, float, float]] = None,
    ):  
        from .driver import (
            DynamixelDriver,
            DynamixelDriverProtocol,
            FakeDynamixelDriver,
        )

        if joint_offsets is None or len(joint_offsets) != 7:
            raise ValueError(f"joint_offsets must be a sequence of length 7, got {joint_offsets}")

        if joint_signs is None or len(joint_signs) != 7:
            raise ValueError(f"joint_signs must be a sequence of length 7, got {joint_signs}")

        if joint_ids is None or len(joint_ids) != 7:
            raise ValueError(f"joint_ids must be a sequence of length 7, got {joint_ids}")


        self.gripper_open_close: Optional[Tuple[float, float]]

        if use_gripper and gripper_config is not None:
            assert joint_offsets is not None
            assert joint_signs is not None
            joint_ids = tuple(joint_ids) + (gripper_config[0],)
            joint_offsets = tuple(joint_offsets) + (0.0,)
            joint_signs = tuple(joint_signs) + (1,)
            self.gripper_open_close = (
                gripper_config[1],
                gripper_config[2],
            )
        else:
            self.gripper_open_close = None
        
        self._use_gripper = use_gripper
        self._hardware_offsets = hardware_offsets
        self._joint_ids = joint_ids
        self._joint_offsets = np.array(joint_offsets)
        self._joint_signs = np.array(joint_signs)
        self._driver: DynamixelDriverProtocol

        if real:
            self._driver = DynamixelDriver(joint_ids, port=port, baudrate=baudrate)
            # self._driver.set_torque_mode(False)
        else:
            self._driver = FakeDynamixelDriver(joint_ids)

        self._torque_on = False
        self._last_pos = None
        self._alpha = 1.0
        self.record_time = 0
        
    def num_dofs(self) -> int:
        return len(self._joint_ids)

    def get_joint_state(self) -> np.ndarray:
        pos = (self._driver.get_positions(self._hardware_offsets) - self._joint_offsets) * self._joint_signs
        assert len(pos) == self.num_dofs()
        if self.gripper_open_close is not None:
            # map pos to [0, 1]
            g_pos = (pos[-1] - self.gripper_open_close[0]) / (
                self.gripper_open_close[1] - self.gripper_open_close[0]
            )

            g_pos = min(max(0, g_pos), 1)
            # g_pos = 0.0 if g_pos < self._close_threshold else 1.0
            pos[-1] = g_pos
            
        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        self._driver.set_joints((joint_state / self._joint_signs + self._joint_offsets).tolist())

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def get_observations(self) -> Dict[str, np.ndarray]:
        """
        Return the current robot observations.

        If `self._use_gripper` is True, split the gripper state from joint_state.
        """
        obs_dict = {}
        joint_state = self.get_joint_state()
        for i in range(len(joint_state[:7])):
            obs_dict[f"joint_{i+1}.pos"] = joint_state[i]

        if not self._use_gripper:
            joint_state[-1] = None

        return {
            **obs_dict,
            "gripper_position": joint_state[-1],  
        }

