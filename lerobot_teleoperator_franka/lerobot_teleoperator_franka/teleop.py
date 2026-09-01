import logging
import os
import sys
from queue import Empty, Queue
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_teleop import FrankaTeleopConfig


try:
    if "DISPLAY" not in os.environ and "linux" in sys.platform:
        raise ImportError("DISPLAY is not set.")
    from pynput import keyboard
except Exception as error:
    keyboard = None
    logging.info("Keyboard listener unavailable: %s", error)


ACTION_KEYS = ("delta_x", "delta_y", "delta_z", "delta_rx", "delta_ry", "delta_rz")
KEY_BINDINGS = {
    "s": ("delta_x", 1.0),
    "w": ("delta_x", -1.0),
    "d": ("delta_y", 1.0),
    "a": ("delta_y", -1.0),
    "q": ("delta_z", 1.0),
    "e": ("delta_z", -1.0),
    "r": ("delta_rx", 1.0),
    "t": ("delta_rx", -1.0),
    "g": ("delta_ry", 1.0),
    "f": ("delta_ry", -1.0),
    "b": ("delta_rz", 1.0),
    "v": ("delta_rz", -1.0),
}


class FrankaTeleop(Teleoperator):
    """Keyboard teleoperation for Franka Cartesian control."""

    config_class = FrankaTeleopConfig
    name = "franka_keyboard_ee"

    def __init__(self, config: FrankaTeleopConfig):
        super().__init__(config)
        self.config = config
        self.event_queue = Queue()
        self.current_pressed: dict[str, bool] = {}
        self.listener = None
        self.default_step_size = config.step_size
        self.default_rot_step_size = config.rot_step_size
        self.alternate_step_size = config.alternate_step_size
        self.alternate_rot_step_size = config.alternate_rot_step_size
        self.use_alternate_step_size = False
        self.step_size = self.default_step_size
        self.rot_step_size = self.default_rot_step_size
        self.reference_frame = config.reference_frame
        if self.reference_frame not in ("base", "tcp"):
            raise ValueError("reference_frame must be 'base' or 'tcp'.")
        self.control_frame_euler_deg = self._validate_control_frame(
            config.control_frame_euler_deg
        )
        self.control_to_base_rotation = R.from_euler(
            "xyz", self.control_frame_euler_deg, degrees=True
        ).as_matrix()
        self.select_vector = self._validate_select_vector(config.select_vector)
        self.robot = None
        self.gripper_action = self._initial_gripper_action()

    @property
    def action_features(self) -> dict:
        names = {name: index for index, name in enumerate(ACTION_KEYS)}
        if self.config.use_gripper:
            names["gripper_position"] = len(ACTION_KEYS)
        return {
            "dtype": "float32",
            "shape": (len(names),),
            "names": names,
        }

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return (
            keyboard is not None
            and isinstance(self.listener, keyboard.Listener)
            and self.listener.is_alive()
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError("Keyboard is already connected.")
        if keyboard is None:
            raise RuntimeError("pynput requires an active graphical DISPLAY.")
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self.listener.start()
        logging.info("Keyboard listener connected.")

    def disconnect(self) -> None:
        if self.listener is not None:
            self.listener.stop()
        self.listener = None
        self.current_pressed.clear()

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def set_robot(self, robot) -> None:
        self.robot = robot

    def _on_press(self, key) -> None:
        key_name = getattr(key, "char", None)
        if key_name is not None:
            self.event_queue.put((key_name, True))

    def _on_release(self, key) -> None:
        key_name = getattr(key, "char", None)
        if key_name is not None:
            self.event_queue.put((key_name, False))

    def _drain_key_events(self) -> list[tuple[str, bool, bool]]:
        events = []
        while True:
            try:
                key_name, is_pressed = self.event_queue.get_nowait()
            except Empty:
                return events
            was_pressed = self.current_pressed.get(key_name, False)
            self.current_pressed[key_name] = is_pressed
            events.append((key_name, is_pressed, was_pressed))

    def _initial_gripper_action(self) -> float:
        if not self.config.use_gripper or self.config.init_gripper:
            return 1.0
        while True:
            value = input("Initial gripper action (0 or 1): ").strip()
            if value in {"0", "1"}:
                return float(value)

    def _toggle_step_size(self) -> None:
        if self.alternate_step_size is None or self.alternate_rot_step_size is None:
            return
        self.use_alternate_step_size = not self.use_alternate_step_size
        if self.use_alternate_step_size:
            self.step_size = self.alternate_step_size
            self.rot_step_size = self.alternate_rot_step_size
            mode = "fine"
        else:
            self.step_size = self.default_step_size
            self.rot_step_size = self.default_rot_step_size
            mode = "default"
        logging.info(
            "====== [TELEOP] %s steps: step_size=%s, rot_step_size=%s ======",
            mode,
            self.step_size,
            self.rot_step_size,
        )

    def reset_step_size(self) -> None:
        self.use_alternate_step_size = False
        self.step_size = self.default_step_size
        self.rot_step_size = self.default_rot_step_size

    @staticmethod
    def _validate_select_vector(select_vector: list | None) -> list[float]:
        if select_vector is None:
            return [1.0] * len(ACTION_KEYS)
        if len(select_vector) != len(ACTION_KEYS):
            raise ValueError("select_vector must contain 6 values.")
        values = [float(value) for value in select_vector]
        if any(value not in (0.0, 1.0) for value in values):
            raise ValueError("select_vector values must be 0 or 1.")
        return values

    @staticmethod
    def _validate_control_frame(euler_deg: list | None) -> list[float]:
        if euler_deg is None:
            return [0.0, 0.0, 0.0]
        if len(euler_deg) != 3:
            raise ValueError("control_frame_euler_deg must contain 3 values.")
        return [float(value) for value in euler_deg]

    @staticmethod
    def _components(action: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
        position = np.asarray([action[key] for key in ACTION_KEYS[:3]], dtype=float)
        rotation = np.asarray([action[key] for key in ACTION_KEYS[3:]], dtype=float)
        return position, rotation

    @staticmethod
    def _action(position: np.ndarray, rotation: np.ndarray) -> dict[str, float]:
        return {
            key: float(value)
            for key, value in zip(
                ACTION_KEYS,
                np.concatenate((position, rotation)),
                strict=True,
            )
        }

    def _control_delta_to_base(self, action: dict[str, float]) -> dict[str, float]:
        position, rotation = self._components(action)
        frame = self.control_to_base_rotation
        position = frame @ position
        rotation_matrix = R.from_euler("xyz", rotation).as_matrix()
        rotation = R.from_matrix(frame @ rotation_matrix @ frame.T).as_euler("xyz")
        return self._action(position, rotation)

    def _base_delta_to_reference(self, action: dict[str, float]) -> dict[str, float]:
        if self.reference_frame == "base" or not any(action.values()):
            return action
        if self.robot is None:
            raise RuntimeError("TCP control requires a robot reference.")
        position, rotation = self._components(action)
        current_rotation = R.from_rotvec(self.robot.get_ee_pose()[3:]).as_matrix()
        position = current_rotation.T @ position
        rotation_matrix = R.from_euler("xyz", rotation).as_matrix()
        rotation = R.from_matrix(
            current_rotation.T @ rotation_matrix @ current_rotation
        ).as_euler("xyz")
        return self._action(position, rotation)

    def _to_reference_delta(self, action: dict[str, float]) -> dict[str, float]:
        masked = {
            key: float(action[key]) * self.select_vector[index]
            for index, key in enumerate(ACTION_KEYS)
        }
        return self._base_delta_to_reference(self._control_delta_to_base(masked))

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError("Keyboard is not connected.")

        for key_name, is_pressed, was_pressed in self._drain_key_events():
            if key_name == "/" and is_pressed and not was_pressed:
                self._toggle_step_size()

        action = {key: 0.0 for key in ACTION_KEYS}
        for key_name, is_pressed in self.current_pressed.items():
            if not is_pressed:
                continue
            if key_name in KEY_BINDINGS:
                axis, sign = KEY_BINDINGS[key_name]
                step = (
                    self.rot_step_size
                    if axis.startswith("delta_r")
                    else self.step_size
                )
                action[axis] += sign * step
            elif key_name == "o":
                self.gripper_action = 1.0
            elif key_name == "l":
                self.gripper_action = 0.0

        action = self._to_reference_delta(action)
        if self.config.use_gripper:
            action["gripper_position"] = self.gripper_action
        return action
