import logging
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_franka import FrankaConfig
from .franka_interface_client import FrankaInterfaceClient
from .grippers import make_gripper_backend


logger = logging.getLogger(__name__)
ACTION_KEYS = ("delta_x", "delta_y", "delta_z", "delta_rx", "delta_ry", "delta_rz")


class Franka(Robot):
    config_class = FrankaConfig
    name = "franka"

    def __init__(self, config: FrankaConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self._robot: FrankaInterfaceClient | None = None
        self._gripper_backend = None
        self._is_connected = False
        self._episode_reference_ee_pose: np.ndarray | None = None
        self._prev_observation: dict[str, Any] | None = None
        self._observation_fallback_used = False
        self._last_gripper_position = 1.0
        self._has_sent_gripper_command = False
        self._gripper_raw_position = 1.0
        self._hold_pose: list[float] | None = None
        self._previous_motion_mask = [False] * 6
        self._validate_config()
        self._selection_to_base_rotation = R.from_euler(
            "xyz",
            self.config.control_frame_euler_deg,
            degrees=True,
        ).as_matrix()

    def _validate_config(self) -> None:
        if self.config.reference_frame not in ("base", "tcp"):
            raise ValueError(
                f"reference_frame must be 'base' or 'tcp', got {self.config.reference_frame!r}."
            )
        for field_name in (
            "select_vector",
            "cartesian_stiffness",
            "cartesian_damping",
            "init_pose",
            "init_pose_range",
        ):
            values = getattr(self.config, field_name)
            if len(values) != 6:
                raise ValueError(f"{field_name} must contain 6 values, got {len(values)}.")
        if any(float(value) not in (0.0, 1.0) for value in self.config.select_vector):
            raise ValueError("select_vector values must be 0 or 1.")
        if len(self.config.control_frame_euler_deg) != 3:
            raise ValueError(
                "control_frame_euler_deg must contain 3 values, got "
                f"{len(self.config.control_frame_euler_deg)}."
            )
        if not np.all(np.isfinite(self.config.control_frame_euler_deg)):
            raise ValueError("control_frame_euler_deg values must be finite.")
        deadband = float(self.config.translation_axis_deadband)
        if not np.isfinite(deadband) or not 0.0 < deadband < self.config.max_translation_step:
            raise ValueError(
                "translation_axis_deadband must be finite, positive, and smaller "
                "than max_translation_step."
            )
        rotation_deadband = float(self.config.rotation_axis_deadband)
        if (
            not np.isfinite(rotation_deadband)
            or not 0.0 < rotation_deadband < self.config.max_rotation_step
        ):
            raise ValueError(
                "rotation_axis_deadband must be finite, positive, and smaller "
                "than max_rotation_step."
            )
        gripper_type = str(self.config.gripper_type).lower()
        if gripper_type not in ("franka_hand", "pgi"):
            raise ValueError("gripper_type must be 'franka_hand' or 'pgi'.")
        if gripper_type == "franka_hand" and self.config.gripper_max_open <= 0:
            raise ValueError("gripper_max_open must be positive.")
        if gripper_type == "pgi" and not (
            0
            <= self.config.gripper_min_position
            < self.config.gripper_max_position
            <= 1000
        ):
            raise ValueError("PGI positions must satisfy 0 <= min < max <= 1000.")

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self.is_connected

    @property
    def action_features(self) -> dict[str, type]:
        features = {key: float for key in ACTION_KEYS}
        if self.config.use_gripper:
            features["gripper_position"] = float
        return features

    @property
    def _state_features(self) -> dict[str, type]:
        axes = ("x", "y", "z", "rx", "ry", "rz")
        return {
            **{f"tcp_pose.{axis}": float for axis in axes},
            **{f"tcp_speed.{axis}": float for axis in axes},
            **{f"tcp_force.{axis}": float for axis in axes},
            "gripper_raw_position": float,
        }

    @property
    def _camera_features(self) -> dict[str, tuple[int, int, int]]:
        return {
            name: (camera.height, camera.width, 3)
            for name, camera in self.cameras.items()
        }

    @property
    def observation_features(self) -> dict[str, Any]:
        return {**self._state_features, **self._camera_features}

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self.name} is already connected.")

        logger.info("===== [ROBOT] Connecting to Franka ZeroRPC server =====")
        robot = FrankaInterfaceClient(
            host=self.config.server_host,
            port=self.config.server_port,
        )
        self._robot = robot
        self._is_connected = True
        self._has_sent_gripper_command = False
        self._prev_observation = None
        self._observation_fallback_used = False

        try:
            joint_positions = robot.robot_get_joint_positions()
            if joint_positions.shape != (7,):
                raise RuntimeError(
                    "Expected 7 Franka joint positions, "
                    f"got shape {joint_positions.shape}."
                )
            logger.info(
                "[ROBOT] Current joint positions: %s",
                np.round(joint_positions, 4).tolist(),
            )
            self._start_cartesian_controller()

            if self.config.use_gripper:
                self._gripper_backend = make_gripper_backend(
                    self.config,
                    self._robot,
                )
                self._gripper_backend.connect(self.config.init_gripper)
                self._update_gripper_state()

            for camera_name, camera in self.cameras.items():
                camera.connect()
                logger.info("[CAM] %s connected.", camera_name)
        except Exception:
            self._disconnect_partial()
            raise

        logger.info(
            "[ROBOT] Franka connected; Cartesian impedance keyboard control is ready."
        )

    def _require_robot(self) -> FrankaInterfaceClient:
        if not self.is_connected or self._robot is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        return self._robot

    def _start_cartesian_controller(self) -> None:
        if self._robot is None:
            raise DeviceNotConnectedError("Franka client is not connected.")
        self._robot.robot_start_cartesian_impedance_control(
            np.asarray(self.config.cartesian_stiffness, dtype=float),
            np.asarray(self.config.cartesian_damping, dtype=float),
        )
        self._reset_hold_state()

    def _terminate_controller_safely(self) -> None:
        if self._robot is None:
            return
        try:
            self._robot.robot_terminate_current_policy()
        except Exception as error:
            logger.debug("No active Franka policy to terminate: %s", error)

    def _disconnect_partial(self) -> None:
        self._disconnect_resources(warn=False)

    def _disconnect_resources(self, *, warn: bool) -> None:
        self._terminate_controller_safely()
        log_failure = logger.warning if warn else logger.debug
        for camera in self.cameras.values():
            try:
                if camera.is_connected:
                    camera.disconnect()
            except Exception:
                log_failure("Failed to close camera during cleanup.", exc_info=True)
        if self._gripper_backend is not None:
            try:
                self._gripper_backend.close()
            except Exception:
                log_failure("Failed to close gripper during cleanup.", exc_info=True)
            self._gripper_backend = None
        if self._robot is not None:
            try:
                self._robot.close()
            except Exception:
                log_failure("Failed to close ZeroRPC client during cleanup.", exc_info=True)
        self._robot = None
        self._is_connected = False
        self._episode_reference_ee_pose = None
        self._prev_observation = None
        self._observation_fallback_used = False
        self._reset_hold_state()

    def get_ee_pose(self) -> list[float]:
        return self._require_robot().robot_get_ee_pose().astype(float).tolist()

    @staticmethod
    def _pose_to_transform(pose: list[float] | np.ndarray) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = R.from_rotvec(pose[3:]).as_matrix()
        transform[:3, 3] = pose[:3]
        return transform

    @staticmethod
    def _transform_to_pose(transform: np.ndarray) -> list[float]:
        return [
            *transform[:3, 3].tolist(),
            *R.from_matrix(transform[:3, :3]).as_rotvec().tolist(),
        ]

    def _target_pose_from_action(
        self,
        action: dict[str, Any],
        current_pose: list[float] | np.ndarray | None = None,
    ) -> list[float]:
        if current_pose is None:
            current_pose = self.get_ee_pose()
        current_pose = np.asarray(current_pose, dtype=float)
        current_rotation = R.from_rotvec(current_pose[3:]).as_matrix()

        delta = np.array([float(action[key]) for key in ACTION_KEYS], dtype=float)
        translation_norm = np.linalg.norm(delta[:3])
        delta_rotation_reference = R.from_euler("xyz", delta[3:]).as_matrix()
        rotation_norm = np.linalg.norm(
            R.from_matrix(delta_rotation_reference).as_rotvec()
        )
        if translation_norm > self.config.max_translation_step:
            raise ValueError(
                f"Translation step {translation_norm:.4f} m exceeds "
                f"max_translation_step={self.config.max_translation_step:.4f} m."
            )
        if rotation_norm > self.config.max_rotation_step:
            raise ValueError(
                f"Rotation step {rotation_norm:.4f} rad exceeds "
                f"max_rotation_step={self.config.max_rotation_step:.4f} rad."
            )

        if self.config.reference_frame == "base":
            delta_position_base = delta[:3]
            delta_rotation_base = delta_rotation_reference
        else:
            delta_position_base = current_rotation @ delta[:3]
            delta_rotation_base = (
                current_rotation @ delta_rotation_reference @ current_rotation.T
            )

        # Apply axis selection in the fixed control frame.
        selection_to_base = self._selection_to_base_rotation
        select_vector = np.asarray(self.config.select_vector, dtype=float)
        delta_position_selection = selection_to_base.T @ delta_position_base
        delta_position_selection *= select_vector[:3]
        delta_position_base = selection_to_base @ delta_position_selection

        delta_rotation_selection = (
            selection_to_base.T @ delta_rotation_base @ selection_to_base
        )
        delta_rotation_vector_selection = R.from_matrix(
            delta_rotation_selection
        ).as_rotvec()
        delta_rotation_vector_selection *= select_vector[3:]
        delta_rotation_base = (
            selection_to_base
            @ R.from_rotvec(delta_rotation_vector_selection).as_matrix()
            @ selection_to_base.T
        )

        target_position = current_pose[:3] + delta_position_base
        target_rotation = delta_rotation_base @ current_rotation

        transform = np.eye(4)
        transform[:3, :3] = target_rotation
        transform[:3, 3] = target_position
        return self._transform_to_pose(transform)

    def _motion_delta_mask(
        self,
        current_pose: list[float],
        action_target_pose: list[float],
    ) -> list[bool]:
        # Ignore cross-axis residuals from separate state reads and transforms.
        position_mask = np.abs(
            np.asarray(action_target_pose[:3]) - np.asarray(current_pose[:3])
        ) >= self.config.translation_axis_deadband
        current_rotation = R.from_rotvec(np.asarray(current_pose[3:], dtype=float))
        action_target_rotation = R.from_rotvec(
            np.asarray(action_target_pose[3:], dtype=float)
        )
        rotation_delta = action_target_rotation * current_rotation.inv()
        rotation_active = (
            np.linalg.norm(rotation_delta.as_rotvec())
            >= self.config.rotation_axis_deadband
        )
        return [*position_mask.tolist(), rotation_active, rotation_active, rotation_active]

    def _merge_selected_orientation(
        self,
        held_pose: list[float],
        moving_pose: list[float],
    ) -> list[float]:
        """Lock disabled Euler axes in the control frame."""
        selection_to_base = R.from_matrix(self._selection_to_base_rotation)
        held_rotation = R.from_rotvec(np.asarray(held_pose[3:], dtype=float))
        moving_rotation = R.from_rotvec(np.asarray(moving_pose[3:], dtype=float))
        held_euler = (selection_to_base.inv() * held_rotation).as_euler("xyz")
        moving_euler = (selection_to_base.inv() * moving_rotation).as_euler("xyz")
        rotation_selection = np.asarray(self.config.select_vector[3:], dtype=bool)
        merged_euler = np.where(rotation_selection, moving_euler, held_euler)
        merged_rotation = selection_to_base * R.from_euler("xyz", merged_euler)
        return merged_rotation.as_rotvec().tolist()

    def _target_pose_from_impedance_action(
        self,
        action: dict[str, Any],
        current_pose: list[float],
    ) -> list[float]:
        action_target = self._target_pose_from_action(action, current_pose)
        if self._hold_pose is None:
            self._hold_pose = list(current_pose)

        motion_mask = self._motion_delta_mask(current_pose, action_target)

        # Hold the measured position when an axis is released.
        for index, (was_moving, is_moving) in enumerate(
            zip(self._previous_motion_mask[:3], motion_mask[:3])
        ):
            if was_moving and not is_moving:
                self._hold_pose[index] = current_pose[index]

        # Treat orientation as one group while locking disabled Euler axes.
        orientation_was_moving = self._previous_motion_mask[3]
        orientation_is_moving = motion_mask[3]
        if orientation_was_moving and not orientation_is_moving:
            self._hold_pose[3:] = self._merge_selected_orientation(
                self._hold_pose,
                current_pose,
            )

        if not any(motion_mask):
            self._previous_motion_mask = motion_mask
            return list(self._hold_pose)

        target_pose = list(self._hold_pose)
        for index, is_moving in enumerate(motion_mask[:3]):
            if is_moving:
                target_pose[index] = action_target[index]
                self._hold_pose[index] = current_pose[index]

        if orientation_is_moving:
            target_pose[3:] = self._merge_selected_orientation(
                self._hold_pose,
                action_target,
            )
            # Keep enabled angles one step ahead of the measured pose.
            self._hold_pose[3:] = self._merge_selected_orientation(
                self._hold_pose,
                current_pose,
            )

        self._previous_motion_mask = motion_mask
        return target_pose

    def _reset_hold_state(self) -> None:
        self._hold_pose = None
        self._previous_motion_mask = [False] * 6

    def _handle_gripper(self, logical_position: float) -> None:
        if not self.config.use_gripper:
            return

        command = 0.0 if logical_position < self.config.close_threshold else 1.0
        if not self._has_sent_gripper_command or command != self._last_gripper_position:
            if self._gripper_backend is None:
                raise RuntimeError("Gripper backend is not connected.")
            self._gripper_backend.command(command, blocking=False)
            self._last_gripper_position = command
            self._has_sent_gripper_command = True

    def _update_gripper_state(self) -> None:
        if not self.config.use_gripper:
            return
        if self._gripper_backend is None:
            raise RuntimeError("Gripper backend is not connected.")
        self._gripper_raw_position = self._gripper_backend.read_normalized()

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        robot = self._require_robot()
        missing_keys = [key for key in ACTION_KEYS if key not in action]
        if missing_keys:
            raise ValueError(
                "Keyboard Franka action is missing: " + ", ".join(missing_keys)
            )

        current_pose = robot.robot_get_ee_pose().astype(float).tolist()
        target_pose = self._target_pose_from_impedance_action(action, current_pose)
        robot.robot_update_desired_ee_pose(np.asarray(target_pose, dtype=float))
        if "gripper_position" in action:
            self._handle_gripper(float(action["gripper_position"]))
        return action

    @staticmethod
    def _pose_euler(pose: list[float] | np.ndarray) -> np.ndarray:
        pose = np.asarray(pose, dtype=float)
        result = np.zeros(6, dtype=float)
        result[:3] = pose[:3]
        result[3:] = R.from_rotvec(pose[3:]).as_euler("xyz")
        return result

    def _relative_pose_euler(self, pose: list[float] | np.ndarray) -> np.ndarray:
        if self._episode_reference_ee_pose is None:
            raise RuntimeError(
                "Episode reference EE pose is not set. "
                "Call set_episode_reference_pose() first."
            )
        relative_transform = (
            np.linalg.inv(self._pose_to_transform(self._episode_reference_ee_pose))
            @ self._pose_to_transform(pose)
        )
        result = np.zeros(6, dtype=float)
        result[:3] = relative_transform[:3, 3]
        result[3:] = R.from_matrix(relative_transform[:3, :3]).as_euler("xyz")
        return result

    def set_episode_reference_pose(self) -> None:
        self._episode_reference_ee_pose = np.asarray(self.get_ee_pose(), dtype=float)
        logger.info(
            "Set episode reference EE pose: %s",
            self._episode_reference_ee_pose.tolist(),
        )

    def get_observation(self) -> dict[str, Any]:
        robot = self._require_robot()
        try:
            ee_state = robot.robot_get_ee_state()
            ee_pose = ee_state["pose"]
            observation_pose = (
                self._pose_euler(ee_pose)
                if self.config.reference_frame == "base"
                else self._relative_pose_euler(ee_pose)
            )

            observation: dict[str, Any] = {}
            for index, axis in enumerate(("x", "y", "z", "rx", "ry", "rz")):
                observation[f"tcp_pose.{axis}"] = float(observation_pose[index])
                observation[f"tcp_speed.{axis}"] = float(ee_state["speed"][index])
                observation[f"tcp_force.{axis}"] = float(ee_state["wrench"][index])
            if self.config.use_gripper:
                self._update_gripper_state()
                observation["gripper_raw_position"] = self._gripper_raw_position
            else:
                observation["gripper_raw_position"] = 0.0

            for camera_name, camera in self.cameras.items():
                observation[camera_name] = camera.read()
        except Exception:
            if (
                self._prev_observation is not None
                and not self._observation_fallback_used
            ):
                self._observation_fallback_used = True
                logger.warning(
                    "Franka observation read failed; reusing the previous frame once."
                )
                return self._prev_observation
            raise

        self._observation_fallback_used = False
        self._prev_observation = observation
        return observation

    def stop_control(self) -> None:
        if not self.is_connected:
            return
        self._reset_hold_state()
        current_pose = np.asarray(self.get_ee_pose(), dtype=float)
        self._require_robot().robot_update_desired_ee_pose(current_pose)

    def _sample_init_pose(
        self,
        init_pose: list[float],
        init_pose_range: list[float],
    ) -> list[float]:
        if len(init_pose) != 6 or len(init_pose_range) != 6:
            raise ValueError("init_pose and init_pose_range must each contain 6 values.")

        target = np.asarray(init_pose, dtype=float).copy()
        ranges = np.abs(np.asarray(init_pose_range, dtype=float))
        target[:3] += np.random.uniform(-ranges[:3], ranges[:3])
        target_euler = target[3:] + np.deg2rad(
            np.random.uniform(-ranges[3:], ranges[3:])
        )
        target[3:] = R.from_euler("xyz", target_euler).as_rotvec()
        return target.tolist()

    def reset_to_init_pose(
        self,
        init_pose: list[float] | None = None,
        init_pose_range: list[float] | None = None,
    ) -> None:
        robot = self._require_robot()
        target_pose = self._sample_init_pose(
            init_pose if init_pose is not None else self.config.init_pose,
            init_pose_range
            if init_pose_range is not None
            else self.config.init_pose_range,
        )
        logger.info("Resetting Franka to configured initial EE pose.")
        target_pose_array = np.asarray(target_pose, dtype=float)
        self._terminate_controller_safely()
        try:
            trajectory_completed = robot.robot_move_to_ee_pose(
                pose=target_pose_array,
                time_to_go=self.config.reset_time_to_go,
                blocking=True,
            )
            if not trajectory_completed:
                raise RuntimeError(
                    "Unable to complete reset trajectory; check the reset IK target."
                )
        finally:
            self._terminate_controller_safely()
            self._start_cartesian_controller()
            current_pose = np.asarray(robot.robot_get_ee_pose(), dtype=float)
            robot.robot_update_desired_ee_pose(current_pose)
            self._reset_hold_state()
        logger.info("Reset trajectory completed.")

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        self._disconnect_resources(warn=True)
        logger.info("[INFO] ===== All Franka connections have been closed =====")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass
