import logging
import threading
from typing import Any

import numpy as np

from .franka_interface_client import FrankaInterfaceClient


logger = logging.getLogger(__name__)


class FrankaHandGripper:
    name = "franka_hand"

    def __init__(
        self,
        client: FrankaInterfaceClient,
        *,
        max_open: float,
        speed: float,
        force: float,
        reverse: bool,
    ):
        self.client = client
        self.max_open = float(max_open)
        self.speed = float(speed)
        self.force = float(force)
        self.reverse = bool(reverse)

    def connect(self, init_gripper: bool) -> None:
        self.client.gripper_initialize()

    def command(self, logical_position: float, blocking: bool = False) -> None:
        command = float(np.clip(logical_position, 0.0, 1.0))
        physical = 1.0 - command if self.reverse else command
        self.client.gripper_goto(
            width=physical * self.max_open,
            speed=self.speed,
            force=self.force,
            blocking=blocking,
        )

    def read_normalized(self) -> float:
        state = self.client.gripper_get_state()
        physical = float(np.clip(float(state["width"]) / self.max_open, 0.0, 1.0))
        return 1.0 - physical if self.reverse else physical

    def read_state(self) -> dict[str, Any]:
        return self.client.gripper_get_state()

    def close(self) -> None:
        pass


class PgiGripper:
    """Direct serial driver for the DH Robotics PGI gripper."""

    name = "pgi"

    def __init__(
        self,
        *,
        port: str,
        speed: int,
        force: int,
        min_position: int,
        max_position: int,
        reverse: bool,
    ):
        self.port = port
        self.speed = int(speed)
        self.force = int(force)
        self.min_position = int(min_position)
        self.max_position = int(max_position)
        self.reverse = bool(reverse)
        self.gripper = None
        self._reader_thread: threading.Thread | None = None
        self._reader_stop = threading.Event()
        self._state_lock = threading.Lock()
        self._pending_logical_position: float | None = None
        self._cached_raw_position = float(self.max_position)
        self._cached_normalized_position = 1.0
        self._reader_error: Exception | None = None
        if not 0 <= self.min_position < self.max_position <= 1000:
            raise ValueError(
                "PGI position range must satisfy 0 <= min < max <= 1000, "
                f"got [{self.min_position}, {self.max_position}]."
            )

    def connect(self, init_gripper: bool) -> None:
        from pyDHgripper import PGE

        if init_gripper:
            self.gripper = PGE(port=self.port)
            self.gripper.init_feedback()
        else:
            import crcmod
            import serial

            self.gripper = PGE.__new__(PGE)
            self.gripper.ser = serial.Serial(
                port=self.port,
                baudrate=115200,
            )
            self.gripper.crc16 = crcmod.mkCrcFun(
                0x18005,
                rev=True,
                initCrc=0xFFFF,
                xorOut=0x0000,
            )

        self.gripper.set_force(self.force)
        self.gripper.set_vel(self.speed)
        logger.info(
            "[GRIPPER] PGI connected at %s; force=%d, speed=%d.",
            self.port,
            self.force,
            self.speed,
        )
        # Keep slow serial I/O off the recording thread.
        self._reader_stop.clear()
        self._reader_error = None
        self._reader_thread = threading.Thread(
            target=self._state_reader_loop,
            name="pgi-state-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def _require_gripper(self):
        if self.gripper is None:
            raise RuntimeError("PGI gripper is not connected.")
        return self.gripper

    def command(self, logical_position: float, blocking: bool = False) -> None:
        self._raise_reader_error()
        logical = float(np.clip(logical_position, 0.0, 1.0))
        # pyDHgripper still waits for a serial response in non-blocking mode.
        with self._state_lock:
            self._pending_logical_position = logical

    def read_normalized(self) -> float:
        self._require_gripper()
        self._raise_reader_error()
        with self._state_lock:
            return self._cached_normalized_position

    def _raise_reader_error(self) -> None:
        with self._state_lock:
            error = self._reader_error
        if error is not None:
            raise RuntimeError("PGI background state reader has stopped.") from error

    def _logical_to_raw(self, logical_position: float) -> int:
        physical = 1.0 - logical_position if self.reverse else logical_position
        return round(
            self.min_position
            + physical * (self.max_position - self.min_position)
        )

    def _state_reader_loop(self) -> None:
        gripper = self._require_gripper()
        try:
            while not self._reader_stop.is_set():
                with self._state_lock:
                    pending = self._pending_logical_position

                if pending is not None:
                    gripper.set_pos(
                        val=int(self._logical_to_raw(pending)),
                        blocking=False,
                    )
                    with self._state_lock:
                        if self._pending_logical_position == pending:
                            self._pending_logical_position = None

                raw_position = float(gripper.read_pos())
                normalized = self._normalize(raw_position)
                with self._state_lock:
                    self._cached_raw_position = raw_position
                    self._cached_normalized_position = normalized
                self._reader_stop.wait(0.01)
        except Exception as error:
            with self._state_lock:
                self._reader_error = error
            logger.exception("PGI background state reader stopped unexpectedly.")

    def _normalize(self, raw_position: float) -> float:
        physical = float(
            np.clip(
                (raw_position - self.min_position)
                / (self.max_position - self.min_position),
                0.0,
                1.0,
            )
        )
        return 1.0 - physical if self.reverse else physical

    def read_state(self) -> dict[str, Any]:
        self._require_gripper()
        self._raise_reader_error()
        with self._state_lock:
            raw_position = self._cached_raw_position
            normalized_position = self._cached_normalized_position
        return {
            "position": raw_position,
            "normalized_position": normalized_position,
            "state": None,
            "reader_error": None,
        }

    def close(self) -> None:
        if self.gripper is None:
            return
        self._reader_stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None
        serial_port = getattr(self.gripper, "ser", None)
        if serial_port is not None and getattr(serial_port, "is_open", False):
            serial_port.close()
        self.gripper = None


def make_gripper_backend(
    config,
    client: FrankaInterfaceClient | None = None,
):
    gripper_type = str(config.gripper_type).lower()
    if gripper_type == "franka_hand":
        if client is None:
            raise ValueError("A ZeroRPC client is required for Franka Hand.")
        return FrankaHandGripper(
            client,
            max_open=config.gripper_max_open,
            speed=config.gripper_speed,
            force=config.gripper_force,
            reverse=config.gripper_reverse,
        )
    if gripper_type == "pgi":
        return PgiGripper(
            port=config.gripper_port,
            speed=int(config.gripper_speed),
            force=int(config.gripper_force),
            min_position=config.gripper_min_position,
            max_position=config.gripper_max_position,
            reverse=config.gripper_reverse,
        )
    raise ValueError(
        "gripper_type must be 'franka_hand' or 'pgi', "
        f"got {config.gripper_type!r}."
    )
