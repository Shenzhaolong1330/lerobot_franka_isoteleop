"""Resolve the FR3 URDF after applying the configured EE-to-TCP transform."""

from __future__ import annotations

import hashlib
import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Sequence


TCP_JOINT_NAME = "fr3v2_1_ee_to_tcp"
TCP_LINK_NAME = "fr3v2_1_tcp"


def tcp_xyz_rpy(model_config: dict) -> tuple[float, ...]:
    """Return [x, y, z, roll, pitch, yaw], with rotations in radians."""
    values = model_config.get("tcp_xyz_rpy", [0.0] * 6)
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError("services.robot_model.tcp_xyz_rpy must contain 6 values.")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("services.robot_model.tcp_xyz_rpy values must be finite.")
    return result


def _format_vector(values: Sequence[float]) -> str:
    return " ".join(format(float(value), ".17g") for value in values)


def write_tcp_urdf(
    source_path: Path,
    output_path: Path,
    pose_xyz_rpy: Sequence[float],
) -> Path:
    """Write a URDF whose fixed TCP joint uses ``pose_xyz_rpy``."""
    values = tuple(float(value) for value in pose_xyz_rpy)
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError("TCP pose must contain 6 finite values.")

    tree = ET.parse(source_path)
    root = tree.getroot()
    joint = root.find(f"./joint[@name='{TCP_JOINT_NAME}']")
    if joint is None:
        raise ValueError(f"TCP joint '{TCP_JOINT_NAME}' is missing from {source_path}.")
    origin = joint.find("origin")
    if origin is None:
        origin = ET.SubElement(joint, "origin")
    origin.set("xyz", _format_vector(values[:3]))
    origin.set("rpy", _format_vector(values[3:]))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


def resolve_robot_description(model_config: dict, repo_root: Path) -> Path:
    """Return the source URDF or a cached URDF with the YAML TCP override."""
    source_path = Path(model_config["urdf_path"]).expanduser()
    if not source_path.is_absolute():
        source_path = repo_root / source_path
    source_path = source_path.resolve()
    if not source_path.is_file():
        raise FileNotFoundError(f"FR3 v2.1 URDF not found: {source_path}")

    pose = tcp_xyz_rpy(model_config)
    if all(value == 0.0 for value in pose):
        return source_path

    digest = hashlib.sha256()
    digest.update(source_path.read_bytes())
    digest.update(repr(pose).encode("ascii"))
    output_path = (
        Path(tempfile.gettempdir())
        / "lerobot_franka_teleop"
        / f"fr3v2_1_tcp_{digest.hexdigest()[:16]}.urdf"
    )
    if not output_path.is_file():
        write_tcp_urdf(source_path, output_path, pose)
    return output_path
