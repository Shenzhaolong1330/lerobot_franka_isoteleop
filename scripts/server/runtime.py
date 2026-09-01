"""Launch the runtime services from this repository's console commands."""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import time
from pathlib import Path

import yaml

from scripts.server.robot_model_config import resolve_robot_description


REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = REPO_ROOT / "scripts" / "config" / "cfg.yaml"
ZERORPC_SERVER_PATH = Path(__file__).with_name("zerorpc_server.py")


def _load_config() -> dict:
    with CONFIG_PATH.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def _service_config() -> tuple[dict, dict]:
    config = _load_config()
    return config["services"], config["record"]["robot"]


def _fairo_script(services: dict, name: str) -> Path:
    fairo_root = Path(services["fairo_root"]).expanduser()
    script = fairo_root / "python" / "scripts" / name
    if not script.is_file():
        raise FileNotFoundError(
            f"Polymetis script not found: {script}. Check services.fairo_root in {CONFIG_PATH}."
        )
    return script


def _polymetis_command(
    services: dict,
    script: Path,
    *args: str,
) -> tuple[list[str], dict[str, str]]:
    command = [
        "conda",
        "run",
        "--no-capture-output",
        "-n",
        str(services.get("conda_env", "polymetis-local")),
        "python",
        str(script),
        *args,
    ]
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    environment["PYTHONNOUSERSITE"] = "1"
    return command, environment


def _exec_in_polymetis(services: dict, script: Path, *args: str) -> None:
    command, environment = _polymetis_command(services, script, *args)
    print("Running:", " ".join(command), flush=True)
    os.execvpe(command[0], command, environment)


def _tcp_port_open(host: str, port: int) -> bool:
    connect_host = "127.0.0.1" if host in {"0.0.0.0", "::"} else host
    try:
        with socket.create_connection((connect_host, port), timeout=0.1):
            return True
    except OSError:
        return False


def _stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    for process_signal, timeout in (
        (signal.SIGINT, 3.0),
        (signal.SIGTERM, 2.0),
        (signal.SIGKILL, 1.0),
    ):
        try:
            process.send_signal(process_signal)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def _find_run_server_pids(port: int) -> list[int]:
    pids = []
    for process_dir in Path("/proc").glob("[0-9]*"):
        try:
            command = (process_dir / "cmdline").read_bytes().split(b"\0")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if not command or Path(os.fsdecode(command[0])).name != "run_server":
            continue
        arguments = [os.fsdecode(value) for value in command[1:] if value]
        if any(
            argument == "-p"
            and index + 1 < len(arguments)
            and arguments[index + 1] == str(port)
            for index, argument in enumerate(arguments)
        ):
            pids.append(int(process_dir.name))
    return sorted(pids)


def _cleanup_polymetis_port(host: str, port: int) -> None:
    port_open = _tcp_port_open(host, port)
    pids = _find_run_server_pids(port)
    if not port_open and not pids:
        return
    print(f"Stopping Polymetis server on {host}:{port}...", flush=True)
    for signal_name, timeout in (("TERM", 2.0), ("KILL", 1.0)):
        pids = _find_run_server_pids(port)
        if not pids:
            break
        subprocess.run(
            ["sudo", "kill", f"-{signal_name}", *[str(pid) for pid in pids]],
            check=False,
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not _find_run_server_pids(port):
                break
            time.sleep(0.05)
    if _find_run_server_pids(port) or _tcp_port_open(host, port):
        raise RuntimeError(f"Failed to stop Polymetis server on {host}:{port}.")


def _run_arm_process(
    services: dict,
    script: Path,
    *args: str,
) -> None:
    arm = services["arm"]
    host = str(arm["bind_host"])
    port = int(arm["port"])
    if _tcp_port_open(host, port):
        raise RuntimeError(
            f"Polymetis port {host}:{port} is already in use. "
            "Stop the existing franka-start-arm process first."
        )

    command, environment = _polymetis_command(services, script, *args)
    print("Running:", " ".join(command), flush=True)
    process = subprocess.Popen(
        command,
        env=environment,
    )

    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def handle_sigterm(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, handle_sigterm)
    interrupted = False
    try:
        return_code = process.wait()
    except KeyboardInterrupt:
        interrupted = True
        return_code = 130
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        _stop_process(process)
        _cleanup_polymetis_port(host, port)

    if not interrupted and return_code != 0:
        raise SystemExit(return_code)


def _hydra_list(values: list[float]) -> str:
    return "[" + ",".join(str(float(value)) for value in values) + "]"


def _fr3_model_args(services: dict) -> list[str]:
    model = services["robot_model"]
    if model.get("type") != "fr3v2_1":
        raise ValueError("services.robot_model.type must be 'fr3v2_1'.")

    urdf_path = resolve_robot_description(model, REPO_ROOT)

    lower = [float(value) for value in model["joint_limits_low"]]
    upper = [float(value) for value in model["joint_limits_high"]]
    velocity = [float(value) for value in model["joint_velocity_limits"]]
    torque = [float(value) for value in model["torque_limits"]]
    if not all(len(values) == 7 for values in (lower, upper, velocity, torque)):
        raise ValueError("All FR3 v2.1 joint limit lists must contain 7 values.")

    # Keep the client safety limits just inside the robot limits.
    safety_lower = [value + 0.1 for value in lower]
    safety_upper = [value - 0.1 for value in upper]
    safety_velocity = [value - 0.1 for value in velocity]
    safety_torque = [
        value - (1.0 if index < 4 else 0.5)
        for index, value in enumerate(torque)
    ]

    return [
        f"robot_model.robot_description_path={urdf_path}",
        f"robot_model.ee_link_name={model['ee_link_name']}",
        f"robot_model.joint_limits_low={_hydra_list(lower)}",
        f"robot_model.joint_limits_high={_hydra_list(upper)}",
        f"robot_model.torque_limits={_hydra_list(torque)}",
        f"robot_client.executable_cfg.limits.joint_pos_lower={_hydra_list(safety_lower)}",
        f"robot_client.executable_cfg.limits.joint_pos_upper={_hydra_list(safety_upper)}",
        f"robot_client.executable_cfg.limits.joint_vel={_hydra_list(safety_velocity)}",
        f"robot_client.executable_cfg.limits.joint_torques={_hydra_list(safety_torque)}",
    ]


def start_arm() -> None:
    services, _ = _service_config()
    arm = services["arm"]
    _run_arm_process(
        services,
        _fairo_script(services, "launch_robot.py"),
        "robot_client=franka_hardware",
        f"robot_client.executable_cfg.robot_ip={services['franka_ip']}",
        "robot_client.executable_cfg.limit_rate="
        f"{str(bool(arm.get('limit_rate', True))).lower()}",
        "robot_client.executable_cfg.lpf_cutoff_frequency="
        f"{float(arm.get('lpf_cutoff_frequency', 100.0))}",
        f"ip={arm['bind_host']}",
        f"port={int(arm['port'])}",
        *_fr3_model_args(services),
    )


def start_gripper() -> None:
    services, robot = _service_config()
    gripper_type = str(robot.get("gripper", {}).get("type", "pgi")).lower()
    if gripper_type == "pgi":
        print(
            "PGI is driven directly over serial by franka-record; "
            "no separate gripper service is needed."
        )
        return
    if gripper_type != "franka_hand":
        raise ValueError("record.robot.gripper.type must be 'pgi' or 'franka_hand'.")

    hand = services["franka_hand"]
    _exec_in_polymetis(
        services,
        _fairo_script(services, "launch_gripper.py"),
        "gripper=franka_hand",
        f"gripper.executable_cfg.robot_ip={services['franka_ip']}",
        f"ip={hand['bind_host']}",
        f"port={int(hand['port'])}",
    )


def start_server() -> None:
    services, _ = _service_config()
    _exec_in_polymetis(
        services,
        ZERORPC_SERVER_PATH,
        "--config",
        str(CONFIG_PATH),
    )
