from setuptools import find_packages, setup


setup(
    name="lerobot_franka_teleop",
    version="0.2.0",
    description="Franka keyboard teleoperation and dataset collection utilities",
    python_requires=">=3.10",
    packages=[
        *find_packages(where="lerobot_robot_franka"),
        *find_packages(where="lerobot_teleoperator_franka"),
        *find_packages(
            where=".",
            include=["scripts", "scripts.*"],
            exclude=["scripts.test", "scripts.test.*"],
        ),
    ],
    package_dir={
        "lerobot_robot_franka": "lerobot_robot_franka/lerobot_robot_franka",
        "lerobot_teleoperator_franka": (
            "lerobot_teleoperator_franka/lerobot_teleoperator_franka"
        ),
    },
    include_package_data=True,
    install_requires=[
        "numpy",
        "pynput",
        "pydhgripper",
        "pyrealsense2",
        "PyYAML",
        "scipy",
        "send2trash",
        "tqdm",
        "zerorpc",
    ],
    entry_points={
        "console_scripts": [
            "franka-start-arm = scripts.server.runtime:start_arm",
            "franka-start-gripper = scripts.server.runtime:start_gripper",
            "franka-start-server = scripts.server.runtime:start_server",
            "franka-record = scripts.core.run_record:main",
            "franka-replay = scripts.core.run_replay:main",
            "franka-visualize = scripts.core.run_visualize:main",
            "tools-check-rs = scripts.tools.rs_devices:list_realsense_devices",
            "tools-check-info = scripts.tools.check_dataset_info:main",
            "tools-check-dataset = scripts.tools.check_dataset:main",
            "tools-prune-dataset = scripts.tools.prune_episodes:main",
            "tools-robot-state = scripts.tools.read_robot_state:main",
            "test-franka-gripper = scripts.tools.gripper_ctrl:main",
            "franka-help = scripts.help.help_info:main",
        ],
    },
    scripts=["scripts/tools/map_gripper.sh"],
)
