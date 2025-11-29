from setuptools import setup, find_packages

setup(
    name="lerobot_robot_ur5e",
    version="0.0.1",
    description="LeRobot UR5e integration",
    author="Chenyu Su",
    author_email="suchenyu@mail.ustc.edu.cn",
    packages=find_packages(),
    install_requires=[
        "pyrealsense2",
        "franky-control"
    ],
    python_requires=">=3.10",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
)
