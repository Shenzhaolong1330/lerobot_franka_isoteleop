from pylibfranka import Robot as FrankaRobot
from pylibfranka import Gripper as FrankaGripper
from pylibfranka import ControllerMode, JointPositions

import argparse
import time

import numpy as np


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="172.16.0.2", help="Robot IP address")
    args = parser.parse_args()

    # Connect to robot
    robot = FrankaRobot(args.ip)

    try:
        # Set collision behavior
        lower_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        upper_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        lower_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        upper_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

        robot.set_collision_behavior(
            lower_torque_thresholds,
            upper_torque_thresholds,
            lower_force_thresholds,
            upper_force_thresholds,
        )

        # First move the robot to a suitable joint configuration
        print("WARNING: This example will move the robot!")
        print("Please make sure to have the user stop button at hand!")
        input("Press Enter to continue...")

        # Start joint position control with external control loop
        active_control = robot.start_joint_position_control(ControllerMode.JointImpedance)
        initial_position = [0.0] * 7
        time_elapsed = 0.0
        motion_finished = False

        robot_state, duration = active_control.readOnce()

        initial_position = robot_state.q_d if hasattr(robot_state, "q_d") else robot_state.q
        delta = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        new_positions = [
                initial_position[0]+delta[0],
                initial_position[1] + delta[1],
                initial_position[2]+delta[2],
                initial_position[3] + delta[3],
                initial_position[4] + delta[4],
                initial_position[5]+delta[5],
                initial_position[6] + delta[6],
            ]
        joint_positions = JointPositions(new_positions)
        active_control.writeOnce(joint_positions)
        joint_positions.motion_finished = True

        # initial_position = [0.0] * 7
        # time_elapsed = 0.0
        # motion_finished = False

        # # External control loop
        # while not motion_finished:
        #     # Read robot state and duration
        #     robot_state, duration = active_control.readOnce()

        #     # Update time
        #     time_elapsed += duration.to_sec()

        #     # On first iteration, capture initial position
        #     if time_elapsed <= duration.to_sec():
        #         initial_position = robot_state.q_d if hasattr(robot_state, "q_d") else robot_state.q

        #     # Calculate delta angle using the same formula as in C++ example
        #     delta_angle = np.pi / 8.0 * (1 - np.cos(np.pi / 2.5 * time_elapsed))
        #     print(f"Time elapsed: {time_elapsed:.4f} s, Delta angle: {delta_angle:.4f}")
        #     # delta_angle = np.pi / 4.0

        #     # Update joint positions
        #     new_positions = [
        #         initial_position[0],
        #         initial_position[1],
        #         initial_position[2],
        #         initial_position[3] + delta_angle,
        #         initial_position[4] + delta_angle,
        #         initial_position[5],
        #         initial_position[6] + delta_angle,
        #     ]
        #     print(f"Time elapsed: {time_elapsed:.4f} s, Target positions: {new_positions}")
        #     # Set joint positions
        #     joint_positions = JointPositions(new_positions)

        #     # Set motion_finished flag to True on the last update
        #     if time_elapsed >= 5.0:
        #         joint_positions.motion_finished = True
        #         motion_finished = True
        #         print("Finished motion, shutting down example")

        #     # Send command to robot
        #     active_control.writeOnce(joint_positions)

    except Exception as e:
        print(f"Error occurred: {e}")
        if robot is not None:
            robot.stop()
        return -1


if __name__ == "__main__":
    main()
