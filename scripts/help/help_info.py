def main() -> None:
    print(
        """
==================================================
 Franka Keyboard Teleoperation Utilities
==================================================

Core Commands:
  franka-start-arm       Start the Polymetis arm service (terminal 1)
  franka-start-gripper   Start Franka Hand service; PGI needs no service (terminal 2)
  franka-start-server    Start this repo's local ZeroRPC bridge (terminal 3)
  franka-record          Record a keyboard teleoperation dataset
  franka-replay          Replay a recorded dataset
  franka-visualize       Visualize a recorded dataset

Tool Commands:
  tools-check-dataset    Check local dataset integrity
  tools-check-info       Clean local dataset information
  tools-check-rs         List connected RealSense cameras
  tools-prune-dataset    Remove episodes into a new dataset
  tools-robot-state      Read Franka state through ZeroRPC
  test-franka-gripper    Test the selected PGI or Franka Hand gripper
  map_gripper.sh         Map the PGI serial device (default: /dev/franka_pgi_gripper)

Keyboard:
  W/S  -/+ X    A/D  -/+ Y    E/Q  -/+ Z
  T/R  -/+ RX   F/G  -/+ RY   V/B  -/+ RZ
  O open        L close        / toggle fine steps

Episode: Right finishes, Left re-records, Esc saves and exits, Enter advances reset.
==================================================
"""
    )


if __name__ == "__main__":
    main()
