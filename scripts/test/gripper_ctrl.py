from pyDHgripper import PGE

def get_vel(gripper):
    return gripper.write_uart(modbus_high_addr=0x01,
                          modbus_low_addr=0x04,
                          is_set=False)
def main():
    gripper = PGE("/dev/franka_left_gripper")
    gripper.init_feedback()
    gripper.set_force(100)
    gripper.set_vel(100)
    print(f"gripper_vel: {get_vel(gripper)}")

    while True:
        val = input("enter: ")
        gripper.set_pos(val=int(val), blocking=False)

