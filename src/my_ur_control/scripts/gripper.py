import time

import crcmod
import serial
from pyDHgripper import AG95


PORT = "/dev/dh_gripper_usb"
BAUDRATE = 115200
MAX_POS = 1000
MIN_POS = 0
START_POS = 1000
STEP_POS = 70
STEP_DELAY = 0.02
CLOSE_VEL = 200
CURRENT_THRESHOLD = 800


class AG95NoInit(AG95):
    def __init__(self, port=PORT):
        self.ser = serial.Serial(port=port, baudrate=BAUDRATE)
        self.crc16 = crcmod.mkCrcFun(
            0x18005,
            rev=True,
            initCrc=0xFFFF,
            xorOut=0x0000,
        )
        self._initialized = False

    def initialize(self):
        """手动初始化夹爪（断电重启后必须调用）"""
        if self._initialized:
            return
        self.init_state()       # 写 0xA5 到 0x0100
        self.init_feedback()    # 等待 0x0200 != 0 且 != 2
        self._initialized = True

    def read_register(self, modbus_high_addr, modbus_low_addr):
        command = [0x01, 0x03, modbus_high_addr, modbus_low_addr, 0x00, 0x01]
        crc_l, crc_h = self.cal_crc(command)
        command.extend([crc_l, crc_h])

        self.ser.reset_input_buffer()
        self.ser.write(command)
        time.sleep(0.08)
        response = self.ser.read_all()

        if len(response) < 5:
            raise RuntimeError(f"invalid response: {list(response)}")

        return response[3:5]


def read_current(gripper):
    data_bytes = gripper.read_register(0x02, 0x04)
    return int.from_bytes(data_bytes, byteorder="big", signed=True)


def slow_close_until_current(gripper, start_pos=START_POS):
    gripper.set_vel(CLOSE_VEL)
    gripper.set_pos(start_pos)
    time.sleep(10.5)

    for target_pos in range(start_pos - STEP_POS, MIN_POS - 1, -STEP_POS):
        gripper.set_pos(target_pos)
        # time.sleep(STEP_DELAY)

        current_pos = gripper.read_pos()
        current_raw = read_current(gripper)
        print(
            f"target_pos={target_pos}, "
            f"current_pos={current_pos}, "
            f"current_raw={current_raw}"
        )

        if current_raw >= CURRENT_THRESHOLD:
            print(
                f"grasp detected: current_raw={current_raw} "
                f">= threshold={CURRENT_THRESHOLD}, stop closing"
            )
            return current_pos, current_raw

    print("max position reached, current threshold not met")
    return gripper.read_pos(), read_current(gripper)


if __name__ == "__main__":
    
    gripper = AG95NoInit(port=PORT)
    final_pos, final_current = slow_close_until_current(gripper)
    print(f"final_pos={final_pos}, final_current={final_current}")
