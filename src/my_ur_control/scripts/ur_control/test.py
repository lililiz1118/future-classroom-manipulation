# import minimalmodbus
# import time

# gripper = minimalmodbus.Instrument('/dev/ttyUSB0', 9)
# gripper.serial.baudrate = 115200
# gripper.serial.timeout = 0.5
# gripper.mode = minimalmodbus.MODE_RTU

# def activate_gripper():
#     print("复位手爪...")
#     gripper.write_register(1000, 0)  # gACT = 0
#     time.sleep(0.5)

#     print("激活手爪...")
#     gripper.write_register(1000, 1)  # gACT = 1
#     time.sleep(0.5)

#     # 等待状态变为3 (Active)
#     while True:
#         sta = gripper.read_register(2002, 0)  # gSTA
#         print("当前状态:", sta)
#         if sta == 3:
#             print("✅ 手爪激活完成")
#             break
#         time.sleep(0.2)

# if __name__ == "__main__":
#     activate_gripper()

# import serial

# ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)

# ser.write(b'SET ACT 1\n')

# reply = ser.readline()
# print(reply.decode())

from Robotiq2F85Driver import Robotiq2F85Driver

# Initialize the driver with the gripper's serial number
gripper = Robotiq2F85Driver(serial_number='DA61P0BD')

# Reset the gripper
gripper.reset()

# Move the gripper to fully open position (opening = 85 mm)
# The motion is done at 150 mm/s with a force of up to 235 Newtons.
gripper.go_to(opening=85, speed=150, force=235)

# Get the current gripper opening
print(gripper.opening)