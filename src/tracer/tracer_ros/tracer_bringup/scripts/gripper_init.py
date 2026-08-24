#!/usr/bin/env python3
"""DH AG-160-95 夹爪初始化（断电重启后运行一次即可）"""
import sys
sys.path.insert(0, '/home/jt001/tracer_ws/src/my_ur_control/scripts')
from gripper import AG95NoInit

PORT = '/dev/dh_gripper_usb'

print("[gripper_init] 正在初始化 DH AG-160-95 夹爪...")
g = AG95NoInit(port=PORT)
try:
    g.initialize()
    pos = g.read_pos()
    print(f"[gripper_init] ✅ 初始化完成，当前位置: {pos}")
finally:
    g.ser.close()
