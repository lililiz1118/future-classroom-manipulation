import numpy as np
import cv2

import urx
import ur_control.base.rotations as rotations
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from scipy.spatial.transform import Rotation

class Robot(object):
    def __init__(self, tcp_host_ip, tcp_port):

        # home位置参数
        self.ur_home_pos = [-0.3713749756942525, 0.031154233492579717, 0.2679500656752863]
        self.ur_home_quat = [0.00833616, 0.68928081, -0.00361613, -0.7244373]

        # IP连接地址
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port

        # 默认关节参数
        self.joint_acc = 2.1  # Safe: 1.4
        self.joint_vel = 1.1  # Safe: 1.05
        self.joint_tolerance = 0.005

        # Default tool speed configuration
        self.tool_acc = 1.2  # Safe: 0.5
        self.tool_vel = 0.25  # Safe: 0.2

        # Tool pose tolerance for blocking calls
        self.tool_pose_tolerance = 0.005

        # 初始化位置
        # self.open_gripper()
        # self.go_home()

    def close_gripper(self):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        self.gripper = Robotiq_Two_Finger_Gripper(self.tcp_urx)
        self.gripper.close_gripper()
        self.tcp_urx.close()

    def open_gripper(self):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        self.gripper = Robotiq_Two_Finger_Gripper(self.tcp_urx)
        self.gripper.open_gripper()
        self.tcp_urx.close()

    def set_gripper(self, percentage):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        self.gripper = Robotiq_Two_Finger_Gripper(self.tcp_urx)
        self.gripper.gripper_action(int(percentage * 255.0 / 100.0))
        self.tcp_urx.close()

    def move_to(self, tool_position, tool_orientation, tool_acc=0.01, tool_vel=0.01, tool_wait=True, tool_pose_tolerance=None):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        self.tcp_urx.movel((tool_position[0], tool_position[1], tool_position[2], tool_orientation[0], tool_orientation[1], tool_orientation[2]), acc=tool_acc,
                           vel=tool_vel, wait=tool_wait, threshold=tool_pose_tolerance)
        self.tcp_urx.close()

    def get_pose(self):
        current_pos = robot.tcp_urx.getl()[:3]
        current_rotv = np.array(robot.tcp_urx.getl()[3:6])
        current_mat = cv2.Rodrigues(current_rotv)[0]
        current_quat = rotations.mat2quat(current_mat)
        return current_pos, current_quat

    def go_home(self):
        new_mat = rotations.quat2mat(self.ur_home_quat)
        orientation = np.transpose(cv2.Rodrigues(new_mat)[0])[0]
        self.move_to(self.ur_home_pos, orientation, tool_acc=0.1, tool_vel=0.2)

    def go_to_target(self, pos, quat):
        new_mat = rotations.quat2mat(quat)
        orientation = np.transpose(cv2.Rodrigues(new_mat)[0])[0]
        robot.move_to(pos, orientation, tool_acc=0.1, tool_vel=0.2)


# rpy转旋转矢量
def rpy2vec(rolldeg, pitchdeg, yawdeg):
    roll = np.deg2rad(rolldeg)
    pitch = np.deg2rad(pitchdeg)
    yaw = np.deg2rad(yawdeg)
    rotation_matrix = Rotation.from_euler('xyz', [yaw, pitch, roll]).as_matrix()
    rotation_vector = Rotation.from_matrix(rotation_matrix).as_rotvec()
    return rotation_vector


# 旋转矢量转rpy
def vec2rpy(rx, ry, rz):
    rotation_vector = np.array([rx, ry, rz])
    rotation_matrix = Rotation.from_rotvec(rotation_vector).as_matrix()
    rpy_angles = Rotation.from_matrix(rotation_matrix).as_euler('xyz')
    roll, pitch, yaw = np.rad2deg(rpy_angles)
    return [roll, pitch, yaw]


# 采集指代物品消歧数据集
# 机器人运动：（1）到达固定指定位置（2）闭合手爪，期间持续估计触觉深度，直到到达固定阈值（3）开始采集图像，夹爪闭合固定行程（4）手爪抬起固定高度
if __name__ == '__main__':
    # ur5初始化
    tcp_host_ip = '192.168.1.19'
    tcp_port = 30002
    workspace_limits = np.asarray([[-0.653, -0.2], [-0.224, 0.5], [-0.35, 0.326]])  # Cols: min max, Rows: x y z

    robot = Robot(tcp_host_ip, tcp_port)
    lift_position = [robot.ur_home_pos[0], robot.ur_home_pos[1], robot.ur_home_pos[2] + 0.04]
    robot.go_to_target(lift_position, robot.ur_home_quat)