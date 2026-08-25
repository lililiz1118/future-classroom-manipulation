import sys
import numpy as np
import cv2
import yaml

import urx
import base.rotations as rotations
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from scipy.spatial.transform import Rotation
import collections.abc
collections.Iterable = collections.abc.Iterable

class Robot(object):
    def __init__(self, tcp_host_ip, tcp_port):

        # 顶抓且离开camera位置参数
        # self.ur_home_pos_away = [-0.31626809894045754, 0.03459522245492497, 0.4428282710382353]
        # self.ur_home_quat_away = [2.50169632e-03, -7.20176438e-01, 6.93786439e-01, 1.30164716e-04]
        self.ur_home_pos_away = [-0.11217987767528742, -0.05743890719380884, 0.4739855842984122]
        self.ur_home_quat_away = [0.01013265,-0.00382972 ,-0.96694831  ,0.25474228]

        # 顶抓home位置参数
        self.ur_home_pos_up = [-0.3619589415245894, -0.09754807560263709, 0.31419301299202335] # 螺丝顶抓参数20250912
        self.ur_home_quat_up = [0, -1, 0, 0]
        self.ur_home_quat_side = [0.02558357 ,0.02805693, -0.95135461 , 0.30574941]

        # IP连接地址
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port

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
    def move_to(self, tool_position, tool_orientation, tool_acc=0.01, tool_vel=0.01, tool_wait=True, tool_pose_tolerance=None):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        tpose = (
            float(tool_position[0]),
            float(tool_position[1]),
            float(tool_position[2]),
            float(tool_orientation[0]),
            float(tool_orientation[1]),
            float(tool_orientation[2]),
        )
        
        self.tcp_urx.movel(tpose,
                           acc=tool_acc,
                           vel=tool_vel,
                           wait=tool_wait,
                           threshold=tool_pose_tolerance)
        
        self.tcp_urx.close()

    def get_pose(self):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        pose = self.tcp_urx.getl()
        self.tcp_urx.close()
        return pose

    def go_home_up(self):
        new_mat = rotations.quat2mat(self.ur_home_quat_up)
        orientation = np.transpose(cv2.Rodrigues(new_mat)[0])[0]
        self.move_to(self.ur_home_pos_up, orientation, tool_acc=0.1, tool_vel=0.2)

    def go_home_away(self):
        new_mat = rotations.quat2mat(self.ur_home_quat_away)
        orientation = np.transpose(cv2.Rodrigues(new_mat)[0])[0]
        self.move_to(self.ur_home_pos_away, orientation, tool_acc=0.1, tool_vel=0.2)

    def go_home_side(self):
        new_mat = rotations.quat2mat(self.ur_home_quat_side)
        orientation = np.transpose(cv2.Rodrigues(new_mat)[0])[0]
        self.move_to(self.ur_home_pos_side, orientation, tool_acc=0.1, tool_vel=0.2)

    def go_to_target(self, pos, quat):
        new_mat = rotations.quat2mat(quat)
        orientation = np.transpose(cv2.Rodrigues(new_mat)[0])[0]
        robot.move_to(pos, orientation, tool_acc=0.1, tool_vel=0.02)


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
    with open('jason_vitai/parameters.yaml', 'r', encoding='utf-8') as f:
        paras = yaml.load(f.read(), Loader=yaml.FullLoader)
    axis = str(sys.argv[1])
    offset = float(sys.argv[2])
    # ur5初始化
    tcp_host_ip =paras['urip']
    tcp_port = paras['urport']
    workspace_limits = np.asarray([[-0.653, -0.2], [-0.224, 0.5], [-0.35, 0.326]])  # Cols: min max, Rows: x y z
    robot = Robot(tcp_host_ip, tcp_port)

    if axis == "up":
        robot.go_home_up()
    elif axis == "side":
        robot.go_home_side()
    elif axis == "away":
        robot.go_home_away()
    elif axis == "x":
        cur_pose = robot.get_pose()
        lift_position = [cur_pose[0] + offset, cur_pose[1], cur_pose[2]]
        current_rotv = np.array(cur_pose[3:6])
        current_mat = cv2.Rodrigues(current_rotv)[0]
        current_quat = rotations.mat2quat(current_mat)
        robot.go_to_target(lift_position, current_quat)
    elif axis == "y":
        cur_pose = robot.get_pose()
        lift_position = [cur_pose[0], cur_pose[1] + offset, cur_pose[2]]
        current_rotv = np.array(cur_pose[3:6])
        current_mat = cv2.Rodrigues(current_rotv)[0]
        current_quat = rotations.mat2quat(current_mat)
        robot.go_to_target(lift_position, current_quat)
    elif axis == "z":
        cur_pose = robot.get_pose()
        lift_position = [cur_pose[0], cur_pose[1], cur_pose[2] + offset]
        current_rotv = np.array(cur_pose[3:6])
        current_mat = cv2.Rodrigues(current_rotv)[0]
        current_quat = rotations.mat2quat(current_mat)
        robot.go_to_target(lift_position, current_quat)
