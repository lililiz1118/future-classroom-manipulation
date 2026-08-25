import numpy as np
import cv2
import sys
import os
import yaml

# 将上一级目录加入 sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import urx
import base.rotations as rotations
from scipy.spatial.transform import Rotation
import collections.abc
collections.Iterable = collections.abc.Iterable

class Robot(object):
    def __init__(self, tcp_host_ip, tcp_port):
        # IP连接地址
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port

    def get_pose(self):
        # current_pos = robot.tcp_urx.getl()[:3]
        # current_rotv = np.array(robot.tcp_urx.getl()[3:6])
        # current_mat = cv2.Rodrigues(current_rotv)[0]
        # current_quat = rotations.mat2quat(current_mat)
        # return current_pos, current_quat
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        pose = self.tcp_urx.getl()
        self.tcp_urx.close()
        return pose


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


if __name__ == '__main__':

    with open('jason_vitai/parameters.yaml', 'r', encoding='utf-8') as f:
        paras = yaml.load(f.read(), Loader=yaml.FullLoader)

    # ur5初始化
    tcp_host_ip =paras['urip']
    tcp_port = paras['urport']
    workspace_limits = np.asarray([[-0.653, -0.2], [-0.224, 0.5], [-0.35, 0.326]])
    robot = Robot(tcp_host_ip, tcp_port)

    # 获取位姿
    cur_pose = robot.get_pose()

    # 计算四元数
    current_rotv = np.array(cur_pose[3:6])
    current_mat = cv2.Rodrigues(current_rotv)[0]
    current_quat = rotations.mat2quat(current_mat)
    print("pose:", cur_pose)
    print("quat:", current_quat)
