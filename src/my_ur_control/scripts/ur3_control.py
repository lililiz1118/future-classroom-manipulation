import sys
import numpy as np
import cv2

import urx
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from scipy.spatial.transform import Rotation
import collections.abc
collections.Iterable = collections.abc.Iterable

from scipy.spatial.transform import Rotation as R



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

def quat2rotvec(quat):
    """
    四元数转旋转矢量
    输入:
        quat: [x, y, z, w] 或 numpy array
    输出:
        rotvec: numpy array [rx, ry, rz]，单位：弧度
    """
    x, y, z, w = quat
    norm = np.linalg.norm([x, y, z])
    
    if norm < 1e-8:
        # 没有旋转
        return np.array([0.0, 0.0, 0.0])
    
    theta = 2 * np.arccos(w)
    sin_half_theta = np.sqrt(1 - w*w)
    
    if sin_half_theta < 1e-8:
        # 接近0度旋转，方向随意
        return np.array([0.0, 0.0, 0.0])
    
    u = np.array([x, y, z]) / sin_half_theta
    rotvec = theta * u
    return rotvec

def transform_target_to_base_urx(robot, target_pos, target_quat, ee_frame_offset=(0,0,0,0,0,0)):
    """
    将目标点从末端坐标系转换到基座坐标系（使用 URX）
    
    参数:
        robot         : urx.Robot 实例
        target_pos    : [x, y, z] 目标点在末端坐标系下的位置
        target_quat   : [x, y, z, w] 目标点在末端坐标系下的四元数
        ee_frame_offset : TCP 偏移 (x, y, z, rx, ry, rz), 如果TCP已经设置为末端工具，可为(0,0,0,0,0,0)
    
    返回:
        pos_base, quat_base : 目标点在基座坐标系下的位置和四元数
    """

    # 获取当前末端位姿（flange / 当前TCP）
    ee_pose = robot.getl()  # 返回 [x, y, z, rx, ry, rz]，旋转向量
    ee_pos = np.array(ee_pose[:3])
    ee_rotvec = np.array(ee_pose[3:])
    ee_rot = R.from_rotvec(ee_rotvec)

    # 构造末端 -> 基座变换矩阵
    T_base_ee = np.eye(4)
    T_base_ee[:3, :3] = ee_rot.as_matrix()
    T_base_ee[:3, 3] = ee_pos

    # 如果 TCP 有偏移，应用 TCP 偏移
    if any(ee_frame_offset):
        # ee_frame_offset = (dx, dy, dz, rx, ry, rz)
        tcp_pos = np.array(ee_frame_offset[:3])
        tcp_rot = R.from_rotvec(ee_frame_offset[3:])
        T_ee_tcp = np.eye(4)
        T_ee_tcp[:3, :3] = tcp_rot.as_matrix()
        T_ee_tcp[:3, 3] = tcp_pos
        T_base_ee = T_base_ee @ T_ee_tcp

    # 目标在末端坐标系下的变换矩阵
    T_ee_target = np.eye(4)
    T_ee_target[:3, :3] = R.from_quat(target_quat).as_matrix()
    T_ee_target[:3, 3] = target_pos

    # 计算基座坐标系下的目标
    T_base_target = T_base_ee @ T_ee_target

    pos_base = T_base_target[:3, 3]
    quat_base = R.from_matrix(T_base_target[:3, :3]).as_quat()  # [x, y, z, w]

    return pos_base.tolist(), quat_base.tolist()


# 采集指代物品消歧数据集
# 机器人运动：（1）到达固定指定位置（2）闭合手爪，期间持续估计触觉深度，直到到达固定阈值（3）开始采集图像，夹爪闭合固定行程（4）手爪抬起固定高度
if __name__ == '__main__':

    home = [0.11245979240280754, -0.14673646872082813, 0.3710103194903948, 0.01721295221533764, -2.5858889969883125, 1.7573556978403213]
    robot = urx.Robot("192.168.131.3")
    # pose = robot.getl()
    # print(pose)   

    #  # 原四元数 [x, y, z, w] 或 [w, x, y, z]，注意 convention
    # q = [-0.68218814 , 0.11433799 ,-0.11937597 , 0.71224683]  # [x, y, z, w]

    # # 原四元数转旋转矩阵
    # r = R.from_quat(q)  # scipy 默认 [x,y,z,w]

    # # 坐标系变换 S (x,y取反)
    # S = np.diag([-1, -1, 1])

    # # 新旋转矩阵
    # R_new = S @ r.as_matrix()

    # # 转回四元数
    # q_new = R.from_matrix(R_new).as_quat()  # [x,y,z,w]

    # # roll, pitch, yaw = R_new.as_euler('xyz', degrees=True)
    # rovec = quat2rotvec(q_new)

    # pose[0] = 0.15410018
    # pose[1] = -0.44202951
    # pose[2] = 0.33368984
    # pose[3] = rovec[0]
    # pose[4] = rovec[1]
    # pose[5] = rovec[2]

    # # robot.movel(pose,
    # #                 acc=0.01,
    # #                 vel=0.05,
    # #                 wait=True,
    # #                 threshold=None)


    # robot.movel(home, acc=0.01,
    #                 vel=0.05,
    #                 wait=True,
    #                 threshold=None)



    
    robot.set_tcp((0,0,0,0,0,0))
    a, b = transform_target_to_base_urx(robot,[-0.0024256  ,-0.12052205  ,0.46764724], [ 0.15101877 ,-0.21389235 , 0.96094277 ,-0.08962362])
    print(a)
    print(b)
    robot.close()

    
