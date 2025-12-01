#!/home/jt001/.conda/envs/global_3.10/bin/python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf.transformations as tft

import urx
import collections.abc
collections.Iterable = collections.abc.Iterable

import numpy as np
from scipy.spatial.transform import Rotation as R
import time

from vis_grasp import visualize_transform

def compensate_gripper_offset(target_xyz, target_q, gripper_in_ee_xyz, gripper_in_ee_q):
    """
    已知：
        target_xyz, target_q: 目标位姿 (希望夹爪到达的位姿)，base 下
        gripper_in_ee_xyz, gripper_in_ee_q: 夹爪在末端执行器下的位姿
    求：
        末端执行器应当移动到的位姿 (ee_in_base)，
        使得夹爪最终到达 target 位姿。

    所有四元数格式均为 [x, y, z, w]
    """

    def to_T(xyz, q):
        q = np.array(q, dtype=float)
        q /= np.linalg.norm(q)
        Rm = R.from_quat(q).as_matrix()
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = xyz
        return T

    def from_T(T):
        xyz = T[:3, 3]
        q = R.from_matrix(T[:3, :3]).as_quat()
        q /= np.linalg.norm(q)
        return xyz, q

    def invert_T(T):
        Rm = T[:3, :3]
        t = T[:3, 3]
        T_inv = np.eye(4)
        T_inv[:3, :3] = Rm.T
        T_inv[:3, 3] = -Rm.T @ t
        return T_inv

    # 转换为齐次矩阵
    T_base_target = to_T(target_xyz, target_q)
    T_ee_gripper = to_T(gripper_in_ee_xyz, gripper_in_ee_q)

    # 求新的末端执行器位姿
    T_base_ee_new = T_base_target @ invert_T(T_ee_gripper)

    # 返回平移和四元数
    return from_T(T_base_ee_new)

def transform_target_to_base(target_pos, target_quat, base_frame="base_link", ee_frame="tool0"):
    """
    将目标点从末端坐标系转换到机械臂基座坐标系

    参数:
        target_pos  : [x, y, z] 目标点在末端坐标系下的位置
        target_quat : [x, y, z, w] 目标点在末端坐标系下的四元数
        base_frame  : 基座坐标系名称 (默认 base_link)
        ee_frame    : 末端执行器坐标系名称 (默认 tool0)
    返回:
        (pos_base, quat_base) : 目标点在基座坐标系下的位置和四元数
    """

    # 初始化 TF 监听器
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    # 等待 TF 可用
    try:
        rospy.loginfo("Waiting for transform from %s to %s...", base_frame, ee_frame)
        tf_buffer.can_transform(base_frame, ee_frame, rospy.Time(), rospy.Duration(5.0))
        trans = tf_buffer.lookup_transform(base_frame, ee_frame, rospy.Time(0), rospy.Duration(3.0))
    except Exception as e:
        rospy.logerr("TF transform lookup failed: %s", e)
        return None, None

    # 提取 base->ee 的位姿
    t_base_ee = np.array([
        trans.transform.translation.x,
        trans.transform.translation.y,
        trans.transform.translation.z
    ])
    q_base_ee = [
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w
    ]

    # 构造变换矩阵
    T_base_ee = tft.quaternion_matrix(q_base_ee)
    T_base_ee[:3, 3] = t_base_ee

    T_ee_target = tft.quaternion_matrix(target_quat)
    T_ee_target[:3, 3] = target_pos

    # 计算 T_base_target
    T_base_target = np.dot(T_base_ee, T_ee_target)

    # 提取结果
    pos_base = T_base_target[:3, 3]
    quat_base = tft.quaternion_from_matrix(T_base_target)

    return pos_base.tolist(), quat_base.tolist()


def get_transform(target_frame, source_frame):
    """
    获取从 source_frame 到 target_frame 的变换关系。
    这是一个简化的、自包含的函数。

    Args:
        target_frame (str): 目标坐标系 (即坐标系1)
        source_frame (str): 源坐标系 (即坐标系2)

    Returns:
        tuple: 一个包含(translation, quaternion)的元组。
               如果查找失败，则返回 (None, None)。
               translation 是 (x, y, z) 的元-组。
               quaternion 是 (x, y, z, w) 的元组。
    """
    try:
        # 等待并查询TF变换，超时时间为1秒
        # rospy.Time() 表示查询最新的可用变换
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        tf_buffer.can_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(5.0))
        trans = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(3.0))
        
        # 提取笛卡尔坐标 (位置)
        translation = (
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        )
        
        # 提取四元数 (姿态)
        quaternion = (
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        )
        
        return translation, quaternion

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("获取坐标变换失败: 从 '%s' 到 '%s' - %s", source_frame, target_frame, e)
        return None, None


def transform_target_to_ee(t_cam_ee, q_cam_ee, t_target_cam, q_target_cam):
    """
    将目标点从相机坐标系转换到末端执行器坐标系
    输入:
        t_cam_ee: [x, y, z] 相机相对于末端执行器的平移 (相机在ee坐标下)
        q_cam_ee: [x, y, z, w] 相机相对于末端执行器的旋转
        t_target_cam: [x, y, z] 目标在相机坐标系下的位置
        q_target_cam: [x, y, z, w] 目标在相机坐标系下的旋转
    输出:
        t_target_ee: [x, y, z] 目标在末端执行器坐标系下的位置
        q_target_ee: [x, y, z, w] 目标在末端执行器坐标系下的旋转
    """
    # 相机在末端坐标系下的旋转和平移
    R_cam_ee = R.from_quat(q_cam_ee).as_matrix()
    t_cam_ee = np.array(t_cam_ee)
    
    # 目标在相机坐标系下的旋转和平移
    R_target_cam = R.from_quat(q_target_cam).as_matrix()
    t_target_cam = np.array(t_target_cam)
    
    # 坐标转换：目标在末端坐标系下
    R_target_ee = R_cam_ee @ R_target_cam
    t_target_ee = R_cam_ee @ t_target_cam + t_cam_ee
    
    q_target_ee = R.from_matrix(R_target_ee).as_quat()
    return t_target_ee, q_target_ee


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



def align_y_down_around_z(q_initial: np.ndarray) -> np.ndarray:
    """
    对一个由四元数表示的坐标系进行操作，绕其自身的Z轴旋转，
    使得其Y轴在世界坐标系中竖直朝下。

    假设:
    - 世界坐标系为Z轴朝上, X轴朝前, Y轴朝左 (标准右手系)。
    - "竖直朝下" 意味着指向世界坐标系的-Z方向, 即 [0, 0, -1]。
    - 输入和输出的四元数格式为 [x, y, z, w]。

    Args:
        q_initial (np.ndarray): 初始姿态的四元数 [x, y, z, w]。

    Returns:
        np.ndarray: 旋转后得到的目标姿态的四元数 [x, y, z, w]。
        
    Raises:
        ValueError: 如果初始Z轴已经接近垂直（与世界Z轴平行），
                    此时无法通过绕Z轴旋转来使Y轴垂直。
    """
    # 1. 从初始四元数创建旋转对象
    r_initial = R.from_quat(q_initial)
    
    # 2. 计算初始坐标系的Z轴在世界坐标系中的方向
    #    这个方向在旋转后保持不变，所以它也是最终坐标系的Z轴方向。
    z_axis_final = r_initial.apply([0, 0, 1])
    
    # 3. 定义目标Y轴在世界坐标系中的方向（竖直朝下）
    y_axis_target = np.array([0, 0, -1.0])
    
    # --- 检查几何约束 ---
    # 检查初始Z轴是否与世界Z轴（或目标Y轴）平行。
    # 如果是，它们之间的叉乘会是零向量，无法构建坐标系。
    # 这意味着从物理上讲，如果Z轴是垂直的，你无法通过绕它旋转来让Y轴也变得垂直。
    if np.allclose(np.abs(np.dot(z_axis_final, y_axis_target)), 1.0):
        raise ValueError(
            "初始Z轴已接近垂直，无法通过绕其自身旋转来使Y轴变为垂直。"
            "该操作在几何上是不可能的。"
        )
        
    # 4. 构建新的标准正交基（新的坐标系）
    #    - 新的X轴垂直于新的Z轴和目标Y轴
    #    - 利用叉乘保证右手系规则
    x_axis_final = np.cross(y_axis_target, z_axis_final)
    x_axis_final /= np.linalg.norm(x_axis_final)  # 归一化
    
    #    - 新的Y轴必须同时垂直于新的Z轴和新的X轴
    y_axis_final = np.cross(z_axis_final, x_axis_final)
    y_axis_final /= np.linalg.norm(y_axis_final) # 归一化（理论上已是单位向量，为保精度）

    # 5. 从三个正交基向量构建最终的旋转矩阵
    #    np.column_stack 将三个列向量合并成一个3x3矩阵
    rotation_matrix_final = np.column_stack([x_axis_final, y_axis_final, z_axis_final])
    
    # 6. 将最终的旋转矩阵转换为四元数
    r_final = R.from_matrix(rotation_matrix_final)
    q_final = r_final.as_quat()
    
    return q_final



def adjust_rotation_if_y_acute_to_z(q_input):
    """
    检查目标坐标系的Y轴与基坐标系Z轴的夹角。
    如果夹角 < 90度，将目标坐标系绕自身的Z轴旋转180度。

    Args:
        q_input (list or np.array): 输入的四元数 [x, y, z, w]

    Returns:
        np.array: 调整后（或保持原样）的四元数 [x, y, z, w]
    """
    # 1. 创建旋转对象
    r_current = R.from_quat(q_input)

    # 2. 获取目标坐标系的Y轴在基坐标系下的向量
    # 方法：将目标系的Y轴单位向量 [0, 1, 0] 通过当前的旋转变换到基坐标系
    y_axis_local = np.array([0, 1, 0])
    y_axis_in_base = r_current.apply(y_axis_local)

    # 3. 定义基坐标系的Z轴
    z_axis_base = np.array([0, 0, 1])

    # 4. 计算点积判定夹角
    # dot_product > 0 意味着 cos(theta) > 0，即 -90 < theta < 90
    # 在 0-180 范围内，意味着夹角小于 90 度
    dot_product = np.dot(y_axis_in_base, z_axis_base)

    if dot_product > 0:
        # print(f"检测到夹角小于90度 (点积={dot_product:.4f})，执行绕自身Z轴旋转180度...")
        
        # 5. 生成绕 Z 轴旋转 180 度的旋转对象
        r_z180 = R.from_euler('z', 180, degrees=True)
        
        # 6. 组合旋转
        # 注意：在 scipy 中，r1 * r2 表示先执行 r1，再在 r1 的局部坐标系下执行 r2
        # 这正是题目要求的 "绕其自己的 Z 轴旋转"
        r_final = r_current * r_z180
        
        return r_final.as_quat()
    else:
        # print(f"夹角大于等于90度 (点积={dot_product:.4f})，保持原姿态。")
        return np.array(q_input)

#四元数转旋转矢量
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


# 初始位姿定义
home = [0.11292229037967165, -0.07696327633952085, 0.39864630828077036, 0.016839013024772205, -2.5858549211986275, 1.75758600019715]
home_j = [1.6062464714050293, -2.692817989979879, 2.243985652923584, -2.314871613179342, -1.6063764731036585, 0.00013182648399379104]
# 标定得到相机相对于末端执行器
ee_cam_xyz = np.array([-0.009597337799696761, -0.07408851538404479, 0.01670505075735617])
ee_cam_q = np.array([0.0006344396200247934, -6.58198103146157e-05, 0.006961144575743825, 0.999975567511685])    

if __name__ == '__main__':

    # 连接机械臂
    robot = None
    while robot is None:
        try:
            robot = urx.Robot("192.168.131.3")
        except Exception:
            time.sleep(1) 

    # 设置 TCP
    robot.set_tcp((0, 0, 0, 0, 0, 0))  # TCP 坐标偏移
    # 由抓姿模型给出的输入目标位置
    cam_tar_xyz = np.array( [-0.07950325  ,0.0284377 ,  0.32099992])
    cam_tar_r = np.array(
[[ 0.0568925  ,-0.9877405 , -0.14536794],
 [-0.43903548,  0.10601813, -0.89219284],
 [ 0.8966667  , 0.11458076, -0.42762148]]
    )

     # 抓姿坐标系转换:graspnet定义的抓姿坐标系和实际夹爪抓姿坐标系不同
    theta_y = np.deg2rad(90)
    Ry_local = np.array([
        [np.cos(theta_y), 0, np.sin(theta_y)],
        [0, 1, 0],
        [-np.sin(theta_y), 0, np.cos(theta_y)]
    ])

    theta_z = np.deg2rad(90)
    Rz_local = np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [np.sin(theta_z),  np.cos(theta_z), 0],
        [0, 0, 1]
    ])

    R_final_matrix = cam_tar_r @ Ry_local @ Rz_local

    r = R.from_matrix(R_final_matrix)
    cam_tar_q = r.as_quat()

    # 相机坐标系下的目标位置
    print("taregt under camera")
    print(cam_tar_xyz)
    print(cam_tar_q)

    # 末端执行器坐标系下的目标位置(z轴超前，y轴朝下， x轴朝右)
    effort_tar_xyz, effort_tar_q = transform_target_to_ee(ee_cam_xyz, ee_cam_q, cam_tar_xyz, cam_tar_q)
    print("taregt under end effort")
    print(effort_tar_xyz)
    print(effort_tar_q)

    # 机械臂base下目标位置(z轴超上，y轴朝后， x轴朝左)
    base_tar_xyz, base_tar_q = transform_target_to_base_urx(robot, effort_tar_xyz, effort_tar_q)
    # base_tar_q_new = align_y_down_around_z(base_tar_q) # 保证姿态正确
    print("taregt under arm base")
    print(f"xyz:{base_tar_xyz}")
    print(f"q:{base_tar_q}")

    # 末端执行器转夹爪, 控制加爪到位需要给ee作补偿
    gripper_xyz, gripper_q = compensate_gripper_offset(base_tar_xyz, base_tar_q, [0 ,0 ,0.15], [0, 0 ,0 , 1])
    # 夹爪姿态调整
    # gripper_q_new= align_y_down_around_z(gripper_q)
    print("taregt under arm base")
    print(f"xyz:{gripper_xyz}")
    print(f"q:{gripper_q}")

    # 对夹爪姿态作处理，保持相机在上方
    gripper_q_adjust = adjust_rotation_if_y_acute_to_z(gripper_q)

    # visualize_transform(gripper_xyz, gripper_q_adjust)

    # 四元数转旋转矢量
    rovec = quat2rotvec(gripper_q_adjust)

    pose1 = robot.getl()
    print("current ee pose:")
    print(f"xyz_r1_r2_r3{pose1}")

    pose = pose1
    pose[0] = float(gripper_xyz[0])
    pose[1] = float(gripper_xyz[1])
    pose[2] = float(gripper_xyz[2])
    pose[3] = float(rovec[0])
    pose[4] = float(rovec[1])
    pose[5] = float(rovec[2])

    print("target ee pose")
    print(f"xyz_r1_r2_r3{pose}")

    robot.movel(pose,
                    acc=0.01,
                    vel=0.1,
                    wait=True,
                    threshold=None)
    time.sleep(4)
    
    # print("done")
    # robot.movel(home, 
    #                 acc=0.01,
    #                 vel=0.1,
    #                 wait=None,
    #                 threshold=None)
    

    # /////需要重新标定！！！！！

    
    robot.movej(home_j,
                    acc=0.05,
                    vel=0.2,
                    wait=True,
                    threshold=None)
    robot.close()








   
    







    
