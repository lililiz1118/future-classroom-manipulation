#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread
import time

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

cur_odom_to_baselink = None
cur_map_to_odom = None

# --- NEW ADDED: 全局变量存储上一时刻的平滑变换矩阵 ---
last_smooth_map_to_odom = None 
# ------------------------------------------------

# ================= 配置区域 =================
# 定义雷达到底盘的静态变换 (T_body_to_base)
# 也就是：在"雷达坐标系"下，"底盘中心"在哪里？
# 如果雷达在底盘前方 0.3m (x=0.3)，那么底盘就在雷达后方 0.3m (x=-0.3)
LIDAR_TO_BASE_X = -0.255
LIDAR_TO_BASE_Y = 0
LIDAR_TO_BASE_Z = -0.022
# T_body_to_base
T_body_to_base = tf.transformations.translation_matrix(
    [LIDAR_TO_BASE_X, LIDAR_TO_BASE_Y, LIDAR_TO_BASE_Z])


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )

# --- NEW ADDED: 插值函数 ---
def interpolate_transform(T_old, T_new, alpha):
    """
    对两个变换矩阵进行插值
    T_old, T_new: 4x4 矩阵
    alpha: 更新系数 [0, 1]，alpha越大越接近 T_new
    """
    # 1. 位置插值 (线性)
    pos_old = tf.transformations.translation_from_matrix(T_old)
    pos_new = tf.transformations.translation_from_matrix(T_new)
    pos_cur = pos_old * (1.0 - alpha) + pos_new * alpha

    # 2. 旋转插值 (Slerp - 球面线性插值)
    quat_old = tf.transformations.quaternion_from_matrix(T_old)
    quat_new = tf.transformations.quaternion_from_matrix(T_new)
    # Slerp 处理四元数，保证旋转路径最短且匀速
    quat_cur = tf.transformations.quaternion_slerp(quat_old, quat_new, alpha)

    # 3. 合成新矩阵
    T_cur = tf.transformations.quaternion_matrix(quat_cur)
    T_cur[0:3, 3] = pos_cur
    return T_cur
# -------------------------


def transform_fusion():
    global cur_odom_to_baselink, cur_map_to_odom, last_smooth_map_to_odom
    global pub_odom_to_baselink, pub_localization

    # --- 参数设置 ---
    # 跳变阈值 (米): 超过此值认为是漂移/重定位，快速更新；小于此值认为是噪声，慢速平滑
    JUMP_THRESHOLD = 0.1 
    # 平滑系数: 小于阈值时的更新速率 (0.0~1.0)。越小越平滑，但延迟越高
    SMOOTH_ALPHA = 0.01
    # 重置系数: 大于阈值时的更新速率。1.0表示立即接受，0.8表示稍微平滑一下大跳变
    RESET_ALPHA = 1.0    
    # ---------------

    br = tf.TransformBroadcaster()
    while True:
        time.sleep(1 / FREQ_PUB_LOCALIZATION)

        # TODO 这里注意线程安全
        cur_odom = copy.copy(cur_odom_to_baselink)
        local_map_to_odom = copy.copy(cur_map_to_odom) # For thread safety and consistent timestamp
        

        # --- MODIFIED: 自适应滤波逻辑 ---
        if local_map_to_odom is not None:
            target_map_to_odom = pose_to_mat(local_map_to_odom)
            
            if last_smooth_map_to_odom is None:
                # 系统刚启动，直接赋值
                last_smooth_map_to_odom = target_map_to_odom
                T_map_to_odom = target_map_to_odom
            else:
                # 计算当前目标与上一次平滑结果的偏差 (位置偏差)
                delta_mat = np.matmul(np.linalg.inv(last_smooth_map_to_odom), target_map_to_odom)
                delta_trans = np.linalg.norm(tf.transformations.translation_from_matrix(delta_mat))
                
                # 动态调整 alpha，但绝不使用 1.0 (除非是刚初始化或手动重置)
                if delta_trans > 0.5: # 只有偏差极大（比如50cm）才认为是由于回环检测导致的重定位，需要快一点
                    current_alpha = 0.3 # 稍微快一点，但不要 1.0
                elif delta_trans > JUMP_THRESHOLD: # > 0.1
                    current_alpha = 0.05 # 依然保持很小的更新率，慢慢拉过去
                else:
                    current_alpha = SMOOTH_ALPHA # 0.01 或 0.05，越小越丝滑
                
                # 执行插值
                T_map_to_odom = interpolate_transform(last_smooth_map_to_odom, target_map_to_odom, current_alpha)
                
                # 更新状态
                last_smooth_map_to_odom = T_map_to_odom
        else:
            T_map_to_odom = np.eye(4)
            # 如果没有map数据，也要重置一下last_smooth，防止后续突变
            if last_smooth_map_to_odom is None:
                last_smooth_map_to_odom = np.eye(4)
        # --------------------------------


        transform_timestamp = rospy.Time.now() + rospy.Duration(0.01)

        br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                         tf.transformations.quaternion_from_matrix(T_map_to_odom),
                         transform_timestamp,
                         'odom', 'map')
        if cur_odom is not None:
            # === 计算 odom → body → base_link ===
            T_odom_to_body = pose_to_mat(cur_odom)
            # 加入雷达到底盘的静态变换
            T_odom_to_base_link = np.matmul(T_odom_to_body, T_body_to_base)
            
            # === 计算 Twist: 修正杆臂效应 ===
            # 获取雷达系的原始速度
            v_body_x = cur_odom.twist.twist.linear.x
            v_body_y = cur_odom.twist.twist.linear.y
            omega_z = cur_odom.twist.twist.angular.z
            
            # v_base = v_body - omega x r_base_to_body
            # r_base_to_body is vector from base to body
            # r_base_to_body = (-LIDAR_TO_BASE_X, -LIDAR_TO_BASE_Y, -LIDAR_TO_BASE_Z)
            r_base_to_body_x = -LIDAR_TO_BASE_X
            r_base_to_body_y = -LIDAR_TO_BASE_Y
            
            # v_base.x = v_body.x + omega_z * r_base_to_body.y
            # v_base.y = v_body.y - omega_z * r_base_to_body.x
            real_base_vx = v_body_x + omega_z * r_base_to_body_y
            real_base_vy = v_body_y - omega_z * r_base_to_body_x
            
            # === 发布 odom → base_link 的变换 ===
            odom_to_baselink = Odometry()
            xyz_odom = tf.transformations.translation_from_matrix(T_odom_to_base_link)
            quat_odom = tf.transformations.quaternion_from_matrix(T_odom_to_base_link)
            odom_to_baselink.pose.pose = Pose(Point(*xyz_odom), Quaternion(*quat_odom))
            
            # 速度修正
            odom_to_baselink.twist.twist.linear.x = real_base_vx
            odom_to_baselink.twist.twist.linear.y = real_base_vy
            odom_to_baselink.twist.twist.angular.z = omega_z
            odom_to_baselink.twist.twist.linear.z = cur_odom.twist.twist.linear.z
            odom_to_baselink.twist.twist.angular.x = cur_odom.twist.twist.angular.x
            odom_to_baselink.twist.twist.angular.y = cur_odom.twist.twist.angular.y
            odom_to_baselink.twist.covariance = cur_odom.twist.covariance
            odom_to_baselink.pose.covariance = cur_odom.pose.covariance
            
            odom_to_baselink.header.stamp = cur_odom.header.stamp
            odom_to_baselink.header.frame_id = 'odom'
            odom_to_baselink.child_frame_id = 'base_link'
            
            pub_localization.publish(odom_to_baselink)
            
            # === NEW ADDED: 发布 map → base_link 的变换 ===
            # 计算 map → base_link (map → odom → body → base_link)
            T_map_to_body = np.matmul(T_map_to_odom, T_odom_to_body)
            T_map_to_base_link = np.matmul(T_map_to_body, T_body_to_base)
            
            map_to_baselink = Odometry()
            xyz_map = tf.transformations.translation_from_matrix(T_map_to_base_link)
            quat_map = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
            map_to_baselink.pose.pose = Pose(Point(*xyz_map), Quaternion(*quat_map))
            
            # 速度与 odom_to_baselink 相同（因为速度是在局部坐标系中）
            map_to_baselink.twist.twist.linear.x = real_base_vx
            map_to_baselink.twist.twist.linear.y = real_base_vy
            map_to_baselink.twist.twist.angular.z = omega_z
            map_to_baselink.twist.twist.linear.z = cur_odom.twist.twist.linear.z
            map_to_baselink.twist.twist.angular.x = cur_odom.twist.twist.angular.x
            map_to_baselink.twist.twist.angular.y = cur_odom.twist.twist.angular.y
            map_to_baselink.twist.covariance = cur_odom.twist.covariance
            map_to_baselink.pose.covariance = cur_odom.pose.covariance
            
            map_to_baselink.header.stamp = cur_odom.header.stamp
            map_to_baselink.header.frame_id = 'map'
            map_to_baselink.child_frame_id = 'base_link'
            
            pub_map_to_baselink.publish(map_to_baselink)
            # ================================================


def cb_save_cur_odom(odom_msg):
    global cur_odom_to_baselink
    cur_odom_to_baselink = odom_msg


def cb_save_map_to_odom(odom_msg):
    global cur_map_to_odom
    cur_map_to_odom = odom_msg


if __name__ == '__main__':
    # tf and localization publishing frequency (HZ)
    FREQ_PUB_LOCALIZATION = 50

    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')

    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)

    pub_map_to_baselink = rospy.Publisher('/map_to_baselink', Odometry, queue_size=1)
    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

    # 发布定位消息
    _thread.start_new_thread(transform_fusion, ())

    rospy.spin()