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

# ================= 修改说明 =================
# 我们这里将雷达到底盘的静态变换写死，而不是通过tf树获取
# 我们将此节点改为发布从map到base_link的odometry消息，从而可使以base_link为目标点导航


# ================= 配置区域 =================
# 定义雷达到底盘的静态变换 (T_body_to_base)
# 也就是：在"雷达坐标系"下，"底盘中心"在哪里？
# 如果雷达在底盘前方 0.3m (x=0.3)，那么底盘就在雷达后方 0.3m (x=-0.3)
LIDAR_TO_BASE_X = -0.3  # 注意符号！通常是负的
LIDAR_TO_BASE_Y = 0.1
LIDAR_TO_BASE_Z = -0.5  # 如果雷达比底盘高0.5m，底盘就在下方
# ===========================================

cur_odom_to_body = None
cur_map_to_odom = None

# ==========================================
# 必须设置为 True 才能接收并应用 "Single Shot" 的修正结果
# 如果设为 False，即使 global_localization 算出了位置，这里也会强制忽略。
USE_LIDAR_REGISTRATION = True 
# ==========================================

def get_static_transform_mat():
    """生成 T_body_to_base 的 4x4 矩阵"""
    # 这里假设只有平移，没有旋转（雷达水平安装）。如果有旋转，需在此处乘上旋转矩阵。
    mat = tf.transformations.translation_matrix([LIDAR_TO_BASE_X, LIDAR_TO_BASE_Y, LIDAR_TO_BASE_Z])
    return mat

# 缓存静态矩阵，避免重复计算
T_body_to_base = get_static_transform_mat()

def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def transform_fusion():
    global cur_odom_to_body, cur_map_to_odom # 这里应该是odom_to_body，即里程计至雷达

    br = tf.TransformBroadcaster()
    while True:
        time.sleep(1 / FREQ_PUB_LOCALIZATION)

        # TODO 这里注意线程安全
        cur_odom = copy.copy(cur_odom_to_body)
        
        if USE_LIDAR_REGISTRATION:
            # 在 Single Shot 模式下，cur_map_to_odom 只会更新一次
            # 这里会一直使用最后一次更新的值，保持 odom 和 map 的静态相对关系
            if cur_map_to_odom is not None:
                T_map_to_odom = pose_to_mat(cur_map_to_odom)
            else:
                T_map_to_odom = np.eye(4)
        else:
            T_map_to_odom = np.eye(4)

        br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                         tf.transformations.quaternion_from_matrix(T_map_to_odom),
                         rospy.Time.now(),
                         'odom', 'map')
        if cur_odom is not None:
            localization = Odometry()
            
            # 1. 准备各种变换矩阵
            T_odom_to_body = pose_to_mat(cur_odom) 
            
            # 2. 计算 Pose (核心修正：加入 T_body_to_base)
            # 链式法则: T_map_to_base = T_map_to_odom * T_odom_to_body * T_body_to_base
            T_map_to_body = np.matmul(T_map_to_odom, T_odom_to_body)
            T_map_to_base_link = np.matmul(T_map_to_body, T_body_to_base)
            
            # 提取位姿填入消息
            xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
            quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            
            # 3. 计算 Twist (速度修正：杆臂效应)
            # 获取雷达系的原始速度
            v_body_x = cur_odom.twist.twist.linear.x
            v_body_y = cur_odom.twist.twist.linear.y
            omega_z = cur_odom.twist.twist.angular.z

            # 公式 v_body = v_base + omega x r (向量叉乘) (杆臂效应补偿)
            # 其中 r = t_body - t_lidar = (dx, dy)
            # omega x r = (-omega*dy, omega*dx)
            
            x_dist_lidar_to_center = -LIDAR_TO_BASE_X 
            y_dist_lidar_to_center = -LIDAR_TO_BASE_Y 
            
            real_base_vx = v_body_x + omega_z * y_dist_lidar_to_center
            real_base_vy = v_body_y - omega_z * x_dist_lidar_to_center
            
            localization.twist.twist.linear.x = real_base_vx
            localization.twist.twist.linear.y = real_base_vy
            localization.twist.twist.angular.z = omega_z

            # 4. 修正 Header
            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = 'map'       # 这是一个基于 Map 的定位消息
            localization.child_frame_id = 'base_link'  # <--- 现在它是名副其实的 base_link 了
            
            pub_localization.publish(localization)

def cb_save_cur_odom(odom_msg):
    global cur_odom_to_body
    cur_odom_to_body = odom_msg


def cb_save_map_to_odom(odom_msg):
    global cur_map_to_odom
    cur_map_to_odom = odom_msg


if __name__ == '__main__':
    # tf and localization publishing frequency (HZ)
    FREQ_PUB_LOCALIZATION = 50

    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')
    
    if USE_LIDAR_REGISTRATION:
        rospy.loginfo('Mode: Lidar Registration ENABLED (Will fuse map_to_odom)')
    else:
        rospy.logwarn('Mode: Lidar Registration DISABLED (Pure Odometry)')

    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)

    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

    # 发布定位消息
    _thread.start_new_thread(transform_fusion, ())

    rospy.spin()