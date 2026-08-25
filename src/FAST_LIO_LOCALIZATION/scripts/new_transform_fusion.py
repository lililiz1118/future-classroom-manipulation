#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import threading
import time

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry

# 全局共享状态
cur_odom_to_baselink = None
cur_wheel_odom = None
cur_map_to_odom = None
last_smooth_map_to_odom = None

# ================= 配置区域 =================
# 雷达到底盘中心的静态外参 (T_body_to_base)
LIDAR_TO_BASE_X = -0.255
LIDAR_TO_BASE_Y = 0.0
LIDAR_TO_BASE_Z = -0.022
T_body_to_base = tf.transformations.translation_matrix(
    [LIDAR_TO_BASE_X, LIDAR_TO_BASE_Y, LIDAR_TO_BASE_Z]
)

# base_link 到 base_footprint 的静态变换 (底盘在地面投影)
BASE_TO_FOOTPRINT_X = 0.0
BASE_TO_FOOTPRINT_Y = 0.0
BASE_TO_FOOTPRINT_Z = -0.151
T_base_to_footprint = tf.transformations.translation_matrix(
    [BASE_TO_FOOTPRINT_X, BASE_TO_FOOTPRINT_Y, BASE_TO_FOOTPRINT_Z]
)


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def interpolate_transform(T_old, T_new, alpha):
    """
    对两个变换矩阵进行插值平滑
    T_old, T_new: 4x4 变换矩阵
    alpha: 更新系数 [0, 1]
    """
    # 1. 位置线性插值
    pos_old = tf.transformations.translation_from_matrix(T_old)
    pos_new = tf.transformations.translation_from_matrix(T_new)
    pos_cur = pos_old * (1.0 - alpha) + pos_new * alpha

    # 2. 旋转球面线性插值 (Slerp)
    quat_old = tf.transformations.quaternion_from_matrix(T_old)
    quat_new = tf.transformations.quaternion_from_matrix(T_new)
    quat_cur = tf.transformations.quaternion_slerp(quat_old, quat_new, alpha)

    # 3. 合成新矩阵
    T_cur = tf.transformations.quaternion_matrix(quat_cur)
    T_cur[0:3, 3] = pos_cur
    return T_cur


def transform_fusion_loop():
    global cur_odom_to_baselink, cur_wheel_odom, cur_map_to_odom, last_smooth_map_to_odom
    global pub_localization, pub_map_to_basefootprint

    # 参数设置
    JUMP_THRESHOLD = 0.1
    SMOOTH_ALPHA = 0.02

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(FREQ_PUB_LOCALIZATION)

    last_time = time.time()
    prev_lio_pos = None
    fused_pos = None

    while not rospy.is_shutdown():
        now_time = time.time()
        dt = max(0.001, min(0.1, now_time - last_time))
        last_time = now_time

        cur_odom = copy.deepcopy(cur_odom_to_baselink)
        cur_wheel = copy.deepcopy(cur_wheel_odom)
        local_map_to_odom = copy.deepcopy(cur_map_to_odom)

        # 1. 自适应平滑 map -> odom 变换
        if local_map_to_odom is not None:
            target_map_to_odom = pose_to_mat(local_map_to_odom)
            if last_smooth_map_to_odom is None:
                last_smooth_map_to_odom = target_map_to_odom
                T_map_to_odom = target_map_to_odom
            else:
                delta_mat = np.matmul(np.linalg.inv(last_smooth_map_to_odom), target_map_to_odom)
                delta_trans = np.linalg.norm(tf.transformations.translation_from_matrix(delta_mat))

                if delta_trans > 0.5:
                    current_alpha = 0.3
                elif delta_trans > JUMP_THRESHOLD:
                    current_alpha = 0.05
                else:
                    current_alpha = SMOOTH_ALPHA

                T_map_to_odom = interpolate_transform(last_smooth_map_to_odom, target_map_to_odom, current_alpha)
                last_smooth_map_to_odom = T_map_to_odom
        else:
            T_map_to_odom = np.eye(4)
            if last_smooth_map_to_odom is None:
                last_smooth_map_to_odom = np.eye(4)

        # 2. 广播 map -> odom TF
        transform_timestamp = rospy.Time.now() + rospy.Duration(0.01)
        br.sendTransform(
            tf.transformations.translation_from_matrix(T_map_to_odom),
            tf.transformations.quaternion_from_matrix(T_map_to_odom),
            transform_timestamp,
            'odom',
            'map',
        )

        # 3. 紧耦合激光-轮速-惯导位置与速度融合 (LIWO)
        if cur_odom is not None:
            T_odom_to_body_raw = pose_to_mat(cur_odom)
            pos_lio_raw = tf.transformations.translation_from_matrix(T_odom_to_body_raw)
            quat_lio_raw = tf.transformations.quaternion_from_matrix(T_odom_to_body_raw)
            yaw_lio = tf.transformations.euler_from_quaternion(quat_lio_raw)[2]

            omega_z = cur_odom.twist.twist.angular.z

            # 判断轮速计是否有效
            wheel_valid = (
                cur_wheel is not None
                and (now_time - cur_wheel.header.stamp.to_sec() < 0.5)
            )

            if wheel_valid:
                real_base_vx = float(cur_wheel.twist.twist.linear.x)
                real_base_vy = 0.0
            else:
                # Fallback: 修正雷达杆臂效应
                v_body_x = cur_odom.twist.twist.linear.x
                v_body_y = cur_odom.twist.twist.linear.y
                r_base_to_body_x = -LIDAR_TO_BASE_X
                r_base_to_body_y = -LIDAR_TO_BASE_Y
                real_base_vx = v_body_x + omega_z * r_base_to_body_y
                real_base_vy = v_body_y - omega_z * r_base_to_body_x

            # --- 紧耦合位置融合 ---
            if fused_pos is None or prev_lio_pos is None:
                fused_pos = np.copy(pos_lio_raw)
                prev_lio_pos = np.copy(pos_lio_raw)
            else:
                d_lio = pos_lio_raw - prev_lio_pos
                prev_lio_pos = np.copy(pos_lio_raw)

                if wheel_valid:
                    # 轮速前向位移
                    d_wheel_forward = real_base_vx * dt

                    # 激光位移在车身前进方向与侧向的投影
                    cos_yaw = np.cos(yaw_lio)
                    sin_yaw = np.sin(yaw_lio)
                    d_lio_forward = d_lio[0] * cos_yaw + d_lio[1] * sin_yaw
                    d_lio_lateral = -d_lio[0] * sin_yaw + d_lio[1] * cos_yaw

                    # 融合前向位移 (80% 信任高精度轮速编码器，消除长走廊打滑漂移)
                    d_fused_forward = 0.80 * d_wheel_forward + 0.20 * d_lio_forward
                    # 差速模型非完整约束 (过滤 90% 激光侧滑抖动)
                    d_fused_lateral = 0.10 * d_lio_lateral

                    # 还原为全局 odom 坐标系位移增量
                    dx = d_fused_forward * cos_yaw - d_fused_lateral * sin_yaw
                    dy = d_fused_forward * sin_yaw + d_fused_lateral * cos_yaw
                    dz = d_lio[2] # 高度 Z 完全由激光保持

                    fused_pos[0] += dx
                    fused_pos[1] += dy
                    fused_pos[2] = pos_lio_raw[2]

                    # 缓和牵引: 缓慢向激光大尺度整体趋势靠拢 (防止长时间微小累积误差)
                    fused_pos[0:2] = fused_pos[0:2] * 0.999 + pos_lio_raw[0:2] * 0.001
                else:
                    fused_pos = np.copy(pos_lio_raw)

            # 重新构建紧耦合后的 T_odom_to_body 矩阵
            T_odom_to_body = tf.transformations.quaternion_matrix(quat_lio_raw)
            T_odom_to_body[0:3, 3] = fused_pos

            T_odom_to_base_link = np.matmul(T_odom_to_body, T_body_to_base)

            # 4. 发布 odom -> base_link 定位消息 (/localization)
            odom_to_baselink = Odometry()
            xyz_odom = tf.transformations.translation_from_matrix(T_odom_to_base_link)
            quat_odom = tf.transformations.quaternion_from_matrix(T_odom_to_base_link)
            odom_to_baselink.pose.pose = Pose(Point(*xyz_odom), Quaternion(*quat_odom))

            odom_to_baselink.twist.twist.linear.x = real_base_vx
            odom_to_baselink.twist.twist.linear.y = real_base_vy
            odom_to_baselink.twist.twist.angular.z = omega_z
            odom_to_baselink.twist.twist.linear.z = 0.0
            odom_to_baselink.twist.twist.angular.x = cur_odom.twist.twist.angular.x
            odom_to_baselink.twist.twist.angular.y = cur_odom.twist.twist.angular.y
            odom_to_baselink.twist.covariance = cur_odom.twist.covariance
            odom_to_baselink.pose.covariance = cur_odom.pose.covariance

            odom_to_baselink.header.stamp = cur_odom.header.stamp
            odom_to_baselink.header.frame_id = 'odom'
            odom_to_baselink.child_frame_id = 'base_link'
            pub_localization.publish(odom_to_baselink)

            # 5. 发布 map -> base_footprint 定位消息
            T_map_to_body = np.matmul(T_map_to_odom, T_odom_to_body)
            T_map_to_base_link = np.matmul(T_map_to_body, T_body_to_base)
            T_map_to_footprint = np.matmul(T_map_to_base_link, T_base_to_footprint)

            map_to_basefootprint = Odometry()
            xyz_map = tf.transformations.translation_from_matrix(T_map_to_footprint)
            quat_map = tf.transformations.quaternion_from_matrix(T_map_to_footprint)
            map_to_basefootprint.pose.pose = Pose(Point(*xyz_map), Quaternion(*quat_map))

            map_to_basefootprint.twist.twist.linear.x = real_base_vx
            map_to_basefootprint.twist.twist.linear.y = real_base_vy
            map_to_basefootprint.twist.twist.angular.z = omega_z
            map_to_basefootprint.twist.covariance = cur_odom.twist.covariance
            map_to_basefootprint.pose.covariance = cur_odom.pose.covariance

            map_to_basefootprint.header.stamp = cur_odom.header.stamp
            map_to_basefootprint.header.frame_id = 'map'
            map_to_basefootprint.child_frame_id = 'base_footprint'
            pub_map_to_basefootprint.publish(map_to_basefootprint)

        rate.sleep()


def cb_save_cur_odom(odom_msg):
    global cur_odom_to_baselink
    cur_odom_to_baselink = odom_msg


def cb_save_cur_wheel_odom(odom_msg):
    global cur_wheel_odom
    cur_wheel_odom = odom_msg


def cb_save_map_to_odom(odom_msg):
    global cur_map_to_odom
    cur_map_to_odom = odom_msg


def cb_initialpose_reset(pose_msg):
    global last_smooth_map_to_odom
    last_smooth_map_to_odom = None
    rospy.loginfo('transform_fusion: reset smooth filter for manual initialpose')


if __name__ == '__main__':
    FREQ_PUB_LOCALIZATION = 50

    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited with Tight LIWO Position & Velocity Integration...')

    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/wheel_odom', Odometry, cb_save_cur_wheel_odom, queue_size=1)
    rospy.Subscriber('/odom', Odometry, cb_save_cur_wheel_odom, queue_size=1)
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, cb_initialpose_reset, queue_size=1)

    pub_map_to_basefootprint = rospy.Publisher('/map_to_basefootprint', Odometry, queue_size=1)
    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

    # 发布静态 TF: base_link -> base_footprint
    static_tf_broadcaster = tf.TransformBroadcaster()

    def publish_static_tf():
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            static_tf_broadcaster.sendTransform(
                (BASE_TO_FOOTPRINT_X, BASE_TO_FOOTPRINT_Y, BASE_TO_FOOTPRINT_Z),
                (0, 0, 0, 1),
                rospy.Time.now(),
                'base_footprint',
                'base_link',
            )
            rate.sleep()

    threading.Thread(target=publish_static_tf, daemon=True).start()
    threading.Thread(target=transform_fusion_loop, daemon=True).start()

    rospy.spin()
