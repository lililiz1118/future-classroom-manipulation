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
        
        # --- MODIFIED: 自适应滤波逻辑 ---
        if cur_map_to_odom is not None:
            target_map_to_odom = pose_to_mat(cur_map_to_odom)
            
            if last_smooth_map_to_odom is None:
                # 系统刚启动，直接赋值
                last_smooth_map_to_odom = target_map_to_odom
                T_map_to_odom = target_map_to_odom
            else:
                # 计算当前目标与上一次平滑结果的偏差 (位置偏差)
                # delta_mat = T_old^-1 * T_new
                delta_mat = np.matmul(np.linalg.inv(last_smooth_map_to_odom), target_map_to_odom)
                delta_trans = np.linalg.norm(tf.transformations.translation_from_matrix(delta_mat))
                
                # # 自适应选择 alpha
                # if delta_trans > JUMP_THRESHOLD:
                #     # 偏差过大，认为是有效重定位或大幅度修正，快速跟随
                #     current_alpha = RESET_ALPHA
                #     # 可选: 打印日志方便调试
                #     # rospy.logwarn_throttle(1.0, "Large drift detected ({:.2f}m), correcting...".format(delta_trans))
                # else:
                #     # 偏差很小，认为是噪声，强平滑
                #     current_alpha = SMOOTH_ALPHA

                # --- 修改开始 ---
                # 方案：动态调整 alpha，但绝不使用 1.0 (除非是刚初始化或手动重置)
                # 如果偏差很大，我们可以稍微加快一点点收敛速度，但不能瞬移
                
                if delta_trans > 0.5: # 只有偏差极大（比如50cm）才认为是由于回环检测导致的重定位，需要快一点
                    current_alpha = 0.3 # 稍微快一点，但不要 1.0
                elif delta_trans > JUMP_THRESHOLD: # > 0.1
                    current_alpha = 0.05 # 依然保持很小的更新率，慢慢拉过去
                else:
                    current_alpha = SMOOTH_ALPHA # 0.01 或 0.05，越小越丝滑
                
                # 极度平滑模式：
                # 甚至可以不用 alpha，而是限制每一帧只能修正 0.005m (5mm)
                # 这样即使算出来偏差 1米，也会像滑冰一样慢慢滑过去，完全消除残影
                # --- 修改结束 ---
                
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

        br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                         tf.transformations.quaternion_from_matrix(T_map_to_odom),
                         rospy.Time.now(),
                         'odom', 'map')
        if cur_odom is not None:
            # 发布全局定位的odometry
            localization = Odometry()
            T_odom_to_base_link = pose_to_mat(cur_odom)
            
            # 这里T_map_to_odom已经经过了平滑处理
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
            
            xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
            quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            localization.twist = cur_odom.twist

            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = 'map'
            localization.child_frame_id = 'body'
            # rospy.loginfo_throttle(1, '{}'.format(np.matmul(T_map_to_odom, T_odom_to_base_link)))
            pub_localization.publish(localization)


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

    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

    # 发布定位消息
    _thread.start_new_thread(transform_fusion, ())

    rospy.spin()
# #!/usr/bin/python3
# # coding=utf8
# from __future__ import print_function, division, absolute_import

# import copy
# import _thread
# import time

# import numpy as np
# import rospy
# import tf
# import tf.transformations
# from geometry_msgs.msg import Pose, Point, Quaternion
# from nav_msgs.msg import Odometry

# cur_odom_to_baselink = None
# cur_map_to_odom = None


# def pose_to_mat(pose_msg):
#     return np.matmul(
#         tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
#         tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
#     )


# def transform_fusion():
#     global cur_odom_to_baselink, cur_map_to_odom

#     br = tf.TransformBroadcaster()
#     while True:
#         time.sleep(1 / FREQ_PUB_LOCALIZATION)

#         # TODO 这里注意线程安全
#         cur_odom = copy.copy(cur_odom_to_baselink)
#         if cur_map_to_odom is not None:
#             T_map_to_odom = pose_to_mat(cur_map_to_odom)
#         else:
#             T_map_to_odom = np.eye(4)

#         br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
#                          tf.transformations.quaternion_from_matrix(T_map_to_odom),
#                          rospy.Time.now(),
#                          'odom', 'map')
#         if cur_odom is not None:
#             # 发布全局定位的odometry
#             localization = Odometry()
#             T_odom_to_base_link = pose_to_mat(cur_odom)
#             # 这里T_map_to_odom短时间内变化缓慢 暂时不考虑与T_odom_to_base_link时间同步
#             T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
#             xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
#             quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
#             localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
#             localization.twist = cur_odom.twist

#             localization.header.stamp = cur_odom.header.stamp
#             localization.header.frame_id = 'map'
#             localization.child_frame_id = 'body'
#             # rospy.loginfo_throttle(1, '{}'.format(np.matmul(T_map_to_odom, T_odom_to_base_link)))
#             pub_localization.publish(localization)


# def cb_save_cur_odom(odom_msg):
#     global cur_odom_to_baselink
#     cur_odom_to_baselink = odom_msg


# def cb_save_map_to_odom(odom_msg):
#     global cur_map_to_odom
#     cur_map_to_odom = odom_msg


# if __name__ == '__main__':
#     # tf and localization publishing frequency (HZ)
#     FREQ_PUB_LOCALIZATION = 50

#     rospy.init_node('transform_fusion')
#     rospy.loginfo('Transform Fusion Node Inited...')

#     rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
#     rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)

#     pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

#     # 发布定位消息
#     _thread.start_new_thread(transform_fusion, ())

#     rospy.spin()


