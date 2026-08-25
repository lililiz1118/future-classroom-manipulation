#!/usr/bin/env python3
# coding=utf8
"""
轮式里程计 + 点云配准定位节点
结合轮式里程计和点云配准，实现精确的全局定位
"""
from __future__ import print_function, division, absolute_import

import copy
import _thread
import time

import open3d as o3d
import rospy
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations

# 全局变量
global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_wheel_odom = None  # 轮式里程计
cur_scan = None

# 配准限制相关
registration_count = 0
MAX_REGISTRATION_COUNT = 5

# 平滑滤波相关
last_smooth_map_to_odom = None


def pose_to_mat(pose_msg):
    """将ROS Odometry消息转换为4x4变换矩阵"""
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def mat_to_pose(mat):
    """将4x4变换矩阵转换为ROS Pose消息"""
    pose = Pose()
    pose.position.x = mat[0, 3]
    pose.position.y = mat[1, 3]
    pose.position.z = mat[2, 3]
    
    quat = tf.transformations.quaternion_from_matrix(mat)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    
    return pose


def msg_to_array(pc_msg):
    """将PointCloud2消息转换为numpy数组"""
    pc_array = ros_numpy.numpify(pc_msg)
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc


def voxel_down_sample(pcd, voxel_size):
    """体素降采样"""
    try:
        return pcd.voxel_down_sample(voxel_size)
    except:
        # for open3d 0.7 or lower
        return o3d.geometry.voxel_down_sample(pcd, voxel_size)


def registration_at_scale(pc_scan, pc_map, initial, scale):
    """多尺度ICP配准"""
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale),
        voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        1.0 * scale,
        initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
    )
    return result_icp.transformation, result_icp.fitness


def inverse_se3(trans):
    """计算SE3变换的逆"""
    trans_inverse = np.eye(4)
    trans_inverse[:3, :3] = trans[:3, :3].T
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    """发布点云"""
    data = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    """裁剪视野范围内的全局地图"""
    # 当前scan原点的位姿
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 提取视角内的地图点
    if FOV > 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        # 非环状lidar 保前视范围
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    # 发布fov内点云
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    return global_map_in_FOV


def interpolate_transform(T_old, T_new, alpha):
    """对两个变换矩阵进行插值平滑"""
    # 位置插值 (线性)
    pos_old = tf.transformations.translation_from_matrix(T_old)
    pos_new = tf.transformations.translation_from_matrix(T_new)
    pos_cur = pos_old * (1.0 - alpha) + pos_new * alpha

    # 旋转插值 (Slerp - 球面线性插值)
    quat_old = tf.transformations.quaternion_from_matrix(T_old)
    quat_new = tf.transformations.quaternion_from_matrix(T_new)
    quat_cur = tf.transformations.quaternion_slerp(quat_old, quat_new, alpha)

    # 合成新矩阵
    T_cur = tf.transformations.quaternion_matrix(quat_cur)
    T_cur[0:3, 3] = pos_cur
    return T_cur


def global_localization(pose_estimation):
    """使用点云配准进行全局定位"""
    global global_map, cur_scan, cur_wheel_odom, T_map_to_odom
    
    rospy.loginfo('Global localization by scan-to-map matching......')

    # 线程安全复制
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    # 裁剪FOV内的地图
    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_wheel_odom)

    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, 
                                             initial=pose_estimation, scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, 
                                                   initial=transformation, scale=1)
    
    toc = time.time()
    rospy.loginfo('Registration time: {:.3f}s, fitness: {:.3f}'.format(toc - tic, fitness))

    # 当全局定位成功时才更新map_to_odom
    if fitness > LOCALIZATION_TH:
        T_map_to_odom = transformation

        # 发布map_to_odom
        map_to_odom = Odometry()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'wheel_odom_frame'
        map_to_odom.header.stamp = cur_wheel_odom.header.stamp
        map_to_odom.pose.pose = mat_to_pose(T_map_to_odom)
        pub_map_to_odom.publish(map_to_odom)

        # 发布当前scan在map系下的位置（用于可视化）
        T_odom_to_base = pose_to_mat(cur_wheel_odom)
        T_map_to_base = np.matmul(T_map_to_odom, T_odom_to_base)
        
        cur_scan_in_map = copy.copy(cur_scan)
        cur_scan_in_map.transform(T_map_to_base)
        
        header = cur_wheel_odom.header
        header.frame_id = 'map'
        publish_point_cloud(pub_pc_in_map, header, np.array(cur_scan_in_map.points)[::3])

        rospy.loginfo('Localization succeeded! Fitness: {:.3f}'.format(fitness))
        return True
    else:
        rospy.logwarn('Localization failed! Fitness too low: {:.3f}'.format(fitness))
        return False


def cb_save_cur_scan(msg):
    """保存当前扫描点云"""
    global cur_scan
    if not initialized:
        return
    
    pc_array = msg_to_array(msg)
    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(pc_array)
    cur_scan = voxel_down_sample(cur_scan, SCAN_VOXEL_SIZE)


def cb_save_cur_wheel_odom(msg):
    """保存当前轮式里程计"""
    global cur_wheel_odom
    cur_wheel_odom = msg


def initialize_global_map(pcd_msg):
    """初始化全局地图"""
    global global_map
    
    rospy.loginfo('Loading global map...')
    pc_array = msg_to_array(pcd_msg)
    
    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(pc_array)
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    
    rospy.loginfo('Global map loaded! Points: {}'.format(len(global_map.points)))


def thread_localization():
    """定位线程"""
    global registration_count
    
    while True:
        time.sleep(1.0 / FREQ_LOCALIZATION)
        
        if cur_scan is None or cur_wheel_odom is None:
            continue
        
        # 检查是否达到最大配准次数
        if registration_count < MAX_REGISTRATION_COUNT:
            result = global_localization(T_map_to_odom)
            if result:
                registration_count += 1
                rospy.loginfo('Registration count: {}/{}'.format(
                    registration_count, MAX_REGISTRATION_COUNT))
                
                # 达到最大次数时输出警告
                if registration_count >= MAX_REGISTRATION_COUNT:
                    rospy.logwarn("="*60)
                    rospy.logwarn("LOCKED: Maximum registration count reached!")
                    rospy.logwarn("No more scan-to-map matching will be performed.")
                    rospy.logwarn("The system will now rely purely on wheel odometry.")
                    rospy.logwarn("="*60)


def fusion_and_publish():
    """融合定位结果并发布"""
    global cur_wheel_odom, T_map_to_odom, last_smooth_map_to_odom
    
    # 平滑滤波参数
    JUMP_THRESHOLD = 0.1
    SMOOTH_ALPHA = 0.01
    
    br = tf.TransformBroadcaster()
    
    while True:
        time.sleep(1.0 / FREQ_PUB_LOCALIZATION)
        
        # 线程安全复制
        local_wheel_odom = copy.copy(cur_wheel_odom)
        
        if local_wheel_odom is None:
            continue
        
        # 平滑map_to_odom变换
        target_map_to_odom = copy.copy(T_map_to_odom)
        
        if last_smooth_map_to_odom is None:
            last_smooth_map_to_odom = target_map_to_odom
            smooth_map_to_odom = target_map_to_odom
        else:
            # 计算偏差
            delta_mat = np.matmul(np.linalg.inv(last_smooth_map_to_odom), target_map_to_odom)
            delta_trans = np.linalg.norm(tf.transformations.translation_from_matrix(delta_mat))
            
            # 动态调整alpha
            if delta_trans > 0.5:
                current_alpha = 0.3
            elif delta_trans > JUMP_THRESHOLD:
                current_alpha = 0.05
            else:
                current_alpha = SMOOTH_ALPHA
            
            # 执行插值
            smooth_map_to_odom = interpolate_transform(last_smooth_map_to_odom, 
                                                       target_map_to_odom, current_alpha)
            last_smooth_map_to_odom = smooth_map_to_odom
        
        # 计算base_link在map系下的位姿
        T_odom_to_base = pose_to_mat(local_wheel_odom)
        T_map_to_base = np.matmul(smooth_map_to_odom, T_odom_to_base)
        
        # 发布TF: map -> wheel_odom_frame
        trans = tf.transformations.translation_from_matrix(smooth_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(smooth_map_to_odom)
        br.sendTransform(trans, quat, local_wheel_odom.header.stamp, 'wheel_odom_frame', 'map')
        
        # 发布轮式里程计 (wheel_odom_frame -> base_link)
        odom_to_baselink = Odometry()
        odom_to_baselink.header.stamp = local_wheel_odom.header.stamp
        odom_to_baselink.header.frame_id = 'wheel_odom_frame'
        odom_to_baselink.child_frame_id = 'base_link'
        odom_to_baselink.pose.pose = local_wheel_odom.pose.pose
        odom_to_baselink.twist.twist = local_wheel_odom.twist.twist
        pub_wheel_odom_out.publish(odom_to_baselink)
        
        # 发布全局定位结果 (map -> base_link)
        localization = Odometry()
        localization.header.stamp = local_wheel_odom.header.stamp
        localization.header.frame_id = 'map'
        localization.child_frame_id = 'base_link'
        localization.pose.pose = mat_to_pose(T_map_to_base)
        
        # 计算base_link在map系下的速度
        # v_map = R_map_to_odom * v_odom
        v_odom = np.array([local_wheel_odom.twist.twist.linear.x,
                          local_wheel_odom.twist.twist.linear.y,
                          local_wheel_odom.twist.twist.linear.z])
        R_map_to_odom = smooth_map_to_odom[:3, :3]
        v_map = np.matmul(R_map_to_odom, v_odom)
        
        localization.twist.twist.linear.x = v_map[0]
        localization.twist.twist.linear.y = v_map[1]
        localization.twist.twist.linear.z = v_map[2]
        localization.twist.twist.angular = local_wheel_odom.twist.twist.angular
        
        pub_localization.publish(localization)


if __name__ == '__main__':
    # 参数配置
    MAP_VOXEL_SIZE = 0.1
    SCAN_VOXEL_SIZE = 0.05
    
    # 全局定位频率 (Hz)
    FREQ_LOCALIZATION = 0.5
    
    # 发布频率 (Hz)
    FREQ_PUB_LOCALIZATION = 50
    
    # 定位阈值
    LOCALIZATION_TH = 0.8
    
    # FOV参数
    FOV = 6.28319  # 360度
    FOV_FAR = 30   # 30米
    
    # 初始化节点
    rospy.init_node('wheel_odom_localization')
    rospy.loginfo('='*60)
    rospy.loginfo('Wheel Odometry + Point Cloud Localization Node Started')
    rospy.loginfo('='*60)
    rospy.loginfo('Maximum registration count: {}'.format(MAX_REGISTRATION_COUNT))
    rospy.loginfo('After that, the system will use pure wheel odometry.')
    rospy.loginfo('='*60)
    
    # 发布器
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)
    pub_wheel_odom_out = rospy.Publisher('/odom_to_baselink', Odometry, queue_size=1)
    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)
    
    # 订阅器
    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/wheel_odom', Odometry, cb_save_cur_wheel_odom, queue_size=1)
    
    # 初始化全局地图    
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('pcd_map', PointCloud2))
    
    # 等待初始位姿
    while not initialized:
        rospy.logwarn('Waiting for initial pose....')
        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        initial_pose = pose_to_mat(pose_msg)
        
        if cur_scan and cur_wheel_odom:
            initialized = global_localization(initial_pose)
        else:
            rospy.logwarn('Scan or wheel odometry not received yet!')
    
    rospy.loginfo('')
    rospy.loginfo('='*60)
    rospy.loginfo('Initialization successful!')
    rospy.loginfo('='*60)
    rospy.loginfo('')
    
    # 重置计数器
    registration_count = 0
    
    # 启动定位线程
    _thread.start_new_thread(thread_localization, ())
    
    # 启动融合发布线程
    _thread.start_new_thread(fusion_and_publish, ())
    
    rospy.spin()
