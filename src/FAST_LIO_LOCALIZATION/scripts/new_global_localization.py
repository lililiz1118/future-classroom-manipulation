#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import threading
import time

import numpy as np
import open3d as o3d
import ros_numpy
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


from sensor_msgs import point_cloud2

def msg_to_array(pc_msg):
    # Sanitize duplicate field names (e.g. pcl_ros padding fields named '_')
    fields = []
    seen = {}
    for f in pc_msg.fields:
        name = f.name
        if name in seen:
            seen[name] += 1
            new_f = copy.deepcopy(f)
            new_f.name = "{}_{}".format(name, seen[name])
            fields.append(new_f)
        else:
            seen[name] = 0
            fields.append(f)
    pc_msg_clean = copy.deepcopy(pc_msg)
    pc_msg_clean.fields = fields

    try:
        pc_array = ros_numpy.numpify(pc_msg_clean)
        pc = np.zeros([len(pc_array), 3], dtype=np.float32)
        pc[:, 0] = pc_array['x']
        pc[:, 1] = pc_array['y']
        pc[:, 2] = pc_array['z']
        return pc
    except Exception:
        pts = list(point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True))
        return np.array(pts, dtype=np.float32)


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    trans_inverse[:3, :3] = trans[:3, :3].T
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    data = np.zeros(
        len(pc),
        dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
        ],
    )
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)


def voxel_down_sample(pcd, voxel_size):
    try:
        return pcd.voxel_down_sample(voxel_size)
    except Exception:
        return o3d.geometry.voxel_down_sample(pcd, voxel_size)


class FastLioLocalization:
    def __init__(self):
        self._lock = threading.Lock()
        self.initialized = False
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        self.global_map = None
        self.pending_initial_pose = None

        self.map_voxel_size = 0.1
        self.scan_voxel_size = 0.03
        self.freq_localization = float(rospy.get_param('~freq_localization', 0.1))
        self.localization_th = float(rospy.get_param('~localization_th', 0.6))
        self.fov = float(rospy.get_param('~fov', 6.28319))
        self.fov_far = float(rospy.get_param('~fov_far', 20.0))

        # Publishers
        self.pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
        self.pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
        self.pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)

        # Subscribers
        rospy.Subscriber('/cloud_registered', PointCloud2, self.cb_save_cur_scan, queue_size=1)
        rospy.Subscriber('/Odometry', Odometry, self.cb_save_cur_odom, queue_size=1)
        # Permanent subscriber for /initialpose right from startup
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.cb_initialpose, queue_size=1)

        rospy.loginfo('Waiting for global map on topic pcd_map ...')
        map_msg = rospy.wait_for_message('pcd_map', PointCloud2)
        self.initialize_global_map(map_msg)

        # Start periodic tracking thread
        threading.Thread(target=self.thread_localization_loop, daemon=True).start()
        rospy.loginfo('Localization node initialized and ready for initialpose.')

    def initialize_global_map(self, pc_msg):
        with self._lock:
            gmap = o3d.geometry.PointCloud()
            gmap.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
            self.global_map = voxel_down_sample(gmap, self.map_voxel_size)
        rospy.loginfo('Global map loaded successfully (%d points).', len(self.global_map.points))

    def cb_save_cur_odom(self, odom_msg):
        with self._lock:
            self.cur_odom = odom_msg

    def cb_save_cur_scan(self, pc_msg):
        pc_msg.header.frame_id = 'odom'
        pc_msg.header.stamp = rospy.Time.now()
        self.pub_pc_in_map.publish(pc_msg)

        # Fix FAST-LIO fields layout if necessary
        if len(pc_msg.fields) >= 8:
            pc_msg.fields = [
                pc_msg.fields[0],
                pc_msg.fields[1],
                pc_msg.fields[2],
                pc_msg.fields[4],
                pc_msg.fields[5],
                pc_msg.fields[6],
                pc_msg.fields[3],
                pc_msg.fields[7],
            ]
        pc = msg_to_array(pc_msg)

        scan_pcd = o3d.geometry.PointCloud()
        scan_pcd.points = o3d.utility.Vector3dVector(pc[:, :3])

        with self._lock:
            self.cur_scan = scan_pcd
            pending = self.pending_initial_pose

        if pending is not None:
            rospy.loginfo('Executing pending initialpose now that first scan arrived...')
            with self._lock:
                self.pending_initial_pose = None
            self.run_global_localization(pending)

    def click_pose_to_map_odom(self, pose_msg):
        initial = pose_to_mat(pose_msg)
        with self._lock:
            cur_odom_copy = copy.deepcopy(self.cur_odom)
        if cur_odom_copy is not None:
            initial = np.matmul(initial, inverse_se3(pose_to_mat(cur_odom_copy)))
        return initial

    def cb_initialpose(self, pose_msg):
        rospy.loginfo('\n>>> [fast_lio_localization] /initialpose received from 2D Pose Estimate! <<<')
        initial_pose = self.click_pose_to_map_odom(pose_msg)

        with self._lock:
            have_scan = self.cur_scan is not None
            have_map = self.global_map is not None

        if not have_map:
            rospy.logwarn('Global map not ready yet, buffering initial pose.')
            with self._lock:
                self.pending_initial_pose = initial_pose
            return

        if not have_scan:
            rospy.logwarn('Current scan not ready yet, buffering initial pose.')
            with self._lock:
                self.pending_initial_pose = initial_pose
            return

        threading.Thread(target=self.run_global_localization, args=(initial_pose,), daemon=True).start()

    def crop_global_map_in_fov(self, global_map, pose_estimation, cur_odom):
        T_odom_to_base_link = pose_to_mat(cur_odom)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = inverse_se3(T_map_to_base_link)

        global_map_in_map = np.array(global_map.points)
        global_map_in_map_homo = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map_homo.T).T

        dist_sq = global_map_in_base_link[:, 0] ** 2 + global_map_in_base_link[:, 1] ** 2
        if self.fov > 3.14:
            # 360 LiDAR: filter by radius
            indices = np.where(dist_sq < (self.fov_far ** 2))[0]
        else:
            # Forward-facing LiDAR
            angles = np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0]))
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0)
                & (dist_sq < (self.fov_far ** 2))
                & (angles < self.fov / 2.0)
            )[0]

        if len(indices) == 0:
            return global_map

        cropped = o3d.geometry.PointCloud()
        cropped.points = o3d.utility.Vector3dVector(global_map_in_map[indices, :3])

        # Publish submap for RViz visualization
        header = cur_odom.header
        header.frame_id = 'map'
        publish_point_cloud(self.pub_submap, header, np.array(cropped.points)[::10])
        return cropped

    def run_global_localization(self, pose_estimation):
        with self._lock:
            if self.global_map is None or self.cur_scan is None or self.cur_odom is None:
                rospy.logwarn('Localization prerequisites not ready yet.')
                return False
            gmap_copy = copy.deepcopy(self.global_map)
            scan_copy = copy.deepcopy(self.cur_scan)
            odom_copy = copy.deepcopy(self.cur_odom)

        tic = time.time()
        submap = self.crop_global_map_in_fov(gmap_copy, pose_estimation, odom_copy)

        # 粗配准 (Coarse registration, scale=5)
        coarse_scan = voxel_down_sample(scan_copy, self.scan_voxel_size * 5)
        coarse_map = voxel_down_sample(submap, self.map_voxel_size * 5)
        res_coarse = o3d.pipelines.registration.registration_icp(
            coarse_scan,
            coarse_map,
            5.0,
            pose_estimation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
        )

        # 精配准 (Fine registration, scale=1)
        fine_scan = voxel_down_sample(scan_copy, self.scan_voxel_size)
        fine_map = voxel_down_sample(submap, self.map_voxel_size)
        res_fine = o3d.pipelines.registration.registration_icp(
            fine_scan,
            fine_map,
            1.0,
            res_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
        )
        toc = time.time()

        fitness = res_fine.fitness
        rospy.loginfo(
            '[fast_lio_localization] ICP cost: %.3fs, fitness score: %.4f (threshold: %.2f)',
            toc - tic,
            fitness,
            self.localization_th,
        )

        if fitness >= self.localization_th:
            with self._lock:
                self.T_map_to_odom = res_fine.transformation
                self.initialized = True
                stamp = self.cur_odom.header.stamp if self.cur_odom else rospy.Time.now()

            map_to_odom = Odometry()
            xyz = tf.transformations.translation_from_matrix(self.T_map_to_odom)
            quat = tf.transformations.quaternion_from_matrix(self.T_map_to_odom)
            map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            map_to_odom.header.stamp = stamp
            map_to_odom.header.frame_id = 'map'
            self.pub_map_to_odom.publish(map_to_odom)
            rospy.loginfo('>>> Localization MATCH SUCCEEDED! Map->Odom transform published. <<<\n')
            return True
        else:
            rospy.logwarn(
                '>>> Localization NOT MATCHED (fitness: %.4f < %.2f). Please click closer to true pose. <<<\n',
                fitness,
                self.localization_th,
            )
            return False

    def thread_localization_loop(self):
        while not rospy.is_shutdown():
            rospy.sleep(1.0 / max(0.01, self.freq_localization))
            with self._lock:
                is_init = self.initialized
                cur_trans = copy.deepcopy(self.T_map_to_odom)
            if is_init:
                self.run_global_localization(cur_trans)


def main():
    rospy.init_node('fast_lio_localization')
    FastLioLocalization()
    rospy.spin()


if __name__ == '__main__':
    main()