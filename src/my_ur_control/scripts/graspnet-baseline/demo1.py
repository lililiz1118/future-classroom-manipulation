#!/home/jt001/.conda/envs/graspnet/bin/python
# -*- coding: utf-8 -*-

""" Demo to show prediction results.
    Author: chenxi-wang
"""

import os
import sys
import numpy as np
import open3d as o3d
import argparse
import importlib
import scipy.io as scio
from PIL import Image
import cv2


import torch
from graspnetAPI import GraspGroup

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from graspnet import GraspNet, pred_decode
from graspnet_dataset import GraspNetDataset
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', required=True, help='Model checkpoint path')
parser.add_argument('--data_dir', default='doc/d405_data_pre/cola1', help='Directory containing color.png/depth.png/workspace_mask.png/meta.mat')
parser.add_argument('--num_point', type=int, default=20000, help='Point Number [default: 20000]')
parser.add_argument('--num_view', type=int, default=300, help='View Number [default: 300]')
parser.add_argument('--median_ksize', type=int, default=5, help='Odd kernel size for median filtering depth (<=1 disables)')
parser.add_argument('--bilateral_diameter', type=int, default=0, help='Bilateral filter diameter on depth (0 disables)')
parser.add_argument('--bilateral_sigma', type=float, default=25.0, help='Sigma for bilateral filter when enabled')
parser.add_argument('--mask_kernel', type=int, default=3, help='Workspace mask morphology kernel size (<=1 disables)')
parser.add_argument('--voxel_downsample', type=float, default=0.0, help='Voxel size for downsampling masked cloud (0 disables)')
parser.add_argument('--remove_outlier', action='store_true', help='Enable statistical outlier removal on masked cloud')
parser.add_argument('--outlier_nb_neighbors', type=int, default=20, help='Neighbors for statistical outlier removal')
parser.add_argument('--outlier_std_ratio', type=float, default=2.0, help='Std ratio for statistical outlier removal')
parser.add_argument('--top_k', type=int, default=10, help='Number of top grasps to keep for visualization')
parser.add_argument('--score_thresh', type=float, default=0.0, help='Minimum score threshold for grasps before NMS')
parser.add_argument('--collision_thresh', type=float, default=0.01, help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument('--collision_approach_dist', type=float, default=0.05, help='Approach distance for collision checking')
parser.add_argument('--voxel_size', type=float, default=0.01, help='Voxel Size to process point clouds before collision detection [default: 0.01]')
parser.add_argument('--no_collision', action='store_true', help='Skip collision detection stage')
parser.add_argument('--headless', action='store_true', help='Skip Open3D visualization (log grasps only)')
cfgs = parser.parse_args()


def get_net():
    # Init the model
    net = GraspNet(input_feature_dim=0, num_view=cfgs.num_view, num_angle=12, num_depth=4,
            cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    # Load checkpoint
    checkpoint = torch.load(cfgs.checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    print("-> loaded checkpoint %s (epoch: %d)"%(cfgs.checkpoint_path, start_epoch))
    # set model to eval mode
    net.eval()
    return net

def _apply_depth_filters(depth):
    depth_filtered = depth.astype(np.float32)
    if cfgs.median_ksize and cfgs.median_ksize > 1:
        k = cfgs.median_ksize if cfgs.median_ksize % 2 == 1 else cfgs.median_ksize + 1
        depth_filtered = cv2.medianBlur(depth_filtered, k)
    if cfgs.bilateral_diameter and cfgs.bilateral_diameter > 0:
        d = cfgs.bilateral_diameter
        sigma = max(cfgs.bilateral_sigma, 1e-3)
        depth_filtered = cv2.bilateralFilter(depth_filtered, d, sigma, sigma)
    return depth_filtered


def _refine_workspace_mask(mask):
    if cfgs.mask_kernel and cfgs.mask_kernel > 1:
        k = cfgs.mask_kernel if cfgs.mask_kernel % 2 == 1 else cfgs.mask_kernel + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def _post_process_cloud(cloud_masked, color_masked):
    cloud_o3d = o3d.geometry.PointCloud()
    cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud_o3d.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))

    if cfgs.voxel_downsample and cfgs.voxel_downsample > 0:
        cloud_o3d = cloud_o3d.voxel_down_sample(cfgs.voxel_downsample)

    if cfgs.remove_outlier:
        if len(cloud_o3d.points) == 0:
            raise RuntimeError('No points available for outlier removal.')
        cloud_o3d, _ = cloud_o3d.remove_statistical_outlier(nb_neighbors=cfgs.outlier_nb_neighbors,
                                                            std_ratio=cfgs.outlier_std_ratio)

    processed_points = np.asarray(cloud_o3d.points)
    processed_colors = np.asarray(cloud_o3d.colors)
    if processed_points.size == 0:
        raise RuntimeError('Point cloud became empty after preprocessing; check masks and filters.')
    return processed_points, processed_colors


def get_and_process_data(data_dir):
    # load data
    color = np.array(Image.open(os.path.join(data_dir, 'color.png')), dtype=np.float32) / 255.0
    depth = np.array(Image.open(os.path.join(data_dir, 'depth.png')))
    depth = _apply_depth_filters(depth)
    workspace_mask = np.array(Image.open(os.path.join(data_dir, 'workspace_mask.png'))) > 0
    workspace_mask = _refine_workspace_mask(workspace_mask.astype(np.uint8)).astype(bool)
    meta = scio.loadmat(os.path.join(data_dir, 'meta.mat'))
    intrinsic = meta['intrinsic_matrix']
    factor_depth = meta['factor_depth']


    # generate cloud
    camera = CameraInfo(1280.0, 720.0, intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2], factor_depth)
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # get valid points
    mask = (workspace_mask & (depth > 0))
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    cloud_masked, color_masked = _post_process_cloud(cloud_masked, color_masked)

    # sample points
    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point-len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # convert data
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    return end_points, cloud

def get_grasps(net, end_points):
    # Forward pass
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg

def collision_detection(gg, cloud):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=cfgs.voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=cfgs.collision_approach_dist,
                                        collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    return gg

def vis_grasps(gg, cloud):
    if cfgs.score_thresh > 0:
        gg = gg[gg.scores >= cfgs.score_thresh]
    gg.nms()
    gg.sort_by_score()
    top_k = min(cfgs.top_k, len(gg))
    gg = gg[:10]
    grippers = gg.to_open3d_geometry_list()
    print(gg)
    # 创建一个“世界坐标系”显示对象
    world_axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.1,      # 坐标轴长度（根据点云大小调整）
        origin=[0, 0, 0]  # 坐标原点
    )

    if cfgs.headless:
        return gg

    o3d.visualization.draw_geometries([cloud, *grippers, world_axis])
    return gg

def demo(data_dir):
    net = get_net()
    end_points, cloud = get_and_process_data(data_dir) # 采样后的导弹
    gg = get_grasps(net, end_points)
    if not cfgs.no_collision and cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    gg = vis_grasps(gg, cloud)
    if cfgs.headless:
        print(f"Kept {len(gg)} grasps after filtering. Top scores: {gg.scores[:min(5, len(gg))]}")

if __name__=='__main__':
    # data_dir = 'doc/example_data'
    demo(cfgs.data_dir)
