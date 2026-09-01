"""Point-cloud preprocessing before AnyGrasp inference."""

import numpy as np
import open3d as o3d

from anygrasp_ros.core import FilteredCloud


def remove_statistical_outliers(cloud, nb_neighbors, std_ratio):
    """Remove points with statistically abnormal neighbour distances."""

    nb_neighbors = int(nb_neighbors)
    std_ratio = float(std_ratio)

    if nb_neighbors < 1:
        raise ValueError("nb_neighbors must be at least 1")
    if std_ratio <= 0.0:
        raise ValueError("std_ratio must be positive")

    point_count = int(cloud.points.shape[0])
    if point_count <= nb_neighbors:
        return cloud

    open3d_cloud = o3d.geometry.PointCloud()
    open3d_cloud.points = o3d.utility.Vector3dVector(
        np.asarray(cloud.points, dtype=np.float64)
    )

    _, inlier_indices = open3d_cloud.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
    )
    inlier_indices = np.asarray(inlier_indices, dtype=np.int64)

    points = np.asarray(
        cloud.points[inlier_indices],
        dtype=np.float32,
    )
    colors = np.asarray(
        cloud.colors[inlier_indices],
        dtype=np.float32,
    )

    return FilteredCloud(
        points=points,
        colors=colors,
        raw_count=cloud.raw_count,
        valid_count=cloud.valid_count,
        workspace_count=int(points.shape[0]),
    )