"""
Point Cloud Registration Module

Implements point cloud registration algorithms:
- PCA-based coarse registration
- FPFH feature-based registration
- ICP fine registration
- Point-to-Plane ICP
"""

import open3d as o3d
import numpy as np
from typing import Tuple, Optional, Dict
from dataclasses import dataclass

from .pointcloud_proc import PointCloudProcessor


@dataclass
class RegistrationResult:
    """Result of point cloud registration."""
    transformation: np.ndarray  # 4x4 transformation matrix
    rmse: float  # Root mean square error
    fitness: float  # Overlap fitness score [0, 1]
    num_iterations: int
    method: str


class Registration:
    """Point cloud registration algorithms."""

    def __init__(self, voxel_size: float = 0.005):
        """
        Initialize registration module.

        Args:
            voxel_size: Voxel size for downsampling (affects feature computation)
        """
        self.voxel_size = voxel_size
        self.processor = PointCloudProcessor(voxel_size)

    def pca_alignment(self,
                      source: o3d.geometry.PointCloud,
                      target: o3d.geometry.PointCloud) -> np.ndarray:
        """
        Perform coarse alignment using PCA (Principal Component Analysis).

        This method aligns source to target by matching their principal axes.
        Good for initial alignment when poses are roughly known.

        Args:
            source: Source point cloud
            target: Target point cloud

        Returns:
            4x4 transformation matrix
        """
        # Compute centroids
        source_points = np.asarray(source.points)
        target_points = np.asarray(target.points)

        source_center = source_points.mean(axis=0)
        target_center = target_points.mean(axis=0)

        # Center the point clouds
        source_centered = source_points - source_center
        target_centered = target_points - target_center

        # Compute covariance matrix
        H = source_centered.T @ target_centered

        # SVD to find rotation
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # Handle reflection case
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # Compute translation
        t = target_center - R @ source_center

        # Build transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        return T

    def fpfh_registration(self,
                          source: o3d.geometry.PointCloud,
                          target: o3d.geometry.PointCloud,
                          voxel_size: Optional[float] = None) -> np.ndarray:
        """
        Perform feature-based registration using FPFH.

        Uses RANSAC to find correspondence based on FPFH features.
        More robust than PCA but slower.

        Args:
            source: Source point cloud (must have normals)
            target: Target point cloud (must have normals)
            voxel_size: Voxel size for feature computation

        Returns:
            4x4 transformation matrix
        """
        if voxel_size is None:
            voxel_size = self.voxel_size

        # Downsample for faster feature computation
        source_down = source.voxel_down_sample(voxel_size)
        target_down = target.voxel_down_sample(voxel_size)

        # Estimate normals if needed
        if not source_down.has_normals():
            source_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=voxel_size * 2,
                    max_nn=30
                )
            )
        if not target_down.has_normals():
            target_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=voxel_size * 2,
                    max_nn=30
                )
            )

        # Compute FPFH features
        source_fpfh = self.processor.compute_fpfh_feature(source_down, voxel_size * 5)
        target_fpfh = self.processor.compute_fpfh_feature(target_down, voxel_size * 5)

        # RANSAC registration
        distance_threshold = voxel_size * 1.5
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down,
            source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold
            )
        )

        return result.transformation

    def icp_registration(self,
                         source: o3d.geometry.PointCloud,
                         target: o3d.geometry.PointCloud,
                         init_transform: Optional[np.ndarray] = None,
                         max_correspondence_distance: float = 0.02,
                         point_to_plane: bool = True) -> RegistrationResult:
        """
        Perform ICP (Iterative Closest Point) fine registration.

        Args:
            source: Source point cloud
            target: Target point cloud
            init_transform: Initial transformation (4x4 matrix)
            max_correspondence_distance: Max distance for correspondence (meters)
            point_to_plane: Use Point-to-Plane ICP (requires normals)

        Returns:
            RegistrationResult with transformation and quality metrics
        """
        if init_transform is None:
            init_transform = np.eye(4)

        # Estimate normals for point-to-plane ICP
        if point_to_plane:
            if not source.has_normals():
                source.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(
                        radius=0.1, max_nn=30
                    )
                )
            if not target.has_normals():
                target.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(
                        radius=0.1, max_nn=30
                    )
                )

            estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane()
            method_name = "Point-to-Plane ICP"
        else:
            estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
            method_name = "Point-to-Point ICP"

        # Run ICP
        result = o3d.pipelines.registration.registration_icp(
            source, target,
            max_correspondence_distance,
            init_transform,
            estimation_method,
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=50
            )
        )

        return RegistrationResult(
            transformation=result.transformation,
            rmse=result.inlier_rmse,
            fitness=result.fitness,
            num_iterations=result.num_iteration,
            method=method_name
        )

    def multi_scale_icp(self,
                        source: o3d.geometry.PointCloud,
                        target: o3d.geometry.PointCloud,
                        init_transform: Optional[np.ndarray] = None,
                        scales: list = [0.04, 0.02, 0.01]) -> RegistrationResult:
        """
        Multi-scale ICP for better convergence.

        Starts with coarse voxel size and progressively refines.

        Args:
            source: Source point cloud
            target: Target point cloud
            init_transform: Initial transformation
            scales: List of max correspondence distances (coarse to fine)

        Returns:
            RegistrationResult with final transformation
        """
        current_transform = init_transform if init_transform is not None else np.eye(4)

        for i, distance_threshold in enumerate(scales):
            result = self.icp_registration(
                source, target,
                init_transform=current_transform,
                max_correspondence_distance=distance_threshold,
                point_to_plane=True
            )
            current_transform = result.transformation

        result.method = "Multi-scale ICP"
        return result

    def colored_icp(self,
                    source: o3d.geometry.PointCloud,
                    target: o3d.geometry.PointCloud,
                    init_transform: Optional[np.ndarray] = None,
                    voxel_size: Optional[float] = None,
                    max_iter: int = 50) -> RegistrationResult:
        """
        Colored ICP - uses both geometry and color information.

        Requires point clouds with RGB colors.

        Args:
            source: Source point cloud (must have colors)
            target: Target point cloud (must have colors)
            init_transform: Initial transformation
            voxel_size: Voxel size for downsampling
            max_iter: Maximum iterations

        Returns:
            RegistrationResult
        """
        if not source.has_colors() or not target.has_colors():
            raise ValueError("Colored ICP requires point clouds with RGB colors")

        if voxel_size is None:
            voxel_size = self.voxel_size

        # Downsample
        source_down = source.voxel_down_sample(voxel_size)
        target_down = target.voxel_down_sample(voxel_size)

        # Run Colored ICP
        result = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down,
            voxel_size * 1.5,  # max_correspondence_distance
            init_transform if init_transform is not None else np.eye(4),
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,
                relative_rmse=1e-6,
                max_iteration=max_iter
            )
        )

        return RegistrationResult(
            transformation=result.transformation,
            rmse=result.inlier_rmse,
            fitness=result.fitness,
            num_iterations=result.num_iteration,
            method="Colored ICP"
        )

    def evaluate_registration(self,
                             source: o3d.geometry.PointCloud,
                             target: o3d.geometry.PointCloud,
                             transform: np.ndarray,
                             max_correspondence_distance: float = 0.02) -> Dict:
        """
        Evaluate registration quality.

        Args:
            source: Source point cloud
            target: Target point cloud
            transform: Transformation to evaluate
            max_correspondence_distance: Max distance for correspondences

        Returns:
            Dictionary with evaluation metrics
        """
        # Transform source
        source_transformed = source.transform(transform)

        # Compute nearest neighbor distances
        pcd_tree = o3d.geometry.KDTreeFlann(target)

        distances = []
        for point in source_transformed.points:
            [_, idx, _] = pcd_tree.search_radius_vector_3d(point, max_correspondence_distance)
            if len(idx) > 0:
                # Get distance to nearest neighbor
                [k, idx, _] = pcd_tree.search_knn_vector_3d(point, 1)
                p2 = np.asarray(target.points)[idx[0]]
                distances.append(np.linalg.norm(np.array(point) - p2))

        if len(distances) == 0:
            return {
                'mean_distance': float('inf'),
                'std_distance': 0,
                'max_distance': float('inf'),
                'num_correspondences': 0
            }

        distances = np.array(distances)

        return {
            'mean_distance': float(np.mean(distances)),
            'std_distance': float(np.std(distances)),
            'max_distance': float(np.max(distances)),
            'num_correspondences': len(distances)
        }


def transform_points(points: np.ndarray, transformation: np.ndarray) -> np.ndarray:
    """
    Transform 3D points using 4x4 transformation matrix.

    Args:
        points: Nx3 array of points
        transformation: 4x4 transformation matrix

    Returns:
        Transformed points (Nx3)
    """
    # Convert to homogeneous coordinates
    ones = np.ones((points.shape[0], 1))
    points_h = np.hstack([points, ones])

    # Apply transformation
    transformed_h = (transformation @ points_h.T).T

    return transformed_h[:, :3]


def get_euler_angles(transformation: np.ndarray) -> Tuple[float, float, float]:
    """
    Extract Euler angles (XYZ) from transformation matrix.

    Args:
        transformation: 4x4 transformation matrix

    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    R = transformation[:3, :3]

    # Extract Euler angles (XYZ sequence)
    pitch = np.arcsin(-R[2, 0])

    if np.cos(pitch) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock case
        roll = 0
        yaw = np.arctan2(-R[0, 1], R[1, 1])

    return (roll, pitch, yaw)
