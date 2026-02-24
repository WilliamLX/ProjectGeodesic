"""
Point Cloud Processing Module

Provides preprocessing functions for point cloud data including:
- Voxel downsampling
- Statistical outlier removal
- Normal estimation
"""

import open3d as o3d
import numpy as np
from typing import Tuple, Optional


class PointCloudProcessor:
    """Point cloud preprocessing and utilities."""

    def __init__(self, voxel_size: float = 0.005):
        """
        Initialize point cloud processor.

        Args:
            voxel_size: Size of voxel grid for downsampling (meters)
        """
        self.voxel_size = voxel_size

    def preprocess(self, pcd: o3d.geometry.PointCloud,
                   enable_downsampling: bool = True,
                   enable_denoising: bool = True,
                   enable_normals: bool = False) -> o3d.geometry.PointCloud:
        """
        Apply full preprocessing pipeline.

        Args:
            pcd: Input point cloud
            enable_downsampling: Apply voxel downsampling
            enable_denoising: Apply statistical outlier removal
            enable_normals: Estimate normals

        Returns:
            Preprocessed point cloud
        """
        processed = o3d.geometry.PointCloud(pcd)

        if enable_downsampling:
            processed = self.voxel_downsample(processed, self.voxel_size)

        if enable_denoising:
            processed = self.remove_statistical_outliers(processed)

        if enable_normals:
            processed = self.estimate_normals(processed)

        return processed

    def voxel_downsample(self, pcd: o3d.geometry.PointCloud,
                         voxel_size: Optional[float] = None) -> o3d.geometry.PointCloud:
        """
        Downsample point cloud using voxel grid filter.

        Args:
            pcd: Input point cloud
            voxel_size: Voxel size (uses self.voxel_size if None)

        Returns:
            Downsampled point cloud
        """
        if voxel_size is None:
            voxel_size = self.voxel_size

        if voxel_size <= 0:
            return pcd

        downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
        return downsampled

    def remove_statistical_outliers(self,
                                     pcd: o3d.geometry.PointCloud,
                                     nb_neighbors: int = 20,
                                     std_ratio: float = 2.0) -> o3d.geometry.PointCloud:
        """
        Remove statistical outliers from point cloud.

        Args:
            pcd: Input point cloud
            nb_neighbors: Number of neighbors to analyze
            std_ratio: Standard deviation ratio threshold

        Returns:
            Cleaned point cloud
        """
        cl, ind = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )
        return cl

    def estimate_normals(self,
                         pcd: o3d.geometry.PointCloud,
                         search_radius: float = 0.1,
                         max_nn: int = 30) -> o3d.geometry.PointCloud:
        """
        Estimate normals for point cloud.

        Args:
            pcd: Input point cloud
            search_radius: Search radius for KD-tree
            max_nn: Maximum number of neighbors

        Returns:
            Point cloud with normals
        """
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=search_radius,
                max_nn=max_nn
            )
        )

        # Orient normals for consistent visualization
        pcd.orient_normals_consistent_tangent_plane(30)

        return pcd

    def compute_fpfh_feature(self,
                             pcd: o3d.geometry.PointCloud,
                             search_radius: Optional[float] = None) -> o3d.pipelines.registration.Feature:
        """
        Compute FPFH (Fast Point Feature Histograms) for point cloud.

        Args:
            pcd: Input point cloud (must have normals)
            search_radius: Search radius (defaults to 5x voxel_size)

        Returns:
            FPFH feature descriptor
        """
        if search_radius is None:
            search_radius = self.voxel_size * 5

        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=search_radius,
                    max_nn=30
                )
            )

        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd,
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=search_radius,
                max_nn=100
            )
        )

        return fpfh

    def get_pointcloud_info(self, pcd: o3d.geometry.PointCloud) -> dict:
        """
        Get information about point cloud.

        Args:
            pcd: Input point cloud

        Returns:
            Dictionary with point cloud statistics
        """
        info = {
            'num_points': len(pcd.points),
            'has_normals': pcd.has_normals(),
            'has_colors': pcd.has_colors(),
        }

        if len(pcd.points) > 0:
            points = np.asarray(pcd.points)
            info['bounds_min'] = points.min(axis=0).tolist()
            info['bounds_max'] = points.max(axis=0).tolist()
            info['centroid'] = points.mean(axis=0).tolist()

        return info


def ros_to_open3d(ros_msg) -> o3d.geometry.PointCloud:
    """
    Convert ROS PointCloud2 message to Open3D PointCloud.

    Args:
        ros_msg: sensor_msgs.msg.PointCloud2

    Returns:
        open3d.geometry.PointCloud
    """
    import sensor_msgs_py.point_cloud2 as pc2
    import struct

    # Extract xyz from point cloud
    pc_data = pc2.read_points(ros_msg, field_names=("x", "y", "z"), skip_nans=True)

    points = np.array(list(pc_data))

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd


def open3d_to_ros(pcd: o3d.geometry.PointCloud, frame_id: str = "camera"):
    """
    Convert Open3D PointCloud to ROS PointCloud2 message.

    Args:
        pcd: open3d.geometry.PointCloud
        frame_id: TF frame ID

    Returns:
        sensor_msgs.msg.PointCloud2
    """
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    import struct

    header = Header()
    header.stamp = rclpy.clock.Clock().now().to_msg()
    header.frame_id = frame_id

    points = np.asarray(pcd.points)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)

    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    msg.is_bigendian = False
    msg.point_step = 12  # 3 * 4 bytes (float32)
    msg.row_step = msg.point_step * msg.width

    msg.data = points.astype(np.float32).tobytes()

    return msg
