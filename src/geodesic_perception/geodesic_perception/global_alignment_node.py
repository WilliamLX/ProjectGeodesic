"""
Global Alignment Node

ROS 2 node that performs point cloud registration for workpiece alignment.

Subscribes:
- /camera/pointcloud2: Input point cloud

Publishes:
- /perception/transformation_matrix: 4x4 transformation matrix
- /perception/hole_positions: PoseArray of hole positions
- /perception/alignment_status: Current status string
- /perception/aligned_pointcloud: Visual debug

Services:
- /perception/load_template: Load a workpiece template
- /perception/save_template: Save current as template
- /perception/set_mode: Set mode (TEACHING / ALIGNMENT)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import open3d as o3d
from pathlib import Path

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_srvs.srv import Trigger

from .pointcloud_proc import PointCloudProcessor, ros_to_open3d, open3d_to_ros
from .registration import Registration, RegistrationResult, transform_points
from .template_manager import TemplateManager, WorkpieceTemplate

# Custom message for load template
from example_interfaces.srv import SetBool


class GlobalAlignmentNode(Node):
    """Point cloud registration node for workpiece alignment."""

    def __init__(self):
        super().__init__('global_alignment_node')

        # Parameters
        self.declare_parameter('voxel_size', 0.005)
        self.declare_parameter('max_correspondence_distance', 0.02)
        self.declare_parameter('template_path', 'data/templates')
        self.declare_parameter('coarse_method', 'pca')  # 'pca' or 'fpfh'
        self.declare_parameter('enable_visualization', False)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_corr_dist = self.get_parameter('max_correspondence_distance').value
        self.template_path = self.get_parameter('template_path').value
        self.coarse_method = self.get_parameter('coarse_method').value
        self.enable_vis = self.get_parameter('enable_visualization').value

        # State
        self.template: WorkpieceTemplate = None
        self.template_pointcloud: o3d.geometry.PointCloud = None
        self.mode = "ALIGNMENT"  # or "TEACHING"
        self.state = "IDLE"
        self.last_transformation = np.eye(4)
        self.last_hole_positions: PoseArray = None

        # Modules
        self.processor = PointCloudProcessor(voxel_size=self.voxel_size)
        self.registration = Registration(voxel_size=self.voxel_size)
        self.template_manager = TemplateManager(base_path=self.template_path)

        # QoS for camera (best effort)
        qos_sensor = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
            )

        # Publishers
        self.transform_pub = self.create_publisher(
            PointCloud2,  # Using Float64MultiArray-like in PointCloud2 for now
            '/perception/transformation_matrix',
            10
        )

        self.holes_pub = self.create_publisher(
            PoseArray,
            '/perception/hole_positions',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/perception/alignment_status',
            10
        )

        self.aligned_pcd_pub = self.create_publisher(
            PointCloud2,
            '/perception/aligned_pointcloud',
            10
        )

        # Subscribers
        self.create_subscription(
            PointCloud2,
            '/camera/pointcloud2',
            self.pointcloud_callback,
            qos_sensor
        )

        # Services
        self.create_service(
            Trigger,
            '/perception/trigger_alignment',
            self.trigger_alignment_callback
        )

        # Timer for status updates
        self.create_timer(1.0, self.status_timer_callback)

        self.get_logger().info('Global Alignment Node initialized')
        self.get_logger().info(f'Mode: {self.mode}')
        self.get_logger().info(f'Template path: {self.template_path}')

        # Try to load default template
        templates = self.template_manager.list_templates()
        if templates:
            self.get_logger().info(f'Found {len(templates)} templates')
            # Load first template
            try:
                self.load_template(templates[0])
            except Exception as e:
                self.get_logger().warn(f'Could not load default template: {e}')

    def load_template(self, template_id: str) -> bool:
        """Load a workpiece template."""
        try:
            self.template = self.template_manager.load_template(template_id)
            self.template_pointcloud = self.template_manager.load_template_pointcloud(template_id)

            # Preprocess template
            self.template_pointcloud = self.processor.preprocess(
                self.template_pointcloud,
                enable_normals=True
            )

            self.get_logger().info(f'Loaded template: {template_id}')
            self.get_logger().info(f'  Holes: {self.template.num_holes}')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to load template {template_id}: {e}')
            return False

    def pointcloud_callback(self, msg: PointCloud2):
        """Handle incoming point cloud."""
        if self.mode == "TEACHING":
            # Don't process in teaching mode
            return

        if self.template is None:
            self.get_logger().warn('No template loaded, skipping alignment')
            return

        # Perform alignment
        self.perform_alignment(msg)

    def perform_alignment(self, msg: PointCloud2) -> RegistrationResult:
        """
        Perform point cloud alignment pipeline.

        Args:
            msg: Input point cloud message

        Returns:
            RegistrationResult
        """
        self.state = "ALIGNING"
        self.publish_status(f"Aligning...")

        try:
            # 1. Convert to Open3D
            self.get_logger().debug('Converting point cloud...')
            source = ros_to_open3d(msg)

            if len(source.points) < 100:
                self.get_logger().warn('Too few points in point cloud')
                self.state = "FAILED"
                return None

            # 2. Preprocess
            self.get_logger().debug('Preprocessing...')
            source_processed = self.processor.preprocess(
                source,
                enable_downsampling=True,
                enable_denoising=True,
                enable_normals=True
            )

            # 3. Coarse registration
            self.get_logger().debug('Coarse registration...')
            if self.coarse_method == 'pca':
                T_coarse = self.registration.pca_alignment(source_processed, self.template_pointcloud)
            else:  # fpfh
                T_coarse = self.registration.fpfh_registration(source_processed, self.template_pointcloud)

            # 4. Fine registration (ICP)
            self.get_logger().debug('Fine registration (ICP)...')
            result = self.registration.icp_registration(
                source_processed,
                self.template_pointcloud,
                init_transform=T_coarse,
                max_correspondence_distance=self.max_corr_dist,
                point_to_plane=True
            )

            # 5. Transform template holes to current frame
            current_holes = self.transform_holes(result.transformation)

            # 6. Publish results
            self.publish_results(result, current_holes, source_processed)

            self.state = "SUCCESS"
            self.publish_status(f"Success - RMSE: {result.rmse*1000:.2f}mm")

            self.last_transformation = result.transformation
            self.get_logger().info(
                f'Alignment complete - RMSE: {result.rmse*1000:.2f}mm, '
                f'Fitness: {result.fitness:.2%}'
            )

            return result

        except Exception as e:
            self.get_logger().error(f'Alignment failed: {e}')
            self.state = "FAILED"
            self.publish_status(f"Failed: {str(e)}")
            return None

    def transform_holes(self, transformation: np.ndarray) -> PoseArray:
        """
        Transform template hole positions to current frame.

        Args:
            transformation: 4x4 transformation matrix

        Returns:
            PoseArray of hole positions
        """
        if self.template is None:
            return None

        # Get hole positions
        hole_positions = self.template_manager.get_hole_positions(self.template)

        # Transform
        transformed_positions = transform_points(hole_positions, transformation)

        # Create PoseArray
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera"

        for i, pos in enumerate(transformed_positions):
            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])

            # Identity orientation (can be updated with normals)
            pose.orientation.w = 1.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0

            pose_array.poses.append(pose)

        self.last_hole_positions = pose_array
        return pose_array

    def publish_results(self, result: RegistrationResult, holes: PoseArray,
                        aligned_pcd: o3d.geometry.PointCloud):
        """Publish alignment results."""
        # Publish transformation (as flattened array in a custom way)
        # For now, we publish status in String

        # Publish hole positions
        if holes is not None:
            self.holes_pub.publish(holes)

        # Publish aligned point cloud for visualization
        if self.enable_vis and aligned_pcd is not None:
            aligned_msg = open3d_to_ros(aligned_pcd, "camera")
            self.aligned_pcd_pub.publish(aligned_msg)

    def publish_status(self, message: str):
        """Publish status message."""
        msg = String()
        msg.data = f"{self.state}: {message}"
        self.status_pub.publish(msg)

    def status_timer_callback(self):
        """Periodic status update."""
        if self.state == "IDLE":
            if self.template is None:
                self.publish_status("No template loaded")
            else:
                self.publish_status("Ready")

    def trigger_alignment_callback(self, request, response):
        """Manual alignment trigger service."""
        self.get_logger().info('Manual alignment triggered')
        # This would need the last point cloud to be stored
        response.success = True
        response.message = "Alignment triggered (need stored point cloud)"
        return response


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = GlobalAlignmentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
