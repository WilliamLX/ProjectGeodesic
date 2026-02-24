"""
Test Camera Node

Publishes dummy point cloud data for testing when no real camera is available.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import time


class TestCameraNode(Node):
    """Test camera node that publishes synthetic point clouds."""

    def __init__(self):
        super().__init__('test_camera_node')

        # Parameters
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('num_points', 10000)

        self.publish_rate = self.get_parameter('publish_rate').value
        self.num_points = self.get_parameter('num_points').value

        # Publisher
        self.pcd_pub = self.create_publisher(
            PointCloud2,
            '/camera/pointcloud2',
            10
        )

        # Timer
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f'Test Camera Node started')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Points per cloud: {self.num_points}')

        # Frame counter
        self.frame_count = 0

    def timer_callback(self):
        """Publish test point cloud."""
        self.frame_count += 1

        # Generate synthetic point cloud
        # Create a flat plane with some noise (simulating a workpiece)
        x = np.random.uniform(-0.3, 0.3, self.num_points)
        y = np.random.uniform(-0.2, 0.2, self.num_points)
        z = np.random.normal(0.5, 0.002, self.num_points)  # Plane at z=0.5

        # Stack points
        points = np.column_stack([x, y, z])

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        msg.height = 1
        msg.width = self.num_points

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12  # 3 * 4 bytes
        msg.row_step = msg.point_step * msg.width

        msg.data = points.astype(np.float32).tobytes()

        self.pcd_pub.publish(msg)

        if self.frame_count % 10 == 0:
            self.get_logger().info(f'Published frame {self.frame_count}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = TestCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
