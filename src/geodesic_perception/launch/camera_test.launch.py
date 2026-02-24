"""
Launch file for camera testing and point cloud visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteCommand, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for camera testing."""

    # Declare arguments
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='mecmind',
        description='Camera type (mecmind, realsense, etc.)'
    )

    # Example: Simple test node that publishes dummy point cloud
    # Replace this with actual camera driver when available
    test_camera_node = Node(
        package='geodesic_perception',
        executable='test_camera_node.py',
        name='test_camera_node',
        output='screen'
    )

    return LaunchDescription([
        camera_type_arg,
        test_camera_node,
    ])
