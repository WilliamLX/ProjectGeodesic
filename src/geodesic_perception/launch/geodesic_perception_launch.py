"""
Launch file for Geodesic Perception package
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    """Generate launch description."""
    # Get package directory
    pkg_dir = get_package_share_directory('geodesic_perception')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'alignment_params.yaml'),
        description='Path to configuration file'
    )

    # Load configuration
    config_file = LaunchConfiguration('config_file')

    # Global alignment node
    global_alignment_node = Node(
        package='geodesic_perception',
        executable='global_alignment_node',
        name='global_alignment_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/camera/pointcloud2', '/camera/pointcloud2')
        ]
    )

    # Optional: Point cloud visualization
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='false',
        description='Launch point cloud visualizer'
    )

    return LaunchDescription([
        config_file_arg,
        visualize_arg,
        global_alignment_node,
    ])
