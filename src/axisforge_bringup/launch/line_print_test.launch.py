"""Launch only the configurable single-line test."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Load the installed YAML or a caller-supplied configuration path."""
    config = os.path.join(
        get_package_share_directory('axisforge_bringup'), 'config', 'line_print_test.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=config, description='ROS parameter YAML file',
        ),
        Node(
            package='print_control', executable='line_print_test', name='line_print_test',
            output='screen', parameters=[LaunchConfiguration('config')],
        ),
    ])
