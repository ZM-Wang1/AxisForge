"""Launch the G-code toolpath planner and acknowledged trajectory executor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Keep the original entry point; one node owns motion and extrusion."""
    config = os.path.join(
        get_package_share_directory('axisforge_bringup'), 'config', 'ur_master.yaml',
    )
    gcode_file = os.path.join(get_package_share_directory('print_control'), 'gcode', 'star3d.gcode')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config, description='ROS parameter YAML'),
        DeclareLaunchArgument('file', default_value=gcode_file, description='G-code file path'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='Plan and check only; send no hardware commands'),
        Node(
            package='print_control', executable='gcode_interpreter', name='gcode_interpreter',
            output='screen', parameters=[LaunchConfiguration('config'), {
                'file': ParameterValue(LaunchConfiguration('file'), value_type=str),
                'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
            }],
        ),
    ])
