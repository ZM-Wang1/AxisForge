from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_share_dir = get_package_share_directory('print_control')
    gcode_file = os.path.join(package_share_dir, 'gcode', 'star3d.gcode')

    file_launch_arg = DeclareLaunchArgument(
        'file',
        default_value=gcode_file,
        description='Full path to the G-code file to be executed'
    )

    move_ur_node = Node(
        package='ur_control',
        executable='move_ur',
        name='move_ur_node',
        output='screen'
    )

    keyboard_node = Node(
        package='print_control',
        executable='keyboard_node',
        name='keyboard_node',
        output='screen',
        emulate_tty=True,
        prefix='xterm -e',
    )

    gcode_interpreter_node = Node(
        package='print_control',
        executable='gcode_interpreter',
        name='gcode_interpreter',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('axisforge_bringup'),
                'config', 'ur_master.yaml',
            ),
            {'file': LaunchConfiguration('file')}
        ]
    )

    delayed_gcode_node = TimerAction(
        period=4.0,
        actions=[gcode_interpreter_node]
    )

    return LaunchDescription([
        file_launch_arg,
        move_ur_node,
        keyboard_node,
        delayed_gcode_node
    ])
