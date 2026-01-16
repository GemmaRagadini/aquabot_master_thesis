from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('aquabot_bringup'),
        'config',
        'aquabot_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='arduino_reader',
            executable='arduino_reader_node',
            name='arduino_reader_node',
            output='screen',
            parameters=[config_path],
        ),
        Node(
            package='master',
            executable='master_node',
            name='master_node',
            output='screen',
            parameters=[config_path],
        ),
        Node(
            package='dynamixel_controller',
            executable='dynamixel_controller_node',
            name='dynamixel_controller_node',
            output='screen',
            parameters=[config_path],
        ),
    ])
