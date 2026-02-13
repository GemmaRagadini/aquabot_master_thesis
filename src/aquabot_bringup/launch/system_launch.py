from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    bringup_pkg = FindPackageShare('aquabot_bringup')
    mock_control_launch = PathJoinSubstitution( [bringup_pkg, 'launch', 'mock_ros2_control.launch.py'])

    return LaunchDescription([  
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mock_control_launch)
        ),
        
        Node(
            package='arduino_reader',
            executable='arduino_reader_node',
            name='arduino_reader_node',
            output='screen',
        ),
        
        Node(
            package='master',
            executable='master_node',
            name='master_node',
            output='screen',
        ),
        
        Node(
            package='dynamixel_controller',
            executable='dynamixel_controller_node',
            name='dynamixel_controller_node',
            output='screen',
            parameters=[{
                'frequency_hz': 0.5,
                'left_rad': 0.0,
                'right_rad':3.14159,
            }],
        ),
    ])
