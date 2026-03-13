from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='master',
            executable='master_node',
            name='master_node',
            output='screen',
            parameters=[{
                'sensor_topic': '/sensor_reading',
                'target_topic': '/aquabot/dynamixel/target_position',
                'traj': 'std',
                'tail_bias_rad': 0.0,
                'tail_amp_rad': 0.4,
                'tail_freq_hz': 1.0,
                'tail_min_rad': -0.7,
                'tail_max_rad': 0.7,
                'control_rate_hz': 50.0,
                'log_rate_hz': 20.0,
                'log_dir': './logs',
            }]
        ),
        Node(
            package='dynamixel_controller',
            executable='dynamixel_controller_node',
            name='dynamixel_controller_node',
            output='screen',
            parameters=[{
                'target_topic': '/aquabot/dynamixel/target_position',
                'device_name': '/dev/ttyUSB0',
                'baudrate': 115200,
                'dxl_id': 1,
                'protocol_version': 2.0,
                'tail_min_rad': -0.7,
                'tail_max_rad': 0.7,
                'profile_velocity': 100,
                'profile_acceleration':10,
            }]
        ),
        Node(
            package='arduino_reader',
            executable='arduino_reader_node',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud_rate': 57600
            }]
        ),
    ])

