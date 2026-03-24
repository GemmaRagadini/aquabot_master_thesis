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
                'mode': 'amp_sweep', # tipo di trial
                'trial_duration_sec': 20.0, 
                'tail_bias_rad': 0.0,
                'tail_amp_rad': 1.2,
                'tail_freq_hz': 1.0,
                'tail_min_rad': -1.5,
                'tail_max_rad': 1.5,
                'control_rate_hz': 50.0,
                'log_rate_hz': 20.0,
                'log_dir': './logs',

                
                'feedback_enabled': False, # per versione variazione centro oscillazione

                'feedback_gain': 0.003,
                'feedback_alpha': 0.1,
                'feedback_max_offset': 1.5,

                # per i sweep
                'amp_min_rad': 0.3,
                'amp_max_rad': 1.5,
                'freq_min_hz': 0.5,   
                'freq_max_hz': 2.0,  
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
                'tail_min_rad': -1.5, 
                'tail_max_rad': 1.5,
                'profile_velocity': 200, #100
                'profile_acceleration':50,# 10
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

