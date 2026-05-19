from launch import LaunchDescription
from launch_ros.actions import Node

# Limiti meccanici misurati empiricamente con Dynamixel Wizard
# Tick 2299 => +0.385 rad   Tick 2975 => +1.422 rad
# Centro di oscillazione: (0.385 + 1.422) / 2 = 0.903 rad
# Semiampiezza massima:   (1.422 - 0.385) / 2 = 0.519 rad
TAIL_MIN_RAD  = 0.385
TAIL_MAX_RAD  = 1.422
TAIL_BIAS_RAD = 0.903
MAX_AMP_RAD   = 0.519


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
                'mode': 'turning_combined',
                'trial_duration_sec': 30.0,

                # centro reale 
                'tail_bias_rad': TAIL_BIAS_RAD,

                # ampiezza e freq default 
                'tail_amp_rad': 0.3,
                'tail_freq_hz': 0.5,

                # LIMITI 
                'tail_min_rad': TAIL_MIN_RAD,
                'tail_max_rad': TAIL_MAX_RAD,

                'control_rate_hz': 20.0,
                'log_rate_hz': 20.0,
                'log_dir': './logs',

                'feedback_enabled': False,
                'feedback_gain': 0.001,
                'feedback_alpha': 0.1,
                'feedback_max_offset': MAX_AMP_RAD,

                # sweep: amp entro la semiampiezza massima
                'amp_min_rad': 0.1,
                'amp_max_rad': MAX_AMP_RAD,
                'freq_min_hz': 0.5,
                'freq_max_hz': 1.0,

                # turning combined
                'turning_bias_amp_rad': 0.2,
                'turning_bias_freq_hz': 0.08,
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

                # limiti assoluti in radianti 
                'tail_min_rad': TAIL_MIN_RAD,
                'tail_max_rad': TAIL_MAX_RAD,

                # limiti assoluti in tick 
                'min_position_tick': 2299,
                'max_position_tick': 2975,

                'profile_velocity': 500,
                'profile_acceleration': 400,
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