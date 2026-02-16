from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    bringup_pkg = FindPackageShare('aquabot_bringup')

    return LaunchDescription([  
        # Node(
        #     package='arduino_reader',
        #     executable='arduino_reader_node',
        #     name='arduino_reader_node',
        #     output='screen',
        # ),
        Node(
            package='master',
            executable='master_node',
            name='master_node',
            output='screen',
            parameters=[{
                # ---- Topics ----
                'sensor_topic': '/sensor_reading',
                'target_topic': '/target_position',
                # ---- Traiettoria pesce ----
                'traj': 'straight',           # usa sinusoide simmetrica
                'tail_bias_rad': 0.0,         # neutro
                'tail_amp_rad': 0.4,          # ampiezza dx/sx
                'tail_freq_hz': 1.0,          # Hz
                'tail_min_rad': -0.7,         # clamp sicurezza
                'tail_max_rad': 0.7,
                # ---- Frequenze ----
                'control_rate_hz': 50.0,      # pubblicazione target
                'log_rate_hz': 20.0,          # scrittura CSV
                # ---- Logging ----
                'log_dir': './logs',
            }]
        ),
        # lettore sensori fake 
        Node( 
            package= 'master',
            executable = 'fake_sensor_node',
            name = 'fake_sensor_node',
            output = 'screen',
            parameters = [{
                'sensor_topic': '/sensor_reading',
                'pub_rate_hz': 50.0,
            }]
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
            }]
        ),
    ])
