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
                'traj': 'straight',
                'tail_bias_rad': 0.0,
                'tail_amp_rad': 0.4,
                'tail_freq_hz': 1.0,
                'tail_min_rad': -0.7,
                'tail_max_rad': 0.7,
                'control_rate_hz': 20.0,
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



# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import PathJoinSubstitution, Command

# def generate_launch_description():
    
#     bringup_pkg = FindPackageShare("aquabot_bringup")

#     xacro_file = PathJoinSubstitution([bringup_pkg, "urdf", "aquabot.urdf.xacro"])
#     controllers_file = PathJoinSubstitution([bringup_pkg, "config", "controllers.yaml"])

#     robot_description = {"robot_description": Command(["xacro", " ", xacro_file])}   
#     # return LaunchDescription([
#         # IncludeLaunchDescription(
#         #     PythonLaunchDescriptionSource(mock_control_launch)
#         # ),  
#         # Node(
#         #     package='arduino_reader',
#         #     executable='arduino_reader_node',
#         #     name='arduino_reader_node',
#         #     output='screen',
#         # ),
#         # Node(
#         #     package='master',
#         #     executable='master_node',
#         #     name='master_node',
#         #     output='screen',
#         #     parameters=[{
#         #         # ---- Topics ----
#         #         'sensor_topic': '/sensor_reading',
#         #         'target_topic': '/target_position',
#         #         # ---- Traiettoria pesce ----
#         #         'traj': 'straight',           # usa sinusoide simmetrica
#         #         'tail_bias_rad': 0.0,         # neutro
#         #         'tail_amp_rad': 0.4,          # ampiezza dx/sx
#         #         'tail_freq_hz': 1.0,          # Hz
#         #         'tail_min_rad': -0.7,         # clamp sicurezza
#         #         'tail_max_rad': 0.7,
#         #         # ---- Frequenze ----
#         #         'control_rate_hz': 50.0,      # pubblicazione target
#         #         'log_rate_hz': 20.0,          # scrittura CSV
#         #         # ---- Logging ----
#         #         'log_dir': './logs',
#         #     }]
#         # ),
#         # lettore sensori fake 
#         # Node( 
#         #     package= 'master',
#         #     executable = 'fake_sensor_node',
#         #     name = 'fake_sensor_node',
#         #     output = 'screen',
#         #     parameters = [{
#         #         'sensor_topic': '/sensor_reading',
#         #         'pub_rate_hz': 50.0,
#         #     }]
#         # ),
#         # Node(
#         #     package='dynamixel_controller',
#         #     executable='dynamixel_controller_node',
#         #     name='dynamixel_controller_node',
#         #     output='screen',
#         #     parameters=[{
#         #         'frequency_hz': 0.5,
#         #         'left_rad': 0.0,
#         #         'right_rad':3.14159,
#         #     }]
#         # ),
#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="screen",
#         parameters=[robot_description],
#     )

#     ros2_control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         output="screen",
#         parameters=[robot_description, controllers_file],
#     )

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
#         output="screen",
#     )

#     forward_position_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
#         output="screen",
#     )

#     return LaunchDescription([
#         robot_state_publisher,
#         ros2_control_node,
#         joint_state_broadcaster_spawner,
#         forward_position_controller_spawner,
#     ])
