from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

      pkg = FindPackageShare('aquabot_bringup')

      xacro_file = PathJoinSubstitution([pkg, 'urdf', 'mock_dynamixel.urdf.xacro'])
      controllers_file = PathJoinSubstitution([pkg, 'config', 'controllers.yaml'])

      robot_description = {'robot_description': Command(['xacro', ' ', xacro_file])}
      
      # 1) Pubblica robot_description su topic (quello che ros2_control_node sta aspettando)
      rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
      )

      # 2) Avvia ros2_control_node (controller_manager)
      control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers_file],
            output='screen',
      )

      # chiama /controller_manager/load_controller, /configure_controller e /activate_controller

      # joint_state_broadcaster egge le state_interface dei joint e pubblica il topic /joint_states
      jsb_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
      )
      # forward_position_controller iceve comandi su: /forward_position_controller/commands e li inoltra all’hardware (mock ora, Dynamixel poi)
      fwd_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
            output='screen'
      )

      return LaunchDescription([rsp, control_node, jsb_spawner, fwd_spawner])
