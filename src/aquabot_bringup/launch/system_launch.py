from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

# Limiti meccanici misurati empiricamente con Dynamixel Wizard
# Conversione: rad = tick * 2*pi/4096 - pi
# Tick 1860 => -0.288 rad   Tick 2320 => +0.417 rad   Tick 2090 (centro) => +0.064 rad
# Semiampiezza massima: 230 tick = 0.353 rad

# DA NON CAMBIARE!! (devono coincidere con le costanti di master_node.py)
TAIL_MIN_RAD  = -0.288
TAIL_MAX_RAD  = 0.417
TAIL_BIAS_RAD = 0.064
MAX_AMP_RAD   = 0.353

TRIAL_DURATION = 30.0   # durata registrazione in secondi
STARTUP_DELAY  = 5.0    # attesa prima di avviare il trial


def generate_launch_description():
    master_node = Node(
        package='master',
        executable='master_node',
        name='master_node',
        output='screen',
        parameters=[{
            'sensor_topic': '/sensor_reading',
            'target_topic': '/aquabot/dynamixel/target_position',
            'mode': 'std',
            'trial_duration_sec': TRIAL_DURATION,

            # centro reale
            'tail_bias_rad': TAIL_BIAS_RAD,

            # ampiezza e freq default
            'tail_amp_rad': 0.35,
            'tail_freq_hz': 0.5,

            # LIMITI
            'tail_min_rad': TAIL_MIN_RAD,
            'tail_max_rad': TAIL_MAX_RAD,

            'control_rate_hz': 20.0,
            'log_rate_hz': 10.0,
            'log_dir': './logs',

            # feedback: SOLO diagnostica
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
            'turning_bias_amp_rad': 0.08,
            'turning_bias_freq_hz': 0.08,
        }]
    )

    dynamixel_node = Node(
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
            'min_position_tick': 1860,
            'max_position_tick': 2320,

            # ritorno al centro allo spegnimento
            'center_tick': 2090,
            'keep_torque_on_shutdown': False,

            'profile_velocity': 500,
            'profile_acceleration': 400,
        }]
    )

    arduino_node = Node(
        package='arduino_reader',
        executable='arduino_reader_node',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud_rate': 57600,
        }]
    )

    # Avvia la registrazione dopo STARTUP_DELAY secondi
    start_trial = TimerAction(
        period=STARTUP_DELAY,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/trial',
                    'std_srvs/srv/SetBool', '{data: true}'
                ],
                output='screen'
            )
        ]
    )

    # Ferma la registrazione dopo STARTUP_DELAY + TRIAL_DURATION secondi
    stop_trial = TimerAction(
        period=STARTUP_DELAY + TRIAL_DURATION,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/trial',
                    'std_srvs/srv/SetBool', '{data: false}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        master_node,
        dynamixel_node,
        arduino_node,
        start_trial,
        stop_trial,
    ])