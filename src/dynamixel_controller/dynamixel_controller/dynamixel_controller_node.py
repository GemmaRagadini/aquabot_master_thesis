#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_sdk import PortHandler, PacketHandler


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class DynamixelController(Node):
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY = 112
    ADDR_PRESENT_POSITION = 132
    ADDR_PRESENT_CURRENT = 126

    def __init__(self):
        super().__init__('dynamixel_controller_node')

        self.declare_parameter('target_topic', '/aquabot/dynamixel/target_position')
        self.declare_parameter('device_name', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('dxl_id', 1)
        self.declare_parameter('protocol_version', 2.0)

        self.declare_parameter('tail_min_rad', -0.7)
        self.declare_parameter('tail_max_rad', 0.7)

        self.declare_parameter('profile_velocity', 100)
        self.declare_parameter('profile_acceleration', 10)

        # legge i parametri ROS
        self.target_topic = self.get_parameter('target_topic').value
        self.device_name = self.get_parameter('device_name').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.dxl_id = int(self.get_parameter('dxl_id').value)
        self.protocol_version = float(self.get_parameter('protocol_version').value)
        self.tail_min_rad = float(self.get_parameter('tail_min_rad').value)
        self.tail_max_rad = float(self.get_parameter('tail_max_rad').value)
        self.profile_velocity = int(self.get_parameter('profile_velocity').value)
        self.profile_acceleration = int(self.get_parameter('profile_acceleration').value)

        # oggetti SDK per porta seriale e protocollo Dynamixel
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        # apre la porta seriale
        if not self.port_handler.openPort():
            raise RuntimeError(f'Impossibile aprire la porta {self.device_name}')

        # imposta il baudrate
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f'Impossibile impostare baudrate {self.baudrate}')

        # scrive Profile Acceleration nel registro 108
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            self.dxl_id,
            self.ADDR_PROFILE_ACCELERATION,
            self.profile_acceleration
        )

        if dxl_comm_result != 0:
            raise RuntimeError(
                f'Errore comunicazione profile acceleration: {dxl_comm_result}'
            )
        if dxl_error != 0:
            raise RuntimeError(
                f'Errore Dynamixel profile acceleration: {dxl_error}'
            )

        # scrive Profile Velocity nel registro 112
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            self.dxl_id,
            self.ADDR_PROFILE_VELOCITY,
            self.profile_velocity
        )

        if dxl_comm_result != 0:
            raise RuntimeError(
                f'Errore comunicazione profile velocity: {dxl_comm_result}'
            )
        if dxl_error != 0:
            raise RuntimeError(
                f'Errore Dynamixel profile velocity: {dxl_error}'
            )

        # abilita la coppia del motore
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            self.dxl_id,
            self.ADDR_TORQUE_ENABLE,
            self.TORQUE_ENABLE
        )

        if dxl_comm_result != 0:
            raise RuntimeError(f'Errore comunicazione torque enable: {dxl_comm_result}')
        if dxl_error != 0:
            raise RuntimeError(f'Errore Dynamixel torque enable: {dxl_error}')

        # subscriber: riceve il target dal master
        self.create_subscription(Float64, self.target_topic, self.on_target, 10)
        # publisher
        self.pub_position = self.create_publisher(Float64, '/aquabot/dynamixel/present_position', 10)
        self.pub_current = self.create_publisher(Float64, '/aquabot/dynamixel/present_current', 10)
        # timer di lettura
        self.read_timer = self.create_timer(1.0 / 20.0, self.read_motor_state)
        
    def read_motor_state(self):
        # leggi posizione
        pos_tick, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.dxl_id, self.ADDR_PRESENT_POSITION
        )
        if result == 0 and error == 0:
            pos_rad = (pos_tick - 2048.0) / 4096.0 * (2.0 * math.pi)
            msg = Float64()
            msg.data = pos_rad
            self.pub_position.publish(msg)

        # leggi corrente (valore signed a 16 bit)
        current_raw, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.dxl_id, self.ADDR_PRESENT_CURRENT
        )
        if result == 0 and error == 0:
            # converti da unsigned a signed
            if current_raw > 32767:
                current_raw -= 65536
            current_ma = current_raw * 2.69  # mA per unit (XM430)
            msg = Float64()
            msg.data = current_ma
            self.pub_current.publish(msg)

    # conversione radianti => tick
    def rad_to_tick(self, angle_rad: float) -> int:
        angle_rad = clamp(angle_rad, self.tail_min_rad, self.tail_max_rad)

        center_tick = 2048.0
        ticks_per_rad = 4096.0 / (2.0 * math.pi)

        tick = int(round(center_tick + angle_rad * ticks_per_rad))
        tick = int(clamp(tick, 0, 4095))
        return tick

    # arriva comando dal master
    def on_target(self, msg: Float64):
        target_rad = float(msg.data)
        goal_tick = self.rad_to_tick(target_rad)

        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            self.dxl_id,
            self.ADDR_GOAL_POSITION,
            goal_tick
        )

        if dxl_comm_result != 0:
            self.get_logger().error(f'Errore comunicazione write goal: {dxl_comm_result}')
            return

        if dxl_error != 0:
            self.get_logger().error(f'Errore dynamixel write goal: {dxl_error}')
            return

        # self.get_logger().info(
        #     f'target_rad={target_rad:.3f} -> goal_tick={goal_tick}'
        # )

    def destroy_node(self):
        try:
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                self.dxl_id,
                self.ADDR_TORQUE_ENABLE,
                self.TORQUE_DISABLE
            )
            self.port_handler.closePort()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()