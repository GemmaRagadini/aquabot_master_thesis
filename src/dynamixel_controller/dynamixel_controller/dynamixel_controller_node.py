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

    def __init__(self):
        super().__init__('dynamixel_controller_node')

        self.declare_parameter('target_topic', '/aquabot/dynamixel/target_position')
        self.declare_parameter('device_name', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('dxl_id', 1)
        self.declare_parameter('protocol_version', 2.0)

        self.declare_parameter('tail_min_rad', -0.7)
        self.declare_parameter('tail_max_rad', 0.7)

        self.target_topic = self.get_parameter('target_topic').value
        self.device_name = self.get_parameter('device_name').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.dxl_id = int(self.get_parameter('dxl_id').value)
        self.protocol_version = float(self.get_parameter('protocol_version').value)
        self.tail_min_rad = float(self.get_parameter('tail_min_rad').value)
        self.tail_max_rad = float(self.get_parameter('tail_max_rad').value)

        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        if not self.port_handler.openPort():
            raise RuntimeError(f'Impossibile aprire la porta {self.device_name}')

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f'Impossibile impostare baudrate {self.baudrate}')

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            self.dxl_id,
            self.ADDR_TORQUE_ENABLE,
            self.TORQUE_ENABLE
        )

        if dxl_comm_result != 0:
            raise RuntimeError(f'Errore comunicazione torque enable: {dxl_comm_result}')
        if dxl_error != 0:
            raise RuntimeError(f'Errore dynamixel torque enable: {dxl_error}')

        self.create_subscription(Float64, self.target_topic, self.on_target, 10)

        self.get_logger().info(
            f'Dynamixel controller ready. topic={self.target_topic}, '
            f'port={self.device_name}, baudrate={self.baudrate}, id={self.dxl_id}'
        )
    
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

        self.get_logger().info(
            f'target_rad={target_rad:.3f} -> goal_tick={goal_tick}'
        )

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