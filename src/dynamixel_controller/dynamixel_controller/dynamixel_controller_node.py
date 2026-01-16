import rclpy
from rclpy.node import Node


class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller_node')
        self.get_logger().info('dynamixel_controller_node started.')

    def close(self):
        self.get_logger().info('dynamixel_controller_node shutting down.')
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        rclpy.shutdown()
