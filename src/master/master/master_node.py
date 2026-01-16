import rclpy
from rclpy.node import Node

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.get_logger().info('master_node started.')

    def close(self):
        self.get_logger().info('master_node shutting down.')
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        rclpy.shutdown()
