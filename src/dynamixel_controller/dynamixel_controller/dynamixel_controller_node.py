import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64

# per adesso legge soltanto i valori della posizione target 
class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller_node')

        # Subscriber dal master
        self.create_subscription(
            Float64,
            '/aquabot/dynamixel/target_position',
            self.on_target,
            10
        )

    def on_target(self, msg: Float64):
        
        target = msg.data
        # Saturazione sicurezza [0, pi]
        if target < 0.0:
            target = 0.0
        if target > 3.14159:
            target = 3.14159

        cmd = Float64MultiArray()
        cmd.data = [target]


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
