import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.get_logger().info('master_node started.')

        self.last_sensor = None
        self.prev_sensor = None

        self.create_subscription(
            Float32MultiArray,
            'sensor_reading',
            self.on_sensor,
            10
        )
    
        # --- PUB: target per dynamixel_controller ---
        self.target_pub = self.create_publisher(
            Float64,
            '/aquabot/dynamixel/target_position',
            10
        )

        self._go_right = True  # toggle state
        self.left_rad = 0.0
        self.right_rad = math.pi

        self.get_logger().info('master_node started (toggle on each sensor message).')



    def on_sensor(self, msg: Float32MultiArray):

        if msg.data is None or len(msg.data) == 0:
            return
        
        # è ok se il messaggio contiene almeno un valore
        target = self.right_rad if self._go_right else self.left_rad
        self._go_right = not self._go_right

        out = Float64()
        out.data = float(target)
        self.target_pub.publish(out)

        self.get_logger().info(f'Recv sensor_reading -> toggling target to {target:.3f} rad')


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
