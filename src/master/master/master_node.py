import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.get_logger().info('master_node started.')

        self.last_sensor = None

        self.create_subscription(
            Float32MultiArray,
            '/sensor_reading',
            self.on_sensor,
            10
        )
    
        # --- PUB: target per dynamixel_controller ---
        self.target_pub = self.create_publisher(
            Float64,
            '/aquabot/dynamixel/target_position',
            10
        )

        # PARAMETRI OSCILLAZIONE 
        self.declare_parameter('frequency_hz', 0.5)  # inversioni al secondo
        self.declare_parameter('left_rad', 0.0)
        self.declare_parameter('right_rad', math.pi)

        self.frequency_hz = float(self.get_parameter('frequency_hz').value)
        self.left_rad = float(self.get_parameter('left_rad').value)
        self.right_rad = float(self.get_parameter('right_rad').value)

        self._go_right = True
        period = 1.0 / max(self.frequency_hz, 1e-6)
        self.timer = self.create_timer(period, self._on_timer)

        self.print_timer = self.create_timer(0.5, self.print_status)



    def on_sensor(self, msg: Float32MultiArray):
        self.last_sensor = list(msg.data)  # copia semplice

    def _on_timer(self):
        target = self.right_rad if self._go_right else self.left_rad
        self._go_right = not self._go_right

        out = Float64()
        out.data = float(target)
        self.target_pub.publish(out)

    def print_status(self):
        s = 'N/A' if self.last_sensor is None else str(self.last_sensor)
        self.get_logger().info(f'sensor_reading={s}')


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
