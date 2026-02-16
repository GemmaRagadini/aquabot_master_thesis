import math
import random
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# pubblica letture del sensore fake 
# genera un segnale nel tempo e poi lo pubblica
class FakeSensorNode(Node):
      def __init__(self): 
            super().__init__('fake_sensor_node')

            self.declare_parameter('sensor_topic', '/sensor_reading')
            self.declare_parameter('pub_rate_hz', 50.0)
            # segnale finto indipendente
            self.declare_parameter('signal_amp', 50.0)
            self.declare_parameter('signal_freq_hz', 1.0)
            self.declare_parameter('offset', 512.0)
            self.declare_parameter('noise_std', 3.0)

            self.sensor_topic = self.get_parameter('sensor_topic').value
            self.pub_rate = float(self.get_parameter('pub_rate_hz').value)

            self.amp = float(self.get_parameter('signal_amp').value)
            self.freq = float(self.get_parameter('signal_freq_hz').value)
            self.offset = float(self.get_parameter('offset').value)
            self.noise_std = float(self.get_parameter('noise_std').value)

            self.pub = self.create_publisher(Float32MultiArray, self.sensor_topic, 10)
            self.t0 = self.get_clock().now()

            self.timer = self.create_timer(1.0 / self.pub_rate, self.step)

      # segnale periodico , con due canali sfasati  + rumore
      def step(self):
            t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
            phase = 2.0 * math.pi * self.freq * t

            n0 = random.gauss(0.0, self.noise_std)
            n1 = random.gauss(0.0, self.noise_std)

            s0 = self.offset + self.amp * math.sin(phase) + n0
            s1 = self.offset + self.amp * math.cos(phase) + n1

            msg = Float32MultiArray()
            msg.data = [float(s0), float(s1)]
            self.pub.publish(msg)


def main(args=None):
      rclpy.init(args=args)
      node = FakeSensorNode()
      try:
            rclpy.spin(node)
      except KeyboardInterrupt:
            pass
      finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
      main()