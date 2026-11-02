# arduino_reader/arduino_reader/arduino_reader_node.py
#
# Reads "a0,a5\n" lines from the Arduino and publishes EVERY complete line
# on /sensor_reading as soon as it arrives (no averaging here).
# Timing and averaging are done by the logger (master_node), on its own clock,
# so the delay between command and sensor is the same in every recording.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time


class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('timeout', 1)
        self.declare_parameter('poll_rate', 200)   # Hz: must be well above the Arduino send rate

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value
        poll_rate = self.get_parameter('poll_rate').get_parameter_value().integer_value

        self.serial_conn = None
        self.buf = ""            # incomplete line carried over to the next read
        self.n_lines = 0
        self.n_bad = 0
        self.t_stats = time.time()

        self.publisher_ = self.create_publisher(Float32MultiArray, '/sensor_reading', 50)
        self.connect_serial()
        self.timer = self.create_timer(1.0 / poll_rate, self.read_data)
        self.get_logger().info(f'Node {self.get_name()} initialized (poll {poll_rate} Hz).')

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            start = time.time()
            while self.serial_conn.in_waiting == 0:
                if time.time() - start > 5:
                    self.get_logger().error("Timeout error: impossible to connect to arduino.")
                    self.serial_conn = None
                    return
                time.sleep(0.01)
            time.sleep(0.15)
            # discard everything accumulated so far (old data and the first partial line)
            self.serial_conn.reset_input_buffer()
            self.buf = ""
            self.get_logger().info("Connection established, input buffer flushed.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error: {e}")
            self.serial_conn = None

    def read_data(self):
        if not self.serial_conn:
            return
        n = self.serial_conn.in_waiting
        if n == 0:
            return
        self.buf += self.serial_conn.read(n).decode(errors='ignore')
        *lines, self.buf = self.buf.split('\n')      # last element = incomplete line
        for line in lines:
            parts = line.strip().split(',')
            if len(parts) != 2:
                self.n_bad += 1
                continue
            try:
                a0, a5 = int(parts[0]), int(parts[1])
            except ValueError:
                self.n_bad += 1
                continue
            msg = Float32MultiArray()
            msg.data = [float(a0), float(a5)]
            self.publisher_.publish(msg)
            self.n_lines += 1
        # once every 10 s: real Arduino rate and malformed lines
        now = time.time()
        if now - self.t_stats >= 10.0:
            self.get_logger().info(f"Arduino: {self.n_lines / (now - self.t_stats):.1f} lines/s, "
                                   f"{self.n_bad} malformed in the last {now - self.t_stats:.0f} s")
            self.n_lines, self.n_bad, self.t_stats = 0, 0, now

    def close(self):
        if self.serial_conn:
            self.serial_conn.close()
            self.get_logger().info("Connection closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()