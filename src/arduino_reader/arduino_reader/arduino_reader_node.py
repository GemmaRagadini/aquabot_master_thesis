# arduino_reader/arduino_reader/arduino_reader_node.py

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial
import time

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('timeout', 1)
        self.declare_parameter('rate', 10)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value

        rate = self.get_parameter('rate').get_parameter_value().integer_value
        self.serial_conn = None

        self.publisher_ = self.create_publisher(Float32MultiArray, 'sensor_reading', 10)

        # connect to serial
        self.connect_serial()
        # create a timer to read data
        self.timer = self.create_timer(1.0 / rate, self.read_data)
        # inizialization finished
        self.get_logger().info(f'Node {self.get_name()} initialized.')

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            _start = time.time()
            while True:
                if self.serial_conn.in_waiting > 0:
                    self.get_logger().info("Connection established.")
                    time.sleep(0.15)
                    break
                if time.time() - _start > 5:
                    self.get_logger().error("Timeout error: impossible to connect to arduino.")
                    return
                
        except serial.SerialException as e:
            self.get_logger().error(f"Error: {e}")
        
    def read_data(self):
        if not self.serial_conn:
            # self.get_logger().warn("Serial connection not established.")
            return None, None, 0

        # Read all data in the buffer (non-blocking)
        raw_data = self.serial_conn.read(self.serial_conn.in_waiting).decode(errors='ignore')
        lines = raw_data.strip().split('\n')
        values = []
        for line in lines:
            try:
                a0, a5 = [int(x) for x in line.strip().split(',')]
                values.append((a0, a5))
            except:
                continue # skip malformed lines
        
        if values:
            means = np.mean(np.array(values), axis=0)
            # Create a message
            msg = Float32MultiArray()
            # msg.data = [means[0], means[1], len(values)]
            msg.data = [means[0], means[1]]
            # Publish the message
            self.publisher_.publish(msg)
            # self.get_logger().info(f'Sensors values: {msg.data}')
        else:
            self.get_logger().warn("No valid data received.")

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
        pass
    finally:
        node.close()
        rclpy.shutdown()

