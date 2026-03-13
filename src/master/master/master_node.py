#!/usr/bin/env python3
import math
import csv
import os
import rclpy
from datetime import datetime
from rclpy.node import Node
from std_msgs.msg import Float64, Float32MultiArray
from std_srvs.srv import SetBool


def clamp(x: float, lo: float, hi: float):
    if x > hi:
        return hi
    elif x < lo:
        return lo
    return x


class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.get_logger().info('master_node started.')

        # PARAMETRI DELLA TRAIETTORIA
        self.declare_parameter('target_topic', '/aquabot/dynamixel/target_position')
        self.declare_parameter('sensor_topic', '/sensor_reading')

        self.declare_parameter('traj', 'std')
        # posizione centrale della coda
        self.declare_parameter('tail_bias_rad', 0.0)
        # ampiezza oscillazione 
        self.declare_parameter('tail_amp_rad', 0.4)
        # frequenza oscillazione
        self.declare_parameter('tail_freq_hz', 1.0)
        # angoli limiti
        self.declare_parameter('tail_min_rad', -0.7)
        self.declare_parameter('tail_max_rad', 0.7)
        # frequenza del controller
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('log_rate_hz', 20.0)
        self.declare_parameter('log_dir', 'logs')
        # parametri per variazione frequenza/ampiezza
        self.declare_parameter('trial_duration_sec',  20.0)
        self.declare_parameter('freq_min_hz', 0.5)
        self.declare_parameter('freq_max_hz', 2.0)
        self.declare_parameter('amp_min_rad', 0.1)
        self.declare_parameter('amp_max_rad', 0.6)

        self.trial_duration = float(self.get_parameter('trial_duration_sec').value)
        self.freq_min= float(self.get_parameter('freq_min_hz').value)
        self.freq_max = float(self.get_parameter('freq_max_hz').value)
        self.amp_min = float(self.get_parameter('amp_min_rad').value)
        self.amp_max = float(self.get_parameter('amp_max_rad').value)
        self.target_topic = self.get_parameter('target_topic').value
        self.sensor_topic = self.get_parameter('sensor_topic').value
        self.traj = self.get_parameter('traj').value
        self.bias = float(self.get_parameter('tail_bias_rad').value)
        self.amp = float(self.get_parameter('tail_amp_rad').value)
        self.freq = float(self.get_parameter('tail_freq_hz').value)
        self.tail_min = float(self.get_parameter('tail_min_rad').value)
        self.tail_max = float(self.get_parameter('tail_max_rad').value)
        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.log_rate = float(self.get_parameter('log_rate_hz').value)
        self.log_dir = self.get_parameter('log_dir').value
        
        # pubblica il target della coda
        self.publisher = self.create_publisher(Float64, self.target_topic, 10)
        self.subscription = self.create_subscription(
            Float32MultiArray, self.sensor_topic, self.sensor_callback, 10
        )

        self.last_sensor = None
        self.last_sensor_time = None
        # tempo iniziale
        self.t0 = None
        self.latest_target = 0.0
        self.recording = False
        self.csv_file = None
        self.csv_writer = None

        self.srv_trial = self.create_service(SetBool, 'trial', self.trial_callback)
        # timer che esegue control step
        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_step)
        self.log_timer = self.create_timer(1.0 / self.log_rate, self.log_step)

        self.phase_acc = 0.0 
        self.last_control_time = None
        self.current_amp = self.amp
        self.current_freq = self.freq
        
        self.get_logger().info(
            f"Master ready. target_topic={self.target_topic} sensor_topic={self.sensor_topic} "
            f"traj={self.traj} bias={self.bias} amp={self.amp} freq={self.freq}Hz"
        )
    # riceve dati sensori
    def sensor_callback(self, msg: Float32MultiArray):
        self.last_sensor = list(msg.data)
        self.last_sensor_time = self.get_clock().now()
        self.get_logger().info(f"Sensors: {self.last_sensor}")

    # start/stop registrazione
    def trial_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            try:
                self.start_new_trial()
                response.success = True
                response.message = "Trial started: recording ON"
            except Exception as e:
                response.success = False
                response.message = f"Failed to start trial: {e}"
        else:
            self.stop_trial()
            response.success = True
            response.message = "Trial stopped: recording OFF"
        return response

    def start_new_trial(self):
        self.stop_trial()
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.log_dir, f"trial_{stamp}.csv")
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "t_ros_sec",
            "t_rel_sec",
            "tail_target_rad",
            "tail_bias_rad",
            "tail_amp_rad",
            "tail_freq_hz",
            "phase_rad",
            "cycle_idx",
            "sensor_len",
            "sensor_values"
        ])
        self.csv_file.flush()
        self.t0 = self.get_clock().now()
        self.recording = True
        self.get_logger().info(f"Started trial -> {filename}")

    def stop_trial(self):
        self.recording = False
        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass
        self.csv_file = None
        self.csv_writer = None
        self.last_control_time = None
        self.phase_acc = 0.0

        msg = Float64()
        msg.data = float(clamp(self.bias, self.tail_min, self.tail_max))
        self.publisher.publish(msg)

    # genera il comando per la coda => funzione chiamata dal timer
    def control_step(self):
        now = self.get_clock().now()
        if self.t0 is None: 
            self.t0 = now
        if self.last_control_time is None: 
            self.last_control_time = now

        t_rel = (now - self.t0).nanoseconds * 1e-9
        dt = (now - self.last_control_time).nanoseconds *1e-9 
        self.last_control_time = now

        target = self.compute_target(t_rel, dt)
        self.latest_target = target 

        msg = Float64()
        msg.data = float(target)
        self.publisher.publish(msg)

     
    # calcola l'angolo della coda 
    def compute_target(self, t_rel: float, dt:float):
        
        # traiettoria standard 
        if self.traj == 'std':
            freq_t = self.freq
            amp_t = self.amp
            self.current_amp = amp_t
            self.current_freq = freq_t
            self.phase_acc += 2.0 * math.pi * freq_t * dt
            theta = self.bias + amp_t * math.sin(self.phase_acc)
            return clamp(theta, self.tail_min, self.tail_max)
        # sweep di frequanza con ampiezza costante
        elif self.traj == 'freq_sweep':
            alpha = self.triangular_profile(t_rel, self.trial_duration)
            freq_t = self.freq_min + alpha * (self.freq_max - self.freq_min)
            amp_t = self.amp
            self.current_amp = amp_t
            self.current_freq = freq_t
            self.phase_acc += 2.0 * math.pi * freq_t * dt
            theta = self.bias + amp_t * math.sin(self.phase_acc)
            return clamp(theta, self.tail_min, self.tail_max)
        
        # sweep di ampiezza con frequenza costante
        elif self.traj == 'amp_sweep': 
            alpha = self.triangular_profile(t_rel, self.trial_duration)
            amp_t = self.amp_min + alpha * (self.amp_max - self.amp_min)
            freq_t = self.freq
            self.current_amp = amp_t
            self.current_freq = freq_t
            self.phase_acc += 2.0 * math.pi * freq_t * dt
            theta = self.bias +  amp_t * math.sin(self.phase_acc)
            return clamp(theta, self.tail_min, self.tail_max)
        
        #fallback 
        return clamp(self.bias, self.tail_min,self.tail_max)



    # parte dal minimo , arriva al massimo, torna al minimo 
    def triangular_profile(self, t_rel: float, duration: float) -> float: 
        if duration <= 0.0: 
            return 0.0
        
        tau = (t_rel % duration) /duration # tra 0 e 1  
        if tau < 0.5: 
            return 2.0 * tau # salita 0->1 
        else: 
            return 2.0 * (1.0 -tau) # discesa 1->0

    def start_new_trial(self): 
        self.stop_trial()
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.log_dir, f"trial_{stamp}.csv")
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "t_ros_sec",
            "t_rel_sec",
            "tail_target_rad",
            "tail_bias_rad",
            "tail_amp_rad"
            "tail_freq_hz", 
            "phase_rad", 
            "cycle_idx",
            "sensor_len",
            "sensor_values"
        ])
        self.csv_file.flush()

        self.t0 = self.get_clock().now() 
        self.last_control_time = self.t0
        self.phase_acc = 0.0 
        self.recording = True 
        self.get_logger().info(f"Started trial -> {filename}")


    # salva i dati sensori
    def log_step(self):
        if not self.recording or self.csv_writer is None:
            return
        if self.last_sensor is None:
            return

        now = self.get_clock().now()
        t_ros_sec = now.nanoseconds * 1e-9
        t_rel = (now - self.t0).nanoseconds * 1e-9 if self.t0 else 0.0
        phase = (2.0 * math.pi * self.freq * t_rel) % (2.0 * math.pi)
        cycle_idx = int(self.phase_acc / (2.0 * math.pi))
        
        self.csv_writer.writerow([
            t_ros_sec,
            t_rel,
            float(self.latest_target),
            float(self.bias),
            float(self.current_amp),
            float(self.current_freq),
            float(self.phase_acc % (2.0 * math.pi)),
            cycle_idx,
            len(self.last_sensor),
            self.last_sensor
        ])
        self.csv_file.flush()


    def destroy_node(self):
        self.stop_trial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()