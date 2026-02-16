#!/usr/bin/env python3
import math
import csv
import os
import rclpy
from datetime import datetime
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64, Float32MultiArray
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool


def clamp(x:float, lo:float, hi:float):
    if x > hi: 
        return hi
    elif x < lo: 
        return lo
    return x

# pubblica target per la coda al Dynamixel controller, sottoscrive i sensor readings e scrive CSV
class MasterNode(Node):
 
    def __init__(self):
        
        super().__init__('master_node')
        self.get_logger().info('master_node started.')

        # topics 
        self.declare_parameter('target_topic', '/acquabot/dynamixel/target_position')
        self.declare_parameter('sensor_topic', '/sensor_reading') 

        # parametri traiettoria dritta 
        self.declare_parameter('traj', 'straight') 
        self.declare_parameter('tail_bias_rad', 0.0) # centro neutro della coda
        self.declare_parameter('tail_amp_rad', 0.4) # di quanto oscilla
        self.declare_parameter('tail_freq_hz', 1.0) # frequenza oscillazioni 
        self.declare_parameter('tail_min_rad', -0.7)
        self.declare_parameter('tail_max_rad', 0.7)
        
        
        # lettura a intervalli timer regolari
        self.declare_parameter('control_rate_hz', 50.0) # freq pubblicazione target 
        self.declare_parameter('log_rate_hz', 20.0) # freq scrittura su csv
        self.declare_parameter('log_dir', 'logs') # cartella

        # parametri
        self.target_topic= self.get_parameter('target_topic').value
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

        self.publisher = self.create_publisher(Float64, self.target_topic, 10) # 10 è la queue
        self.subscription = self.create_subscription(Float32MultiArray, self.sensor_topic, self.sensor_callback, 10)

        self.last_sensor = None # ultimo vettore sensore ricevuto
        self.last_sensor_time = None # quando è arrivato
        self.t0 = None # riferimento per t_rel
        self.latest_target = 0.0  # ultimo comando pubblicato
        self.recording = False 
        self.csv_file = None 
        self.csv_writer = None

         # se trial è true = > inizi un nuovo csv e memorizzazione, altirmenti stop e chiudi
        self.srv_trial = self.create_service(SetBool, 'trial', self.trial_callback)

        self.control_timer = self.create_timer(1.0/ self.control_rate, self.control_step) 
        self.log_timer = self.create_timer(1.0/self.log_rate, self.log_step)
        self.get_logger().info( f"TrajectoryNode ready. target_topic={self.target_topic} sensor_topic={self.sensor_topic} "f"traj={self.traj} bias={self.bias} amp={self.amp} freq={self.freq}Hz")

    
    # quando arriva una lettura del sensore
    def sensor_callback(self, msg: Float32MultiArray):
        self.last_sensor = list(msg.data)
        self.last_sensor_time = self.get_clock().now()

    # servizio start stop
    def trial_callback(self, request: SetBool.Request, response: SetBool.Response): 
        if request.data: 
            try:
                self.start_new_trial()
                response.success = True
                response.message= "Trial started: recording ON"
            except Exception as e:
                response.success = False
                response.message = f"Failed to start trial: {e}"
        else:
            self.stop_trial()
            response.success = True
            response.message = "Trial stopped: recording OFF"
        return response
    
    # crea csv e azzera tempo relativo
    def start_new_trial(self): 
        self.stop_trial()
        os.makedirs(self.log_dir, exist_ok = True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.log_dir, f"trial_{stamp}.csv")
        self.csv_file = os.path.join(self.log_dir, f"trial_{stamp}.csv")
        self.csv_file = open(filename,'w', newline='')
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

        #riporta coda a neutro
        msg = Float64()
        msg.data = float(clamp(self.bias, self.tail_min, self.tail_max))
        self.publisher.publish(msg)
    
    # genera e pubblica un comando per la coda
    def control_step(self):
        now = self.get_clock().now()
        if self.t0 is None: 
            self.t0 = now
        t_rel = (now - self.t0).nanoseconds * 1e-9 
        target = self.compute_target(t_rel)
        self.latest_target = target 
        msg = Float64() 
        msg.data = float(target)
        self.publisher.publish(msg)
    
    # calcola il comando => per ora solo oscillazione simmetrica intorno a bias
    def compute_target(self, t_rel:float):
        if self.traj == 'straight': 
            phase = 2.0 * math.pi * self.freq * t_rel 
            theta = self.bias + self.amp * math.sin(phase)
            return clamp(theta, self.tail_min, self.tail_max)
        return clamp(self.bias, self.tail_min, self.tail_max)

    # salva una riga csv 
    def log_step(self): 
        if not self.recording or self.csv_writer is None: 
            return 
        if self.last_sensor is None: 
            return
        now = self.get_clock().now()
        t_ros_sec = now.nanoseconds * 1e-9
        t_rel = (now - self.t0).nanoseconds *1e-9 if self.t0 else 0.0 
        phase = (2.0 * math.pi * self.freq * t_rel) % (2.0 * math.pi)
        cycle_idx = int(math.floor(self.freq * t_rel)) if self.freq > 0.0 else 0
        self.csv_writer.writerow([
            t_ros_sec,
            t_rel,
            float(self.latest_target),
            float(self.bias),
            float(self.amp),
            float(self.freq),
            float(phase),
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