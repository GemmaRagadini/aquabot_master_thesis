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
        self.declare_parameter('mode', 'amp_sweep')
        self.declare_parameter('tail_bias_rad', 0.0)
        self.declare_parameter('tail_amp_rad', 0.4)
        self.declare_parameter('tail_freq_hz', 0.5)
        self.declare_parameter('tail_min_rad', -1.5)
        self.declare_parameter('tail_max_rad', 1.5)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('log_rate_hz', 20.0)
        self.declare_parameter('log_dir', 'logs')

        # parametri per variazione frequenza/ampiezza
        self.declare_parameter('trial_duration_sec', 20.0)
        self.declare_parameter('freq_min_hz', 0.5)
        self.declare_parameter('freq_max_hz', 2.0)
        self.declare_parameter('amp_min_rad', 0.1)
        self.declare_parameter('amp_max_rad', 0.6)

        # collegamento sensori -> motore
        self.declare_parameter("feedback_enabled", False)
        self.declare_parameter("feedback_gain", 0.0)
        self.declare_parameter("feedback_alpha", 0.1)
        self.declare_parameter("feedback_max_offset", 0.5)

        self.trial_duration = float(self.get_parameter('trial_duration_sec').value)
        self.freq_min = float(self.get_parameter('freq_min_hz').value)
        self.freq_max = float(self.get_parameter('freq_max_hz').value)
        self.amp_min = float(self.get_parameter('amp_min_rad').value)
        self.amp_max = float(self.get_parameter('amp_max_rad').value)
        self.target_topic = self.get_parameter('target_topic').value
        self.sensor_topic = self.get_parameter('sensor_topic').value
        self.mode = self.get_parameter('mode').value
        self.bias = float(self.get_parameter('tail_bias_rad').value)
        self.amp = float(self.get_parameter('tail_amp_rad').value)
        self.freq = float(self.get_parameter('tail_freq_hz').value)
        self.tail_min = float(self.get_parameter('tail_min_rad').value)
        self.tail_max = float(self.get_parameter('tail_max_rad').value)
        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.log_rate = float(self.get_parameter('log_rate_hz').value)
        self.log_dir = self.get_parameter('log_dir').value
        self.feedback_enabled = bool(self.get_parameter("feedback_enabled").value)
        self.feedback_gain = float(self.get_parameter("feedback_gain").value)
        self.feedback_alpha = float(self.get_parameter("feedback_alpha").value)
        self.feedback_max_offset = float(self.get_parameter("feedback_max_offset").value)

        # pubblica il target della coda
        self.publisher = self.create_publisher(Float64, self.target_topic, 10)
        
        self.create_subscription(
            Float32MultiArray, self.sensor_topic, self.sensor_callback, 10
        )
        self.create_subscription(Float64, '/aquabot/dynamixel/present_position',
            self.position_callback, 10)
        self.create_subscription(Float64, '/aquabot/dynamixel/present_current',
            self.current_callback, 10)

        self.last_sensor = None
        self.last_sensor_time = None
        self.t0 = None
        self.latest_target = 0.0
        self.recording = False
        self.csv_file = None
        self.csv_writer = None

        self.srv_trial = self.create_service(SetBool, 'trial', self.trial_callback)
        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_step)
        self.log_timer = self.create_timer(1.0 / self.log_rate, self.log_step)

        self.phase_acc = 0.0
        self.last_control_time = None
        self.current_amp = self.amp
        self.current_freq = self.freq
        self.current_bias_offset = 0.0

        self.sensor_diff_offset = 0.0
        self.calibration_samples = []
        self.calibration_done = False
        self.calibration_n = 50  # numero di campioni per la calibrazione
        # per lettura dal motore
        self.present_position = 0.0
        self.present_current = 0.0

        # aggiornamento parametri a runtime senza riavviare il nodo
        self.add_on_set_parameters_callback(self.parameter_callback)


    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'mode':
                self.mode = p.value
                self.get_logger().info(f"mode -> {self.mode}")
            elif p.name == 'tail_freq_hz':
                self.freq = float(p.value)
                self.current_freq = self.freq
            elif p.name == 'tail_amp_rad':
                self.amp = float(p.value)
                self.current_amp = self.amp
            elif p.name == 'amp_min_rad':
                self.amp_min = float(p.value)
            elif p.name == 'amp_max_rad':
                self.amp_max = float(p.value)
            elif p.name == 'freq_min_hz':
                self.freq_min = float(p.value)
            elif p.name == 'freq_max_hz':
                self.freq_max = float(p.value)
            elif p.name == 'trial_duration_sec':
                self.trial_duration = float(p.value)
            elif p.name == 'tail_bias_rad':
                self.bias = float(p.value)
            elif p.name == 'feedback_enabled':
                self.feedback_enabled = bool(p.value)
            elif p.name == 'feedback_gain':
                self.feedback_gain = float(p.value)
        return SetParametersResult(successful=True)

    def sensor_callback(self, msg: Float32MultiArray):
        
        self.last_sensor = list(msg.data)
        self.last_sensor_time = self.get_clock().now()
        # calibrazione automatica all'avvio
        if not self.calibration_done:
            if len(self.last_sensor) >= 2:
                diff = float(self.last_sensor[0]) - float(self.last_sensor[1])
                self.calibration_samples.append(diff)

                if len(self.calibration_samples) >= self.calibration_n:
                    self.sensor_diff_offset = sum(self.calibration_samples) / len(self.calibration_samples)
                    self.calibration_done = True
                    self.get_logger().info(
                        f"Calibrazione completata: sensor_diff_offset={self.sensor_diff_offset:.2f} "
                        f"(media su {self.calibration_n} campioni)"
                    )

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

    def position_callback(self, msg: Float64):
        self.present_position = msg.data

    def current_callback(self, msg: Float64):
        self.present_current = msg.data

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
            "tail_bias_rad",         # bias fisso dal parametro
            "tail_bias_offset_rad",  # contributo del feedback
            "tail_amp_rad",
            "tail_freq_hz",
            "phase_rad",
            "cycle_idx",
            "present_position_rad",   
            "present_current_ma",     
            "sensor_len",
            "sensor_values"
        ])
        self.csv_file.flush()
        self.t0 = self.get_clock().now()
        self.last_control_time = self.t0
        self.phase_acc = 0.0
        self.current_bias_offset = 0.0  # reset offset ad ogni trial
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

    def control_step(self):
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now

        if self.last_control_time is None:
            self.last_control_time = now

        t_rel = (now - self.t0).nanoseconds * 1e-9
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now

        target = self.compute_target(t_rel, dt)
        self.latest_target = target

        msg = Float64()
        msg.data = float(target)
        self.publisher.publish(msg)

        # if int(t_rel * 4) != int((t_rel - dt) * 4):
        #     # self.get_logger().info(
        #     #     f"[control] t={t_rel:.2f}s  target={target:.3f} rad  "
        #     #     f"amp={self.current_amp:.3f} rad  freq={self.current_freq:.3f} Hz  "
        #     #     f"bias_offset={self.current_bias_offset:.4f} rad"
        #     # )

    def compute_target(self, t_rel: float, dt: float):

        if self.mode == 'std':
            freq_t = self.freq
            amp_t = self.amp
            self.current_amp = amp_t
            self.current_freq = freq_t
            self.phase_acc += 2.0 * math.pi * freq_t * dt
            bias_offset = self.compute_bias_offset()
            theta = self.bias + bias_offset + amp_t * math.sin(self.phase_acc)
            return clamp(theta, self.tail_min, self.tail_max)

        elif self.mode == 'freq_sweep':
            alpha = self.triangular_profile(t_rel, self.trial_duration)
            freq_t = self.freq_min + alpha * (self.freq_max - self.freq_min)
            amp_t = self.amp
            self.current_amp = amp_t
            self.current_freq = freq_t
            self.phase_acc += 2.0 * math.pi * freq_t * dt
            bias_offset = self.compute_bias_offset()
            theta = self.bias + bias_offset + amp_t * math.sin(self.phase_acc)
            return clamp(theta, self.tail_min, self.tail_max)

        elif self.mode == 'amp_sweep':
            alpha = self.triangular_profile(t_rel, self.trial_duration)
            amp_t = self.amp_min + alpha * (self.amp_max - self.amp_min)
            freq_t = self.freq
            self.current_amp = amp_t
            self.current_freq = freq_t
            self.phase_acc += 2.0 * math.pi * freq_t * dt
            bias_offset = self.compute_bias_offset()
            theta = self.bias + bias_offset + amp_t * math.sin(self.phase_acc)
            return clamp(theta, self.tail_min, self.tail_max)
        
        elif self.mode == 'combined_sweep':
            self.get_logger().info(f"combined_sweep")
            # amp e freq variano con profili triangolari a periodi incommensurabili
            # (rapporto aureo) => esplorazione scorrelata dello spazio (amp, freq)
            PHI = 1.6180339887
            alpha_amp  = self.triangular_profile(t_rel, self.trial_duration)
            alpha_freq = self.triangular_profile(t_rel, self.trial_duration / PHI)
            amp_t  = self.amp_min  + alpha_amp  * (self.amp_max  - self.amp_min)
            freq_t = self.freq_min + alpha_freq * (self.freq_max - self.freq_min)
            self.current_amp  = amp_t
            self.current_freq = freq_t
            self.phase_acc += 2.0 * math.pi * freq_t * dt
            bias_offset = self.compute_bias_offset()
            theta = self.bias + bias_offset + amp_t * math.sin(self.phase_acc)
            return clamp(theta, self.tail_min, self.tail_max)

        elif self.mode == '1to1':
            # la posizione target segue direttamente i sensori
            if self.last_sensor is None or len(self.last_sensor) < 2:
                return clamp(self.bias, self.tail_min, self.tail_max)
            
            sensor_diff = float(self.last_sensor[0]) - float(self.last_sensor[1])
            sensor_diff_calibrated = sensor_diff - self.sensor_diff_offset

            # conversione in radianti            
            theta = self.bias + self.feedback_gain * sensor_diff_calibrated
            
            self.current_amp = 0.0
            self.current_freq = 0.0
            return clamp(theta, self.tail_min, self.tail_max)
        
        # fallback
        bias_offset = self.compute_bias_offset()
        return clamp(self.bias + bias_offset, self.tail_min, self.tail_max)

    def triangular_profile(self, t_rel: float, duration: float) -> float:
        if duration <= 0.0:
            return 0.0
        tau = (t_rel % duration) / duration
        if tau < 0.5:
            return 2.0 * tau
        else:
            return 2.0 * (1.0 - tau)

    def compute_bias_offset(self):
        if not self.feedback_enabled:
            self.current_bias_offset = 0.0
            return 0.0
        if self.last_sensor is None or len(self.last_sensor) < 2:
            self.current_bias_offset = 0.0
            return 0.0

        sensor_diff = float(self.last_sensor[0]) - float(self.last_sensor[1])
        # sottrai l'offset a riposo per centrare il feedback a zero
        sensor_diff_calibrated = sensor_diff - self.sensor_diff_offset
        target_offset = self.feedback_gain * sensor_diff_calibrated

        # filtro passa-basso
        self.current_bias_offset = (
            (1.0 - self.feedback_alpha) * self.current_bias_offset
            + self.feedback_alpha * target_offset
        )
        # clamp di sicurezza
        self.current_bias_offset = clamp(
            self.current_bias_offset,
            -self.feedback_max_offset,
            self.feedback_max_offset
        )
        return self.current_bias_offset

    def log_step(self):
        if not self.recording or self.csv_writer is None:
            return

        now = self.get_clock().now()
        t_ros_sec = now.nanoseconds * 1e-9
        t_rel = (now - self.t0).nanoseconds * 1e-9 if self.t0 else 0.0
        phase = self.phase_acc % (2.0 * math.pi)
        cycle_idx = int(self.phase_acc / (2.0 * math.pi))

        if self.last_sensor is None:
            sensor_len = 0
            sensor_values = []
        else:
            sensor_len = len(self.last_sensor)
            sensor_values = self.last_sensor

        self.csv_writer.writerow([
            t_ros_sec,
            t_rel,
            float(self.latest_target),
            float(self.bias),                    # bias fisso
            float(self.current_bias_offset),     # offset feedback
            float(self.current_amp),
            float(self.current_freq),
            float(phase),
            cycle_idx,
            float(self.present_position),   
            float(self.present_current),    
            sensor_len,
            sensor_values
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
    pass


if __name__ == '__main__':
    main()