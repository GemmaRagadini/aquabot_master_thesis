#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_sdk import PortHandler, PacketHandler


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class DynamixelController(Node):
    ADDR_OPERATING_MODE       = 11   # EEPROM
    ADDR_MIN_POSITION_LIMIT   = 52   # EEPROM
    ADDR_MAX_POSITION_LIMIT   = 48   # EEPROM
    ADDR_TORQUE_ENABLE        = 64
    ADDR_GOAL_POSITION        = 116
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY     = 112
    ADDR_PRESENT_POSITION     = 132
    ADDR_PRESENT_CURRENT      = 126

    TORQUE_ENABLE           = 1
    TORQUE_DISABLE          = 0
    OPERATING_MODE_POSITION = 3   # Position Control 

    # Limiti
    # Tick 1860 => -0.288 rad   Tick 2320 => +0.417 rad   Tick 2090 (centro) => +0.064 rad
    DEFAULT_TAIL_MIN_RAD      = -0.288
    DEFAULT_TAIL_MAX_RAD      = 0.417
    DEFAULT_MIN_POSITION_TICK = 1860
    DEFAULT_MAX_POSITION_TICK = 2320
    DEFAULT_CENTER_TICK       = 2090

    def __init__(self):
        super().__init__('dynamixel_controller_node')

        self.declare_parameter('target_topic', '/aquabot/dynamixel/target_position')
        self.declare_parameter('device_name', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('dxl_id', 1)
        self.declare_parameter('protocol_version', 2.0)

        self.declare_parameter('tail_min_rad', self.DEFAULT_TAIL_MIN_RAD)
        self.declare_parameter('tail_max_rad', self.DEFAULT_TAIL_MAX_RAD)

        self.declare_parameter('min_position_tick', self.DEFAULT_MIN_POSITION_TICK)
        self.declare_parameter('max_position_tick', self.DEFAULT_MAX_POSITION_TICK)

        self.declare_parameter('profile_velocity', 100)
        self.declare_parameter('profile_acceleration', 10)

        # ritorno al centro allo spegnimento
        self.declare_parameter('center_tick', self.DEFAULT_CENTER_TICK)
        self.declare_parameter('shutdown_center_timeout_sec', 3.0)  # attesa massima
        self.declare_parameter('shutdown_center_tolerance_tick', 10)  # "arrivato" se entro +/- tick
        # False = dopo il centraggio la coppia viene spenta (coda libera, come prima)
        # True  = la coppia resta attiva e il motore tiene il centro finche' alimentato
        self.declare_parameter('keep_torque_on_shutdown', False)

        # legge i parametri ROS
        self.target_topic         = self.get_parameter('target_topic').value
        self.device_name          = self.get_parameter('device_name').value
        self.baudrate             = int(self.get_parameter('baudrate').value)
        self.dxl_id               = int(self.get_parameter('dxl_id').value)
        self.protocol_version     = float(self.get_parameter('protocol_version').value)
        self.tail_min_rad         = float(self.get_parameter('tail_min_rad').value)
        self.tail_max_rad         = float(self.get_parameter('tail_max_rad').value)
        self.min_position_tick    = int(self.get_parameter('min_position_tick').value)
        self.max_position_tick    = int(self.get_parameter('max_position_tick').value)
        self.profile_velocity     = int(self.get_parameter('profile_velocity').value)
        self.profile_acceleration = int(self.get_parameter('profile_acceleration').value)
        self.center_tick          = int(clamp(int(self.get_parameter('center_tick').value),
                                              self.min_position_tick, self.max_position_tick))
        self.shutdown_timeout     = float(self.get_parameter('shutdown_center_timeout_sec').value)
        self.shutdown_tolerance   = int(self.get_parameter('shutdown_center_tolerance_tick').value)
        self.keep_torque_on_shutdown = bool(self.get_parameter('keep_torque_on_shutdown').value)

        
        self.port_handler   = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        if not self.port_handler.openPort():
            raise RuntimeError(f'Impossibile aprire la porta {self.device_name}')

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f'Impossibile impostare baudrate {self.baudrate}')

        # Disabilita torque prima di scrivere in EEPROM 
        self._write1(self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE,
                     "torque disable (pre-EEPROM)")

        # Imposta position control
        self._write1(self.ADDR_OPERATING_MODE, self.OPERATING_MODE_POSITION,
                     "operating mode = Position Control (3)")
        self.get_logger().info('Operating mode impostato: Position Control (mode 3)')

        #  Min/Max Position Limit 
        self._write4(self.ADDR_MAX_POSITION_LIMIT, self.max_position_tick,
                     f"max_position_limit = {self.max_position_tick}")
        self._write4(self.ADDR_MIN_POSITION_LIMIT, self.min_position_tick,
                     f"min_position_limit = {self.min_position_tick}")
        self.get_logger().info(
            f'Limiti EEPROM scritti: min={self.min_position_tick} ({self.tail_min_rad:.3f} rad), '
            f'max={self.max_position_tick} ({self.tail_max_rad:.3f} rad)'
        )

        # ── 4. Profile Acceleration e Velocity ───────────────────────────────
        self._write4(self.ADDR_PROFILE_ACCELERATION, self.profile_acceleration,
                     "profile acceleration")
        self._write4(self.ADDR_PROFILE_VELOCITY, self.profile_velocity,
                     "profile velocity")

        # ── 5. Abilita torque ─────────────────────────────────────────────────
        self._write1(self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE,
                     "torque enable")
        self.get_logger().info('Torque abilitato. Nodo pronto.')

        # subscriber e publisher
        self.create_subscription(Float64, self.target_topic, self.on_target, 10)
        self.pub_position = self.create_publisher(Float64, '/aquabot/dynamixel/present_position', 10)
        self.pub_current  = self.create_publisher(Float64, '/aquabot/dynamixel/present_current', 10)
        self.read_timer   = self.create_timer(1.0 / 20.0, self.read_motor_state)

    # ── Helper di scrittura ──────────────────────────────────────────────────

    def _write1(self, addr, value, label):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, addr, value)
        self._check(result, error, label)

    def _write4(self, addr, value, label):
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, addr, value)
        self._check(result, error, label)

    def _check(self, result, error, label):
        if result != 0:
            raise RuntimeError(f'Errore comunicazione [{label}]: {result}')
        if error != 0:
            raise RuntimeError(f'Errore Dynamixel [{label}]: {error}')

    # ── Lettura stato motore ─────────────────────────────────────────────────

    def read_motor_state(self):
        pos_tick, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.dxl_id, self.ADDR_PRESENT_POSITION)
        if result == 0 and error == 0:
            pos_rad = (pos_tick - 2048.0) / 4096.0 * (2.0 * math.pi)
            msg = Float64()
            msg.data = pos_rad
            self.pub_position.publish(msg)

        current_raw, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.dxl_id, self.ADDR_PRESENT_CURRENT)
        if result == 0 and error == 0:
            if current_raw > 32767:
                current_raw -= 65536
            current_ma = current_raw * 2.69
            msg = Float64()
            msg.data = current_ma
            self.pub_current.publish(msg)

    # ── Conversione radianti => tick ─────────────────────────────────────────

    def rad_to_tick(self, angle_rad: float) -> int:
        # Clamp software (secondo livello di protezione, il firmware fa il primo)
        angle_rad = clamp(angle_rad, self.tail_min_rad, self.tail_max_rad)

        center_tick   = 2048.0
        ticks_per_rad = 4096.0 / (2.0 * math.pi)

        tick = int(round(center_tick + angle_rad * ticks_per_rad))
        # Clamp finale sui tick EEPROM (ridondante ma esplicito)
        tick = int(clamp(tick, self.min_position_tick, self.max_position_tick))
        return tick

    # ── Callback comando master ──────────────────────────────────────────────

    def on_target(self, msg: Float64):
        target_rad = float(msg.data)
        goal_tick  = self.rad_to_tick(target_rad)

        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, self.ADDR_GOAL_POSITION, goal_tick)

        if result != 0:
            self.get_logger().error(f'Errore comunicazione write goal: {result}')
            return
        if error != 0:
            self.get_logger().error(f'Errore dynamixel write goal: {error}')
            return

    # ── Shutdown ─────────────────────────────────────────────────────────────

    def return_to_center(self):
        """Porta il motore al centro scrivendo direttamente sulla seriale e
        aspetta che ci arrivi (o che scada il timeout). Non usa messaggi ROS,
        quindi funziona anche se il master o il contesto ROS sono gia' chiusi."""
        # Il Ctrl+C puo' interrompere a meta' una lettura/scrittura del timer:
        # l'SDK lascia allora la porta segnata come "occupata" (is_using = True)
        # e ogni nuova richiesta fallisce con COMM_PORT_BUSY (-1000).
        # Qui nessun altro usa piu' la porta: si libera il flag e si svuota il buffer.
        self.port_handler.is_using = False
        try:
            self.port_handler.clearPort()
        except Exception:
            pass

        for _ in range(5):
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.dxl_id, self.ADDR_GOAL_POSITION, self.center_tick)
            if result == 0 and error == 0:
                break
            self.port_handler.is_using = False
            time.sleep(0.02)
        if result != 0 or error != 0:
            print(f'[dynamixel_controller_node] ritorno al centro: errore scrittura '
                  f'(result={result}, error={error})', flush=True)
            return False

        t_end = time.monotonic() + self.shutdown_timeout
        pos_tick = None
        while time.monotonic() < t_end:
            self.port_handler.is_using = False
            pos_tick, result, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.dxl_id, self.ADDR_PRESENT_POSITION)
            if result == 0 and error == 0 and abs(pos_tick - self.center_tick) <= self.shutdown_tolerance:
                print(f'[dynamixel_controller_node] coda al centro (tick {pos_tick}).', flush=True)
                return True
            time.sleep(0.05)
        print(f'[dynamixel_controller_node] ritorno al centro: timeout dopo '
              f'{self.shutdown_timeout:.1f} s (ultima posizione: {pos_tick}).', flush=True)
        return False

    def destroy_node(self):
        try:
            self.return_to_center()
            if not self.keep_torque_on_shutdown:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, self.dxl_id,
                    self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            self.port_handler.closePort()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C diretto oppure arresto da ros2 launch
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()