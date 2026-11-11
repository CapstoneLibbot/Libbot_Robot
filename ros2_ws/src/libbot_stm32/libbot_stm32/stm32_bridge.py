
import math
import time

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, Range
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from libbot_stm32.protocol import parse_imu_line, parse_range_line
from libbot_stm32.velocity_limits import clamp_velocity


class STM32Bridge(Node):
    # Physical dimensions (meters), center-to-center measurements
    WHEEL_DIAMETER = 0.06
    WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
    ENCODER_CPR = 1980.0  
    WHEEL_BASE = 0.27     # front to rear wheel centers
    TRACK_WIDTH = 0.33    # left to right wheel centers
    L = WHEEL_BASE / 2.0
    W = TRACK_WIDTH / 2.0
    L_PLUS_W = L + W

    # M1=front-left, M2=front-right, M3=rear-left, M4=rear-right.
    M1_SIGN = 1
    M2_SIGN = -1
    M3_SIGN = 1
    M4_SIGN = -1

    ENCODER_TIMEOUT_SEC = 0.30
    ENCODER_PACKET_PERIOD_SEC = 0.02
    ENCODER_DIAGNOSTIC_LOG_INTERVAL_SEC = 2.0
    ODOM_MAX_WHEEL_SPEED_MPS = 0.30
    ENCODER_JUMP_MARGIN = 1.5
    CMD_TIMEOUT_SEC = 0.50  # Safety stop command; STM32 needs its own watchdog
    STM32_MAX_VX_MPS = 1.0
    STM32_MAX_VY_MPS = 1.0
    STM32_MAX_WZ_RAD_S = 3.0
    MAX_RX_BYTES = 8192
    MAX_LINE_BYTES = 256

    def __init__(self):
        super().__init__('stm32_bridge')
        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')
        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.serial = serial.Serial(self.port, self.baudrate, timeout=0, write_timeout=0.1)
        self.get_logger().info(f'Connected to STM32: {self.port} @ {self.baudrate}')

        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )
        self.status_pub = self.create_publisher(String, '/stm32_bridge/status', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 20)
        self.range_publishers = {
            'front': self.create_publisher(Range, '/range/front', 10),
            'left': self.create_publisher(Range, '/range/left', 10),
            'right': self.create_publisher(Range, '/range/right', 10),
            'rear': self.create_publisher(Range, '/range/rear', 10),
        }
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_enc = None
        self.last_encoder_monotonic = None
        self.last_encoder_receive_monotonic = None
        self.last_encoder_diagnostic_log = 0.0
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_wz = 0.0
        self.battery_voltage = float('nan')
        self.battery_percentage = float('nan')

        self.last_cmd_monotonic = time.monotonic()
        self.command_is_active = False
        self.rx_buffer = bytearray()
        self.last_encoder_timeout_logged = False
        self.last_serial_error_logged = 0.0
        self.last_velocity_ack_logged = 0.0
        self.last_imu_calibration = None
        self.last_blocked_mask = None

        self.serial_timer = self.create_timer(0.005, self.serial_callback)
        self.odom_timer = self.create_timer(0.02, self.publish_odom)
        self.cmd_watchdog_timer = self.create_timer(0.1, self.command_watchdog)

        self.get_logger().info(
            f'STM32 bridge started: wheel={self.WHEEL_DIAMETER:.3f}m, '
            f'CPR={self.ENCODER_CPR:.0f}, wheelbase={self.WHEEL_BASE:.3f}m, '
            f'track={self.TRACK_WIDTH:.3f}m, L+W={self.L_PLUS_W:.3f}m, '
            f'cmd_vel={self.cmd_vel_topic}'
        )

    def send_velocity(self, vx, vy, wz):
        requested = (vx, vy, wz)
        vx, vy, wz = clamp_velocity(
            vx,
            vy,
            wz,
            maximums=(
                self.STM32_MAX_VX_MPS,
                self.STM32_MAX_VY_MPS,
                self.STM32_MAX_WZ_RAD_S,
            ),
        )

        cmd = (
            f'VEL,{int(vx * 1000.0)},'
            f'{int(vy * 1000.0)},'
            f'{int(wz * 1000.0)}\n'
        )

        try:
            data = cmd.encode('ascii')
            written = self.serial.write(data)

            if written != len(data):
                error = f'UART short write: {written}/{len(data)} bytes'
                self.get_logger().error(error)
                self.publish_status(f'UART_TX_ERROR,{error}')
                return

            limited = (vx, vy, wz)
            suffix = '' if limited == requested else (
                f' | limited m/s,rad/s={vx:.3f},{vy:.3f},{wz:.3f}'
            )
            self.get_logger().debug(f'TX UART: {cmd.strip()} | written={written}/{len(data)}{suffix}')

        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'UART error: {exc}')
            self.publish_status(f'UART_TX_ERROR,{exc}')

    def publish_status(self, status):
        message = String()
        message.data = str(status)
        self.status_pub.publish(message)

    def cmd_vel_callback(self, msg):
        vx, vy, wz = float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            self.get_logger().warning(
                f'Ignoring non-finite velocity on {self.cmd_vel_topic}'
            )
            return
        self.last_cmd_monotonic = time.monotonic()
        self.command_is_active = any(v != 0.0 for v in (vx, vy, wz))
        self.send_velocity(vx, vy, wz)

    def command_watchdog(self):
        if self.command_is_active and (time.monotonic() - self.last_cmd_monotonic) > self.CMD_TIMEOUT_SEC:
            self.send_velocity(0.0, 0.0, 0.0)
            self.command_is_active = False
            self.get_logger().warning(
                f'No {self.cmd_vel_topic} recently; sent zero velocity to STM32'
            )

    def serial_callback(self):
        try:
            available = self.serial.in_waiting
            if not available:
                return
            chunk = self.serial.read(min(available, 4096))
            if not chunk:
                return
            self.rx_buffer.extend(chunk)
            if len(self.rx_buffer) > self.MAX_RX_BYTES:
                self.rx_buffer.clear()
                self.get_logger().warning('UART receive buffer overflow; discarded incomplete data')
                return
            while b'\n' in self.rx_buffer:
                line_bytes, _, remainder = self.rx_buffer.partition(b'\n')
                self.rx_buffer = bytearray(remainder)
                if len(line_bytes) > self.MAX_LINE_BYTES:
                    continue
                line = line_bytes.decode('utf-8', errors='ignore').strip()
                if line.startswith('ENC,'):
                    self.process_encoder(line)
                elif line.startswith('BAT,'):
                    self.process_battery(line)
                elif line.startswith('IMU,'):
                    self.process_imu(line)
                elif line.startswith('RNG,'):
                    self.process_ranges(line)
                elif line.startswith('SAFE,'):
                    self.process_safety(line)
                elif line.startswith('LiBot STM32 READY'):
                    self.get_logger().info(line)
                    self.publish_status('STM32_READY')
                elif line == 'OK,VEL':
                    self.publish_status('ACK,VEL')
                    now = time.monotonic()
                    if now - self.last_velocity_ack_logged >= 1.0:
                        self.get_logger().info('STM32 acknowledged VEL commands')
                        self.last_velocity_ack_logged = now
                elif line.startswith('ERR,'):
                    self.get_logger().error(f'STM32 rejected command: {line}')
                    self.publish_status(f'MCU_ERROR,{line}')
                elif line:
                    self.get_logger().debug(f'STM32: {line}')
        except (serial.SerialException, OSError) as exc:
            if time.monotonic() - self.last_serial_error_logged > 2.0:
                self.get_logger().error(f'UART read error: {exc}')
                self.last_serial_error_logged = time.monotonic()

    @staticmethod
    def battery_voltage_to_percentage(voltage):
        # Approximate voltage-only 3S LiPo indication; not a calibrated SOC.
        table = [
            (12.60, 100.0), (12.39, 90.0), (12.18, 80.0),
            (11.97, 70.0), (11.76, 60.0), (11.55, 50.0),
            (11.34, 40.0), (11.13, 30.0), (10.92, 20.0),
            (10.71, 10.0), (10.50, 0.0),
        ]
        if voltage >= table[0][0]:
            return 100.0
        if voltage <= table[-1][0]:
            return 0.0
        for (hi_v, hi_p), (lo_v, lo_p) in zip(table, table[1:]):
            if lo_v <= voltage <= hi_v:
                return lo_p + (voltage - lo_v) * (hi_p - lo_p) / (hi_v - lo_v)
        return 0.0

    def process_battery(self, line):
        try:
            parts = line.split(',')
            if len(parts) != 2:
                return
            if parts[1].strip() == 'ERROR':
                self.get_logger().warning('STM32: INA226 voltage read error')
                return
            voltage = float(parts[1].strip()) / 1000.0
            if not math.isfinite(voltage) or voltage <= 0.0:
                return
            percentage = self.battery_voltage_to_percentage(voltage)
            self.battery_voltage = voltage
            self.battery_percentage = percentage

            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.voltage = voltage
            msg.percentage = percentage / 100.0  # BatteryState uses 0..1
            msg.current = float('nan')
            msg.charge = float('nan')
            msg.capacity = 3.0
            msg.design_capacity = 3.0
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            msg.present = True
            self.battery_pub.publish(msg)
        except (ValueError, OverflowError) as exc:
            self.get_logger().warning(f'Battery parse error: {exc}')

    def process_imu(self, line):
        try:
            values = parse_imu_line(line)
            qw, qx, qy, qz = (value / 16384.0 for value in values[0:4])
            norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if not 0.5 <= norm <= 1.5:
                raise ValueError(f'invalid quaternion norm {norm:.3f}')
            qw, qx, qy, qz = (value / norm for value in (qw, qx, qy, qz))
        except (ValueError, OverflowError) as exc:
            self.get_logger().warning(f'IMU parse error: {exc}')
            return

        calibration = values[10:14]
        calibrated = (
            calibration[0] >= 2
            and calibration[1] >= 2
            and calibration[3] >= 2
        )
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.angular_velocity.x = values[4] / 1000.0
        msg.angular_velocity.y = values[5] / 1000.0
        msg.angular_velocity.z = values[6] / 1000.0
        msg.linear_acceleration.x = values[7] / 1000.0
        msg.linear_acceleration.y = values[8] / 1000.0
        msg.linear_acceleration.z = values[9] / 1000.0

        orientation_variance = 0.03 if calibrated else 1.0
        msg.orientation_covariance = [
            orientation_variance, 0.0, 0.0,
            0.0, orientation_variance, 0.0,
            0.0, 0.0, orientation_variance,
        ]
        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02,
        ]
        msg.linear_acceleration_covariance = [
            0.2, 0.0, 0.0,
            0.0, 0.2, 0.0,
            0.0, 0.0, 0.2,
        ]
        self.imu_pub.publish(msg)

        if calibration != self.last_imu_calibration:
            self.last_imu_calibration = calibration
            status = 'IMU_CAL,' + ','.join(str(value) for value in calibration)
            self.publish_status(status)
            self.get_logger().info(status)

    def process_ranges(self, line):
        try:
            distances, valid_mask, blocked_mask = parse_range_line(line)
        except (ValueError, OverflowError) as exc:
            self.get_logger().warning(f'Range parse error: {exc}')
            return

        stamp = self.get_clock().now().to_msg()
        names = ('front', 'left', 'right', 'rear')
        for index, name in enumerate(names):
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = f'ultrasonic_{name}_link'
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = math.radians(30.0)
            msg.min_range = 0.02
            msg.max_range = 4.0
            if (valid_mask & (1 << index)) and distances[index] >= 0:
                msg.range = distances[index] / 1000.0
            else:
                # A timeout means no return inside the sensor range, not a
                # close obstacle. A max-range reading also clears RangeLayer.
                msg.range = msg.max_range
            self.range_publishers[name].publish(msg)

        if blocked_mask != self.last_blocked_mask:
            self.last_blocked_mask = blocked_mask
            self.publish_status(f'SAFETY_BLOCKED,{blocked_mask}')

    def process_safety(self, line):
        try:
            blocked_mask = int(line.split(',', 1)[1])
            if blocked_mask < 0 or blocked_mask > 0x0F:
                raise ValueError('mask outside 0..15')
        except (IndexError, ValueError):
            self.get_logger().warning(f'Invalid safety packet: {line[:80]}')
            return
        self.last_blocked_mask = blocked_mask
        self.publish_status(f'SAFETY_BLOCKED,{blocked_mask}')
    def process_encoder(self, line):
        try:
            parts = line.split(',')
            if len(parts) != 5:
                self._log_encoder_diagnostic(
                    f'Invalid encoder packet: expected ENC plus 4 counters, got {line[:80]}'
                )
                return
            enc = tuple(int(value) for value in parts[1:])

        except ValueError:
            self._log_encoder_diagnostic(f'Invalid encoder packet: {line[:80]}')
            return

        now = time.monotonic()
        self.last_encoder_receive_monotonic = now
        self.last_encoder_timeout_logged = False
        if self.last_enc is None:
            self.last_enc = enc
            self.last_encoder_monotonic = now
            return

        dt = now - self.last_encoder_monotonic
        deltas = [new - old for new, old in zip(enc, self.last_enc)]
        self.last_enc = enc
        self.last_encoder_monotonic = now
        if dt <= 0.0:
            return

        # A reset of the STM32 changes its cumulative counters abruptly. Do
        # not integrate that discontinuity into odometry: it would move the
        # robot pose instantaneously and create a second, offset SLAM map.
        # Several buffered UART lines can arrive in one ROS timer callback,
        # so the physical packet period is the lower bound for this check.
        scale = self.WHEEL_CIRCUMFERENCE / self.ENCODER_CPR
        effective_dt = max(dt, self.ENCODER_PACKET_PERIOD_SEC)
        max_delta = max(
            100.0,
            self.ODOM_MAX_WHEEL_SPEED_MPS * effective_dt
            / scale * self.ENCODER_JUMP_MARGIN,
        )
        if any(abs(delta) > max_delta for delta in deltas):
            self.odom_vx = 0.0
            self.odom_vy = 0.0
            self.odom_wz = 0.0
            self.get_logger().warning(
                f'Encoder discontinuity ignored: deltas={deltas}, '
                f'limit={max_delta:.0f} counts'
            )
            return

        d1 = deltas[0] * self.M1_SIGN
        d2 = deltas[1] * self.M2_SIGN
        d3 = deltas[2] * self.M3_SIGN
        d4 = deltas[3] * self.M4_SIGN


        self.get_logger().debug(
            f"ENC RAW DELTA: "
            f"M1={deltas[0]}, "
            f"M2={deltas[1]}, "
            f"M3={deltas[2]}, "
            f"M4={deltas[3]} | "
            f"ENC NORMALIZED: "
            f"M1={d1}, M2={d2}, M3={d3}, M4={d4}"
        )

        s1, s2, s3, s4 = (d1 * scale, d2 * scale, d3 * scale, d4 * scale)

        # Mecanum X-roller configuration, wheel order FL, FR, RL, RR.
        # +x forward, +y left, +yaw counterclockwise.
        dx = (s1 + s2 + s3 + s4) / 4.0
        dy = (s1 - s2 - s3 + s4) / 4.0
        dtheta = (s1 - s2 + s3 - s4) / (4.0 * self.L_PLUS_W)

        theta_mid = self.theta + 0.5 * dtheta
        self.x += dx * math.cos(theta_mid) - dy * math.sin(theta_mid)
        self.y += dx * math.sin(theta_mid) + dy * math.cos(theta_mid)
        self.theta = math.atan2(math.sin(self.theta + dtheta), math.cos(self.theta + dtheta))

        self.odom_vx = dx / dt
        self.odom_vy = dy / dt
        self.odom_wz = dtheta / dt

    def _log_encoder_diagnostic(self, message):
        now = time.monotonic()
        if (
            now - self.last_encoder_diagnostic_log
            < self.ENCODER_DIAGNOSTIC_LOG_INTERVAL_SEC
        ):
            return
        self.last_encoder_diagnostic_log = now
        self.get_logger().warning(message)

    def publish_odom(self):
        now = self.get_clock().now()
        encoder_timed_out = (
            self.last_encoder_receive_monotonic is None
            or time.monotonic() - self.last_encoder_receive_monotonic > self.ENCODER_TIMEOUT_SEC
        )
        if encoder_timed_out:
            self.odom_vx = 0.0
            self.odom_vy = 0.0
            self.odom_wz = 0.0
            if not self.last_encoder_timeout_logged:
                if self.last_encoder_receive_monotonic is None:
                    message = 'No encoder packets received; odom pose will remain unchanged'
                else:
                    message = 'Encoder timeout; odom twist set to zero (pose not reset)'
                self.get_logger().warning(message)
                self.publish_status('ENCODER_TIMEOUT')
                self.last_encoder_timeout_logged = True

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.odom_vx
        odom.twist.twist.linear.y = self.odom_vy
        odom.twist.twist.angular.z = self.odom_wz

        # Placeholder covariances: tune using measurements; planar robot.
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.1
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def shutdown(self):
        try:
            self.send_velocity(0.0, 0.0, 0.0)
        except Exception:
            pass
        if self.serial.is_open:
            self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = STM32Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
