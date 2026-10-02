#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster


class STM32Bridge(Node):

    # =========================================================
    # ROBOT PARAMETERS
    # =========================================================

    WHEEL_DIAMETER = 0.060       # meter
    WHEEL_RADIUS = 0.030        # meter

    WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER

    ENCODER_CPR = 1980.0

    # Distance between left/right wheel centers = 0.30 m
    # Distance between front/rear wheel centers = 0.30 m
    #
    # For mecanum kinematic model:
    # L = half front/rear distance
    # W = half left/right distance

    L = 0.15
    W = 0.15

    L_PLUS_W = L + W

    # =========================================================
    # INIT
    # =========================================================

    def __init__(self):

        super().__init__('stm32_bridge')

        # -----------------------------------------------------
        # PARAMETERS
        # -----------------------------------------------------

        self.declare_parameter(
            'port',
            '/dev/serial0'
        )

        self.declare_parameter(
            'baudrate',
            115200
        )

        self.port = self.get_parameter(
            'port'
        ).value

        self.baudrate = self.get_parameter(
            'baudrate'
        ).value

        # -----------------------------------------------------
        # SERIAL
        # -----------------------------------------------------

        try:

            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.01
            )

            self.get_logger().info(
                f'Connected to STM32: {self.port}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Cannot open STM32 serial: {e}'
            )

            raise

        # -----------------------------------------------------
        # CMD_VEL
        # -----------------------------------------------------

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # -----------------------------------------------------
        # ODOM
        # -----------------------------------------------------

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            20
        )

        # -----------------------------------------------------
        # TF
        # -----------------------------------------------------

        self.tf_broadcaster = TransformBroadcaster(
            self
        )

        # -----------------------------------------------------
        # ROBOT POSE
        # -----------------------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # -----------------------------------------------------
        # ENCODER STATE
        # -----------------------------------------------------

        self.last_enc = None

        # -----------------------------------------------------
        # VELOCITY
        # -----------------------------------------------------

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # -----------------------------------------------------
        # TIMING
        # -----------------------------------------------------

        self.last_time = self.get_clock().now()

        # -----------------------------------------------------
        # SERIAL TIMER
        # -----------------------------------------------------

        self.serial_timer = self.create_timer(
            0.005,
            self.serial_callback
        )

        # -----------------------------------------------------
        # ODOM TIMER
        # -----------------------------------------------------

        self.odom_timer = self.create_timer(
            0.02,
            self.publish_odom
        )

        self.get_logger().info(
            'STM32 bridge started'
        )

        self.get_logger().info(
            f'Wheel diameter: {self.WHEEL_DIAMETER} m'
        )

        self.get_logger().info(
            f'Encoder CPR: {self.ENCODER_CPR}'
        )

        self.get_logger().info(
            f'L+W: {self.L_PLUS_W} m'
        )

    # =========================================================
    # CMD_VEL
    # =========================================================

    def cmd_vel_callback(self, msg):

        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

        # ROS units:
        #
        # vx = m/s
        # vy = m/s
        # wz = rad/s
        #
        # STM32 protocol:
        #
        # vx = mm/s
        # vy = mm/s
        # wz = mrad/s

        vx_mm = int(self.vx * 1000.0)
        vy_mm = int(self.vy * 1000.0)
        wz_mrad = int(self.wz * 1000.0)

        command = (
            f'VEL,{vx_mm},{vy_mm},{wz_mrad}\n'
        )

        try:

            self.serial.write(
                command.encode()
            )

        except Exception as e:

            self.get_logger().error(
                f'UART write error: {e}'
            )

    # =========================================================
    # SERIAL
    # =========================================================

    def serial_callback(self):

        try:

            while self.serial.in_waiting:

                line = self.serial.readline()

                if not line:
                    continue

                line = line.decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if not line:
                    continue

                # -------------------------------------------------
                # READY
                # -------------------------------------------------

                if line.startswith('LiBot STM32 READY'):

                    self.get_logger().info(
                        line
                    )

                # -------------------------------------------------
                # ENCODER
                # -------------------------------------------------

                elif line.startswith('ENC,'):

                    self.process_encoder(
                        line
                    )

                else:

                    self.get_logger().debug(
                        f'STM32: {line}'
                    )

        except Exception as e:

            self.get_logger().error(
                f'UART read error: {e}'
            )

    # =========================================================
    # PROCESS ENCODER
    # =========================================================

    def process_encoder(self, line):

        try:

            parts = line.split(',')

            if len(parts) != 5:
                return

            m1 = int(parts[1])
            m2 = int(parts[2])
            m3 = int(parts[3])
            m4 = int(parts[4])

            current_enc = (
                m1,
                m2,
                m3,
                m4
            )

            # -------------------------------------------------
            # FIRST MESSAGE
            # -------------------------------------------------

            if self.last_enc is None:

                self.last_enc = current_enc

                self.last_time = (
                    self.get_clock().now()
                )

                return

            # -------------------------------------------------
            # ENCODER DELTA
            # -------------------------------------------------

            d1 = m1 - self.last_enc[0]
            d2 = m2 - self.last_enc[1]
            d3 = m3 - self.last_enc[2]
            d4 = m4 - self.last_enc[3]

            self.last_enc = current_enc

            # -------------------------------------------------
            # COUNTS -> WHEEL DISTANCE
            # -------------------------------------------------

            meters_per_count = (
                self.WHEEL_CIRCUMFERENCE
                / self.ENCODER_CPR
            )

            wheel_d1 = d1 * meters_per_count
            wheel_d2 = d2 * meters_per_count
            wheel_d3 = d3 * meters_per_count
            wheel_d4 = d4 * meters_per_count

            # -------------------------------------------------
            # MECANUM ODOMETRY
            # -------------------------------------------------

            dx = (
                wheel_d1
                + wheel_d2
                + wheel_d3
                + wheel_d4
            ) / 4.0

            dy = (
                -wheel_d1
                + wheel_d2
                + wheel_d3
                -wheel_d4
            ) / 4.0

            dtheta = (
                -wheel_d1
                + wheel_d2
                -wheel_d3
                + wheel_d4
            ) / (
                4.0 * self.L_PLUS_W
            )

            # -------------------------------------------------
            # UPDATE WORLD POSITION
            # -------------------------------------------------

            cos_theta = math.cos(
                self.theta
            )

            sin_theta = math.sin(
                self.theta
            )

            self.x += (
                dx * cos_theta
                - dy * sin_theta
            )

            self.y += (
                dx * sin_theta
                + dy * cos_theta
            )

            self.theta += dtheta

            # Normalize theta to [-pi, pi]

            self.theta = math.atan2(
                math.sin(self.theta),
                math.cos(self.theta)
            )

        except Exception as e:

            self.get_logger().error(
                f'Encoder parse error: {e}'
            )

    # =========================================================
    # PUBLISH ODOM
    # =========================================================

    def publish_odom(self):

        now = self.get_clock().now()

        # -----------------------------------------------------
        # ODOM MESSAGE
        # -----------------------------------------------------

        odom = Odometry()

        odom.header.stamp = now.to_msg()

        odom.header.frame_id = 'odom'

        odom.child_frame_id = 'base_link'

        # -----------------------------------------------------
        # POSITION
        # -----------------------------------------------------

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # -----------------------------------------------------
        # YAW -> QUATERNION
        # -----------------------------------------------------

        qz = math.sin(
            self.theta / 2.0
        )

        qw = math.cos(
            self.theta / 2.0
        )

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # -----------------------------------------------------
        # VELOCITY
        # -----------------------------------------------------

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz

        # -----------------------------------------------------
        # COVARIANCE
        # -----------------------------------------------------

        odom.pose.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    0.1,  0,    0,    0,
            0,    0,    0,    0.1,  0,    0,
            0,    0,    0,    0,    0.1,  0,
            0,    0,    0,    0,    0,    0.1
        ]

        odom.twist.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    0.1,  0,    0,    0,
            0,    0,    0,    0.1,  0,    0,
            0,    0,    0,    0,    0.1,  0,
            0,    0,    0,    0,    0,    0.1
        ]

        self.odom_pub.publish(
            odom
        )

        # -----------------------------------------------------
        # TF: odom -> base_link
        # -----------------------------------------------------

        transform = TransformStamped()

        transform.header.stamp = now.to_msg()

        transform.header.frame_id = 'odom'

        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(
            transform
        )


def main(args=None):

    rclpy.init(args=args)

    node = STM32Bridge()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        try:

            node.serial.close()

        except Exception:
            pass

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()