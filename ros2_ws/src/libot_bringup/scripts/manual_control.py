#!/usr/bin/env python3

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ManualControl(Node):

    def __init__(self):
        super().__init__('manual_control')

        # =====================================================
        # ROS 2
        # =====================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # =====================================================
        # SPEED
        # =====================================================

        # Mapping needs slow, smooth motion. A fast turn distorts each
        # RPLIDAR revolution and makes scan matching much less reliable.
        self.linear_speed = 0.5
        self.strafe_speed = 0.5
        self.angular_speed = 1

        # =====================================================
        # CURRENT COMMAND
        # =====================================================

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # =====================================================
        # PUBLISH 20 Hz
        # =====================================================

        self.timer = self.create_timer(
            0.05,
            self.publish_command
        )

        # =====================================================
        # INFO
        # =====================================================

        self.get_logger().info('')
        self.get_logger().info('========================================')
        self.get_logger().info('          LIBOT MANUAL CONTROL')
        self.get_logger().info('========================================')
        self.get_logger().info('W : Tien')
        self.get_logger().info('S : Lui')
        self.get_logger().info('A : Sang trai')
        self.get_logger().info('D : Sang phai')
        self.get_logger().info('Q : Xoay trai')
        self.get_logger().info('E : Xoay phai')
        self.get_logger().info('X : Dung')
        self.get_logger().info('R : Reset speed')
        self.get_logger().info('ESC : Thoat')
        self.get_logger().info('========================================')
        self.get_logger().info(
            f'Linear  : {self.linear_speed:.2f} m/s'
        )
        self.get_logger().info(
            f'Strafe  : {self.strafe_speed:.2f} m/s'
        )
        self.get_logger().info(
            f'Angular : {self.angular_speed:.2f} rad/s'
        )
        self.get_logger().info('========================================')

    # =========================================================
    # SET COMMAND
    # =========================================================

    def set_command(self, x=0.0, y=0.0, z=0.0):

        self.current_x = float(x)
        self.current_y = float(y)
        self.current_z = float(z)

        self.get_logger().info(
            f'CMD: x={self.current_x:.2f}, '
            f'y={self.current_y:.2f}, '
            f'z={self.current_z:.2f}'
        )

    # =========================================================
    # PUBLISH COMMAND
    # =========================================================

    def publish_command(self):

        msg = Twist()

        msg.linear.x = self.current_x
        msg.linear.y = self.current_y
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.current_z

        self.cmd_pub.publish(msg)

    # =========================================================
    # STOP
    # =========================================================

    def stop(self):

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

        self.get_logger().info('STOP')

    # =========================================================
    # RESET SPEED
    # =========================================================

    def reset_speed(self):

        self.linear_speed = 0.5
        self.strafe_speed = 0.5
        self.angular_speed = 1

        self.stop()

        self.get_logger().info(
            'Speed reset: '
            'linear=0.15 m/s, '
            'strafe=0.10 m/s, '
            'angular=0.30 rad/s'
        )


# =============================================================
# KEYBOARD
# =============================================================

def get_key(settings):

    tty.setraw(sys.stdin.fileno())

    key = sys.stdin.read(1)

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        settings
    )

    return key


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = ManualControl()

    settings = termios.tcgetattr(sys.stdin)

    try:

        while rclpy.ok():

            # ---------------------------------------------
            # Check keyboard
            # ---------------------------------------------

            if select.select(
                [sys.stdin],
                [],
                [],
                0.01
            )[0]:

                key = get_key(settings)

                # ESC
                if ord(key) == 27:

                    node.stop()
                    break

                key = key.lower()

                # -----------------------------------------
                # W - FORWARD
                # -----------------------------------------

                if key == 'w':

                    node.set_command(
                        x=node.linear_speed,
                        y=0.0,
                        z=0.0
                    )

                # -----------------------------------------
                # S - BACKWARD
                # -----------------------------------------

                elif key == 's':

                    node.set_command(
                        x=-node.linear_speed,
                        y=0.0,
                        z=0.0
                    )

                # -----------------------------------------
                # A - LEFT
                # -----------------------------------------

                elif key == 'a':

                    node.set_command(
                        x=0.0,
                        y=node.strafe_speed,
                        z=0.0
                    )

                # -----------------------------------------
                # D - RIGHT
                # -----------------------------------------

                elif key == 'd':

                    node.set_command(
                        x=0.0,
                        y=-node.strafe_speed,
                        z=0.0
                    )

                # -----------------------------------------
                # Q - ROTATE LEFT
                # -----------------------------------------

                elif key == 'q':

                    node.set_command(
                        x=0.0,
                        y=0.0,
                        z=node.angular_speed
                    )

                # -----------------------------------------
                # E - ROTATE RIGHT
                # -----------------------------------------

                elif key == 'e':

                    node.set_command(
                        x=0.0,
                        y=0.0,
                        z=-node.angular_speed
                    )

                # -----------------------------------------
                # X - STOP
                # -----------------------------------------

                elif key == 'x':

                    node.stop()

                # -----------------------------------------
                # R - RESET SPEED
                # -----------------------------------------

                elif key == 'r':

                    node.reset_speed()

            # ---------------------------------------------
            # ROS 2
            # ---------------------------------------------

            rclpy.spin_once(
                node,
                timeout_sec=0.01
            )

    except KeyboardInterrupt:

        pass

    finally:

        # ---------------------------------------------
        # SAFETY STOP
        # ---------------------------------------------

        node.stop()

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings
        )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
