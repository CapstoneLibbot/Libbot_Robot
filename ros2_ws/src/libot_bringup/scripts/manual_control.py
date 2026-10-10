#!/usr/bin/env python3

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ManualControl(Node):

    PUBLISH_PERIOD = 0.05
    MAX_LINEAR_ACCEL = 0.35   # m/s^2; limit chassis shock while mapping
    MAX_ANGULAR_ACCEL = 0.80  # rad/s^2

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

        self.linear_speed = 0.20
        self.strafe_speed = 0.15
        self.angular_speed = 0.50

        # =====================================================
        # CURRENT COMMAND
        # =====================================================

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        # =====================================================
        # PUBLISH 20 Hz
        # =====================================================

        self.timer = self.create_timer(
            self.PUBLISH_PERIOD,
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

        self.target_x = float(x)
        self.target_y = float(y)
        self.target_z = float(z)

        self.get_logger().info(
            f'TARGET: x={self.target_x:.2f}, '
            f'y={self.target_y:.2f}, '
            f'z={self.target_z:.2f}'
        )

    # =========================================================
    # PUBLISH COMMAND
    # =========================================================

    def publish_command(self):

        linear_step = self.MAX_LINEAR_ACCEL * self.PUBLISH_PERIOD
        angular_step = self.MAX_ANGULAR_ACCEL * self.PUBLISH_PERIOD

        self.current_x = self.move_towards(
            self.current_x, self.target_x, linear_step
        )
        self.current_y = self.move_towards(
            self.current_y, self.target_y, linear_step
        )
        self.current_z = self.move_towards(
            self.current_z, self.target_z, angular_step
        )

        msg = Twist()

        msg.linear.x = self.current_x
        msg.linear.y = self.current_y
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.current_z

        self.cmd_pub.publish(msg)

    @staticmethod
    def move_towards(current, target, maximum_step):

        if current < target:
            return min(current + maximum_step, target)
        if current > target:
            return max(current - maximum_step, target)
        return target

    # =========================================================
    # STOP
    # =========================================================

    def stop(self):

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
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

        self.linear_speed = 0.20
        self.strafe_speed = 0.15
        self.angular_speed = 0.50

        self.stop()

        self.get_logger().info(
            'Speed reset: '
            'linear=0.20 m/s, '
            'strafe=0.15 m/s, '
            'angular=0.50 rad/s'
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

                    node.set_command()

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
