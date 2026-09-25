#!/usr/bin/env python3

import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class STM32Bridge(Node):

    def __init__(self):
        super().__init__('stm32_bridge')

        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1
            )

            self.get_logger().info(
                f'Connected to STM32: {port} @ {baudrate}'
            )

        except serial.SerialException as e:
            self.serial = None

            self.get_logger().error(
                f'Failed to open STM32 UART: {e}'
            )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info(
            'STM32 Bridge started'
        )

    def cmd_vel_callback(self, msg: Twist):

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        command = f'VEL,{vx:.4f},{vy:.4f},{wz:.4f}\n'

        if self.serial is None:
            return

        try:
            self.serial.write(command.encode('utf-8'))

            self.get_logger().info(
                f'UART TX: {command.strip()}'
            )

        except serial.SerialException as e:
            self.get_logger().error(
                f'UART TX error: {e}'
            )

    def destroy_node(self):

        if self.serial is not None:
            try:
                self.serial.write(
                    b'VEL,0.0000,0.0000,0.0000\n'
                )
                self.serial.close()
            except serial.SerialException:
                pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = STM32Bridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()