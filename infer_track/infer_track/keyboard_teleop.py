#!/usr/bin/env python3
"""键盘控制小车移动，发布 /cmd_vel (geometry_msgs/msg/Twist)."""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 默认参数
LINEAR_SPEED = 0.2   # 前进/后退速度 (m/s)
ANGULAR_SPEED = 1.0  # 旋转速度 (rad/s)
TOPIC_NAME = '/cmd_vel'

# 按键映射
KEY_BINDINGS = {
    'w': (LINEAR_SPEED, 0.0),          # 前进
    's': (-LINEAR_SPEED, 0.0),         # 后退
    'a': (0.0, ANGULAR_SPEED),         # 左转
    'd': (0.0, -ANGULAR_SPEED),        # 右转
    'q': (LINEAR_SPEED, ANGULAR_SPEED),  # 左前
    'e': (LINEAR_SPEED, -ANGULAR_SPEED), # 右前
    'z': (-LINEAR_SPEED, ANGULAR_SPEED), # 左后
    'c': (-LINEAR_SPEED, -ANGULAR_SPEED),# 右后
    ' ': (0.0, 0.0),                   # 停止
}

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, TOPIC_NAME, 10)
        self.get_logger().info(
            f"键盘控制启动，发布到 {TOPIC_NAME}\n"
            "按键: w/a/s/d 移动, q/e/z/c 斜向, 空格停止, Ctrl+C 退出"
        )
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        """非阻塞读取单个按键."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in KEY_BINDINGS:
                    linear, angular = KEY_BINDINGS[key]
                    twist = Twist()
                    twist.linear.x = linear
                    twist.angular.z = angular
                    self.publisher.publish(twist)
                    self.get_logger().info(f"vx={linear:.1f}, wz={angular:.1f}")
                elif key == '\x03':  # Ctrl+C
                    break
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            # 停止小车
            twist = Twist()
            self.publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
