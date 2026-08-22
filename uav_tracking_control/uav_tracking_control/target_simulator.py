import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class TargetSimulator(Node):
    def __init__(self):
        super().__init__('target_simulator')
        self.pub = self.create_publisher(PoseStamped, '/perception/target_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.radius = 10.0
        self.center = (40.0, 0.0)
        self.omega = 0.3

    def publish_pose(self):
        now = time.time()
        angle = self.omega * now
        x = self.center[0] + self.radius * math.cos(angle)
        y = self.center[1] + self.radius * math.sin(angle)
        z = 0.0
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TargetSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
