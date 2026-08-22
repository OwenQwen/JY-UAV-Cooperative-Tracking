#!/usr/bin/env python3
"""接收YOLO UDP数据并发布ROS 2话题."""

import os
import socket
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


UDP_BIND_ADDRESS = os.getenv("UDP_BIND_ADDRESS", "0.0.0.0")
UDP_PORT = int(os.getenv("UDP_PORT", "5005"))
DETECTIONS_TOPIC = "/target_detections"
POLL_PERIOD = 0.01


class YoloUdpReceiver(Node):
    """把最新一帧YOLO UDP数据原样发布到ROS 2."""

    def __init__(self):
        super().__init__("yolo_udp_receiver")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(
            String,
            DETECTIONS_TOPIC,
            qos,
        )

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((UDP_BIND_ADDRESS, UDP_PORT))
        self.udp_socket.setblocking(False)
        self.timer = self.create_timer(POLL_PERIOD, self.receive_udp)

        self.get_logger().info(
            f"YOLO UDP receiver listening on "
            f"{UDP_BIND_ADDRESS}:{UDP_PORT}"
        )

    def receive_udp(self):
        packet = None
        while True:
            try:
                packet, _address = self.udp_socket.recvfrom(65535)
            except BlockingIOError:
                break

        if packet is not None:

            # self.publisher.publish(
            #     String(data=packet.decode("utf-8"))
            # )
            data = json.loads(packet.decode("utf-8"))
            # 进入ROS系统后统一使用ROS时间
            data["stamp"] = (self.get_clock().now().nanoseconds * 1e-9)

            self.publisher.publish(String(
                data=json.dumps(
                    data,
                    allow_nan=False,
                    separators=(",", ":"),
                )
            ))

    def destroy_node(self):
        self.udp_socket.close()
        return super().destroy_node()


def main(args=None):
    """启动YOLO UDP接收节点."""
    rclpy.init(args=args)
    node = None
    try:
        node = YoloUdpReceiver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
