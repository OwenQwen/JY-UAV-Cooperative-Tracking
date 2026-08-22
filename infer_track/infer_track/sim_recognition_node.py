#!/usr/bin/env python3
"""订阅仿真图像话题，转为OpenCV格式后执行YOLO识别，UDP发送结果."""

import json
import os
import socket
import time

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import torch  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from ultralytics import YOLO  # noqa: E402


MODEL_PATH = os.getenv("MODEL_PATH")
UDP_HOST = os.getenv("UDP_HOST", "127.0.0.1")
UDP_PORT = int(os.getenv("UDP_PORT", "5005"))
IMAGE_TOPIC = os.getenv(
    "IMAGE_TOPIC",
    "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image",
)


class YoloDetector:
    """执行单帧YOLO检测."""

    def __init__(
        self,
        model_path,
        confidence_threshold=0.6,
        iou_threshold=0.5,
        image_size=640,
        device="0",
        half=True,
        max_detections=1,
    ):
        self.use_cuda = str(device).lower() != "cpu" and torch.cuda.is_available()
        if self.use_cuda:
            torch.backends.cudnn.benchmark = True
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
        else:
            self.device = "cpu"
            self.half = False

        self.model = YOLO(model_path)
        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold
        self.image_size = image_size
        self.device = "cuda:0" if self.use_cuda else "cpu"
        self.half = bool(half and self.use_cuda)
        self.max_detections = max_detections

    def warmup(self):
        frame = np.zeros(
            (self.image_size, self.image_size, 3),
            dtype=np.uint8,
        )
        self.detect(frame)

    @torch.inference_mode()
    def detect(self, frame):
        result = self.model.predict(
            frame,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            imgsz=self.image_size,
            device=self.device,
            half=self.half,
            max_det=self.max_detections,
            verbose=False,
        )[0]

        image_height, image_width = frame.shape[:2]
        try:
            box_data = result.boxes.data[:, :6].detach().cpu().numpy()
        except Exception as e:
            print(f"YOLO conversion error: {e}")
            return [], 0, 0, 0, 0
        coordinates = np.rint(box_data[:, :4]).astype(np.int32)
        detections = []
        x1, y1, x2, y2 = 0, 0, 0, 0
        for coordinate, box in zip(coordinates, box_data):
            x1, y1, x2, y2 = coordinate.tolist()
            confidence = float(box[4])
            class_id = int(box[5])
            x1 = max(0, min(image_width - 1, x1))
            y1 = max(0, min(image_height - 1, y1))
            x2 = max(x1, min(image_width - 1, x2))
            y2 = max(y1, min(image_height - 1, y2))

            detections.append(
                {
                    "class_id": class_id + 1,
                    "confidence": confidence,
                    "center_x": (x1 + x2) * 0.5,
                    "center_y": (y1 + y2) * 0.5,
                }
            )
        return detections, x1, y1, x2, y2


class TargetIdFilter:
    """原ID超过指定时间未出现后才接受不同ID."""

    def __init__(self, change_interval=1.0):
        self.change_interval = change_interval
        self.current_id = None
        self.current_id_last_seen = None

    def update(self, detections, now):
        if not detections:
            return []

        detection = detections[0]
        detected_id = detection["class_id"]

        # 第一次识别，直接接受
        if self.current_id is None:
            self.current_id = detected_id
            self.current_id_last_seen = now
            return detections

        # 仍然是当前ID，更新时间
        if detected_id == self.current_id:
            self.current_id_last_seen = now
            return detections

        # 不同ID出现，但原ID最后出现还不足1秒
        if now - self.current_id_last_seen < self.change_interval:
            return []

        # 原ID已经超过1秒没有出现，接受新ID
        self.current_id = detected_id
        self.current_id_last_seen = now
        return detections


class SimRecognitionNode(Node):
    """订阅仿真相机图像，识别目标并通过UDP发送检测结果."""

    def __init__(self):
        super().__init__("sim_recognition_node")
        if not MODEL_PATH:
            raise RuntimeError(
                "MODEL_PATH is required; point it at an authorized YOLO weight file"
            )
        self.declare_parameter("image_topic", IMAGE_TOPIC)
        image_topic = self.get_parameter("image_topic").value

        self.detector = YoloDetector(MODEL_PATH)
        self.detector.warmup()
        self.id_filter = TargetIdFilter(change_interval=1.0)
        self.bridge = CvBridge()
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination = (UDP_HOST, UDP_PORT)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos,
        )
        self.get_logger().info(
            f"sim recognition started: model={MODEL_PATH}, "
            f"image_topic={image_topic}, "
            f"destination={UDP_HOST}:{UDP_PORT}"
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"image conversion failed: {exc}")
            return

        if frame is None or frame.size == 0:
            return

        detections, x1, y1, x2, y2 = self.detector.detect(frame)
        self.get_logger().info(f"Raw detections: {detections}")
        if not detections:
            return
        if detections[0]["confidence"] < 0.6:
            return
        # if detections[0]["class_id"] == 0:
        #     return
        height, width = frame.shape[:2]
        # border = 20

        # if (
        #     x1 < border or y1 < border
        #     or x2 > width - border or y2 > height - border
        # ):
        #     return
        # area = (x2 - x1) * (y2 - y1)

        # if area < 500:
        #     return
        detections = self.id_filter.update(
            detections,
            time.monotonic(),
        )
        if not detections:
            return
        payload = {
            "stamp": time.time(),
            "image_width": width,
            "image_height": height,
            "detections": detections,
        }
        packet = json.dumps(
            payload,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self.udp_socket.sendto(packet, self.destination)
        print(f"sent {len(packet)} bytes to {self.destination}: {payload}")

    def destroy_node(self):
        self.udp_socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimRecognitionNode()
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
