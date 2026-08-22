#!/usr/bin/env python3
"""使用YOLOv8识别目标并通过UDP发送检测结果."""

import json
import os
import socket
import time


import cv2  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from ultralytics import YOLO  # noqa: E402


MODEL_PATH = os.getenv("MODEL_PATH")
UDP_HOST = os.getenv("UDP_HOST", "127.0.0.1")
UDP_PORT = int(os.getenv("UDP_PORT", "5005"))
CAMERA_DEVICE = 0
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1200
CAMERA_FPS = 30.0


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
        self.use_cuda = str(device).lower() != "cpu"
        if self.use_cuda and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but no NVIDIA GPU is available")
        if self.use_cuda:
            torch.backends.cudnn.benchmark = True
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

        self.model = YOLO(model_path)
        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold
        self.image_size = image_size
        self.device = device
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
        box_data = result.boxes.data[:, :6].detach().cpu().numpy()
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
                    "class_id": class_id,
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


class RecognitionUdpSender:
    """读取相机、执行检测并发送UDP数据."""

    def __init__(self):
        if not MODEL_PATH:
            raise RuntimeError(
                "MODEL_PATH is required; point it at an authorized YOLO weight file"
            )
        self.detector = YoloDetector(MODEL_PATH)
        self.detector.warmup()
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination = (UDP_HOST, UDP_PORT)
        # self.tartget_history = deque(maxlen=20)
        # self.last_detection = None
        # self.last_class_id = None
        # self.chage_period = 1
        self.capture = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(f"cannot open /dev/video{CAMERA_DEVICE}")
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.capture.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.id_filter = TargetIdFilter(change_interval=1.0)

    def run(self):
        frame_period = 1.0 / CAMERA_FPS
        print(
            f"YOLO UDP sender started: model={MODEL_PATH}, "
            f"destination={UDP_HOST}:{UDP_PORT}"
        )
        try:
            while True:
                started = time.perf_counter()
                success, frame = self.capture.read()
                if not success or frame is None:
                    time.sleep(0.05)
                    continue

                detections, x1, y1, x2, y2 = self.detector.detect(frame)

                if not detections:
                    continue
                if detections[0]["confidence"] < 0.6:
                    continue
                if detections[0]["class_id"] == 0:
                    continue
                height, width = frame.shape[:2]
                border = 20

                if (
                    x1 < border or y1 < border
                    or x2 > width - border or y2 > height - border
                ):
                    continue
                area = (x2 - x1) * (y2 - y1)

                if area < 500:
                    continue
                detections = self.id_filter.update(
                    detections,
                    time.monotonic(),
                )
                if not detections:
                    continue
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

                remaining = frame_period - (time.perf_counter() - started)
                if remaining > 0.0:
                    time.sleep(remaining)
        finally:
            self.capture.release()
            self.udp_socket.close()


def main():
    sender = RecognitionUdpSender()
    try:
        sender.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
