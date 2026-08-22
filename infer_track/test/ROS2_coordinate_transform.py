#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
import json
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PointStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String, Float32MultiArray

EARTH_R = 6378137.0  # WGS84赤道半径

# 自定义消息（如果有的话）
# from your_package.msg import TargetGPS


class CoordinateTransform(Node):
    """坐标转换节点."""

    def __init__(self):
        super().__init__('coordinate_transform')

        # 相机内参
        for name, default in (
            ('fx', 1410.0),
            ('fy', 1410.0),
            ('cx', 960.0),
            ('cy', 600.0),
        ):
            self.declare_parameter(name, default)
            setattr(self, name, float(self.get_parameter(name).value))

        # ========== UAV状态变量 ==========
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.rel_altitude = None
        self.current_yaw = 0.0
        self.gps_velocity = None
        self.last_gps_time = None  # 新增：GPS时间戳

        # ========== 卡尔曼滤波器 ==========
        self.kf_X = np.zeros((4, 1))
        self.kf_P = np.eye(4) * 100.0
        self.kf_ref_lat = None
        self.kf_ref_lon = None
        self.kf_ref_set = False
        self.kf_dt = 0.1
        self.kf_A = np.array([
            [1, 0, self.kf_dt, 0],
            [0, 1, 0, self.kf_dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.kf_H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        self.kf_Q = np.diag([0.01, 0.01, 0.1, 0.1])
        self.kf_R = np.diag([0.1, 0.1])
        self.kf_initialized = False
        self.last_kf_update_time = None

        # 卡尔曼预测定时器 - 使用更精确的时间
        self.timer = self.create_timer(0.1, self._predict_step)
        self._last_predict_time = self.get_clock().now()

        # ========== QoS配置 ==========
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # ========== 订阅器 ==========
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self._gps_callback,
            sensor_qos
        )

        self.alt_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self._rel_alt_callback,
            sensor_qos
        )

        self.vel_sub = self.create_subscription(
            TwistStamped,
            '/mavros/global_position/raw/gps_vel',
            self._vel_callback,
            sensor_qos
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            sensor_qos
        )

        self.pixel_sub = self.create_subscription(
            Float32MultiArray,
            '/pixel_coordinates',
            self._pixel_callback,
            sensor_qos
        )

        self.detection_sub = self.create_subscription(
            String,
            '/target_detections',
            self._detection_callback,
            sensor_qos
        )

        # ========== 发布器 ==========
        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/target_gps_coordinates',
            sensor_qos
        )

        self.gps_kalman_pub = self.create_publisher(
            NavSatFix,
            '/target_gps_coordinates_kalman',
            sensor_qos
        )

        self.local_pub = self.create_publisher(
            PointStamped,
            '/target_local_coordinates',
            sensor_qos
        )

        self.detection_gps_pub = self.create_publisher(
            String,
            '/detections_with_gps',
            sensor_qos
        )

        self.target_positions_pub = self.create_publisher(
            String,
            'target_positions',
            sensor_qos
        )

        # 新增：发布带class_id的GPS结果（使用PointStamped）
        self.gps_with_class_pub = self.create_publisher(
            PointStamped,
            '/target_gps_with_class',
            sensor_qos
        )

        self.get_logger().info("coordinate_transform started")

    # ========== 回调函数 ==========

    def _gps_callback(self, msg: NavSatFix):
        """处理GPS数据."""
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        if math.isfinite(msg.altitude):
            self.altitude = msg.altitude
        self.last_gps_time = self.get_clock().now()
        self.get_logger().debug(
            f"GPS更新: lat={self.latitude:.7f}, lon={self.longitude:.7f}"
        )

    def _rel_alt_callback(self, msg: Float64):
        if math.isfinite(msg.data):
            self.rel_altitude = msg.data

    def _vel_callback(self, msg: TwistStamped):
        self.gps_velocity = msg.twist

    def _pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _pixel_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn("像素坐标数据格式错误")
            return

        u = float(msg.data[0])
        v = float(msg.data[1])
        target_alt = float(msg.data[2]) if len(msg.data) >= 3 else 0.0

        self._process_coordinates(u, v, target_alt)

    def _detection_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            detections = data.get('detections', [])

            if not detections:
                return

            for detection in detections:
                center_x = detection.get('center_x')
                center_y = detection.get('center_y')

                if center_x is None or center_y is None:
                    continue

                class_id = detection.get('class_id', -1)
                class_name = detection.get('class_name', 'unknown')
                confidence = detection.get('confidence', 0.0)
                target_alt = 0.0

                self._process_coordinates_with_info(
                    center_x, center_y, target_alt,
                    class_id, class_name, confidence,
                    detection
                )

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON解析错误: {e}")
        except Exception as e:
            self.get_logger().error(f"处理检测结果时出错: {e}")

    # ========== 坐标处理函数 ==========

    def _process_coordinates(self, u, v, target_alt=0.0):
        if not self._ready():
            self.get_logger().warn("UAV数据未就绪")
            return

        gps_result = self._pixel_to_gps(u, v, target_alt)
        gps_kalman_result = self._pixel_to_gps_kalman(u, v, target_alt)
        dx, dy = self._pixel_to_xy(u, v, target_alt)

        if gps_result:
            self._publish_gps(gps_result[0], gps_result[1], self.gps_pub, kalman=False)

        if gps_kalman_result:
            self._publish_gps(gps_kalman_result[0], gps_kalman_result[1],
                              self.gps_kalman_pub, kalman=True)

        self._publish_local(dx, dy)

    def _process_coordinates_with_info(self, u, v, target_alt,
                                       class_id, class_name, confidence,
                                       detection_info=None):
        if not self._ready():
            return

        gps_result = self._pixel_to_gps(u, v, target_alt)
        gps_kalman_result = self._pixel_to_gps_kalman(u, v, target_alt)
        dx, dy = self._pixel_to_xy(u, v, target_alt)

        if gps_result:
            self._publish_target_position(
                class_id, confidence, dx, dy, gps_result
            )

        # 发布带class_id的GPS（使用PointStamped）
        if gps_result:
            self._publish_gps_with_class(
                gps_result[0], gps_result[1],
                class_id, class_name, confidence,
                kalman=False
            )

        if gps_kalman_result:
            self._publish_gps_with_class(
                gps_kalman_result[0], gps_kalman_result[1],
                class_id, class_name, confidence,
                kalman=True
            )

        # 发布完整检测结果
        self._publish_detection_gps(
            class_id, class_name, confidence,
            u, v, dx, dy,
            gps_result, gps_kalman_result,
            detection_info
        )

    # ========== 发布函数 ==========

    def _publish_gps(self, lat, lon, publisher, kalman=False):
        """发布GPS坐标."""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0

        # 使用标准状态码
        msg.status.status = 1  # GPS_FIX
        msg.status.service = 1  # GPS

        publisher.publish(msg)

    def _publish_gps_with_class(self, lat, lon, class_id, class_name, confidence, kalman=False):
        """发布带类别信息的GPS坐标."""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = lat
        msg.point.y = lon
        msg.point.z = float(class_id)  # 使用z存储class_id

        self.gps_with_class_pub.publish(msg)

        self.get_logger().debug(
            f"{'[滤波]' if kalman else ''}发布GPS: "
            f"lat={lat:.7f}, lon={lon:.7f}, class={class_name}({class_id})"
        )

    def _publish_local(self, x, y):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "local"
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self.local_pub.publish(msg)

    def _publish_target_position(
        self, target_id, confidence, offset_x, offset_y, gps_result
    ):
        """发布任务和跟踪节点共同使用的标准目标数据."""
        payload = {
            "stamp": self.get_clock().now().nanoseconds * 1e-9,
            "target_id": int(target_id),
            "confidence": float(confidence),
            "offset_x": float(offset_x),
            "offset_y": float(offset_y),
            "target_latitude": float(gps_result[0]),
            "target_longitude": float(gps_result[1]),
        }
        self.target_positions_pub.publish(
            String(
                data=json.dumps(
                    payload,
                    allow_nan=False,
                    separators=(",", ":"),
                )
            )
        )

    def _publish_detection_gps(self, class_id, class_name, confidence,
                               u, v, dx, dy,
                               gps_result, gps_kalman_result,
                               detection_info=None):
        payload = {
            "stamp": self.get_clock().now().nanoseconds * 1e-9,
            "class_id": class_id,
            "class_name": class_name,
            "confidence": confidence,
            "pixel_u": u,
            "pixel_v": v,
            "local_x": dx,
            "local_y": dy,
        }

        if detection_info:
            payload["detection"] = detection_info

        if gps_result:
            payload["gps_lat"] = gps_result[0]
            payload["gps_lon"] = gps_result[1]

        if gps_kalman_result:
            payload["gps_lat_kalman"] = gps_kalman_result[0]
            payload["gps_lon_kalman"] = gps_kalman_result[1]

        msg = String()
        msg.data = json.dumps(payload, allow_nan=False, separators=(",", ":"))
        self.detection_gps_pub.publish(msg)

    # ========== 坐标转换核心函数 ==========

    def _ready(self):
        return all(
            value is not None and math.isfinite(value)
            for value in (self.latitude, self.longitude, self.altitude)
        )

    def _pixel_to_xy(self, u, v, target_alt=0.0):
        x = (float(u) - self.cx) / self.fx
        y = (float(v) - self.cy) / self.fy

        if (
            self.rel_altitude is not None
            and math.isfinite(self.rel_altitude)
        ):
            dz = self.rel_altitude - float(target_alt)
        else:
            dz = self.altitude - float(target_alt)

        dx_cam = x * dz
        dy_cam = y * dz

        cyaw = math.cos(self.current_yaw)
        syaw = math.sin(self.current_yaw)
        Xr = dx_cam * cyaw - dy_cam * syaw
        Yr = dx_cam * syaw + dy_cam * cyaw

        return Xr, Yr

    def _pixel_to_gps(self, u, v, target_alt=0.0):
        if not self._ready():
            return None

        dx, dy = self._pixel_to_xy(u, v, target_alt)

        dLat = (dy / EARTH_R) * (180.0 / math.pi)
        dLon = (dx / (EARTH_R * math.cos(math.radians(self.latitude)))) * (180.0 / math.pi)

        return (self.latitude + dLat, self.longitude + dLon)

    # ========== 卡尔曼滤波相关 ==========

    def _predict_step(self):
        """执行卡尔曼预测."""
        current_time = self.get_clock().now()
        dt = (current_time - self._last_predict_time).nanoseconds / 1e9
        self._last_predict_time = current_time

        # 限制最小/最大步长
        dt = max(0.01, min(0.2, dt))

        # 更新状态转移矩阵
        self.kf_A[0, 2] = dt
        self.kf_A[1, 3] = dt

        if self.kf_initialized:
            self.kf_X = self.kf_A @ self.kf_X
            self.kf_P = self.kf_A @ self.kf_P @ self.kf_A.T + self.kf_Q * dt / 0.1  # 按时间缩放

    def _pixel_to_gps_kalman(self, u, v, target_alt=0.0):
        if not self._ready():
            return None

        dx, dy = self._pixel_to_xy(u, v, target_alt)

        if not self.kf_ref_set:
            self.kf_ref_lat = self.latitude
            self.kf_ref_lon = self.longitude
            self.kf_ref_set = True
            self.kf_X[0, 0] = dx
            self.kf_X[1, 0] = dy
            self.kf_initialized = True
            self.last_kf_update_time = self.get_clock().now()
            return self._xy_to_latlon(self.kf_X[0, 0], self.kf_X[1, 0])

        Z = np.array([[dx], [dy]])

        # 卡尔曼更新
        S = self.kf_H @ self.kf_P @ self.kf_H.T + self.kf_R
        try:
            K = self.kf_P @ self.kf_H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warn("S矩阵奇异，跳过更新")
            return self._xy_to_latlon(self.kf_X[0, 0], self.kf_X[1, 0])

        y = Z - self.kf_H @ self.kf_X
        self.kf_X = self.kf_X + K @ y
        self.kf_P = (np.eye(4) - K @ self.kf_H) @ self.kf_P

        return self._xy_to_latlon(self.kf_X[0, 0], self.kf_X[1, 0])

    def _xy_to_latlon(self, x, y):
        if self.kf_ref_lat is None or self.kf_ref_lon is None:
            return 0.0, 0.0

        R = EARTH_R
        lat = self.kf_ref_lat + y / R * 180.0 / math.pi
        lon = self.kf_ref_lon + x / (R * math.cos(math.radians(self.kf_ref_lat))) * 180.0 / math.pi
        return lat, lon


def main(args=None):
    rclpy.init(args=args)
    node = None
    node = CoordinateTransform()
    try:
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
