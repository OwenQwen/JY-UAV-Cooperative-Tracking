#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
import json
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition, VehicleAttitude

EARTH_R = 6378137.0


class TargetPositionFilter:
    def __init__(self, measurement_std, acceleration_std, max_speed):
        self.measurement_variance = measurement_std ** 2
        self.acceleration_variance = acceleration_std ** 2
        self.max_speed = max_speed
        self.reference_latitude = None
        self.reference_longitude = None
        self.state = np.zeros((4, 1))
        self.covariance = self._initial_covariance()
        self.stamp = None
        self.last_measurement_stamp = None

    def _initial_covariance(self):
        return np.diag([
            self.measurement_variance,
            self.measurement_variance,
            self.max_speed ** 2,
            self.max_speed ** 2,
        ])

    def _initialize(self, latitude, longitude, stamp):
        self.reference_latitude = latitude
        self.reference_longitude = longitude
        self.state = np.zeros((4, 1))
        self.covariance = self._initial_covariance()
        self.stamp = stamp
        self.last_measurement_stamp = stamp

    def _gps_to_xy(self, latitude, longitude):
        latitude_scale = math.pi * EARTH_R / 180.0
        longitude_scale = math.cos(math.radians(self.reference_latitude)) * latitude_scale
        return (
            (longitude - self.reference_longitude) * longitude_scale,
            (latitude - self.reference_latitude) * latitude_scale,
        )

    def _xy_to_gps(self, x, y):
        latitude_scale = math.pi * EARTH_R / 180.0
        longitude_scale = math.cos(math.radians(self.reference_latitude)) * latitude_scale
        return (
            self.reference_latitude + y / latitude_scale,
            self.reference_longitude + x / longitude_scale,
        )

    def _predict(self, dt):
        transition = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        process_noise = self.acceleration_variance * np.array([
            [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
            [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
            [dt3 / 2.0, 0.0, dt2, 0.0],
            [0.0, dt3 / 2.0, 0.0, dt2],
        ])
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process_noise

    def update(self, latitude, longitude, stamp):
        if not all(math.isfinite(value) for value in (latitude, longitude, stamp)):
            return None
        if self.reference_latitude is None:
            self._initialize(latitude, longitude, stamp)
            return self.result()

        dt = stamp - self.stamp
        if stamp - self.last_measurement_stamp > 3.0:
            self._initialize(latitude, longitude, stamp)
            return self.result()
        if dt <= 0.0:
            return None
        self._predict(dt)
        self.stamp = stamp

        measured_x, measured_y = self._gps_to_xy(latitude, longitude)
        measurement = np.array([[measured_x], [measured_y]])
        observation = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ])
        measurement_noise = np.eye(2) * self.measurement_variance
        innovation = measurement - observation @ self.state
        innovation_covariance = observation @ self.covariance @ observation.T + measurement_noise
        mahalanobis_squared = (innovation.T @ np.linalg.solve(innovation_covariance, innovation)).item()
        if mahalanobis_squared > 9.21:
            return None
        gain = self.covariance @ observation.T @ np.linalg.inv(innovation_covariance)
        self.state = self.state + gain @ innovation
        identity = np.eye(4)
        self.covariance = (identity - gain @ observation) @ self.covariance

        speed = math.hypot(self.state[2, 0], self.state[3, 0])
        if speed > self.max_speed:
            scale = self.max_speed / speed
            self.state[2, 0] *= scale
            self.state[3, 0] *= scale
        self.last_measurement_stamp = stamp
        return self.result()

    def result(self):
        latitude, longitude = self._xy_to_gps(self.state[0, 0], self.state[1, 0])
        return {
            "target_latitude": float(latitude),
            "target_longitude": float(longitude),
            "target_velocity_x": float(self.state[2, 0]),
            "target_velocity_y": float(self.state[3, 0]),
        }


class CoordinateTransform(Node):
    def __init__(self):
        super().__init__('coordinate_transform')

        for name, default in (
            ('fx', 539.9), ('fy', 539.9), ('cx', 640.0), ('cy', 480.0),
            ('target_filter_measurement_std', 2.0),
            ('target_filter_acceleration_std', 2.0),
            ('max_target_speed', 7.0),
        ):
            self.declare_parameter(name, default)
            setattr(self, name, float(self.get_parameter(name).value))

        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.rel_altitude = None
        self.current_yaw = 0.0
        self.uav_velocity_x = 0.0
        self.uav_velocity_y = 0.0
        self.target_filters = {}

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        publisher_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.gps_sub = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position',
            self._gps_callback, sensor_qos)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_position_callback, sensor_qos)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._attitude_callback, sensor_qos)
        self.detection_sub = self.create_subscription(
            String, '/target_detections', self._detection_callback, sensor_qos)

        self.target_positions_pub = self.create_publisher(
            String, 'target_positions', publisher_qos)

        self.get_logger().info("coordinate_transform started")

    def _gps_callback(self, msg):
        self.latitude = msg.lat
        self.longitude = msg.lon
        if math.isfinite(msg.alt):
            self.altitude = msg.alt

    def _local_position_callback(self, msg):
        self.rel_altitude = -msg.z
        self.uav_velocity_x = msg.vy   # east
        self.uav_velocity_y = msg.vx   # north

    def _attitude_callback(self, msg):
        q = msg.q
        w, x, y, z = q[0], q[1], q[2], q[3]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _detection_callback(self, msg):
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
                class_id = detection.get('class_id', -1) + 1
                target_alt = 0.0
                stamp = float(data['stamp'])
                self._process_detection(center_x, center_y, target_alt, class_id, stamp)
        except Exception as e:
            self.get_logger().error(f"detection error: {e}")

    def _process_detection(self, u, v, target_alt, class_id, stamp):
        if not self._ready():
            return
        gps_result = self._pixel_to_gps(u, v, target_alt)
        if gps_result is None:
            return
        target_filter = self.target_filters.get(class_id)
        if target_filter is None:
            target_filter = TargetPositionFilter(
                self.target_filter_measurement_std,
                self.target_filter_acceleration_std,
                self.max_target_speed,
            )
            self.target_filters[class_id] = target_filter
        filtered = target_filter.update(gps_result[0], gps_result[1], stamp)
        if filtered is None:
            return
        offset_x, offset_y = self._gps_offset(
            self.latitude, self.longitude,
            filtered["target_latitude"], filtered["target_longitude"],
        )
        self._publish_target_position(class_id, offset_x, offset_y, filtered, stamp)

    def _publish_target_position(self, class_id, offset_x, offset_y, filtered, stamp):
        payload = {
            "target_id": int(class_id),
            "offset_x": float(offset_x),
            "offset_y": float(offset_y),
            "stamp": stamp,
            "target_latitude": filtered["target_latitude"],
            "target_longitude": filtered["target_longitude"],
            "target_velocity_x": filtered["target_velocity_x"],
            "target_velocity_y": filtered["target_velocity_y"],
            "uav_velocity_x": self.uav_velocity_x,
            "uav_velocity_y": self.uav_velocity_y,
        }
        msg = String()
        msg.data = json.dumps(payload, allow_nan=False, separators=(",", ":"))
        self.target_positions_pub.publish(msg)

    def _ready(self):
        return all(v is not None and math.isfinite(v) for v in (
            self.latitude, self.longitude, self.altitude))
    def _pixel_to_xy(self, u, v, target_alt=0.0):
        """
        将像素坐标转换为机体水平面内的偏移（前向、右向）
        相机朝下安装，光轴沿机体 z 轴正方向（NED 下）
        """
        x_norm = (float(u) - self.cx) / self.fx   # 图像右为正
        y_norm = (float(v) - self.cy) / self.fy   # 图像下为正

        # 高度差：无人机高度 - 目标高度
        if self.rel_altitude is not None and math.isfinite(self.rel_altitude):
            dz = self.rel_altitude - float(target_alt)
        else:
            dz = self.altitude - float(target_alt)

        # 机体水平偏移（前向、右向）
        body_forward = -y_norm * dz   # 图像下方对应机体后方 -> 前向为负
        body_right   =  x_norm * dz   # 图像右方对应机体右方

        # 用 yaw 旋转到世界 NED（前向->北，右向->东）
        cyaw = math.cos(self.current_yaw)
        syaw = math.sin(self.current_yaw)
        world_north = body_forward * cyaw - body_right * syaw
        world_east  = body_forward * syaw + body_right * cyaw

        # 返回世界 NED 下的水平偏移（x 北，y 东）
        return world_north, world_east


    def _pixel_to_gps(self, u, v, target_alt=0.0):
        if not self._ready():
            return None
        north_offset, east_offset = self._pixel_to_xy(u, v, target_alt)
        dLat = (north_offset / EARTH_R) * (180.0 / math.pi)
        dLon = (east_offset / (EARTH_R * math.cos(math.radians(self.latitude)))) * (180.0 / math.pi)
        return (self.latitude + dLat, self.longitude + dLon)

    @staticmethod
    def _gps_offset(uav_latitude, uav_longitude, target_latitude, target_longitude):
        mean_latitude = math.radians((uav_latitude + target_latitude) * 0.5)
        offset_x = math.radians(target_longitude - uav_longitude) * EARTH_R * math.cos(mean_latitude)
        offset_y = math.radians(target_latitude - uav_latitude) * EARTH_R
        return offset_x, offset_y


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
