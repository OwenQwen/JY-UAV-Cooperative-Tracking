
#!/usr/bin/env python3
"""把当前目标的定位偏移转发给飞行控制节点."""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition


class TrackingGate:
    """保存跟踪开关，并只放行指定车辆的有效偏移."""

    def __init__(self, prediction_lead=0.1, prediction_start_delay=0.2,
                 prediction_stop_delay=3.0):
        self.prediction_lead = prediction_lead
        self.prediction_start_delay = prediction_start_delay
        self.prediction_stop_delay = prediction_stop_delay
        self.requested_enable = False
        self.active_target_id = None
        self.last_target_time = None
        self.last_target = None
        self.control_active = False
        self.uav_velocity_x = 0.0
        self.uav_velocity_y = 0.0
        self.has_direct_uav_velocity = False
        self.uav_displacement_x = 0.0
        self.uav_displacement_y = 0.0
        self.uav_integration_time = None

    @staticmethod
    def stop_message(target_id, now):
        return {
            "enable": False,
            "target_id": target_id,
            "offset_x": 0.0,
            "offset_y": 0.0,
            "stamp": now,
        }

    def predicted_message(self, target, now):
        target_stamp = float(target["stamp"])
        prediction_time = min(
            now + self.prediction_lead,
            target_stamp + self.prediction_stop_delay,
        )
        target_dt = prediction_time - target_stamp
        uav_dt = prediction_time - self.uav_integration_time
        uav_displacement_x = (
            self.uav_displacement_x + self.uav_velocity_x * uav_dt
        )
        uav_displacement_y = (
            self.uav_displacement_y + self.uav_velocity_y * uav_dt
        )
        return {
            "enable": True,
            "target_id": self.active_target_id,
            "offset_x": (
                float(target["offset_x"])
                + float(target.get("target_velocity_x", 0.0)) * target_dt
                - uav_displacement_x
            ),
            "offset_y": (
                float(target["offset_y"])
                + float(target.get("target_velocity_y", 0.0)) * target_dt
                - uav_displacement_y
            ),
            "stamp": prediction_time,
        }

    def update_uav_velocity(self, velocity_x, velocity_y, now):
        if not all(math.isfinite(float(value)) for value in (
            velocity_x, velocity_y, now,
        )):
            return
        if self.uav_integration_time is not None:
            dt = now - self.uav_integration_time
            if dt > 0.0:
                self.uav_displacement_x += self.uav_velocity_x * dt
                self.uav_displacement_y += self.uav_velocity_y * dt
                self.uav_integration_time = now
        self.uav_velocity_x = float(velocity_x)
        self.uav_velocity_y = float(velocity_y)
        self.has_direct_uav_velocity = True

    def _timeout_stop(self, now):
        target_id = self.active_target_id
        self.requested_enable = False
        self.control_active = False
        self.last_target_time = None
        self.last_target = None
        self.uav_integration_time = None
        return self.stop_message(target_id, now)

    def command(self, command, now):
        enable = bool(command["enable"])
        target_id = int(command["target_id"])
        messages = []

        if enable:
            if self.control_active and self.active_target_id != target_id:
                messages.append(self.stop_message(self.active_target_id, now))
            if self.active_target_id != target_id:
                self.last_target_time = None
                self.last_target = None
                self.control_active = False
                self.uav_integration_time = None
            self.requested_enable = True
            self.active_target_id = target_id
            return messages

        if self.control_active:
            messages.append(self.stop_message(self.active_target_id, now))
        self.requested_enable = False
        self.active_target_id = None
        self.last_target_time = None
        self.last_target = None
        self.control_active = False
        self.uav_integration_time = None
        return messages

    def target(self, target, now):
        if not self.requested_enable or int(target["target_id"]) != self.active_target_id:
            return None
        target_stamp = float(target["stamp"])
        if (self.control_active and self.last_target_time is not None
                and target_stamp - self.last_target_time > self.prediction_stop_delay):
            return self._timeout_stop(now)
        self.last_target_time = target_stamp
        self.last_target = dict(target)
        source_uav_velocity_x = float(target.get("uav_velocity_x", 0.0))
        source_uav_velocity_y = float(target.get("uav_velocity_y", 0.0))
        receive_delay = now - self.last_target_time
        self.uav_displacement_x = source_uav_velocity_x * receive_delay
        self.uav_displacement_y = source_uav_velocity_y * receive_delay
        self.uav_integration_time = now
        if not self.has_direct_uav_velocity:
            self.uav_velocity_x = source_uav_velocity_x
            self.uav_velocity_y = source_uav_velocity_y
        self.control_active = True
        return self.predicted_message(target, now)

    def update(self, now):
        if not self.requested_enable or not self.control_active or self.last_target is None:
            return None
        lost_duration = now - self.last_target_time
        if lost_duration + 1e-9 < self.prediction_start_delay:
            return None
        if lost_duration <= self.prediction_stop_delay:
            return self.predicted_message(self.last_target, now)
        return self._timeout_stop(now)

    def stop(self, now):
        if not self.control_active:
            return None
        self.control_active = False
        return self.stop_message(self.active_target_id, now)


def json_message(data):
    return String(data=json.dumps(data, allow_nan=False, separators=(",", ":")))


class TrackingNode(Node):
    def __init__(self):
        super().__init__("tracking_node")
        defaults = {
            "update_period": 0.1,
            "prediction_lead": 0.1,
            "prediction_start_delay": 0.2,
            "prediction_stop_delay": 3.0,
            "tracking_command_topic": "/tracking_command",
            "target_positions_topic": "/target_positions",
            "control_offset_topic": "/control_target_offset",
            "uav_velocity_topic": "/fmu/out/vehicle_local_position",
        }
        values = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            values[name] = self.get_parameter(name).value

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.control_pub = self.create_publisher(
            String, values["control_offset_topic"], reliable_qos)
        self.command_sub = self.create_subscription(
            String, values["tracking_command_topic"], self.command_callback, reliable_qos)
        self.target_sub = self.create_subscription(
            String, values["target_positions_topic"], self.target_callback, sensor_qos)
        self.gate = TrackingGate(
            values["prediction_lead"],
            values["prediction_start_delay"],
            values["prediction_stop_delay"],
        )
        self.velocity_sub = self.create_subscription(
            VehicleLocalPosition,
            values["uav_velocity_topic"],
            self.velocity_callback,
            sensor_qos,
        )
        self.timer = self.create_timer(values["update_period"], self.update)
        self.get_logger().info("tracking_node started")

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_commands(self, commands):
        for command in commands:
            self.control_pub.publish(json_message(command))

    def command_callback(self, msg):
        commands = self.gate.command(json.loads(msg.data), self.now_sec())
        self.publish_commands(commands)

    def target_callback(self, msg):
        command = self.gate.target(json.loads(msg.data), self.now_sec())
        if command is not None:
            self.control_pub.publish(json_message(command))

    def velocity_callback(self, msg):
        self.gate.update_uav_velocity(
            msg.vy,   # east
            msg.vx,   # north
            self.now_sec(),
        )

    def update(self):
        command = self.gate.update(self.now_sec())
        if command is not None:
            self.control_pub.publish(json_message(command))

    def stop(self):
        command = self.gate.stop(self.now_sec())
        if command is not None:
            self.control_pub.publish(json_message(command))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrackingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
