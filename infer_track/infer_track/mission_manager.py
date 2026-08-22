#!/usr/bin/env python3
"""ROS 2 双目标搜索与移动跟踪任务管理节点."""

import json

import rclpy
from infer_track_interfaces.msg import RefereeCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

try:
    from .mission_state_machine import MissionStateMachine, STATE_NAMES
except ImportError:
    from mission_state_machine import MissionStateMachine, STATE_NAMES


DEFAULTS = {
    "confirm_max_spread": 5.0,
    "confirm_min_count": 5,
    "confirm_min_duration": 0.4,
    "confirm_hover_distance": 5.0,
    "hover_confirm_min_duration": 0.4,
    "target_confirm_count": 5,
    "target_confirm_min_duration": 0.4,
    "target_timeout": 1.5,
    "track_lost_timeout": 3.0,
    "gps_timeout": 0.5,
    "history_capacity": 50,
    "prediction_window": 2.0,  # 只使用最近2秒的目标GPS。
    "max_target_speed": 7.0,
    "max_extrapolation": 3.0,  # 最外推3秒。
    "prediction_start_delay": 0.2,
    "prediction_stop_delay": 3.0,
    "moving_report_period": 20.0,
    "moving_report_count": 16,
    "update_period": 0.1,
    "tracking_command_topic": "/tracking_command",
    "navigation_command_topic": "/navigation_command",
    "target_report_topic": "/target_report",
    "referee_command_topic": "/referee/command",
    "target_positions_topic": "target_positions",
    "uav_gps_topic": "/fmu/out/vehicle_global_position",
}


def json_message(data):
    return String(data=json.dumps(
        data, allow_nan=False, separators=(",", ":")
    ))


class MissionManagerNode(Node):
    """连接裁判、定位、任务状态机和控制话题."""

    def __init__(self):
        super().__init__("mission_manager")
        values = {}
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
            values[name] = self.get_parameter(name).value

        reliable_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE)
        sensor_qos = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        report_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tracking_pub = self.create_publisher(
            String,
            values["tracking_command_topic"],
            reliable_qos,
        )
        # 向控制节点发送任务级飞行动作。
        self.navigation_pub = self.create_publisher(
            String,
            values["navigation_command_topic"],
            reliable_qos,
        )
        # 把确认后的目标GPS发给裁判UDP节点。
        self.report_pub = self.create_publisher(
            String,
            values["target_report_topic"],
            report_qos,
        )
        # 裁判指令。
        self.referee_sub = self.create_subscription(
            RefereeCommand,
            values["referee_command_topic"],
            self.referee_callback,
            reliable_qos,
        )
        # 目标位置。
        self.target_sub = self.create_subscription(
            String,
            values["target_positions_topic"],
            self.target_callback,
            sensor_qos,
        )
        # 无人机GPS。
        self.gps_sub = self.create_subscription(
            NavSatFix,
            values["uav_gps_topic"],
            self.gps_callback,
            sensor_qos,
        )

        self.machine = MissionStateMachine(
            values,
            self.publish_tracking,
            self.publish_navigation,
            self.publish_report,
            self.log_transition,
        )
        self.machine.start(self.now_sec())
        self.timer = self.create_timer(values["update_period"], self.update)
        self.get_logger().info("mission_manager started")

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def referee_callback(self, msg):
        """Handle one referee command."""
        self.machine.handle_referee(
            int(msg.custom_mode),
            int(msg.target_system),
            int(msg.base_mode),
            self.now_sec(),
        )

    def target_callback(self, msg):
        self.machine.receive_target(
            json.loads(msg.data),
            self.now_sec(),
        )

    def gps_callback(self, msg):
        stamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        self.machine.receive_uav_gps(
            float(msg.latitude),
            float(msg.longitude),
            stamp,
        )

    def update(self):
        self.machine.update(self.now_sec())

    def publish_tracking(self, command):
        self.tracking_pub.publish(json_message(command))

    def publish_navigation(self, command):
        self.navigation_pub.publish(json_message(command))

    def publish_report(self, report):
        self.report_pub.publish(json_message(report))

    def log_transition(self, old_state, new_state):
        self.get_logger().info(
            f"{STATE_NAMES[old_state]} -> {STATE_NAMES[new_state]}"
        )

    def stop(self):
        self.machine.stop(self.now_sec())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionManagerNode()
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
