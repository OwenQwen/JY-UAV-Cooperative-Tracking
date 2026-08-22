#!/usr/bin/env python3
"""Bridge referee UDP datagrams and mission reports to ROS 2."""

import json
import math
import socket
import struct

import rclpy
from infer_track_interfaces.msg import (
    RefereeCommand,
    RefereeTx,
    RefereeTxResult,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .referee_protocol import (
    decode_frames,
    decode_set_mode_datagram,
    encode_custom_position,
    encode_uav_position,
    gps_to_local,
    parse_global_position_int,
)

MAX_UDP_PAYLOAD = 65507


class RefereeUdpNode(Node):
    """Own the referee UDP socket and bridge it to ROS 2."""

    def __init__(self):
        super().__init__("referee_udp_node")
        defaults = {
            "bind_address": "0.0.0.0",
            "bind_port": 14552,
            "drone_telemetry_port": 14553,
            "nano_port": 14554,
            "allowed_address": "",
            "poll_period": 0.005,
            "max_datagrams_per_poll": 100,
            "receive_buffer_bytes": 1048576,
            "tx_default_address": "",
            "tx_default_port": 0,
            "nano_forward_address": "127.0.0.1",
            "nano_forward_port": 14555,
            "uav_id": 1,
            "ref_latitude": 0.0,
            "ref_longitude": 0.0,
            "uav_position_hz": 1.0,
            "command_topic": "/referee/command",
            "tx_topic": "/referee/tx",
            "tx_result_topic": "/referee/tx_result",
            "target_report_topic": "/target_report",
            "mavlink_system_id": 1,
            "mavlink_component_id": 1,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)
        self.validate_parameters()

        reliable_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command_pub = self.create_publisher(
            RefereeCommand, self.command_topic, reliable_qos)
        self.tx_result_pub = self.create_publisher(
            RefereeTxResult, self.tx_result_topic, reliable_qos)
        self.tx_sub = self.create_subscription(
            RefereeTx, self.tx_topic, self.tx_callback, reliable_qos)
        self.target_report_sub = self.create_subscription(
            String,
            self.target_report_topic,
            self.target_report_callback,
            reliable_qos,
        )

        self.last_referee_endpoint = None
        self.mavlink_packet_sequence = 0
        self.last_warning_time = {}
        self.drone_connected = False
        self.drone_position = None
        self.udp_socket = self.open_socket(self.bind_port)
        self.drone_telemetry_socket = self.open_socket(
            self.drone_telemetry_port
        )
        self.nano_socket = self.open_socket(self.nano_port)
        self.poll_timer = self.create_timer(
            self.poll_period, self.poll_udp)
        self.uav_position_timer = self.create_timer(
            1.0 / self.uav_position_hz,
            self.send_uav_position,
        )
        self.get_logger().info(
            f"referee UDP bridge listening on {self.bind_address}:"
            f"{self.bind_port}")

    def validate_parameters(self):
        if not isinstance(self.bind_address, str) \
                or not isinstance(self.allowed_address, str) \
                or not isinstance(self.tx_default_address, str) \
                or not isinstance(self.nano_forward_address, str):
            raise ValueError("UDP addresses must be strings")
        for name in (
            "bind_port",
            "drone_telemetry_port",
            "nano_port",
        ):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= 65535:
                raise ValueError(f"{name} must be between 1 and 65535")
        for name in ("tx_default_port", "nano_forward_port"):
            value = getattr(self, name)
            if type(value) is not int or not 0 <= value <= 65535:
                raise ValueError(f"{name} must be between 0 and 65535")
        if isinstance(self.poll_period, bool) \
                or not isinstance(self.poll_period, (int, float)) \
                or self.poll_period <= 0:
            raise ValueError("poll_period must be positive")
        for name in ("max_datagrams_per_poll", "receive_buffer_bytes"):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        for name in ("mavlink_system_id", "mavlink_component_id"):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= 255:
                raise ValueError(f"{name} must be between 1 and 255")
        if type(self.uav_id) is not int or not -128 <= self.uav_id <= 127:
            raise ValueError("uav_id must be between -128 and 127")
        if not -90.0 <= float(self.ref_latitude) <= 90.0:
            raise ValueError("ref_latitude must be between -90 and 90")
        if not -180.0 <= float(self.ref_longitude) <= 180.0:
            raise ValueError("ref_longitude must be between -180 and 180")
        if isinstance(self.uav_position_hz, bool) \
                or not isinstance(self.uav_position_hz, (int, float)) \
                or self.uav_position_hz <= 0:
            raise ValueError("uav_position_hz must be positive")

    def open_socket(self, port):
        try:
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            udp_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_RCVBUF,
                self.receive_buffer_bytes)
            udp_socket.setblocking(False)
            udp_socket.bind((self.bind_address, port))
            return udp_socket
        except OSError as exc:
            if "udp_socket" in locals():
                udp_socket.close()
            raise RuntimeError(
                f"cannot bind UDP {self.bind_address}:{port}: {exc}"
            ) from exc

    def now_msg(self):
        return self.get_clock().now().to_msg()

    def warn_throttle(self, key, period, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self.last_warning_time.get(key, -float("inf"))
        if now < last or now - last >= period:
            self.get_logger().warning(message)
            self.last_warning_time[key] = now

    def poll_udp(self):
        self.poll_socket(
            self.udp_socket,
            self.handle_referee_datagram,
            "referee",
        )
        self.poll_socket(
            self.drone_telemetry_socket,
            self.handle_drone_telemetry,
            "telemetry",
        )
        self.poll_socket(
            self.nano_socket,
            self.handle_nano_datagram,
            "nano",
        )

    def poll_socket(self, udp_socket, handler, channel):
        for _ in range(self.max_datagrams_per_poll):
            try:
                datagram, source = udp_socket.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                self.warn_throttle(
                    f"receive_{channel}",
                    1.0,
                    f"{channel} UDP receive failed: {exc}",
                )
                return
            handler(datagram, source)

    def handle_referee_datagram(self, datagram, source):
        """Publish referee commands and forward their wire frame to Nano."""
        if self.allowed_address and source[0] != self.allowed_address:
            self.warn_throttle(
                "sender", 2.0,
                f"ignored referee datagram from {source[0]}")
            return
        try:
            commands = decode_set_mode_datagram(datagram)
        except ValueError as exc:
            self.warn_throttle(
                "decode", 1.0,
                f"invalid referee datagram from {source}: {exc}")
            return
        if not commands:
            return

        self.last_referee_endpoint = source
        if self.nano_forward_address and self.nano_forward_port:
            if self.allowed_address and source[0] != self.allowed_address:
                return
            try:
                sent = self.nano_socket.sendto(
                    datagram,
                    (self.nano_forward_address, self.nano_forward_port),
                )
                if sent != len(datagram):
                    raise OSError(
                        f"partial Nano send: {sent}/{len(datagram)} bytes"
                    )
            except OSError as exc:
                self.warn_throttle(
                    "nano_forward",
                    1.0,
                    f"Nano command forward failed: {exc}",
                )
        for command in commands:
            self.publish_command(command, source)

    def handle_drone_telemetry(self, datagram, source):
        """Remember the newest valid GLOBAL_POSITION_INT telemetry frame."""
        for message_id, payload in decode_frames(datagram):
            if message_id != 33:
                continue
            try:
                self.drone_position = parse_global_position_int(payload)
            except (struct.error, ValueError) as exc:
                self.warn_throttle(
                    "telemetry_decode", 1.0,
                    f"invalid aircraft telemetry from {source}: {exc}",
                )
                continue
            self.drone_connected = True

    def handle_nano_datagram(self, datagram, source):
        """Convert Nano GPS target reports to local metres and relay them."""
        for message_id, payload in decode_frames(datagram):
            if message_id != 999 or len(payload) != 10:
                continue
            try:
                latitude, longitude, target_id, sequence_id = struct.unpack(
                    "<ffbb", payload
                )
                x, y = gps_to_local(
                    latitude,
                    longitude,
                    self.ref_latitude,
                    self.ref_longitude,
                )
                outgoing = encode_custom_position(
                    x,
                    y,
                    target_id,
                    sequence_id,
                    packet_sequence=self.mavlink_packet_sequence,
                    system_id=self.mavlink_system_id,
                    component_id=self.mavlink_component_id,
                )
                destination = self.resolve_destination()
                sent = self.udp_socket.sendto(outgoing, destination)
                if sent != len(outgoing):
                    raise OSError(
                        f"partial UDP send: {sent}/{len(outgoing)} bytes"
                    )
                self.mavlink_packet_sequence = (
                    self.mavlink_packet_sequence + 1
                ) & 0xFF
            except (OSError, struct.error, ValueError) as exc:
                self.warn_throttle(
                    "nano_report", 1.0,
                    f"invalid Nano report from {source}: {exc}",
                )

    def publish_command(self, command, source):
        msg = RefereeCommand()
        msg.custom_mode = command["custom_mode"]
        msg.target_system = command["target_system"]
        msg.base_mode = command["base_mode"]
        self.command_pub.publish(msg)
        self.get_logger().info(
            "referee command: "
            f"mode={msg.custom_mode}, target={msg.target_system}, "
            f"base={msg.base_mode}, source={source[0]}:{source[1]}")

    def resolve_destination(self, address="", port=0):
        address = address or self.tx_default_address
        port = port or self.tx_default_port
        if self.last_referee_endpoint is not None:
            if not address:
                address = self.last_referee_endpoint[0]
            if port == 0:
                port = self.last_referee_endpoint[1]
        if not address or port == 0:
            raise ValueError(
                "no UDP destination; set it in RefereeTx, configure "
                "tx_default_address/tx_default_port, or receive a command "
                "first"
            )
        return address, port

    @staticmethod
    def parse_target_report(data):
        """Validate one JSON target report and return its wire fields."""
        report = json.loads(data)
        if not isinstance(report, dict):
            raise ValueError("target report must be a JSON object")

        target_id = report.get("target_id")
        sequence_id = report.get("sequence_id")
        if type(target_id) is not int or target_id not in (1, 2):
            raise ValueError("target_id must be 1 or 2")
        if type(sequence_id) is not int or not 0 <= sequence_id <= 16:
            raise ValueError("sequence_id must be between 0 and 16")

        try:
            latitude = float(report["target_latitude"])
            longitude = float(report["target_longitude"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                "target_latitude and target_longitude must be numbers"
            ) from exc
        if not math.isfinite(latitude) or not -90.0 <= latitude <= 90.0:
            raise ValueError("target_latitude must be between -90 and 90")
        if (
            not math.isfinite(longitude)
            or not -180.0 <= longitude <= 180.0
        ):
            raise ValueError("target_longitude must be between -180 and 180")
        return latitude, longitude, target_id, sequence_id

    def target_report_callback(self, msg):
        """Encode a mission target report as MAVLink #999 and send UDP."""
        try:
            latitude, longitude, target_id, sequence_id = (
                self.parse_target_report(msg.data)
            )
            x, y = gps_to_local(
                latitude,
                longitude,
                self.ref_latitude,
                self.ref_longitude,
            )
            payload = encode_custom_position(
                x,
                y,
                target_id,
                sequence_id,
                packet_sequence=self.mavlink_packet_sequence,
                system_id=self.mavlink_system_id,
                component_id=self.mavlink_component_id,
            )
            destination = self.resolve_destination()
            sent = self.udp_socket.sendto(payload, destination)
            if sent != len(payload):
                raise OSError(f"partial UDP send: {sent}/{len(payload)} bytes")
            self.mavlink_packet_sequence = (
                self.mavlink_packet_sequence + 1
            ) & 0xFF
            self.get_logger().info(
                "sent target report: "
                f"target={target_id}, sequence={sequence_id}, "
                f"latitude={latitude:.7f}, longitude={longitude:.7f}, "
                f"destination={destination[0]}:{destination[1]}"
            )
        except (json.JSONDecodeError, OSError, TypeError, ValueError) as exc:
            self.get_logger().error(f"target report UDP send failed: {exc}")

    def send_uav_position(self):
        """Send the latest aircraft position to the active referee endpoint."""
        if not self.drone_connected or self.drone_position is None:
            return
        try:
            position = self.drone_position
            payload = encode_uav_position(
                position["lat"],
                position["lon"],
                position["alt_m"],
                position["vx"],
                position["vy"],
                position["vz"],
                self.uav_id,
                packet_sequence=self.mavlink_packet_sequence,
                system_id=self.mavlink_system_id,
                component_id=self.mavlink_component_id,
            )
            destination = self.resolve_destination()
            sent = self.udp_socket.sendto(payload, destination)
            if sent != len(payload):
                raise OSError(f"partial UDP send: {sent}/{len(payload)} bytes")
            self.mavlink_packet_sequence = (
                self.mavlink_packet_sequence + 1
            ) & 0xFF
        except (OSError, TypeError, ValueError) as exc:
            self.warn_throttle(
                "uav_position", 1.0,
                f"aircraft position UDP send failed: {exc}",
            )

    def tx_callback(self, msg):
        result = RefereeTxResult()
        result.stamp = self.now_msg()
        result.request_id = msg.request_id
        try:
            payload = bytes(msg.payload)
            if not payload:
                raise ValueError("UDP payload must not be empty")
            if len(payload) > MAX_UDP_PAYLOAD:
                raise ValueError(
                    f"UDP payload exceeds {MAX_UDP_PAYLOAD} bytes")
            destination = self.resolve_destination(
                msg.destination_address,
                msg.destination_port,
            )
            result.destination_address = destination[0]
            result.destination_port = destination[1]
            sent = self.udp_socket.sendto(payload, destination)
            if sent != len(payload):
                raise OSError(f"partial UDP send: {sent}/{len(payload)} bytes")
            result.success = True
            result.bytes_sent = sent
            self.get_logger().info(
                f"sent referee UDP request_id={msg.request_id}, "
                f"bytes={sent}, destination={destination[0]}:{destination[1]}")
        except (OSError, TypeError, ValueError) as exc:
            result.success = False
            result.error = str(exc)
            self.get_logger().error(
                f"referee UDP send request_id={msg.request_id} failed: {exc}")
        self.tx_result_pub.publish(result)

    def destroy_node(self):
        for name in (
            "udp_socket",
            "drone_telemetry_socket",
            "nano_socket",
        ):
            udp_socket = getattr(self, name, None)
            if udp_socket is not None:
                udp_socket.close()
                setattr(self, name, None)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RefereeUdpNode()
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
