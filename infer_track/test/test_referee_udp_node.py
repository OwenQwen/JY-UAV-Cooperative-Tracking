# Copyright 2026 csz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Behavioral tests for RefereeUdpNode using real UDP loopback sockets.

ROS 2 is not required: rclpy and the message interfaces are replaced by
small fakes before the node is imported, so these tests run on any Python.
"""

import json
import importlib.util
import os
import socket
import struct
import sys
import time
import types
from types import SimpleNamespace

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from infer_track.referee_protocol import (  # noqa: E402
    decode_frames,
    gps_to_local,
    mavlink_pack,
    make_target_position_payload,
)

REF_LAT = 20.0894
REF_LON = 125.0916

TEST_ONE_PATHS = [
    path for path in (os.getenv("REFEREE_REFERENCE_PATH"),) if path
]


def _load_test_one():
    """Import the organizer-provided Nano reference program, if present."""
    for path in TEST_ONE_PATHS:
        if not os.path.isfile(path):
            continue
        spec = importlib.util.spec_from_file_location("test_one", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    return None


test_one = _load_test_one()


# ------------------------- fake ROS 2 layer -------------------------

class FakePub:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeSub:
    def __init__(self, callback):
        self.callback = callback

    def trigger(self, message):
        self.callback(message)


class FakeLogger:
    def __init__(self):
        self.records = []

    def _record(self, level, message):
        self.records.append((level, message))

    def info(self, message):
        self._record("info", message)

    def warning(self, message):
        self._record("warning", message)

    def error(self, message):
        self._record("error", message)


class FakeTime:
    def __init__(self):
        self.nanoseconds = 0

    def to_msg(self):
        return SimpleNamespace(sec=0, nanosec=0)


class FakeClock:
    def now(self):
        return FakeTime()


class FakeNode:
    OVERRIDES = {}

    def __init__(self, name):
        self.name = name
        self.logger = FakeLogger()
        self.params = {}
        self.publishers = []
        self.subscriptions = []
        self.timers = []

    def declare_parameter(self, name, default):
        self.params[name] = self.OVERRIDES.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePub()
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = FakeSub(callback)
        self.subscriptions.append(subscription)
        return subscription

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return SimpleNamespace()

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return FakeClock()

    def destroy_node(self):
        return None


class RefereeCommand:
    def __init__(self):
        self.custom_mode = 0
        self.target_system = 0
        self.base_mode = 0


class RefereeTx:
    def __init__(self):
        self.request_id = 0
        self.payload = []
        self.destination_address = ""
        self.destination_port = 0


class RefereeTxResult:
    def __init__(self):
        self.stamp = None
        self.request_id = 0
        self.destination_address = ""
        self.destination_port = 0
        self.success = False
        self.bytes_sent = 0
        self.error = ""


class String:
    def __init__(self, data=""):
        self.data = data


def install_fake_ros():
    if "rclpy" in sys.modules:
        return

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *args, **kwargs: None
    rclpy.ok = lambda: False
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.shutdown = lambda *args, **kwargs: None

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kwargs: kwargs
    rclpy_qos.DurabilityPolicy = SimpleNamespace(
        VOLATILE="volatile", TRANSIENT_LOCAL="transient_local")
    rclpy_qos.ReliabilityPolicy = SimpleNamespace(
        RELIABLE="reliable", BEST_EFFORT="best_effort")

    interfaces = types.ModuleType("infer_track_interfaces")
    interfaces_msg = types.ModuleType("infer_track_interfaces.msg")
    interfaces_msg.RefereeCommand = RefereeCommand
    interfaces_msg.RefereeTx = RefereeTx
    interfaces_msg.RefereeTxResult = RefereeTxResult
    interfaces.msg = interfaces_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg

    sys.modules.update({
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "infer_track_interfaces": interfaces,
        "infer_track_interfaces.msg": interfaces_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    })


# ------------------------- helpers -------------------------

def make_udp(bind_port=None):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.setblocking(False)
    if bind_port is not None:
        udp_socket.bind(("127.0.0.1", bind_port))
    return udp_socket


def recv_until(udp_socket, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            return udp_socket.recvfrom(65535)
        except BlockingIOError:
            time.sleep(0.01)
    raise AssertionError("timed out waiting for UDP datagram")


def make_node():
    install_fake_ros()
    from infer_track.referee_udp_node import RefereeUdpNode
    FakeNode.OVERRIDES = {
        "bind_address": "127.0.0.1",
        "bind_port": 25550,
        "drone_telemetry_port": 25551,
        "nano_port": 25554,
        "nano_forward_address": "127.0.0.1",
        "nano_forward_port": 25552,
        "tx_default_address": "",
        "tx_default_port": 0,
        "uav_id": 1,
        "ref_latitude": REF_LAT,
        "ref_longitude": REF_LON,
        "uav_position_hz": 1.0,
    }
    return RefereeUdpNode()


def set_mode_frame(custom_mode, target_system, base_mode):
    return mavlink_pack(
        11, 1, 1, 0,
        struct.pack("<IBB", custom_mode, target_system, base_mode))


def gps_position_frame(lat, lon, target_id, sequence_id):
    payload = make_target_position_payload(
        lat, lon, target_id, sequence_id)
    if test_one is not None:
        return test_one.pack_mavlink(999, 1, 1, 5, payload)
    return mavlink_pack(999, 1, 1, 5, payload)


def telemetry_frame(lat, lon):
    payload = struct.pack(
        "<IiiiiHHHH", 1000, int(lat * 1e7), int(lon * 1e7),
        12000, 10000, 100, 200, 50, 18000)
    return mavlink_pack(33, 1, 1, 2, payload)


# ------------------------- tests -------------------------

def test_set_mode_published_and_forwarded_to_nano():
    node = make_node()
    judge = make_udp(25560)
    nano = make_udp(25552)
    try:
        frame = set_mode_frame(2, 1, 1)
        judge.sendto(frame, ("127.0.0.1", node.bind_port))
        node.poll_udp()

        command = node.command_pub.messages[-1]
        assert command.custom_mode == 2
        assert command.target_system == 1
        assert command.base_mode == 1

        forwarded, source = recv_until(nano)
        assert forwarded == frame
        assert source[0] == "127.0.0.1"
    finally:
        node.destroy_node()
        judge.close()
        nano.close()


def test_nano_999_relayed_to_judge():
    node = make_node()
    judge = make_udp(25561)
    nano = make_udp(25563)
    try:
        judge.sendto(
            set_mode_frame(1, 1, 1), ("127.0.0.1", node.bind_port))
        node.poll_udp()

        gps_lat, gps_lon = 20.0912, 125.0951
        sent_payload = make_target_position_payload(gps_lat, gps_lon, 2, 3)
        sent_gps_x, sent_gps_y, _, _ = struct.unpack("<ffbb", sent_payload)
        nano.sendto(
            gps_position_frame(gps_lat, gps_lon, 2, 3),
            ("127.0.0.1", node.nano_port))
        node.poll_udp()

        data, _ = recv_until(judge)
        frames = list(decode_frames(data))
        assert len(frames) == 1
        assert frames[0][0] == 999
        raw_x, raw_y, target_id, sequence_id = struct.unpack(
            "<ffbb", frames[0][1])
        expected_x, expected_y = gps_to_local(
            sent_gps_x, sent_gps_y, REF_LAT, REF_LON)
        assert abs(raw_x - expected_x) < 0.01
        assert abs(raw_y - expected_y) < 0.01
        assert target_id == 2
        assert sequence_id == 3
    finally:
        node.destroy_node()
        judge.close()
        nano.close()


def test_telemetry_feeds_uav_position_report():
    node = make_node()
    judge = make_udp(25564)
    telemetry = make_udp(25565)
    try:
        judge.sendto(
            set_mode_frame(1, 1, 1), ("127.0.0.1", node.bind_port))
        node.poll_udp()

        telemetry.sendto(
            telemetry_frame(20.0894, 125.0916),
            ("127.0.0.1", node.drone_telemetry_port))
        node.poll_udp()
        assert node.drone_connected

        node.send_uav_position()
        data, _ = recv_until(judge)
        frames = list(decode_frames(data))
        assert frames[0][0] == 1000
        fields = struct.unpack("<Iiifhhhb", frames[0][1])
        assert fields[1] == int(20.0894e7)
        assert fields[2] == int(125.0916e7)
        assert fields[7] == 1  # uav_id
    finally:
        node.destroy_node()
        judge.close()
        telemetry.close()


def test_target_report_json_to_999():
    node = make_node()
    judge = make_udp(25566)
    try:
        judge.sendto(
            set_mode_frame(1, 1, 1), ("127.0.0.1", node.bind_port))
        node.poll_udp()

        report = json.dumps({
            "target_id": 1,
            "sequence_id": 4,
            "target_latitude": 20.0912,
            "target_longitude": 125.0951,
        })
        node.target_report_sub.trigger(String(data=report))

        data, _ = recv_until(judge)
        frames = list(decode_frames(data))
        assert frames[0][0] == 999
        raw_x, raw_y, target_id, sequence_id = struct.unpack(
            "<ffbb", frames[0][1])
        expected_x, expected_y = gps_to_local(
            20.0912, 125.0951, REF_LAT, REF_LON)
        assert abs(raw_x - expected_x) < 0.01
        assert abs(raw_y - expected_y) < 0.01
        assert target_id == 1
        assert sequence_id == 4
    finally:
        node.destroy_node()
        judge.close()


def test_tx_callback_forwards_payload_and_reports_result():
    node = make_node()
    receiver = make_udp(25567)
    try:
        tx = RefereeTx()
        tx.request_id = 7
        tx.payload = list(b"hello")
        tx.destination_address = "127.0.0.1"
        tx.destination_port = 25567
        node.tx_sub.trigger(tx)

        data, _ = recv_until(receiver)
        assert data == b"hello"

        result = node.tx_result_pub.messages[-1]
        assert result.request_id == 7
        assert result.success is True
        assert result.bytes_sent == 5
        assert result.destination_address == "127.0.0.1"
        assert result.destination_port == 25567
    finally:
        node.destroy_node()
        receiver.close()


if __name__ == "__main__":
    tests = [
        test_set_mode_published_and_forwarded_to_nano,
        test_nano_999_relayed_to_judge,
        test_telemetry_feeds_uav_position_report,
        test_target_report_json_to_999,
        test_tx_callback_forwards_payload_and_reports_result,
    ]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("all node tests passed")
