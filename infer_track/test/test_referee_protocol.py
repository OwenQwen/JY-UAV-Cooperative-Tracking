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

"""Tests for the MAVLink 2.0 referee wire protocol (no ROS needed)."""

import math
import importlib.util
import os
import struct
import sys
import time

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from infer_track.referee_protocol import (  # noqa: E402
    METER_PER_DEG,
    crc16_mcrf4xx,
    decode_frames,
    decode_set_mode_datagram,
    encode_custom_position,
    encode_uav_position,
    gps_to_local,
    mavlink_pack,
    make_target_position_payload,
    parse_global_position_int,
)

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


def ref_crc(data):
    """main.py's crc16_mcrf4xx, kept as an independent reference."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def ref_pack(msgid, sysid, compid, seq, payload):
    """main.py's mavlink_pack, kept as an independent reference."""
    crc_extra = {11: 89, 33: 104, 999: 28, 1000: 14}[msgid]
    header = struct.pack(
        "<BBBBBBBB", 0xFD, len(payload), 0, 0,
        seq & 0xFF, sysid & 0xFF, compid & 0xFF, msgid & 0xFF)
    header += struct.pack("<H", (msgid >> 8) & 0xFFFF)
    crc_data = header[1:] + payload + bytes([crc_extra])
    return header + payload + struct.pack("<H", ref_crc(crc_data))


def ref_nano_to_local(raw_x, raw_y, ref_lat, ref_lon):
    """main.py's nano_to_local, kept as an independent reference."""
    x = (raw_y - ref_lon) * math.cos(
        raw_x * math.pi / 180.0) * METER_PER_DEG
    y = (raw_x - ref_lat) * METER_PER_DEG
    return x, y


def test_crc_matches_mcrf4xx_reference():
    samples = [b"", b"\x01", b"hello", bytes(range(256))]
    for data in samples:
        assert crc16_mcrf4xx(data) == ref_crc(data)


def test_mavlink_pack_matches_main_py():
    for msgid, payload in (
        (11, struct.pack("<IBB", 2, 1, 1)),
        (999, make_target_position_payload(9.03, 12.12, 1, 0)),
        (33, struct.pack("<IiiiiHHHH", 1, 2, 3, 4, 5, 6, 7, 8, 9)),
    ):
        ours = mavlink_pack(msgid, 1, 1, 7, payload)
        assert ours == ref_pack(msgid, 1, 1, 7, payload)


def test_encode_custom_position_matches_main_py():
    ours = encode_custom_position(
        9.03, 12.12, 1, 0, packet_sequence=7)
    ref = ref_pack(
        999, 1, 1, 7, make_target_position_payload(9.03, 12.12, 1, 0))
    assert ours == ref


def test_encode_uav_position_matches_main_py(monkeypatch=None):
    original_time = time.time

    def fake_time():
        return 12345.678

    time.time = fake_time
    try:
        ours = encode_uav_position(
            20.0894, 125.0916, 12.5, 100, -200, 300, 1,
            packet_sequence=9)
        time_boot_ms = int((fake_time() % 86400) * 1000)
        payload = struct.pack(
            "<Iiifhhhb", time_boot_ms, int(20.0894e7), int(125.0916e7),
            1250.0, 100, -200, 300, 1)
        ref = ref_pack(1000, 1, 1, 9, payload)
        assert ours == ref
    finally:
        time.time = original_time


def test_set_mode_roundtrip():
    frame = mavlink_pack(11, 1, 1, 3, struct.pack("<IBB", 2, 1, 1))
    assert decode_set_mode_datagram(frame) == [{
        "custom_mode": 2,
        "target_system": 1,
        "base_mode": 1,
    }]


def test_multi_frame_and_garbage_tolerance():
    frame = mavlink_pack(11, 1, 1, 3, struct.pack("<IBB", 2, 1, 1))
    junk = b"\x00\x01garbage" + frame + b"\xfd\x06\x00\x00\x00\x01\x01"
    assert decode_set_mode_datagram(junk) == [{
        "custom_mode": 2,
        "target_system": 1,
        "base_mode": 1,
    }]


def test_truncated_and_bad_crc_frames_skipped():
    good = mavlink_pack(11, 1, 1, 3, struct.pack("<IBB", 2, 1, 1))
    bad_crc = good[:-2] + b"\x00\x00"
    truncated = good[:12]
    assert decode_set_mode_datagram(bad_crc + good) == [{
        "custom_mode": 2,
        "target_system": 1,
        "base_mode": 1,
    }]
    assert decode_set_mode_datagram(truncated) == []


def test_unknown_msgid_skipped():
    frame = bytes((0xFD, 4, 0, 0, 1, 1, 1, 66, 0, 0))
    frame += b"\x00" * 4 + b"\x00\x00"
    assert list(decode_frames(frame)) == []


def test_mavlink_v1_and_raw_payload_ignored():
    payload = struct.pack("<IBB", 2, 1, 1)
    v1_header = bytes([0xFE, 6, 1, 1, 11])
    v1_crc = ref_crc(v1_header[1:] + payload + bytes([89]))
    v1_frame = v1_header + payload + struct.pack("<H", v1_crc)
    assert decode_set_mode_datagram(v1_frame) == []
    assert decode_set_mode_datagram(payload) == []


def test_parse_global_position_int():
    payload = struct.pack(
        "<IiiiiHHHH", 1000, int(20.0894e7), int(125.0916e7),
        12000, 10000, 100, 200, 50, 18000)
    position = parse_global_position_int(payload)
    assert position["lat"] == 20.0894
    assert position["lon"] == 125.0916
    assert position["alt_m"] == 10.0
    assert position["alt_msl_m"] == 12.0
    assert position["vx"] == 100
    assert position["heading"] == 180.0


def test_gps_to_local_matches_main_py():
    ref_lat, ref_lon = 20.0894, 125.0916
    lat, lon = 20.0912, 125.0951
    ours = gps_to_local(lat, lon, ref_lat, ref_lon)
    ref = ref_nano_to_local(lat, lon, ref_lat, ref_lon)
    assert tuple(round(v, 9) for v in ours) == \
        tuple(round(v, 9) for v in ref)


def test_non_bytes_datagram_raises():
    try:
        decode_set_mode_datagram("not bytes")
    except ValueError:
        return
    raise AssertionError("expected ValueError for non-bytes datagram")


def test_interop_with_test_one_crc_and_pack():
    if test_one is None:
        return
    for data in (b"", b"\x01", b"hello", bytes(range(256))):
        assert crc16_mcrf4xx(data) == test_one.crc16(data)
    payload = struct.pack("<ffbb", 20.0895, 125.0917, 1, 0)
    ours = mavlink_pack(999, 1, 1, 3, payload)
    assert ours == test_one.pack_mavlink(999, 1, 1, 3, payload)


def test_interop_with_test_one_set_mode_both_ways():
    if test_one is None:
        return
    payload = struct.pack("<IBB", 2, 1, 1)
    theirs = test_one.pack_mavlink(11, 1, 1, 0, payload)
    assert decode_set_mode_datagram(theirs) == [{
        "custom_mode": 2,
        "target_system": 1,
        "base_mode": 1,
    }]
    ours = mavlink_pack(11, 1, 1, 0, payload)
    state = {"step": 0}
    assert test_one.parse_once(ours, state) == (2, 1, 1)


def test_conversion_matches_judge_recorded_coordinates():
    """Nano TARGET1 must convert to the judge's logged 10.20/11.17 m."""
    if test_one is None:
        return
    payload = struct.pack("<ffbb", *test_one.TARGET1, 1, 0)
    frame = test_one.pack_mavlink(999, 1, 1, 1, payload)
    (message_id, body), = decode_frames(frame)
    assert message_id == 999
    raw_x, raw_y, target_id, sequence_id = struct.unpack("<ffbb", body)
    local_x, local_y = gps_to_local(
        raw_x, raw_y, 20.0894, 125.0916)
    assert target_id == 1
    assert sequence_id == 0
    assert abs(local_x - 10.20201969) < 0.01
    assert abs(local_y - 11.16699982) < 0.01


if __name__ == "__main__":
    tests = [
        test_crc_matches_mcrf4xx_reference,
        test_mavlink_pack_matches_main_py,
        test_encode_custom_position_matches_main_py,
        test_encode_uav_position_matches_main_py,
        test_set_mode_roundtrip,
        test_multi_frame_and_garbage_tolerance,
        test_truncated_and_bad_crc_frames_skipped,
        test_unknown_msgid_skipped,
        test_mavlink_v1_and_raw_payload_ignored,
        test_parse_global_position_int,
        test_gps_to_local_matches_main_py,
        test_non_bytes_datagram_raises,
        test_interop_with_test_one_crc_and_pack,
        test_interop_with_test_one_set_mode_both_ways,
        test_conversion_matches_judge_recorded_coordinates,
    ]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("all protocol tests passed")
