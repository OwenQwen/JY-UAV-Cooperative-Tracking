"""Small, dependency-free MAVLink helpers for the referee UDP bridge.

Only the message IDs used by this project are accepted.  Malformed, unknown,
or incomplete frames are ignored so one bad UDP datagram cannot stop the
control process.
"""

import math
import struct
import time


MAVLINK_V2_MAGIC = 0xFD
MAVLINK_V2_SIGNED_FLAG = 0x01
MAVLINK_SET_MODE_MSG_ID = 11
MAVLINK_GLOBAL_POSITION_INT_MSG_ID = 33
MAVLINK_CUSTOM_POSITION_MSG_ID = 999
MAVLINK_UAV_POSITION_MSG_ID = 1000

MAVLINK_SET_MODE_PAYLOAD_LENGTH = 6
MAVLINK_CUSTOM_POSITION_PAYLOAD_LENGTH = 10

CRC_EXTRAS = {
    MAVLINK_SET_MODE_MSG_ID: 89,
    MAVLINK_GLOBAL_POSITION_INT_MSG_ID: 104,
    MAVLINK_CUSTOM_POSITION_MSG_ID: 28,
    MAVLINK_UAV_POSITION_MSG_ID: 14,
}

METER_PER_DEG = 111_111.0


def crc16_mcrf4xx(data, crc=0xFFFF):
    """Calculate the MAVLink X.25/MCRF4XX checksum."""
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (
            (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        ) & 0xFFFF
    return crc


def mavlink_crc(data, crc=0xFFFF):
    """Backward-compatible name for :func:`crc16_mcrf4xx`."""
    return crc16_mcrf4xx(data, crc)


def mavlink_pack(
    message_id,
    system_id,
    component_id,
    sequence,
    payload,
):
    """Pack one unsigned MAVLink 2 frame for a supported message ID."""
    if not isinstance(payload, bytes):
        raise ValueError("MAVLink payload must be bytes")
    if message_id not in CRC_EXTRAS:
        raise ValueError(f"unsupported MAVLink message ID: {message_id}")
    if len(payload) > 255:
        raise ValueError("MAVLink payload must not exceed 255 bytes")

    header = bytes((
        len(payload),
        0,
        0,
        sequence & 0xFF,
        system_id & 0xFF,
        component_id & 0xFF,
        message_id & 0xFF,
        (message_id >> 8) & 0xFF,
        (message_id >> 16) & 0xFF,
    ))
    checksum = crc16_mcrf4xx(
        header + payload + bytes((CRC_EXTRAS[message_id],))
    )
    return (
        bytes((MAVLINK_V2_MAGIC,))
        + header
        + payload
        + struct.pack("<H", checksum)
    )


def decode_frames(datagram):
    """Yield validated supported MAVLink 2 ``(message_id, payload)`` pairs.

    The UDP stream may contain unrelated bytes or more than one frame.  Bad
    checksums, unknown message IDs, MAVLink 1 packets and truncated tails are
    skipped instead of being treated as fatal input.
    """
    if not isinstance(datagram, bytes):
        raise ValueError("UDP datagram must be bytes")

    offset = 0
    while offset < len(datagram):
        if datagram[offset] != MAVLINK_V2_MAGIC:
            offset += 1
            continue
        if offset + 10 > len(datagram):
            break

        payload_length = datagram[offset + 1]
        signature_length = (
            13
            if datagram[offset + 2] & MAVLINK_V2_SIGNED_FLAG
            else 0
        )
        frame_length = 10 + payload_length + 2 + signature_length
        frame_end = offset + frame_length
        if frame_end > len(datagram):
            break

        frame = datagram[offset:frame_end]
        message_id = frame[7] | (frame[8] << 8) | (frame[9] << 16)
        payload_end = 10 + payload_length
        crc_extra = CRC_EXTRAS.get(message_id)
        if crc_extra is not None:
            received_crc = struct.unpack_from("<H", frame, payload_end)[0]
            calculated_crc = crc16_mcrf4xx(
                frame[1:payload_end] + bytes((crc_extra,))
            )
            if received_crc == calculated_crc:
                yield message_id, frame[10:payload_end]

        offset = frame_end


def decode_set_mode_payload(payload):
    """Decode the six-byte little-endian SET_MODE payload."""
    if len(payload) != MAVLINK_SET_MODE_PAYLOAD_LENGTH:
        raise ValueError("SET_MODE payload must contain exactly 6 bytes")
    custom_mode, target_system, base_mode = struct.unpack("<IBB", payload)
    return {
        "custom_mode": custom_mode,
        "target_system": target_system,
        "base_mode": base_mode,
    }


def decode_set_mode_datagram(datagram):
    """Extract every valid MAVLink 2 SET_MODE command from a datagram."""
    return [
        decode_set_mode_payload(payload)
        for message_id, payload in decode_frames(datagram)
        if message_id == MAVLINK_SET_MODE_MSG_ID
    ]


def make_target_position_payload(x, y, target_id, sequence_id):
    """Build the payload of project-specific message #999."""
    return struct.pack("<ffbb", x, y, target_id, sequence_id)


def encode_custom_position(
    x,
    y,
    target_id,
    sequence_id,
    packet_sequence=0,
    system_id=1,
    component_id=1,
):
    """Encode one project-specific #999 target-position frame."""
    return mavlink_pack(
        MAVLINK_CUSTOM_POSITION_MSG_ID,
        system_id,
        component_id,
        packet_sequence,
        make_target_position_payload(x, y, target_id, sequence_id),
    )


def encode_uav_position(
    latitude,
    longitude,
    altitude_m,
    velocity_x=0,
    velocity_y=0,
    velocity_z=0,
    target_id=0,
    packet_sequence=0,
    system_id=1,
    component_id=1,
):
    """Encode the project's #1000 aircraft-position MAVLink 2 frame."""
    time_boot_ms = int((time.time() % 86_400) * 1000)
    payload = struct.pack(
        "<Iiifhhhb",
        time_boot_ms,
        int(round(latitude * 1e7)),
        int(round(longitude * 1e7)),
        float(altitude_m * 100.0),
        int(velocity_x),
        int(velocity_y),
        int(velocity_z),
        int(target_id),
    )
    return mavlink_pack(
        MAVLINK_UAV_POSITION_MSG_ID,
        system_id,
        component_id,
        packet_sequence,
        payload,
    )


def parse_global_position_int(payload):
    """Parse MAVLink GLOBAL_POSITION_INT (#33) into SI-friendly values."""
    if len(payload) != 28:
        raise ValueError("GLOBAL_POSITION_INT payload must contain 28 bytes")
    (
        time_boot_ms,
        latitude_e7,
        longitude_e7,
        altitude_mm,
        relative_altitude_mm,
        velocity_x,
        velocity_y,
        velocity_z,
        heading_cdeg,
    ) = struct.unpack("<IiiiihhhH", payload)
    return {
        "time_boot_ms": time_boot_ms,
        "lat": latitude_e7 / 1e7,
        "lon": longitude_e7 / 1e7,
        "alt_msl_m": altitude_mm / 1000.0,
        "alt_m": relative_altitude_mm / 1000.0,
        "vx": velocity_x,
        "vy": velocity_y,
        "vz": velocity_z,
        "heading": heading_cdeg / 100.0,
    }


def gps_to_local(latitude, longitude, reference_latitude, reference_longitude):
    """Approximate WGS84 coordinates as local east/x and north/y metres."""
    x = (
        (longitude - reference_longitude)
        * math.cos(math.radians(latitude))
        * METER_PER_DEG
    )
    y = (latitude - reference_latitude) * METER_PER_DEG
    return x, y
