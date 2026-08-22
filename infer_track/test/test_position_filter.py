"""Tests for per-target position filtering before control publication."""

import math

import pytest

from infer_track.ROS2_coordinate_transform import (
    EARTH_R,
    TargetPositionFilter,
)


LATITUDE = 30.0
LONGITUDE = 120.0
LONGITUDE_SCALE = (
    math.cos(math.radians(LATITUDE)) * math.pi * EARTH_R / 180.0
)


def gps_at_x(x):
    return LATITUDE, LONGITUDE + x / LONGITUDE_SCALE


def filtered_x(result):
    return (
        result["target_longitude"] - LONGITUDE
    ) * LONGITUDE_SCALE


def test_single_large_jump_is_rejected_without_poisoning_filter():
    target_filter = TargetPositionFilter(1.0, 1.0, 7.0)

    for stamp, x in ((0.0, 0.0), (1.0, 1.0), (2.0, 2.0)):
        target_filter.update(*gps_at_x(x), stamp)

    assert target_filter.update(*gps_at_x(30.0), 3.0) is None
    recovered = target_filter.update(*gps_at_x(4.0), 4.0)

    assert filtered_x(recovered) == pytest.approx(4.0, abs=0.5)
    assert math.hypot(
        recovered["target_velocity_x"],
        recovered["target_velocity_y"],
    ) <= 7.0


def test_filter_resets_after_more_than_three_seconds_without_measurement():
    target_filter = TargetPositionFilter(1.0, 1.0, 7.0)
    target_filter.update(*gps_at_x(0.0), 0.0)
    target_filter.update(*gps_at_x(1.0), 1.0)

    reset = target_filter.update(*gps_at_x(50.0), 5.1)

    assert filtered_x(reset) == pytest.approx(50.0, abs=1e-6)
    assert reset["target_velocity_x"] == 0.0
    assert reset["target_velocity_y"] == 0.0


def test_each_target_filter_keeps_an_independent_state():
    first = TargetPositionFilter(1.0, 1.0, 7.0)
    second = TargetPositionFilter(1.0, 1.0, 7.0)

    first.update(*gps_at_x(0.0), 0.0)
    first_result = first.update(*gps_at_x(1.0), 1.0)
    second_result = second.update(*gps_at_x(100.0), 1.0)

    assert filtered_x(first_result) < 5.0
    assert filtered_x(second_result) == pytest.approx(100.0, abs=1e-6)
