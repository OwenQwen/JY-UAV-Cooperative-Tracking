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

"""Tests for the ROS-independent mission logic."""

import math

import pytest

from infer_track.mission_state_machine import (
    ACQUIRE_MOVING_TARGET,
    COMMAND_END,
    COMMAND_START,
    HOVER_ABOVE_TARGET,
    MODE_MOVING_TRACK,
    MODE_STATIC_SEARCH,
    RETURN_HOME,
    SEARCH_STATIC_TARGET,
    TRACK_MOVING_TARGET,
    WAIT_MOVING_COMMAND,
    MissionStateMachine,
)
from infer_track.target_evidence import EARTH_M_PER_DEG, TargetEvidence


CONFIG = {
    "confirm_max_spread": 5.0,
    "confirm_min_count": 5,
    "confirm_min_duration": 0.4,
    "confirm_hover_distance": 5.0,
    "hover_confirm_min_duration": 0.4,
    "target_confirm_count": 5,
    "target_confirm_min_duration": 0.4,
    "target_timeout": 1.5,
    "track_lost_timeout": 3.0,
    "gps_timeout": 1.5,
    "history_capacity": 50,
    "prediction_window": 2.0,
    "max_target_speed": 7.0,
    "max_extrapolation": 3.0,
    "prediction_start_delay": 0.2,
    "prediction_stop_delay": 3.0,
    "moving_report_period": 20.0,
    "moving_report_count": 16,
}


def make_machine():
    """Create a state machine and capture all of its output messages."""
    output = {
        "tracking": [],
        "navigation": [],
        "reports": [],
        "states": [],
    }
    machine = MissionStateMachine(
        CONFIG,
        output["tracking"].append,
        output["navigation"].append,
        output["reports"].append,
        lambda old, new: output["states"].append((old, new)),
    )
    machine.start(0.0)
    return machine, output


def observation(target_id, latitude, longitude, offset=0.0, stamp=None):
    """Build one standardized target-position observation."""
    result = {
        "target_id": target_id,
        "target_latitude": latitude,
        "target_longitude": longitude,
        "offset_x": offset,
        "offset_y": 0.0,
    }
    if stamp is not None:
        result["stamp"] = stamp
    return result


def feed_confirmation(
    machine,
    target_id,
    start_time,
    latitude,
    longitude,
    offset=0.0,
):
    """Feed five observations spanning more than 0.4 seconds."""
    last_time = start_time
    for index in range(5):
        last_time = start_time + index * 0.11
        machine.receive_target(
            observation(
                target_id,
                latitude,
                longitude,
                offset,
                last_time,
            ),
            last_time,
        )
        machine.update(last_time)
    return last_time


def complete_static_target(
    machine,
    target_id,
    start_time,
    target_latitude,
    target_longitude,
    uav_latitude,
    uav_longitude,
):
    """Confirm a target, then confirm the UAV is hovering above it."""
    confirmed_at = feed_confirmation(
        machine,
        target_id,
        start_time,
        target_latitude,
        target_longitude,
        offset=10.0,
    )
    assert machine.state == HOVER_ABOVE_TARGET

    hover_start = confirmed_at + 0.11
    machine.receive_uav_gps(
        uav_latitude,
        uav_longitude,
        hover_start + 0.44,
    )
    finished_at = feed_confirmation(
        machine,
        target_id,
        hover_start,
        target_latitude,
        target_longitude,
    )
    return finished_at


def complete_both_static_targets(machine):
    """Drive the full static ID1-then-ID2 workflow."""
    machine.handle_referee(
        MODE_STATIC_SEARCH,
        0,
        COMMAND_START,
        0.0,
    )
    first_done = complete_static_target(
        machine,
        1,
        0.10,
        30.00001,
        120.00001,
        30.10001,
        120.10001,
    )
    second_done = complete_static_target(
        machine,
        2,
        first_done + 0.20,
        30.00002,
        120.00002,
        30.10002,
        120.10002,
    )
    return second_done


def test_static_search_completes_id1_then_id2():
    """Static search must report ID1, redirect, then report ID2."""
    machine, output = make_machine()

    machine.handle_referee(
        MODE_STATIC_SEARCH,
        0,
        COMMAND_START,
        0.0,
    )
    assert machine.state == SEARCH_STATIC_TARGET
    assert output["navigation"] == [
        {
            "command": "start_search",
            "target_id": 1,
            "stamp": 0.0,
        }
    ]

    first_done = complete_static_target(
        machine,
        1,
        0.10,
        30.00001,
        120.00001,
        30.10001,
        120.10001,
    )
    assert machine.state == SEARCH_STATIC_TARGET
    assert machine.active_target_id == 2
    assert machine.tracking_enabled is False
    assert output["reports"][0] == {
        "target_id": 1,
        "sequence_id": 0,
        "target_latitude": 30.10001,
        "target_longitude": 120.10001,
        "stamp": pytest.approx(first_done),
    }
    assert output["navigation"][-1]["command"] == "switch_target"
    assert output["navigation"][-1]["observed_target_id"] == 1
    assert output["navigation"][-1]["requested_target_id"] == 2

    complete_static_target(
        machine,
        2,
        first_done + 0.20,
        30.00002,
        120.00002,
        30.10002,
        120.10002,
    )
    assert machine.state == WAIT_MOVING_COMMAND
    assert machine.tracking_enabled is True
    assert machine.tracking_target_id == 2
    assert [item["target_id"] for item in output["reports"]] == [1, 2]
    assert [item["sequence_id"] for item in output["reports"]] == [0, 0]
    assert output["reports"][1]["target_latitude"] == 30.10002
    assert output["reports"][1]["target_longitude"] == 120.10002


def test_static_end_does_not_disable_id2_tracking():
    """Static END must leave the UAV tracking above ID2."""
    machine, output = make_machine()
    complete_both_static_targets(machine)
    tracking_before = list(output["tracking"])

    machine.handle_referee(
        MODE_STATIC_SEARCH,
        0,
        COMMAND_END,
        5.0,
    )

    assert machine.state == WAIT_MOVING_COMMAND
    assert machine.tracking_enabled is True
    assert machine.tracking_target_id == 2
    assert output["tracking"] == tracking_before


def test_moving_start_on_current_id_keeps_tracking_enabled():
    """Selecting ID2 must continue tracking without duplicate commands."""
    machine, output = make_machine()
    complete_both_static_targets(machine)
    tracking_before = list(output["tracking"])

    machine.handle_referee(
        MODE_MOVING_TRACK,
        2,
        COMMAND_START,
        10.0,
    )

    assert machine.state == TRACK_MOVING_TARGET
    assert machine.moving_target_id == 2
    assert machine.moving_start_time == 10.0
    assert machine.tracking_enabled is True
    assert machine.tracking_target_id == 2
    assert output["tracking"] == tracking_before


def test_moving_start_on_other_id_redirects_and_reacquires():
    """Selecting ID1 must redirect from ID2 and reacquire ID1."""
    machine, output = make_machine()
    complete_both_static_targets(machine)

    machine.handle_referee(
        MODE_MOVING_TRACK,
        1,
        COMMAND_START,
        10.0,
    )

    assert machine.state == ACQUIRE_MOVING_TARGET
    assert machine.tracking_enabled is False
    assert output["tracking"][-1]["enable"] is False
    assert output["tracking"][-1]["target_id"] == 2
    assert output["navigation"][-1] == {
        "command": "wrong_target",
        "observed_target_id": 2,
        "requested_target_id": 1,
        "stamp": 10.0,
    }

    feed_confirmation(
        machine,
        1,
        10.10,
        30.00003,
        120.00003,
    )

    assert machine.state == TRACK_MOVING_TARGET
    assert machine.current_target_id == 1
    assert machine.tracking_enabled is True
    assert machine.tracking_target_id == 1
    assert output["tracking"][-1]["enable"] is True
    assert output["tracking"][-1]["target_id"] == 1


def test_moving_reports_use_20_second_intervals_and_sequences_1_to_16():
    """Moving reports must be emitted every 20 seconds exactly 16 times."""
    machine, output = make_machine()
    complete_both_static_targets(machine)
    machine.handle_referee(
        MODE_MOVING_TRACK,
        2,
        COMMAND_START,
        10.0,
    )

    for index in range(1, 681):
        sample_time = 10.0 + index * 0.5
        machine.receive_target(
            observation(
                2,
                30.0,
                120.0,
                stamp=sample_time,
            ),
            sample_time,
        )
        machine.receive_uav_gps(30.2, 120.2, sample_time)
        machine.update(sample_time)

    moving_reports = [
        report
        for report in output["reports"]
        if report["sequence_id"] > 0
    ]
    assert [item["sequence_id"] for item in moving_reports] == list(
        range(1, 17)
    )
    assert [item["stamp"] for item in moving_reports] == [
        10.0 + sequence_id * 20.0
        for sequence_id in range(1, 17)
    ]
    assert all(item["target_id"] == 2 for item in moving_reports)
    assert all(item["target_latitude"] == 30.2 for item in moving_reports)
    assert all(item["target_longitude"] == 120.2 for item in moving_reports)

    machine.update(350.1)
    assert len(
        [
            report
            for report in output["reports"]
            if report["sequence_id"] > 0
        ]
    ) == 16


def test_overdue_report_waits_for_new_position_without_losing_sequence():
    """A stale sample must pause and later resume the same report number."""
    machine, output = make_machine()
    complete_both_static_targets(machine)
    machine.handle_referee(
        MODE_MOVING_TRACK,
        2,
        COMMAND_START,
        10.0,
    )

    machine.update(30.0)
    assert machine.next_sequence_id == 1
    assert not any(
        report["sequence_id"] > 0
        for report in output["reports"]
    )

    machine.receive_uav_gps(30.2, 120.2, 30.94)
    feed_confirmation(
        machine,
        2,
        30.5,
        30.2,
        120.2,
    )

    assert machine.next_sequence_id == 2
    assert output["reports"][-1]["sequence_id"] == 1
    assert output["reports"][-1]["stamp"] == pytest.approx(30.94)


def test_track_loss_disables_control_until_reacquired():
    """Three seconds without a target must disable active tracking."""
    machine, output = make_machine()
    complete_both_static_targets(machine)
    machine.handle_referee(
        MODE_MOVING_TRACK,
        2,
        COMMAND_START,
        10.0,
    )

    machine.update(13.1)

    assert machine.state == ACQUIRE_MOVING_TARGET
    assert machine.tracking_enabled is False
    assert output["tracking"][-1]["enable"] is False
    assert output["tracking"][-1]["target_id"] == 2


def test_late_first_frame_enters_acquire_before_timer_runs():
    """A frame after a three-second gap must not bypass reacquisition."""
    machine, output = make_machine()
    complete_both_static_targets(machine)
    machine.handle_referee(MODE_MOVING_TRACK, 2, COMMAND_START, 10.0)

    machine.receive_target(
        observation(2, 30.0, 120.0, stamp=13.1),
        13.1,
    )

    assert machine.state == ACQUIRE_MOVING_TARGET
    assert machine.tracking_enabled is False
    assert output["tracking"][-1]["enable"] is False

    for stamp in (13.21, 13.32, 13.43, 13.54):
        machine.receive_target(
            observation(2, 30.0, 120.0, stamp=stamp),
            stamp,
        )
        machine.update(stamp)

    assert machine.state == TRACK_MOVING_TARGET
    assert machine.tracking_enabled is True


def test_repeated_static_start_does_not_restart_completed_id1():
    """A repeated static START must not erase completed work."""
    machine, output = make_machine()
    machine.handle_referee(MODE_STATIC_SEARCH, 0, COMMAND_START, 0.0)
    complete_static_target(
        machine,
        1,
        0.10,
        30.00001,
        120.00001,
        30.10001,
        120.10001,
    )
    navigation_before = list(output["navigation"])

    machine.handle_referee(MODE_STATIC_SEARCH, 0, COMMAND_START, 2.0)

    assert machine.state == SEARCH_STATIC_TARGET
    assert machine.active_target_id == 2
    assert machine.completed_static_ids == {1}
    assert output["navigation"] == navigation_before


def test_long_loss_recovery_does_not_publish_duplicate_backlog():
    """Recovery must publish one pending point and restart the interval."""
    machine, output = make_machine()
    complete_both_static_targets(machine)
    machine.handle_referee(MODE_MOVING_TRACK, 2, COMMAND_START, 10.0)
    machine.update(100.0)
    assert machine.state == ACQUIRE_MOVING_TARGET

    machine.receive_uav_gps(30.2, 120.2, 100.54)
    recovered_at = feed_confirmation(
        machine,
        2,
        100.1,
        30.2,
        120.2,
    )
    moving_reports = [
        report for report in output["reports"]
        if report["sequence_id"] > 0
    ]

    assert len(moving_reports) == 1
    assert moving_reports[0]["sequence_id"] == 1
    assert moving_reports[0]["stamp"] == pytest.approx(recovered_at)
    assert machine.next_report_time == pytest.approx(recovered_at + 20.0)


def test_moving_end_disables_tracking_and_returns_home_once():
    """Moving END must stop tracking and issue one return-home command."""
    machine, output = make_machine()
    complete_both_static_targets(machine)
    machine.handle_referee(
        MODE_MOVING_TRACK,
        2,
        COMMAND_START,
        10.0,
    )

    machine.handle_referee(
        MODE_MOVING_TRACK,
        2,
        COMMAND_END,
        12.0,
    )

    assert machine.state == RETURN_HOME
    assert machine.tracking_enabled is False
    assert output["tracking"][-1] == {
        "enable": False,
        "target_id": 2,
        "stamp": 12.0,
    }
    assert output["navigation"][-1] == {
        "command": "return_home",
        "stamp": 12.0,
    }

    machine.handle_referee(
        MODE_MOVING_TRACK,
        2,
        COMMAND_END,
        13.0,
    )
    return_commands = [
        item
        for item in output["navigation"]
        if item["command"] == "return_home"
    ]
    assert len(return_commands) == 1


def test_prediction_accepts_three_seconds_and_rejects_more():
    """GPS prediction must stop beyond the three-second limit."""
    evidence = TargetEvidence(
        capacity=10,
        timeout=1.5,
        prediction_window=2.0,
        max_speed=7.0,
        max_extrapolation=3.0,
    )
    evidence.record(
        observation(1, 30.0, 120.0, stamp=0.0),
        0.0,
    )
    evidence.record(
        observation(
            1,
            30.0 + 2.0 / EARTH_M_PER_DEG,
            120.0,
            stamp=2.0,
        ),
        2.0,
    )

    at_limit = evidence.predict_gps(1, 5.0)
    beyond_limit = evidence.predict_gps(1, 5.0001)

    assert at_limit is not None
    expected_latitude = 30.0 + 5.0 / EARTH_M_PER_DEG
    assert at_limit["target_latitude"] == pytest.approx(
        expected_latitude,
        abs=1e-10,
    )
    assert beyond_limit is None


def test_prediction_prefers_filtered_velocity_when_available():
    """Referee prediction must use the same velocity as control prediction."""
    evidence = TargetEvidence(
        capacity=10,
        timeout=1.5,
        prediction_window=2.0,
        max_speed=7.0,
        max_extrapolation=3.0,
    )
    first = observation(1, 30.0, 120.0, stamp=0.0)
    latest = observation(
        1,
        30.0 + 2.0 / EARTH_M_PER_DEG,
        120.0,
        stamp=2.0,
    )
    latest["target_velocity_x"] = 0.0
    latest["target_velocity_y"] = 2.0
    evidence.record(first, 0.0)
    evidence.record(latest, 2.0)

    predicted = evidence.predict_gps(1, 3.0)

    assert predicted["target_latitude"] == pytest.approx(
        30.0 + 4.0 / EARTH_M_PER_DEG,
        abs=1e-10,
    )


def test_lost_report_uses_predicted_target_gps():
    """After 0.2 seconds without detection, report the predicted target."""
    machine, output = make_machine()
    complete_both_static_targets(machine)
    machine.handle_referee(MODE_MOVING_TRACK, 2, COMMAND_START, 10.0)

    for index in range(1, 40):
        stamp = 10.0 + index * 0.5
        target = observation(2, 30.0, 120.0, stamp=stamp)
        target["target_velocity_x"] = 1.0
        target["target_velocity_y"] = 0.0
        machine.receive_target(target, stamp)
        machine.update(stamp)

    machine.receive_uav_gps(31.0, 121.0, 30.0)
    machine.update(30.0)
    report = output["reports"][-1]

    longitude_scale = math.cos(math.radians(30.0)) * EARTH_M_PER_DEG
    assert report["sequence_id"] == 1
    assert report["target_latitude"] == pytest.approx(30.0)
    assert report["target_longitude"] == pytest.approx(
        120.0 + 0.5 / longitude_scale
    )
    assert report["stamp"] == 30.0
