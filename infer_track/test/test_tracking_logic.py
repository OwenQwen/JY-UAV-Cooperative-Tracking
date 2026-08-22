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

"""Tests for the target-offset tracking gate."""

import pytest

from infer_track.tracking_node import TrackingGate


def target(
    target_id,
    offset_x=1.0,
    offset_y=-2.0,
    stamp=0.2,
    target_velocity_x=0.0,
    target_velocity_y=0.0,
    uav_velocity_x=0.0,
    uav_velocity_y=0.0,
):
    return {
        "target_id": target_id,
        "offset_x": offset_x,
        "offset_y": offset_y,
        "stamp": stamp,
        "target_velocity_x": target_velocity_x,
        "target_velocity_y": target_velocity_y,
        "uav_velocity_x": uav_velocity_x,
        "uav_velocity_y": uav_velocity_y,
    }


def test_only_selected_target_is_forwarded():
    gate = TrackingGate()
    gate.command({"enable": True, "target_id": 1}, 0.0)

    assert gate.target(target(2), 0.1) is None
    command = gate.target(target(1), 0.2)

    assert command == {
        "enable": True,
        "target_id": 1,
        "offset_x": 1.0,
        "offset_y": -2.0,
        "stamp": pytest.approx(0.3),
    }


def test_switching_target_stops_previous_control():
    gate = TrackingGate()
    gate.command({"enable": True, "target_id": 1}, 0.0)
    gate.target(target(1), 0.1)

    commands = gate.command(
        {"enable": True, "target_id": 2},
        0.2,
    )

    assert commands == [
        {
            "enable": False,
            "target_id": 1,
            "offset_x": 0.0,
            "offset_y": 0.0,
            "stamp": 0.2,
        }
    ]
    assert gate.target(target(1), 0.3) is None
    assert gate.target(target(2), 0.3)["target_id"] == 2


def test_loss_predicts_then_stops_once_and_requires_new_enable():
    gate = TrackingGate()
    gate.command({"enable": True, "target_id": 1}, 0.0)
    gate.target(target(1, stamp=0.1), 0.1)

    assert gate.update(0.29) is None
    prediction = gate.update(0.3)
    assert prediction["enable"] is True
    assert gate.update(3.1)["enable"] is True

    stop = gate.update(3.1001)
    assert stop["enable"] is False
    assert stop["target_id"] == 1
    assert gate.update(3.2) is None

    assert gate.target(target(1, 0.5, 0.25, stamp=3.3), 3.3) is None

    gate.command({"enable": True, "target_id": 1}, 3.31)
    resumed = gate.target(target(1, 0.5, 0.25, stamp=3.4), 3.4)
    assert resumed["enable"] is True
    assert resumed["offset_x"] == 0.5
    assert resumed["offset_y"] == 0.25


def test_prediction_uses_target_minus_uav_velocity():
    gate = TrackingGate()
    gate.command({"enable": True, "target_id": 1}, 1.0)

    command = gate.target(
        target(
            1,
            offset_x=1.0,
            offset_y=2.0,
            stamp=1.0,
            target_velocity_x=2.0,
            target_velocity_y=-1.0,
            uav_velocity_x=0.5,
            uav_velocity_y=-0.5,
        ),
        1.0,
    )

    assert command["offset_x"] == pytest.approx(1.15)
    assert command["offset_y"] == pytest.approx(1.95)
    assert command["stamp"] == pytest.approx(1.1)


def test_prediction_integrates_uav_velocity_changes_during_loss():
    gate = TrackingGate()
    gate.update_uav_velocity(0.0, 0.0, 0.0)
    gate.command({"enable": True, "target_id": 1}, 0.0)
    gate.target(
        target(1, offset_x=10.0, stamp=0.0, target_velocity_x=1.0),
        0.0,
    )

    gate.update_uav_velocity(2.0, 0.0, 0.2)
    first = gate.update(0.2)
    gate.update_uav_velocity(4.0, 0.0, 0.3)
    second = gate.update(0.4)

    assert first["offset_x"] == pytest.approx(10.1)
    assert second["offset_x"] == pytest.approx(9.5)


def test_late_frame_stops_instead_of_resuming_before_timer():
    gate = TrackingGate()
    gate.command({"enable": True, "target_id": 1}, 0.0)
    gate.target(target(1, stamp=0.0), 0.0)

    stop = gate.target(target(1, stamp=3.1), 3.1)

    assert stop["enable"] is False
    assert gate.requested_enable is False
    assert gate.target(target(1, stamp=3.2), 3.2) is None


def test_disable_stops_active_tracking():
    gate = TrackingGate()
    gate.command({"enable": True, "target_id": 1}, 0.0)
    gate.target(target(1), 0.1)

    commands = gate.command(
        {"enable": False, "target_id": 1},
        0.2,
    )

    assert len(commands) == 1
    assert commands[0]["enable"] is False
    assert gate.requested_enable is False
    assert gate.target(target(1), 0.3) is None
