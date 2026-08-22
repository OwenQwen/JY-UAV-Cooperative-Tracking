#!/usr/bin/env python3
"""与 ROS 通信解耦的双目标任务状态机."""

try:
    from .target_evidence import TargetEvidence, VALID_TARGET_IDS
except ImportError:
    from target_evidence import TargetEvidence, VALID_TARGET_IDS


MODE_STATIC_SEARCH = 1
MODE_MOVING_TRACK = 2
COMMAND_START = 1
COMMAND_END = 2

WAIT_START = 0
SEARCH_STATIC_TARGET = 1
HOVER_ABOVE_TARGET = 2
WAIT_MOVING_COMMAND = 3
ACQUIRE_MOVING_TARGET = 4
TRACK_MOVING_TARGET = 5
RETURN_HOME = 6

STATE_NAMES = {
    WAIT_START: "WAIT_START",
    SEARCH_STATIC_TARGET: "SEARCH_STATIC_TARGET",
    HOVER_ABOVE_TARGET: "HOVER_ABOVE_TARGET",
    WAIT_MOVING_COMMAND: "WAIT_MOVING_COMMAND",
    ACQUIRE_MOVING_TARGET: "ACQUIRE_MOVING_TARGET",
    TRACK_MOVING_TARGET: "TRACK_MOVING_TARGET",
    RETURN_HOME: "RETURN_HOME",
}


class MissionStateMachine:
    """完成 ID1→ID2 静态搜索、移动跟踪采点和返航流程."""

    def __init__(
        self,
        config,
        publish_tracking,
        publish_navigation,
        publish_report,
        state_changed=None,
    ):
        self.config = config
        self.publish_tracking = publish_tracking
        self.publish_navigation = publish_navigation
        self.publish_report = publish_report
        self.state_changed = state_changed or (lambda *_args: None)
        self.evidence = TargetEvidence(
            config["history_capacity"],
            config["target_timeout"],
            config["prediction_window"],
            config["max_target_speed"],
            config["max_extrapolation"],
        )

        self.state = WAIT_START
        self.state_enter_time = 0.0
        self.active_target_id = 1
        self.current_target_id = None
        self.static_candidate = None
        self.completed_static_ids = set()
        self.uav_gps = None

        self.tracking_enabled = False
        self.tracking_target_id = None
        self.last_valid_track_time = 0.0

        self.moving_target_id = None
        self.moving_start_time = None
        self.next_report_time = None
        self.next_sequence_id = 1

        self.handlers = {
            SEARCH_STATIC_TARGET: self._search_static_target,
            HOVER_ABOVE_TARGET: self._hover_above_target,
            ACQUIRE_MOVING_TARGET: self._acquire_moving_target,
            TRACK_MOVING_TARGET: self._track_moving_target,
        }

    def start(self, now):
        self.state_enter_time = now

    def receive_uav_gps(self, latitude, longitude, stamp):
        self.uav_gps = {
            "latitude": latitude,
            "longitude": longitude,
            "stamp": stamp,
        }

    def receive_target(self, target, now):
        target_id = target.get("target_id")
        target_stamp = target.get("stamp")
        if (
            self.state == TRACK_MOVING_TARGET
            and target_id == self.moving_target_id
            and target_stamp is not None
            and target_stamp - self.last_valid_track_time
            > self.config["prediction_stop_delay"]
        ):
            self._set_tracking(False, now)
            self.evidence.clear(target_id)
            self._transition(ACQUIRE_MOVING_TARGET, now)

        observation = self.evidence.record(target, now)
        if observation is None:
            return
        if (
            self.tracking_enabled
            and observation["target_id"] == self.tracking_target_id
        ):
            self.last_valid_track_time = observation["stamp"]

    def handle_referee(self, mode, target_id, command, now):
        if mode == MODE_STATIC_SEARCH and target_id == 0:
            if command == COMMAND_START:
                self._start_static_search(now)
            elif command == COMMAND_END:
                self._end_static_search()
            return

        if mode != MODE_MOVING_TRACK or target_id not in VALID_TARGET_IDS:
            return
        if command == COMMAND_START:
            self._start_moving_track(target_id, now)
        elif command == COMMAND_END:
            self._finish_mission(now)

    def update(self, now):
        handler = self.handlers.get(self.state)
        if handler is not None:
            handler(now)
        self._publish_due_moving_reports(now)

    def _start_static_search(self, now):
        if self.state not in {WAIT_START, RETURN_HOME}:
            return
        self.evidence.clear()  # 清除目标历史记录。
        self.completed_static_ids.clear()  # 清除已完成静态目标。
        self.static_candidate = None        # 清除静态候选目标
        self.current_target_id = None
        self.moving_target_id = None
        self.moving_start_time = None
        self.next_report_time = None      # 清除下次报告时间
        self.next_sequence_id = 1
        self.active_target_id = 1
        self._set_tracking(False, now)
        self._transition(SEARCH_STATIC_TARGET, now)
        self._send_navigation(
            {"command": "start_search", "target_id": 1},
            now,
        )

    def _end_static_search(self):
        """静态结束不关闭正在 ID2 上方运行的跟踪."""

    def _start_moving_track(self, target_id, now):
        if (
            self.moving_start_time is not None
            and self.moving_target_id == target_id
            and self.state in {
                ACQUIRE_MOVING_TARGET,
                TRACK_MOVING_TARGET,
            }
        ):
            return

        self.moving_target_id = target_id
        self.active_target_id = target_id
        self.moving_start_time = now
        self.next_report_time = now + self.config["moving_report_period"]
        self.next_sequence_id = 1  # 报告序号。

        if (
            self.current_target_id == target_id
            and self.tracking_enabled
            and self.tracking_target_id == target_id
        ):
            latest = self.evidence.latest(
                target_id,
                now,
                self.config["track_lost_timeout"],
            )
            self.last_valid_track_time = (
                latest["stamp"] if latest is not None else now
            )
            self._transition(TRACK_MOVING_TARGET, now)
            return

        previous_target_id = self.current_target_id
        self._set_tracking(False, now)
        if previous_target_id in VALID_TARGET_IDS:
            self._send_navigation(
                {
                    "command": "wrong_target",
                    "observed_target_id": previous_target_id,
                    "requested_target_id": target_id,
                },
                now,
            )
        self._transition(ACQUIRE_MOVING_TARGET, now)

    def _finish_mission(self, now):
        if self.state == RETURN_HOME:
            return
        self._set_tracking(False, now)
        self.moving_start_time = None
        self.next_report_time = None
        self._send_navigation({"command": "return_home"}, now)
        self._transition(RETURN_HOME, now)

    def _search_static_target(self, now):
        target = self.evidence.confirm_position(
            self.active_target_id,
            now,
            self.state_enter_time,
            self.config["confirm_min_count"],
            self.config["confirm_min_duration"],
            self.config["confirm_max_spread"],
        )
        if target is None:
            return
        self.static_candidate = target
        self._set_tracking(True, self.active_target_id, now)
        self._transition(HOVER_ABOVE_TARGET, now)

    def _hover_above_target(self, now):
        if not self.evidence.confirm_hover(
            self.active_target_id,
            now,
            self.state_enter_time,
            self.config["confirm_hover_distance"],
            self.config["confirm_min_count"],
            self.config["hover_confirm_min_duration"],
        ):
            return
        if (
            self.uav_gps is None
            or now - self.uav_gps["stamp"] > self.config["gps_timeout"]
        ):
            return

        target_id = self.active_target_id
        self.publish_report(
            {
                "target_id": target_id,
                "sequence_id": 0,
                "target_latitude": self.uav_gps["latitude"],
                "target_longitude": self.uav_gps["longitude"],
                "stamp": self.uav_gps["stamp"],
            }
        )
        self.completed_static_ids.add(target_id)
        self.current_target_id = target_id
        self.static_candidate = None

        if target_id == 1:
            self._set_tracking(False, now)
            self.active_target_id = 2
            self._send_navigation(
                {
                    "command": "switch_target",
                    "observed_target_id": 1,
                    "requested_target_id": 2,
                },
                now,
            )
            self._transition(SEARCH_STATIC_TARGET, now)
            return

        self._transition(WAIT_MOVING_COMMAND, now)

    def _acquire_moving_target(self, now):
        target = self.evidence.confirm_seen(
            self.moving_target_id,
            now,
            self.state_enter_time,
            self.config["target_confirm_count"],
            self.config["target_confirm_min_duration"],
        )
        if target is None:
            return
        self.current_target_id = self.moving_target_id
        self.last_valid_track_time = target["stamp"]
        self._set_tracking(True, self.moving_target_id, now)
        self._transition(TRACK_MOVING_TARGET, now)

    def _track_moving_target(self, now):
        latest = self.evidence.latest(
            self.moving_target_id,
            now,
            self.config["prediction_stop_delay"],
        )
        if latest is not None:
            self.last_valid_track_time = latest["stamp"]
            return
        if (
            now - self.last_valid_track_time
            > self.config["prediction_stop_delay"]
        ):
            self._set_tracking(False, now)
            self.evidence.clear(self.moving_target_id)
            self._transition(ACQUIRE_MOVING_TARGET, now)

    def _publish_due_moving_reports(self, now):
        if (
            self.state != TRACK_MOVING_TARGET
            or self.moving_start_time is None
            or self.next_report_time is None
            or self.next_sequence_id > self.config["moving_report_count"]
        ):
            return

        due_time = self.next_report_time
        if now < due_time:
            return
        latest = self.evidence.latest(
            self.moving_target_id,
            now,
            self.config["prediction_stop_delay"],
        )
        if latest is None:
            return
        lost_duration = now - latest["stamp"]

        if lost_duration < self.config["prediction_start_delay"]:
            if (
                self.uav_gps is None
                or now - self.uav_gps["stamp"]
                > self.config["gps_timeout"]
            ):
                return
            position = {
                "target_latitude": self.uav_gps["latitude"],
                "target_longitude": self.uav_gps["longitude"],
            }
            report_time = self.uav_gps["stamp"]
        else:
            position = self.evidence.predict_gps(
                self.moving_target_id,
                due_time,
            )
            if position is None:
                return
            report_time = due_time

        self.publish_report(
            {
                "target_id": self.moving_target_id,
                "sequence_id": self.next_sequence_id,
                "target_latitude": position["target_latitude"],
                "target_longitude": position["target_longitude"],
                "stamp": report_time,
            }
        )
        self.next_sequence_id += 1
        schedule_time = due_time
        if now - due_time > self.config["prediction_stop_delay"]:
            schedule_time = now
        self.next_report_time = (
            schedule_time + self.config["moving_report_period"]
        )

    def _set_tracking(self, enable, target_id_or_now, now=None):
        if now is None:
            now = target_id_or_now
            target_id = self.tracking_target_id
        else:
            target_id = target_id_or_now

        if enable:
            if self.tracking_enabled and self.tracking_target_id == target_id:
                return
            self.publish_tracking(
                {
                    "enable": True,
                    "target_id": target_id,
                    "stamp": now,
                }
            )
            self.tracking_enabled = True
            self.tracking_target_id = target_id
            return

        if not self.tracking_enabled:
            return
        self.publish_tracking(
            {
                "enable": False,
                "target_id": self.tracking_target_id,
                "stamp": now,
            }
        )
        self.tracking_enabled = False
        self.tracking_target_id = None

    def _send_navigation(self, command, now):
        self.publish_navigation(dict(command, stamp=now))

    def _transition(self, new_state, now):
        if new_state == self.state:
            self.state_enter_time = now
            return
        old_state = self.state
        self.state = new_state
        self.state_enter_time = now
        self.state_changed(old_state, new_state)

    def stop(self, now):
        self._set_tracking(False, now)
