#!/usr/bin/env python3
"""目标多帧确认和短时位置预测."""

import math
from collections import deque
from statistics import median


VALID_TARGET_IDS = (1, 2)
EARTH_M_PER_DEG = 111194.9266


def _window(observations, count, duration):
    """返回最近一段同时满足数量和持续时间的观测."""
    if len(observations) < count:
        return []
    start = len(observations) - count
    latest_time = observations[-1]["stamp"]
    while start > 0:
        if latest_time - observations[start]["stamp"] >= duration:
            break
        start -= 1
    result = observations[start:]
    if (
        result[-1]["stamp"] - result[0]["stamp"] + 1e-9
        < duration
    ):
        return []
    return result


def _trailing(observations, predicate):
    start = len(observations)
    while start and predicate(observations[start - 1]):
        start -= 1
    return observations[start:]


def _gps_distance(first, second):
    mean_lat = math.radians(
        (first["target_latitude"] + second["target_latitude"]) * 0.5
    )
    delta_lon = (
        second["target_longitude"] - first["target_longitude"] + 180.0
    ) % 360.0 - 180.0
    dx = delta_lon * math.cos(mean_lat) * EARTH_M_PER_DEG
    dy = (
        second["target_latitude"] - first["target_latitude"]
    ) * EARTH_M_PER_DEG
    return math.hypot(dx, dy)


class TargetEvidence:
    """按车辆 ID 保存观测，并完成目标确认与位置预测."""

    def __init__(
        self,
        capacity,
        timeout,
        prediction_window=2.0,
        max_speed=7.0,
        max_extrapolation=3.0,
    ):
        self.timeout = timeout
        self.prediction_window = prediction_window
        self.max_speed = max_speed
        self.max_extrapolation = max_extrapolation
        self.histories = {
            target_id: deque(maxlen=capacity)
            for target_id in VALID_TARGET_IDS
        }

    def clear(self, target_id=None):
        """清除一个目标或全部目标的历史."""
        if target_id in self.histories:
            self.histories[target_id].clear()
            return
        for history in self.histories.values():
            history.clear()

    def record(self, target, now):
        """保存一条定位观测."""
        target_id = target.get("target_id")
        if target_id not in self.histories:
            return None
        if (
            "target_latitude" not in target
            or "target_longitude" not in target
        ):
            return None

        observation = dict(target)
        history = self.histories[target_id]
        if history:
            previous = history[-1]
            dt = observation["stamp"] - previous["stamp"]

            if dt <= 0:
                return None

            distance = _gps_distance(previous, observation)

            gps_noise_margin = 3.0
            allowed_distance = (
                self.max_speed * dt + gps_noise_margin
            )

            if distance > allowed_distance:
                return None

        history.append(observation)
        return observation

    def recent(self, target_id, now, not_before=None, timeout=None):
        """取得仍在有效期内的指定目标观测."""
        valid_for = self.timeout if timeout is None else timeout
        return [
            item
            for item in self.histories[target_id]
            if 0.0 <= now - item["stamp"] <= valid_for
            and (not_before is None or item["stamp"] >= not_before)
        ]

    def latest(self, target_id, now, timeout=None):
        recent = self.recent(target_id, now, timeout=timeout)
        return recent[-1] if recent else None

    def confirm_position(
        self,
        target_id,
        now,
        not_before,
        count,
        duration,
        max_spread,
    ):
        """确认连续多帧目标 GPS 聚集在给定范围内."""
        recent = self.recent(target_id, now, not_before)
        matching = _trailing(
            recent,
            lambda item: (
                "target_latitude" in item
                and "target_longitude" in item
            ),
        )
        window = _window(matching, count, duration)
        if not window:
            return None
        center = {
            "target_id": target_id,
            "target_latitude": median(
                item["target_latitude"] for item in window
            ),
            "target_longitude": median(
                item["target_longitude"] for item in window
            ),
            "stamp": window[-1]["stamp"],
        }
        if any(_gps_distance(item, center) > max_spread for item in window):
            return None
        return center

    def confirm_hover(
        self,
        target_id,
        now,
        not_before,
        max_distance,
        count,
        duration,
    ):
        """确认目标偏移量连续位于悬停阈值内."""
        recent = self.recent(target_id, now, not_before)
        close = _trailing(
            recent,
            lambda item: (
                "offset_x" in item
                and "offset_y" in item
                and math.hypot(item["offset_x"], item["offset_y"])
                <= max_distance
            ),
        )
        return bool(_window(close, count, duration))

    def confirm_seen(self, target_id, now, not_before, count, duration):
        """确认指定车辆已经被稳定识别."""
        window = _window(
            self.recent(target_id, now, not_before),
            count,
            duration,
        )
        return window[-1] if window else None

    def predict_gps(self, target_id, sample_time):
        """用最近观测拟合匀速运动，并预测采样时刻的 GPS."""
        points = [
            item
            for item in self.histories[target_id]
            if "target_latitude" in item and "target_longitude" in item
        ]
        if not points:
            return None

        latest = points[-1]
        if abs(sample_time - latest["stamp"]) > self.max_extrapolation:
            return None

        if (
            "target_velocity_x" in latest
            and "target_velocity_y" in latest
        ):
            velocity_x = float(latest["target_velocity_x"])
            velocity_y = float(latest["target_velocity_y"])
            if math.isfinite(velocity_x) and math.isfinite(velocity_y):
                speed = math.hypot(velocity_x, velocity_y)
                if speed > self.max_speed:
                    scale = self.max_speed / speed
                    velocity_x *= scale
                    velocity_y *= scale
                delta_time = sample_time - latest["stamp"]
                longitude_scale = (
                    math.cos(math.radians(latest["target_latitude"]))
                    * EARTH_M_PER_DEG
                )
                return {
                    "target_latitude": (
                        latest["target_latitude"]
                        + velocity_y * delta_time / EARTH_M_PER_DEG
                    ),
                    "target_longitude": (
                        latest["target_longitude"]
                        + velocity_x * delta_time / longitude_scale
                    ),
                }

        window_start = latest["stamp"] - self.prediction_window
        points = [
            item for item in points
            if item["stamp"] >= window_start
        ]
        if len(points) == 1:
            return {
                "target_latitude": latest["target_latitude"],
                "target_longitude": latest["target_longitude"],
            }

        reference_lat = points[0]["target_latitude"]
        reference_lon = points[0]["target_longitude"]
        longitude_scale = (
            math.cos(math.radians(reference_lat)) * EARTH_M_PER_DEG
        )
        times = [item["stamp"] for item in points]
        xs = [
            (item["target_longitude"] - reference_lon) * longitude_scale
            for item in points
        ]
        ys = [
            (item["target_latitude"] - reference_lat) * EARTH_M_PER_DEG
            for item in points
        ]
        mean_time = sum(times) / len(times)
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        denominator = sum((value - mean_time) ** 2 for value in times)
        if denominator <= 1e-9:
            return {
                "target_latitude": latest["target_latitude"],
                "target_longitude": latest["target_longitude"],
            }

        velocity_x = sum(
            (time_value - mean_time) * (x_value - mean_x)
            for time_value, x_value in zip(times, xs)
        ) / denominator
        velocity_y = sum(
            (time_value - mean_time) * (y_value - mean_y)
            for time_value, y_value in zip(times, ys)
        ) / denominator
        speed = math.hypot(velocity_x, velocity_y)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            velocity_x *= scale
            velocity_y *= scale

        delta_time = sample_time - mean_time
        predicted_x = mean_x + velocity_x * delta_time
        predicted_y = mean_y + velocity_y * delta_time
        return {
            "target_latitude": (
                reference_lat + predicted_y / EARTH_M_PER_DEG
            ),
            "target_longitude": (
                reference_lon + predicted_x / longitude_scale
            ),
        }
