"""Short reobservation of an already acquired ball, never initial acquisition."""

import math


class BallApproachObservation:
    """Keep a strong anchor while checking distinct, current candidate frames."""

    MAX_ANCHOR_AGE_SEC = 0.5
    MAX_DEPTH_AGE_SEC = 0.2
    REQUIRED_HITS = 3
    MAX_CENTER_SHIFT_NORM = 0.05
    MIN_AREA_RATIO = 0.7
    MAX_DISTANCE_CHANGE_M = 0.15
    MIN_BOTTOM_DISTANCE_PX = 120  # Exclude the head-down pickup region.

    def __init__(self):
        self.reset()

    def reset(self):
        self.anchor = None
        self.anchor_received_at = None
        self.last_stamp_ns = None
        self.hits = 0
        self.observation = None
        self.observation_received_at = None

    @staticmethod
    def number(info, key):
        value = info.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        return float(value) if math.isfinite(value) else None

    def usable(self, info, config):
        distance = self.number(info, "distance_m")
        depth_age = self.number(info, "depth_age_sec")
        confidence = self.number(info, "confidence")
        bottom = self.number(info, "bottom_distance_px")
        return bool(
            info.get("depth_valid") is True
            and distance is not None
            and config.pickup_sequence_start_distance_m < distance <= config.control_start_depth_m
            and depth_age is not None and 0.0 <= depth_age <= self.MAX_DEPTH_AGE_SEC
            and confidence is not None and confidence >= config.min_confidence
            and bottom is not None and bottom > self.MIN_BOTTOM_DISTANCE_PX
            and info.get("head_down_requested") is not True
        )

    def matches_anchor(self, info):
        anchor = self.anchor
        if anchor is None:
            return False
        if any(info.get(key) != anchor.get(key) for key in ("image_width", "image_height")):
            return False
        width = self.number(info, "image_width")
        height = self.number(info, "image_height")
        if width is None or height is None or min(width, height) <= 0:
            return False
        boxes = []
        for sample in (anchor, info):
            bbox = sample.get("bbox")
            if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
                return False
            box = [self.number({"v": value}, "v") for value in bbox]
            if any(value is None for value in box):
                return False
            left, top, right, bottom = box
            if not (0 <= left < right <= width and 0 <= top < bottom <= height):
                return False
            boxes.append(box)
        a, b = boxes
        shift = math.hypot(
            ((a[0] + a[2]) - (b[0] + b[2])) / (2 * width),
            ((a[1] + a[3]) - (b[1] + b[3])) / (2 * height),
        )
        areas = [(box[2] - box[0]) * (box[3] - box[1]) for box in boxes]
        return (
            shift <= self.MAX_CENTER_SHIFT_NORM
            and min(areas) / max(areas) >= self.MIN_AREA_RATIO
            and abs(info["distance_m"] - anchor["distance_m"]) <= self.MAX_DISTANCE_CHANGE_M
        )

    def observe(self, info, received_at, config):
        """Prepare geometry during motion; do not grant a motion or pickup lock."""
        self.observation = None
        self.observation_received_at = None
        stamp = info.get("rgb_stamp_ns")
        if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0:
            self.hits = 0
            return
        if self.last_stamp_ns is not None and stamp <= self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp
        if info.get("detected") is True:
            self.hits = 0
            if self.usable(info, config):
                self.anchor = dict(info)
                self.anchor_received_at = received_at
            else:
                self.anchor = None
            return
        candidate = info.get("approach_candidate")
        if (
            info.get("raw_detected") is not True
            or not isinstance(candidate, dict)
            or self.anchor is None
            or not 0 <= received_at - self.anchor_received_at <= self.MAX_ANCHOR_AGE_SEC
            or not 0 < (stamp - self.anchor["rgb_stamp_ns"]) / 1e9 <= self.MAX_ANCHOR_AGE_SEC
        ):
            self.hits = 0
            return
        current = {**info, **candidate, "rgb_stamp_ns": stamp}
        if not self.usable(current, config) or not self.matches_anchor(current):
            # A changed target cannot reuse the old ball's confirmation.
            self.anchor = None
            self.hits = 0
            return
        self.hits += 1
        if self.hits >= self.REQUIRED_HITS:
            self.observation = {
                **current, "detected": True, "pickup_now": False,
                "pickup_ready": False, "approach_reobserved": True,
            }
            self.observation_received_at = received_at
            # Do not refresh the strong anchor from a weak reobservation.

    def is_current(self, now):
        return bool(
            self.observation is not None
            and 0 <= now - self.observation_received_at <= self.MAX_DEPTH_AGE_SEC
            and 0 <= now - self.anchor_received_at <= self.MAX_ANCHOR_AGE_SEC
        )
