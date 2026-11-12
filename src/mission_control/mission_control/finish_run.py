"""Latched finish-line walking with at most one reserved six-cycle gait."""

from math import isfinite

from step.approach_distance import HURDLE_FINE_DISTANCE_M


class FinishRun:
    PHASE = "HURDLE_FINISH_RUN"
    ACTION = "STRAIGHT_3"
    MOTION = "line_forward_6"
    STATUS_TIMEOUT_SEC = 24.0

    def __init__(self):
        self.latched = False
        self.clear_seen = False
        self.fault = None
        self.current = None
        self.queued = None
        self.running = False
        self.last_status_at = None
        self.completed_batches = 0

    @staticmethod
    def armed(manager):
        return manager.required_hurdles >= 3 and manager.hurdles_completed >= 2

    @staticmethod
    def near(info, min_confidence):
        def number(key):
            value = info.get(key)
            return value if type(value) in {int, float} and isfinite(value) else None

        depth, age, confidence = (number(key) for key in ("depth_m", "depth_age_sec", "confidence"))
        return (all(info.get(key) is True for key in
                    ("detected", "confirmation_confirmed", "depth_valid"))
                and depth is not None and 0 < depth <= HURDLE_FINE_DISTANCE_M
                and age is not None and 0 <= age <= .7
                and confidence is not None and confidence >= min_confidence)

    def can_send(self, now):
        if self.current is not None and now - self.last_status_at > self.STATUS_TIMEOUT_SEC:
            self.fault = "finish_motion_status_timeout"
        return (self.latched and self.fault is None and self.queued is None
                and (self.current is None or self.running))

    def sent(self, command_id, now):
        if self.current is None:
            self.current = command_id
            self.running = False
            self.last_status_at = now
        else:
            self.queued = command_id

    def observe(self, payload, now):
        command_id = payload.get("command_id")
        if (type(command_id) is not int or command_id not in {self.current, self.queued}
                or payload.get("action") != self.ACTION or payload.get("event_id") is not None
                or payload.get("request_id", command_id) != command_id):
            return False
        status = payload.get("status")
        if status in {"FAILED", "TIMEOUT", "REJECTED", "CANCELED", "CANCELLED", "UNSUPPORTED"}:
            self.fault = payload.get("error_code") or status
        elif payload.get("motion_id") == self.MOTION:
            if command_id == self.current and status == "RUNNING":
                self.running = True
                self.last_status_at = now
            elif command_id == self.current and status == "SUCCEEDED":
                self.completed_batches += 1
                self.current, self.queued = self.queued, None
                self.running = False
                self.last_status_at = now
        return True
