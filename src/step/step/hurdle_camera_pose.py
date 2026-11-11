"""Match hurdle angle hypotheses to the executor's settled camera posture."""

import math


class HurdleCameraPose:
    """Use commanded posture feedback; this is not a measured joint encoder."""

    HEARTBEAT_TIMEOUT_SEC = 1.0
    DOWN_SERVO_DEG = -64.0
    SERVO_TOLERANCE_DEG = 0.1

    def __init__(self):
        self.pose = None
        self.ready_stamp_ns = None
        self.heartbeat_at = None
        self.sequence = None

    def heartbeat(self, payload, now, ros_now):
        sequence = payload.get("sequence")
        if type(sequence) is not int or sequence < 0:
            return
        if sequence == self.sequence:
            return
        continuous = (
            self.sequence is not None and sequence > self.sequence
            and self.heartbeat_at is not None
            and 0 <= now - self.heartbeat_at <= self.HEARTBEAT_TIMEOUT_SEC
        )
        pose = None
        if payload.get("auto_ready") is True:
            if payload.get("hurdle_head_override_active") is True:
                current = payload.get("hurdle_head_override_current_deg")
                if (type(current) in {int, float} and math.isfinite(current)
                        and abs(current - self.DOWN_SERVO_DEG) <= self.SERVO_TOLERANCE_DEG):
                    pose = "0"
            elif all(payload.get(key) is False for key in (
                "hurdle_head_override_active", "ball_head_override_active",
                "goal_head_override_active",
            )):
                # The ordinary approach posture is calibrated as angle@45 (-34 deg).
                pose = "45"
        if (not continuous or pose != self.pose or type(ros_now) is not int
                or self.ready_stamp_ns is None or ros_now < self.ready_stamp_ns):
            self.ready_stamp_ns = ros_now if pose is not None and type(ros_now) is int else None
        self.pose = pose
        self.heartbeat_at = now
        self.sequence = sequence

    @staticmethod
    def capture_stamp_ns(info):
        stamp = info.get("stamp")
        if not isinstance(stamp, dict):
            return None
        sec, nanosec = stamp.get("sec"), stamp.get("nanosec")
        if (type(sec) is not int or sec < 0 or type(nanosec) is not int
                or not 0 <= nanosec < 1_000_000_000):
            return None
        return sec * 1_000_000_000 + nanosec

    def annotate(self, info, now):
        stamp = self.capture_stamp_ns(info)
        ready = (
            self.pose is not None and self.ready_stamp_ns is not None
            and stamp is not None and stamp > self.ready_stamp_ns
            and self.heartbeat_at is not None
            and 0 <= now - self.heartbeat_at <= self.HEARTBEAT_TIMEOUT_SEC
            # A request can arrive before the executor publishes its moving state.
            and not (self.pose == "45" and info.get("head_down_requested") is True)
        )
        return {**info, "hurdle_camera_pose": self.pose, "hurdle_camera_ready": ready}
