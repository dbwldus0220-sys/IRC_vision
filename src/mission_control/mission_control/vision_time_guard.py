"""Validate image age without rewriting camera capture timestamps."""


def capture_is_fresh(stamp_ns, ros_now_ns, max_age_sec, minimum_stamp_ns=None):
    return (
        type(stamp_ns) is int and stamp_ns > 0
        and type(ros_now_ns) is int
        and (minimum_stamp_ns is None or stamp_ns > minimum_stamp_ns)
        and 0 <= ros_now_ns - stamp_ns <= int(max_age_sec * 1e9)
    )


class VisionTimeGuard:
    # Compare elapsed ROS and steady time, so callback delays are not jumps.
    JUMP_THRESHOLD_SEC = 0.1
    RECOVERY_FRAMES = 3
    RECOVERY_STABLE_SEC = 0.2

    def __init__(self):
        self.previous = None
        self.minimum_stamp_ns = None
        self.recovering = False
        self.recovery_started_at = None
        self.frames = {}

    def check_clock(self, ros_now_ns, now):
        if type(ros_now_ns) is not int:
            return None
        previous = self.previous
        self.previous = (ros_now_ns, now)
        if previous is None:
            return None
        jump = (ros_now_ns - previous[0]) / 1e9 - (now - previous[1])
        if abs(jump) <= self.JUMP_THRESHOLD_SEC:
            return None
        self.minimum_stamp_ns = ros_now_ns
        self.recovering = True
        self.recovery_started_at = now
        self.frames.clear()
        return jump

    def accept(self, source, stamp_ns, ros_now_ns, now, max_age_sec):
        if not capture_is_fresh(stamp_ns, ros_now_ns, max_age_sec, self.minimum_stamp_ns):
            self.frames.pop(source, None)
            return False
        if not self.recovering:
            return True
        previous_stamp, count, received_at = self.frames.get(source, (None, 0, now))
        if now - received_at > max_age_sec:
            count = 0
        if previous_stamp is not None and stamp_ns <= previous_stamp:
            return False
        count += 1
        self.frames[source] = (stamp_ns, count, now)
        if (count >= self.RECOVERY_FRAMES
                and now - self.recovery_started_at >= self.RECOVERY_STABLE_SEC):
            self.recovering = False
            self.frames.clear()
            return True
        return False
