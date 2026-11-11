"""Check raw RGB/depth clocks independently of object detection and motion."""

from collections import deque

from .vision_time_guard import VisionTimeGuard, capture_is_fresh


class CameraTimeHealth:
    MIN_SYSTEM_TIME_NS = 1_577_836_800_000_000_000  # 2020-01-01; reject unset RTC.
    STABLE_SEC = 1.0
    FRAME_COUNT = 3
    MAX_AGE_SEC = 0.5
    MAX_PAIR_DELTA_SEC = 0.1
    RECOVERY_GRACE_SEC = 2.0
    STARTUP_GRACE_SEC = 15.0

    def __init__(self, now):
        self.clock = VisionTimeGuard()
        self.stable_since = now
        self.started_at = now
        self.bad_since = now
        self.minimum_stamp_ns = None
        self.frames = {}
        self.history = {source: deque(maxlen=32) for source in ('rgb', 'depth')}
        self.frame_failure = None
        self.was_ready = False

    def check_clock(self, ros_now_ns, now):
        jump = self.clock.check_clock(ros_now_ns, now)
        if jump is not None:
            self.stable_since = now
            self.bad_since = now
            self.minimum_stamp_ns = ros_now_ns
            self.frames.clear()
            for history in self.history.values():
                history.clear()
        return jump

    def restarted(self, ros_now_ns, now):
        self.started_at = now
        self.bad_since = now
        self.minimum_stamp_ns = ros_now_ns
        self.frames.clear()
        for history in self.history.values():
            history.clear()
        self.was_ready = False

    def observe(self, source, stamp_ns, ros_now_ns, now):
        self.check_clock(ros_now_ns, now)
        previous = self.frames.get(source)
        if not capture_is_fresh(stamp_ns, ros_now_ns, self.MAX_AGE_SEC, self.minimum_stamp_ns):
            # One bad packet must not erase a still-fresh accepted frame.
            return
        if previous is not None and stamp_ns <= previous[0]:
            # Repeated/backwards images cannot renew the freshness deadline.
            return
        count = previous[2] if previous and now - previous[1] <= self.MAX_AGE_SEC else 0
        self.frames[source] = (stamp_ns, now, min(count + 1, self.FRAME_COUNT))
        self.history[source].append(stamp_ns)

    def status(self, ros_now_ns, now):
        self.check_clock(ros_now_ns, now)
        self.frame_failure = None
        if type(ros_now_ns) is not int or ros_now_ns < self.MIN_SYSTEM_TIME_NS:
            self.stable_since = now
            return 'WAIT_SYSTEM_TIME', False
        if now - self.stable_since < self.STABLE_SEC:
            return 'WAIT_CLOCK_STABLE', False
        for source in ('rgb', 'depth'):
            frame = self.frames.get(source)
            if frame is None:
                self.frame_failure = source + '_missing'
            elif (not 0 <= now - frame[1] <= self.MAX_AGE_SEC
                  or not capture_is_fresh(frame[0], ros_now_ns,
                                          self.MAX_AGE_SEC, self.minimum_stamp_ns)):
                self.frame_failure = source + '_expired'
            elif frame[2] < self.FRAME_COUNT:
                self.frame_failure = source + '_warming_up'
            if self.frame_failure:
                break
        if self.frame_failure is None:
            # Independent ROS callbacks need not expose the same newest pair.
            # Reuse only genuinely fresh pairs; arrivals never renew their age.
            fresh = {
                source: [stamp for stamp in self.history[source]
                         if capture_is_fresh(stamp, ros_now_ns, self.MAX_AGE_SEC,
                                             self.minimum_stamp_ns)]
                for source in ('rgb', 'depth')
            }
            if not any(abs(rgb - depth) <= int(self.MAX_PAIR_DELTA_SEC * 1e9)
                       for rgb in fresh['rgb'] for depth in fresh['depth']):
                self.frame_failure = 'rgb_depth_unpaired'
        if self.frame_failure is None:
            self.was_ready = True
            self.bad_since = None
            return 'READY', False
        if self.bad_since is None:
            self.bad_since = now
        grace = self.RECOVERY_GRACE_SEC if self.was_ready else self.STARTUP_GRACE_SEC
        needs_restart = now - max(self.started_at, self.bad_since) >= grace
        return 'WAIT_FRAMES', needs_restart


class CameraRecoveryGate:
    """Admission after startup/recovery; counts belong to the required source."""

    STATUS_TIMEOUT_SEC = 1.0

    def __init__(self):
        self.status = {}
        self.received_at = None
        self.reobserve = True
        self.frames = {}
        self.reason = 'WAIT_CAMERA_STATUS'
        self.episode_active = False
        self.minimum_stamp_ns = None

    def raw_ready(self, now):
        return (self.received_at is not None
                and 0 <= now - self.received_at <= self.STATUS_TIMEOUT_SEC
                and self.status.get('state') == 'READY')

    def requires_recovery(self, now):
        """Reserve destructive resets for lost supervision or real recovery."""
        return (self.received_at is None
                or not 0 <= now - self.received_at <= self.STATUS_TIMEOUT_SEC
                or self.status.get('state') not in {'READY', 'WAIT_FRAMES'}
                or self.status.get('restart_needed') is True)

    def accepts_input(self, now):
        # Individual capture/depth checks still apply while raw streams catch up.
        return not self.requires_recovery(now)

    def invalidate(self, ros_now_ns):
        self.reobserve = True
        self.frames.clear()
        self.minimum_stamp_ns = ros_now_ns
        self.episode_active = True

    def observe(self, source, stamp_ns, now, *, usable):
        if not self.raw_ready(now) or not self.reobserve:
            return
        if (not usable or type(stamp_ns) is not int
                or (self.minimum_stamp_ns is not None and stamp_ns <= self.minimum_stamp_ns)):
            self.frames.pop(source, None)
            return
        previous = self.frames.get(source)
        if previous is not None and stamp_ns <= previous[0]:
            return
        count = previous[2] if previous and now - previous[1] <= .5 else 0
        self.frames[source] = (stamp_ns, now, min(count + 1, 3))

    def phase_ready(self, source, now, max_age):
        if not self.raw_ready(now):
            return False
        if not self.reobserve:
            self.reason = 'READY'
            return True
        frame = self.frames.get(source)
        if frame is None or frame[2] < 3 or not 0 <= now - frame[1] <= max_age:
            self.reason = 'WAIT_PHASE_' + source.upper()
            return False
        self.reobserve = False
        self.episode_active = False
        self.reason = 'READY'
        return True
