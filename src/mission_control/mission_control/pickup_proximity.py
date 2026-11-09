"""Confirm visible-ball proximity using distinct images after head interpolation."""

import math


class PickupProximity:
    ENTER_PX = 100.0
    RELEASE_PX = 120.0
    REQUIRED_FRAMES = 3

    def __init__(self):
        self.reset()

    def reset(self):
        self.head_id = None
        self.head_ready_stamp = None
        self.heartbeat_at = None
        self.reset_observations()

    def reset_observations(self):
        self.last_stamp = None
        self.close_count = 0
        self.release_count = 0
        self.close_active = False
        self.retreat_sent = False

    def heartbeat(self, payload, now, ros_now):
        head_id = payload.get('ball_head_transition_id')
        stamp = payload.get('ball_head_transition_complete_stamp_ns')
        ready = (payload.get('ball_head_override_active') is True
                 and payload.get('ball_head_transition_complete') is True
                 and type(head_id) is int and type(stamp) is int
                 and type(ros_now) is int and 0 < stamp <= ros_now)
        if head_id != self.head_id or not ready:
            self.reset_observations()
            self.head_ready_stamp = None
        self.head_id = head_id
        self.heartbeat_at = now
        if ready and self.head_ready_stamp is None:
            self.head_ready_stamp = stamp

    def head_ready(self, stamp, now, heartbeat_timeout):
        return (self.head_ready_stamp is not None and type(stamp) is int
                and stamp > self.head_ready_stamp and self.heartbeat_at is not None
                and 0 <= now - self.heartbeat_at <= heartbeat_timeout)

    def observe(self, info, stamp, ros_now, now, *, max_age, heartbeat_timeout, min_confidence):
        ready = self.head_ready(stamp, now, heartbeat_timeout)
        info['pickup_head_ready'] = ready
        bottom = info.get('bottom_distance_px')
        confidence = info.get('confidence')
        valid = (ready and type(ros_now) is int
                 and 0 <= ros_now - stamp <= int(max_age * 1e9)
                 and info.get('detected') is True
                 and info.get('confirmation_confirmed') is True
                 and type(bottom) in {int, float} and math.isfinite(bottom) and bottom >= 0
                 and type(confidence) in {int, float} and math.isfinite(confidence)
                 and confidence >= min_confidence)
        if not valid:
            self.close_count = self.release_count = 0
            info['pickup_proximity'] = 'HOLD' if self.close_active else 'UNAVAILABLE'
            return
        if self.last_stamp is None or stamp > self.last_stamp:
            if self.last_stamp is not None and stamp - self.last_stamp > int(max_age * 1e9):
                self.close_count = self.release_count = 0
            self.last_stamp = stamp
            self.close_count = self.close_count + 1 if bottom < self.ENTER_PX else 0
            self.release_count = self.release_count + 1 if bottom >= self.RELEASE_PX else 0
            if self.release_count >= self.REQUIRED_FRAMES:
                self.close_active = self.retreat_sent = False
            elif self.close_count >= self.REQUIRED_FRAMES:
                self.close_active = True
        # Clearance for alignment and permission to retreat again are distinct.
        # Keep the retreat latch until RELEASE_PX, but do not freeze legal crabs
        # or pickup continuation in the 100..119 px hysteresis band.
        if bottom >= self.ENTER_PX:
            info['pickup_proximity'] = 'CLEAR'
        elif self.close_active:
            info['pickup_proximity'] = (
                'RETREAT' if not self.retreat_sent and bottom < self.ENTER_PX else 'HOLD')
        else:
            info['pickup_proximity'] = 'PENDING' if bottom < self.ENTER_PX else 'CLEAR'

    def mark_retreat_sent(self):
        self.retreat_sent = True
        self.close_active = True
        self.close_count = self.release_count = 0
