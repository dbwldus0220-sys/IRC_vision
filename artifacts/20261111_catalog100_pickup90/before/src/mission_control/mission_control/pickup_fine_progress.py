"""Accumulate approach evidence during one physical, head-down fine gait."""

from step.approach_distance import BALL_PICKUP_DISTANCE_M

FINE_MOTIONS = {'ball_general_fine_forward_8', 'pickup_fine_forward_0'}


class PickupFineProgress:
    REQUIRED_HITS = 3
    MIN_HIT_SPAN_NS = 200_000_000
    MIN_HIT_RATIO = .20
    LATE_RATIO = .60
    MAX_OFFSET_CHANGE_NORM = .18
    MAX_DISTANCE_CHANGE_M = .15

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_stamp = None
        self.reference = None
        self.valid_count = 0
        self.hit_count = 0
        self.first_hit_stamp = None
        self.complete = False

    def observe(self, planner, info, stamp, ros_now, start_ns, duration, max_age):
        number = planner._number
        distance = number(info, 'distance_m')
        age = number(info, 'depth_age_sec')
        confidence = number(info, 'confidence')
        bottom = number(info, 'bottom_distance_px')
        offset = number(info, 'offset_x_norm')
        if (info.get('detected') is not True or info.get('confirmation_confirmed') is not True
                or info.get('head_down_requested') is not True or info.get('depth_valid') is not True
                or distance is None or not 0 < distance <= BALL_PICKUP_DISTANCE_M
                or age is None or not 0 <= age <= planner.ball_planner.config.max_pickup_depth_age_sec
                or confidence is None or confidence < planner.ball_planner.config.min_confidence
                or bottom is None or bottom < 0 or offset is None
                or type(stamp) is not int or type(ros_now) is not int or start_ns is None
                or not 0 <= ros_now - stamp <= int(max_age * 1e9)):
            self.reset()
            return
        if stamp < start_ns or self.last_stamp is not None and stamp <= self.last_stamp:
            return
        if self.reference is not None:
            old_offset, old_distance = self.reference
            if (stamp - self.last_stamp > int(max_age * 1e9)
                    or abs(offset - old_offset) > self.MAX_OFFSET_CHANGE_NORM
                    or abs(distance - old_distance) > self.MAX_DISTANCE_CHANGE_M):
                self.reset()
        self.reference = offset, distance
        self.last_stamp = stamp
        self.valid_count += 1
        if bottom <= planner.config.pickup_fine_align_bottom_distance_px:
            self.hit_count += 1
            if self.first_hit_stamp is None:
                self.first_hit_stamp = stamp
            if (self.hit_count >= self.REQUIRED_HITS
                    and stamp - self.first_hit_stamp >= self.MIN_HIT_SPAN_NS
                    and stamp >= start_ns + int(duration * self.LATE_RATIO * 1e9)
                    and self.hit_count / self.valid_count >= self.MIN_HIT_RATIO):
                # Later sway alone must not undo evidence from this same gait.
                self.complete = True
