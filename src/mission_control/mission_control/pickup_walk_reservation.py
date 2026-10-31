"""Reserve only identical pickup gaits from fresh late-motion Ball frames."""

from copy import copy
from dataclasses import replace

from step.approach_distance import BALL_PICKUP_DISTANCE_M


PICKUP_WALK_DURATIONS = {
    'ball_general_fine_forward_8': .376 * 5 / .9975,
    'pickup_fine_forward_0': .376 * 5 / .9975,
    'pickup_crab_left_0': .500 / .9025,
    'pickup_crab_right_0': .540 / .9025,
}

PICKUP_WALK_ACTIONS = {
    'BALL_PICKUP_FINE_FORWARD': 'pickup_fine_forward_0',
    'BALL_PICKUP_CRAB_LEFT': 'pickup_crab_left_0',
    'BALL_PICKUP_CRAB_RIGHT': 'pickup_crab_right_0',
    'BALL_PICKUP_INITIAL_CRAB_LEFT': 'pickup_crab_left_0',
    'BALL_PICKUP_INITIAL_CRAB_RIGHT': 'pickup_crab_right_0',
}


def pickup_walk_motion(action, current):
    """Retain the legacy first-step alias; both fine IDs resolve to the same gait."""
    selected = PICKUP_WALK_ACTIONS.get(action)
    if selected == 'pickup_fine_forward_0' and current == 'ball_general_fine_forward_8':
        return current
    return selected


class PickupWalkReservation:
    """One reservation per running gait; turns and posture changes never queue."""

    CAPTURE_RATIO = .70
    REQUIRED_FRAMES = 3

    def __init__(self):
        self.reset()

    def reset(self):
        self.key = None
        self.motion_id = None
        self.stage = None
        self.capture_after_ns = None
        self.started_at = None
        self.last_stamp = None
        self.count = 0
        self.pending = False
        self.forward_complete = False
        self.ready = None
        self.observed_at = None

    def on_status(self, payload, now, ros_now):
        motion = payload.get('motion_id')
        if (payload.get('status') != 'RUNNING' or motion not in PICKUP_WALK_DURATIONS
                or payload.get('pickup_alignment_stage') not in {'initial', 'fine'}):
            self.reset()
            return
        key = (payload.get('command_id'), payload.get('request_id'), motion)
        if key == self.key:
            return
        self.reset()
        self.key = key
        self.motion_id = motion
        self.stage = payload['pickup_alignment_stage']
        self.started_at = now
        if type(ros_now) is int:
            self.capture_after_ns = ros_now + int(PICKUP_WALK_DURATIONS[motion] * self.CAPTURE_RATIO * 1e9)

    def observe(self, planner, info, now, ros_now, max_age, image_stamp=None):
        self.ready = None
        if self.observed_at is not None and now - self.observed_at > max_age:
            self.count = 0
        self.observed_at = now
        if self.key is None or self.pending:
            return
        number = planner._number
        distance = number(info, 'distance_m')
        age = number(info, 'depth_age_sec')
        confidence = number(info, 'confidence')
        stamp = image_stamp if image_stamp is not None else (info or {}).get('rgb_stamp_ns')
        if (not info or info.get('detected') is not True
                or info.get('confirmation_confirmed') is False
                or info.get('depth_valid') is not True
                or distance is None or not 0 < distance <= BALL_PICKUP_DISTANCE_M
                or age is None or not 0 <= age <= planner.ball_planner.config.max_pickup_depth_age_sec
                or confidence is None or confidence < planner.ball_planner.config.min_confidence
                or type(stamp) is not int or type(ros_now) is not int
                or self.capture_after_ns is None or stamp < self.capture_after_ns
                or now < self.started_at + PICKUP_WALK_DURATIONS[self.motion_id] * self.CAPTURE_RATIO
                or not 0 <= ros_now - stamp <= int(max_age * 1e9)):
            self.count = 0
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        bottom = number(info, 'bottom_distance_px')
        if bottom is not None and 0 <= bottom <= planner.config.pickup_fine_align_bottom_distance_px:
            self.forward_complete = True
        # Preview must not latch completion or advance loss recovery on the real planner.
        preview = copy(planner)
        decision = (preview.plan_ball_pickup_initial_alignment(info) if self.stage == 'initial'
                    else preview.plan_ball_pickup_fine_alignment(info))
        if (not decision.valid or pickup_walk_motion(decision.action, self.motion_id) != self.motion_id
                or self.motion_id in {'pickup_fine_forward_0', 'ball_general_fine_forward_8'}
                and self.forward_complete):
            self.count = 0
            return
        self.count += 1
        if self.count >= self.REQUIRED_FRAMES:
            self.ready = replace(decision, reason='pickup_same_gait_reserved_from_late_ball_frames',
                                 source_command={**decision.source_command,
                                                 'pickup_walk_prequeue': True,
                                                 'pickup_running_request_id': self.key[1],
                                                 'pickup_queue_motion_id': self.motion_id,
                                                 'pickup_queue_frame_count': self.count})

    def decision(self, now, max_age):
        if (not self.pending and self.ready is not None and self.observed_at is not None
                and 0 <= now - self.observed_at <= max_age):
            return self.ready
        return None

    def published(self):
        self.pending = True
        self.ready = None
