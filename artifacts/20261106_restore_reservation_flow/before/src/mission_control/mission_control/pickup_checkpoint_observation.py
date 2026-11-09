"""Prepare pickup checkpoint decisions without queueing posture changes."""

from copy import copy
import math

from step.approach_distance import BALL_PICKUP_DISTANCE_M

from .pickup_walk_reservation import PICKUP_WALK_DURATIONS, pickup_walk_motion


class PickupCheckpointObservation:
    """Observation only: never reserve an SDK motion or hold a checkpoint lock."""

    CAPTURE_RATIO = .60
    COMMIT_RATIO = .70
    REQUIRED_FRAMES = 3

    def __init__(self):
        self.reset()

    def reset(self):
        self.key = None
        self.started_at = None
        self.duration = None
        self.capture_after_ns = None
        self.commit_after_ns = None
        self.stage = None
        self.candidate = None
        self.completed = False
        self.last_stamp = None
        self.count = 0
        self.sample = None
        self.received_at = None

    def on_status(self, payload, now, ros_now, max_age=.5):
        if self.completed_sample(payload, now, ros_now, max_age) is not None:
            # Initial alignment can enter fine alignment without physical motion.
            self.completed = True
            return
        motion = payload.get('motion_id')
        if (payload.get('status') != 'RUNNING' or motion not in PICKUP_WALK_DURATIONS
                or payload.get('pickup_alignment_stage') not in {'initial', 'fine'}):
            self.reset()
            return
        key = (payload.get('command_id'), payload.get('event_id'),
               payload.get('request_id'), motion)
        if key == self.key and not self.completed:
            return
        self.reset()
        self.key = key
        self.stage = payload['pickup_alignment_stage']
        self.started_at = now
        self.duration = PICKUP_WALK_DURATIONS[motion]
        if type(ros_now) is int:
            self.capture_after_ns = ros_now + int(PICKUP_WALK_DURATIONS[motion] * self.CAPTURE_RATIO * 1e9)
            self.commit_after_ns = ros_now + int(PICKUP_WALK_DURATIONS[motion] * self.COMMIT_RATIO * 1e9)

    def begin_entry(self, payload, now, ros_now, duration):
        """Remember the owning approach gait; release entry only on its success."""
        key = (payload.get('command_id'), payload.get('event_id'),
               payload.get('request_id'), payload.get('motion_id'))
        if key == self.key:
            return
        self.reset()
        self.key, self.stage, self.started_at, self.duration = key, 'entry', now, duration
        if type(ros_now) is int:
            self.capture_after_ns = ros_now + int(duration * self.CAPTURE_RATIO * 1e9)
            self.commit_after_ns = ros_now + int(duration * self.COMMIT_RATIO * 1e9)

    def observe(self, planner, info, now, ros_now, max_age, *, image_stamp=None):
        if self.key is None or self.completed:
            if self.completed:
                self.reset()
            return
        self.sample = None
        if self.received_at is not None and now - self.received_at > max_age:
            self.count = 0
        self.received_at = now
        number = planner._number
        distance = number(info, 'distance_m')
        depth_age = number(info, 'depth_age_sec')
        confidence = number(info, 'confidence')
        stamp = image_stamp if image_stamp is not None else info.get('rgb_stamp_ns')
        if (info.get('detected') is not True or info.get('confirmation_confirmed') is False
                or info.get('depth_valid') is not True
                or distance is None or not 0 < distance <= BALL_PICKUP_DISTANCE_M
                or depth_age is None or not 0 <= depth_age <= planner.ball_planner.config.max_pickup_depth_age_sec
                or confidence is None or confidence < planner.ball_planner.config.min_confidence
                or type(stamp) is not int or type(ros_now) is not int
                or self.capture_after_ns is None or stamp < self.capture_after_ns
                or now < self.started_at + self.duration * self.CAPTURE_RATIO
                or not 0 <= ros_now - stamp <= int(max_age * 1e9)
                or self.last_stamp is not None and stamp <= self.last_stamp):
            self.count = 0
            return
        self.last_stamp = stamp
        preview = copy(planner)
        preview.pickup_last_completed_motion_id = self.key[3]
        decision = (preview.plan_ball_pickup_initial_alignment(info) if self.stage in {'initial', 'entry'}
                    else preview.plan_ball_pickup_fine_alignment(info))
        fine_action = None
        if self.stage == 'initial':
            fine = copy(planner)
            fine.pickup_last_completed_motion_id = self.key[3]
            fine_action = fine.plan_ball_pickup_fine_alignment(info).action
        if not decision.valid or decision.requires_ack:
            self.count = 0
            return
        # Also confirm the finer decision when two checkpoints have no motion between them.
        candidate = (decision.action, pickup_walk_motion(decision.action, self.key[3], decision.source_command),
                     decision.source_command.get('pickup_approach_motion'),
                     fine_action,
                     info.get('head_down_requested'))
        if candidate != self.candidate:
            self.count = 0
        self.candidate = candidate
        self.count += 1
        if (self.count >= self.REQUIRED_FRAMES and stamp >= self.commit_after_ns
                and now >= self.started_at + self.duration * self.COMMIT_RATIO):
            self.sample = dict(info)

    def completed_sample(self, payload, now, ros_now, max_age):
        """The bridge reports completed_motion_id only for a successful correction."""
        key = (payload.get('command_id'), payload.get('event_id'),
               payload.get('request_id'), payload.get('completed_motion_id'))
        if self.stage == 'entry':
            key = (*key[:3], payload.get('motion_id'))
        # Rest credit is emitted only after an exactly matched physical success.
        elapsed = payload.get('stationary_elapsed_sec')
        checkpoint = (payload.get('status') == 'SUCCEEDED' if self.stage == 'entry' else
                      payload.get('status') == 'RUNNING'
                      and payload.get('motion_id') in ({'__BALL_PICKUP_INITIAL_ALIGN_CHECK__', '__BALL_PICKUP_FINE_ALIGN_CHECK__'}
                                                      if self.stage == 'initial' else {'__BALL_PICKUP_FINE_ALIGN_CHECK__'}))
        if (checkpoint
                and self.key is not None and key == self.key and self.sample is not None
                and type(elapsed) in {int, float} and math.isfinite(elapsed) and elapsed >= 0
                and self.received_at is not None and 0 <= now - self.received_at <= max_age
                and type(ros_now) is int and self.last_stamp is not None
                and 0 <= ros_now - self.last_stamp <= int(max_age * 1e9)):
            return self.sample, self.received_at
        return None
