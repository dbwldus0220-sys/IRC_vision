"""Retain advisory geometry for initial Ball entry without granting motion."""

from step.ball_navigation_planner import valid_ball_ground_steering
from step.approach_distance import BALL_FINE_APPROACH_DISTANCE_M
from .ball_approach_observation import BallApproachObservation


class BallEntryCandidateHistory:
    def __init__(self):
        self.matcher = BallApproachObservation()
        self.reset()

    def reset(self):
        self.key = None
        self.frames = []

    def observe(self, info, *, command_id, capture_after_ns, ros_now_ns, config):
        track_id = info.get('confirmation_track_id')
        if type(track_id) is not int or track_id < 0:
            self.reset()
            return
        key = (command_id, track_id)
        if key != self.key:
            self.reset()
            self.key = key
        confirmed = info.get('detected') is True and info.get('confirmation_confirmed') is True
        candidate = info if confirmed else info.get('approach_candidate')
        if (info.get('raw_detected') is not True or not isinstance(candidate, dict)
                or info.get('confirmation_spatial_pending') is True):
            self.reset()
            return
        current = {**info, **candidate}
        stamp = info.get('rgb_stamp_ns')
        if (type(stamp) is not int or type(ros_now_ns) is not int
                or stamp < capture_after_ns
                or not 0 <= ros_now_ns - stamp <= int(self.matcher.MAX_RGB_CAPTURE_AGE_SEC * 1e9)
                or not self.matcher.usable(current, config)
                or current['distance_m'] <= BALL_FINE_APPROACH_DISTANCE_M
                or valid_ball_ground_steering(current) is None
                or (self.matcher.number(current, 'ground_forward_distance_m') or 0.) <= 0.):
            self.reset()
            return
        if self.frames and stamp <= self.frames[-1]['rgb_stamp_ns']:
            return
        self.frames = [f for f in self.frames
                       if 0 <= ros_now_ns - f['rgb_stamp_ns'] <= int(self.matcher.MAX_ANCHOR_AGE_SEC * 1e9)]
        if self.frames and (not self.matcher.matches_anchor(current, anchor=self.frames[-1])
                            or not self.matcher.matches_anchor(current, anchor=self.frames[0], cumulative=True)):
            self.frames = []
        # These flags are used only for private previews; never publish a candidate as confirmed.
        current.update(rgb_stamp_ns=stamp, detected=True, confirmation_confirmed=True)
        self.frames = (self.frames + [current])[-3:]

    def take(self, info, command_id):
        frames = self.frames
        matches = (self.key == (command_id, info.get('confirmation_track_id'))
                   and info.get('detected') is True and info.get('confirmation_confirmed') is True
                   and frames and frames[-1]['rgb_stamp_ns'] == info.get('rgb_stamp_ns'))
        self.reset()
        # The caller adds the confirming frame itself exactly once.
        return frames[:-1] if matches else []
