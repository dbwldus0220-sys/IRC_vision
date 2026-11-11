"""Remember a downward image exit for one retreat before camera lowering."""

import math


class BallBottomLoss:
    ACTION = 'BALL_LOST_HEAD_UP_BACKWARD_1'
    MAX_FRAME_GAP_NS = 350_000_000
    MAX_DEPTH_MEMORY_NS = 700_000_000
    MAX_LOSS_MEMORY_NS = 2_000_000_000
    BOTTOM_EDGE_RATIO = .95
    MIN_CENTER_RATIO = .75
    HEAD_DOWN_DEPTH_M = .550

    def __init__(self):
        self.camera_up = False
        self.heartbeat_at = None
        self.reset()

    def reset(self):
        self.last_stamp = None
        self.depth_stamp = None
        self.previous = None
        self.bottom_exit = False
        self.loss_stamp = None
        self.state = 'IDLE'
        self.completed_stamp = None

    def heartbeat(self, payload, now):
        self.camera_up = all(payload.get(key) is False for key in (
            'ball_head_override_active', 'goal_head_override_active',
            'hurdle_head_override_active',
        ))
        self.heartbeat_at = now
        if not self.camera_up:
            self.reset()

    @staticmethod
    def number(value):
        return (float(value) if type(value) in (int, float)
                and math.isfinite(value) else None)

    def observe(self, info, *, active, min_confidence):
        if not active or not self.camera_up:
            self.reset()
            return
        stamp = info.get('rgb_stamp_ns')
        if type(stamp) is not int or stamp <= 0:
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        if self.completed_stamp is not None and stamp <= self.completed_stamp:
            return
        self.last_stamp = stamp
        visible = info.get('detected') is True or info.get('raw_detected') is True
        if not visible:
            if self.loss_stamp is None:
                self.loss_stamp = stamp
                if self.previous is None or stamp - self.previous[0] > self.MAX_FRAME_GAP_NS:
                    self.bottom_exit = False
            return
        if self.state == 'PENDING':
            return
        confirmed = info.get('detected') is True and info.get('confirmation_confirmed') is not False
        if confirmed:
            # Only a new confirmed target may authorize another retreat.
            self.state = 'IDLE'
        self.loss_stamp = None
        candidate = info.get('approach_candidate')
        current = {**info, **candidate} if isinstance(candidate, dict) else info
        confidence = self.number(info.get('confidence'))
        depth = self.number(current.get('distance_m'))
        age = self.number(current.get('depth_age_sec'))
        if (current.get('head_down_requested') is True
                or depth is not None and depth <= self.HEAD_DOWN_DEPTH_M):
            self.depth_stamp = None
            self.bottom_exit = False
            self.previous = None
            return
        if (confirmed and confidence is not None and confidence >= min_confidence
                and current.get('depth_valid') is True and depth is not None
                and depth > self.HEAD_DOWN_DEPTH_M and age is not None and 0 <= age <= .70):
            self.depth_stamp = stamp
        width, height = self.number(info.get('image_width')), self.number(info.get('image_height'))
        box = info.get('bbox')
        if (confidence is None or confidence < min_confidence or not width or not height
                or not isinstance(box, (tuple, list)) or len(box) != 4):
            self.previous = None
            self.bottom_exit = False
            return
        values = [self.number(v) for v in box]
        if any(v is None for v in values):
            self.previous = None
            self.bottom_exit = False
            return
        left, top, right, bottom = values
        if not (0 <= left < right <= width and 0 <= top < bottom <= height):
            self.previous = None
            self.bottom_exit = False
            return
        x, y = (left + right) / (2 * width), (top + bottom) / (2 * height)
        old = self.previous
        self.bottom_exit = bool(
            old is not None and 0 < stamp - old[0] <= self.MAX_FRAME_GAP_NS
            and (width, height) == old[3:] and abs(x - old[1]) <= .10
            and (.01 <= y - old[2] <= .25
                 or self.bottom_exit and -.02 <= y - old[2] <= .25)
            and y >= self.MIN_CENTER_RATIO and bottom / height >= self.BOTTOM_EDGE_RATIO
            and self.depth_stamp is not None and stamp - self.depth_stamp <= self.MAX_DEPTH_MEMORY_NS
        )
        self.previous = (stamp, x, y, width, height)

    def eligible(self, info, now, heartbeat_timeout):
        stamp = info.get('rgb_stamp_ns')
        return bool(
            self.camera_up and self.heartbeat_at is not None
            and 0 <= now - self.heartbeat_at <= heartbeat_timeout
            and info.get('detected') is False and info.get('raw_detected') is not True
            and info.get('ball_loss_confirmed') is True
            and self.state == 'IDLE' and self.bottom_exit
            and type(stamp) is int and self.loss_stamp is not None
            and 0 <= stamp - self.loss_stamp <= self.MAX_LOSS_MEMORY_NS
        )

    def complete(self, stamp, succeeded):
        self.state = 'COMPLETE' if succeeded else 'FAILED'
        self.completed_stamp = stamp
        self.previous = None
        self.bottom_exit = False
        self.loss_stamp = None
