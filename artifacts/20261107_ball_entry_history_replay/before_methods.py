def _prepare_pending_ball_entry_walk(
        self, decision: MotionDecision, observations: dict, now: float,
    ) -> None:
        """Collect Ball captures from 60 percent; commit with one after 70 percent."""
        started = self.active_line_motion_started_at
        capture_start = (started + self.active_line_motion_duration_sec
                         * MotionDecisionNode.BALL_ENTRY_CAPTURE_START_RATIO
                         if started is not None else None)
        info = observations.get("ball") or {}
        stamp = MotionDecisionNode._detection_stamp_ns(info)
        if stamp is None:
            stamp = info.get("rgb_stamp_ns")
        ros_now = MotionDecisionNode._current_ros_time_ns(self)
        if (not self.general_motion_gate.locked or capture_start is None or now < capture_start
                or getattr(self, "queued_general_command_id", None) is not None
                or not decision.valid or decision.requires_ack
                or not MotionDecisionNode._is_continuous_line_action(decision.action)
                or not MotionDecisionNode._is_continuous_line_action(self.general_motion_gate.active_action)
                or getattr(self, "ball_pickup_entry_pending", False)
                or not MotionDecisionNode._recent_walking_input(self, "ball", now)
                or type(stamp) is not int or type(ros_now) is not int
                or not 0 <= ros_now - stamp <= int((now - capture_start) * 1e9)):
            self.approach_walk_candidate = None
            self.approach_walk_capture_count = 0
            self.ball_entry_walk_frames = []
            return
        last_stamp = getattr(self, "approach_walk_last_stamp_ns", None)
        if last_stamp is not None and stamp <= last_stamp:
            return
        self.approach_walk_last_stamp_ns = stamp
        frames = [frame for frame in getattr(self, "ball_entry_walk_frames", [])
                  if 0 <= ros_now - frame["rgb_stamp_ns"] <= int(
                      min(BallApproachObservation.MAX_ANCHOR_AGE_SEC, now - capture_start) * 1e9)]
        observer = getattr(self, "ball_approach_observation", None)
        changed_target = bool(frames and observer is not None
                              and info.get("bbox") and frames[-1].get("bbox")
                              and not observer.matches_anchor(info, anchor=frames[-1]))
        if decision.action != getattr(self, "approach_walk_candidate", None) or changed_target:
            self.approach_walk_candidate = decision.action
            frames = []
        self.ball_entry_walk_frames = (frames + [dict(info)])[-3:]
        self.approach_walk_capture_count = len(self.ball_entry_walk_frames)
        if (self.approach_walk_capture_count >= MotionDecisionNode.APPROACH_WALK_CAPTURE_FRAMES
                and MotionDecisionNode._ball_entry_walk_commit_ready(self, info, now)):
            MotionDecisionNode._publish_decision(self, decision, queue_while_locked=True)
