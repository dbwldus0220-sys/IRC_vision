def _prepare_pending_ball_entry_walk(
        self, decision: MotionDecision, observations: dict, now: float,
    ) -> None:
        """Queue Ball-owned gaits from fresh Ball captures without requiring Line."""
        started = self.active_line_motion_started_at
        capture_start = (started + self.active_line_motion_duration_sec
                         * MotionDecisionNode.LINE_MOTION_CAPTURE_START_RATIO
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
        if self.approach_walk_capture_count >= MotionDecisionNode.APPROACH_WALK_CAPTURE_FRAMES:
            MotionDecisionNode._publish_decision(self, decision, queue_while_locked=True)

def _walking_prequeue_allowed(self, decision: MotionDecision, observations: dict, now: float) -> bool:
        if decision.source_command.get("goal_crab_prequeue") is True:
            action = self.general_motion_gate.active_action
            info = observations.get("goal")
            duration = MotionDecisionNode.GOAL_CRAB_DURATION_SEC.get(action)
            started = getattr(self, "goal_forward_started_at", None)
            return bool(
                self.mission_phase == "GOAL_APPROACH" and self.active_general_source == "goal"
                and self.general_motion_gate.locked and self.active_special_command_id is None
                and getattr(self, "queued_general_command_id", None) is None
                and decision.source == "goal" and decision.valid and not decision.requires_ack
                and duration is not None and started is not None and now >= started + duration * .70
                and decision.action == action and self.goal_forward_capture_action == action
                and self.goal_forward_capture_count >= MotionDecisionNode.GOAL_FORWARD_CAPTURE_FRAMES
                and decision.source_command.get("goal_crab_running_command_id") == self.general_motion_gate.active_command_id
                and self.planner._is_detected_goal(info) and info.get("confirmation_confirmed") is not False
                and self.planner._plan_source("goal", info, 0.0).get("action") == action
            )
        if decision.source_command.get("post_shot_exit_prequeue") is True:
            preview = MotionDecisionNode._post_shot_exit_preview(self, now)
            return bool(self.general_motion_gate.locked and self.active_general_source == "line"
                        and self.general_motion_gate.active_action == "POST_SHOT_FORWARD"
                        and getattr(self, "queued_general_command_id", None) is None
                        and MotionDecisionNode._is_continuous_line_action(decision.action)
                        and preview is not None and preview.action == decision.action)
        if decision.source_command.get("line_turn_prequeue") is True:
            return MotionDecisionNode._line_turn_queue_ready(self, decision, observations, now)
        if decision.source_command.get("ball_fine_alignment") is True:
            command = MotionDecisionNode._ball_fine_preview(self, observations, now)
            capture = getattr(self, "ball_fine_capture", None)
            required = 1 if self.general_motion_gate.active_action in BALL_FINE_PREPARATIONS else 3
            return bool(decision.source == "ball" and decision.valid and not decision.requires_ack
                        and getattr(self, "queued_general_command_id", None) is None
                        and command and command.get("valid") and command["motion"] == decision.action
                        and MotionDecisionNode._ball_fine_commit_ready(self, observations.get("ball") or {}, now)
                        and capture and capture[0] == self.general_motion_gate.active_command_id
                        and capture[1] == decision.action and capture[3] >= required)
        if self.mission_phase == "LINE_TRACK_AFTER_PICKUP" and decision.source == "line":
            if not (decision.valid and not decision.requires_ack
                        and MotionDecisionNode._post_pickup_line_action(decision.action)
                        and MotionDecisionNode._post_pickup_line_action(self.general_motion_gate.active_action)
                        and getattr(self, "queued_general_command_id", None) is None
                        and MotionDecisionNode._post_pickup_line_capture_ready(self, now)
                        and MotionDecisionNode._line_only_prequeue_allowed(self, observations)
                        and MotionDecisionNode._line_prequeue_fits_deadline(self, decision, now)):
                return False
        if decision.source_command.get("post_shot_forward_prequeue") is True:
            return bool(decision.valid and not decision.requires_ack
                        and decision.source == "line" and decision.action == "POST_SHOT_FORWARD"
                        and MotionDecisionNode._post_shot_forward_preview(self, now) is not None)
        input_source = MotionDecisionNode._walking_observation_source(decision)
        if MotionDecisionNode._ball_entry_line_active(self):
            entry = self.ball_entry_line_alignment
            if (decision.source != "line" or not decision.valid or decision.requires_ack
                    or not decision.source_command.get("ball_entry_line_alignment")
                    or not MotionDecisionNode._is_continuous_line_action(decision.action)
                    or getattr(self, "ball_pickup_entry_pending", False)
                    or not MotionDecisionNode._recent_walking_input(self, "ball", now)
                    or (input_source == "line" and observations.get("line") is None)):
                return False
            preview = entry.select(self.planner, observations.get("line"), 0.0,
                                   observations.get("ball"))
            if (not preview.valid or preview.action != decision.action
                    or MotionDecisionNode._walking_observation_source(preview) != input_source):
                return False
            if input_source == "ball" and (
                    getattr(self, "approach_walk_candidate", None) != decision.action
                    or getattr(self, "approach_walk_capture_count", 0)
                    < MotionDecisionNode.APPROACH_WALK_CAPTURE_FRAMES):
                return False
        if getattr(self, "queued_general_command_id", None) is not None:
            return False
        if decision.source_command.get("post_ball_goal_handoff") is True:
            preview = getattr(self, "post_ball_goal_preview", None) or {}
            return bool(
                self.mission_phase == "LINE_TRACK_AFTER_PICKUP"
                and self.active_general_source == "line"
                and self.general_motion_gate.locked
                and MotionDecisionNode._post_pickup_line_action(self.general_motion_gate.active_action)
                and MotionDecisionNode._post_ball_line_handoff_due(self, now)
                and not getattr(self, "post_ball_line_run_failed", False)
                and self.phase_manager.grasp_result_for_ball(self.phase_manager.completed_ball_index) == "GRABBED"
                and getattr(self, "post_ball_goal_camera_ready_at", None) is not None
                and self.active_special_command_id is None
                and decision.source == "goal" and decision.valid and not decision.requires_ack
                and MotionDecisionNode._post_pickup_goal_action(decision.action)
                and preview.get("valid") and preview.get("action") == decision.action
                and getattr(self, "post_ball_goal_capture_count", 0) >= MotionDecisionNode.GOAL_FORWARD_CAPTURE_FRAMES
                and self.planner._is_detected_goal(observations.get("goal"))
                and observations["goal"].get("confirmation_confirmed") is not False
                and self.planner._plan_source("goal", observations["goal"], 0.0).get("action") == decision.action
            )
        if decision.source in {"ball", "hurdle"}:
            if not decision.valid or decision.requires_ack:
                return False
            command = MotionDecisionNode._approach_walk_preview(self, observations, now)
            key = "motion" if decision.source == "ball" else "action"
            return bool((decision.source == self.active_general_source
                         or decision.source == "ball" and MotionDecisionNode._ball_entry_line_running(self)) and command
                        and command.get(key) == decision.action)
        if decision.source == "line":
            recovery_chain = all(
                action in {"LEFT", "RIGHT"} or bool(action and action.startswith("RECOVER_"))
                for action in (self.general_motion_gate.active_action, decision.action)
            )
            if recovery_chain:
                started_at = self.active_line_motion_started_at
                duration = self.active_line_motion_duration_sec
                if (started_at is None or duration <= 0.0
                        or now < started_at + duration - MotionDecisionNode.LINE_RECOVERY_PREQUEUE_LEAD_SEC):
                    return False
                # Receipt time alone can admit an image taken before the turn
                # had an effect. Require capture within the late-motion window.
                info = observations.get(input_source) or {}
                stamp = MotionDecisionNode._detection_stamp_ns(info)
                if stamp is None:
                    stamp = info.get("rgb_stamp_ns")
                ros_now = MotionDecisionNode._current_ros_time_ns(self)
                capture_start = started_at + duration * MotionDecisionNode.LINE_MOTION_CAPTURE_START_RATIO
                if (type(stamp) is not int or type(ros_now) is not int
                        or not 0 <= ros_now - stamp <= int(max(0.0, now - capture_start) * 1e9)):
                    return False
            return (MotionDecisionNode._line_only_prequeue_allowed(self, observations)
                    and MotionDecisionNode._line_prequeue_fits_deadline(self, decision, now))
        if (decision.source != "goal" or self.mission_phase != "GOAL_APPROACH"
                or self.active_general_source != "goal"
                or not MotionDecisionNode._is_continuous_goal_action(self.general_motion_gate.active_action)
                or not MotionDecisionNode._is_continuous_goal_action(decision.action)
                or not decision.valid or decision.requires_ack
                or self.active_special_command_id is not None):
            return False
        info = observations.get("goal")
        return bool(self.planner._is_detected_goal(info)
                    and info.get("confirmation_confirmed") is not False
                    and self.planner._plan_source("goal", info, 0.0).get("action") == decision.action)
