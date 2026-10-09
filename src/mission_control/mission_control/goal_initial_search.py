"""Two completion-driven scans after raising the camera without seeing a goal."""

from dataclasses import replace

from .motion_decision_planner import MotionDecision, MotionDecisionPlanner


class GoalInitialSearch:
    def __init__(self):
        self.last_line_side = None
        self.active = False
        self.sequence = ()
        self.step = 0
        self.command_id = None
        self.failed = False

    def observe_line(self, info, min_quality):
        """Remember the visible near point relative to the calibrated robot center."""
        if self.active or info.get("detected") is not True:
            return
        number = MotionDecisionPlanner._number
        qualities = [number(info, key) for key in (
            "heading_quality", "geometry_quality", "detection_quality",
        )]
        if any(q is None or q < min_quality for q in qualities):
            return
        offset = number(info, "lateral_offset_px")
        if offset is None:
            offset = number(info, "lateral_offset_norm")
        points = info.get("center_points_px")
        center, width = number(info, "robot_center_x_px"), number(info, "image_width")
        if (isinstance(points, list) and points and isinstance(points[0], (list, tuple))
                and len(points[0]) == 2 and center is not None and width is not None
                and 0 <= center < width):
            x = number({"x": points[0][0]}, "x")
            if x is not None and 0 <= x < width:
                offset = x - center
        if offset is not None and offset != 0:
            self.last_line_side = "RIGHT" if offset > 0 else "LEFT"

    def arm(self):
        self.sequence = (("LEFT", 6), ("RIGHT", 9)) if self.last_line_side == "LEFT" else (
            ("RIGHT", 5), ("LEFT", 6))
        self.active = True
        self.step = 0
        self.command_id = None
        self.failed = False

    def cancel(self):
        self.active = False
        self.command_id = None

    def select(self, decision: MotionDecision, info: dict | None) -> MotionDecision:
        if not self.active:
            return decision
        if decision.phase != "GOAL_APPROACH":
            self.cancel()
            return decision
        if decision.reason == "invalid_vision_boolean_type":
            return decision
        if self.failed:
            return self._wait(decision, "goal_initial_search_motion_failed")
        if self.command_id is not None:
            return self._wait(decision, "goal_initial_search_running")
        # A candidate also stops scanning while Vision confirms the detection.
        if info is not None and (info.get("detected") is True or info.get("raw_detected") is True):
            if (info.get("detected") is True
                    and info.get("confirmation_confirmed", True) is True):
                self.cancel()
                return decision
            return self._wait(decision, "goal_initial_search_waiting_for_confirmation")
        if info is None or info.get("detected") is not False:
            return self._wait(decision, "goal_initial_search_waiting_for_fresh_vision")
        if self.step >= len(self.sequence):
            return self._wait(decision, "goal_initial_search_exhausted")
        direction, count = self.sequence[self.step]
        action = f"GOAL_CAMERA90_TURN_{direction}_{count}"
        yaw = MotionDecisionPlanner._turn_angle_deg(count, direction)
        return MotionDecision(
            phase="GOAL_APPROACH", source="goal", action=action, valid=True,
            reason="goal_initial_search", sdk_motion_requested=False, requires_ack=False,
            source_command={
                "valid": True, "motion": action, "goal_initial_search": True,
                "goal_initial_search_step": self.step + 1,
                "last_line_side": self.last_line_side, "turn_direction": direction,
                "turn_count": count, "turn_angle_deg": yaw,
                "target_heading_change_deg": yaw if direction == "RIGHT" else -yaw,
                "catalog_motion_available": True,
            },
        )

    @staticmethod
    def _wait(decision, reason):
        return replace(decision, source="goal", action="WAIT", valid=False, reason=reason,
                       sdk_motion_requested=False, requires_ack=False, source_command={})

    def published(self, decision, command_id):
        if decision.valid and decision.source_command.get("goal_initial_search") is True:
            self.command_id = command_id

    def completed(self, command_id, status):
        if self.command_id is None or command_id != self.command_id:
            return
        self.command_id = None
        if status == "SUCCEEDED":
            self.step += 1
        else:
            self.failed = True
