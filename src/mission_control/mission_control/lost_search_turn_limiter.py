"""Bound published stationary search turns during one target loss."""

from dataclasses import replace
import math
import re

from .motion_decision_planner import MotionDecision, MotionDecisionPlanner


class LostSearchTurnLimiter:
    """Keep independent budgets until a target supports a published command."""

    def __init__(self, max_turns: int = 3, max_angle_deg: float = 90.0):
        if max_turns < 0 or not math.isfinite(max_angle_deg) or max_angle_deg < 0:
            raise ValueError("lost search limits must be finite and nonnegative")
        self.max_turns = max_turns
        self.max_angle_deg = max_angle_deg
        self.counts = dict.fromkeys(("line", "ball", "hurdle", "goal"), 0)
        self.angles = dict.fromkeys(self.counts, 0.0)

    @staticmethod
    def search_angle(decision: MotionDecision, info: dict | None) -> float | None:
        """Use catalog yaw bins; exclude the mandatory post-shot exit turn."""
        action = decision.action
        if not decision.valid or action.startswith("POST_SHOT_TURN_"):
            return None
        stationary = (
            "TURN" in action and "RECOVER_LEFT_" not in action
            and "RECOVER_RIGHT_" not in action
            and not action.startswith("BALL_APPROACH_RECOVER_")
        ) or action in {
            "ALIGN_LEFT", "ALIGN_RIGHT", "BALL_PICKUP_FINE_SEARCH_LEFT",
            "BALL_PICKUP_FINE_SEARCH_RIGHT",
        }
        explicit_search = (
            "LOST_TURN" in action or "FINE_SEARCH_" in action
            or decision.reason in {
                "post_ball_line_search", "post_shot_line_search",
                "turn_toward_last_seen_goal_side",
            }
        )
        if not stationary or (
            not explicit_search and info is not None and info.get("detected") is True
        ):
            return None
        match = re.search(r"(?:TURN|SEARCH)_(LEFT|RIGHT)(?:_(\d+))?$", action)
        if match:
            direction, count = match.groups()
            if count is None and action.startswith(("LINE_LOST_TURN_", "BALL_PICKUP_FINE_SEARCH_")):
                count = 2 if direction == "LEFT" else 5
            if count is not None:
                table = (
                    MotionDecisionPlanner.LEFT_TURN_ANGLES_DEG if direction == "LEFT"
                    else MotionDecisionPlanner.RIGHT_TURN_ANGLES_DEG
                )
                return table.get(int(count), math.inf)
        # Missing calibration must not silently bypass the search budget.
        angle = decision.source_command.get("turn_angle_deg")
        if (
            isinstance(angle, (int, float)) and not isinstance(angle, bool)
            and math.isfinite(angle) and angle > 0
        ):
            return angle
        return math.inf

    def filter(self, decision: MotionDecision, observations: dict) -> MotionDecision:
        """Inspect candidates without spending budget on unpublished decisions."""
        source = decision.source
        if source not in self.counts:
            return decision
        angle = self.search_angle(decision, observations.get(source))
        if angle is None:
            return decision
        diagnostic = {
            **decision.source_command,
            "lost_search_turns_used": self.counts[source],
            "lost_search_angle_used_deg": self.angles[source],
            "lost_search_max_turns": self.max_turns,
            "lost_search_max_angle_deg": self.max_angle_deg,
        }
        if self.counts[source] >= self.max_turns or self.angles[source] + angle > self.max_angle_deg:
            return replace(
                decision, action="WAIT", valid=False, sdk_motion_requested=False,
                requires_ack=False,
                reason=("lost_search_turn_limit_reached" if math.isfinite(angle)
                        else "lost_search_turn_angle_unavailable"),
                source_command={**diagnostic, "blocked_search_action": decision.action},
            )
        return replace(decision, source_command=diagnostic)

    def record_published(self, decision: MotionDecision, observations: dict) -> None:
        """Charge each request once, including failures; rearm on usable tracking."""
        source = decision.source
        if source not in self.counts or not decision.valid:
            return
        info = observations.get(source)
        angle = self.search_angle(decision, info)
        if angle is not None:
            self.counts[source] += 1
            self.angles[source] += angle
        elif (
            info is not None and info.get("detected") is True
            and info.get("confirmation_confirmed", True) is True
            and decision.action not in {"WAIT", "STOP", "BALL_LOST_STOP", "GOAL_LOST_STOP"}
            and not decision.action.startswith("POST_SHOT_TURN_")
        ):
            # A raw detection or repeatedly planned candidate cannot rearm search.
            self.counts[source] = 0
            self.angles[source] = 0.0
