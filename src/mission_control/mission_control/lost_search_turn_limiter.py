"""Bound published stationary search turns during one target loss."""

from dataclasses import replace
import math
import re

from .motion_decision_planner import MotionDecision, MotionDecisionPlanner
from .sparse_line_recovery import usable_two_point_line


class LostSearchTurnLimiter:
    """Keep independent budgets until a target supports a published command."""

    CORNER_SEARCH_MAX_ANGLE_DEG = 45.0
    CORNER_SEARCH_MAX_TURNS = 3

    def __init__(
        self, max_turns: int = 3, max_angle_deg: float = 90.0, *,
        staged_sources: tuple[str, ...] = (), max_reverse_turns: int = 6,
    ):
        if max_turns < 0 or not math.isfinite(max_angle_deg) or max_angle_deg < 0:
            raise ValueError("lost search limits must be finite and nonnegative")
        if max_reverse_turns < 0:
            raise ValueError("reverse search limit must be nonnegative")
        self.staged_sources = frozenset(staged_sources)
        self.max_reverse_turns = min(max_reverse_turns, 6)
        self.first_directions: dict[str, str] = {}
        self.corner_directions: dict[str, str] = {}
        self.failed_sources: set[str] = set()
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
        ) or action in {"ALIGN_LEFT", "ALIGN_RIGHT"} or action.startswith((
            "BALL_PICKUP_FINE_SEARCH_LEFT", "BALL_PICKUP_FINE_SEARCH_RIGHT",
        ))
        explicit_search = (
            "LOST_TURN" in action or "FINE_SEARCH_" in action
            or decision.reason in {
                "post_ball_line_search", "post_shot_line_search",
                "turn_toward_last_seen_goal_side", "turn_toward_last_seen_ball_side",
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
        if decision.phase not in {"AUTO", "LINE_TRACK"} or source not in {"line", "none"}:
            self.corner_directions.pop("line", None)
        if source not in self.counts:
            return decision
        angle = self.search_angle(decision, observations.get(source))
        if angle is None:
            return decision
        corner_direction = self.corner_directions.get(source)
        if (source == "line" and decision.action.startswith("LINE_LOST_TURN_")
                and decision.source_command.get("corner_search_pending") is True):
            corner_direction = corner_direction or decision.source_command.get("remembered_corner_direction")
        if (source == "line" and decision.action.startswith("LINE_LOST_TURN_")
                and corner_direction in {"LEFT", "RIGHT"}):
            return self._filter_corner(decision, observations.get(source), corner_direction)
        if source in self.staged_sources:
            return self._filter_staged(decision, observations.get(source))
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
            if decision.source_command.get("lost_search_stage") == "CORNER":
                self.corner_directions.setdefault(source, decision.source_command["turn_direction"])
            if decision.source_command.get("lost_search_stage") in {"INITIAL", "EXTEND", "REVERSE"}:
                self.first_directions.setdefault(
                    source, decision.source_command["lost_search_initial_direction"],
                )
            self.counts[source] += 1
            self.angles[source] += angle
        elif (
            info is not None and info.get("detected") is True
            and decision.source_command.get("corner_pending") is not True
            and info.get("confirmation_confirmed", True) is True
            and decision.action not in {"WAIT", "STOP", "BALL_LOST_STOP", "GOAL_LOST_STOP"}
            and not decision.action.startswith("POST_SHOT_TURN_")
        ):
            # A raw detection or repeatedly planned candidate cannot rearm search.
            self.counts[source] = 0
            self.angles[source] = 0.0
            self.first_directions.pop(source, None)
            self.corner_directions.pop(source, None)
            self.failed_sources.discard(source)

    def complete_corner(self) -> None:
        """Rearm only after the actual corner motion has succeeded."""
        self.counts["line"] = 0
        self.angles["line"] = 0.0
        self.first_directions.pop("line", None)
        self.corner_directions.pop("line", None)
        self.failed_sources.discard("line")

    def _filter_corner(self, decision: MotionDecision, info: dict | None, direction: str) -> MotionDecision:
        """Reobserve after each small turn; never reverse or refund earlier search."""
        source = decision.source
        max_turns = min(self.max_turns, self.CORNER_SEARCH_MAX_TURNS)
        max_angle = min(self.max_angle_deg, self.CORNER_SEARCH_MAX_ANGLE_DEG)
        count = 1 if direction == "LEFT" else 2
        angle = MotionDecisionPlanner._turn_angle_deg(count, direction)
        command = {
            **decision.source_command,
            "lost_search_stage": "CORNER", "lost_search_reverse": False,
            "lost_search_turns_used": self.counts[source],
            "lost_search_angle_used_deg": self.angles[source],
            "lost_search_max_turns": max_turns, "lost_search_max_angle_deg": max_angle,
        }
        reason = None
        visible_reacquire = bool(
            decision.source_command.get("corner_ground_reacquire") is True
            and info is not None and info.get("detected") is True
            and (
                (info.get("corner_preview_confirmed") is True
                 and info.get("corner_preview_raw_detected") is True
                 and info.get("corner_preview_held") is not True
                 and info.get("corner_direction") == direction)
                or (decision.source_command.get("corner_search_pending") is True
                    and decision.source_command.get("remembered_corner_direction") == direction
                    and usable_two_point_line(info)
                    and not (info.get("corner_preview_raw_detected") is True
                             and info.get("corner_direction") in {"LEFT", "RIGHT"}
                             and info["corner_direction"] != direction))
            )
        )
        if source in self.failed_sources:
            reason = "lost_search_motion_failed"
        elif (self.counts[source] >= max_turns or self.angles[source] + angle > max_angle):
            reason = "lost_search_turn_limit_reached"
        elif info is None or (info.get("detected") is not False and not visible_reacquire):
            reason = "lost_search_waiting_for_fresh_vision"
        elif info.get("raw_detected") is True and not visible_reacquire:
            reason = "lost_search_waiting_for_confirmation"
        if reason is not None:
            return replace(decision, action="WAIT", valid=False, sdk_motion_requested=False,
                           requires_ack=False, reason=reason,
                           source_command={**command, "blocked_search_action": decision.action})
        action = f"LINE_LOST_TURN_{direction}_{count}"
        command.update(
            motion=action, reason="line_lost_turn_toward_remembered_corner",
            direction_source="remembered_corner",
            turn_direction=direction, turn_count=count, turn_angle_deg=angle,
            turn_repeat_deg=None,
        )
        if "target_heading_change_deg" in command:
            command["target_heading_change_deg"] = angle if direction == "LEFT" else -angle
        return replace(decision, action=action,
                       reason="line_lost_turn_toward_remembered_corner", source_command=command)

    @staticmethod
    def _search_family(action: str) -> tuple[str, str] | None:
        for prefix in (
            "LINE_LOST_TURN_", "POST_BALL_LINE_TURN_", "POST_SHOT_LINE_TURN_",
            "BALL_APPROACH_TURN_", "BALL_PICKUP_CAMERA_DOWN_TURN_",
            "BALL_PICKUP_FINE_SEARCH_", "GOAL_CAMERA90_TURN_",
        ):
            if action.startswith(prefix):
                direction = action[len(prefix):].split("_", 1)[0]
                if direction in {"LEFT", "RIGHT"}:
                    return prefix, direction
        return None

    def record_failure(self, source: str, action: str) -> None:
        """A failed turn cannot advance into a differently named search motion."""
        if ((source in self.staged_sources or source in self.corner_directions) and self.counts[source] > 0
                and (self._search_family(action) is not None or action == "PICKUP_NOW")):
            self.failed_sources.add(source)

    def _filter_staged(self, decision: MotionDecision, info: dict | None) -> MotionDecision:
        source = decision.source
        used = self.counts[source]
        family = self._search_family(decision.action)
        first = self.first_directions.get(source, family[1] if family else "")
        # The public count limit is the additional same-direction search budget.
        # Each phase has its own angle ceiling; reversing never refunds travel.
        angle_limit = min(self.max_angle_deg, 90.0)
        extra_turns = min(self.max_turns, 3, max(0, int((angle_limit - 45.0) // 15.0)))
        reverse_turns = min(self.max_reverse_turns, int(angle_limit // 15.0))
        reverse_start = 1 + extra_turns
        total_turns = reverse_start + reverse_turns
        stage = ("INITIAL" if used == 0 else "EXTEND" if used < reverse_start
                 else "REVERSE" if used < total_turns else "EXHAUSTED")
        command = {
            **decision.source_command,
            "lost_search_stage": stage,
            "lost_search_initial_direction": first,
            "lost_search_turns_used": used,
            "lost_search_angle_used_deg": self.angles[source],
            "lost_search_max_turns": total_turns,
            "lost_search_max_angle_deg": 45.0 + extra_turns * 15.0 + reverse_turns * 15.0,
            "lost_search_phase_max_angle_deg": angle_limit,
            "lost_search_extra_turns": extra_turns,
            "lost_search_reverse_turns": reverse_turns,
        }
        reason = None
        if source in self.failed_sources:
            reason = "lost_search_motion_failed"
        elif family is None:
            reason = "lost_search_turn_angle_unavailable"
        elif angle_limit < 45.0 or self.max_turns == 0 or stage == "EXHAUSTED":
            reason = "lost_search_turn_limit_reached"
        elif info is None and not (
            decision.phase == "POST_BALL_LINE_ALIGN"
            and decision.action.startswith("POST_BALL_LINE_TURN_")
        ):
            reason = "lost_search_waiting_for_fresh_vision"
        elif info is not None and (info.get("raw_detected") is True or info.get("detected") is True):
            reason = "lost_search_waiting_for_confirmation"
        if reason is not None:
            return replace(
                decision, action="WAIT", valid=False, sdk_motion_requested=False,
                requires_ack=False, reason=reason,
                source_command={**command, "blocked_search_action": decision.action},
            )
        direction = first if stage != "REVERSE" else ("RIGHT" if first == "LEFT" else "LEFT")
        count = (3 if direction == "LEFT" else 5) if stage == "INITIAL" else (
            1 if direction == "LEFT" else 2
        )
        angle = MotionDecisionPlanner._turn_angle_deg(count, direction)
        command.update(
            turn_direction=direction, turn_count=count, turn_angle_deg=angle,
            turn_repeat_deg=None,
            lost_search_stage_step=(1 if stage == "INITIAL" else used if stage == "EXTEND"
                                    else used - reverse_start + 1),
            lost_search_reverse=stage == "REVERSE",
        )
        # Preserve each planner's existing sign convention for heading metadata.
        if "target_heading_change_deg" in command:
            sign = 1.0 if direction == ("RIGHT" if source == "goal" else "LEFT") else -1.0
            command["target_heading_change_deg"] = sign * angle
        if "angular_speed_rad_s" in command and source == "goal":
            command["angular_speed_rad_s"] = math.copysign(
                abs(command["angular_speed_rad_s"]), 1.0 if direction == "RIGHT" else -1.0,
            )
        return replace(decision, action=f"{family[0]}{direction}_{count}", source_command=command)
