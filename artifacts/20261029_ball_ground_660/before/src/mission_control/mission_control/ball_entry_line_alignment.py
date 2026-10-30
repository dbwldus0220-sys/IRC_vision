"""Ball-only Line approach until the confirmed pickup distance is reached."""

from dataclasses import dataclass
import math

from step.approach_distance import BALL_HURDLE_FINE_DISTANCE_M
from step.line_navigation_planner import line_heading

from .motion_decision_planner import MotionDecision, MotionDecisionPlanner

FORWARD_MAX_HEADING_DEG = 15.0
STATIONARY_MIN_HEADING_DEG = 45.0


@dataclass
class BallEntryLineAlignment:
    """Repeat completed Line steps without releasing Ball mission ownership."""

    enabled: bool = True
    state: str = 'IDLE'
    command_id: int | None = None
    queued_command_id: int | None = None

    @property
    def pending(self) -> bool:
        return self.state == 'PENDING'

    @property
    def active(self) -> bool:
        return self.state in {'PENDING', 'QUEUED', 'RUNNING'}

    def arm(self, queued_line_command_id: int | None = None) -> bool:
        if not self.enabled or self.state != 'IDLE':
            return False
        self.state = 'PENDING'
        if queued_line_command_id is not None:
            # An accepted SDK reservation cannot safely be replaced mid-stride.
            self.published(queued_line_command_id)
        return True

    def published(self, command_id: int) -> None:
        if self.pending:
            self.command_id = command_id
            self.state = 'QUEUED'
        elif self.state == 'RUNNING' and self.queued_command_id is None:
            self.queued_command_id = command_id

    def on_status(self, command_id: int | None, status: str) -> None:
        if command_id is not None and command_id == self.queued_command_id:
            if status in {'FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED', 'UNSUPPORTED'}:
                self.queued_command_id = None
                self.state = 'FAILED'
            return
        if self.command_id is None or command_id != self.command_id or not self.active:
            return
        if status == 'RUNNING':
            self.state = 'RUNNING'
        elif status == 'SUCCEEDED':
            self.command_id = self.queued_command_id
            self.queued_command_id = None
            self.state = 'QUEUED' if self.command_id is not None else 'PENDING'
        elif status in {'FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED', 'UNSUPPORTED'}:
            self.state = 'FAILED'
            self.queued_command_id = None

    def reset(self) -> None:
        self.state = 'IDLE'
        self.command_id = None
        self.queued_command_id = None

    def finish(self) -> None:
        """Finish only between motions; an executing step must complete first."""
        if self.pending:
            self.state = 'DONE'
            self.command_id = None

    def select(self, planner: MotionDecisionPlanner, line_info: dict | None,
               dt_sec: float, ball_info: dict | None = None) -> MotionDecision:
        if (self.pending and ball_info is not None
                and ball_info.get('detected') is False
                and ball_info.get('raw_detected') is not True
                and ball_info.get('ball_loss_confirmed') is True):
            # Keep Line approach armed for reacquisition. The existing Ball
            # search owns turns, fresh-frame waits and the two-direction budget.
            return planner.plan_lost_ball_approach_alignment(ball_info)
        # Keep this policy local: normal Line, Goal and Hurdle retain their planners.
        command = self._line_command(planner, line_info, ball_info)
        return MotionDecision(
            phase='BALL_APPROACH', source='line', action=command['motion'],
            valid=bool(command['valid']), reason=command['reason'],
            sdk_motion_requested=False, requires_ack=False,
            source_command={**command, 'ball_entry_line_alignment': True},
        )

    @staticmethod
    def _line_command(planner: MotionDecisionPlanner, line_info: dict | None,
                      ball_info: dict | None) -> dict:
        line = planner.line_planner
        config = line.config
        ball_config = planner.ball_planner.config
        number = planner._number
        distance = number(ball_info, 'distance_m')
        confidence = number(ball_info, 'confidence')
        depth_age = number(ball_info, 'depth_age_sec')
        if (not ball_info or ball_info.get('detected') is not True
                or ball_info.get('confirmation_confirmed') is False
                or ball_info.get('depth_valid') is not True
                or distance is None or distance <= BALL_HURDLE_FINE_DISTANCE_M
                or confidence is None or confidence < ball_config.min_confidence
                or depth_age is None or not 0 <= depth_age <= ball_config.max_pickup_depth_age_sec):
            return line.stop('ball_entry_waiting_for_valid_far_ball').to_dict()
        if not line_info or line_info.get('detected') is not True:
            return BallEntryLineAlignment._forward_without_line(
                planner, distance, 'missing_line')
        heading = line_heading(line_info, config.heading_source)
        offset = number(line_info, 'filtered_lateral_offset_norm')
        if offset is None:
            offset = number(line_info, 'lateral_offset_norm')
        qualities = [number(line_info, key) for key in (
            'heading_quality', 'geometry_quality', 'detection_quality',
        )]
        qualities = [value for value in qualities if value is not None]
        if (heading is None or offset is None or not qualities
                or min(qualities) < config.min_line_quality):
            return BallEntryLineAlignment._forward_without_line(
                planner, distance, 'invalid_line_geometry')

        quality = min(1.0, min(qualities))
        command = line.stop('ball_entry_line_forward').to_dict()
        # STRAIGHT_2 selects the four-repeat gait and its matching capture window.
        command.update(valid=True, motion='STRAIGHT_2', heading_error_deg=heading,
                       steering_error_deg=heading, heading_component_deg=heading,
                       lateral_offset_norm=offset, line_quality=quality,
                       ball_entry_distance_m=distance,
                       ball_entry_stop_distance_m=BALL_HURDLE_FINE_DISTANCE_M,
                       heading_source=config.heading_source)
        if abs(heading) <= FORWARD_MAX_HEADING_DEG:
            speed = line._calculate_linear_speed(heading, quality)
            command.update(linear_speed_mps=speed,
                           travel_distance_m=speed * config.command_duration_sec)
            return command

        direction = 'RIGHT' if heading > 0.0 else 'LEFT'
        if abs(heading) < STATIONARY_MIN_HEADING_DEG:
            # Same direction rule as LineNavigationPlanner._classify_recovery:
            # offset chooses line side; heading sign chooses the recovery turn.
            side = 'RIGHT' if offset > 0.0 else 'LEFT' if offset < 0.0 else direction
            motion = f'RECOVER_{side}_TURN_{direction}_4'
            recovery = line._recovery_command(motion, heading, offset, quality).to_dict()
            return {**command, **recovery, 'reason': 'ball_entry_line_recovery',
                    'steering_error_deg': heading}

        count = planner._turn_repeat_count(heading, direction)
        yaw = planner._turn_angle_deg(count, direction)
        command.update(motion=f'LINE_HEADING_TURN_{direction}_{count}',
                       reason='ball_entry_line_stationary_alignment',
                       turn_direction=direction, turn_count=count, turn_angle_deg=yaw,
                       target_heading_change_deg=math.copysign(yaw, heading),
                       alignment_reference='line_heading', catalog_motion_available=True)
        return command

    @staticmethod
    def _forward_without_line(planner: MotionDecisionPlanner, distance: float,
                              reason: str) -> dict:
        """Use one four-repeat gait after the far-ball checks have passed."""
        line = planner.line_planner
        command = line.stop('ball_entry_forward_without_line').to_dict()
        # Keep geometry unknown; this is a fixed gait, not a measured line heading.
        speed = line.config.max_linear_speed_mps
        command.update(valid=True, motion='STRAIGHT_2',
                       linear_speed_mps=speed,
                       travel_distance_m=speed * line.config.command_duration_sec,
                       ball_entry_line_fallback=True,
                       ball_entry_line_fallback_reason=reason,
                       ball_entry_distance_m=distance,
                       ball_entry_stop_distance_m=BALL_HURDLE_FINE_DISTANCE_M,
                       heading_source=line.config.heading_source)
        return command
