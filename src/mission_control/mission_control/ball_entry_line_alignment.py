"""Optional one-motion Line alignment at the start of each Ball mission."""

from dataclasses import dataclass

from .motion_decision_planner import MotionDecision, MotionDecisionPlanner


@dataclass
class BallEntryLineAlignment:
    """Count an accepted command, never candidate decisions or duplicate statuses."""

    enabled: bool = True
    state: str = 'IDLE'
    command_id: int | None = None

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

    def on_status(self, command_id: int | None, status: str) -> None:
        if self.command_id is None or command_id != self.command_id or not self.active:
            return
        if status == 'RUNNING':
            self.state = 'RUNNING'
        elif status == 'SUCCEEDED':
            self.state = 'DONE'
        elif status in {'FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED', 'UNSUPPORTED'}:
            self.state = 'FAILED'

    def reset(self) -> None:
        self.state = 'IDLE'
        self.command_id = None

    def select(self, planner: MotionDecisionPlanner, line_info: dict | None,
               dt_sec: float) -> MotionDecision:
        # Use the live Line planner, including its offset and heading policies.
        # Node-level corner/freshness/settle gates still run with source=line.
        command = planner._plan_source('line', line_info, dt_sec)
        return MotionDecision(
            phase='BALL_APPROACH', source='line', action=command['motion'],
            valid=bool(command['valid']), reason=command['reason'],
            sdk_motion_requested=False, requires_ack=False,
            source_command={**command, 'ball_entry_line_alignment': True},
        )
