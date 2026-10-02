"""Tests for the navigation-to-C++-executor motion bridge."""

import inspect
import json
from types import MethodType

import pytest

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from mission_control.safety_interlock import RECOVERABLE_MOTOR_ERROR_CODES
from mission_control.motion_decision_planner import MotionDecisionPlanner
from std_msgs.msg import String


class FakeLogger:
    """Provide logger methods required by callbacks."""

    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class CapturePublisher:
    """Capture published ROS messages."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeBridge:
    """Provide bridge state without constructing a ROS node."""

    ACTION_TO_MOTION_ID = MotionCommandBridgeNode.ACTION_TO_MOTION_ID
    TERMINAL_STATUSES = MotionCommandBridgeNode.TERMINAL_STATUSES
    DEFAULT_TIMEOUT_MS = MotionCommandBridgeNode.DEFAULT_TIMEOUT_MS
    FINE_FORWARD_MOTION_IDS = MotionCommandBridgeNode.FINE_FORWARD_MOTION_IDS
    DWELL_MARKER = MotionCommandBridgeNode.DWELL_MARKER
    SHOT_PREPARE_DWELL_MARKER = MotionCommandBridgeNode.SHOT_PREPARE_DWELL_MARKER
    SHOT_PREPARE_MOTION_ID = MotionCommandBridgeNode.SHOT_PREPARE_MOTION_ID
    SHOT_PREPARE_DWELL_SEC = MotionCommandBridgeNode.SHOT_PREPARE_DWELL_SEC
    GOAL_CRAB_ACTIONS = MotionCommandBridgeNode.GOAL_CRAB_ACTIONS
    GOAL_FINE_RIGHT_CRAB_PREPARE_MOTION_ID = (
        MotionCommandBridgeNode.GOAL_FINE_RIGHT_CRAB_PREPARE_MOTION_ID
    )
    PICKUP_FINE_RIGHT_CRAB_PREPARE_MOTION_ID = (
        MotionCommandBridgeNode.PICKUP_FINE_RIGHT_CRAB_PREPARE_MOTION_ID
    )
    GOAL_FORWARD_CRAB_PREPARE_MOTION_ID = (
        MotionCommandBridgeNode.GOAL_FORWARD_CRAB_PREPARE_MOTION_ID
    )
    PICKUP_INITIAL_ALIGN_DWELL_MARKER = (
        MotionCommandBridgeNode.PICKUP_INITIAL_ALIGN_DWELL_MARKER
    )
    PICKUP_INITIAL_ALIGN_MARKER = (
        MotionCommandBridgeNode.PICKUP_INITIAL_ALIGN_MARKER
    )
    FINE_ALIGN_MARKER = MotionCommandBridgeNode.FINE_ALIGN_MARKER
    POST_BACKWARD_ALIGN_MARKER = (
        MotionCommandBridgeNode.POST_BACKWARD_ALIGN_MARKER
    )
    PICKUP_POSITIONING_LOSS_LATCH_ACTION = (
        MotionCommandBridgeNode.PICKUP_POSITIONING_LOSS_LATCH_ACTION
    )
    PICKUP_FIXED_SEQUENCE_FIRST_MOTION = (
        MotionCommandBridgeNode.PICKUP_FIXED_SEQUENCE_FIRST_MOTION
    )
    PICKUP_GRASP_CHECK_MOTION_ID = (
        MotionCommandBridgeNode.PICKUP_GRASP_CHECK_MOTION_ID
    )
    PICKUP_DWELL_SEC = MotionCommandBridgeNode.PICKUP_DWELL_SEC
    HURDLE_PRE_GO_DWELL_MARKER = MotionCommandBridgeNode.HURDLE_PRE_GO_DWELL_MARKER
    HURDLE_PRE_GO_DWELL_SEC = MotionCommandBridgeNode.HURDLE_PRE_GO_DWELL_SEC
    PICKUP_FINE_PREPARE_MOTION_ID = MotionCommandBridgeNode.PICKUP_FINE_PREPARE_MOTION_ID
    PICKUP_INITIAL_ALIGN_ACTIONS = (
        MotionCommandBridgeNode.PICKUP_INITIAL_ALIGN_ACTIONS
    )
    PICKUP_CAMERA_DOWN_TURN_MOTION_IDS = (
        MotionCommandBridgeNode.PICKUP_CAMERA_DOWN_TURN_MOTION_IDS
    )
    PICKUP_FINE_ALIGN_ACTIONS = (
        MotionCommandBridgeNode.PICKUP_FINE_ALIGN_ACTIONS
    )
    PICKUP_FINE_ALIGN_MOTION_IDS = (
        MotionCommandBridgeNode.PICKUP_FINE_ALIGN_MOTION_IDS
    )
    PICKUP_POST_BACKWARD_ALIGN_ACTIONS = (
        MotionCommandBridgeNode.PICKUP_POST_BACKWARD_ALIGN_ACTIONS
    )
    ATOMIC_SEQUENCE_ACTIONS = MotionCommandBridgeNode.ATOMIC_SEQUENCE_ACTIONS
    PICKUP_CAMERA_DOWN_MOTION_IDS = (
        MotionCommandBridgeNode.PICKUP_CAMERA_DOWN_MOTION_IDS
    )
    PICKUP_MOTION_TAIL = MotionCommandBridgeNode.PICKUP_MOTION_TAIL
    PICKUP_NO_BALL_MOTION_TAIL = (
        MotionCommandBridgeNode.PICKUP_NO_BALL_MOTION_TAIL
    )
    POST_BALL_GOAL_TRANSITION_SEQUENCE = (
        MotionCommandBridgeNode.POST_BALL_GOAL_TRANSITION_SEQUENCE
    )

    def __init__(self):
        self.last_sent_command_id = None
        self.motion_in_progress = False
        self.active_command_id = None
        self.active_event_id = None
        self.active_action = None
        self.active_request_id = None
        self.active_motion_id = None
        self.active_timeout_ms = None
        self.active_pickup_sequence = ()
        self.active_sequence_index = 0
        self.active_dwell_until = None
        self.goal_fine_forward_completed = False
        self.hurdle_depth_fine_completed = False
        self.hurdle_sequence_fine_completed = 0
        self.hurdle_fine_sequence_pending = False
        self.goal_crab_completed = False
        self.last_completed_motion_id = None
        self.last_physical_motion_id = None
        self.pending_turn_request = None
        self.turn_prepare_motion_id = None
        self.head_override_state = {}
        self.head_override_received_at = None
        self.pickup_initial_align_dwell_until = None
        self.pickup_initial_align_waiting = False
        self.pickup_initial_align_correction_active = False
        self.pickup_fine_align_waiting = False
        self.pickup_fine_align_correction_active = False
        self.pickup_post_backward_align_waiting = False
        self.pickup_post_backward_align_correction_active = False
        self.pickup_checkpoint_after_dwell = None
        self.pickup_positioning_loss_pending = False
        self.pickup_fixed_sequence_started = False
        self.pickup_fine_positioning_complete = False
        self.last_pickup_motion_id = None
        self.pickup_consecutive_motor_failures = 0
        self.pickup_motor_failures = []
        self.pickup_last_stage_succeeded = True
        self.pending_pickup_crab_motion_id = None
        self.pickup_positioning_dwell_motion_id = None
        self.queued_command_id = None
        self.queued_event_id = None
        self.queued_action = None
        self.queued_request_id = None
        self.queued_motion_id = None
        self.queued_timeout_ms = None
        self.queued_request_deferred = False
        self.queued_pickup_sequence = ()
        self.executor_request_publisher = CapturePublisher()
        self.motion_status_publisher = CapturePublisher()
        self.logger = FakeLogger()

        for name in (
            "publish_motion_status",
            "_publish_local_rejection",
            "_record_goal_motion_success",
            "_start_pickup_initial_align_dwell",
            "_enter_pickup_initial_align_checkpoint",
            "_handle_pickup_initial_align_command",
            "_enter_pickup_fine_align_checkpoint",
            "_handle_pickup_fine_align_command",
            "_enter_pickup_positioning_loss_reacquisition",
            "_handle_pickup_positioning_loss_latch",
            "_enter_pickup_post_backward_align_checkpoint",
            "_handle_pickup_post_backward_align_command",
            "_start_pickup_checkpoint_dwell",
            "executor_heartbeat_callback",
            "_is_stationary_turn",
            "_turn_prepare_motion",
            "_publish_executor_request",
            "navigation_command_callback",
            "_start_next_pickup_motion",
            "_check_atomic_dwell",
            "_valid_executor_status",
            "_clear_active_request",
            "_clear_queued_request",
            "_promote_queued_request",
            "executor_status_callback",
        ):
            setattr(
                self,
                name,
                MethodType(getattr(MotionCommandBridgeNode, name), self),
            )

    @staticmethod
    def _is_integer(value):
        return MotionCommandBridgeNode._is_integer(value)

    def motion_id_for_action(self, action):
        return self.ACTION_TO_MOTION_ID.get(action)

    def timeout_ms_from_payload(self, payload):
        return MotionCommandBridgeNode.timeout_ms_from_payload(payload)

    def _with_pickup_motion_dwells(self, sequence):
        return MotionCommandBridgeNode._with_pickup_motion_dwells(sequence)

    def _pickup_motion_sequence(self, payload):
        return MotionCommandBridgeNode._pickup_motion_sequence(payload)

    def get_logger(self):
        return self.logger


def string_message(payload):
    """Create a String message containing JSON or raw text."""
    message = String()
    message.data = payload if isinstance(payload, str) else json.dumps(payload)
    return message


def navigation_message(**overrides):
    """Create one valid navigation command."""
    payload = {
        "command_id": 8000,
        "event_id": 8,
        "action": "STRAIGHT",
        "valid": True,
        "source_command": {"pickup_approach_motion": "STRAIGHT_2"},
        "mission_progress": {
            "pickups_completed": 0,
            "required_pickups": 2,
        },
    }
    payload.update(overrides)
    return string_message(payload)


def executor_status(**overrides):
    """Create one valid executor status."""
    payload = {
        "status": "RUNNING",
        "command_id": 8000,
        "event_id": 8,
        "request_id": 8000,
        "motion_id": "forward",
        "error_code": "",
        "message": "",
    }
    payload.update(overrides)
    return string_message(payload)


def decoded_messages(publisher):
    """Decode all captured String messages."""
    return [json.loads(message.data) for message in publisher.messages]


def complete_active_motion(bridge, status="SUCCEEDED", error_code=""):
    """Complete preparation, if present, then the original executor motion."""
    if bridge.pending_turn_request is not None:
        bridge.executor_status_callback(executor_status(
            status=status, error_code=error_code,
            command_id=bridge.active_command_id, event_id=bridge.active_event_id,
            request_id=bridge.active_request_id, motion_id=bridge.turn_prepare_motion_id,
        ))
        if status != "SUCCEEDED":
            return
    bridge.executor_status_callback(executor_status(
        status=status,
        error_code=error_code,
        command_id=bridge.active_command_id,
        event_id=bridge.active_event_id,
        request_id=bridge.active_request_id,
        motion_id=bridge.active_motion_id,
    ))


@pytest.mark.parametrize("count", [1, 2, 3, 4])
def test_fine_only_goal_approach_prepares_pose_then_waits_two_seconds(count, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr("mission_control.motion_command_bridge_node.time.monotonic", lambda: clock[0])
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        action=f"GOAL_CAMERA90_FINE_FORWARD_{count}", command_id=1,
    ))
    complete_active_motion(bridge)
    bridge.navigation_command_callback(navigation_message(action="SHOT", command_id=2))
    assert bridge.active_motion_id == "goal_fine_to_default"
    assert bridge.active_dwell_until is None

    clock[0] = 104.0
    complete_active_motion(bridge)
    assert bridge.active_dwell_until == 106.0
    assert bridge.motion_in_progress
    assert bridge.active_action == "SHOT"
    assert not any(m["action"] == "SHOT" and m["status"] == "SUCCEEDED"
                   for m in decoded_messages(bridge.motion_status_publisher))

    # Repeated completion messages must neither skip nor restart the dwell.
    bridge.executor_status_callback(executor_status(
        status="SUCCEEDED", command_id=2, request_id=2,
        motion_id="goal_fine_to_default",
    ))
    assert bridge.active_dwell_until == 106.0
    bridge.navigation_command_callback(navigation_message(action="STRAIGHT", command_id=3))
    assert decoded_messages(bridge.motion_status_publisher)[-1]["error_code"] == "ATOMIC_SEQUENCE_LOCKED"
    bridge._check_atomic_dwell(105.999)
    assert len(bridge.executor_request_publisher.messages) == 2
    bridge._check_atomic_dwell(106.0)
    assert [m["motion_id"] for m in decoded_messages(bridge.executor_request_publisher)] == [
        f"goal_camera_90_fine_forward_{count}", "goal_fine_to_default", "goal_shot",
    ]
    assert bridge.motion_in_progress
    complete_active_motion(bridge)
    terminal = decoded_messages(bridge.motion_status_publisher)[-1]
    assert (terminal["action"], terminal["status"]) == ("SHOT", "SUCCEEDED")
    assert not bridge.motion_in_progress
    assert not bridge.goal_fine_forward_completed
    assert not bridge.goal_crab_completed


@pytest.mark.parametrize("actions", [
    [], ["BALL_FINE_FORWARD_8"], ["GOAL_CAMERA_90_FORWARD"],
    ["GOAL_CAMERA90_CRAB_LEFT"],
    ["GOAL_CAMERA90_FINE_FORWARD_1", "GOAL_CAMERA90_CRAB_RIGHT"],
    ["GOAL_CAMERA90_CRAB_LEFT", "GOAL_CAMERA90_FINE_FORWARD_2"],
])
def test_shot_without_fine_only_approach_keeps_direct_scoring(actions):
    bridge = FakeBridge()
    for command_id, action in enumerate(actions, 1):
        bridge.navigation_command_callback(navigation_message(action=action, command_id=command_id))
        complete_active_motion(bridge)
        if bridge.motion_in_progress:
            complete_active_motion(bridge)
    bridge.navigation_command_callback(navigation_message(action="SHOT", command_id=10))
    assert bridge.active_motion_id == "goal_shot"
    assert bridge.active_pickup_sequence == ()


@pytest.mark.parametrize("status,error_code", [
    ("FAILED", "BACKEND_FAILED"), ("TIMEOUT", "TIMEOUT"),
    ("CANCELLED", ""), ("REJECTED", "MOTION_NOT_FOUND"),
    *[("FAILED", code) for code in sorted(RECOVERABLE_MOTOR_ERROR_CODES)],
])
def test_shot_pose_preparation_failure_never_advances_to_scoring(status, error_code):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="GOAL_CAMERA90_FINE_FORWARD_1", command_id=1))
    complete_active_motion(bridge)
    bridge.navigation_command_callback(navigation_message(action="SHOT", command_id=2))
    complete_active_motion(bridge, status, error_code)
    bridge._check_atomic_dwell(float("inf"))
    assert not bridge.motion_in_progress
    assert bridge.active_dwell_until is None
    assert "goal_shot" not in [m["motion_id"] for m in decoded_messages(bridge.executor_request_publisher)]
    terminal = decoded_messages(bridge.motion_status_publisher)[-1]
    assert (terminal["action"], terminal["status"]) == ("SHOT", status)
    assert terminal["error_code"] == error_code


def test_shot_waits_for_running_fine_motion_and_uses_only_completed_history():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="GOAL_CAMERA90_FINE_FORWARD_1", command_id=1))
    assert not bridge.goal_fine_forward_completed
    bridge.navigation_command_callback(navigation_message(action="SHOT", command_id=2))
    assert decoded_messages(bridge.motion_status_publisher)[-1]["error_code"] == "REJECTED_BUSY"
    assert len(bridge.executor_request_publisher.messages) == 1
    complete_active_motion(bridge, "FAILED", "SDK_POSITION_TIMEOUT")
    assert not bridge.goal_fine_forward_completed
    bridge.navigation_command_callback(navigation_message(action="SHOT", command_id=3))
    assert bridge.active_motion_id == "goal_shot"


def test_shot_retry_does_not_repeat_completed_pose_preparation():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="GOAL_CAMERA90_FINE_FORWARD_1", command_id=1))
    complete_active_motion(bridge)
    bridge.navigation_command_callback(navigation_message(action="SHOT", command_id=2))
    complete_active_motion(bridge)
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    complete_active_motion(bridge, "FAILED", "SDK_POSITION_TIMEOUT")
    bridge.navigation_command_callback(navigation_message(action="SHOT", command_id=3))
    assert bridge.active_motion_id == "goal_shot"
    assert [m["motion_id"] for m in decoded_messages(bridge.executor_request_publisher)].count("goal_fine_to_default") == 1


def test_new_pickup_clears_previous_goal_approach_history():
    bridge = FakeBridge()
    for command_id, action in enumerate(("GOAL_CAMERA90_CRAB_LEFT", "GOAL_CAMERA90_FINE_FORWARD_1"), 1):
        bridge.navigation_command_callback(navigation_message(action=action, command_id=command_id))
        complete_active_motion(bridge)
    assert bridge.goal_fine_forward_completed and bridge.goal_crab_completed
    bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW", command_id=3))
    assert not bridge.goal_fine_forward_completed
    assert not bridge.goal_crab_completed


def continue_pickup_after_fine_alignment(bridge, command_id=8001):
    """Resume the active pickup from a centered fresh-Ball decision."""
    bridge.navigation_command_callback(
        navigation_message(
            action="BALL_PICKUP_FINE_ALIGN_CONTINUE",
            command_id=command_id,
            event_id=None,
            active_special_command_id=bridge.active_command_id,
            active_special_event_id=bridge.active_event_id,
        )
    )


def assert_pickup_initial_alignment_started(bridge):
    """Confirm initial alignment starts without a timed pause."""
    assert bridge.pickup_initial_align_dwell_until is None
    assert bridge.pickup_initial_align_waiting is True


def continue_pickup_after_initial_alignment(
    bridge,
    approach_motion="STRAIGHT_2",
    command_id=8001,
):
    """Select the distance approach from a fresh post-alignment Ball frame."""
    bridge.navigation_command_callback(
        navigation_message(
            action="BALL_PICKUP_INITIAL_ALIGN_CONTINUE",
            command_id=command_id,
            event_id=None,
            source_command={"pickup_approach_motion": approach_motion},
            active_special_command_id=bridge.active_command_id,
            active_special_event_id=bridge.active_event_id,
        )
    )


def complete_pickup_fine_preparation(bridge):
    """Finish preparation without advancing the fine-motion checkpoint."""
    assert bridge.active_motion_id == bridge.PICKUP_FINE_PREPARE_MOTION_ID
    bridge.executor_status_callback(executor_status(
        status="SUCCEEDED", motion_id=bridge.PICKUP_FINE_PREPARE_MOTION_ID,
    ))
    assert bridge.active_motion_id == MotionCommandBridgeNode.PICKUP_FINE_PRE_DWELL_MARKER
    deadline = bridge.active_dwell_until
    before = len(bridge.executor_request_publisher.messages)
    bridge._check_atomic_dwell(deadline - .001)
    assert len(bridge.executor_request_publisher.messages) == before
    bridge._check_atomic_dwell(deadline)
    assert bridge.active_motion_id == "pickup_fine_forward_0"
    assert bridge.active_dwell_until is None


def complete_pickup_motion_and_dwell(bridge, motion_id):
    """Complete a pickup motion and its configured non-blocking dwell."""
    if motion_id == "pickup_fine_forward_0" and (
        bridge.active_motion_id == bridge.PICKUP_FINE_PREPARE_MOTION_ID
    ):
        complete_pickup_fine_preparation(bridge)
    bridge.executor_status_callback(
        executor_status(status="SUCCEEDED", motion_id=motion_id)
    )
    if bridge.active_dwell_until is not None:
        bridge._check_atomic_dwell(bridge.active_dwell_until)
    if bridge.pickup_initial_align_waiting:
        continue_pickup_after_initial_alignment(
            bridge,
            approach_motion="STRAIGHT_0",
            command_id=bridge.last_sent_command_id + 1,
        )


def assert_correction_dwell_blocks_then_finishes(bridge):
    """Check that the executor remains reserved until the correction pause ends."""
    deadline = bridge.active_dwell_until
    assert deadline is not None
    count = len(bridge.executor_request_publisher.messages)
    bridge._check_atomic_dwell(deadline - 0.001)
    assert bridge.active_dwell_until == deadline
    assert len(bridge.executor_request_publisher.messages) == count
    assert bridge.motion_in_progress
    bridge._check_atomic_dwell(deadline)


def pickup_positioning_loss_message(bridge, motion_id, command_id=8002):
    """Build one non-motion pickup positioning loss latch command."""
    return navigation_message(
        action=bridge.PICKUP_POSITIONING_LOSS_LATCH_ACTION,
        command_id=command_id,
        event_id=None,
        active_special_command_id=bridge.active_command_id,
        active_special_event_id=bridge.active_event_id,
        source_command={
            "pickup_positioning_ball_lost": True,
            "pickup_positioning_motion_id": motion_id,
        },
    )


EXPECTED_PRODUCTION_ACTIONS = {
    "BALL_APPROACH_RECOVER_LEFT_4": "line_recovery_left_4",
    "BALL_APPROACH_RECOVER_RIGHT_4": "line_recovery_right_4",
    "STRAIGHT": "line_forward_6",
    "STRAIGHT_1": "line_forward_2",
    "STRAIGHT_2": "line_forward_4",
    "STRAIGHT_3": "line_forward_6",
    "STRAIGHT_4": "line_forward_8",
    "APPROACH": "forward",
    "LEFT": "line_recovery_left_4",
    "RIGHT": "line_recovery_right_4",
    "POST_BALL_GOAL_TRANSITION": "post_ball_camera_90",
    "POST_SHOT_TURN_RIGHT_9": "post_shot_default_turn_right",
    "POST_SHOT_TURN_LEFT_4": "post_shot_default_turn_left",
    "POST_SHOT_FORWARD": "line_forward_6",
    **{
        f"POST_SHOT_LINE_TURN_{direction}_{count}": (
            f"post_ball_line_turn_{direction.lower()}_{count}"
        )
        for direction, counts in (
            ("RIGHT", (2, 3, 5, 7, 9)), ("LEFT", (1, 2, 3, 4, 5, 6)),
        )
        for count in counts
    },
    "BALL_FINE_FORWARD_8": "ball_general_fine_forward_8",
    "BALL_LOST_FORWARD_2": "ball_camera_down_forward_2",
    "BALL_LOST_FORWARD_4": "ball_camera_down_forward_4",
    "LINE_LOST_TURN_LEFT": "line_search_left_2",
    "LINE_LOST_TURN_RIGHT": "line_search_right_5",
    "LINE_LOST_TURN_LEFT_1": "post_ball_line_turn_left_1",
    "LINE_LOST_TURN_LEFT_3": "post_ball_line_turn_left_3",
    "LINE_LOST_TURN_RIGHT_2": "post_ball_line_turn_right_2",
    "LINE_LOST_TURN_RIGHT_5": "post_ball_line_turn_right_5",
    "GOAL_CAMERA_90_FORWARD": "goal_camera_90_forward_6",
    "GOAL_CAMERA_90_FORWARD_1": "goal_camera_90_forward_2",
    "GOAL_CAMERA_90_FORWARD_2": "goal_camera_90_forward_4",
    "GOAL_CAMERA90_BACKWARD_1": "goal_camera_90_backward_1",
    **{
        f"GOAL_CAMERA90_FINE_FORWARD_{count}": f"goal_camera_90_fine_forward_{count}"
        for count in range(1, 5)
    },
    "GOAL_CAMERA90_CRAB_RIGHT": "goal_camera_90_crab_right",
    "GOAL_CAMERA90_CRAB_LEFT": "goal_camera_90_crab_left",
    "SHOT": "goal_shot",
    "GO": "hurdle",
    "TURN_LEFT": "stationary_turn_left",
    "TURN_RIGHT": "stationary_turn_right",
    "ALIGN_LEFT": "stationary_turn_left",
    "ALIGN_RIGHT": "stationary_turn_right",
}
for count in (2, 3, 5, 7, 9):
    EXPECTED_PRODUCTION_ACTIONS[
        f"POST_BALL_LINE_TURN_RIGHT_{count}"
    ] = f"post_ball_line_turn_right_{count}"
    EXPECTED_PRODUCTION_ACTIONS[
        f"LINE_OFFSET_TURN_RIGHT_{count}"
    ] = f"post_ball_line_turn_right_{count}"
    EXPECTED_PRODUCTION_ACTIONS[
        f"GOAL_CAMERA90_TURN_RIGHT_{count}"
    ] = f"goal_camera_90_turn_right_{count}"
for count in (2, 3, 5, 7, 9):
    EXPECTED_PRODUCTION_ACTIONS[
        f"BALL_APPROACH_TURN_RIGHT_{count}"
    ] = f"post_ball_line_turn_right_{count}"
for count in range(1, 7):
    EXPECTED_PRODUCTION_ACTIONS[
        f"GOAL_CAMERA90_TURN_LEFT_{count}"
    ] = f"goal_camera_90_turn_left_{count}"
for count in (1, 2, 3, 4, 5, 6):
    EXPECTED_PRODUCTION_ACTIONS[
        f"POST_BALL_LINE_TURN_LEFT_{count}"
    ] = f"post_ball_line_turn_left_{count}"
    EXPECTED_PRODUCTION_ACTIONS[
        f"LINE_OFFSET_TURN_LEFT_{count}"
    ] = f"post_ball_line_turn_left_{count}"
for count in (1, 2, 3, 4, 5, 6):
    EXPECTED_PRODUCTION_ACTIONS[
        f"BALL_APPROACH_TURN_LEFT_{count}"
    ] = f"post_ball_line_turn_left_{count}"
for recovery_side in ("LEFT", "RIGHT"):
    for direction, counts in (
        ("LEFT", (4,)),
        ("RIGHT", (4,)),
    ):
        for count in counts:
            EXPECTED_PRODUCTION_ACTIONS[
                f"RECOVER_{recovery_side}_TURN_{direction}_{count}"
            ] = f"line_recovery_{direction.lower()}_{count}"


def test_production_action_mapping_contains_only_approved_contract():
    assert MotionCommandBridgeNode.ACTION_TO_MOTION_ID == (
        EXPECTED_PRODUCTION_ACTIONS
    )


@pytest.mark.parametrize(
    ("action", "motion_id"),
    sorted(EXPECTED_PRODUCTION_ACTIONS.items()),
)
def test_supported_action_builds_executor_request(action, motion_id):
    bridge = FakeBridge()

    bridge.navigation_command_callback(navigation_message(action=action))
    if action == "GO":
        assert bridge.active_motion_id == "pickup_fine_forward_0"
        for _ in range(2):
            complete_active_motion(bridge)
            bridge._check_atomic_dwell(bridge.active_dwell_until)
        assert bridge.active_motion_id == "hurdle"
        return

    requests = decoded_messages(bridge.executor_request_publisher)
    assert requests == [
        {
            "action": action,
            "command_id": 8000,
            "event_id": 8,
            "request_id": 8000,
            "motion_id": motion_id,
            "timeout_ms": 12000,
        }
    ]
    assert bridge.motion_in_progress is True
    assert bridge.active_command_id == 8000
    assert bridge.active_event_id == 8
    assert bridge.active_action == action
    assert bridge.active_request_id == 8000
    assert bridge.active_motion_id == motion_id


@pytest.mark.parametrize("timeout", [0, -1, "100", True, None])
def test_invalid_timeout_uses_default(timeout):
    bridge = FakeBridge()

    bridge.navigation_command_callback(
        navigation_message(timeout_ms=timeout)
    )

    request = decoded_messages(bridge.executor_request_publisher)[0]
    assert request["timeout_ms"] == 12000


def test_positive_source_timeout_is_preserved():
    bridge = FakeBridge()

    bridge.navigation_command_callback(
        navigation_message(source_command={"timeout_ms": 2500})
    )

    request = decoded_messages(bridge.executor_request_publisher)[0]
    assert request["timeout_ms"] == 2500


@pytest.mark.parametrize(
    "action",
    [
        "RETREAT_GOAL",
        "STRAIGHT_5",
        "SLOW_APPROACH",
        "FINE_FORWARD_STEP",
        "APPROACH_GOAL",
        "APPROACH_HURDLE",
        "CROSS_FINISH",
        "STOP",
        "WAIT",
        "BALL_LOST_STOP",
        "GOAL_LOST_STOP",
        "HEAD_SCAN_LEFT",
        "HEAD_SCAN_RIGHT",
        "HEAD_CENTER",
        "RECOVER_GOAL_TURN_LEFT",
        "RECOVER_GOAL_TURN_RIGHT",
        "RECOVER_LEFT_TURN_LEFT_6",
        "RECOVER_RIGHT_TURN_LEFT_2",
        "RECOVER_LEFT_TURN_RIGHT_6",
        "RECOVER_RIGHT_TURN_RIGHT_8",
        "WAIT_SCORE_CONFIRMATION",
        "WAIT_GO_CONFIRMATION",
        "UNKNOWN",
        *[
            f"{prefix}_RIGHT_{count}"
            for prefix in (
                "POST_BALL_LINE_TURN",
                "GOAL_CAMERA90_TURN",
            )
            for count in (1, 4, 6, 8)
        ],
    ],
)
def test_unsupported_action_does_not_publish_executor_request(action):
    bridge = FakeBridge()

    bridge.navigation_command_callback(
        navigation_message(action=action, sdk_motion_requested=True)
    )

    assert bridge.executor_request_publisher.messages == []
    status = decoded_messages(bridge.motion_status_publisher)[0]
    assert status["status"] == "UNSUPPORTED"
    assert status["error_code"] == "UNSUPPORTED_ACTION"
    assert status["action"] == action


def test_duplicate_command_id_is_rejected():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())
    bridge.motion_in_progress = False

    bridge.navigation_command_callback(navigation_message())

    assert len(bridge.executor_request_publisher.messages) == 1
    status = decoded_messages(bridge.motion_status_publisher)[0]
    assert status["status"] == "REJECTED"
    assert status["error_code"] == "DUPLICATE_COMMAND_ID"


def test_new_command_while_running_is_queued():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())

    bridge.navigation_command_callback(
        navigation_message(command_id=8001, event_id=9)
    )

    assert len(bridge.executor_request_publisher.messages) == 2
    assert bridge.queued_request_id == 8001
    assert bridge.queued_action == "STRAIGHT"


def test_pickup_waits_locally_for_incompatible_active_motion_to_finish():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())
    bridge.navigation_command_callback(
        navigation_message(
            command_id=8001,
            event_id=9,
            action="PICKUP_NOW",
        )
    )

    requests = decoded_messages(bridge.executor_request_publisher)
    assert [request["motion_id"] for request in requests] == [
        "line_forward_6"
    ]
    assert bridge.queued_request_deferred is True

    bridge.executor_status_callback(
        executor_status(status="SUCCEEDED", motion_id="line_forward_6")
    )

    requests = decoded_messages(bridge.executor_request_publisher)
    assert [request["motion_id"] for request in requests] == [
        "line_forward_6",
    ]
    assert bridge.active_request_id == 8001
    assert bridge.active_action == "PICKUP_NOW"
    assert bridge.queued_request_deferred is False
    assert bridge.active_motion_id == bridge.PICKUP_INITIAL_ALIGN_MARKER
    assert bridge.pickup_initial_align_dwell_until is None
    assert bridge.pickup_initial_align_waiting is True


def test_first_pickup_finishes_after_backward_turn_dwell_without_pose_transition():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)

    for completed_motion in (
        "ball_camera_down_forward_4",
        "ball_general_fine_forward_8",
    ):
        complete_pickup_motion_and_dwell(bridge, completed_motion)
    continue_pickup_after_fine_alignment(bridge)
    for completed_motion in (
        "pickup_pre_backward_camera_down",
        "pickup",
        "pickup_grasp_check_pose",
        "pickup_retreat_2",
    ):
        complete_pickup_motion_and_dwell(bridge, completed_motion)
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="pickup_first_backward_turn_right",
        )
    )

    assert [
        request["motion_id"]
        for request in decoded_messages(bridge.executor_request_publisher)
    ] == [
        "ball_camera_down_forward_4",
        "ball_general_fine_forward_8",
        "pickup_pre_backward_camera_down",
        "pickup",
        "pickup_grasp_check_pose",
        "pickup_retreat_2",
        "pickup_first_backward_turn_right",
    ]
    request_motion_ids = [
        request["motion_id"]
        for request in decoded_messages(bridge.executor_request_publisher)
    ]
    assert request_motion_ids.count("pickup_grasp_check_pose") == 1
    verification_statuses = [
        status
        for status in decoded_messages(bridge.motion_status_publisher)
        if status.get("verification_window_complete") is not None
    ]
    assert [
        status["verification_window_complete"]
        for status in verification_statuses
    ] == [False, True]
    assert_correction_dwell_blocks_then_finishes(bridge)
    assert bridge.active_dwell_until is None
    assert [
        request["motion_id"]
        for request in decoded_messages(bridge.executor_request_publisher)
    ] == request_motion_ids

    statuses = decoded_messages(bridge.motion_status_publisher)
    assert statuses[-1]["status"] == "SUCCEEDED"
    assert statuses[-1]["action"] == "PICKUP_NOW"
    assert bridge.motion_in_progress is False


def test_grasp_verification_uses_three_second_pause_after_pose(monkeypatch):
    from test_motion_decision_node import (
        FakeDecisionNode, arm_special_command, send_grasp_detections,
    )
    from mission_control.motion_decision_node import MotionDecisionNode

    monkeypatch.setattr("mission_control.motion_command_bridge_node.time.monotonic", lambda: 10.0)
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 8000, 8)
    continue_pickup_after_fine_alignment(bridge)
    complete_pickup_motion_and_dwell(bridge, "pickup_pre_backward_camera_down")
    complete_pickup_motion_and_dwell(bridge, "pickup")
    assert bridge.active_motion_id == "pickup_grasp_check_pose"
    offset = len(bridge.motion_status_publisher.messages)

    def deliver_statuses():
        nonlocal offset
        for message in bridge.motion_status_publisher.messages[offset:]:
            MotionDecisionNode._motion_status_callback(node, message)
        offset = len(bridge.motion_status_publisher.messages)

    send_grasp_detections(node, 1, [{"class_name": "grab", "confidence": 0.9}])
    bridge.executor_status_callback(executor_status(
        status="RUNNING", motion_id="pickup_grasp_check_pose",
    ))
    deliver_statuses()
    assert not getattr(node, "grasp_verification_active", False)
    send_grasp_detections(node, 2, [{"class_name": "grab", "confidence": 0.9}])
    bridge.executor_status_callback(executor_status(
        status="SUCCEEDED", motion_id="pickup_grasp_check_pose",
    ))
    deliver_statuses()
    assert node.grasp_verification_active
    assert bridge.active_dwell_until == 13.0
    request_count = len(bridge.executor_request_publisher.messages)
    bridge._check_atomic_dwell(12.999)
    assert len(bridge.executor_request_publisher.messages) == request_count
    for stamp in range(3, 18):
        send_grasp_detections(node, stamp, [{"class_name": "grab", "confidence": 0.9}])
    bridge._check_atomic_dwell(13.0)
    assert bridge.active_motion_id == "pickup_retreat_2"
    assert bridge.active_dwell_until is None
    deliver_statuses()
    assert not node.grasp_verification_active
    assert node.phase_manager.grasp_result_for_ball(1) == "GRABBED"
    send_grasp_detections(node, 18, [])
    assert node.grasp_verify_accepted_frames == 15
    assert node.phase_manager.grasp_result_for_ball(1) == "GRABBED"


def test_completed_pickup_does_not_restart_on_later_timer_ticks():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)

    for completed_motion in (
        "ball_camera_down_forward_4",
        "ball_general_fine_forward_8",
    ):
        complete_pickup_motion_and_dwell(bridge, completed_motion)
    continue_pickup_after_fine_alignment(bridge)
    for completed_motion in (
        "pickup_pre_backward_camera_down",
        "pickup",
        "pickup_grasp_check_pose",
        "pickup_retreat_2",
    ):
        complete_pickup_motion_and_dwell(bridge, completed_motion)
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="pickup_first_backward_turn_right",
        )
    )

    request_count = len(bridge.executor_request_publisher.messages)
    bridge._check_atomic_dwell(float("inf"))

    assert len(bridge.executor_request_publisher.messages) == request_count
    assert bridge.motion_in_progress is False


def test_pickup_forward_rechecks_immediately_without_advancing_clock(
    monkeypatch,
):
    now = [100.0]
    monkeypatch.setattr(
        "mission_control.motion_command_bridge_node.time.monotonic",
        lambda: now[0],
    )
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    continue_pickup_after_initial_alignment(bridge)

    now[0] = 110.0
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="ball_camera_down_forward_4",
        )
    )
    request_count = len(bridge.executor_request_publisher.messages)
    assert now[0] == 110.0
    assert bridge.active_dwell_until is None
    requests = decoded_messages(bridge.executor_request_publisher)
    assert len(requests) == request_count
    assert bridge.pickup_initial_align_waiting is True


def test_pickup_positioning_loss_reacquires_immediately_after_motion(
    monkeypatch,
):
    now = [100.0]
    monkeypatch.setattr(
        "mission_control.motion_command_bridge_node.time.monotonic",
        lambda: now[0],
    )
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    now[0] = 103.0
    bridge._check_atomic_dwell(now[0])
    continue_pickup_after_initial_alignment(bridge)
    request_count = len(bridge.executor_request_publisher.messages)

    bridge.executor_status_callback(
        executor_status(
            status="RUNNING",
            motion_id="ball_camera_down_forward_4",
        )
    )
    bridge.navigation_command_callback(
        pickup_positioning_loss_message(
            bridge,
            "ball_camera_down_forward_4",
        )
    )
    assert bridge.pickup_positioning_loss_pending is True
    assert len(bridge.executor_request_publisher.messages) == request_count

    now[0] = 110.0
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="ball_camera_down_forward_4",
        )
    )
    assert now[0] == 110.0
    assert bridge.active_dwell_until is None
    assert bridge.pickup_initial_align_waiting is True
    assert bridge.pickup_positioning_loss_pending is False
    assert len(bridge.executor_request_publisher.messages) == request_count
    assert decoded_messages(bridge.motion_status_publisher)[-1][
        "motion_id"
    ] == bridge.PICKUP_INITIAL_ALIGN_MARKER


def test_pickup_positioning_loss_latch_is_ignored_after_fixed_grasp_starts():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    bridge.navigation_command_callback(
        fine_alignment_message("BALL_PICKUP_FINE_ALIGN_CONTINUE")
    )
    assert bridge.pickup_fixed_sequence_started is True
    assert decoded_messages(bridge.executor_request_publisher)[-1][
        "motion_id"
    ] == "pickup_pre_backward_camera_down"

    bridge.navigation_command_callback(
        pickup_positioning_loss_message(
            bridge,
            "pickup_pre_backward_camera_down",
            command_id=8003,
        )
    )
    assert bridge.pickup_positioning_loss_pending is False
    assert decoded_messages(bridge.motion_status_publisher)[-1][
        "error_code"
    ] == "PICKUP_POSITIONING_LOSS_LATCH_NOT_ACTIVE"

    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="pickup_pre_backward_camera_down",
        )
    )
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert decoded_messages(bridge.executor_request_publisher)[-1][
        "motion_id"
    ] == "pickup"


def test_pickup_crab_loss_reacquires_before_returning_to_fine_checkpoint():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    bridge.navigation_command_callback(
        fine_alignment_message("BALL_PICKUP_CRAB_RIGHT")
    )
    assert bridge.active_motion_id == "pickup_fine_to_crab_right_0"
    complete_active_motion(bridge)
    bridge.executor_status_callback(
        executor_status(status="RUNNING", motion_id="pickup_crab_right_0")
    )
    bridge.navigation_command_callback(
        pickup_positioning_loss_message(
            bridge,
            "pickup_crab_right_0",
            command_id=8002,
        )
    )

    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="pickup_crab_right_0",
        )
    )
    assert_correction_dwell_blocks_then_finishes(bridge)
    assert bridge.active_dwell_until is None
    assert bridge.pickup_initial_align_waiting is False
    assert bridge.pickup_fine_align_waiting is True

    bridge.navigation_command_callback(
        fine_alignment_message("BALL_PICKUP_FINE_FORWARD", command_id=8003)
    )
    complete_pickup_motion_and_dwell(
        bridge,
        "pickup_fine_forward_0",
    )
    assert bridge.pickup_fine_align_waiting is True
    assert bridge.pickup_fixed_sequence_started is False

    bridge.navigation_command_callback(
        fine_alignment_message(
            "BALL_PICKUP_FINE_ALIGN_CONTINUE",
            command_id=8004,
        )
    )
    assert decoded_messages(bridge.executor_request_publisher)[-1][
        "motion_id"
    ] == "pickup_pre_backward_camera_down"


def test_pickup_entry_requests_fresh_alignment_immediately():
    bridge = FakeBridge()
    assert bridge.PICKUP_DWELL_SEC == 1.0
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )

    assert bridge.motion_in_progress is True
    assert bridge.active_dwell_until is None
    assert bridge.pickup_initial_align_dwell_until is None
    assert bridge.pickup_initial_align_waiting is True
    assert bridge.executor_request_publisher.messages == []
    status = decoded_messages(bridge.motion_status_publisher)[-1]
    assert status["status"] == "RUNNING"
    assert status["motion_id"] == bridge.PICKUP_INITIAL_ALIGN_MARKER

    assert bridge.pickup_initial_align_dwell_until is None
    assert bridge.pickup_initial_align_waiting is True
    assert decoded_messages(bridge.motion_status_publisher)[-1][
        "motion_id"
    ] == bridge.PICKUP_INITIAL_ALIGN_MARKER


def test_pickup_missing_ball_aligns_after_camera_down_backward_motion():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)

    bridge.navigation_command_callback(
        navigation_message(
            action="BALL_PICKUP_INITIAL_ALIGN_CONTINUE",
            command_id=8001,
            event_id=None,
            source_command={
                "pickup_approach_motion": "STRAIGHT_0",
                "use_no_ball_pickup_path": True,
            },
            active_special_command_id=bridge.active_command_id,
            active_special_event_id=bridge.active_event_id,
        )
    )

    requests = decoded_messages(bridge.executor_request_publisher)
    assert [request["motion_id"] for request in requests] == [
        "ball_general_fine_forward_8"
    ]
    assert bridge.active_pickup_sequence.count(
        "ball_general_fine_forward_8"
    ) == 1
    assert bridge.active_pickup_sequence[:11] == (
        "ball_general_fine_forward_8",
        bridge.DWELL_MARKER,
        "pickup_pre_backward_camera_down",
        bridge.DWELL_MARKER,
        bridge.POST_BACKWARD_ALIGN_MARKER,
        "pickup",
        bridge.DWELL_MARKER,
        "pickup_grasp_check_pose",
        bridge.DWELL_MARKER,
        "pickup_retreat_2",
        bridge.DWELL_MARKER,
    )

    complete_pickup_motion_and_dwell(bridge, "ball_general_fine_forward_8")
    assert decoded_messages(bridge.executor_request_publisher)[-1][
        "motion_id"
    ] == "pickup_pre_backward_camera_down"

    complete_pickup_motion_and_dwell(
        bridge,
        "pickup_pre_backward_camera_down",
    )
    assert bridge.pickup_post_backward_align_waiting is True
    assert decoded_messages(bridge.motion_status_publisher)[-1][
        "motion_id"
    ] == bridge.POST_BACKWARD_ALIGN_MARKER

    bridge.navigation_command_callback(
        navigation_message(
            action="BALL_PICKUP_POST_BACKWARD_TURN_RIGHT_3",
            command_id=8002,
            event_id=None,
            active_special_command_id=bridge.active_command_id,
            active_special_event_id=bridge.active_event_id,
        )
    )
    assert decoded_messages(bridge.executor_request_publisher)[-1][
        "motion_id"
    ] == "pickup_camera_down_turn_right_3"
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="pickup_camera_down_turn_right_3",
        )
    )
    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert bridge.pickup_post_backward_align_waiting is True

    bridge.navigation_command_callback(
        navigation_message(
            action="BALL_PICKUP_POST_BACKWARD_CRAB_LEFT",
            command_id=8003,
            event_id=None,
            active_special_command_id=bridge.active_command_id,
            active_special_event_id=bridge.active_event_id,
        )
    )
    assert decoded_messages(bridge.executor_request_publisher)[-1][
        "motion_id"
    ] == "pickup_crab_left_0"
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="pickup_crab_left_0",
        )
    )
    assert_correction_dwell_blocks_then_finishes(bridge)
    assert bridge.active_dwell_until is None
    assert bridge.pickup_post_backward_align_waiting is True

    bridge.navigation_command_callback(
        navigation_message(
            action="BALL_PICKUP_POST_BACKWARD_ALIGN_CONTINUE",
            command_id=8004,
            event_id=None,
            active_special_command_id=bridge.active_command_id,
            active_special_event_id=bridge.active_event_id,
        )
    )
    assert decoded_messages(bridge.executor_request_publisher)[-1][
        "motion_id"
    ] == "pickup"


def test_camera_down_turn_success_requires_fresh_heading_recheck():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)

    bridge.navigation_command_callback(
        navigation_message(
            action="BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_3",
            command_id=8001,
            event_id=None,
            active_special_command_id=8000,
            active_special_event_id=8,
        )
    )
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request["motion_id"] == "pickup_camera_down_turn_right_3"
    assert bridge.pickup_initial_align_correction_active is True

    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="pickup_camera_down_turn_right_3",
        )
    )

    assert bridge.active_dwell_until is not None
    assert bridge.pickup_initial_align_waiting is False
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert bridge.pickup_initial_align_waiting is True
    assert decoded_messages(bridge.motion_status_publisher)[-1][
        "motion_id"
    ] == bridge.PICKUP_INITIAL_ALIGN_MARKER


@pytest.mark.parametrize(
    ("action", "motion_id"),
    [
        ("BALL_PICKUP_INITIAL_CRAB_LEFT", "pickup_crab_left_0"),
        ("BALL_PICKUP_INITIAL_CRAB_RIGHT", "pickup_crab_right_0"),
    ],
)
def test_initial_close_crab_success_requires_fresh_alignment_recheck(
    action,
    motion_id,
):
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)

    bridge.navigation_command_callback(
        navigation_message(
            action=action,
            command_id=8001,
            event_id=None,
            active_special_command_id=8000,
            active_special_event_id=8,
        )
    )

    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request["motion_id"] == motion_id
    assert bridge.pickup_initial_align_correction_active is True

    bridge.executor_status_callback(
        executor_status(status="SUCCEEDED", motion_id=motion_id)
    )
    bridge._check_atomic_dwell(bridge.active_dwell_until)

    assert bridge.pickup_initial_align_waiting is True


def test_camera_down_turn_can_repeat_after_each_fresh_heading_recheck():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)

    bridge.navigation_command_callback(
        navigation_message(
            action="BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_3",
            command_id=8001,
            event_id=None,
            source_command={
                "bottom_distance_px": 500,
                "pickup_fine_step_distance_m": 0.570,
            },
            active_special_command_id=8000,
            active_special_event_id=8,
        )
    )
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="pickup_camera_down_turn_right_3",
        )
    )
    bridge._check_atomic_dwell(bridge.active_dwell_until)

    bridge.navigation_command_callback(
        navigation_message(
            action="BALL_PICKUP_CAMERA_DOWN_TURN_LEFT_2",
            command_id=8002,
            event_id=None,
            source_command={
                "bottom_distance_px": 500,
                "pickup_fine_step_distance_m": 0.570,
            },
            active_special_command_id=8000,
            active_special_event_id=8,
        )
    )

    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request["motion_id"] == "pickup_camera_down_turn_left_2"
    assert bridge.pickup_initial_align_correction_active is True


def test_pickup_forward_turn_forward_keeps_only_turn_dwell():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)

    continue_pickup_after_initial_alignment(
        bridge,
        approach_motion="STRAIGHT_2",
        command_id=8001,
    )
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="ball_camera_down_forward_4",
        )
    )
    assert bridge.active_dwell_until is None
    assert bridge.pickup_initial_align_waiting is True

    bridge.navigation_command_callback(
        navigation_message(
            action="BALL_PICKUP_CAMERA_DOWN_TURN_RIGHT_3",
            command_id=8002,
            event_id=None,
            active_special_command_id=8000,
            active_special_event_id=8,
        )
    )
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="pickup_camera_down_turn_right_3",
        )
    )
    second_dwell = bridge.active_dwell_until
    assert second_dwell is not None
    bridge._check_atomic_dwell(second_dwell)
    assert bridge.pickup_initial_align_waiting is True

    continue_pickup_after_initial_alignment(
        bridge,
        approach_motion="STRAIGHT_2",
        command_id=8003,
    )
    assert [
        request["motion_id"]
        for request in decoded_messages(bridge.executor_request_publisher)
    ] == [
        "ball_camera_down_forward_4",
        "pickup_camera_down_turn_right_3",
        "ball_camera_down_forward_4",
    ]


@pytest.mark.parametrize("approach_motion", ["STRAIGHT_1", "STRAIGHT_3", "STRAIGHT_4"])
def test_pickup_rejects_removed_intermediate_distance_buckets(
    approach_motion,
):
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(
        bridge,
        approach_motion=approach_motion,
    )

    assert bridge.executor_request_publisher.messages == []
    status = decoded_messages(bridge.motion_status_publisher)[-1]
    assert status["status"] == "REJECTED"
    assert status["error_code"] == "UNSUPPORTED_PICKUP_DISTANCE_APPROACH"


def test_new_command_cannot_enter_during_atomic_pickup_sequence():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )

    bridge.navigation_command_callback(
        navigation_message(command_id=8001, event_id=9, action="SHOT")
    )

    assert len(bridge.executor_request_publisher.messages) == 0
    status = decoded_messages(bridge.motion_status_publisher)[-1]
    assert status["status"] == "REJECTED"
    assert status["error_code"] == "ATOMIC_SEQUENCE_LOCKED"


def test_new_command_cannot_enter_during_post_ball_transition():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="POST_BALL_GOAL_TRANSITION")
    )

    bridge.navigation_command_callback(
        navigation_message(command_id=8001, event_id=9, action="STRAIGHT")
    )

    requests = decoded_messages(bridge.executor_request_publisher)
    assert [request["motion_id"] for request in requests] == [
        "post_ball_camera_90"
    ]
    status = decoded_messages(bridge.motion_status_publisher)[-1]
    assert status["status"] == "REJECTED"
    assert status["error_code"] == "ATOMIC_SEQUENCE_LOCKED"


def test_pickup_sequence_stops_when_an_intermediate_motion_fails():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)
    complete_pickup_motion_and_dwell(
        bridge,
        "ball_camera_down_forward_4",
    )

    bridge.executor_status_callback(
        executor_status(
            status="FAILED",
            motion_id="ball_general_fine_forward_8",
            error_code="MOTION_FAILED",
        )
    )

    requests = decoded_messages(bridge.executor_request_publisher)
    assert [request["motion_id"] for request in requests] == [
        "ball_camera_down_forward_4",
        "ball_general_fine_forward_8",
    ]
    assert bridge.motion_in_progress is False


@pytest.mark.parametrize(
    "error_code",
    [
        "SDK_COMMUNICATION_ERROR",
        "SDK_FRAME_SEND_FAILED",
        "SDK_PRESENT_POSITION_READ_FAILED",
        "SDK_POSITION_TIMEOUT",
    ],
)
def test_pickup_sequence_continues_after_recoverable_motor_fault(error_code):
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)
    complete_pickup_motion_and_dwell(
        bridge,
        "ball_camera_down_forward_4",
    )

    bridge.executor_status_callback(
        executor_status(
            status="FAILED",
            motion_id="ball_general_fine_forward_8",
            error_code=error_code,
        )
    )

    assert bridge.motion_in_progress is True
    assert_correction_dwell_blocks_then_finishes(bridge)
    assert bridge.active_dwell_until is None
    assert bridge.pickup_fine_align_waiting is True


@pytest.mark.parametrize("error_code", sorted(RECOVERABLE_MOTOR_ERROR_CODES))
def test_pickup_stops_after_two_consecutive_motor_failures(error_code):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    continue_pickup_after_fine_alignment(bridge)
    first = executor_status(
        status="FAILED", motion_id="pickup_pre_backward_camera_down",
        error_code=error_code, message="injected first fault",
    )
    bridge.executor_status_callback(first)
    assert bridge.motion_in_progress
    assert bridge.last_physical_motion_id is None
    assert bridge.last_pickup_motion_id is None
    assert bridge.pickup_consecutive_motor_failures == 1
    # A repeated message during the dwell is not a second execution failure.
    bridge.executor_status_callback(first)
    assert bridge.pickup_consecutive_motor_failures == 1
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert bridge.active_motion_id == "pickup"
    bridge.executor_status_callback(executor_status(
        status="RUNNING", motion_id="pickup",
    ))
    assert bridge.pickup_consecutive_motor_failures == 1
    count = len(bridge.executor_request_publisher.messages)
    bridge.executor_status_callback(executor_status(
        status="FAILED", motion_id="pickup", error_code=error_code,
        message="injected second fault",
    ))
    terminal = decoded_messages(bridge.motion_status_publisher)[-1]
    assert terminal["status"] == "FAILED"
    assert terminal["error_code"] == error_code
    assert "consecutive motor failures" in terminal["message"]
    assert [fault["status"] for fault in terminal["motor_failures"]] == ["FAILED", "FAILED"]
    assert not bridge.motion_in_progress
    bridge._check_atomic_dwell()
    assert len(bridge.executor_request_publisher.messages) == count


def test_pickup_recovers_normally_and_retains_faults_in_sequence_summary():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    continue_pickup_after_fine_alignment(bridge)
    complete_pickup_motion_and_dwell(bridge, "pickup_pre_backward_camera_down")
    bridge.executor_status_callback(executor_status(
        status="FAILED", motion_id="pickup", error_code="SDK_FRAME_SEND_FAILED",
        message="injected Goal write failure",
    ))
    assert bridge.last_completed_motion_id != "pickup"
    skipped = decoded_messages(bridge.motion_status_publisher)[-1]
    assert skipped["status"] == "RUNNING"
    assert "completed_motion_id" not in skipped
    assert skipped["motor_failures"][0]["status"] == "FAILED"
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    complete_pickup_motion_and_dwell(bridge, "pickup_grasp_check_pose")
    assert bridge.pickup_consecutive_motor_failures == 0
    # Successful intervening execution permits another isolated fault.
    bridge.executor_status_callback(executor_status(
        status="FAILED", motion_id="pickup_retreat_2", error_code="SDK_FRAME_SEND_FAILED",
    ))
    assert bridge.motion_in_progress
    assert bridge.pickup_consecutive_motor_failures == 1
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    complete_pickup_motion_and_dwell(bridge, "pickup_first_backward_turn_right")
    terminal = decoded_messages(bridge.motion_status_publisher)[-1]
    assert terminal["status"] == "SUCCEEDED"  # The sequence finished, including skipped stages.
    assert [fault["motion_id"] for fault in terminal["motor_failures"]] == ["pickup", "pickup_retreat_2"]
    assert all(fault["status"] == "FAILED" for fault in terminal["motor_failures"])
    assert not bridge.motion_in_progress
    assert bridge.pickup_motor_failures == []


def test_failed_grasp_check_pose_keeps_grasp_unknown_and_continues_retreat():
    from test_motion_decision_node import FakeDecisionNode, arm_special_command
    from mission_control.motion_decision_node import MotionDecisionNode

    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    continue_pickup_after_fine_alignment(bridge)
    for motion in ("pickup_pre_backward_camera_down", "pickup"):
        complete_pickup_motion_and_dwell(bridge, motion)
    node = FakeDecisionNode("BALL_APPROACH")
    arm_special_command(node, "PICKUP_NOW", 8000, 8)
    offset = len(bridge.motion_status_publisher.messages)
    bridge.executor_status_callback(executor_status(
        status="FAILED", motion_id="pickup_grasp_check_pose",
        error_code="SDK_PRESENT_POSITION_READ_FAILED",
    ))
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert bridge.active_motion_id == "pickup_retreat_2"
    for message in bridge.motion_status_publisher.messages[offset:]:
        payload = json.loads(message.data)
        assert "verification_window_complete" not in payload
        MotionDecisionNode._motion_status_callback(node, message)
    assert not getattr(node, "grasp_verification_active", False)
    assert node.phase_manager.grasp_result_for_ball(1) == "UNKNOWN"
    assert not node.safety_interlock.latched


def test_second_pickup_finishes_with_backward_left_turn_composite():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(
            action="PICKUP_NOW",
            mission_progress={
                "pickups_completed": 1,
                "required_pickups": 2,
            },
        )
    )
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)

    for completed_motion in (
        "ball_camera_down_forward_4",
        "ball_general_fine_forward_8",
    ):
        complete_pickup_motion_and_dwell(bridge, completed_motion)
    continue_pickup_after_fine_alignment(bridge)
    for completed_motion in (
        "pickup_pre_backward_camera_down",
        "pickup",
        "pickup_grasp_check_pose",
        "pickup_retreat_2",
    ):
        complete_pickup_motion_and_dwell(bridge, completed_motion)
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="pickup_second_backward_turn_left",
        )
    )

    requests = decoded_messages(bridge.executor_request_publisher)
    assert requests[-1]["motion_id"] == "pickup_second_backward_turn_left"
    assert_correction_dwell_blocks_then_finishes(bridge)
    assert bridge.active_dwell_until is None
    statuses = decoded_messages(bridge.motion_status_publisher)
    assert statuses[-1]["status"] == "SUCCEEDED"
    assert bridge.motion_in_progress is False


@pytest.mark.parametrize(
    ("approach_motion", "motion_id"),
    [
        ("STRAIGHT_0", "ball_general_fine_forward_8"),
        ("STRAIGHT_2", "ball_camera_down_forward_4"),
    ],
)
def test_pickup_reuses_existing_straight_bucket_for_camera_down_forward(
    approach_motion,
    motion_id,
):
    bridge = FakeBridge()

    bridge.navigation_command_callback(
        navigation_message(
            action="PICKUP_NOW",
        )
    )
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(
        bridge,
        approach_motion=approach_motion,
    )

    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request["motion_id"] == motion_id


def test_unsupported_pickup_sequence_is_rejected_without_motion():
    bridge = FakeBridge()

    bridge.navigation_command_callback(
        navigation_message(
            action="PICKUP_NOW",
            mission_progress={
                "pickups_completed": 2,
                "required_pickups": 2,
            },
        )
    )

    assert bridge.executor_request_publisher.messages == []
    status = decoded_messages(bridge.motion_status_publisher)[0]
    assert status["status"] == "UNSUPPORTED"
    assert status["error_code"] == "UNSUPPORTED_PICKUP_SEQUENCE"


def test_pickup_sequence_ignores_a_stale_stage_status():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="ball_camera_down_forward_4",
        )
    )

    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="ball_camera_down_forward_4",
        )
    )

    assert bridge.active_dwell_until is None

    assert [
        request["motion_id"]
        for request in decoded_messages(bridge.executor_request_publisher)
    ] == [
        "ball_camera_down_forward_4",
    ]
    assert bridge.pickup_initial_align_waiting is True


def enter_pickup_fine_alignment(bridge):
    """Advance a pickup through fine forward into its Vision checkpoint."""
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)
    for motion_id in (
        "ball_camera_down_forward_4",
        "ball_general_fine_forward_8",
    ):
        complete_pickup_motion_and_dwell(bridge, motion_id)


def fine_alignment_message(action, command_id=8001):
    """Build one checkpoint command correlated to the active pickup."""
    return navigation_message(
        action=action,
        command_id=command_id,
        event_id=None,
        active_special_command_id=8000,
        active_special_event_id=8,
    )


@pytest.mark.parametrize("direction", ["LEFT", "RIGHT"])
def test_forward4_then_crab_starts_directly_and_preserves_checkpoint(direction):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW"))
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)
    complete_active_motion(bridge)
    assert bridge.pickup_initial_align_waiting

    action = f"BALL_PICKUP_INITIAL_CRAB_{direction}"
    crab = f"pickup_crab_{direction.lower()}_0"
    bridge.navigation_command_callback(fine_alignment_message(action, 8002))
    assert bridge.active_motion_id == crab
    assert bridge.pending_pickup_crab_motion_id is None
    bridge.executor_status_callback(executor_status(
        status="SUCCEEDED", motion_id="ball_camera_down_forward_4",
    ))
    assert bridge.active_motion_id == crab
    complete_active_motion(bridge)
    assert_correction_dwell_blocks_then_finishes(bridge)
    assert bridge.active_dwell_until is None
    assert bridge.pickup_initial_align_waiting
    bridge.navigation_command_callback(fine_alignment_message(action, 8003))
    assert bridge.active_motion_id == crab
    requests = decoded_messages(bridge.executor_request_publisher)
    assert [r["motion_id"] for r in requests] == [
        "ball_camera_down_forward_4", crab, crab,
    ]


@pytest.mark.parametrize("status,error_code", [
    ("FAILED", "MOTION_FAILED"), ("TIMEOUT", "TIMEOUT"), ("CANCELLED", ""),
    *[("FAILED", code) for code in sorted(RECOVERABLE_MOTOR_ERROR_CODES)],
])
def test_post_ball_camera_fault_cannot_complete_transition(status, error_code):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="POST_BALL_GOAL_TRANSITION"))
    complete_active_motion(bridge, status, error_code)
    assert not bridge.motion_in_progress
    assert len(bridge.executor_request_publisher.messages) == 1
    assert bridge.post_ball_camera_pause_until is None
    assert decoded_messages(bridge.motion_status_publisher)[-1]["status"] == status


def test_pickup_distance_approach_skips_preparation_and_waits_for_fine_alignment():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW"))
    assert_pickup_initial_alignment_started(bridge)
    bridge.navigation_command_callback(navigation_message(
        action="BALL_PICKUP_INITIAL_ALIGN_CONTINUE", command_id=8001,
        event_id=None, source_command={"pickup_approach_motion": "STRAIGHT_0"},
        active_special_command_id=8000, active_special_event_id=8,
    ))
    requests = decoded_messages(bridge.executor_request_publisher)
    assert [r["motion_id"] for r in requests] == ["ball_general_fine_forward_8"]
    assert bridge.pickup_fine_align_waiting is False
    complete_pickup_motion_and_dwell(bridge, "ball_general_fine_forward_8")
    assert bridge.pickup_fine_align_waiting is True
    assert len(bridge.executor_request_publisher.messages) == 1


def test_pickup_fine_preparation_must_finish_before_pre_grasp_fine_forward():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    start = len(bridge.executor_request_publisher.messages)
    bridge.navigation_command_callback(fine_alignment_message("BALL_PICKUP_FINE_FORWARD"))
    requests = decoded_messages(bridge.executor_request_publisher)[start:]
    assert [r["motion_id"] for r in requests] == ["pickup_fine_prepare"]
    bridge.executor_status_callback(executor_status(
        status="RUNNING", motion_id="pickup_fine_prepare",
    ))
    assert len(bridge.executor_request_publisher.messages) == start + 1
    assert bridge.pickup_fine_align_waiting is False
    complete_pickup_fine_preparation(bridge)
    requests = decoded_messages(bridge.executor_request_publisher)[start:]
    assert [r["motion_id"] for r in requests] == [
        "pickup_fine_prepare", "pickup_fine_forward_0",
    ]
    for key in ("action", "command_id", "event_id", "request_id"):
        assert requests[0][key] == requests[1][key]
    # A duplicate preparation completion cannot skip the pending fine motion.
    bridge.executor_status_callback(executor_status(
        status="SUCCEEDED", motion_id="pickup_fine_prepare",
    ))
    assert len(bridge.executor_request_publisher.messages) == start + 2
    assert bridge.active_dwell_until is None
    complete_pickup_motion_and_dwell(bridge, "pickup_fine_forward_0")
    assert bridge.pickup_fine_align_waiting is True


@pytest.mark.parametrize("status,error_code", [
    ("FAILED", "MOTION_FAILED"), ("TIMEOUT", "TIMEOUT"),
    ("CANCELLED", ""), ("REJECTED", "MOTION_NOT_FOUND"),
    *[("FAILED", code) for code in sorted(RECOVERABLE_MOTOR_ERROR_CODES)],
])
def test_pickup_fine_preparation_failure_never_starts_fine_motion(status, error_code):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    start = len(bridge.executor_request_publisher.messages)
    bridge.navigation_command_callback(fine_alignment_message("BALL_PICKUP_FINE_FORWARD"))
    complete_active_motion(bridge, status, error_code)
    bridge._check_atomic_dwell(float("inf"))
    requests = decoded_messages(bridge.executor_request_publisher)[start:]
    assert [r["motion_id"] for r in requests] == ["pickup_fine_prepare"]
    assert not bridge.motion_in_progress
    terminal = decoded_messages(bridge.motion_status_publisher)[-1]
    assert terminal["status"] == status
    assert terminal["error_code"] == error_code


def test_fine_forward_success_waits_before_grab_stage():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)

    requests = decoded_messages(bridge.executor_request_publisher)
    assert [request["motion_id"] for request in requests] == [
        "ball_camera_down_forward_4",
        "ball_general_fine_forward_8",
    ]
    assert bridge.pickup_fine_align_waiting is True
    assert bridge.motion_in_progress is True
    status = decoded_messages(bridge.motion_status_publisher)[-1]
    assert status["status"] == "RUNNING"
    assert status["motion_id"] == bridge.FINE_ALIGN_MARKER


def test_centered_fine_alignment_starts_backward_without_extra_fine_step():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)

    bridge.navigation_command_callback(
        fine_alignment_message("BALL_PICKUP_FINE_ALIGN_CONTINUE")
    )

    requests = decoded_messages(bridge.executor_request_publisher)
    assert [request["motion_id"] for request in requests] == [
        "ball_camera_down_forward_4",
        "ball_general_fine_forward_8",
        "pickup_pre_backward_camera_down",
    ]


def test_fine_forward_success_requires_another_checkpoint_before_grab():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)

    bridge.navigation_command_callback(
        fine_alignment_message("BALL_PICKUP_FINE_FORWARD")
    )
    complete_pickup_fine_preparation(bridge)
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request["motion_id"] == "pickup_fine_forward_0"
    assert request["action"] == "PICKUP_NOW"
    assert request["command_id"] == 8000

    bridge.executor_status_callback(
        executor_status(status="SUCCEEDED", motion_id="pickup_fine_forward_0")
    )

    assert_correction_dwell_blocks_then_finishes(bridge)
    assert bridge.active_dwell_until is None
    assert bridge.pickup_initial_align_waiting is False
    assert bridge.pickup_fine_align_waiting is True
    assert decoded_messages(bridge.motion_status_publisher)[-1][
        "motion_id"
    ] == bridge.FINE_ALIGN_MARKER


def test_pickup_distance_decisions_repeat_fine_then_crab_then_backward():
    bridge = FakeBridge()
    planner = MotionDecisionPlanner()
    enter_pickup_fine_alignment(bridge)
    threshold = planner.config.pickup_fine_align_bottom_distance_px
    sample = {
        "detected": True,
        "confidence": 0.9,
        "offset_x_px": 56,
    }

    for command_id, bottom_distance_px in enumerate((threshold + 150, threshold + 100, threshold + 1), 8100):
        decision = planner.plan_ball_pickup_fine_alignment({
            **sample,
            "bottom_distance_px": bottom_distance_px,
        })
        assert decision.action == "BALL_PICKUP_FINE_FORWARD"
        bridge.navigation_command_callback(
            fine_alignment_message(decision.action, command_id=command_id)
        )
        complete_pickup_fine_preparation(bridge)
        requests = decoded_messages(bridge.executor_request_publisher)
        assert requests[-1]["motion_id"] == "pickup_fine_forward_0"
        complete_pickup_motion_and_dwell(bridge, "pickup_fine_forward_0")
        assert bridge.pickup_fine_align_waiting is True
        assert bridge.pickup_fixed_sequence_started is False
        assert len(decoded_messages(bridge.executor_request_publisher)) == len(
            requests
        )

    decision = planner.plan_ball_pickup_fine_alignment({
        **sample,
        "bottom_distance_px": threshold,
    })
    assert decision.action == "BALL_PICKUP_CRAB_RIGHT"
    bridge.navigation_command_callback(
        fine_alignment_message(decision.action, command_id=8103)
    )
    assert bridge.active_motion_id == "pickup_fine_to_crab_right_0"
    complete_active_motion(bridge)
    complete_pickup_motion_and_dwell(bridge, "pickup_crab_right_0")
    assert bridge.pickup_fine_align_waiting is True
    assert bridge.pickup_fixed_sequence_started is False

    decision = planner.plan_ball_pickup_fine_alignment({
        **sample,
        "bottom_distance_px": threshold,
        "offset_x_px": 0,
    })
    assert decision.action == "BALL_PICKUP_FINE_ALIGN_CONTINUE"
    bridge.navigation_command_callback(
        fine_alignment_message(decision.action, command_id=8104)
    )
    assert bridge.pickup_fixed_sequence_started is True
    assert decoded_messages(bridge.executor_request_publisher)[-1][
        "motion_id"
    ] == "pickup_pre_backward_camera_down"


def test_right_crab_success_requires_another_checkpoint_before_grab():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)

    bridge.navigation_command_callback(
        fine_alignment_message("BALL_PICKUP_CRAB_RIGHT")
    )
    assert bridge.active_motion_id == "pickup_fine_to_crab_right_0"
    complete_active_motion(bridge)
    crab_request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert crab_request["motion_id"] == "pickup_crab_right_0"
    assert crab_request["action"] == "PICKUP_NOW"
    assert crab_request["command_id"] == 8000
    assert crab_request["event_id"] == 8

    bridge.executor_status_callback(
        executor_status(status="SUCCEEDED", motion_id="pickup_crab_right_0")
    )

    assert_correction_dwell_blocks_then_finishes(bridge)
    assert bridge.active_dwell_until is None
    assert bridge.pickup_initial_align_waiting is False
    assert bridge.pickup_fine_align_waiting is True
    assert decoded_messages(bridge.executor_request_publisher)[-1][
        "motion_id"
    ] == "pickup_crab_right_0"
    assert decoded_messages(bridge.motion_status_publisher)[-1][
        "motion_id"
    ] == bridge.FINE_ALIGN_MARKER

    bridge.navigation_command_callback(
        fine_alignment_message("BALL_PICKUP_CRAB_RIGHT", command_id=8002)
    )
    assert decoded_messages(bridge.executor_request_publisher)[-1][
        "motion_id"
    ] == "pickup_crab_right_0"


def test_left_crab_success_requires_another_checkpoint_before_grab():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    bridge.navigation_command_callback(
        fine_alignment_message("BALL_PICKUP_CRAB_LEFT")
    )
    crab_request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert crab_request["motion_id"] == "pickup_crab_left_0"
    assert crab_request["action"] == "PICKUP_NOW"

    bridge.executor_status_callback(
        executor_status(status="SUCCEEDED", motion_id="pickup_crab_left_0")
    )
    assert_correction_dwell_blocks_then_finishes(bridge)
    assert bridge.active_dwell_until is None
    assert bridge.pickup_initial_align_waiting is False
    assert bridge.pickup_fine_align_waiting is True
    assert decoded_messages(bridge.motion_status_publisher)[-1][
        "motion_id"
    ] == bridge.FINE_ALIGN_MARKER

    bridge.navigation_command_callback(
        fine_alignment_message("BALL_PICKUP_CRAB_LEFT", command_id=8002)
    )
    assert decoded_messages(bridge.executor_request_publisher)[-1][
        "motion_id"
    ] == "pickup_crab_left_0"


def test_pickup_crab_repeats_without_limit_until_pixel_window_is_met():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)

    for command_id in (8001, 8002, 8003):
        bridge.navigation_command_callback(
            fine_alignment_message(
                "BALL_PICKUP_CRAB_RIGHT",
                command_id=command_id,
            )
        )
        if command_id == 8001:
            assert bridge.active_motion_id == "pickup_fine_to_crab_right_0"
            complete_active_motion(bridge)
        bridge.executor_status_callback(
            executor_status(
                status="SUCCEEDED",
                motion_id="pickup_crab_right_0",
            )
        )
        assert_correction_dwell_blocks_then_finishes(bridge)
        assert bridge.active_dwell_until is None
        assert bridge.pickup_fine_align_waiting is True

    bridge.navigation_command_callback(
        fine_alignment_message(
            "BALL_PICKUP_FINE_ALIGN_CONTINUE",
            command_id=8004,
        )
    )

    motion_ids = [
        request["motion_id"]
        for request in decoded_messages(bridge.executor_request_publisher)
    ]
    assert motion_ids.count("pickup_crab_right_0") == 3
    assert motion_ids[-1] == "pickup_pre_backward_camera_down"


def test_unrelated_motion_is_rejected_during_fine_alignment_wait():
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)

    bridge.navigation_command_callback(
        navigation_message(command_id=9000, action="SHOT")
    )

    status = decoded_messages(bridge.motion_status_publisher)[-1]
    assert status["status"] == "REJECTED"
    assert status["error_code"] == "ATOMIC_SEQUENCE_LOCKED"


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT"])
def test_right_crab_failure_aborts_original_pickup(status):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    bridge.navigation_command_callback(
        fine_alignment_message("BALL_PICKUP_CRAB_RIGHT")
    )
    assert bridge.active_motion_id == "pickup_fine_to_crab_right_0"
    complete_active_motion(bridge)

    bridge.executor_status_callback(
        executor_status(
            status=status,
            motion_id="pickup_crab_right_0",
            error_code="MOTION_FAILED",
        )
    )

    assert bridge.motion_in_progress is False
    result = decoded_messages(bridge.motion_status_publisher)[-1]
    assert result["status"] == status
    assert result["action"] == "PICKUP_NOW"


@pytest.mark.parametrize("camera_duration", [0.8, 3.5])
def test_post_ball_camera_completes_without_extra_stationary_pause(monkeypatch, camera_duration):
    now = [20.0]
    monkeypatch.setattr("mission_control.motion_command_bridge_node.time.monotonic", lambda: now[0])
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="POST_BALL_GOAL_TRANSITION"))
    assert [r["motion_id"] for r in decoded_messages(bridge.executor_request_publisher)] == ["post_ball_camera_90"]
    assert bridge.post_ball_camera_pause_until == 20.0
    now[0] += camera_duration
    bridge._check_atomic_dwell(now[0])
    assert bridge.motion_in_progress is True
    complete_active_motion(bridge)
    assert bridge.active_dwell_until is None
    assert bridge.motion_in_progress is False
    assert bridge.post_ball_camera_pause_until is None
    assert decoded_messages(bridge.motion_status_publisher)[-1]["status"] == "SUCCEEDED"
    assert len(bridge.executor_request_publisher.messages) == 1


def test_post_ball_duplicate_camera_success_does_not_complete_twice(monkeypatch):
    monkeypatch.setattr("mission_control.motion_command_bridge_node.time.monotonic", lambda: 20.0)
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="POST_BALL_GOAL_TRANSITION"))
    complete_active_motion(bridge)
    bridge.executor_status_callback(executor_status(
        status="SUCCEEDED", motion_id="post_ball_camera_90",
    ))
    assert bridge.active_dwell_until is None
    successes = [s for s in decoded_messages(bridge.motion_status_publisher)
                 if s["status"] == "SUCCEEDED"]
    assert len(successes) == 1
    assert bridge.motion_in_progress is False


@pytest.mark.parametrize("status,error_code", [
    ("FAILED", "MOTION_FAILED"), ("TIMEOUT", "TIMEOUT"),
    *[("FAILED", code) for code in RECOVERABLE_MOTOR_ERROR_CODES],
])
def test_camera90_failure_does_not_complete_ball_mode(status, error_code):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        action="POST_BALL_GOAL_TRANSITION",
    ))
    bridge.executor_status_callback(executor_status(
        status=status, motion_id="post_ball_camera_90", error_code=error_code,
    ))
    assert decoded_messages(bridge.motion_status_publisher)[-1]["status"] == status
    assert bridge.motion_in_progress is False


def test_goal_camera90_right_turn_uses_exact_counted_motion():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="GOAL_CAMERA90_TURN_RIGHT_3")
    )

    request = decoded_messages(bridge.executor_request_publisher)[0]
    assert request["motion_id"] == "goal_camera_90_turn_right_3"


def test_goal_camera90_left_turn_uses_exact_counted_motion():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="GOAL_CAMERA90_TURN_LEFT_3")
    )

    request = decoded_messages(bridge.executor_request_publisher)[0]
    assert request["motion_id"] == "goal_camera_90_turn_left_3"


@pytest.mark.parametrize("terminal_status", ["FAILED", "TIMEOUT"])
def test_post_ball_goal_transition_aborts_without_later_stages(
    terminal_status,
):
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="POST_BALL_GOAL_TRANSITION")
    )

    bridge.executor_status_callback(
        executor_status(
            status=terminal_status,
            motion_id="post_ball_camera_90",
            error_code="MOTION_FAILED",
        )
    )

    requests = decoded_messages(bridge.executor_request_publisher)
    assert [request["motion_id"] for request in requests] == [
        "post_ball_camera_90"
    ]
    assert bridge.motion_in_progress is False
    assert decoded_messages(bridge.motion_status_publisher)[-1][
        "status"
    ] == terminal_status


def test_running_status_keeps_lock_and_preserves_fields():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())

    bridge.executor_status_callback(executor_status(message="executing"))

    status = decoded_messages(bridge.motion_status_publisher)[0]
    assert status == {
        "status": "RUNNING",
        "command_id": 8000,
        "event_id": 8,
        "request_id": 8000,
        "motion_id": "forward",
        "error_code": "",
        "message": "executing",
        "action": "STRAIGHT",
        "source_node": "motion_command_bridge_node",
        "motion_in_progress": True,
    }
    assert bridge.motion_in_progress is True
    assert bridge.active_request_id == 8000


@pytest.mark.parametrize(
    ("terminal_status", "error_code"),
    [
        ("SUCCEEDED", ""),
        ("FAILED", "COMMUNICATION_ERROR"),
        ("REJECTED", "INVALID_MOTION"),
        ("CANCELLED", ""),
    ],
)
def test_terminal_status_releases_lock(terminal_status, error_code):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())

    bridge.executor_status_callback(
        executor_status(
            status=terminal_status,
            error_code=error_code,
            message="executor detail",
        )
    )

    status = decoded_messages(bridge.motion_status_publisher)[0]
    assert status["status"] == terminal_status
    assert status["error_code"] == error_code
    assert status["message"] == "executor detail"
    assert status["action"] == "STRAIGHT"
    assert status["motion_in_progress"] is False
    assert bridge.motion_in_progress is False
    assert bridge.active_command_id is None
    assert bridge.active_event_id is None
    assert bridge.active_action is None
    assert bridge.active_request_id is None
    assert bridge.active_motion_id is None


def test_mismatched_request_id_is_ignored():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())

    bridge.executor_status_callback(executor_status(request_id=9000))

    assert bridge.motion_status_publisher.messages == []
    assert bridge.motion_in_progress is True


@pytest.mark.parametrize(
    "payload",
    [
        "not-json",
        [],
        {"status": "RUNNING"},
        {
            "status": "UNKNOWN",
            "command_id": 8000,
            "event_id": 8,
            "request_id": 8000,
            "motion_id": "forward",
            "error_code": "",
            "message": "",
        },
    ],
)
def test_malformed_executor_status_is_ignored(payload):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message())

    bridge.executor_status_callback(string_message(payload))

    assert bridge.motion_status_publisher.messages == []
    assert bridge.motion_in_progress is True


def test_bridge_source_has_no_robot_msgs_or_dynamics_interface():
    source = inspect.getsource(MotionCommandBridgeNode)

    assert "robot_msgs" not in source
    assert '"/motion_command"' not in source
    assert "/motion_end" not in source
    assert "active_dynamics_command" not in source


@pytest.mark.parametrize("direction", ["LEFT", "RIGHT"])
def test_pickup_fine_search_uses_camera_down_turn_and_returns_after_dwell(direction):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    action = f"BALL_PICKUP_FINE_SEARCH_{direction}"
    bridge.navigation_command_callback(fine_alignment_message(action))
    motion = f"pickup_camera_down_turn_{direction.lower()}_{5 if direction == 'RIGHT' else 2}"
    assert decoded_messages(bridge.executor_request_publisher)[-1]["motion_id"] == "fine_to_turn_ready_0"
    bridge.executor_status_callback(executor_status(
        status="SUCCEEDED", motion_id="fine_to_turn_ready_0",
    ))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request["motion_id"] == motion
    assert request["action"] == "PICKUP_NOW"
    count = len(bridge.executor_request_publisher.messages)
    bridge.executor_status_callback(executor_status(status="SUCCEEDED", motion_id=motion))
    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until - 0.001)
    assert bridge.pickup_fine_align_waiting is False
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert bridge.pickup_fine_align_waiting is True
    assert bridge.pickup_fixed_sequence_started is False
    assert len(bridge.executor_request_publisher.messages) == count


@pytest.mark.parametrize("stage", ["INITIAL", "FINE"])
def test_pickup_top_loss_forward_returns_to_checkpoint_without_dwell(stage):
    bridge = FakeBridge()
    if stage == "FINE":
        enter_pickup_fine_alignment(bridge)
    else:
        bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW"))
        bridge._check_atomic_dwell(bridge.pickup_initial_align_dwell_until)
    bridge.navigation_command_callback(fine_alignment_message(
        f"BALL_PICKUP_{stage}_SEARCH_FORWARD"
    ))
    motion = "ball_camera_down_forward_2"
    assert decoded_messages(bridge.executor_request_publisher)[-1]["motion_id"] == motion
    bridge.executor_status_callback(executor_status(status="SUCCEEDED", motion_id=motion))
    count = len(bridge.executor_request_publisher.messages)
    assert bridge.active_dwell_until is None
    assert bridge.pickup_initial_align_waiting is (stage == "INITIAL")
    assert bridge.pickup_fine_align_waiting is (stage == "FINE")
    assert bridge.pickup_fixed_sequence_started is False
    assert len(bridge.executor_request_publisher.messages) == count


@pytest.mark.parametrize('earlier_fine_count', [0, 2])
def test_centered_pickup_starts_backward_without_repeating_fine_steps(earlier_fine_count):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    for index in range(earlier_fine_count):
        bridge.navigation_command_callback(fine_alignment_message(
            'BALL_PICKUP_FINE_FORWARD', command_id=8100 + index,
        ))
        complete_pickup_motion_and_dwell(bridge, 'pickup_fine_forward_0')
    start = len(bridge.executor_request_publisher.messages)
    continue_pickup_after_fine_alignment(bridge, command_id=8200)
    assert bridge.active_motion_id == 'pickup_pre_backward_camera_down'
    assert bridge.pickup_fixed_sequence_started is True
    assert [request['motion_id'] for request in
            decoded_messages(bridge.executor_request_publisher)[start:]] == [
        'pickup_pre_backward_camera_down',
    ]
    complete_pickup_motion_and_dwell(bridge, 'pickup_pre_backward_camera_down')
    assert bridge.active_motion_id == 'pickup'


@pytest.mark.parametrize('failed_motion', ['pickup_fine_prepare', 'pickup_fine_forward_0'])
def test_nonrecoverable_fine_correction_failure_never_starts_backward(failed_motion):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    bridge.navigation_command_callback(fine_alignment_message('BALL_PICKUP_FINE_FORWARD'))
    if failed_motion == 'pickup_fine_forward_0':
        complete_pickup_fine_preparation(bridge)
    complete_active_motion(bridge, status='FAILED', error_code='MOTION_FAILED')
    assert bridge.motion_in_progress is False
    assert all(request['motion_id'] != 'pickup_pre_backward_camera_down'
               for request in decoded_messages(bridge.executor_request_publisher))
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'FAILED'


def test_camera_transition_cannot_start_while_line_motion_is_running():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="STRAIGHT"))
    bridge.navigation_command_callback(navigation_message(
        action="POST_BALL_GOAL_TRANSITION", command_id=8001, event_id=9,
    ))
    assert len(bridge.executor_request_publisher.messages) == 1
    assert decoded_messages(bridge.motion_status_publisher)[-1]["error_code"] == "REJECTED_BUSY"
    assert bridge.queued_request_id is None


@pytest.mark.parametrize("stage", ["INITIAL", "FINE"])
def test_close_ball_backward_runs_once_per_checkpoint_and_preserves_pickup_tail(stage):
    bridge = FakeBridge()
    if stage == "FINE":
        enter_pickup_fine_alignment(bridge)
    else:
        bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW"))
        bridge._check_atomic_dwell(bridge.pickup_initial_align_dwell_until)
    original_tail = bridge.active_pickup_sequence
    original_index = bridge.active_sequence_index
    action = f"BALL_PICKUP_{stage}_SEARCH_BACKWARD"
    motion = "pickup_lost_ball_backward_1"
    for command_id in (8200, 8202):
        before = len(bridge.executor_request_publisher.messages)
        command = fine_alignment_message(action, command_id)
        bridge.navigation_command_callback(command)
        assert decoded_messages(bridge.executor_request_publisher)[-1]["motion_id"] == motion
        # Repeated or newly numbered requests while running cannot queue more retreat.
        bridge.navigation_command_callback(command)
        bridge.navigation_command_callback(fine_alignment_message(action, command_id + 1))
        assert len(bridge.executor_request_publisher.messages) == before + 1
        bridge.executor_status_callback(executor_status(status="SUCCEEDED", motion_id=motion))
        assert_correction_dwell_blocks_then_finishes(bridge)
        assert bridge.active_dwell_until is None
        assert bridge.pickup_initial_align_waiting is (stage == "INITIAL")
        assert bridge.pickup_fine_align_waiting is (stage == "FINE")
        assert len(bridge.executor_request_publisher.messages) == before + 1
        assert bridge.active_pickup_sequence == original_tail
        assert bridge.active_sequence_index == original_index
        assert not bridge.pickup_fixed_sequence_started
    if stage == "FINE":
        # Reacquisition and centering still run the separate mandatory pre-grasp backward.
        continue_pickup_after_fine_alignment(bridge, command_id=8300)
        assert bridge.active_motion_id == "pickup_pre_backward_camera_down"
        complete_pickup_motion_and_dwell(bridge, "pickup_pre_backward_camera_down")
        assert bridge.active_motion_id == "pickup"


@pytest.mark.parametrize("stage", ["INITIAL", "FINE"])
@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT", "CANCELLED"])
def test_close_ball_backward_failure_does_not_repeat_or_start_grasp(stage, status):
    bridge = FakeBridge()
    if stage == "FINE":
        enter_pickup_fine_alignment(bridge)
    else:
        bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW"))
        bridge._check_atomic_dwell(bridge.pickup_initial_align_dwell_until)
    bridge.navigation_command_callback(fine_alignment_message(
        f"BALL_PICKUP_{stage}_SEARCH_BACKWARD", 8200,
    ))
    before = len(bridge.executor_request_publisher.messages)
    complete_active_motion(bridge, status=status)
    assert not bridge.motion_in_progress
    assert bridge.active_dwell_until is None
    bridge.navigation_command_callback(fine_alignment_message(
        f"BALL_PICKUP_{stage}_SEARCH_BACKWARD", 8201,
    ))
    assert len(bridge.executor_request_publisher.messages) == before


@pytest.mark.parametrize("direction", ["LEFT", "RIGHT"])
@pytest.mark.parametrize("approach,prepare", [
    ("GOAL_CAMERA_90_FORWARD", "goal_forward_to_crab_right_90"),
    ("GOAL_CAMERA_90_FORWARD_1", "goal_forward_to_crab_right_90"),
    ("GOAL_CAMERA_90_FORWARD_2", "goal_forward_to_crab_right_90"),
    *[(f"GOAL_CAMERA90_FINE_FORWARD_{count}", "goal_fine_to_crab_right_90")
      for count in range(1, 5)],
])
def test_goal_crab_prepares_after_completed_approach(approach, prepare, direction):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action=approach, command_id=1))
    complete_active_motion(bridge)
    action = f"GOAL_CAMERA90_CRAB_{direction}"
    crab = f"goal_camera_90_crab_{direction.lower()}"
    bridge.navigation_command_callback(navigation_message(action=action, command_id=2))
    if direction == "RIGHT":
        assert bridge.active_motion_id == prepare
        assert bridge.active_action == action
        complete_active_motion(bridge)
        assert bridge.active_motion_id == crab
        assert bridge.active_dwell_until is None
        assert not any(m["command_id"] == 2 and m["status"] == "SUCCEEDED"
                       for m in decoded_messages(bridge.motion_status_publisher))

        # A duplicate preparation completion cannot finish the crab command.
        bridge.executor_status_callback(executor_status(
            status="SUCCEEDED", command_id=2, request_id=2, motion_id=prepare,
        ))
    assert bridge.motion_in_progress and bridge.active_motion_id == crab
    complete_active_motion(bridge)
    terminal = decoded_messages(bridge.motion_status_publisher)[-1]
    assert (terminal["action"], terminal["status"]) == (action, "SUCCEEDED")

    # Consecutive crab corrections do not repeat the approach preparation.
    bridge.navigation_command_callback(navigation_message(action=action, command_id=3))
    assert bridge.active_motion_id == crab
    expected = [bridge.motion_id_for_action(approach)]
    if direction == "RIGHT":
        expected.append(prepare)
    assert [m["motion_id"] for m in decoded_messages(bridge.executor_request_publisher)] == expected + [crab, crab]


@pytest.mark.parametrize("approaches,prepare", [
    (["GOAL_CAMERA_90_FORWARD", "GOAL_CAMERA90_FINE_FORWARD_2"], "goal_fine_to_crab_right_90"),
    (["GOAL_CAMERA90_FINE_FORWARD_2", "GOAL_CAMERA_90_FORWARD"], "goal_forward_to_crab_right_90"),
])
def test_goal_crab_uses_latest_completed_gait(approaches, prepare):
    bridge = FakeBridge()
    for command_id, action in enumerate(approaches, 1):
        bridge.navigation_command_callback(navigation_message(action=action, command_id=command_id))
        complete_active_motion(bridge)
    bridge.navigation_command_callback(navigation_message(action="GOAL_CAMERA90_CRAB_RIGHT", command_id=3))
    assert bridge.active_motion_id == prepare


@pytest.mark.parametrize("approach", ["GOAL_CAMERA_90_FORWARD", "GOAL_CAMERA90_FINE_FORWARD_1"])
@pytest.mark.parametrize("status,error_code", [
    ("FAILED", "SDK_POSITION_TIMEOUT"), ("TIMEOUT", "TIMEOUT"),
    ("CANCELLED", ""), ("REJECTED", "MOTION_NOT_FOUND"),
    *[("FAILED", code) for code in sorted(RECOVERABLE_MOTOR_ERROR_CODES)],
])
def test_goal_crab_preparation_failure_never_starts_crab(approach, status, error_code):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action=approach, command_id=1))
    complete_active_motion(bridge)
    bridge.navigation_command_callback(navigation_message(action="GOAL_CAMERA90_CRAB_RIGHT", command_id=2))
    complete_active_motion(bridge, status, error_code)
    assert not bridge.motion_in_progress
    assert "goal_camera_90_crab_right" not in [
        m["motion_id"] for m in decoded_messages(bridge.executor_request_publisher)
    ]
    terminal = decoded_messages(bridge.motion_status_publisher)[-1]
    assert terminal["status"] == status
    assert terminal["error_code"] == error_code


def test_goal_crab_waits_for_completed_approach_and_locks_preparation():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="GOAL_CAMERA_90_FORWARD", command_id=1))
    bridge.navigation_command_callback(navigation_message(action="GOAL_CAMERA90_CRAB_RIGHT", command_id=2))
    assert decoded_messages(bridge.motion_status_publisher)[-1]["error_code"] == "REJECTED_BUSY"
    assert len(bridge.executor_request_publisher.messages) == 1
    complete_active_motion(bridge)
    bridge.navigation_command_callback(navigation_message(action="GOAL_CAMERA90_CRAB_RIGHT", command_id=3))
    bridge.navigation_command_callback(navigation_message(action="STRAIGHT", command_id=4))
    assert decoded_messages(bridge.motion_status_publisher)[-1]["error_code"] == "ATOMIC_SEQUENCE_LOCKED"
    assert bridge.active_motion_id == "goal_forward_to_crab_right_90"
    assert bridge.queued_request_id is None


@pytest.mark.parametrize("intervening", ["GOAL_CAMERA90_TURN_LEFT_1", "GOAL_CAMERA90_BACKWARD_1"])
@pytest.mark.parametrize("direction", ["LEFT", "RIGHT"])
@pytest.mark.parametrize("approach", ["GOAL_CAMERA_90_FORWARD", "GOAL_CAMERA90_FINE_FORWARD_1"])
def test_goal_crab_does_not_prepare_from_old_forward_history(intervening, direction, approach):
    bridge = FakeBridge()
    for command_id, action in enumerate([approach, intervening], 1):
        bridge.navigation_command_callback(navigation_message(action=action, command_id=command_id))
        complete_active_motion(bridge)
    bridge.navigation_command_callback(navigation_message(action=f"GOAL_CAMERA90_CRAB_{direction}", command_id=3))
    assert bridge.active_motion_id == f"goal_camera_90_crab_{direction.lower()}"


@pytest.mark.parametrize("direction", ["LEFT", "RIGHT"])
def test_pickup_fine_to_crab_preserves_left_and_prepares_right_once(direction):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    before = len(bridge.executor_request_publisher.messages)
    action = f"BALL_PICKUP_CRAB_{direction}"
    crab = f"pickup_crab_{direction.lower()}_0"
    prepare = "pickup_fine_to_crab_right_0"
    bridge.navigation_command_callback(fine_alignment_message(action))
    if direction == "RIGHT":
        assert bridge.active_motion_id == prepare
        bridge.executor_status_callback(executor_status(
            status="SUCCEEDED", motion_id="pickup_fine_forward_0",
        ))
        assert bridge.active_motion_id == prepare
        bridge.navigation_command_callback(navigation_message(action="SHOT", command_id=9000))
        assert decoded_messages(bridge.motion_status_publisher)[-1]["error_code"] == "ATOMIC_SEQUENCE_LOCKED"
        complete_active_motion(bridge)
        assert bridge.active_dwell_until is None
        bridge.executor_status_callback(executor_status(status="SUCCEEDED", motion_id=prepare))
    assert bridge.active_motion_id == crab
    complete_active_motion(bridge)
    assert_correction_dwell_blocks_then_finishes(bridge)
    assert bridge.active_dwell_until is None
    assert bridge.pickup_fine_align_waiting
    bridge.navigation_command_callback(fine_alignment_message(action, 9001))
    assert bridge.active_motion_id == crab
    requests = decoded_messages(bridge.executor_request_publisher)[before:]
    assert [r["motion_id"] for r in requests] == (
        [prepare, crab, crab] if direction == "RIGHT" else [crab, crab]
    )
    assert all((r["command_id"], r["event_id"]) == (8000, 8) for r in requests)


@pytest.mark.parametrize("status,error_code", [
    ("FAILED", "MOTION_FAILED"), ("TIMEOUT", "TIMEOUT"),
    ("CANCELLED", ""), ("REJECTED", "INVALID_MOTION"),
    *[("FAILED", code) for code in sorted(RECOVERABLE_MOTOR_ERROR_CODES)],
])
def test_pickup_fine_right_crab_preparation_failure_stops(status, error_code):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    bridge.navigation_command_callback(fine_alignment_message("BALL_PICKUP_CRAB_RIGHT"))
    assert bridge.active_motion_id == "pickup_fine_to_crab_right_0"
    complete_active_motion(bridge, status, error_code)
    assert not bridge.motion_in_progress
    assert bridge.pending_pickup_crab_motion_id is None
    assert "pickup_crab_right_0" not in [
        r["motion_id"] for r in decoded_messages(bridge.executor_request_publisher)
    ]
    assert decoded_messages(bridge.motion_status_publisher)[-1]["status"] == status


@pytest.mark.parametrize("intervening", ["BALL_PICKUP_FINE_SEARCH_LEFT", "BALL_PICKUP_FINE_SEARCH_BACKWARD"])
def test_pickup_right_crab_does_not_prepare_from_old_fine_history(intervening):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    bridge.navigation_command_callback(fine_alignment_message(intervening))
    complete_active_motion(bridge)
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    bridge.navigation_command_callback(fine_alignment_message("BALL_PICKUP_CRAB_RIGHT", 8002))
    assert bridge.active_motion_id == "pickup_crab_right_0"


@pytest.mark.parametrize("source", ["ball", "hurdle"])
@pytest.mark.parametrize("distance,action,motion_name", [
    (0.549, "STRAIGHT_0", "찐미세45도-4"),
    (0.550, "STRAIGHT_0", "찐미세45도-4"),
    (0.550001, "STRAIGHT", "찐찐전진45(4회)"),
    (0.570, "STRAIGHT", "찐찐전진45(4회)"),
    (0.700, "STRAIGHT", "찐찐전진45(4회)"),
    (0.700001, "STRAIGHT", "찐찐전진45(4회)"),
])
def test_ball_hurdle_distance_policy_reaches_runtime_catalog(
    source, distance, action, motion_name,
):
    """Exercise normal approach and the default close-Ball pickup checkpoint."""
    from pathlib import Path
    import yaml
    from test_motion_decision_planner import line_info

    planner = MotionDecisionPlanner()
    sample = {
        "detected": True, "confirmation_confirmed": True,
        "confidence": 0.9, "depth_valid": True, "depth_age_sec": 0.05,
        "distance_m": distance, "depth_m": distance,
        "bearing_deg": 0.0, "steering_angle_deg": 0.0,
        "offset_x_px": 0, "offset_x_norm": 0.0,
        "bottom_distance_px": 200, "hurdle_angle_deg": 0.0,
        "go_now": False, "pickup_ready": False, "pickup_now": False,
    }
    sample.update(bbox=[400, 300, 1000, 500], image_width=1280, image_height=720)
    route = {**line_info(), "robot_center_x_px": 710,
             "center_points_px": [[710, 650], [710, 580], [710, 250]]}
    decision = planner.plan("AUTO", {source: sample, "line": route}, 0.1)
    if source == "hurdle":
        assert decision.source == "hurdle"
        motion_name = "찐미세0도-4" if distance <= 0.700 else "찐찐전진45(4회)"
        action = "STRAIGHT_0" if distance <= 0.700 else action
    bridge = FakeBridge()
    if source == "ball" and distance <= 0.550:
        assert decision.action == "PICKUP_NOW"
        bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW"))
        assert_pickup_initial_alignment_started(bridge)
        decision = planner.plan_ball_pickup_initial_alignment(sample)
        assert decision.valid
        assert decision.source_command["pickup_approach_motion"] == action
        continue_pickup_after_initial_alignment(bridge, approach_motion=action)
    else:
        assert decision.valid
        assert decision.action == action
        bridge.navigation_command_callback(navigation_message(
            source=decision.source, action=decision.action,
            source_command=decision.source_command,
        ))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / "src/irc_step_motion_executor/config/motion_aliases.yaml").read_text())["motion_aliases"]
    assert aliases[request["motion_id"]] == motion_name
    runtime = json.loads((root / "artifacts/robot_motions_runtime.json").read_text())
    assert motion_name in {motion["name"] for motion in runtime["motions"]}


@pytest.mark.parametrize("error,count,turn_angle", [
    (8.001, 1, 15), (12, 1, 15), (15, 1, 15), (29.999, 1, 15),
    (30, 2, 30), (45, 3, 45), (60, 4, 60), (75, 5, 75),
    (90, 6, 90), (120, 6, 90),
    (-8.001, 2, 15), (-12, 2, 15), (-15, 2, 15), (-29.999, 2, 15),
    (-30, 3, 30), (-44.999, 3, 30), (-45, 5, 45),
    (-64.999, 5, 45), (-65, 7, 65), (-94.999, 7, 65),
    (-95, 9, 95), (-120, 9, 95),
])
def test_hurdle_fine_center_error_never_dispatches_turn(error, count, turn_angle):
    from pathlib import Path
    import yaml

    decision = MotionDecisionPlanner().plan("AUTO", {"hurdle": {
        "detected": True, "confirmation_confirmed": True,
        "confidence": 0.9, "depth_valid": True, "depth_m": 0.55,
        "hurdle_angle_deg": 0, "bearing_deg": -error, "bottom_distance_px": 200,
    }}, 0.1)
    assert decision.valid and decision.source == "hurdle"
    assert decision.action == "STRAIGHT_0"
    assert "turn_count" not in decision.source_command
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        source=decision.source, action=decision.action,
        source_command=decision.source_command,
    ))
    assert decoded_messages(bridge.executor_request_publisher)[-1]["motion_id"] == "pickup_fine_forward_0"


@pytest.mark.parametrize("error,bottom,depth,expected", [
    (8, 200, 0.55, "STRAIGHT_0"), (-8, 200, 0.55, "STRAIGHT_0"),
    (0, 200, 0.55, "STRAIGHT_0"), (0, 200, 0.20, "GO"),
    (40, 100, 0.4, "GO"), (-40, 100, 0.4, "GO"),
])
def test_hurdle_turn_sizing_preserves_parallel_and_close_behavior(
    error, bottom, depth, expected,
):
    decision = MotionDecisionPlanner().plan("AUTO", {"hurdle": {
        "detected": True, "confirmation_confirmed": True, "confidence": 0.9,
        "depth_valid": True, "depth_m": depth,
        "hurdle_angle_deg": error, "bottom_distance_px": bottom,
        "camera_center_offset_x_px": 0,
    }}, 0.1)
    assert decision.valid and decision.action == expected
    assert "turn_count" not in decision.source_command


@pytest.mark.parametrize("action,count", [
    ("ALIGN_LEFT", None), ("ALIGN_LEFT", True), ("ALIGN_LEFT", 1.0),
    ("ALIGN_LEFT", "1"), ("ALIGN_LEFT", 0), ("ALIGN_LEFT", 7),
    ("ALIGN_RIGHT", None), ("ALIGN_RIGHT", 1), ("ALIGN_RIGHT", 4),
    ("ALIGN_RIGHT", 10),
])
def test_hurdle_alignment_without_supported_count_never_uses_fixed_turn(action, count):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        source="hurdle", action=action, source_command={"turn_count": count},
    ))
    assert not bridge.executor_request_publisher.messages
    status = decoded_messages(bridge.motion_status_publisher)[-1]
    assert status["status"] == "UNSUPPORTED"


@pytest.mark.parametrize("source", ["line", "goal", None])
def test_object_approach_mapping_does_not_change_other_sources(source):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(source=source, action="STRAIGHT"))
    assert decoded_messages(bridge.executor_request_publisher)[-1]["motion_id"] == "line_forward_6"
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(source=source, action="STRAIGHT_0"))
    assert bridge.executor_request_publisher.messages == []


def hurdle_depth_fallback_message(command_id=8000):
    planner = MotionDecisionPlanner()
    decision = planner.plan("HURDLE_POSITIONING", {"hurdle": {
        "detected": True, "confirmation_confirmed": True,
        "confidence": 0.9, "bottom_distance_px": 100,
        "depth_valid": False, "depth_m": None, "hurdle_angle_deg": None,
    }}, 0.1)
    assert decision.action == "WAIT" and not decision.valid
    return navigation_message(
        source="hurdle", action=decision.action, command_id=command_id,
        source_command=decision.source_command, valid=decision.valid,
    )


def test_hurdle_depth_loss_does_not_dispatch_any_motion():
    bridge = FakeBridge()
    bridge.navigation_command_callback(hurdle_depth_fallback_message())
    assert not bridge.executor_request_publisher.messages
    assert not bridge.motion_in_progress



@pytest.mark.parametrize("status,error_code", [
    ("FAILED", "SDK_COMMUNICATION_ERROR"), ("TIMEOUT", "TIMEOUT"),
    ("CANCELLED", ""), ("REJECTED", "INVALID_MOTION"),
])
def test_failed_hurdle_fine_step_never_starts_hurdle(status, error_code):
    bridge = FakeBridge()
    bridge.navigation_command_callback(hurdle_fine_sequence_message())
    complete_active_motion(bridge, status, error_code)
    bridge._check_atomic_dwell(1e12)
    assert not bridge.motion_in_progress
    assert not bridge.hurdle_depth_fine_completed
    assert [r["motion_id"] for r in decoded_messages(bridge.executor_request_publisher)] == ["pickup_fine_forward_0"]


def test_missing_hurdle_motion_is_not_success_and_retry_does_not_repeat_fine():
    bridge = FakeBridge()
    bridge.navigation_command_callback(hurdle_fine_sequence_message())
    for _ in range(2):
        complete_active_motion(bridge)
        bridge._check_atomic_dwell(bridge.active_dwell_until)
    complete_active_motion(bridge, "REJECTED", "INVALID_MOTION")
    assert decoded_messages(bridge.motion_status_publisher)[-1]["status"] == "REJECTED"
    bridge.navigation_command_callback(hurdle_fine_sequence_message(command_id=8001))
    assert [r["motion_id"] for r in decoded_messages(bridge.executor_request_publisher)] == [
        "pickup_fine_forward_0", "pickup_fine_forward_0", "hurdle",
    ]
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert [r["motion_id"] for r in decoded_messages(bridge.executor_request_publisher)] == [
        "pickup_fine_forward_0", "pickup_fine_forward_0", "hurdle", "hurdle",
    ]


def test_hurdle_final_sequence_waits_until_current_motion_finishes():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="STRAIGHT", command_id=7999))
    bridge.navigation_command_callback(hurdle_fine_sequence_message())
    assert [r["motion_id"] for r in decoded_messages(bridge.executor_request_publisher)] == ["line_forward_6"]
    assert decoded_messages(bridge.motion_status_publisher)[-1]["error_code"] == "REJECTED_BUSY"


@pytest.mark.parametrize("depth,motion_id", [
    (0.550, "pickup_fine_forward_0"),
    (0.550001, "pickup_fine_forward_0"), (0.7, "pickup_fine_forward_0"),
    (0.700001, "pickup_fine_forward_0"),
])
@pytest.mark.parametrize("angle", [None, 20.0])
def test_close_hurdle_valid_depth_selects_actual_motion_without_turning(depth, motion_id, angle):
    decision = MotionDecisionPlanner().plan("HURDLE_POSITIONING", {"hurdle": {
        "detected": True, "confirmation_confirmed": True,
        "confidence": 0.9, "bottom_distance_px": 100,
        "depth_valid": True, "depth_m": depth, "hurdle_angle_deg": angle,
        "camera_center_offset_x_px": 0,
    }}, 0.1)
    assert decision.valid
    assert decision.requires_ack is False
    assert decision.source_command["depth_fallback_requested"] is False
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        source=decision.source, action=decision.action,
        source_command=decision.source_command,
    ))
    assert [request["motion_id"] for request in decoded_messages(bridge.executor_request_publisher)] == [motion_id]


@pytest.mark.parametrize("source", [None, "hurdle"])
def test_every_normal_go_holds_one_second_before_hurdle(monkeypatch, source):
    clock = [10.0]
    monkeypatch.setattr("mission_control.motion_command_bridge_node.time.monotonic", lambda: clock[0])
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="GO", source=source))
    for _ in range(2):
        assert bridge.active_motion_id == "pickup_fine_forward_0"
        complete_active_motion(bridge)
        if bridge.active_motion_id == bridge.DWELL_MARKER:
            clock[0] = bridge.active_dwell_until
            bridge._check_atomic_dwell(clock[0])
    assert bridge.active_dwell_until == clock[0] + 1.0
    assert bridge.active_motion_id == bridge.HURDLE_PRE_GO_DWELL_MARKER
    assert len(bridge.executor_request_publisher.messages) == 2
    status = decoded_messages(bridge.motion_status_publisher)[-1]
    assert status["status"] == "RUNNING" and status["action"] == "GO"
    bridge.navigation_command_callback(navigation_message(action="STRAIGHT", command_id=8001))
    assert decoded_messages(bridge.motion_status_publisher)[-1]["error_code"] == "ATOMIC_SEQUENCE_LOCKED"
    bridge._check_atomic_dwell(bridge.active_dwell_until - 0.001)
    assert len(bridge.executor_request_publisher.messages) == 2
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    bridge._check_atomic_dwell(14.0)
    requests = decoded_messages(bridge.executor_request_publisher)
    assert [r["motion_id"] for r in requests] == ["pickup_fine_forward_0", "pickup_fine_forward_0", "hurdle"]
    assert bridge.motion_in_progress
    assert not any(s["status"] == "SUCCEEDED" for s in decoded_messages(bridge.motion_status_publisher))
    complete_active_motion(bridge)
    assert not bridge.motion_in_progress
    assert decoded_messages(bridge.motion_status_publisher)[-1]["status"] == "SUCCEEDED"



@pytest.mark.parametrize("fallback", [False, True])
def test_each_go_retry_has_a_new_full_one_second_hold(monkeypatch, fallback):
    clock = [10.0]
    monkeypatch.setattr("mission_control.motion_command_bridge_node.time.monotonic", lambda: clock[0])
    bridge = FakeBridge()
    def request(command_id):
        return (hurdle_fine_sequence_message(command_id) if fallback else
                navigation_message(action="GO", source="hurdle", command_id=command_id))
    bridge.navigation_command_callback(request(8000))
    for _ in range(2):
        complete_active_motion(bridge)
        if bridge.active_motion_id == bridge.DWELL_MARKER:
            clock[0] = bridge.active_dwell_until
            bridge._check_atomic_dwell(clock[0])
    clock[0] = bridge.active_dwell_until
    bridge._check_atomic_dwell(clock[0])
    complete_active_motion(bridge, "REJECTED", "INVALID_MOTION")
    clock[0] = 30.0
    bridge.navigation_command_callback(request(8001))
    assert bridge.active_dwell_until == 31.0
    before = len(bridge.executor_request_publisher.messages)
    bridge._check_atomic_dwell(30.999)
    assert len(bridge.executor_request_publisher.messages) == before
    bridge._check_atomic_dwell(31.0)
    motions = [r["motion_id"] for r in decoded_messages(bridge.executor_request_publisher)]
    assert motions == ["pickup_fine_forward_0", "pickup_fine_forward_0"] + ["hurdle", "hurdle"]



def hurdle_fine_sequence_message(command_id=8100, depth=0.20):
    decision = MotionDecisionPlanner().plan("HURDLE_POSITIONING", {"hurdle": {
        "detected": True, "confirmation_confirmed": True, "confidence": 0.9,
        "depth_valid": True, "depth_m": depth, "hurdle_angle_deg": None,
        "bottom_distance_px": 200, "camera_center_offset_x_px": 0, "go_now": False,
    }}, 0.1)
    assert decision.valid and decision.action == "GO" and decision.requires_ack
    return navigation_message(
        source="hurdle", action="GO", command_id=command_id,
        source_command=decision.source_command,
    )


@pytest.mark.parametrize("depth", [0.20, 0.43, 0.54])
def test_hurdle_sequence_runs_two_fine_motions_then_one_second_then_hurdle(monkeypatch, depth):
    clock = [10.0]
    monkeypatch.setattr("mission_control.motion_command_bridge_node.time.monotonic", lambda: clock[0])
    bridge = FakeBridge()
    bridge.navigation_command_callback(hurdle_fine_sequence_message(depth=depth))
    first_request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert bridge.active_motion_id == "pickup_fine_forward_0"
    assert bridge.active_dwell_until is None
    clock[0] = 20.0
    complete_active_motion(bridge)
    assert bridge.active_dwell_until == 21.0
    assert_correction_dwell_blocks_then_finishes(bridge)
    second_request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert second_request["motion_id"] == "pickup_fine_forward_0"
    assert first_request["request_id"] != second_request["request_id"]
    assert bridge.hurdle_sequence_fine_completed == 1
    assert bridge.active_dwell_until is None
    # A delayed completion from the first repetition cannot finish the second.
    bridge.executor_status_callback(executor_status(
        status="SUCCEEDED", request_id=first_request["request_id"],
        motion_id="pickup_fine_forward_0",
    ))
    assert bridge.hurdle_sequence_fine_completed == 1
    bridge.navigation_command_callback(navigation_message(action="STRAIGHT", command_id=8101))
    assert decoded_messages(bridge.motion_status_publisher)[-1]["error_code"] == "ATOMIC_SEQUENCE_LOCKED"
    clock[0] = 30.0
    complete_active_motion(bridge)
    assert bridge.hurdle_sequence_fine_completed == 2
    assert bridge.active_dwell_until == 31.0
    bridge._check_atomic_dwell(30.999)
    assert len(bridge.executor_request_publisher.messages) == 2
    bridge._check_atomic_dwell(31.0)
    assert [r["motion_id"] for r in decoded_messages(bridge.executor_request_publisher)] == [
        "pickup_fine_forward_0", "pickup_fine_forward_0", "hurdle",
    ]
    assert not any(s["status"] == "SUCCEEDED" for s in decoded_messages(bridge.motion_status_publisher))
    complete_active_motion(bridge)
    assert not bridge.motion_in_progress
    assert bridge.hurdle_sequence_fine_completed == 0
    assert decoded_messages(bridge.motion_status_publisher)[-1]["status"] == "SUCCEEDED"



@pytest.mark.parametrize("completed_fine", [0, 1, 2])
@pytest.mark.parametrize("status,error_code", [
    ("FAILED", "SDK_COMMUNICATION_ERROR"), ("TIMEOUT", "TIMEOUT"),
    ("CANCELLED", ""), ("REJECTED", "INVALID_MOTION"),
])
def test_hurdle_sequence_failure_stops_and_retry_skips_completed_fine(completed_fine, status, error_code):
    bridge = FakeBridge()
    bridge.navigation_command_callback(hurdle_fine_sequence_message())
    for _ in range(completed_fine):
        complete_active_motion(bridge)
        if bridge.active_motion_id == bridge.DWELL_MARKER:
            assert_correction_dwell_blocks_then_finishes(bridge)
    if completed_fine == 2:
        bridge._check_atomic_dwell(bridge.active_dwell_until)
    complete_active_motion(bridge, status, error_code)
    assert not bridge.motion_in_progress
    assert bridge.hurdle_sequence_fine_completed == completed_fine
    before = len(bridge.executor_request_publisher.messages)
    bridge._check_atomic_dwell(1e12)
    assert len(bridge.executor_request_publisher.messages) == before
    bridge.navigation_command_callback(hurdle_fine_sequence_message(8101))
    for _ in range(2 - completed_fine):
        complete_active_motion(bridge)
        if bridge.active_motion_id == bridge.DWELL_MARKER:
            assert_correction_dwell_blocks_then_finishes(bridge)
    assert bridge.active_motion_id == bridge.HURDLE_PRE_GO_DWELL_MARKER
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    retry_motions = [r["motion_id"] for r in decoded_messages(bridge.executor_request_publisher)[before:]]
    assert retry_motions == ["pickup_fine_forward_0"] * (2 - completed_fine) + ["hurdle"]



def test_hurdle_retry_waits_for_valid_depth_without_replaying_completed_steps():
    bridge = FakeBridge()
    bridge.navigation_command_callback(hurdle_fine_sequence_message())
    complete_active_motion(bridge)
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    complete_active_motion(bridge, "FAILED", "SDK_COMMUNICATION_ERROR")
    assert bridge.hurdle_sequence_fine_completed == 1
    before = len(bridge.executor_request_publisher.messages)
    bridge.navigation_command_callback(hurdle_depth_fallback_message(command_id=8101))
    assert len(bridge.executor_request_publisher.messages) == before
    bridge.navigation_command_callback(hurdle_fine_sequence_message(command_id=8102))
    assert bridge.active_motion_id == "pickup_fine_forward_0"
    complete_active_motion(bridge)
    assert bridge.hurdle_sequence_fine_completed == 2
    assert bridge.active_motion_id == bridge.HURDLE_PRE_GO_DWELL_MARKER
