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
    DWELL_MARKER = MotionCommandBridgeNode.DWELL_MARKER
    SHOT_PREPARE_DWELL_MARKER = MotionCommandBridgeNode.SHOT_PREPARE_DWELL_MARKER
    SHOT_PREPARE_MOTION_ID = MotionCommandBridgeNode.SHOT_PREPARE_MOTION_ID
    SHOT_PREPARE_DWELL_SEC = MotionCommandBridgeNode.SHOT_PREPARE_DWELL_SEC
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
    PICKUP_FINE_PREPARE_MOTION_ID = MotionCommandBridgeNode.PICKUP_FINE_PREPARE_MOTION_ID
    PICKUP_CRAB_PREPARE_MOTION_ID = MotionCommandBridgeNode.PICKUP_CRAB_PREPARE_MOTION_ID
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
        self.goal_crab_completed = False
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
    """Return a correlated terminal status for the current executor motion."""
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
    """Advance the initial stationary dwell and confirm heading alignment."""
    assert bridge.pickup_initial_align_dwell_until is not None
    assert bridge.pickup_initial_align_waiting is False
    bridge._check_atomic_dwell(bridge.pickup_initial_align_dwell_until)
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
    assert bridge.active_motion_id == "pickup_fine_forward_0"
    assert bridge.active_dwell_until is None


def complete_pickup_motion_and_dwell(bridge, motion_id):
    """Complete one pickup motion and advance its three-second dwell."""
    if motion_id == "pickup_fine_forward_0" and (
        bridge.active_motion_id == bridge.PICKUP_FINE_PREPARE_MOTION_ID
    ):
        complete_pickup_fine_preparation(bridge)
    bridge.executor_status_callback(
        executor_status(status="SUCCEEDED", motion_id=motion_id)
    )
    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    if bridge.pickup_initial_align_waiting:
        continue_pickup_after_initial_alignment(
            bridge,
            approach_motion="STRAIGHT_0",
            command_id=bridge.last_sent_command_id + 1,
        )


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
    "STRAIGHT": "line_forward_6",
    "STRAIGHT_1": "line_forward_2",
    "STRAIGHT_2": "line_forward_4",
    "STRAIGHT_3": "line_forward_6",
    "STRAIGHT_4": "line_forward_8",
    "STRAIGHT_5": "line_forward_10",
    "APPROACH": "forward",
    "LEFT": "line_turn_left_15",
    "RIGHT": "line_turn_right_large",
    "POST_BALL_GOAL_TRANSITION": "post_ball_camera_90",
    "POST_SHOT_TURN_RIGHT_9": "post_shot_default_turn_right",
    "POST_SHOT_TURN_LEFT_4": "post_shot_default_turn_left",
    "POST_SHOT_FORWARD": "line_forward_6",
    **{
        f"POST_SHOT_LINE_TURN_{direction}_{count}": (
            f"post_ball_line_turn_{direction.lower()}_{count}"
        )
        for direction, counts in (
            ("RIGHT", (2, 3, 5, 7, 9)), ("LEFT", (1, 2, 3, 4, 5)),
        )
        for count in counts
    },
    "BALL_FINE_FORWARD_8": "ball_general_fine_forward_8",
    "BALL_LOST_FORWARD_2": "ball_camera_down_forward_2",
    "LINE_LOST_TURN_LEFT": "line_search_left_1",
    "LINE_LOST_TURN_RIGHT": "line_search_right_3",
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
for count in (1, 2, 3, 4, 5, 6):
    EXPECTED_PRODUCTION_ACTIONS[
        f"BALL_APPROACH_TURN_LEFT_{count}"
    ] = f"post_ball_line_turn_left_{count}"
for recovery_side in ("LEFT", "RIGHT"):
    for direction, counts in (
        ("LEFT", (2, 3, 4, 5, 6, 7, 8, 10, 13)),
        ("RIGHT", (2, 3, 4, 5, 6, 7, 8, 10, 12, 15)),
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
    assert bridge.active_motion_id == (
        bridge.PICKUP_INITIAL_ALIGN_DWELL_MARKER
    )
    assert bridge.pickup_initial_align_dwell_until is not None
    assert bridge.pickup_initial_align_waiting is False


def test_first_pickup_finishes_after_backward_turn_and_dwell_without_pose_transition():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)

    for completed_motion in (
        "ball_camera_down_forward_4",
        "pickup_fine_forward_0",
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
        "pickup_fine_forward_0",
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
        if status.get("completed_motion_id")
        == "pickup_grasp_check_pose"
    ]
    assert [
        status["verification_window_complete"]
        for status in verification_statuses
    ] == [False, True]
    intermediate_statuses = decoded_messages(bridge.motion_status_publisher)
    assert intermediate_statuses[-1]["status"] == "RUNNING"
    assert intermediate_statuses[-1]["motion_id"] == bridge.DWELL_MARKER
    assert bridge.motion_in_progress is True

    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert [
        request["motion_id"]
        for request in decoded_messages(bridge.executor_request_publisher)
    ] == request_motion_ids

    statuses = decoded_messages(bridge.motion_status_publisher)
    assert statuses[-1]["status"] == "SUCCEEDED"
    assert statuses[-1]["action"] == "PICKUP_NOW"
    assert bridge.motion_in_progress is False


def test_pickup_dwell_is_non_blocking_and_does_not_start_early():
    bridge = FakeBridge()
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)

    for completed_motion in (
        "ball_camera_down_forward_4",
        "pickup_fine_forward_0",
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
    bridge._check_atomic_dwell(bridge.active_dwell_until - 0.001)

    assert len(bridge.executor_request_publisher.messages) == request_count
    assert bridge.motion_in_progress is True


def test_consecutive_pickup_motions_have_one_exact_three_second_dwell(
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

    now[0] = 110.0
    bridge.executor_status_callback(
        executor_status(
            status="SUCCEEDED",
            motion_id="ball_camera_down_forward_4",
        )
    )
    assert bridge.active_dwell_until == pytest.approx(113.0)
    request_count = len(bridge.executor_request_publisher.messages)

    bridge._check_atomic_dwell(112.999)
    assert len(bridge.executor_request_publisher.messages) == request_count
    assert bridge.active_dwell_until == pytest.approx(113.0)

    bridge._check_atomic_dwell(113.0)
    assert bridge.active_dwell_until is None
    requests = decoded_messages(bridge.executor_request_publisher)
    assert len(requests) == request_count
    assert bridge.pickup_initial_align_waiting is True


def test_pickup_positioning_loss_finishes_motion_then_reacquires_after_dwell(
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
    assert bridge.active_dwell_until == pytest.approx(113.0)
    assert len(bridge.executor_request_publisher.messages) == request_count
    bridge._check_atomic_dwell(112.999)
    assert len(bridge.executor_request_publisher.messages) == request_count

    bridge._check_atomic_dwell(113.0)
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
    dwell_until = bridge.active_dwell_until
    assert dwell_until is not None
    bridge._check_atomic_dwell(dwell_until)
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


def test_pickup_entry_holds_still_three_seconds_before_initial_alignment():
    bridge = FakeBridge()
    assert bridge.PICKUP_DWELL_SEC == 3.0
    bridge.navigation_command_callback(
        navigation_message(action="PICKUP_NOW")
    )

    assert bridge.motion_in_progress is True
    assert bridge.active_dwell_until is None
    assert bridge.pickup_initial_align_dwell_until is not None
    assert bridge.pickup_initial_align_waiting is False
    assert bridge.executor_request_publisher.messages == []
    status = decoded_messages(bridge.motion_status_publisher)[-1]
    assert status["status"] == "RUNNING"
    assert status["motion_id"] == bridge.PICKUP_INITIAL_ALIGN_DWELL_MARKER

    dwell_until = bridge.pickup_initial_align_dwell_until
    bridge._check_atomic_dwell(dwell_until - 0.001)
    assert bridge.pickup_initial_align_waiting is False

    bridge._check_atomic_dwell(dwell_until)
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
        "pickup_fine_prepare"
    ]
    assert bridge.active_pickup_sequence.count(
        "pickup_fine_forward_0"
    ) == 1
    assert bridge.active_pickup_sequence[:11] == (
        "pickup_fine_forward_0",
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

    complete_pickup_motion_and_dwell(bridge, "pickup_fine_forward_0")
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
    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until)
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


def test_pickup_forward_turn_forward_each_use_one_dwell_and_fresh_check():
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
    first_dwell = bridge.active_dwell_until
    assert first_dwell is not None
    bridge._check_atomic_dwell(first_dwell)
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
            motion_id="pickup_fine_forward_0",
            error_code="MOTION_FAILED",
        )
    )

    requests = decoded_messages(bridge.executor_request_publisher)
    assert [request["motion_id"] for request in requests] == [
        "ball_camera_down_forward_4",
        "pickup_fine_forward_0",
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
            motion_id="pickup_fine_forward_0",
            error_code=error_code,
        )
    )

    assert bridge.motion_in_progress is True
    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert bridge.pickup_fine_align_waiting is True


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
        "pickup_fine_forward_0",
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
    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    statuses = decoded_messages(bridge.motion_status_publisher)
    assert statuses[-1]["status"] == "SUCCEEDED"
    assert bridge.motion_in_progress is False


@pytest.mark.parametrize(
    ("approach_motion", "motion_id"),
    [
        ("STRAIGHT_0", "pickup_fine_forward_0"),
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

    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until)

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
        "pickup_fine_forward_0",
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
def test_forward4_then_crab_prepares_once_and_preserves_checkpoint(direction):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW"))
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)
    complete_active_motion(bridge)
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert bridge.pickup_initial_align_waiting

    action = f"BALL_PICKUP_INITIAL_CRAB_{direction}"
    crab = f"pickup_crab_{direction.lower()}_0"
    bridge.navigation_command_callback(fine_alignment_message(action, 8002))
    assert bridge.active_motion_id == "pickup_crab_prepare"
    # A stale forward completion cannot skip preparation.
    bridge.executor_status_callback(executor_status(
        status="SUCCEEDED", motion_id="ball_camera_down_forward_4",
    ))
    assert bridge.active_motion_id == "pickup_crab_prepare"
    complete_active_motion(bridge)
    assert bridge.active_motion_id == crab
    assert bridge.active_dwell_until is None
    bridge.executor_status_callback(executor_status(
        status="SUCCEEDED", motion_id="pickup_crab_prepare",
    ))
    complete_active_motion(bridge)
    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert bridge.pickup_initial_align_waiting
    bridge.navigation_command_callback(fine_alignment_message(action, 8003))
    assert bridge.active_motion_id == crab
    requests = decoded_messages(bridge.executor_request_publisher)
    assert [r["motion_id"] for r in requests] == [
        "ball_camera_down_forward_4", "pickup_crab_prepare", crab, crab,
    ]


@pytest.mark.parametrize("status,error_code", [
    ("FAILED", "MOTION_FAILED"), ("TIMEOUT", "TIMEOUT"),
    ("CANCELLED", ""), ("REJECTED", "INVALID_MOTION"),
    *[("FAILED", code) for code in sorted(RECOVERABLE_MOTOR_ERROR_CODES)],
])
def test_crab_preparation_failure_stops_without_crab(status, error_code):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW"))
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)
    complete_active_motion(bridge)
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    bridge.navigation_command_callback(fine_alignment_message(
        "BALL_PICKUP_INITIAL_CRAB_LEFT", 8002,
    ))
    assert bridge.active_motion_id == "pickup_crab_prepare"
    complete_active_motion(bridge, status, error_code)
    assert not bridge.motion_in_progress
    assert bridge.pending_pickup_crab_motion_id is None
    assert bridge.last_pickup_motion_id is None
    assert [r["motion_id"] for r in decoded_messages(
        bridge.executor_request_publisher
    )] == ["ball_camera_down_forward_4", "pickup_crab_prepare"]
    assert decoded_messages(bridge.motion_status_publisher)[-1]["status"] == status


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
    assert [r["motion_id"] for r in requests] == ["pickup_fine_forward_0"]
    assert bridge.pickup_fine_align_waiting is False
    complete_pickup_motion_and_dwell(bridge, "pickup_fine_forward_0")
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
        "pickup_fine_forward_0",
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
        "pickup_fine_forward_0",
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

    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert bridge.pickup_initial_align_waiting is False
    assert bridge.pickup_fine_align_waiting is True
    assert decoded_messages(bridge.motion_status_publisher)[-1][
        "motion_id"
    ] == bridge.FINE_ALIGN_MARKER


def test_pickup_distance_decisions_repeat_fine_then_crab_then_backward():
    bridge = FakeBridge()
    planner = MotionDecisionPlanner()
    enter_pickup_fine_alignment(bridge)
    sample = {
        "detected": True,
        "confidence": 0.9,
        "offset_x_px": 56,
    }

    for command_id, bottom_distance_px in enumerate((450, 400, 301), 8100):
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
        "bottom_distance_px": 300,
    })
    assert decision.action == "BALL_PICKUP_CRAB_RIGHT"
    bridge.navigation_command_callback(
        fine_alignment_message(decision.action, command_id=8103)
    )
    complete_pickup_motion_and_dwell(bridge, "pickup_crab_right_0")
    assert bridge.pickup_fine_align_waiting is True
    assert bridge.pickup_fixed_sequence_started is False

    decision = planner.plan_ball_pickup_fine_alignment({
        **sample,
        "bottom_distance_px": 300,
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
    crab_request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert crab_request["motion_id"] == "pickup_crab_right_0"
    assert crab_request["action"] == "PICKUP_NOW"
    assert crab_request["command_id"] == 8000
    assert crab_request["event_id"] == 8

    bridge.executor_status_callback(
        executor_status(status="SUCCEEDED", motion_id="pickup_crab_right_0")
    )

    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until)
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
    assert bridge.active_dwell_until is not None
    bridge._check_atomic_dwell(bridge.active_dwell_until)
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
        bridge.executor_status_callback(
            executor_status(
                status="SUCCEEDED",
                motion_id="pickup_crab_right_0",
            )
        )
        assert bridge.active_dwell_until is not None
        bridge._check_atomic_dwell(bridge.active_dwell_until)
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
def test_post_ball_camera_overlaps_three_second_stationary_pause(monkeypatch, camera_duration):
    now = [20.0]
    monkeypatch.setattr("mission_control.motion_command_bridge_node.time.monotonic", lambda: now[0])
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="POST_BALL_GOAL_TRANSITION"))
    assert [r["motion_id"] for r in decoded_messages(bridge.executor_request_publisher)] == ["post_ball_camera_90"]
    assert bridge.post_ball_camera_pause_until == 23.0
    now[0] += camera_duration
    bridge._check_atomic_dwell(now[0])
    assert bridge.motion_in_progress is True
    complete_active_motion(bridge)
    assert bridge.motion_in_progress is True
    assert bridge.active_dwell_until == 23.0
    if now[0] < 23.0:
        bridge._check_atomic_dwell(22.999)
        assert bridge.motion_in_progress is True
    bridge._check_atomic_dwell(max(now[0], 23.0))
    assert bridge.motion_in_progress is False
    assert bridge.post_ball_camera_pause_until is None
    assert decoded_messages(bridge.motion_status_publisher)[-1]["status"] == "SUCCEEDED"
    assert len(bridge.executor_request_publisher.messages) == 1


def test_post_ball_duplicate_camera_success_does_not_skip_pause(monkeypatch):
    monkeypatch.setattr("mission_control.motion_command_bridge_node.time.monotonic", lambda: 20.0)
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="POST_BALL_GOAL_TRANSITION"))
    complete_active_motion(bridge)
    bridge.executor_status_callback(executor_status(
        status="SUCCEEDED", motion_id="post_ball_camera_90",
    ))
    assert bridge.motion_in_progress is True
    assert bridge.active_dwell_until == 23.0
    bridge._check_atomic_dwell(23.0)
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
def test_pickup_top_loss_forward_returns_to_checkpoint_after_dwell(stage):
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
    bridge._check_atomic_dwell(bridge.active_dwell_until - 0.001)
    assert not bridge.pickup_initial_align_waiting
    assert not bridge.pickup_fine_align_waiting
    bridge._check_atomic_dwell(bridge.active_dwell_until)
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
