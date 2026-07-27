import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from mock_motion_player import MockRobotMotionPlayer, MotionError
from motion_executor_core import (
    ExecutorState,
    MotionExecutionResult,
    MotionExecutorCore,
)
from motion_executor_node import (
    CancelRequest,
    ExecutionPublicationState,
    MotionRequest,
    RequestValidationError,
    build_status_payload,
    handle_cancel_request,
    parse_cancel_request,
    parse_motion_request,
)


def result(status, error_code="NONE", message="done"):
    return MotionExecutionResult(
        motion_id="forward",
        final_status=status,
        success=status is ExecutorState.SUCCEEDED,
        error_code=error_code,
        message=message,
    )


def test_parse_valid_request_json():
    request = parse_motion_request(
        '{"request_id": 1, "motion_id": "forward", "timeout_ms": 5000}'
    )
    assert request == MotionRequest(1, "forward", 5000)


def test_missing_required_field():
    with pytest.raises(RequestValidationError) as exc_info:
        parse_motion_request('{"request_id": 1, "motion_id": "forward"}')
    assert exc_info.value.error_code == "INVALID_REQUEST"


def test_invalid_json():
    with pytest.raises(RequestValidationError) as exc_info:
        parse_motion_request("{invalid")
    assert exc_info.value.error_code == "INVALID_REQUEST"


def test_non_positive_timeout():
    for timeout_ms in (0, -1):
        with pytest.raises(RequestValidationError) as exc_info:
            parse_motion_request(
                '{"request_id": 1, "motion_id": "forward", '
                f'"timeout_ms": {timeout_ms}}}'
            )
        assert exc_info.value.error_code == "INVALID_REQUEST"


def test_running_status_payload():
    state = ExecutionPublicationState()
    state.begin(MotionRequest(7, "turn_left", 1000))
    assert state.running_payload() == {
        "request_id": 7,
        "motion_id": "turn_left",
        "status": "RUNNING",
        "error_code": "",
        "message": "",
    }


def test_succeeded_terminal_payload():
    state = ExecutionPublicationState()
    state.begin(MotionRequest(1, "forward", 5000))
    payload = state.terminal_payload(result(ExecutorState.SUCCEEDED))
    assert payload["status"] == "SUCCEEDED"
    assert payload["error_code"] == "NONE"


def test_failed_payload():
    payload = build_status_payload(
        1, "forward", "FAILED", "COMMUNICATION_ERROR", "send failed"
    )
    assert payload["status"] == "FAILED"
    assert payload["error_code"] == "COMMUNICATION_ERROR"


def test_rejected_payload():
    payload = build_status_payload(
        2, "fly", "REJECTED", "INVALID_MOTION", "unsupported motion_id"
    )
    assert payload["status"] == "REJECTED"
    assert payload["error_code"] == "INVALID_MOTION"


def test_original_request_id_and_motion_id_are_preserved():
    state = ExecutionPublicationState()
    state.begin(MotionRequest(42, "shoot", 8000))
    payload = state.terminal_payload(
        MotionExecutionResult(
            motion_id="internal_motion",
            final_status=ExecutorState.SUCCEEDED,
            success=True,
            error_code="NONE",
            message="done",
        )
    )
    assert payload["request_id"] == 42
    assert payload["motion_id"] == "shoot"


def test_terminal_payload_is_not_duplicated():
    state = ExecutionPublicationState()
    state.begin(MotionRequest(1, "forward", 5000))
    terminal = result(ExecutorState.SUCCEEDED)
    assert state.terminal_payload(terminal) is not None
    assert state.terminal_payload(terminal) is None


def active_execution(request_id=10, motion_id="forward", player=None):
    player = player or MockRobotMotionPlayer()
    core = MotionExecutorCore(player)
    state = ExecutionPublicationState()
    request = MotionRequest(request_id, motion_id, 5000)
    assert core.start_motion(motion_id, 5000) is None
    state.begin(request)
    return core, state, player


def test_parse_cancel_request():
    assert parse_cancel_request('{"request_id": 10}') == CancelRequest(10)


def test_cancel_with_matching_request_id():
    core, state, _ = active_execution(request_id=10, motion_id="shoot")
    handled = handle_cancel_request('{"request_id": 10}', core, state)
    assert handled.terminal is True
    assert handled.payload["request_id"] == 10
    assert handled.payload["motion_id"] == "shoot"
    assert handled.payload["status"] == "CANCELLED"


def test_cancel_with_mismatched_request_id_is_rejected():
    core, state, _ = active_execution(request_id=10)
    handled = handle_cancel_request('{"request_id": 11}', core, state)
    assert handled.terminal is False
    assert handled.payload["status"] == "REJECTED"
    assert handled.payload["error_code"] == "REQUEST_ID_MISMATCH"
    assert core.busy() is True


def test_cancel_when_not_running_is_rejected():
    core = MotionExecutorCore(MockRobotMotionPlayer())
    state = ExecutionPublicationState()
    handled = handle_cancel_request('{"request_id": 10}', core, state)
    assert handled.terminal is False
    assert handled.payload["status"] == "REJECTED"
    assert handled.payload["error_code"] == "NOT_RUNNING"


def test_cancelled_terminal_is_generated_only_once():
    core, state, _ = active_execution()
    first = handle_cancel_request('{"request_id": 10}', core, state)
    second = state.terminal_payload(core.terminal_result())
    assert first.payload["status"] == "CANCELLED"
    assert second is None


def test_mock_failure_payload_preserves_original_request():
    player = MockRobotMotionPlayer(
        fail_after_updates=1,
        failure_error=MotionError.COMMUNICATION_ERROR,
        failure_message="injected communication failure",
    )
    core, state, _ = active_execution(
        request_id=77, motion_id="turn_right", player=player
    )

    terminal = core.tick(10)
    payload = state.terminal_payload(terminal)
    assert payload["request_id"] == 77
    assert payload["motion_id"] == "turn_right"
    assert payload["status"] == "FAILED"
    assert payload["error_code"] == "COMMUNICATION_ERROR"
    assert isinstance(payload["error_code"], str)
    assert "injected communication failure" in payload["message"]
    assert state.terminal_payload(terminal) is None
