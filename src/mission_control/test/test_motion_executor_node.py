import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "mission_control"))

from motion_executor_core import ExecutorState, MotionExecutionResult
from motion_executor_node import (
    ExecutionPublicationState,
    MotionRequest,
    RequestValidationError,
    build_status_payload,
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
