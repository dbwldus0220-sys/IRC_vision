"""Verify all six gait transitions before sending the next motion to hardware."""

import hashlib
import json
from pathlib import Path

import pytest
import yaml

from test_motion_command_bridge import (
    FakeBridge, complete_active_motion, decoded_messages, executor_status,
    navigation_message, assert_pickup_initial_alignment_started,
    continue_pickup_after_initial_alignment, fine_alignment_message,
)


CASES = [
    ("STRAIGHT", "BALL_FINE_FORWARD_8", "pickup_fine_prepare"),
    ("TURN_LEFT", "BALL_FINE_FORWARD_8", "pickup_fine_prepare"),
    ("TURN_RIGHT", "BALL_FINE_FORWARD_8", "pickup_fine_prepare"),
    ("GOAL_CAMERA_90_FORWARD", "GOAL_CAMERA90_FINE_FORWARD_1", "pickup_fine_prepare"),
    ("GOAL_CAMERA90_TURN_LEFT_1", "GOAL_CAMERA90_FINE_FORWARD_1", "pickup_fine_prepare"),
    ("GOAL_CAMERA90_TURN_RIGHT_2", "GOAL_CAMERA90_FINE_FORWARD_1", "pickup_fine_prepare"),
    *[("GOAL_CAMERA_90_FORWARD", f"GOAL_CAMERA90_CRAB_{side}", "goal_forward_to_crab_right_90")
      for side in ("LEFT", "RIGHT")],
    *[("GOAL_CAMERA90_FINE_FORWARD_1", f"GOAL_CAMERA90_CRAB_{side}", "goal_fine_to_crab_right_90")
      for side in ("LEFT", "RIGHT")],
    ("BALL_FINE_FORWARD_8", "STRAIGHT", "fine_to_turn_ready_45"),
    ("GOAL_CAMERA90_FINE_FORWARD_1", "GOAL_CAMERA_90_FORWARD", "fine_to_turn_ready_90"),
    ("BALL_FINE_FORWARD_8", "TURN_LEFT", "fine_to_turn_ready_45"),
    ("GOAL_CAMERA90_FINE_FORWARD_1", "GOAL_CAMERA90_TURN_RIGHT_2", "fine_to_turn_ready_90"),
]


def requests(bridge):
    return decoded_messages(bridge.executor_request_publisher)


def finish_request(bridge, status="SUCCEEDED", error=""):
    request = requests(bridge)[-1]
    bridge.executor_status_callback(executor_status(
        **request, status=status, error_code=error,
    ))


@pytest.mark.parametrize("previous,target,prepare", CASES)
def test_transition_finishes_before_target_and_rejects_interleaved_commands(previous, target, prepare):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action=previous, command_id=1))
    finish_request(bridge)
    start = len(requests(bridge))
    bridge.navigation_command_callback(navigation_message(action=target, command_id=2))
    prep_request = requests(bridge)[-1]
    assert prep_request["motion_id"] == prepare
    bridge.navigation_command_callback(navigation_message(action="STRAIGHT", command_id=3))
    assert decoded_messages(bridge.motion_status_publisher)[-1]["status"] == "REJECTED"
    bridge.executor_status_callback(executor_status(**prep_request, status="RUNNING"))
    assert len(requests(bridge)) == start + 1
    finish_request(bridge)
    if target.startswith("GOAL_CAMERA90_CRAB_"):
        assert bridge.active_dwell_until is not None
        assert len(requests(bridge)) == start + 1
        bridge._check_atomic_dwell(bridge.active_dwell_until)
    expected_target = bridge.motion_id_for_action(target)
    assert [r["motion_id"] for r in requests(bridge)[start:]] == [prepare, expected_target]
    bridge.executor_status_callback(executor_status(**prep_request, status="SUCCEEDED"))
    assert bridge.motion_in_progress
    assert len(requests(bridge)) == start + 2
    finish_request(bridge)
    assert not bridge.motion_in_progress


@pytest.mark.parametrize("previous,target,prepare", CASES)
@pytest.mark.parametrize("status,error", [
    ("FAILED", "SDK_COMMUNICATION_ERROR"), ("TIMEOUT", "TIMEOUT"),
    ("CANCELLED", ""), ("REJECTED", "INVALID_MOTION"),
])
def test_preparation_failure_never_sends_target(previous, target, prepare, status, error):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action=previous, command_id=1))
    finish_request(bridge)
    start = len(requests(bridge))
    bridge.navigation_command_callback(navigation_message(action=target, command_id=2))
    assert requests(bridge)[-1]["motion_id"] == prepare
    finish_request(bridge, status, error)
    assert not bridge.motion_in_progress
    assert bridge.active_dwell_until is None
    assert [r["motion_id"] for r in requests(bridge)[start:]] == [prepare]


@pytest.mark.parametrize("previous,target,prepare", [
    ("STRAIGHT", "BALL_FINE_FORWARD_8", "pickup_fine_prepare"),
    ("TURN_LEFT", "BALL_FINE_FORWARD_8", "pickup_fine_prepare"),
    ("BALL_FINE_FORWARD_8", "STRAIGHT", "fine_to_turn_ready_45"),
])
@pytest.mark.parametrize("status", ["SUCCEEDED", "FAILED"])
def test_queued_transition_uses_actual_completed_predecessor(previous, target, prepare, status):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action=previous, command_id=1))
    bridge.navigation_command_callback(navigation_message(action=target, command_id=2))
    assert len(requests(bridge)) == 1
    assert bridge.queued_request_deferred
    finish_request(bridge, status)
    if status == "SUCCEEDED":
        assert requests(bridge)[-1]["motion_id"] == prepare
        assert bridge.active_command_id == 2
        complete_active_motion(bridge)
        assert not bridge.motion_in_progress
    else:
        assert len(requests(bridge)) == 1
        assert bridge.queued_request_id is None


def test_hurdle_forward_and_turn_prepare_before_fine_and_final_ten_cycles():
    for previous in ("STRAIGHT", "ALIGN_LEFT", "ALIGN_RIGHT"):
        for target in ("STRAIGHT_0", "GO"):
            bridge = FakeBridge()
            bridge.navigation_command_callback(navigation_message(
                action=previous, source="hurdle", command_id=1,
                source_command={"turn_count": 2},
            ))
            finish_request(bridge)
            bridge.navigation_command_callback(navigation_message(
                action=target, source="hurdle", command_id=2,
            ))
            assert requests(bridge)[-1]["motion_id"] == "pickup_fine_prepare"
            finish_request(bridge)
            assert requests(bridge)[-1]["motion_id"] == (
                "hurdle_fine_forward_10" if target == "GO" else "pickup_fine_forward_0"
            )


@pytest.mark.parametrize("action", ["STRAIGHT", "BALL_FINE_FORWARD_8"])
def test_same_gait_keeps_existing_queue_behavior(action):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action=action, command_id=1))
    bridge.navigation_command_callback(navigation_message(action=action, command_id=2))
    assert not bridge.queued_request_deferred
    assert bridge.pending_turn_request is None
    assert [r["motion_id"] for r in requests(bridge)] == [bridge.motion_id_for_action(action)] * 2


def test_queued_fine_does_not_use_history_from_before_active_retreat():
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="STRAIGHT", command_id=1))
    finish_request(bridge)
    bridge.navigation_command_callback(navigation_message(action="GOAL_CAMERA90_BACKWARD_1", command_id=2))
    bridge.navigation_command_callback(navigation_message(action="GOAL_CAMERA90_FINE_FORWARD_1", command_id=3))
    assert bridge.pending_turn_request is None
    assert requests(bridge)[-1]["motion_id"] == "goal_camera_90_fine_forward_1"


@pytest.mark.parametrize("side", ["LEFT", "RIGHT"])
@pytest.mark.parametrize("status,error", [
    ("FAILED", "SDK_COMMUNICATION_ERROR"), ("FAILED", "SDK_POSITION_TIMEOUT"),
    ("TIMEOUT", "TIMEOUT"), ("CANCELLED", ""),
])
def test_pickup_forward_to_crab_preparation_cannot_be_skipped_on_motor_fault(side, status, error):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW"))
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)
    finish_request(bridge)
    bridge.navigation_command_callback(fine_alignment_message(
        f"BALL_PICKUP_INITIAL_CRAB_{side}", command_id=8002,
    ))
    assert requests(bridge)[-1]["motion_id"] == "pickup_forward_to_crab_0"
    finish_request(bridge, status, error)
    assert not bridge.motion_in_progress
    assert f"pickup_crab_{side.lower()}_0" not in [r["motion_id"] for r in requests(bridge)]


@pytest.mark.parametrize("error", ["SDK_COMMUNICATION_ERROR", "SDK_POSITION_TIMEOUT"])
def test_pickup_approach_transition_failure_does_not_use_motor_fault_skip(error):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW"))
    assert_pickup_initial_alignment_started(bridge)
    continue_pickup_after_initial_alignment(bridge)
    finish_request(bridge)
    continue_pickup_after_initial_alignment(bridge, approach_motion="STRAIGHT_0", command_id=8002)
    assert requests(bridge)[-1]["motion_id"] == "pickup_fine_prepare"
    finish_request(bridge, "FAILED", error)
    assert not bridge.motion_in_progress
    assert decoded_messages(bridge.motion_status_publisher)[-1]["error_code"] == "MOTION_PREPARATION_FAILED"
    assert "ball_general_fine_forward_8" not in [r["motion_id"] for r in requests(bridge)]


def test_aliases_reuse_supplied_poses_without_changing_motion_data():
    root = Path(__file__).resolve().parents[3]
    catalog_path = root / "artifacts/robot_motions_runtime.json"
    manifest = json.loads((root / "artifacts/20261005_catalog8_geon_motion_update/manifest.json").read_text())
    snapshot = root / "artifacts/20261005_catalog9_right_turn_update/robot_motions_runtime.before.json"
    # The later right-turn import may change the catalog, but not transition poses.
    assert hashlib.sha256(snapshot.read_bytes()).hexdigest() == manifest["after_sha256"]
    before = {m["name"]: m for m in json.loads(snapshot.read_text())["motions"]}
    current = {m["name"]: m for m in json.loads(catalog_path.read_text())["motions"]}
    aliases = yaml.safe_load((root / "src/irc_step_motion_executor/config/motion_aliases.yaml").read_text())["motion_aliases"]
    expected = {
        "pickup_fine_prepare": "건오뒤에서 미세오뒤45도",
        "pickup_forward_to_crab_0": "건오뒤에서 기본자세(0도)",
        "pickup_fine_to_crab_right_0": "건미세오뒤에서 기본자세45도",
        "goal_forward_to_crab_right_90": "건오뒤에서 기본자세(카메라45도)",
        "goal_fine_to_crab_right_90": "건미세오뒤에서 기본자세90도",
        "goal_fine_to_default": "건미세오뒤에서 기본자세45도",
        "fine_to_turn_ready_0": "건미세오뒤에서 오뒤0도",
        "fine_to_turn_ready_45": "건미세오뒤에서 오뒤45도",
        "fine_to_turn_ready_90": "찐미세오뒤에서 오뒤(골대 카메라90도)",
    }
    for alias, name in expected.items():
        assert aliases[alias] == name
        assert current[name] == before[name]
