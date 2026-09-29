"""Check physical fine-step/ready-pose/turn ordering across mission paths."""

import pytest

from test_motion_command_bridge import (
    FakeBridge, complete_active_motion, decoded_messages,
    enter_pickup_fine_alignment, executor_status, fine_alignment_message,
    navigation_message, string_message,
)


def requests(bridge):
    return decoded_messages(bridge.executor_request_publisher)


def finish_physical(bridge, status="SUCCEEDED", **updates):
    request = requests(bridge)[-1]
    bridge.executor_status_callback(executor_status(**{
        **request, "status": status, **updates,
    }))


@pytest.mark.parametrize("source,fine,turn,angle", [
    ("line", "BALL_FINE_FORWARD_8", "TURN_LEFT", 45),
    ("line", "BALL_FINE_FORWARD_8", "TURN_RIGHT", 45),
    ("ball", "BALL_FINE_FORWARD_8", "BALL_APPROACH_TURN_LEFT_2", 45),
    ("ball", "BALL_FINE_FORWARD_8", "BALL_APPROACH_TURN_RIGHT_3", 45),
    ("hurdle", "STRAIGHT_0", "ALIGN_LEFT", 0),
    ("hurdle", "STRAIGHT_0", "ALIGN_RIGHT", 0),
    *[("goal", f"GOAL_CAMERA90_FINE_FORWARD_{count}",
       f"GOAL_CAMERA90_TURN_{side}_{repeat}", 90)
      for count in range(1, 5) for side, repeat in (("LEFT", 1), ("RIGHT", 3))],
])
def test_each_mission_prepares_before_turn_and_keeps_command_lock(source, fine, turn, angle):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        source=source, action=fine, command_id=1,
    ))
    finish_physical(bridge)
    bridge.navigation_command_callback(navigation_message(
        source=source, action=turn, command_id=2,
        source_command={"turn_count": 2 if turn == "ALIGN_LEFT" else 3},
    ))
    prep = requests(bridge)[-1]
    target = bridge.active_motion_id
    assert prep["motion_id"] == f"fine_to_turn_ready_{angle}"
    assert bridge.motion_in_progress
    bridge.executor_status_callback(executor_status(**{**prep, "status": "RUNNING"}))
    assert bridge.motion_in_progress
    bridge.navigation_command_callback(navigation_message(action="STRAIGHT", command_id=3))
    assert decoded_messages(bridge.motion_status_publisher)[-1]["status"] == "REJECTED"
    finish_physical(bridge)
    assert requests(bridge)[-1]["motion_id"] == target
    assert bridge.motion_in_progress
    assert not any(s["command_id"] == 2 and s["status"] == "SUCCEEDED"
                   for s in decoded_messages(bridge.motion_status_publisher))
    # A duplicated preparation completion cannot complete the pending turn.
    bridge.executor_status_callback(executor_status(**{**prep, "status": "SUCCEEDED"}))
    assert bridge.motion_in_progress
    finish_physical(bridge)
    assert not bridge.motion_in_progress
    assert decoded_messages(bridge.motion_status_publisher)[-1]["action"] == turn
    bridge.navigation_command_callback(navigation_message(source=source, action=turn, command_id=4))
    assert requests(bridge)[-1]["motion_id"] == target


@pytest.mark.parametrize("flag,angle", [
    ("ball_head_override_active", 0), ("hurdle_head_override_active", 0),
    ("goal_head_override_active", 90),
])
def test_live_camera_hold_selects_transition(flag, angle, monkeypatch):
    monkeypatch.setattr("mission_control.motion_command_bridge_node.time.monotonic", lambda: 10.0)
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="BALL_FINE_FORWARD_8", command_id=1))
    finish_physical(bridge)
    bridge.executor_heartbeat_callback(string_message({flag: True}))
    bridge.navigation_command_callback(navigation_message(action="TURN_RIGHT", command_id=2))
    assert requests(bridge)[-1]["motion_id"] == f"fine_to_turn_ready_{angle}"


@pytest.mark.parametrize("status,error", [
    ("FAILED", "SDK_POSITION_TIMEOUT"), ("FAILED", "SDK_COMMUNICATION_ERROR"),
    ("TIMEOUT", "TIMEOUT"), ("CANCELLED", ""), ("REJECTED", "INVALID_MOTION"),
])
@pytest.mark.parametrize("pickup", [False, True])
def test_preparation_failure_never_starts_turn_even_in_pickup(status, error, pickup):
    bridge = FakeBridge()
    if pickup:
        enter_pickup_fine_alignment(bridge)
        bridge.navigation_command_callback(fine_alignment_message("BALL_PICKUP_FINE_SEARCH_LEFT"))
    else:
        bridge.navigation_command_callback(navigation_message(action="BALL_FINE_FORWARD_8", command_id=1))
        finish_physical(bridge)
        bridge.navigation_command_callback(navigation_message(action="TURN_LEFT", command_id=2))
    target = bridge.active_motion_id
    finish_physical(bridge, status, error_code=error)
    assert target not in [r["motion_id"] for r in requests(bridge)]
    assert not bridge.motion_in_progress
    assert bridge.pending_turn_request is None
    terminal = decoded_messages(bridge.motion_status_publisher)[-1]
    assert terminal["status"] == status
    assert terminal["error_code"] == "TURN_PREPARATION_FAILED"


@pytest.mark.parametrize("fine_status", ["SUCCEEDED", "FAILED", "CANCELLED"])
def test_queued_turn_waits_for_actual_fine_completion(fine_status):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="BALL_FINE_FORWARD_8", command_id=1))
    bridge.navigation_command_callback(navigation_message(action="TURN_RIGHT", command_id=2))
    assert len(requests(bridge)) == 1
    assert bridge.queued_request_deferred
    finish_physical(bridge, fine_status)
    if fine_status == "SUCCEEDED":
        assert requests(bridge)[-1]["motion_id"] == "fine_to_turn_ready_45"
        assert bridge.active_command_id == 2
        finish_physical(bridge)
        assert requests(bridge)[-1]["motion_id"] == "stationary_turn_right"
    else:
        assert len(requests(bridge)) == 1
        assert not bridge.motion_in_progress


@pytest.mark.parametrize("action", ["STRAIGHT", "BALL_APPROACH_RECOVER_LEFT_4", "BALL_APPROACH_RECOVER_RIGHT_4"])
def test_walking_is_not_classified_as_stationary_turn(action):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(action="BALL_FINE_FORWARD_8", command_id=1))
    finish_physical(bridge)
    bridge.navigation_command_callback(navigation_message(action=action, command_id=2))
    assert not requests(bridge)[-1]["motion_id"].startswith("fine_to_turn_ready_")


def test_intervening_walk_discards_old_fine_pose():
    bridge = FakeBridge()
    for command_id, action in enumerate(("BALL_FINE_FORWARD_8", "STRAIGHT", "TURN_LEFT"), 1):
        bridge.navigation_command_callback(navigation_message(action=action, command_id=command_id))
        finish_physical(bridge)
    assert not any(r["motion_id"].startswith("fine_to_turn_ready_") for r in requests(bridge))
