"""Check real timer boundaries around posture transitions without driving hardware."""

import pytest

from test_gait_transitions import CASES, finish_request, requests
from test_motion_command_bridge import (
    FakeBridge, decoded_messages, executor_status, fine_alignment_message,
    navigation_message,
)


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr("mission_control.motion_command_bridge_node.time.monotonic", lambda: now[0])
    return now


def check_both_pauses(bridge, clock, prepare, target):
    start = len(requests(bridge))
    beginning = clock[0]
    assert bridge.motion_in_progress
    assert bridge.transition_dwell_until == beginning + 1.0
    clock[0] = beginning + .999
    bridge._check_atomic_dwell()
    assert len(requests(bridge)) == start
    clock[0] = beginning + 1.0
    bridge._check_atomic_dwell()
    assert [r["motion_id"] for r in requests(bridge)[start:]] == [prepare]
    prep_request = requests(bridge)[-1]
    clock[0] += 3.0
    finish_request(bridge)
    completion = clock[0]
    assert bridge.motion_in_progress
    assert len(requests(bridge)) == start + 1
    deadline = bridge.transition_dwell_until or bridge.active_dwell_until
    assert deadline == completion + 1.0
    # Repeated completions must neither restart the timer nor finish the command.
    clock[0] = completion + .5
    bridge.executor_status_callback(executor_status(**prep_request, status="SUCCEEDED"))
    assert (bridge.transition_dwell_until or bridge.active_dwell_until) == deadline
    bridge._check_atomic_dwell()
    assert len(requests(bridge)) == start + 1
    clock[0] = completion + .999
    bridge._check_atomic_dwell()
    assert len(requests(bridge)) == start + 1
    clock[0] = completion + 1.0
    bridge._check_atomic_dwell()
    assert [r["motion_id"] for r in requests(bridge)[start:]] == [prepare, target]
    assert bridge.transition_dwell_until is None
    assert bridge.active_dwell_until is None
    bridge._check_atomic_dwell()
    assert len(requests(bridge)) == start + 2


@pytest.mark.parametrize("previous,target,prepare", CASES)
def test_all_six_transition_types_pause_exactly_once_on_both_sides(clock, previous, target, prepare):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action=previous, command_id=1))
    finish_request(bridge)
    bridge.navigation_command_callback(navigation_message(action=target, command_id=2))
    check_both_pauses(bridge, clock, prepare, bridge.motion_id_for_action(target))
    finish_request(bridge)
    assert not bridge.motion_in_progress


@pytest.mark.parametrize("previous,action,prepare,target", [
    ("ball_camera_down_forward_4", "BALL_PICKUP_CRAB_LEFT", "pickup_forward_to_crab_0", "pickup_crab_left_0"),
    ("ball_camera_down_forward_4", "BALL_PICKUP_CRAB_RIGHT", "pickup_forward_to_crab_0", "pickup_crab_right_0"),
    ("pickup_fine_forward_0", "BALL_PICKUP_CRAB_LEFT", "pickup_fine_to_crab_right_0", "pickup_crab_left_0"),
    ("pickup_fine_forward_0", "BALL_PICKUP_CRAB_RIGHT", "pickup_fine_to_crab_right_0", "pickup_crab_right_0"),
    ("ball_camera_down_forward_4", "BALL_PICKUP_FINE_FORWARD", "pickup_fine_prepare", "pickup_fine_forward_0"),
])
def test_pickup_explicit_preparations_share_the_same_timing(clock, previous, action, prepare, target):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action="PICKUP_NOW"))
    # Enter the fine checkpoint after the specified completed predecessor.
    bridge.pickup_initial_align_waiting = False
    bridge.pickup_fine_align_waiting = True
    bridge.active_motion_id = bridge.FINE_ALIGN_MARKER
    bridge.last_physical_motion_id = previous
    bridge.navigation_command_callback(fine_alignment_message(action))
    check_both_pauses(bridge, clock, prepare, target)
    assert bridge.active_action == "PICKUP_NOW"
    assert not bridge.pickup_fine_align_waiting


@pytest.mark.parametrize("previous,target", [("STRAIGHT", "STRAIGHT_0"), ("ALIGN_RIGHT", "GO")])
def test_hurdle_transition_preserves_one_second_pauses(clock, previous, target):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(
        source="hurdle", action=previous, command_id=1, source_command={"turn_count": 2},
    ))
    finish_request(bridge)
    bridge.navigation_command_callback(navigation_message(source="hurdle", action=target, command_id=2))
    check_both_pauses(bridge, clock, "pickup_fine_prepare",
                      "hurdle_fine_forward_10" if target == "GO" else "pickup_fine_forward_0")


@pytest.mark.parametrize("status", ["FAILED", "TIMEOUT", "CANCELLED", "REJECTED"])
def test_transition_failure_never_starts_post_pause_or_target(clock, status):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = "line_forward_4"
    bridge.navigation_command_callback(navigation_message(action="BALL_FINE_FORWARD_8"))
    clock[0] += 1.0
    bridge._check_atomic_dwell()
    finish_request(bridge, status, "TEST_FAILURE")
    assert not bridge.motion_in_progress
    assert bridge.transition_dwell_until is None
    clock[0] += 10.0
    bridge._check_atomic_dwell()
    assert [r["motion_id"] for r in requests(bridge)] == ["pickup_fine_prepare"]


def test_transition_lock_covers_both_pauses_and_ignores_wrong_status(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = "pickup_fine_forward_0"
    bridge.navigation_command_callback(navigation_message(action="TURN_RIGHT"))
    for command_id in (8001, 8002):
        bridge.navigation_command_callback(navigation_message(action="STRAIGHT", command_id=command_id))
        assert decoded_messages(bridge.motion_status_publisher)[-1]["error_code"] == "ATOMIC_SEQUENCE_LOCKED"
        bridge.executor_status_callback(executor_status(status="SUCCEEDED", motion_id="stationary_turn_right"))
        assert bridge.motion_in_progress
        clock[0] += 1.0
        bridge._check_atomic_dwell()
        if command_id == 8001:
            finish_request(bridge)
    assert [r["motion_id"] for r in requests(bridge)] == ["fine_to_turn_ready_0", "stationary_turn_right"]


@pytest.mark.parametrize("during_post_pause", [False, True])
def test_clearing_command_discards_pending_timer_work(clock, during_post_pause):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = "line_forward_4"
    bridge.navigation_command_callback(navigation_message(action="BALL_FINE_FORWARD_8"))
    if during_post_pause:
        clock[0] += 1.0
        bridge._check_atomic_dwell()
        finish_request(bridge)
    bridge._clear_active_request()
    count = len(requests(bridge))
    clock[0] += 10.0
    bridge._check_atomic_dwell()
    assert len(requests(bridge)) == count


def test_queued_transition_pause_begins_after_predecessor_completion(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(navigation_message(action="STRAIGHT", command_id=1))
    bridge.navigation_command_callback(navigation_message(action="BALL_FINE_FORWARD_8", command_id=2))
    assert bridge.transition_dwell_until is None
    clock[0] += 5.0
    bridge._check_atomic_dwell()
    assert len(requests(bridge)) == 1
    finish_request(bridge)
    check_both_pauses(bridge, clock, "pickup_fine_prepare", "ball_general_fine_forward_8")
