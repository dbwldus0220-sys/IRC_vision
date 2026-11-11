"""Replay system-clock steps without a camera, ROS graph, or motors."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.vision_time_guard import VisionTimeGuard, capture_is_fresh
from test_mission_phase_flow import line_info
from test_wait_refresh_and_fine_settle import LiveInputHarness


@pytest.mark.parametrize('stamp, valid', [
    (10_000_000_000, True), (9_500_000_000, True),
    (9_499_999_999, False), (10_000_000_001, False),
    (None, False), (True, False), (0, False),
])
def test_capture_age(stamp, valid):
    assert capture_is_fresh(stamp, 10_000_000_000, .5) is valid


@pytest.mark.parametrize('jump', [371., -371.])
def test_jump_needs_distinct_post_jump_frames_and_stable_time(jump):
    guard = VisionTimeGuard()
    guard.check_clock(1_000_000_000_000, 10.)
    ros = round((1000.1 + jump) * 1e9)
    assert guard.check_clock(ros, 10.1) == pytest.approx(jump)
    assert guard.recovering
    assert not guard.accept('line', 1_000_000_000_000, ros, 10.1, .5)
    for delta in (.05, .10):
        stamp = ros + round(delta * 1e9)
        assert not guard.accept('line', stamp, stamp, 10.1 + delta, .5)
        assert not guard.accept('line', stamp, stamp, 10.1 + delta, .5)
    stamp = ros + 250_000_000
    assert guard.accept('line', stamp, stamp, 10.35, .5)
    assert not guard.recovering


def test_callback_delay_is_not_a_clock_jump():
    guard = VisionTimeGuard()
    guard.check_clock(10_000_000_000, 10.)
    assert guard.check_clock(500_000_000_000, 500.) is None
    assert not guard.recovering


def test_bad_frame_resets_recovery_evidence():
    guard = VisionTimeGuard()
    guard.check_clock(10_000_000_000, 10.)
    guard.check_clock(20_000_000_000, 10.1)
    for i in (1, 2):
        stamp = 20_000_000_000 + i * 100_000_000
        assert not guard.accept('ball', stamp, stamp, 10.1 + i * .1, .5)
    assert not guard.accept('ball', 10_000_000_000, 20_300_000_000, 10.4, .5)
    assert not guard.accept('ball', 20_400_000_000, 20_400_000_000, 10.5, .5)


@pytest.fixture
def clocks(monkeypatch):
    values = [10., 1000.]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: values[0])
    monkeypatch.setattr(Node, '_current_ros_time_ns', lambda _: round(values[1] * 1e9))
    return values


def deliver(node, clocks, source='line', age=0., info=None):
    info = line_info() if info is None else info
    payload = {**info, 'rgb_stamp_ns': round((clocks[1] - age) * 1e9)}
    Node._info_callback(node, source)(String(data=json.dumps(payload)))


@pytest.mark.parametrize('source', Node.SOURCES)
@pytest.mark.parametrize('age', [1., -1.])
def test_callback_and_normal_decision_reject_same_bad_time(clocks, source, age):
    node = LiveInputHarness(phase='LINE_TRACK')
    deliver(node, clocks, source, age)
    assert node.latest_info[source] is None
    assert node.vision_transport[source]['status'] == 'capture_time_rejected'
    # Bypassing the callback cannot make a stale cache acceptable either.
    node.latest_info[source] = {'rgb_stamp_ns': round((clocks[1] - age) * 1e9)}
    node.latest_time[source] = clocks[0]
    observations, _ = Node._fresh_observations(node, clocks[0])
    assert observations[source] is None
    assert not Node._recent_walking_input(node, source, clocks[0])


@pytest.mark.parametrize('jump', [371., -371.])
def test_running_motion_survives_jump_but_preview_and_input_do_not(clocks, jump):
    node = LiveInputHarness(phase='LINE_TRACK')
    deliver(node, clocks)
    Node._publish_decision(node)
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    node.pending_line_decision = node.last_selected_decision
    node.pending_line_corner = {'direction': 'RIGHT'}
    node.line_offset_min_rgb_stamp_ns = round(clocks[1] * 1e9)
    clocks[0] += .1
    clocks[1] += .1 + jump
    count = len(node.publisher.messages)
    Node._publish_decision(node)
    assert node.vision_time_guard.recovering
    assert node.general_motion_gate.active_command_id == first['command_id']
    assert node.pending_line_decision is None and node.pending_line_corner is None
    assert all(value is None for value in node.latest_info.values())
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert len(node.publisher.messages) == count
    for _ in range(3):
        clocks[0] += .1
        clocks[1] += .1
        deliver(node, clocks)
    assert not node.vision_time_guard.recovering
    Node._publish_decision(node)
    assert node.publisher.messages[-1]['valid']
    assert node.publisher.messages[-1]['command_id'] != first['command_id']


def test_already_submitted_queue_identity_is_preserved(clocks):
    node = LiveInputHarness(phase='LINE_TRACK')
    deliver(node, clocks)
    node.queued_general_command_id = 42
    node.queued_general_action = 'STRAIGHT'
    clocks[1] += 371.
    Node._publish_decision(node)
    assert node.queued_general_command_id == 42
    assert node.queued_general_action == 'STRAIGHT'
    assert node.vision_time_guard.recovering


def test_high_confidence_ball_with_old_stamp_cannot_latch_entry(clocks):
    from test_motion_decision_planner import ball_info
    node = LiveInputHarness(phase='AUTO')
    deliver(node, clocks, 'ball', age=371., info=ball_info(confidence=.99, confirmation_confirmed=True))
    assert not node.ball_approach_entry_pending
    assert node.latest_info['ball'] is None


def test_clock_jump_before_enter_does_not_disable_start_input(clocks, monkeypatch):
    node = LiveInputHarness(phase='LINE_TRACK')
    node.decision_started = False
    Node._check_vision_clock(node, clocks[0])
    clocks[1] += 371.
    polled = []
    monkeypatch.setattr(Node, '_poll_start_enter', lambda _: polled.append(True))
    Node._publish_decision(node)
    assert node.vision_time_guard.recovering
    assert polled == [True]


def test_lost_ros_clock_blocks_a_production_guard(clocks, monkeypatch):
    node = LiveInputHarness(phase='LINE_TRACK')
    deliver(node, clocks)
    monkeypatch.setattr(Node, '_current_ros_time_ns', lambda _: None)
    Node._publish_decision(node)
    assert not node.publisher.messages
    assert not Node._capture_time_valid(node, 'line', node.latest_info['line'])
