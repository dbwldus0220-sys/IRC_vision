"""Regress acquired Ball tracking and bounded reuse of valid Line captures."""

import pytest

from mission_control.ball_approach_observation import BallApproachObservation
from mission_control.motion_decision_node import MotionDecisionNode as Node
from step.ball_navigation_planner import BallNavigationConfig
from test_ball_approach_latency import sample
from test_ball_entry_line_alignment import node_with_entry, start_line, send, status
from test_ball_entry_reobservation import observation
from test_continuous_approach import clock, fresh
from test_motion_command_bridge import FakeBridge
from test_vision_time_guard import clocks, deliver
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_mission_phase_flow import line_info


def test_initial_acquisition_survives_walking_with_current_matched_candidates(clock):
    node, bridge = node_with_entry(), FakeBridge(advance_transition_dwells=False)
    _, running = start_line(node, bridge, clock)
    started = node.active_line_motion_started_at
    fresh(node, clock, 'ball', observation(clock, confirmed=True), plan=False)
    acquired = node.ball_approach_observation.anchor_received_at
    for _ in range(35):
        clock[0] += .07
        fresh(node, clock, 'ball', observation(clock), plan=False)
        if node.queued_general_command_id is not None:
            break
    assert clock[0] - acquired > .5
    assert clock[0] >= started + node.active_line_motion_duration_sec * .6
    assert node.queued_general_action == 'STRAIGHT_2'
    assert node.ball_approach_observation.anchor_received_at == acquired
    assert node.ball_approach_observation.reference_received_at > acquired
    assert not node.ball_confirmation_pending_latched
    # No new Line input was needed after Ball took ownership.
    queued = send(bridge, node.publisher.messages[-1])
    status(node, bridge, running, 'SUCCEEDED')
    status(node, bridge, queued, 'RUNNING')
    assert bridge.transition_dwell_until is None
    assert node.general_motion_gate.active_command_id == queued['command_id']


@pytest.mark.parametrize('invalid', ['gap', 'target', 'depth', 'camera', 'missing', 'unconfirmed_loss'])
def test_retained_identity_never_supplies_old_geometry(invalid):
    observer, config = BallApproachObservation(), BallNavigationConfig()
    observer.observe(sample(10), 10, config, preserve_gaps=True)
    for i in range(1, 15):
        now = 10 + i * .1
        observer.observe(sample(now, detected=False), now, config, preserve_gaps=True)
    assert observer.is_current(now)
    now += .51 if invalid == 'gap' else .1
    info = sample(now, detected=False)
    if invalid == 'target':
        info['bbox'] = [850, 240, 900, 290]
    elif invalid == 'depth':
        info['approach_candidate']['depth_valid'] = False
    elif invalid == 'camera':
        info['approach_candidate']['head_down_requested'] = True
    elif invalid in {'missing', 'unconfirmed_loss'}:
        info = dict(detected=False, raw_detected=False, rgb_stamp_ns=round(now * 1e9),
                    ball_loss_confirmed=invalid == 'missing')
    observer.observe(info, now, config, preserve_gaps=True)
    assert not observer.is_current(now)
    assert observer.observation is None and observer.candidate is None


@pytest.mark.parametrize('age', [1., -1.])
def test_bad_line_packet_retains_recent_good_capture_without_refreshing_age(clocks, age):
    node = LiveInputHarness(phase='LINE_TRACK')
    deliver(node, clocks)
    old, received = node.latest_info['line'], node.latest_time['line']
    clocks[0] += .1
    clocks[1] += .1
    deliver(node, clocks, age=age)
    assert node.latest_info['line'] is old
    assert node.latest_time['line'] == received
    assert node.vision_transport['line']['retained_valid_line']
    assert Node._fresh_observations(node, clocks[0])[0]['line'] is old
    clocks[0] += .41
    clocks[1] += .41
    deliver(node, clocks, age=age)
    assert node.latest_info['line'] is None
    assert not node.vision_transport['line']['retained_valid_line']


@pytest.mark.parametrize('boundary', ['line_motion_min_rgb_stamp_ns', 'line_offset_min_rgb_stamp_ns'])
def test_line_reuse_cannot_cross_pose_or_motion_boundary(clocks, boundary):
    node = LiveInputHarness(phase='LINE_TRACK')
    deliver(node, clocks)
    setattr(node, boundary, round(clocks[1] * 1e9))
    clocks[0] += .1
    clocks[1] += .1
    deliver(node, clocks, age=1.)
    assert node.latest_info['line'] is None


def test_current_line_loss_supersedes_cached_geometry(clocks):
    node = LiveInputHarness(phase='LINE_TRACK')
    deliver(node, clocks)
    clocks[0] += .1
    clocks[1] += .1
    deliver(node, clocks, info={**line_info(), 'detected': False})
    assert node.latest_info['line']['detected'] is False
    deliver(node, clocks, age=1.)
    assert node.latest_info['line'] is None
