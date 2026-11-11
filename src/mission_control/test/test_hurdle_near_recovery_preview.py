"""Regression tests for recovery limited to fresh near-mode non-detections.

These synthetic observations
reconstruct the logged 564 mm entry; they are not a replay of recorded RGB/depth.
"""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_continuous_approach import clock
from test_hurdle_580_sequence import near, observe, executable, start_far
from test_hurdle_positioning import HurdleHarness, hurdle
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status


RETREAT = 'HURDLE_LOST_BACKWARD_2'
MOTION = 'ball_lost_camera_down_backward_2'
MISSING = {'detected': False, 'raw_detected': False,
           'hurdle_camera_pose': '0', 'hurdle_camera_ready': True}


def enter_after_logged_crossing(clock, *, remember_bottom=False):
    node = HurdleHarness()
    node.planner = MotionDecisionPlanner()
    first = start_far(node, clock)
    clock[0] += .2
    observe(node, clock, hurdle(.564, bottom_distance_px=80,
                               head_down_requested=True, hurdle_camera_ready=False))
    assert node.hurdle_positioning_entry_pending
    if not remember_bottom:
        observe(node, clock, near(bottom_distance_px=200))
    observe(node, clock, MISSING)
    node.publish()
    assert [c['action'] for c in executable(node)] == ['STRAIGHT_3']
    clock[0] += 2.
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED', 'line_forward_6')
    assert node.latest_info['hurdle'] is None
    # Reject a frame captured before completion; require the observation interval.
    clock[0] += .2
    observe(node, clock, MISSING, delay=.3)
    assert node.latest_info['hurdle'] is None
    node.publish()
    assert len(executable(node)) == 1
    clock[0] += .8
    return node


@pytest.mark.parametrize('info', [
    {**MISSING, 'hurdle_camera_ready': False},
    {**MISSING, 'hurdle_camera_pose': '45'},
    {**MISSING, 'raw_detected': True},
    near(depth_valid=False), near(hurdle_angle_0_deg=None),
])
def test_does_not_retreat_before_camera_ready_or_for_visible_waits(clock, info):
    node = enter_after_logged_crossing(clock)
    observe(node, clock, info)
    node.publish()
    assert len(executable(node)) == 1
    clock[0] += .1
    observe(node, clock, MISSING)
    node.publish()
    assert executable(node)[-1]['action'] == RETREAT


@pytest.mark.parametrize('remember_bottom', [True, False])
@pytest.mark.parametrize('next_info,expected', [
    (MISSING, RETREAT), (near(heading=0), 'GO'),
    (near(heading=30), 'ALIGN_RIGHT'), (near(heading=-30), 'ALIGN_LEFT'),
    ({**MISSING, 'raw_detected': True}, None),
])
def test_logged_entry_retreat_and_fresh_reobservation(clock, remember_bottom, next_info, expected):
    node = enter_after_logged_crossing(clock, remember_bottom=remember_bottom)
    observe(node, clock, MISSING)
    node.publish()
    retreat = executable(node)[-1]
    assert retreat['action'] == RETREAT
    node.send_status(RETREAT, retreat['command_id'], 'RUNNING', MOTION)
    # Many new frames and timer callbacks cannot create another physical action.
    for _ in range(100):
        clock[0] += .02
        observe(node, clock, MISSING)
        node.publish()
    assert len(executable(node)) == 2
    node.send_status(RETREAT, retreat['command_id'], 'SUCCEEDED', MOTION)
    clock[0] += .2
    observe(node, clock, next_info, delay=.3)
    node.publish()
    assert node.latest_info['hurdle'] is None
    assert len(executable(node)) == 2
    clock[0] += .8
    node.publish()
    assert len(executable(node)) == 2
    clock[0] += .01
    observe(node, clock, next_info)
    node.publish()
    if expected is None:
        assert len(executable(node)) == 2
    else:
        assert len(executable(node)) == 3
        assert executable(node)[-1]['action'] == expected
    print(json.dumps({'remember_bottom': remember_bottom,
                      'next_action': expected or 'WAIT_CONFIRMATION',
                      'published_actions': [c['action'] for c in executable(node)],
                      'missing_frames_during_retreat': 100}))


def test_go_sequence_is_not_interrupted_by_missing_hurdle_frames(clock):
    node = enter_after_logged_crossing(clock)
    observe(node, clock, near(heading=0))
    node.publish()
    go = executable(node)[-1]
    assert go['action'] == 'GO'
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(String(data=json.dumps(go)))
    forwarded = 0

    def sync():
        nonlocal forwarded
        for message in bridge.motion_status_publisher.messages[forwarded:]:
            Node._motion_status_callback(node, message)
        forwarded = len(bridge.motion_status_publisher.messages)

    sync()
    for _ in range(30):
        assert node.active_special_action == 'GO'
        for _ in range(5):
            clock[0] += .02
            observe(node, clock, MISSING)
            node.publish()
        assert [c['action'] for c in executable(node)] == ['STRAIGHT_3', 'GO']
        deadline = bridge.transition_dwell_until or bridge.active_dwell_until
        if deadline is not None:
            clock[0] = max(clock[0], deadline)
            bridge._check_atomic_dwell()
        else:
            request = decoded_messages(bridge.executor_request_publisher)[-1]
            clock[0] += 1.
            bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
        sync()
        if not bridge.motion_in_progress:
            break
    assert node.phase_manager.hurdles_completed == 1
    assert node.mission_phase == 'LINE_TRACK'
    assert all(r['motion_id'] != MOTION for r in decoded_messages(bridge.executor_request_publisher))


def test_far_wait_does_not_retreat():
    info = {'hurdle': hurdle(.8, hurdle_angle_45_deg=None)}
    assert MotionDecisionPlanner().plan('HURDLE_APPROACH', info, .1).action == 'WAIT'


def test_near_phase_without_a_confirmed_near_entry_does_not_retreat():
    decision = MotionDecisionPlanner().plan('HURDLE_POSITIONING', {'hurdle': MISSING}, .1)
    assert decision.action == 'WAIT' and not decision.valid
