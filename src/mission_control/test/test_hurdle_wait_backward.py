"""Near-hurdle loss retreats after camera-down and never overlaps a motion."""

import json
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_continuous_approach import clock
from test_hurdle_580_sequence import near, observe, executable, start_far
from test_hurdle_positioning import HurdleHarness, hurdle
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status


@pytest.mark.parametrize('info,reason', [
    (None, 'waiting_for_hurdle_info'),
    (near(detected=False, raw_detected=True), 'hurdle_confirmation_pending'),
    (near(confirmation_confirmed=False), 'hurdle_confirmation_pending'),
    (near(confidence=.1), 'low_hurdle_confidence'),
    (near(depth_valid=False), 'missing_valid_hurdle_depth'),
    (near(hurdle_camera_ready=False), 'waiting_for_hurdle_camera_zero_frame'),
    (near(hurdle_camera_pose='45'), 'waiting_for_hurdle_camera_zero_frame'),
    (near(hurdle_angle_0_deg=None), 'waiting_for_fresh_hurdle_calibrated_angle'),
])
def test_other_hurdle_wait_reasons_do_not_retreat(info, reason):
    planner = MotionDecisionPlanner()
    planner.hurdle_planner.begin_near()
    planner.hurdle_planner.first_near_decision = False
    command = planner.plan('HURDLE_POSITIONING', {'hurdle': info}, .1)
    assert not command.valid and command.action == 'WAIT'
    assert command.reason == reason
    assert not command.sdk_motion_requested and not command.requires_ack
    assert not planner.hurdle_go_requested


def test_far_hurdle_wait_remains_wait_and_valid_alignment_is_unchanged():
    planner = MotionDecisionPlanner()
    wait = planner.plan('HURDLE_APPROACH', {'hurdle': hurdle(.8, hurdle_angle_45_deg=None)}, .1)
    assert not wait.valid and wait.action == 'WAIT'
    assert planner.plan('HURDLE_APPROACH', {'hurdle': hurdle(.8)}, .1).action == 'STRAIGHT_3'


def test_special_motion_lock_and_other_sources_do_not_retreat():
    for phase in ('HURDLE_POSITIONING_LOCK', 'BALL_APPROACH', 'GOAL_APPROACH', 'LINE_TRACK'):
        decision = MotionDecisionPlanner().plan(phase, {}, .1)
        assert decision.action != 'HURDLE_LOST_BACKWARD_2'
        assert not decision.valid


def heartbeat(bridge, clock, current=-64., active=True):
    bridge.executor_heartbeat_callback(String(data=json.dumps({
        'hurdle_head_override_active': active,
        'hurdle_head_override_current_deg': current,
    })))


def test_retreat_waits_for_camera_zero_then_runs_once_and_finishes_post_hold(clock):
    planner = MotionDecisionPlanner()
    planner.hurdle_planner.begin_near()
    decision = planner.plan('HURDLE_POSITIONING', {'hurdle': {
        'detected': False, 'raw_detected': False,
        'hurdle_camera_ready': True, 'hurdle_camera_pose': '0',
    }}, .1)
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.navigation_command_callback(String(data=json.dumps({
        **decision.to_dict(), 'command_id': 1,
    })))
    clock[0] += .5
    for current, active in ((-34., True), (-63., True), (-64., False), (None, True)):
        heartbeat(bridge, clock, current, active)
        bridge._check_transition_dwell()
        assert not bridge.executor_request_publisher.messages
    heartbeat(bridge, clock)
    clock[0] += 1.01
    bridge._check_transition_dwell()
    assert not bridge.executor_request_publisher.messages
    heartbeat(bridge, clock)
    bridge._check_transition_dwell()
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == 'ball_lost_camera_down_backward_2'
    for _ in range(5):
        bridge._check_transition_dwell()
    assert len(bridge.executor_request_publisher.messages) == 1
    bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    assert bridge.motion_in_progress
    clock[0] += .5
    bridge._check_transition_dwell()
    assert not bridge.motion_in_progress


def test_near_entry_wait_retreats_again_only_after_completion_and_new_observation(clock):
    node = HurdleHarness()
    first = start_far(node, clock)
    clock[0] += .2
    observe(node, clock, hurdle(.564, bottom_distance_px=80))
    # A later interior observation removes the edge memory seen at entry.
    observe(node, clock, near(bottom_distance_px=200))
    missing = {'detected': False, 'raw_detected': False,
               'hurdle_camera_pose': '0', 'hurdle_camera_ready': True}
    observe(node, clock, missing)
    assert len(executable(node)) == 1
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED', 'line_forward_6')
    clock[0] += 1.
    observe(node, clock, missing)
    node.publish()
    retreat = executable(node)[-1]
    assert retreat['action'] == 'HURDLE_LOST_BACKWARD_2'
    node.send_status(retreat['action'], retreat['command_id'], 'RUNNING',
                     'ball_lost_camera_down_backward_2')
    for _ in range(5):
        clock[0] += .1
        observe(node, clock, missing)
        node.publish()
    assert len(executable(node)) == 2
    node.send_status(retreat['action'], retreat['command_id'], 'SUCCEEDED',
                     'ball_lost_camera_down_backward_2')
    node.publish()
    assert len(executable(node)) == 2
    clock[0] += 1.
    observe(node, clock, missing)
    node.publish()
    assert len(executable(node)) == 3
    assert executable(node)[-1]['action'] == 'HURDLE_LOST_BACKWARD_2'


def test_existing_retreat_catalog_still_has_exactly_two_cycles():
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())
    name = aliases['motion_aliases']['ball_lost_camera_down_backward_2']
    motions = json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']
    assert next(m for m in motions if m['name'] == name)['repeat_count'] == 2
