"""Replay current observations at walking boundaries without operating motors."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock, fresh
from test_corner_reacquisition import scene
from test_mission_phase_flow import line_info
from test_motion_decision_planner import ball_info, goal_info
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe


def start_ball(node, clock, bearing=0.):
    fresh(node, clock, 'ball', ball_info(depth_m=1.2, bearing_deg=bearing))
    first = node.publisher.messages[-1]
    assert first['valid'] and first['source'] == 'ball'
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    return first


@pytest.mark.parametrize('initial', [0., -25., 25.])
@pytest.mark.parametrize('next_bearing,expected', [
    (0., 'STRAIGHT'), (-25., 'BALL_APPROACH_RECOVER_LEFT_4'),
    (25., 'BALL_APPROACH_RECOVER_RIGHT_4'),
])
def test_ball_walking_dispatches_latest_direction_in_completion_callback(clock, initial, next_bearing, expected):
    node = LiveInputHarness(phase='BALL_APPROACH')
    first = start_ball(node, clock, initial)
    clock[0] += 1.9
    fresh(node, clock, 'ball', ball_info(depth_m=.9, bearing_deg=next_bearing), plan=False)
    before = len(node.publisher.messages)
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert len(node.publisher.messages) == before + 1
    command = node.publisher.messages[-1]
    assert command['valid'] and command['action'] == expected
    assert node.general_motion_gate.active_command_id == command['command_id']
    assert node.ball_post_motion_dwell_until is None
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert len(node.publisher.messages) == before + 1


@pytest.mark.parametrize('invalid', ['expired', 'future', 'depth', 'confidence', 'failure'])
def test_ball_boundary_never_repeats_forward_on_invalid_input(clock, invalid):
    node = LiveInputHarness(phase='BALL_APPROACH')
    first = start_ball(node, clock)
    clock[0] += 1.9
    changes = {'depth_valid': False} if invalid == 'depth' else {'confidence': .1} if invalid == 'confidence' else {}
    info = ball_info(depth_m=.9, **changes)
    fresh(node, clock, 'ball', info, plan=False)
    if invalid == 'expired':
        clock[0] += .6
    elif invalid == 'future':
        node.latest_info['ball']['rgb_stamp_ns'] = round((clock[0] + 1.) * 1e9)
    before = len(node.publisher.messages)
    node.send_status(first['action'], first['command_id'], 'FAILED' if invalid == 'failure' else 'SUCCEEDED')
    assert not any(m['valid'] and m['action'] == 'STRAIGHT' for m in node.publisher.messages[before:])


def test_pickup_crossing_does_not_commit_another_forward(clock):
    node = LiveInputHarness(phase='BALL_APPROACH')
    first = start_ball(node, clock)
    clock[0] += 1.9
    fresh(node, clock, 'ball', ball_info(depth_m=.8), plan=False)
    fresh(node, clock, 'ball', ball_info(depth_m=.54), plan=False)
    assert node.ball_pickup_entry_pending
    assert getattr(node, 'queued_general_command_id', None) is None
    before = len(node.publisher.messages)
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert not any(m['valid'] and m['action'] == 'STRAIGHT' for m in node.publisher.messages[before:])


@pytest.mark.parametrize('blocked', ['safety', 'executor', 'special'])
def test_completion_dispatch_respects_execution_guards(clock, blocked):
    node = LiveInputHarness(phase='BALL_APPROACH')
    first = start_ball(node, clock)
    clock[0] += 1.9
    fresh(node, clock, 'ball', ball_info(depth_m=.9), plan=False)
    if blocked == 'safety':
        node.safety_interlock.observe_executor_status(
            error_code='SDK_INTERNAL_ERROR', message='Test fault',
            action=first['action'], command_id=first['command_id'], event_id=99,
        )
    elif blocked == 'executor':
        node.executor_auto_ready = False
    else:
        node.phase_manager.start_special_action('PICKUP_NOW', 99)
    before = len(node.publisher.messages)
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert not any(m['valid'] for m in node.publisher.messages[before:])


def test_line_to_ball_handoff_uses_confirmed_frame_received_while_walking(clock):
    node = LiveInputHarness(phase='AUTO')
    fresh(node, clock, 'line', line_info())
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] += 2.
    fresh(node, clock, 'ball', ball_info(depth_m=1.), plan=False)
    before = len(node.publisher.messages)
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    assert len(node.publisher.messages) == before + 1
    assert node.publisher.messages[-1]['source'] == 'ball'
    assert node.publisher.messages[-1]['valid']


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP'])
@pytest.mark.parametrize('bad', [None, 'stale', 'held', 'quality', 'calibration', 'target', 'deadline'])
def test_short_corner_fit_cannot_prequeue_a_corner_or_forward(clock, phase, bad):
    node = LiveInputHarness(phase=phase)
    node.post_ball_line_run_until = clock[0] + (2.1 if bad == 'deadline' else 30.)
    fresh(node, clock, 'line', line_info())
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] += 2.
    for _ in range(16):
        info = scene(clock, distance=.3587, bearing=2.52, offset=-29.228,
                     ground_projection_enabled=True, ground_projection_valid=False,
                     ground_heading_error_deg=None, ground_fit_segment='PRE_CORNER',
                     ground_fit_reason='too_few_segment_points', ground_fit_input_point_count=2)
        if bad == 'held': info['corner_preview_held'] = True
        if bad == 'quality': info['heading_quality'] = .1
        if bad == 'calibration': info['ground_fit_reason'] = 'invalid_camera_intrinsics'
        if bad == 'target': info['corner_start_lateral_offset_m'] = None
        if bad == 'stale': info['rgb_stamp_ns'] = 1
        observe(node, clock, 'line', info, plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    assert node.general_motion_gate.active_command_id == first['command_id']


@pytest.mark.parametrize('source,info', [
    ('ball', ball_info(depth_m=2.)), ('goal', goal_info(depth_m=3., score_now=False)),
])
def test_far_objects_do_not_block_normal_line_prequeue(clock, source, info):
    node = LiveInputHarness(phase='AUTO')
    fresh(node, clock, 'line', line_info())
    first = node.publisher.messages[-1]
    node.send_status(first['action'], first['command_id'], 'RUNNING')
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec * .75
    fresh(node, clock, source, info, plan=False)
    for _ in range(12):
        fresh(node, clock, 'line', line_info(), plan=False)
    assert node.queued_general_action == 'STRAIGHT'
