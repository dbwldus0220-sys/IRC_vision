"""Replay confirmed corner approach across sparse fits and actual completion gates."""

import math

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from mission_control.lost_search_turn_limiter import LostSearchTurnLimiter
from test_line_ground_fit_recovery import clock, node_for, frame
from test_sparse_line_recovery import sparse_info, prepare, wait_decision
from mission_control.sparse_line_recovery import SparseLineRecovery


def corner(distance=.38, direction='RIGHT', lateral=.06, heading=0.):
    sign = 1 if direction == 'RIGHT' else -1
    info = sparse_info(heading=heading, direction=direction, distance=distance)
    slope = math.tan(math.radians(heading))
    info.update(lateral_offset_px=sign*87.8, filtered_lateral_offset_norm=sign*.137,
                turn_angle_deg=sign*78.2, ground_two_point_candidate={
                    'heading_deg': heading,
                    'points_m': [[sign*lateral + slope*z, z] for z in (distance, distance+.08)],
                })
    return info


def next_motion(node, clock, info):
    for _ in range(20):
        clock[0] += .1
        commands = frame(node, clock, info)
        valid = [m for m in commands if m['valid']]
        if valid:
            assert len(valid) == 1
            return valid[0]
    pytest.fail(f'No motion: {node.publisher.messages[-1]}')


def complete(node, clock, command, status='SUCCEEDED'):
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    clock[0] += .5
    node.send_status(command['action'], command['command_id'], status)
    deadline = getattr(node, 'correction_post_motion_dwell_until', None)
    if deadline:
        clock[0] = max(clock[0], deadline) + .01
        Node._publish_decision(node)


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP', 'POST_BALL_LINE_ALIGN'])
@pytest.mark.parametrize('direction', ['RIGHT', 'LEFT'])
def test_two_point_corner_approaches_then_executes_mapped_recovery(clock, phase, direction):
    node = node_for(phase)
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 0.
    command = next_motion(node, clock, corner(direction=direction))
    assert command['action'] == 'LINE_SPARSE_FORWARD'
    assert command['source_command']['ground_fit_mode'] == 'TWO_POINT_CONFIRMED'
    assert node.latest_info['line']['ground_projection_valid'] is False
    complete(node, clock, command)
    command = next_motion(node, clock, corner(.12, direction=direction))
    assert command['action'] == direction
    assert command['source_command']['corner_turn_authorized']
    assert Bridge.motion_id_for_action(command['action']) == f'line_recovery_{direction.lower()}_4'


def test_three_image_turns_do_not_block_confirmed_corner_forward(clock):
    node = node_for()
    node.lost_search_turn_limiter = LostSearchTurnLimiter(staged_sources=('line',))
    limiter = node.lost_search_turn_limiter
    limiter.image_alignment_count = 3
    limiter.image_alignment_angle = 45.
    limiter.counts['line'] = 3
    limiter.angles['line'] = 45.
    result = next_motion(node, clock, corner())
    assert result['action'] == 'LINE_SPARSE_FORWARD'
    assert limiter.image_alignment_count == 0
    # Seeing geometry is not permission to refund a blind corner search.
    assert limiter.counts['line'] == 3


def test_progress_allows_more_than_two_short_steps_but_static_input_stops(clock):
    node = node_for()
    for distance in (.60, .48, .36):
        command = next_motion(node, clock, corner(distance))
        assert command['action'] == 'LINE_SPARSE_FORWARD'
        complete(node, clock, command)
    # One unchanged observation may lead to another step; repeated lack of
    # improvement exhausts the existing two-motion allowance.
    command = next_motion(node, clock, corner(.36))
    complete(node, clock, command)
    for _ in range(6):
        clock[0] += .1
        assert not any(m['valid'] for m in frame(node, clock, corner(.36)))
    assert node.last_selected_decision.reason == 'sparse_line_motion_limit_reached'


def test_failed_sparse_recovery_blocks_retries(clock):
    node = node_for()
    command = next_motion(node, clock, corner())
    complete(node, clock, command, 'FAILED')
    for _ in range(6):
        clock[0] += .1
        assert not any(m['valid'] for m in frame(node, clock, corner()))
    assert node.sparse_line_recovery.failed


def test_confirmed_two_points_supply_bounded_ground_target():
    from mission_control.motion_decision_planner import MotionDecisionPlanner
    policy = SparseLineRecovery()
    info = corner(lateral=.25)
    for t in (10., 10.1, 10.2):
        prepared = prepare(policy, info, t)
    target = MotionDecisionPlanner()._line_offset_ground_target(prepared)
    assert target is not None and .38 <= target[1] <= .46
    assert target[3] == pytest.approx(.25)
    assert MotionDecisionPlanner()._line_offset_ground_target(info) is None


def test_motion_boundary_rejects_earlier_two_point_samples():
    policy = SparseLineRecovery()
    for t in (10., 10.1, 10.2):
        info = prepare(policy, corner(), t)
    decision = policy._motion(wait_decision(), 'LINE_SPARSE_FORWARD', 'two_point_short_forward')
    policy.published(decision, 10_200_000_000)
    policy.completed(decision.action, 'SUCCEEDED', 11_000_000_000)
    for t in (10.3, 10.4, 10.5):
        assert not prepare(policy, corner(), t, ros_now=11_100_000_000)['ground_projection_valid']


@pytest.mark.parametrize('changes', [
    {'corner_preview_held': True}, {'corner_start_depth_valid': False},
    {'geometry_quality': .1}, {'ground_two_point_candidate': None},
])
def test_no_corner_forward_from_unreliable_two_point_evidence(clock, changes):
    node = node_for()
    for _ in range(8):
        clock[0] += .1
        commands = frame(node, clock, {**corner(), **changes})
        assert not any(m['valid'] for m in commands)


def image_turn(error):
    from dataclasses import replace
    side, count = ('RIGHT', 2) if error > 0 else ('LEFT', 1)
    return replace(wait_decision(), valid=True, action=f'LINE_OFFSET_TURN_{side}_{count}',
                   source_command={'image_line_alignment': True,
                                   'image_target_bearing_deg': error,
                                   'turn_direction': side})


def test_image_alignment_only_charges_published_turns_and_requires_progress():
    limiter = LostSearchTurnLimiter(staged_sources=('line',))
    observations = {'line': {'detected': True}}
    for _ in range(8):
        assert limiter.filter(image_turn(40.), observations).valid
    assert limiter.image_alignment_count == 0
    limiter.record_published(image_turn(40.), observations)
    assert limiter.counts['line'] == 0
    assert not limiter.filter(image_turn(39.), observations).valid
    assert limiter.filter(image_turn(37.), observations).valid
    assert limiter.filter(image_turn(40.), observations).reason == 'line_image_alignment_no_progress'


def test_image_alignment_detects_left_right_oscillation():
    limiter = LostSearchTurnLimiter()
    observations = {'line': {'detected': True}}
    for error in (40., -25.):
        assert limiter.filter(image_turn(error), observations).valid
        limiter.record_published(image_turn(error), observations)
    result = limiter.filter(image_turn(15.), observations)
    assert not result.valid and result.reason == 'line_image_alignment_oscillating'


def test_lost_search_exhaustion_does_not_block_current_image_alignment():
    limiter = LostSearchTurnLimiter(staged_sources=('line',))
    limiter.counts['line'] = 3
    limiter.angles['line'] = 45.
    assert limiter.filter(image_turn(40.), {'line': {'detected': True}}).valid
