"""Replay confirmed corners through the real callbacks and motion boundaries."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.motion_command_bridge_node import MotionCommandBridgeNode
from test_mission_phase_flow import MissionFlowHarness, line_info, release_general


@pytest.fixture
def clock(monkeypatch):
    now = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: now[0])
    return now


def receive_line(node, clock, info):
    clock[0] += .01
    MotionDecisionNode._info_callback(node, 'line')(String(data=json.dumps(info)))


def corner_info(direction='RIGHT', **overrides):
    info = {
        **line_info(), 'corner_preview_confirmed': True,
        'corner_direction': direction, 'corner_start_distance_m': .74,
        # A right corner can have its nearest visible point on the left.
        'center_points_px': [[550, 700], [950, 400]], 'image_width': 1280,
    }
    info.update(overrides)
    return info


def start_line(node):
    command = node.publish_vision(line=line_info())[-1]
    node.send_status(command['action'], command['command_id'], 'RUNNING')
    return command


@pytest.mark.parametrize('phase', ['AUTO', 'LINE_TRACK'])
@pytest.mark.parametrize('direction', ['LEFT', 'RIGHT'])
@pytest.mark.parametrize('after_motion', [{'detected': False}, None])
def test_corner_survives_loss_and_waits_for_matching_completion(clock, phase, direction, after_motion):
    node = MissionFlowHarness(phase=phase)
    current = start_line(node)
    receive_line(node, clock, corner_info(direction, ground_projection_valid=False))
    receive_line(node, clock, {'detected': False})
    count = len(node.publisher.messages)
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == count
    node.send_status('STRAIGHT', current['command_id'] + 1, 'SUCCEEDED')
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == count
    node.send_status('STRAIGHT', current['command_id'], 'SUCCEEDED')
    command = node.publish_vision(line=after_motion)[-1]
    assert command['action'] == direction
    assert command['source'] == 'line'
    assert command['reason'] == 'line_corner_remembered_during_motion'
    assert command['source_command']['corner_from_memory'] is True
    assert node.pending_line_corner is None
    assert MotionCommandBridgeNode.ACTION_TO_MOTION_ID[direction] == f'line_recovery_{direction.lower()}_4'
    assert node.publish_vision(line={'detected': False}) == []


def test_corner_is_kept_through_recovery_and_fresh_negative_frame(clock):
    node = MissionFlowHarness(phase='AUTO')
    current = node.publish_vision(line=line_info(heading=-20., offset=-.3))[-1]
    assert current['action'] == 'RECOVER_LEFT_TURN_LEFT_4'
    node.send_status(current['action'], current['command_id'], 'RUNNING')
    receive_line(node, clock, corner_info('RIGHT'))
    release_general(node, current)
    # Recovery retains its existing requirement for post-motion Vision.
    receive_line(node, clock, {'detected': False})
    assert node.publish_vision()[-1]['action'] == 'RIGHT'


@pytest.mark.parametrize('overrides', [
    {'corner_preview_confirmed': False}, {'corner_preview_confirmed': 'true'},
    {'detected': False}, {'corner_direction': 'UNKNOWN'},
    {'heading_quality': .1},
])
def test_unconfirmed_or_low_quality_corner_does_not_override_search(clock, overrides):
    node = MissionFlowHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info(**overrides))
    assert getattr(node, 'pending_line_corner', None) is None
    release_general(node, current)
    command = node.publish_vision(line={'detected': False})[-1]
    assert command['action'] not in {'LEFT', 'RIGHT'}


def test_first_confirmed_direction_survives_conflicting_preview(clock):
    node = MissionFlowHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info('RIGHT'))
    receive_line(node, clock, corner_info('LEFT'))
    release_general(node, current)
    assert node.publish_vision(line={'detected': False})[-1]['action'] == 'RIGHT'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_unsuccessful_current_motion_discards_corner(clock, status):
    node = MissionFlowHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info())
    node.send_status(current['action'], current['command_id'], status)
    assert node.pending_line_corner is None
    assert node.line_corner_rearm_required


def test_other_mission_takes_priority_and_clears_corner(clock):
    node = MissionFlowHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, current)
    ball = dict(detected=True, confidence=.9, depth_valid=True, depth_age_sec=.05,
                distance_m=1., depth_m=1., bearing_deg=0., offset_x_norm=0.)
    command = node.publish_vision(ball=ball, line={'detected': False})[-1]
    assert command['source'] == 'ball'
    assert node.pending_line_corner is None


def test_phase_override_discards_corner(clock):
    node = MissionFlowHarness()
    start_line(node)
    receive_line(node, clock, corner_info())
    node.send_phase('BALL_APPROACH')
    assert node.pending_line_corner is None


def test_memory_blocks_new_prequeue_but_preserves_normal_prequeue_policy(clock):
    node = MissionFlowHarness()
    start_line(node)
    assert MotionDecisionNode._line_only_prequeue_allowed(node, {})
    receive_line(node, clock, corner_info())
    assert not MotionDecisionNode._line_only_prequeue_allowed(node, {})
    count = len(node.publisher.messages)
    MotionDecisionNode._publish_decision(node, node.last_selected_decision, queue_while_locked=True)
    assert len(node.publisher.messages) == count


def test_already_published_straight_is_finished_before_remembered_corner(clock):
    node = MissionFlowHarness()
    first = start_line(node)
    MotionDecisionNode._publish_decision(node, node.last_selected_decision, queue_while_locked=True)
    queued = node.publisher.messages[-1]
    assert queued['command_id'] != first['command_id']
    receive_line(node, clock, corner_info())
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED')
    count = len(node.publisher.messages)
    MotionDecisionNode._publish_decision(node)
    assert len(node.publisher.messages) == count
    node.send_status(queued['action'], queued['command_id'], 'RUNNING')
    receive_line(node, clock, {'detected': False})
    node.send_status(queued['action'], queued['command_id'], 'SUCCEEDED')
    assert node.publish_vision(line={'detected': False})[-1]['action'] == 'RIGHT'


def test_same_corner_is_not_remembered_again_until_visible_clear(clock):
    node = MissionFlowHarness()
    first = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, first)
    turn = node.publish_vision(line={'detected': False})[-1]
    node.send_status(turn['action'], turn['command_id'], 'RUNNING')
    receive_line(node, clock, corner_info())
    assert node.pending_line_corner is None
    release_general(node, turn)
    assert node.planner.last_line_seen_direction is None
    receive_line(node, clock, corner_info())
    next_straight = node.publish_vision()[-1]
    assert next_straight['action'] == 'STRAIGHT'
    receive_line(node, clock, {'detected': False})
    receive_line(node, clock, corner_info())
    assert node.pending_line_corner is None
    receive_line(node, clock, {**line_info(), 'corner_preview_confirmed': False})
    receive_line(node, clock, corner_info('LEFT'))
    assert node.pending_line_corner['corner_direction'] == 'LEFT'


def test_remembered_corner_retains_turn_settle_and_is_consumed_only_on_publish(clock):
    node = MissionFlowHarness()
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = 1.
    current = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, current)
    assert node.publish_vision(line={'detected': False}) == []
    assert node.pending_line_corner is not None
    clock[0] += .9
    assert node.publish_vision(line={'detected': False}) == []
    clock[0] += .11
    assert node.publish_vision(line={'detected': False})[-1]['action'] == 'RIGHT'
    assert node.pending_line_corner is None


def test_remembered_corner_cannot_bypass_executor_fault(clock):
    node = MissionFlowHarness()
    current = start_line(node)
    receive_line(node, clock, corner_info())
    release_general(node, current)
    node.safety_interlock.observe_critical_fault(
        error_code='SDK_HARDWARE_NOT_READY', message='test fault', source='executor',
        action='STRAIGHT', command_id=current['command_id'], event_id=None,
    )
    assert node.publish_vision(line={'detected': False}) == []


@pytest.mark.parametrize('phase', ['POST_BALL_LINE_ALIGN', 'POST_SHOT_LINE_ALIGN', 'LINE_TRACK_AFTER_PICKUP'])
def test_route_specific_alignment_does_not_remember_normal_corners(clock, phase):
    node = MissionFlowHarness()
    start_line(node)
    node.phase_manager.set_phase(phase)
    receive_line(node, clock, corner_info())
    assert getattr(node, 'pending_line_corner', None) is None
