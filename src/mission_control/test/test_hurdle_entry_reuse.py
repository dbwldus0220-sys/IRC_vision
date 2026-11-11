"""Replay first hurdle entry at straight/turn completion without robot hardware."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from step.hurdle_camera_pose import HurdleCameraPose
from test_boundary_decision_reservation import BoundaryHarness
from test_continuous_approach import clock
from test_hurdle_positioning import HurdleHarness, hurdle


class EntryHarness(HurdleHarness):
    boundary_decision_enabled = True
    FINE_FORWARD_PRE_MOTION_SETTLE_SEC = Node.FINE_FORWARD_PRE_MOTION_SETTLE_SEC
    create_timer = BoundaryHarness.create_timer
    destroy_timer = BoundaryHarness.destroy_timer


def start_entry(clock, source='line', action='STRAIGHT_1'):
    node = EntryHarness()
    node.hurdle_camera_pose = HurdleCameraPose()
    node.command_id = 1
    node.boundary_decision_pending = True
    node.active_general_source = source
    node.general_motion_gate.on_command_published(action, 1)
    node.send_status(action, 1, 'RUNNING')
    Node._executor_heartbeat_callback(node, String(data=json.dumps({
        'sequence': 1, 'active': True, 'auto_ready': True,
        'hurdle_head_override_active': False,
        'ball_head_override_active': False, 'goal_head_override_active': False,
    })))
    clock[0] += .1
    stamp = round(clock[0] * 1e9)
    node.receive(hurdle(.60, stamp={
        'sec': stamp // 1_000_000_000, 'nanosec': stamp % 1_000_000_000,
    }))
    assert node.hurdle_positioning_entry_pending
    assert not node.publisher.messages
    clock[0] += .05
    return node


@pytest.mark.parametrize('source', ['line', 'hurdle'])
@pytest.mark.parametrize('action', ['STRAIGHT', 'STRAIGHT_1'])
def test_straight_entry_dispatches_once_on_completion_without_new_image(clock, source, action):
    node = start_entry(clock, source, action)
    node.send_status(action, 1, 'SUCCEEDED')
    commands = [c for c in node.publisher.messages if c['valid']]
    assert len(commands) == 1
    assert commands[0]['source'] == 'hurdle'
    assert commands[0]['phase'] == 'HURDLE_POSITIONING'
    assert commands[0]['action'] == 'STRAIGHT_1'
    assert node.latest_info['hurdle'] is not None
    node.send_status(action, 1, 'SUCCEEDED')
    assert len([c for c in node.publisher.messages if c['valid']]) == 1


@pytest.mark.parametrize('source', ['line', 'hurdle'])
@pytest.mark.parametrize('action', ['RECOVER_LEFT_TURN_LEFT_4', 'RECOVER_RIGHT_TURN_RIGHT_4'])
def test_turn_entry_still_discards_observation(clock, source, action):
    node = start_entry(clock, source, action)
    node.send_status(action, 1, 'SUCCEEDED')
    assert node.latest_info['hurdle'] is None
    assert not any(c['valid'] for c in node.publisher.messages)


@pytest.mark.parametrize('invalid', [
    'expired_capture', 'expired_receipt', 'future', 'missing_stamp',
    'unconfirmed', 'depth', 'lost', 'angle', 'needs_turn', 'camera_moving',
    'camera_missing', 'camera_stale', 'before_camera_ready', 'head_request',
])
def test_invalid_entry_observation_still_waits(clock, invalid):
    node = start_entry(clock)
    info = node.latest_info['hurdle']
    if invalid in {'expired_capture', 'future', 'before_camera_ready'}:
        t = clock[0] + (1. if invalid == 'future' else -.6 if invalid == 'expired_capture' else -.2)
        stamp = round(t * 1e9)
        info['stamp'] = {'sec': stamp // 1_000_000_000, 'nanosec': stamp % 1_000_000_000}
    elif invalid == 'expired_receipt':
        node.latest_time['hurdle'] = clock[0] - .6
    elif invalid == 'missing_stamp':
        info.pop('stamp')
    elif invalid == 'camera_missing':
        node.hurdle_camera_pose = None
    elif invalid == 'camera_stale':
        node.hurdle_camera_pose.heartbeat_at = clock[0] - 1.1
    elif invalid == 'camera_moving':
        node.hurdle_camera_pose.pose = None
    else:
        info.update({
            'unconfirmed': {'confirmation_confirmed': False},
            'depth': {'depth_valid': False}, 'lost': {'detected': False},
            'angle': {'hurdle_angle_45_deg': None},
            'needs_turn': {'hurdle_angle_45_deg': 30.},
            'head_request': {'head_down_requested': True},
        }[invalid])
    node.send_status('STRAIGHT_1', 1, 'SUCCEEDED')
    assert node.latest_info['hurdle'] is None
    assert not any(c['valid'] for c in node.publisher.messages)


@pytest.mark.parametrize('status', ['FAILED', 'CANCELLED', 'REJECTED'])
def test_failed_gait_cannot_reuse_entry_input(clock, status):
    node = start_entry(clock)
    node.send_status('STRAIGHT_1', 1, status)
    assert not node.general_motion_gate.has_required_fresh_vision()
    assert not any(c['valid'] for c in node.publisher.messages)
