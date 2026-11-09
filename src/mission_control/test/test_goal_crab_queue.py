"""Replay same-direction Goal crab reservations and their completion boundaries."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock, fresh
from test_motion_decision_planner import goal_info
from test_motion_command_bridge import FakeBridge, decoded_messages, navigation_message
from test_pickup_continuous_walk import forward_status, finish_rest
from test_reservation_expansion import deliver
from test_wait_refresh_and_fine_settle import LiveInputHarness
from test_wait_refresh_and_fine_settle import observe


def sample(side='left', **changes):
    offset = -230 if side == 'left' else 230
    return goal_info(depth_m=.450, offset_x_px=offset, offset_x_norm=offset / 640,
                     bearing_deg=-7.9 if side == 'left' else 14.2, **changes)


def start(clock, side):
    node = LiveInputHarness(phase='GOAL_APPROACH')
    node.queued_general_command_id = None
    node.queued_general_action = None
    node.queued_general_source = None
    bridge = FakeBridge(advance_transition_dwells=False)
    # Already at the shared crab posture: initial Goal observation rest is preserved.
    bridge.last_physical_motion_id = f'goal_camera_90_crab_{side}'
    fresh(node, clock, 'goal', sample(side))
    first = node.publisher.messages[-1]
    assert first['action'] == f'GOAL_CAMERA90_CRAB_{side.upper()}'
    deliver(node, bridge, first)
    assert getattr(node, 'goal_forward_started_at', None) is None
    clock[0] = bridge.active_dwell_until
    bridge._check_atomic_dwell()
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, request, 'RUNNING')
    assert node.goal_forward_started_at == clock[0]
    return node, bridge, request


def images(node, clock, side, fractions=(.61, .66, .76), info=None):
    started = node.goal_forward_started_at
    duration = Node.GOAL_CRAB_DURATION_SEC[node.general_motion_gate.active_action]
    for fraction in fractions:
        clock[0] = started + duration * fraction
        fresh(node, clock, 'goal', sample(side) if info is None else info, plan=False)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_same_crab_reservation_preserves_half_second_rest(clock, side):
    node, bridge, first = start(clock, side)
    images(node, clock, side, (.1, .2, .3))
    assert node.queued_general_command_id is None
    images(node, clock, side, (.61, .64, .67))
    assert node.queued_general_command_id is None
    images(node, clock, side, (.76, .80, .84))
    second = node.publisher.messages[-1]
    assert second['source_command']['goal_crab_prequeue']
    assert second['action'] == first['action']
    assert node.queued_general_command_id == second['command_id']
    deliver(node, bridge, second)
    assert len(bridge.executor_request_publisher.messages) == 1
    assert bridge.queued_request_deferred
    assert bridge.transition_dwell_until is None
    count = len(node.publisher.messages)
    images(node, clock, side, (.88, .90, .92))
    assert len(node.publisher.messages) == count
    forward_status(node, bridge, first, 'SUCCEEDED')
    assert bridge.transition_dwell_until == pytest.approx(clock[0] + .5)
    finish_rest(bridge, clock)
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['command_id'] == second['command_id']
    assert request['motion_id'] == first['motion_id']
    assert bridge.active_request_id == request['request_id']
    assert bridge.active_dwell_until is None
    assert getattr(node, 'correction_post_motion_dwell_until', None) is None
    assert node.latest_info['goal'] is not None
    forward_status(node, bridge, request, 'RUNNING')
    assert node.general_motion_gate.active_command_id == second['command_id']
    assert node.goal_forward_started_at == clock[0]
    # A third gait needs new images from the newly running gait.
    assert node.goal_forward_capture_count == 0
    images(node, clock, side)
    assert node.queued_general_command_id is not None


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('change', ['opposite', 'shot', 'forward', 'lost', 'unconfirmed', 'depth'])
def test_other_actions_or_invalid_images_never_reserve_crab(clock, side, change):
    node, _, _ = start(clock, side)
    info = sample(side)
    if change == 'opposite':
        info = sample('right' if side == 'left' else 'left')
    elif change == 'shot':
        info = goal_info(depth_m=.510, offset_x_px=0, offset_x_norm=0, bearing_deg=0)
    elif change == 'forward':
        info = goal_info(depth_m=1.5, offset_x_px=0, offset_x_norm=0, bearing_deg=0)
    else:
        info[{'lost': 'detected', 'unconfirmed': 'confirmation_confirmed', 'depth': 'depth_valid'}[change]] = False
    images(node, clock, side, info=info)
    assert node.queued_general_command_id is None
    assert not node.planner.goal_terminal_requested


@pytest.mark.parametrize('status', ['FAILED', 'CANCELLED', 'TIMEOUT'])
def test_current_failure_discards_reserved_crab(clock, status):
    node, bridge, first = start(clock, 'left')
    images(node, clock, 'left')
    deliver(node, bridge, node.publisher.messages[-1])
    second = decoded_messages(bridge.executor_request_publisher)[-1]
    forward_status(node, bridge, second, 'QUEUED')
    forward_status(node, bridge, first, status)
    assert node.queued_general_command_id is None
    assert bridge.queued_request_id is None
    assert not bridge.motion_in_progress
    assert node.latest_info['goal'] is None


@pytest.mark.parametrize('bad', ['early_capture', 'duplicate', 'future', 'expired'])
def test_reservation_requires_three_distinct_fresh_late_images(clock, bad):
    node, _, _ = start(clock, 'left')
    started = node.goal_forward_started_at
    duration = Node.GOAL_CRAB_DURATION_SEC['GOAL_CAMERA90_CRAB_LEFT']
    clock[0] = started + duration * .8
    stamp = round((started + duration * .75) * 1e9)
    if bad == 'early_capture':
        stamp = round((started + duration * .2) * 1e9)
    elif bad == 'future':
        stamp = round((clock[0] + 2) * 1e9)
    elif bad == 'expired':
        stamp = round((clock[0] - 5) * 1e9)
    for i in range(3):
        observe(node, clock, 'goal', {**sample(), 'rgb_stamp_ns': stamp + (0 if bad == 'duplicate' else i)}, plan=False)
    assert node.queued_general_command_id is None


def test_reserved_request_rejection_clears_reservation_but_keeps_current_crab(clock):
    node, bridge, first = start(clock, 'left')
    images(node, clock, 'left')
    deliver(node, bridge, node.publisher.messages[-1])
    second = {**first, 'command_id': bridge.queued_command_id,
              'request_id': bridge.queued_request_id}
    forward_status(node, bridge, second, 'REJECTED')
    assert node.queued_general_command_id is None
    assert bridge.queued_request_id is None
    assert bridge.active_request_id == first['request_id']
    assert node.general_motion_gate.active_command_id == first['command_id']


def test_bridge_still_limits_reservations_to_one(clock):
    node, bridge, _ = start(clock, 'left')
    images(node, clock, 'left')
    command = json.loads(json.dumps(node.publisher.messages[-1]))
    deliver(node, bridge, command)
    count = len(bridge.executor_request_publisher.messages)
    command['command_id'] += 1
    deliver(node, bridge, command)
    assert len(bridge.executor_request_publisher.messages) == count
    assert decoded_messages(bridge.motion_status_publisher)[-1]['error_code'] == 'QUEUE_FULL'


@pytest.mark.parametrize('change', ['opposite', 'parent', 'frames', 'preparation'])
def test_bridge_rejects_invalid_same_gait_exception(clock, change):
    node, bridge, first = start(clock, 'left')
    images(node, clock, 'left')
    command = json.loads(json.dumps(node.publisher.messages[-1]))
    if change == 'opposite':
        command['action'] = 'GOAL_CAMERA90_CRAB_RIGHT'
    elif change == 'parent':
        command['source_command']['goal_crab_running_command_id'] = -1
    elif change == 'frames':
        command['source_command']['goal_crab_queue_frame_count'] = 2
    else:
        bridge.active_motion_id = 'goal_fine_to_crab_left_90'
    count = len(bridge.executor_request_publisher.messages)
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert len(bridge.executor_request_publisher.messages) == count
    assert decoded_messages(bridge.motion_status_publisher)[-1]['error_code'] == 'INVALID_GOAL_CRAB_RESERVATION'


@pytest.mark.parametrize('side', ['left', 'right'])
def test_goal_direction_change_has_one_half_second_rest(clock, side):
    bridge = FakeBridge(advance_transition_dwells=False)
    other = 'right' if side == 'left' else 'left'
    bridge.last_physical_motion_id = f'goal_camera_90_crab_{side}'
    bridge.stationary_since = clock[0]
    bridge.navigation_command_callback(navigation_message(action=f'GOAL_CAMERA90_CRAB_{other.upper()}'))
    started = clock[0]
    clock[0] = started + .499
    bridge._check_atomic_dwell()
    assert not decoded_messages(bridge.executor_request_publisher)
    clock[0] = started + .5
    bridge._check_atomic_dwell()
    assert [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)] == [f'goal_camera_90_crab_{other}']
    assert bridge.transition_dwell_until is None
