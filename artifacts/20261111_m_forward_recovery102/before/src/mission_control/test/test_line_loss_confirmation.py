"""Replay transient line loss without publishing a premature search or queue."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock, fresh
from test_line_turn_prequeue import start, late
from test_mission_phase_flow import line_info
from test_motion_command_bridge import decoded_messages
from test_pickup_continuous_walk import forward_status
from test_wait_refresh_and_fine_settle import LiveInputHarness, observe


MISSING = {'detected': False, 'raw_detected': False, 'confirmation_confirmed': False}
PENDING = {**MISSING, 'raw_detected': True}


def idle(clock):
    node = LiveInputHarness(phase='LINE_TRACK')
    node.planner.last_line_seen_direction = 'RIGHT'
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = Node.LINE_TURN_PRE_MOTION_SETTLE_SEC
    node.motion_stationary_since = clock[0] - .6
    return node


def publish(node):
    Node._publish_decision(node)
    return node.last_selected_decision


def requests(node):
    return [m for m in node.publisher.messages if m['valid']]


@pytest.mark.parametrize('count', [1, 2])
def test_transient_loss_waits_and_tracking_resumes_without_search(clock, count):
    node = idle(clock)
    for _ in range(count):
        fresh(node, clock, 'line', MISSING)
    for _ in range(10):
        publish(node)
    assert len(node.line_loss_stamps) == count
    assert not requests(node)
    assert node.last_selected_decision.reason == 'line_loss_waiting_for_fresh_misses'
    fresh(node, clock, 'line', line_info())
    assert not node.line_loss_stamps
    assert requests(node)[-1]['action'] == 'STRAIGHT'


def test_reconfirmation_breaks_the_loss_sequence(clock):
    node = idle(clock)
    for _ in range(2):
        fresh(node, clock, 'line', MISSING)
    for _ in range(5):
        fresh(node, clock, 'line', PENDING)
    assert not node.line_loss_stamps and not requests(node)
    for _ in range(2):
        fresh(node, clock, 'line', MISSING)
    assert not requests(node)
    fresh(node, clock, 'line', MISSING)
    assert requests(node)[-1]['action'] == 'LINE_LOST_TURN_RIGHT_5'
    assert requests(node)[-1]['source_command']['line_loss_frames'] == 3


@pytest.mark.parametrize('bad', ['missing', 'future', 'old', 'duplicate', 'out_of_order', 'raw_unknown'])
def test_invalid_or_repeated_frames_cannot_complete_loss(clock, bad):
    node = idle(clock)
    fresh(node, clock, 'line', MISSING)
    stamp = node.line_loss_stamps[-1]
    for _ in range(4):
        current = round((clock[0] + .01) * 1e9)
        packet = dict(MISSING)
        packet['rgb_stamp_ns'] = {
            'missing': None, 'future': current + 1_000_000_000,
            'old': current - 1_000_000_000, 'duplicate': stamp,
            'out_of_order': stamp - 1, 'raw_unknown': current,
        }[bad]
        if bad == 'raw_unknown':
            packet.pop('raw_detected')
        observe(node, clock, 'line', packet)
    assert not requests(node)
    assert len(node.line_loss_stamps) < 3
    for _ in range(3):
        fresh(node, clock, 'line', MISSING)
    assert requests(node)[-1]['action'] == 'LINE_LOST_TURN_RIGHT_5'


def test_expired_or_absent_input_does_not_authorize_search(clock):
    node = idle(clock)
    for _ in range(3):
        fresh(node, clock, 'line', MISSING, plan=False)
    clock[0] += node.timeouts['line'] + .01
    for _ in range(3):
        publish(node)
    assert not requests(node)
    for _ in range(2):
        fresh(node, clock, 'line', MISSING)
    assert len(node.line_loss_stamps) == 2 and not requests(node)
    fresh(node, clock, 'line', MISSING)
    assert requests(node)[-1]['action'].startswith('LINE_LOST_TURN_')


@pytest.mark.parametrize('delay', [0., .2, .35])
@pytest.mark.parametrize('action', ['STRAIGHT', 'RECOVER_RIGHT_TURN_LEFT_4',
                                   'LINE_OFFSET_TURN_LEFT_2', 'LINE_LOST_TURN_LEFT_3'])
def test_motion_misses_cannot_prequeue_and_new_stationary_frames_release_once(clock, delay, action):
    node, bridge, request = start(clock, action)
    node.LINE_TURN_PRE_MOTION_SETTLE_SEC = Node.LINE_TURN_PRE_MOTION_SETTLE_SEC
    late(node, clock, MISSING, 6)
    assert not node.line_loss_stamps
    assert node.queued_general_command_id is None
    assert len(requests(node)) == 1
    clock[0] = max(clock[0], node.active_line_motion_started_at + node.active_line_motion_duration_sec)
    forward_status(node, bridge, request, 'SUCCEEDED')
    completed = clock[0]
    assert not node.line_loss_stamps
    # Delayed captures from the walk cannot count after completion.
    observe(node, clock, 'line', {**MISSING, 'rgb_stamp_ns': round((completed - .01) * 1e9)})
    assert not node.line_loss_stamps
    # Three new captures arrive during the existing half-second rest.
    for i in range(3):
        clock[0] = completed + .04 * (i + 1) + delay
        Node._info_callback(node, 'line')(String(data=json.dumps({
            **MISSING, 'rgb_stamp_ns': round((completed + .04 * (i + 1)) * 1e9),
        })))
        publish(node)
    assert len(node.line_loss_stamps) == 3
    clock[0] = completed + .499
    publish(node)
    assert len(requests(node)) == 1
    clock[0] = completed + .501
    publish(node)
    # A normal completion can invalidate the previous input at its dwell boundary.
    if len(requests(node)) == 1:
        fresh(node, clock, 'line', MISSING)
    command = requests(node)[-1]
    expected = 'LINE_LOST_TURN_LEFT_1' if action.startswith('LINE_LOST_') else 'LINE_LOST_TURN_LEFT_3'
    assert command['action'] == expected
    assert clock[0] - completed < .55
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    if bridge.transition_dwell_until is not None:
        clock[0] = bridge.transition_dwell_until
        bridge._check_atomic_dwell()
    search_request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert search_request['action'] == command['action']
    forward_status(node, bridge, search_request, 'RUNNING')
    for _ in range(5):
        fresh(node, clock, 'line', MISSING)
    assert len(requests(node)) == 2 and not node.line_loss_stamps
    assert node.queued_general_command_id is None
    forward_status(node, bridge, search_request, 'SUCCEEDED')
    clock[0] += .6
    publish(node)
    fresh(node, clock, 'line', line_info())
    assert requests(node)[-1]['action'] == 'STRAIGHT'


def test_reacquisition_during_settle_cancels_search_without_delaying_forward(clock):
    node = idle(clock)
    node.motion_stationary_since = clock[0]
    for _ in range(3):
        fresh(node, clock, 'line', MISSING)
    assert not requests(node)
    fresh(node, clock, 'line', line_info())
    assert requests(node)[-1]['action'] == 'STRAIGHT'
    assert not node.line_loss_stamps
    assert node.pre_motion_settle_started_at is None


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_unsuccessful_motion_clears_loss_evidence(clock, status):
    node, bridge, request = start(clock)
    late(node, clock, MISSING, 3)
    forward_status(node, bridge, request, status, error_code='TEST_FAILURE')
    assert not node.line_loss_stamps
    assert node.line_loss_last_stamp is None
    clock[0] += .6
    publish(node)
    assert not any(m['action'].startswith('LINE_LOST_') for m in requests(node))


def test_clock_rollback_clears_old_loss_boundary_and_can_recover(clock):
    node = idle(clock)
    Node._record_stationary_status(node, {'status': 'SUCCEEDED'}, clock[0])
    for _ in range(2):
        fresh(node, clock, 'line', MISSING, plan=False)
    clock[0] -= 2.
    Node._invalidate_clock_observations(node, round(clock[0] * 1e9))
    assert not node.line_loss_stamps and node.line_loss_last_stamp is None
    # The existing clock guard independently confirms the new clock domain.
    node.vision_time_guard = None
    node.motion_stationary_since = clock[0] - .6
    for _ in range(3):
        fresh(node, clock, 'line', MISSING)
    assert requests(node)[-1]['action'].startswith('LINE_LOST_TURN_')
