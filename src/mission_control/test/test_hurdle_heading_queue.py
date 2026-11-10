"""Verify line-heading changes across real hurdle reservation callbacks."""

import json

import pytest
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from test_continuous_approach import clock, fresh
from test_hurdle_positioning import hurdle, line_info
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status
from test_wait_refresh_and_fine_settle import LiveInputHarness


def feed(node, clock, heading, x=500):
    fresh(node, clock, 'line', line_info(x=x, heading=heading), plan=False)
    fresh(node, clock, 'hurdle', hurdle(.9, bbox=[400, 300, 1000, 719]), plan=False)


@pytest.mark.parametrize('heading', [-30, 0, 30])
@pytest.mark.parametrize('x', [500, 1100])
def test_forward_reservation_uses_heading_even_with_opposite_or_missing_intersection(clock, heading, x):
    node, bridge = LiveInputHarness(phase='HURDLE_APPROACH'), FakeBridge()
    feed(node, clock, 0, x)
    MotionDecisionNode._publish_decision(node)
    first = node.publisher.messages[-1]
    assert first['valid'] and first['action'] == 'STRAIGHT'
    bridge.navigation_command_callback(String(data=json.dumps(first)))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    node.send_status('STRAIGHT', first['command_id'], 'RUNNING', 'line_forward_4')
    clock[0] = node.approach_walk_started_at + node.approach_walk_duration_sec - .25
    for _ in range(3):
        feed(node, clock, heading, x)

    if heading == 0:
        queued = node.publisher.messages[-1]
        assert queued['action'] == 'STRAIGHT'
        assert queued['command_id'] != first['command_id']
        assert node.queued_general_command_id == queued['command_id']
        bridge.navigation_command_callback(String(data=json.dumps(queued)))
        bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
        MotionDecisionNode._motion_status_callback(node, bridge.motion_status_publisher.messages[-1])
        assert bridge.transition_dwell_until is None
        next_request = decoded_messages(bridge.executor_request_publisher)[-1]
        bridge.executor_status_callback(executor_status(**next_request, status='RUNNING'))
        MotionDecisionNode._motion_status_callback(node, bridge.motion_status_publisher.messages[-1])
        assert node.general_motion_gate.active_command_id == queued['command_id']
        return

    assert getattr(node, 'queued_general_command_id', None) is None
    assert len([message for message in node.publisher.messages if message['valid']]) == 1
    node.send_status('STRAIGHT', first['command_id'], 'SUCCEEDED', 'line_forward_4')
    clock[0] += .5
    feed(node, clock, heading, x)
    MotionDecisionNode._publish_decision(node)
    turn = node.publisher.messages[-1]
    assert turn['valid'] and turn['action'] == ('ALIGN_LEFT' if heading < 0 else 'ALIGN_RIGHT')
    assert turn['source_command']['turn_count'] == (2 if heading < 0 else 3)
    assert getattr(node, 'queued_general_command_id', None) is None
