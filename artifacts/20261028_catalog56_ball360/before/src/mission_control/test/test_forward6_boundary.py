"""Exercise repeated AUTO six-cycle requests through decision and bridge callbacks."""
import json

from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode
from test_line_continuous_missions import clock, observe_walk_end  # noqa: F401
from test_line_corner_memory import CornerHarness
from test_mission_phase_flow import line_info
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status


def test_auto_forward6_requeues_after_each_activation_without_restart(clock, monkeypatch):
    monkeypatch.setattr(MotionDecisionNode, '_current_ros_time_ns', lambda _: round(clock[0] * 1e9))
    node, bridge = CornerHarness(phase='AUTO'), FakeBridge(advance_transition_dwells=False)
    command = node.publish_vision(line={**line_info(), 'rgb_stamp_ns': round(clock[0] * 1e9)})[-1]
    bridge.navigation_command_callback(String(data=json.dumps(command)))

    def status(request, state):
        message = executor_status(status=state, action=request['action'],
                                  command_id=request['command_id'], event_id=request['event_id'],
                                  request_id=request['request_id'], motion_id=request['motion_id'])
        bridge.executor_status_callback(message)
        node.send_status(request['action'], request['command_id'], state)

    current = decoded_messages(bridge.executor_request_publisher)[-1]
    assert current['motion_id'] == 'line_forward_6'
    status(current, 'RUNNING')
    for expected_requests in (2, 3):
        observe_walk_end(node, clock, line_info())
        queued = node.publisher.messages[-1]
        assert queued['action'] == 'STRAIGHT'
        assert queued['command_id'] != current['command_id']
        bridge.navigation_command_callback(String(data=json.dumps(queued)))
        requests = decoded_messages(bridge.executor_request_publisher)
        assert len(requests) == expected_requests
        assert not bridge.queued_request_deferred
        following = requests[-1]
        assert following['motion_id'] == 'line_forward_6'
        status(following, 'QUEUED')
        assert node.general_motion_gate.active_command_id == current['command_id']
        clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec
        status(current, 'SUCCEEDED')
        clock[0] += .005
        status(following, 'RUNNING')
        assert len(bridge.executor_request_publisher.messages) == expected_requests
        assert node.general_motion_gate.active_command_id == following['command_id']
        assert node.active_line_motion_started_at == clock[0]
        assert node.queued_general_command_id is None
        assert bridge.transition_dwell_until is None
        current = following
