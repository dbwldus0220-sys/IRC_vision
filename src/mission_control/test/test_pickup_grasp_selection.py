"""Post-retreat vision selects one grasp without releasing mission ownership."""

import pytest

from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.mission_phase_manager import MissionPhaseManager
from test_motion_command_bridge import (
    FakeBridge, continue_pickup_after_fine_alignment, enter_pickup_fine_alignment,
    executor_status, navigation_message, decoded_messages,
)
from test_mission_phase_flow import MissionFlowHarness, pickup_ready_ball, publish_special
from mission_control.motion_decision_node import MotionDecisionNode
from mission_control.pickup_grasp_selection import pickup_grasp_motion_id


@pytest.mark.parametrize('pixels,expected', [
    (0, 'pickup_400'), (319.99, 'pickup_400'), (320, 'pickup_400'),
    (320.01, 'pickup_500'), (400, 'pickup_500'), (450, 'pickup_500'),
    (500, 'pickup_500'),
])
def test_grasp_selection_uses_320_pixel_boundary(pixels, expected):
    assert pickup_grasp_motion_id(pixels) == expected


@pytest.mark.parametrize('pixels,expected', [(0, 'pickup_400'), (300, 'pickup_400'),
    (319.99, 'pickup_400'), (320, 'pickup_400'), (320.01, 'pickup_500'), (500, 'pickup_500')])
def test_post_retreat_selects_grasp_and_keeps_check_pose(pixels, expected):
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    continue_pickup_after_fine_alignment(bridge)
    count = len(bridge.executor_request_publisher.messages)
    payload = dict(action='BALL_PICKUP_POST_BACKWARD_ALIGN_CONTINUE',
                   command_id=9000, event_id=None,
                   active_special_command_id=bridge.active_command_id,
                   active_special_event_id=bridge.active_event_id,
                   source_command={'bottom_distance_px': pixels})
    # Selection cannot jump ahead of the retreat or its settle time.
    bridge.navigation_command_callback(navigation_message(**payload))
    assert len(bridge.executor_request_publisher.messages) == count
    bridge.executor_status_callback(executor_status(
        status='SUCCEEDED', motion_id='pickup_pre_backward_camera_down'))
    deadline = bridge.active_dwell_until
    bridge._check_atomic_dwell(deadline - .001)
    assert len(bridge.executor_request_publisher.messages) == count
    bridge._check_atomic_dwell(deadline)
    assert bridge.pickup_post_backward_align_waiting
    assert len(bridge.executor_request_publisher.messages) == count
    bridge.navigation_command_callback(navigation_message(**payload))
    assert bridge.active_motion_id == expected
    assert bridge.active_action == 'PICKUP_NOW'
    assert len(bridge.executor_request_publisher.messages) == count + 1
    bridge.navigation_command_callback(navigation_message(**payload))
    assert len(bridge.executor_request_publisher.messages) == count + 1
    bridge.executor_status_callback(executor_status(status='SUCCEEDED', motion_id=expected))
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert bridge.active_motion_id == 'pickup_grasp_check_pose'
    bridge.executor_status_callback(executor_status(
        status='SUCCEEDED', motion_id='pickup_grasp_check_pose'))
    statuses = decoded_messages(bridge.motion_status_publisher)
    assert statuses[-1]['verification_window_complete'] is False
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    assert bridge.active_motion_id == 'pickup_first_backward_turn_right'
    assert any(s.get('verification_window_complete') is True
               for s in decoded_messages(bridge.motion_status_publisher))


@pytest.mark.parametrize('pixels', [None, -1, True, '450', float('nan'), float('inf')])
def test_invalid_pixels_wait_in_planner_and_cannot_resume_bridge(pixels):
    planner = MotionDecisionPlanner()
    assert planner.plan_ball_pickup_post_backward_alignment({
        'detected': True, 'confidence': .9, 'bottom_distance_px': pixels,
    }).action == 'WAIT'
    bridge = FakeBridge()
    enter_pickup_fine_alignment(bridge)
    continue_pickup_after_fine_alignment(bridge)
    bridge.executor_status_callback(executor_status(
        status='SUCCEEDED', motion_id='pickup_pre_backward_camera_down'))
    bridge._check_atomic_dwell(bridge.active_dwell_until)
    count = len(bridge.executor_request_publisher.messages)
    bridge.navigation_command_callback(navigation_message(
        action='BALL_PICKUP_POST_BACKWARD_ALIGN_CONTINUE', command_id=9000, event_id=None,
        source_command={'bottom_distance_px': pixels, 'pickup_grasp_motion_id': 'pickup_400'},
        active_special_command_id=bridge.active_command_id,
        active_special_event_id=bridge.active_event_id,
    ))
    assert bridge.pickup_post_backward_align_waiting
    assert len(bridge.executor_request_publisher.messages) == count
    assert decoded_messages(bridge.motion_status_publisher)[-1]['error_code'] == 'INVALID_PICKUP_GRASP_SELECTION'


@pytest.mark.parametrize('pixels,expected', [(320, 'pickup_400'), (321, 'pickup_500')])
def test_node_discards_pre_retreat_frame_then_selects_from_fresh_pixels(pixels, expected):
    node = MissionFlowHarness(phase='BALL_APPROACH')
    command = publish_special(node, 'ball', pickup_ready_ball(), 'PICKUP_NOW')
    node.send_status('PICKUP_NOW', command['command_id'], 'RUNNING',
                     motion_id=MotionDecisionNode.PICKUP_POST_BACKWARD_ALIGN_MARKER)
    assert node.observations['ball'] is None
    assert node.pickup_post_backward_align_waiting
    assert node.publish_vision()[-1]['action'] == 'WAIT'
    decision = node.publish_vision(ball={**pickup_ready_ball(), 'bottom_distance_px': pixels})[-1]
    assert decision['action'] == 'BALL_PICKUP_POST_BACKWARD_ALIGN_CONTINUE'
    assert decision['source_command']['pickup_grasp_motion_id'] == expected
    assert decision['active_special_command_id'] == command['command_id']
    assert not node.pickup_post_backward_align_waiting


@pytest.mark.parametrize('motion', ['pickup_400', 'pickup_500'])
def test_selected_grasp_counts_once(motion):
    manager = MissionPhaseManager(initial_phase='BALL_APPROACH')
    assert manager.start_special_action('PICKUP_NOW', 1)
    for _ in range(2):
        manager.handle_motion_status('PICKUP_NOW', 1, 'RUNNING', motion_id=motion)
    assert manager.pickups_executed == 1
