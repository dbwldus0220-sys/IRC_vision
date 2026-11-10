"""Ring distance selects a shot before motor-signed waist aiming."""

import pytest

from step.goal_scoring import goal_shot_region, GOAL_SCORE_TARGET_DEPTH_M
from step.goal_navigation_planner import GoalNavigationPlanner
from mission_control.motion_decision_node import MotionDecisionNode
from test_catalog64_goal_prepare import goal_node
from test_mission_phase_flow import score_ready_goal
from test_motion_command_bridge import (
    FakeBridge, navigation_message, decoded_messages, string_message, executor_status,
)
from test_shot_start_posture import finish_sequence
from test_continuous_approach import clock, fresh


def sample(depth=.660, offset=0, angle=-12.345, **changes):
    return {**score_ready_goal(), 'depth_m': .950, 'distance_m': .950,
            'ring_center_depth_m': depth, 'waist_ring_aim_deg': angle,
            'offset_x_px': offset, 'offset_x_norm': offset / 640,
            'bearing_deg': 0, 'camera_fx_px': 900, **changes}


@pytest.mark.parametrize('depth,region', [
    (.450, 1), (.529999, 1), (.530, 2), (.589999, 2),
    (.590, 8), (.629999, 8), (.630, 5), (.659999, 5),
    (.660, 6), (.699999, 6), (.700, 7), (.759999, 7),
])
@pytest.mark.parametrize('offset', [-120, -1, 0, 1, 120])
def test_ring_depth_selects_same_region_regardless_of_pixel_sign(depth, region, offset):
    command = GoalNavigationPlanner().plan(sample(depth, offset))
    assert command.action == 'SHOT' and command.shot_region == region
    assert command.depth_m == depth
    assert command.waist_ring_aim_deg == -12.345
    assert GOAL_SCORE_TARGET_DEPTH_M == .660


@pytest.mark.parametrize('depth', [.449999, .760, float('nan'), float('inf')])
def test_no_region_outside_ring_depth_bands(depth):
    assert goal_shot_region(depth) is None
    assert GoalNavigationPlanner().plan(sample(depth)).action != 'SHOT'


@pytest.mark.parametrize('offset', [-350, -121, 121, 350])
@pytest.mark.parametrize('confirmed', [False, True])
def test_scoring_distance_never_requests_pixel_correction(offset, confirmed):
    command = GoalNavigationPlanner().plan(sample(.660, offset, score_now=confirmed))
    assert command.shot_region == 6
    assert not command.is_centered
    assert command.action == ('SHOT' if confirmed else 'WAIT_SCORE_CONFIRMATION')


@pytest.mark.parametrize('field', ['ring_center_depth_m', 'waist_ring_aim_deg'])
@pytest.mark.parametrize('invalid', [None, True, float('nan'), float('inf')])
def test_missing_ring_measurement_never_falls_back_to_board(field, invalid):
    command = GoalNavigationPlanner().plan(sample(**{field: invalid}))
    assert not command.valid and command.action == 'WAIT'


@pytest.mark.parametrize('depth,region', [(.48, 1), (.55, 2), (.61, 8), (.645, 5), (.68, 6), (.73, 7)])
@pytest.mark.parametrize('angle', [-23.456, 0, 17.891])
def test_bridge_transmits_unmodified_angle_to_selected_motion(depth, region, angle):
    command = GoalNavigationPlanner().plan(sample(depth, angle=angle))
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'goal_fine_prepare_90'
    bridge.navigation_command_callback(navigation_message(
        source='goal', action='SHOT', source_command=command.to_dict()))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == f'goal_shot_region_{region}'
    assert request['waist_ring_aim_deg'] == angle


@pytest.mark.parametrize('changes', [
    {'shot_region': 3}, {'shot_region': 4}, {'shot_region': None},
    {'waist_ring_aim_deg': None}, {'waist_ring_aim_deg': True},
    {'waist_ring_aim_deg': float('nan')}, {'waist_ring_aim_deg': 90},
])
def test_invalid_or_disabled_shot_never_reaches_executor(changes):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        action='SHOT', source_command={'shot_region': 6, 'waist_ring_aim_deg': 5, **changes}))
    assert not bridge.executor_request_publisher.messages
    assert decoded_messages(bridge.motion_status_publisher)[-1]['status'] == 'REJECTED'


def test_preparation_has_no_waist_angle_and_preserves_it_for_actual_shot():
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'goal_camera_90_crab_left'
    bridge.navigation_command_callback(navigation_message(
        action='SHOT', source_command={'shot_region': 6, 'waist_ring_aim_deg': -11.234}))
    assert 'waist_ring_aim_deg' not in decoded_messages(bridge.executor_request_publisher)[-1]
    finish_sequence(bridge)
    requests = decoded_messages(bridge.executor_request_publisher)
    assert requests[-1]['motion_id'] == 'goal_shot_region_6'
    assert requests[-1]['waist_ring_aim_deg'] == -11.234
    assert bridge.goal_ring_shot is None


@pytest.mark.parametrize('status', ['SUCCEEDED', 'FAILED', 'CANCELLED'])
def test_terminal_status_drops_angle_and_next_motion_is_unmodified(status):
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'goal_fine_prepare_90'
    bridge.navigation_command_callback(navigation_message(
        action='SHOT', source_command={'shot_region': 1, 'waist_ring_aim_deg': 14.25}))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    bridge.executor_status_callback(executor_status(**request, status=status))
    assert bridge.goal_ring_shot is None
    bridge.navigation_command_callback(navigation_message(action='STRAIGHT', command_id=8001))
    assert 'waist_ring_aim_deg' not in decoded_messages(bridge.executor_request_publisher)[-1]


def test_busy_shot_cannot_replace_active_angle():
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'goal_fine_prepare_90'
    bridge.navigation_command_callback(navigation_message(
        action='SHOT', source_command={'shot_region': 1, 'waist_ring_aim_deg': 12}))
    owner = bridge.goal_ring_shot
    bridge.navigation_command_callback(navigation_message(
        action='SHOT', command_id=8001, source_command={'shot_region': 6, 'waist_ring_aim_deg': -20}))
    assert len(bridge.executor_request_publisher.messages) == 1
    assert bridge.goal_ring_shot == owner


@pytest.mark.parametrize('offset', [-350, 0, 350])
def test_node_reselects_ring_region_and_angle_after_preparation(goal_node, offset):
    node, clock = goal_node
    bridge = FakeBridge()
    bridge.last_physical_motion_id = 'goal_camera_90_crab_left'
    first = node.publish_vision(goal=sample(.48, offset, angle=18))[-1]
    assert first['action'] == 'GOAL_SHOT_PREPARE'
    bridge.navigation_command_callback(string_message(first))
    finish_sequence(bridge)
    for status in bridge.motion_status_publisher.messages:
        MotionDecisionNode._motion_status_callback(node, status)
    clock[0] = node.goal_shot_observe_until
    shot = node.publish_vision(goal=sample(.68, offset, angle=-13.789))[-1]
    assert shot['action'] == 'SHOT'
    bridge.navigation_command_callback(string_message(shot))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == 'goal_shot_region_6'
    assert request['waist_ring_aim_deg'] == -13.789


@pytest.mark.parametrize('depth,region', [(.48, 1), (.55, 2), (.61, 8), (.645, 5), (.68, 6), (.73, 7)])
@pytest.mark.parametrize('entry_depth,action', [(1.0, 'GOAL_CAMERA_90_FORWARD_2'), (.8, 'GOAL_CAMERA90_FINE_FORWARD_3')])
@pytest.mark.parametrize('shot_index', [1, 2])
@pytest.mark.parametrize('offset', [-350, 350])
def test_ring_region_entry_stops_walk_reservation_and_finishes_shot(
        clock, monkeypatch, depth, region, entry_depth, action, shot_index, offset):
    import test_goal_region_queue_audit as audit
    def ring_sample(depth, confirmed=True):
        return sample(depth, offset, score_now=confirmed)
    monkeypatch.setattr(audit, 'sample', ring_sample)
    node, first, motion, started, duration = audit.start(
        clock, action, sample(entry_depth, score_now=False), shot_index)
    for fraction in (.72, .80, .88):
        clock[0] = started + duration * fraction
        fresh(node, clock, 'goal', sample(depth, offset), plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    assert not node.planner.goal_terminal_requested
    clock[0] = started + duration
    node.send_status(action, first['command_id'], 'SUCCEEDED', motion)
    audit.shoot_after_completion(node, clock, motion, region, depth, shot_index)


@pytest.mark.parametrize('depth,region', [(.48, 1), (.55, 2), (.61, 8), (.645, 5), (.68, 6), (.73, 7)])
@pytest.mark.parametrize('shot_index', [1, 2])
def test_accepted_walk_reservation_finishes_once_before_ring_shot(
        clock, monkeypatch, depth, region, shot_index):
    import test_goal_region_queue_audit as audit
    # Even a large lateral offset must not restart body alignment after the queue.
    monkeypatch.setattr(audit, 'sample', lambda depth: sample(depth, offset=350))
    monkeypatch.setattr(audit, 'ENTRIES', [
        ('GOAL_CAMERA_90_FORWARD_2', sample(1.0, score_now=False)),
    ])
    audit.test_committed_queue_finishes_once_then_reaches_correct_shot(
        clock, region, depth, shot_index)
