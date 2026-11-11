"""Replay long-range scoring across walking reservations and event deadlines."""

import pytest

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_boundary_decision_reservation import BoundaryHarness, executable
from test_continuous_approach import clock, fresh
from test_goal_extended_regions import sample
from test_goal_target_steering import target_sample
from test_mission_phase_flow import mark_next_ball_grabbed
from test_motion_command_bridge import FakeBridge, decoded_messages
from test_reservation_expansion import deliver
from test_shot_start_posture import finish_sequence


REGIONS = [(8, .61), (5, .645), (6, .68), (7, .73)]
ENTRIES = [
    ('GOAL_CAMERA_90_FORWARD_2', target_sample(0, 1.)),
    ('GOAL_CAMERA90_RECOVER_LEFT_4', target_sample(-30, 1.)),
    ('GOAL_CAMERA90_RECOVER_RIGHT_4', target_sample(30, 1.)),
    ('GOAL_CAMERA90_FINE_FORWARD_4', sample(.85, -40, False)),
    ('GOAL_CAMERA90_CRAB_LEFT', sample(.68, -90, False)),
    ('GOAL_CAMERA90_CRAB_RIGHT', sample(.68, 10, False)),
]


def start(clock, action, info, shot_index=1):
    node = BoundaryHarness(phase='GOAL_APPROACH')
    node.phase_manager.shots_completed = node.phase_manager.shots_executed = shot_index - 1
    node.phase_manager.ball_sections_processed = shot_index - 1
    mark_next_ball_grabbed(node)
    fresh(node, clock, 'goal', info)
    first = executable(node)[-1]
    assert first['action'] == action
    motion = FakeBridge.ACTION_TO_MOTION_ID[action]
    node.send_status(action, first['command_id'], 'RUNNING', motion)
    duration = (Node.GOAL_CRAB_DURATION_SEC.get(action)
                or Node.GOAL_FORWARD_DURATION_SEC[action])
    return node, first, motion, node.goal_forward_started_at, duration


def finish_bridge(node, bridge):
    before = len(bridge.motion_status_publisher.messages)
    finish_sequence(bridge)
    for status in bridge.motion_status_publisher.messages[before:]:
        Node._motion_status_callback(node, status)


def shoot_after_completion(node, clock, previous_motion, region, depth, shot_index):
    # A fresh goal frame releases the existing motion/posture gates.
    fresh(node, clock, 'goal', sample(depth))
    timer = getattr(node, 'boundary_decision_timer', None)
    if executable(node)[-1]['action'] != 'GOAL_SHOT_PREPARE' and timer is not None:
        clock[0] = node.boundary_decision_deadline
        if not timer.cancelled:
            timer.callback()
        # Crab rest expiry discards earlier images; use a new stationary frame.
        fresh(node, clock, 'goal', sample(depth), plan=False)
    prepare = executable(node)[-1]
    assert prepare['action'] == 'GOAL_SHOT_PREPARE'
    bridge = FakeBridge()
    bridge.last_physical_motion_id = previous_motion
    deliver(node, bridge, prepare)
    finish_bridge(node, bridge)
    deadline = node.goal_shot_observe_until
    assert deadline == pytest.approx(clock[0] + 1.)
    assert not any(r['motion_id'].startswith('goal_shot')
                   for r in decoded_messages(bridge.executor_request_publisher))
    clock[0] = deadline - .05
    fresh(node, clock, 'goal', sample(depth), plan=False)
    assert executable(node)[-1]['action'] == 'GOAL_SHOT_PREPARE'
    timer = node.boundary_decision_timer
    clock[0] = deadline
    timer.callback()
    shot = executable(node)[-1]
    assert shot['action'] == 'SHOT'
    assert shot['source_command']['shot_region'] == region
    deliver(node, bridge, shot)
    assert bridge.active_motion_id == f'goal_shot_region_{region}'
    node.send_status('SHOT', shot['command_id'], 'RUNNING', bridge.active_motion_id)
    finish_bridge(node, bridge)
    assert node.shots_completed == shot_index
    assert node.active_special_command_id is None
    assert node.mission_phase == 'POST_SHOT_TURN'
    if region in {5, 6, 7, 8}:
        expected = ('POST_SHOT_EXIT_RIGHT_5_FORWARD_4' if shot_index == 1
                    else 'POST_SHOT_EXIT_LEFT_3_FORWARD_4')
    elif region in {1, 2}:
        expected = ('POST_SHOT_EXIT_RIGHT_9_RECOVERY_RIGHT_4' if shot_index == 1
                    else 'POST_SHOT_EXIT_LEFT_4_RECOVERY_LEFT_4')
    else:
        expected = 'POST_SHOT_TURN_RIGHT_9' if shot_index == 1 else 'POST_SHOT_TURN_LEFT_4'
    assert node.phase_manager.post_shot_turn_action() == expected


@pytest.mark.parametrize('region,depth', REGIONS)
@pytest.mark.parametrize('action,info', ENTRIES)
@pytest.mark.parametrize('confirmed', [False, True])
def test_zone_entry_blocks_further_walk_reservation_and_reaches_shot(clock, region, depth, action, info, confirmed):
    node, first, motion, started, duration = start(clock, action, info)
    for fraction in (.72, .80, .88):
        clock[0] = started + duration * fraction
        fresh(node, clock, 'goal', sample(depth, confirmed=confirmed), plan=False)
    assert getattr(node, 'queued_general_command_id', None) is None
    assert not node.planner.goal_terminal_requested
    clock[0] = started + duration
    node.send_status(action, first['command_id'], 'SUCCEEDED', motion)
    shoot_after_completion(node, clock, motion, region, depth, 1)


@pytest.mark.parametrize('region,depth', REGIONS)
@pytest.mark.parametrize('shot_index', [1, 2])
def test_committed_queue_finishes_once_then_reaches_correct_shot(clock, region, depth, shot_index):
    action, info = ENTRIES[0]
    node, first, motion, started, duration = start(clock, action, info, shot_index)
    for fraction in (.75, .80, .85):
        clock[0] = started + duration * fraction
        fresh(node, clock, 'goal', info, plan=False)
    queued = executable(node)[-1]
    assert node.queued_general_command_id == queued['command_id'] != first['command_id']
    node.send_status(action, queued['command_id'], 'QUEUED', motion)
    clock[0] = started + duration * .93
    fresh(node, clock, 'goal', sample(depth), plan=False)
    assert node.queued_general_command_id == queued['command_id']
    clock[0] = started + duration
    node.send_status(action, first['command_id'], 'SUCCEEDED', motion)
    node.send_status(action, queued['command_id'], 'RUNNING', motion)
    second_started = clock[0]
    assert node.queued_general_command_id is None
    for fraction in (.75, .80, .85):
        clock[0] = second_started + duration * fraction
        fresh(node, clock, 'goal', sample(depth), plan=False)
    assert node.queued_general_command_id is None
    clock[0] = second_started + duration
    node.send_status(action, queued['command_id'], 'SUCCEEDED', motion)
    shoot_after_completion(node, clock, motion, region, depth, shot_index)


@pytest.mark.parametrize('region,depth', REGIONS)
def test_rejected_reservation_does_not_block_new_region_shot(clock, region, depth):
    action, info = ENTRIES[0]
    node, first, motion, started, duration = start(clock, action, info)
    for fraction in (.75, .80, .85):
        clock[0] = started + duration * fraction
        fresh(node, clock, 'goal', info, plan=False)
    queued = executable(node)[-1]
    assert node.queued_general_command_id == queued['command_id'] != first['command_id']
    node.send_status(action, queued['command_id'], 'REJECTED', motion)
    assert node.queued_general_command_id is None
    assert node.general_motion_gate.active_command_id == first['command_id']
    clock[0] = started + duration
    node.send_status(action, first['command_id'], 'SUCCEEDED', motion)
    shoot_after_completion(node, clock, motion, region, depth, 1)
