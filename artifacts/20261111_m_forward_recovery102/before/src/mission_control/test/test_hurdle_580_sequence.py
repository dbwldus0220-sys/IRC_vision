"""Replay the 580 mm handoff, camera-zero alignment and fixed hurdle tail."""

import json

import pytest
from std_msgs.msg import String

from step.hurdle_navigation_planner import HurdleNavigationPlanner
from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_hurdle_positioning import HurdleHarness, hurdle
from test_continuous_approach import clock
from test_motion_command_bridge import FakeBridge, navigation_message, decoded_messages, executor_status


def near(depth=.57, heading=0., **changes):
    return hurdle(depth, **{'hurdle_camera_pose': '0', 'hurdle_angle_0_deg': -heading, **changes})


@pytest.mark.parametrize('depth', [.580001, .7, 1.])
@pytest.mark.parametrize('heading,action,count', [
    (0, 'STRAIGHT_3', None), (14.999, 'STRAIGHT_3', None),
    (-15, 'RECOVER_LEFT_TURN_LEFT_4', None), (15, 'RECOVER_RIGHT_TURN_RIGHT_4', None),
    (-50, 'RECOVER_LEFT_TURN_LEFT_4', None), (50, 'RECOVER_RIGHT_TURN_RIGHT_4', None),
    (-50.001, 'ALIGN_LEFT', 3), (50.001, 'ALIGN_RIGHT', 5),
])
def test_far_approach_uses_hurdle_yaw_and_all_three_gaits(depth, heading, action, count):
    command = HurdleNavigationPlanner().plan(hurdle(depth, hurdle_angle_45_deg=-heading))
    assert command.valid and command.action == action
    assert command.turn_count == count
    assert not command.near_control and not command.go_now
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        source='hurdle', action=action, source_command=command.to_dict()))
    expected = ('line_forward_6' if action == 'STRAIGHT_3' else
                f'post_ball_line_turn_{"left" if heading < 0 else "right"}_{count}' if count else
                f'line_recovery_{"left" if heading < 0 else "right"}_4')
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == expected


@pytest.mark.parametrize('heading,action,count', [
    (-10, 'GO', None), (0, 'GO', None), (10, 'GO', None),
    (-10.001, 'ALIGN_LEFT', 1), (10.001, 'ALIGN_RIGHT', 1),
    (-30, 'ALIGN_LEFT', 2), (30, 'ALIGN_RIGHT', 3),
    (-45, 'ALIGN_LEFT', 3), (45, 'ALIGN_RIGHT', 5),
    (-60, 'ALIGN_LEFT', 4), (60, 'ALIGN_RIGHT', 6),
    (-75, 'ALIGN_LEFT', 5), (75, 'ALIGN_RIGHT', 7),
])
def test_near_alignment_has_strict_ten_degree_trigger_and_old_counts(heading, action, count):
    planner = MotionDecisionPlanner()
    decision = planner.plan('HURDLE_POSITIONING', {'hurdle': near(.580, heading)}, .1)
    assert decision.valid and decision.action == action
    assert decision.source_command['hurdle_near_control']
    if count is not None:
        assert decision.source_command['turn_count'] == count
        bridge = FakeBridge()
        bridge.navigation_command_callback(navigation_message(
            source='hurdle', action=action, source_command=decision.source_command))
        assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == (
            f'pickup_camera_down_turn_{"left" if heading < 0 else "right"}_{count}')


@pytest.mark.parametrize('changes', [
    {'hurdle_camera_ready': False}, {'hurdle_camera_pose': '45'},
    {'hurdle_angle_0_deg': None}, {'depth_valid': False},
    {'confirmation_confirmed': False}, {'depth_age_sec': .701},
])
def test_close_depth_alone_never_authorizes_go(changes):
    assert not HurdleNavigationPlanner().plan(near(.1, **changes)).valid


def test_near_entry_survives_depth_rebound_and_never_returns_to_far_gaits():
    planner = HurdleNavigationPlanner()
    assert planner.plan(near(.58, 20)).action == 'ALIGN_RIGHT'
    assert planner.plan(near(.8, 20)).action == 'ALIGN_RIGHT'
    assert planner.plan(near(.8, 0)).action == 'GO'


@pytest.mark.parametrize('side,initial', [('RIGHT', 3), ('LEFT', 2)])
def test_first_top_loss_searches_from_robot_line_side_then_one_at_a_time(side, initial):
    planner = HurdleNavigationPlanner()
    planner.begin_near(side)
    planner.observe(near(center_y=30, bottom_distance_px=689))
    missing = {'detected': False, 'raw_detected': False,
               'hurdle_camera_pose': '0', 'hurdle_camera_ready': True}
    for index in range(20):
        command = planner.plan(missing)
        assert command.action == f'ALIGN_{side}'
        assert command.turn_count == (initial if index == 0 else 1)
        assert command.to_dict()['hurdle_search_until_visible']
        # Merely evaluating the same image must not consume the initial search.
        assert planner.plan(missing).turn_count == command.turn_count
        planner.record_published(command.action, command.reason)
    assert planner.plan(near(heading=5)).action == 'GO'


def test_first_bottom_loss_retreats_twice_only_once():
    planner = HurdleNavigationPlanner()
    planner.plan(near(bottom_distance_px=40))
    missing = {'detected': False, 'raw_detected': False,
               'hurdle_camera_pose': '0', 'hurdle_camera_ready': True}
    command = planner.plan(missing)
    assert command.action == 'HURDLE_LOST_BACKWARD_2'
    planner.record_published(command.action, command.reason)
    assert planner.plan(missing).action == 'WAIT'
    assert planner.plan(near(heading=-30)).action == 'ALIGN_LEFT'


def observe(node, clock, info, delay=0.):
    stamp = round((clock[0] - delay) * 1e9)
    node.receive({**info, 'rgb_stamp_ns': stamp,
                  'stamp': {'sec': stamp // 1_000_000_000, 'nanosec': stamp % 1_000_000_000}})


def executable(node):
    return [c for c in node.publisher.messages if c['valid']]


def start_far(node, clock):
    observe(node, clock, hurdle(.8))
    node.publish()
    command = executable(node)[-1]
    assert command['action'] == 'STRAIGHT_3'
    node.send_status(command['action'], command['command_id'], 'RUNNING', 'line_forward_6')
    return command


@pytest.mark.parametrize('heading,expected', [(0, 'GO'), (-30, 'ALIGN_LEFT'), (30, 'ALIGN_RIGHT')])
def test_crossing_keeps_current_motion_then_uses_images_from_one_second_rest(clock, heading, expected):
    node = HurdleHarness()
    command = start_far(node, clock)
    clock[0] += .3
    observe(node, clock, hurdle(.57, head_down_requested=True, hurdle_camera_ready=False))
    assert node.hurdle_positioning_entry_pending
    assert getattr(node, 'pending_hurdle_final_sequence', None) is None
    assert node.general_motion_gate.active_command_id == command['command_id']
    assert getattr(node, 'queued_general_command_id', None) is None
    clock[0] += 2.
    node.send_status(command['action'], command['command_id'], 'SUCCEEDED', 'line_forward_6')
    stopped = clock[0]
    count = len(executable(node))
    clock[0] = stopped + .75
    observe(node, clock, near(heading=heading))
    assert len(executable(node)) == count
    clock[0] = stopped + .999
    node.publish()
    assert len(executable(node)) == count
    clock[0] = stopped + 1.
    node.publish()
    assert len(executable(node)) == count + 1
    assert executable(node)[-1]['action'] == expected


def test_old_image_and_camera_transition_cannot_release_near_sequence(clock):
    node = HurdleHarness()
    command = start_far(node, clock)
    clock[0] += .3
    observe(node, clock, hurdle(.57, hurdle_camera_ready=False))
    clock[0] += 2.
    node.send_status(command['action'], command['command_id'], 'SUCCEEDED', 'line_forward_6')
    stopped = clock[0]
    clock[0] += .3
    observe(node, clock, near(), delay=.4)
    assert node.latest_info['hurdle'] is None
    clock[0] = stopped + 1.
    observe(node, clock, near(hurdle_camera_ready=False))
    node.publish()
    assert len(executable(node)) == 1
    clock[0] += .01
    observe(node, clock, near())
    node.publish()
    assert executable(node)[-1]['action'] == 'GO'


def test_one_second_node_rest_satisfies_bridge_turn_dwell(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'line_forward_6'
    bridge.stationary_since = clock[0] - 1.
    bridge.navigation_command_callback(navigation_message(
        source='hurdle', action='ALIGN_RIGHT',
        source_command={'hurdle_near_control': True, 'turn_count': 3}))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'pickup_camera_down_turn_right_3'
    assert bridge.transition_dwell_until is None


def test_fixed_tail_and_post_hurdle_second_remain_atomic(clock):
    bridge = FakeBridge(advance_transition_dwells=False)
    bridge.last_physical_motion_id = 'pickup_camera_down_turn_right_3'
    bridge.stationary_since = clock[0] - 1.
    bridge.head_override_state = {'hurdle_head_override_active': True}
    bridge.head_override_received_at = clock[0]
    bridge.navigation_command_callback(navigation_message(source='hurdle', action='GO'))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'r_forward_0_4'
    for _ in range(20):
        if not bridge.motion_in_progress:
            break
        deadline = bridge.transition_dwell_until or bridge.active_dwell_until
        if deadline is not None:
            if bridge.active_motion_id == bridge.HURDLE_POST_GO_DWELL_MARKER:
                assert deadline == pytest.approx(clock[0] + 1.)
                clock[0] = deadline - .001
                bridge._check_atomic_dwell()
                assert bridge.motion_in_progress
            clock[0] = max(clock[0], deadline)
            bridge._check_atomic_dwell()
        else:
            request = decoded_messages(bridge.executor_request_publisher)[-1]
            clock[0] += 1.
            bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    assert not bridge.motion_in_progress
    ids = [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)]
    assert ids == ['r_forward_0_4', 'hurdle_fine_prepare_0', 'hurdle_fine_forward_10', 'hurdle']
    statuses = decoded_messages(bridge.motion_status_publisher)
    assert statuses[-1]['status'] == 'SUCCEEDED'
    assert any(s['motion_id'] == bridge.HURDLE_POST_GO_DWELL_MARKER for s in statuses)


def test_manual_near_phase_entry_rearms_one_second_and_fresh_frame(clock):
    node = HurdleHarness()
    Node._phase_callback(node, String(data='HURDLE_POSITIONING'))
    observe(node, clock, near())
    node.publish()
    assert not executable(node)
    assert node.hurdle_stationary_since == clock[0]
    clock[0] += .9
    observe(node, clock, near())
    node.publish()
    assert not executable(node)
    clock[0] += .1
    node.publish()
    assert executable(node)[-1]['action'] == 'GO'


def test_clock_reset_rebases_near_capture_boundary(clock):
    node = HurdleHarness()
    node.hurdle_near_min_rgb_stamp_ns = 100_000_000_000
    Node._invalidate_clock_observations(node, round(clock[0] * 1e9))
    assert node.hurdle_near_min_rgb_stamp_ns == round(clock[0] * 1e9)
    clock[0] += .1
    observe(node, clock, near())
    assert node.latest_info['hurdle'] is not None


def test_node_completes_mission_only_after_full_tail_and_post_wait(clock):
    node, bridge = HurdleHarness(), FakeBridge(advance_transition_dwells=False)
    observe(node, clock, near())
    node.publish()
    clock[0] += .9
    observe(node, clock, near())
    clock[0] += .1
    node.publish()
    command = executable(node)[-1]
    assert command['action'] == 'GO'
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    forwarded = 0

    def sync_status():
        nonlocal forwarded
        for message in bridge.motion_status_publisher.messages[forwarded:]:
            Node._motion_status_callback(node, message)
        forwarded = len(bridge.motion_status_publisher.messages)

    sync_status()
    for _ in range(20):
        assert node.phase_manager.hurdles_completed == 0
        assert node.active_special_action == 'GO'
        deadline = bridge.transition_dwell_until or bridge.active_dwell_until
        if deadline is not None:
            clock[0] = deadline
            bridge._check_atomic_dwell()
        else:
            request = decoded_messages(bridge.executor_request_publisher)[-1]
            clock[0] += 1.
            bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
        sync_status()
        if not bridge.motion_in_progress:
            break
    assert node.phase_manager.hurdles_completed == 1
    assert node.mission_phase == 'LINE_TRACK'
    assert node.active_special_command_id is None
    assert not node.planner.hurdle_planner.fine_approach_active
    assert node.hurdle_near_min_rgb_stamp_ns is None


@pytest.mark.parametrize('side,offset,first_count', [('LEFT', -70, 2), ('RIGHT', 70, 3)])
def test_node_top_loss_uses_entry_line_and_waits_between_search_turns(clock, side, offset, first_count):
    from test_mission_phase_flow import line_info
    node = HurdleHarness()
    first = start_far(node, clock)
    clock[0] += .2
    Node._info_callback(node, 'line')(String(data=json.dumps({
        **line_info(), 'lateral_offset_px': offset, 'rgb_stamp_ns': round(clock[0] * 1e9)})))
    observe(node, clock, hurdle(.57, center_y=20, bottom_distance_px=699))
    clock[0] += 2.
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED', 'line_forward_6')
    missing = {'detected': False, 'raw_detected': False,
               'hurdle_camera_pose': '0', 'hurdle_camera_ready': True}
    for count in (first_count, 1, 1):
        stopped = clock[0]
        clock[0] += .9
        observe(node, clock, missing)
        before = len(executable(node))
        clock[0] = stopped + 1.
        node.publish()
        assert len(executable(node)) == before + 1
        command = executable(node)[-1]
        assert command['action'] == f'ALIGN_{side}'
        assert command['source_command']['turn_count'] == count
        motion = f'pickup_camera_down_turn_{side.lower()}_{count}'
        node.send_status(command['action'], command['command_id'], 'RUNNING', motion)
        clock[0] += .5
        observe(node, clock, missing)
        assert len(executable(node)) == before + 1
        clock[0] += 1.
        node.send_status(command['action'], command['command_id'], 'SUCCEEDED', motion)
    stopped = clock[0]
    clock[0] += .9
    observe(node, clock, near(heading=0))
    clock[0] = stopped + 1.
    node.publish()
    assert executable(node)[-1]['action'] == 'GO'


def test_node_first_bottom_loss_uses_two_cycle_backward_alias(clock):
    node = HurdleHarness()
    first = start_far(node, clock)
    clock[0] += .2
    observe(node, clock, hurdle(.57, bottom_distance_px=50))
    clock[0] += 2.
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED', 'line_forward_6')
    stopped = clock[0]
    clock[0] += .9
    observe(node, clock, {'detected': False, 'raw_detected': False,
                         'hurdle_camera_pose': '0', 'hurdle_camera_ready': True})
    clock[0] = stopped + 1.
    node.publish()
    command = executable(node)[-1]
    assert command['action'] == 'HURDLE_LOST_BACKWARD_2'
    bridge = FakeBridge()
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert decoded_messages(bridge.executor_request_publisher)[-1]['motion_id'] == 'ball_lost_camera_down_backward_2'


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_failed_near_turn_does_not_launch_fixed_tail(clock, status):
    node = HurdleHarness()
    first = start_far(node, clock)
    clock[0] += .2
    observe(node, clock, hurdle(.57))
    clock[0] += 2.
    node.send_status(first['action'], first['command_id'], 'SUCCEEDED', 'line_forward_6')
    clock[0] += .9
    observe(node, clock, near(heading=30))
    clock[0] += .1
    node.publish()
    turn = executable(node)[-1]
    assert turn['action'] == 'ALIGN_RIGHT'
    node.send_status(turn['action'], turn['command_id'], 'RUNNING', 'pickup_camera_down_turn_right_3')
    node.send_status(turn['action'], turn['command_id'], status, 'pickup_camera_down_turn_right_3')
    clock[0] += 2.
    observe(node, clock, near())
    node.publish()
    assert not any(c['action'] == 'GO' for c in executable(node))


@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED'])
def test_failed_forward_stage_never_reaches_fine_or_hurdle(status):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(source='hurdle', action='GO'))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert request['motion_id'] == 'r_forward_0_4'
    bridge.executor_status_callback(executor_status(**request, status=status, error_code='TEST_FAILURE'))
    assert not bridge.motion_in_progress
    assert [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)] == ['r_forward_0_4']


@pytest.mark.parametrize('failed_stage', ['r_forward_0_4', 'hurdle_fine_forward_10', 'hurdle'])
def test_go_retry_does_not_repeat_successfully_completed_walking(failed_stage):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(source='hurdle', action='GO', command_id=9000))
    for _ in range(8):
        request = decoded_messages(bridge.executor_request_publisher)[-1]
        if request['motion_id'] == failed_stage:
            break
        bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    assert request['motion_id'] == failed_stage
    bridge.executor_status_callback(executor_status(**request, status='FAILED', error_code='TEST_FAILURE'))
    before = len(bridge.executor_request_publisher.messages)
    bridge.navigation_command_callback(navigation_message(source='hurdle', action='GO', command_id=9001))
    for _ in range(8):
        request = decoded_messages(bridge.executor_request_publisher)[-1]
        if request['motion_id'] == failed_stage:
            break
        bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
    retried = [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)[before:]]
    assert retried[-1] == failed_stage
    if failed_stage != 'r_forward_0_4':
        assert 'r_forward_0_4' not in retried
    if failed_stage == 'hurdle':
        assert 'hurdle_fine_forward_10' not in retried
