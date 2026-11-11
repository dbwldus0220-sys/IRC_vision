"""Replay the two distance boundaries through the node and motion bridge."""

import json
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from mission_control.motion_decision_planner import MotionDecisionPlanner
from test_hurdle_positioning import HurdleHarness, clock, hurdle, line_info
from test_motion_command_bridge import FakeBridge, decoded_messages, executor_status


@pytest.mark.parametrize('depth', [.700001, .7, .699999, .450001, .45, .449999])
@pytest.mark.parametrize('x', [500, 710, 900])
def test_separate_fine_entry_and_normal_turn_stop(depth, x):
    planner = MotionDecisionPlanner()
    decision = planner.plan('AUTO', {'hurdle': hurdle(depth, hurdle_angle_45_deg=40 if x < 710 else -40 if x > 710 else 0), 'line': line_info(x=x, heading=-40 if x < 710 else 40 if x > 710 else 0)}, .1)
    assert decision.valid
    assert planner.hurdle_planner.fine_approach_active == (depth <= .7)
    assert decision.source_command['close_rotation_blocked'] == (depth <= .45)
    expected = ('GO' if depth <= .45 else
                'ALIGN_LEFT' if x < 710 else
                'ALIGN_RIGHT' if x > 710 else
                'STRAIGHT_1')
    assert decision.action == expected


def test_turn_stop_latch_survives_depth_rebound_and_resets():
    planner = MotionDecisionPlanner()
    planner.plan('AUTO', {'hurdle': hurdle(.45)}, .1)
    rebound = planner.plan('AUTO', {'hurdle': hurdle(.8), 'line': line_info(x=500, heading=-40)}, .1)
    assert rebound.action == 'STRAIGHT_1'
    assert rebound.source_command['close_rotation_blocked']
    planner.hurdle_planner.reset()
    assert not planner.hurdle_planner.fine_approach_active
    assert not planner.hurdle_planner.close_rotation_blocked


@pytest.mark.parametrize('x,direction', [(500, 'LEFT'), (900, 'RIGHT')])
@pytest.mark.parametrize('head_down', [None, False, True])
@pytest.mark.parametrize('checkpoint_during_turn', [False, True])
def test_forward2_turn_final_checkpoint_and_hurdle_release_at_actual_speeds(
    clock, x, direction, head_down, checkpoint_during_turn,
):
    root = Path(__file__).resolve().parents[3]
    motions = {m['name']: m for m in json.loads((root / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    aliases = yaml.safe_load((root / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    node, bridge = HurdleHarness(), FakeBridge(advance_transition_dwells=False)
    forwarded = 0

    def sync():
        nonlocal forwarded
        for message in bridge.motion_status_publisher.messages[forwarded:]:
            Node._motion_status_callback(node, message)
        forwarded = len(bridge.motion_status_publisher.messages)

    def dispatch(command):
        if head_down is not None:
            bridge.head_override_received_at = clock[0]
            bridge.head_override_state = {'hurdle_head_override_active': head_down}
        bridge.navigation_command_callback(String(data=json.dumps(command)))
        sync()

    def finish(during_fine=None):
        for _ in range(20):
            if not bridge.motion_in_progress:
                for name in ('correction_post_motion_dwell_until', 'hurdle_post_motion_dwell_until'):
                    deadline = getattr(node, name, None)
                    if deadline is not None:
                        clock[0] = max(clock[0], deadline)
                        node.publish()  # Release the rest before supplying a fresh frame.
                return
            deadline = bridge.transition_dwell_until or bridge.active_dwell_until
            if deadline is not None:
                if bridge.active_motion_id == bridge.HURDLE_POST_GO_DWELL_MARKER:
                    assert node.phase_manager.hurdles_completed == 0
                    clock[0] = deadline - .001
                    bridge._check_atomic_dwell()
                    sync()
                    assert node.active_special_action == 'GO'
                clock[0] = deadline
                bridge._check_atomic_dwell()
                sync()
                continue
            request = decoded_messages(bridge.executor_request_publisher)[-1]
            bridge.executor_status_callback(executor_status(**request, status='RUNNING'))
            sync()
            motion = motions[aliases[request['motion_id']]]
            duration = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000 * motion['repeat_count'] / motion['playback_speed']
            clock[0] += duration / 2
            if during_fine is not None and (
                request['motion_id'] == 'line_forward_2'
                or request['motion_id'].startswith('post_ball_line_turn_')
            ):
                during_fine()
                during_fine = None
            clock[0] += duration / 2
            bridge.executor_status_callback(executor_status(**request, status='SUCCEEDED'))
            sync()
        pytest.fail('hurdle request did not release within its bounded sequence')

    for depth, target_x, expected in [
        (.7, 710, 'STRAIGHT_1'),
        (.6, x, f'ALIGN_{direction}'),
    ]:
        node.receive_line(x=target_x, heading=-40 if target_x < 710 else 40 if target_x > 710 else 0)
        node.receive(hurdle(depth, hurdle_angle_45_deg=40 if target_x < 710 else -40 if target_x > 710 else 0))
        if expected.startswith('ALIGN_'):
            # Forward no longer supplies a blanket rest; the turn still requires it.
            assert node.publish() == []
            clock[0] += .5
            node.receive(hurdle(depth, hurdle_angle_45_deg=40 if direction == 'LEFT' else -40))
        command = node.publish()[-1]
        assert command['action'] == expected and command['phase'] == 'HURDLE_POSITIONING'
        dispatch(command)
        finish()
        assert not node.general_motion_gate.locked
        if expected.startswith('ALIGN_'):
            requests = decoded_messages(bridge.executor_request_publisher)
            assert [r['motion_id'] for r in requests[-2:]] == [
                'line_forward_2',
                f'post_ball_line_turn_{direction.lower()}_{2 if direction == "LEFT" else 3}',
            ]

    node.receive_line(x=x if checkpoint_during_turn else 710,
                      heading=(-40 if x < 710 else 40) if checkpoint_during_turn else 0)
    node.receive(hurdle(.58, hurdle_angle_45_deg=(40 if x < 710 else -40) if checkpoint_during_turn else 0))
    fine = node.publish()[-1]
    assert fine['action'] == (f'ALIGN_{direction}' if checkpoint_during_turn else 'STRAIGHT_1')
    dispatch(fine)

    def cross_final_depth():
        node.receive_line(x=x, heading=-40 if x < 710 else 40)
        node.receive(hurdle(.45))
        assert node.pending_hurdle_final_sequence is not None
        assert node.publish() == []
        assert bridge.queued_request_id is None

    finish(cross_final_depth)
    assert node.pending_hurdle_final_sequence is not None
    go = node.publish()[-1]
    assert go['action'] == 'GO'
    dispatch(go)
    finish()
    assert node.phase_manager.hurdles_completed == 1
    assert node.mission_phase == 'LINE_TRACK'
    assert not node.general_motion_gate.locked
    assert node.active_special_command_id is None
    assert bridge.queued_request_id is None
    ids = [r['motion_id'] for r in decoded_messages(bridge.executor_request_publisher)]
    assert ids.count('hurdle_fine_forward_10') == ids.count('hurdle') == 1
    assert ids.index('hurdle_fine_forward_10') < ids.index('hurdle')
    node.receive_line()
    following = node.publish()[-1]
    assert following['action'] == 'STRAIGHT' and following['valid']
    dispatch(following)
    finish()
