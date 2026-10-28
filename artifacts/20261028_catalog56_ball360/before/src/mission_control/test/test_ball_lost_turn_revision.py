"""Connect confirmed ball loss to catalog turns without bypassing checkpoints."""

import json
from pathlib import Path

import pytest
import yaml

from mission_control.motion_decision_planner import MotionDecisionPlanner
from mission_control.lost_search_turn_limiter import LostSearchTurnLimiter
from test_motion_decision_planner import ball_info
from test_motion_command_bridge import (
    FakeBridge, navigation_message, fine_alignment_message,
    enter_pickup_fine_alignment, complete_active_motion, decoded_messages, executor_status,
)


MISSING = {'detected': False, 'raw_detected': False, 'ball_loss_confirmed': True}


def tracked_planner(direction):
    planner = MotionDecisionPlanner()
    planner._update_ball_tracking(ball_info(
        camera_center_offset_x_px=-200 if direction == 'LEFT' else 200,
        bottom_distance_px=300,
    ), 0.0)
    planner.ball_lost_elapsed_sec = 1.0
    return planner


def plan(planner, stage, info):
    if stage == 'normal':
        return planner.plan('BALL_APPROACH', {'ball': info}, .1)
    if stage == 'approach':
        return planner.plan_lost_ball_approach_alignment(info)
    method = (planner.plan_ball_pickup_initial_alignment if stage == 'INITIAL'
              else planner.plan_ball_pickup_fine_alignment)
    return method(info)


@pytest.mark.parametrize('stage', ['normal', 'approach', 'INITIAL', 'FINE'])
@pytest.mark.parametrize('direction,count,yaw,max_turns', [('LEFT', 2, 30., 3), ('RIGHT', 5, 45., 2)])
def test_loss_turn_reaches_catalog_and_stays_bounded(stage, direction, count, yaw, max_turns):
    planner = tracked_planner(direction)
    limiter = LostSearchTurnLimiter()
    root = Path(__file__).resolve().parents[3]
    aliases = yaml.safe_load((root/'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    catalog = {m['name'] for m in json.loads((root/'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for _ in range(max_turns):
        decision = limiter.filter(plan(planner, stage, MISSING), {'ball': MISSING})
        assert decision.valid
        assert decision.reason == 'turn_toward_last_seen_ball_side'
        assert decision.source_command['turn_angle_deg'] == yaw
        bridge = FakeBridge()
        if stage in {'INITIAL', 'FINE'}:
            if stage == 'FINE':
                enter_pickup_fine_alignment(bridge)
            else:
                bridge.navigation_command_callback(navigation_message(action='PICKUP_NOW'))
            bridge.navigation_command_callback(fine_alignment_message(decision.action))
        else:
            bridge.navigation_command_callback(navigation_message(action=decision.action, source='ball'))
        request = decoded_messages(bridge.executor_request_publisher)[-1]
        if stage == 'FINE':
            assert request['motion_id'] == 'fine_to_turn_ready_0'
            bridge.executor_status_callback(executor_status(**{**request, 'status': 'SUCCEEDED'}))
            request = decoded_messages(bridge.executor_request_publisher)[-1]
        camera = 0 if stage in {'INITIAL', 'FINE'} else 45
        name = f'찐제자리{"좌" if direction == "LEFT" else "우"}회전{camera}도-1({count}회)'
        assert aliases[request['motion_id']] == name
        assert name in catalog
        if camera == 0:
            assert request['action'] == 'PICKUP_NOW'
            complete_active_motion(bridge)
            assert bridge.active_dwell_until is not None
            bridge._check_atomic_dwell(bridge.active_dwell_until - .001)
            assert not bridge.pickup_initial_align_waiting
            assert not bridge.pickup_fine_align_waiting
            bridge._check_atomic_dwell(bridge.active_dwell_until)
            assert bridge.pickup_initial_align_waiting == (stage == 'INITIAL')
            assert bridge.pickup_fine_align_waiting == (stage == 'FINE')
            assert not bridge.pickup_fixed_sequence_started
        limiter.record_published(decision, {'ball': MISSING})
    blocked = limiter.filter(plan(planner, stage, MISSING), {'ball': MISSING})
    assert not blocked.valid and blocked.reason == 'lost_search_turn_limit_reached'


@pytest.mark.parametrize('stage', ['normal', 'approach', 'INITIAL', 'FINE'])
@pytest.mark.parametrize('missing', [None, {'detected': False},
    {**MISSING, 'raw_detected': True}, {**MISSING, 'detected': True}])
def test_unconfirmed_or_missing_observation_cannot_authorize_memory_turn(stage, missing):
    assert plan(tracked_planner('RIGHT'), stage, missing).reason != 'turn_toward_last_seen_ball_side'


@pytest.mark.parametrize('stage', ['normal', 'approach', 'INITIAL', 'FINE'])
def test_translation_then_turn_and_reacquisition(stage):
    planner = tracked_planner('RIGHT')
    planner.last_ball_inside_image = True
    assert 'FORWARD_4' in plan(planner, stage, MISSING).action
    planner.ball_interior_loss_forward_sent = True
    planner.last_ball_top_edge_ratio = .05
    assert 'FORWARD' in plan(planner, stage, MISSING).action
    planner.ball_top_loss_forward_sent = True
    assert plan(planner, stage, MISSING).reason == 'turn_toward_last_seen_ball_side'
    assert plan(planner, stage, ball_info()).reason != 'turn_toward_last_seen_ball_side'


@pytest.mark.parametrize('stage', ['INITIAL', 'FINE'])
def test_close_loss_preserves_backward_recovery(stage):
    planner = tracked_planner('RIGHT')
    planner.pickup_last_visible_bottom_distance_px = 90
    assert plan(planner, stage, MISSING).action == f'BALL_PICKUP_{stage}_SEARCH_BACKWARD'


def test_unknown_screen_side_never_defaults_to_right():
    planner = tracked_planner('RIGHT')
    planner.last_ball_turn_direction = None
    assert not plan(planner, 'approach', MISSING).valid


def test_node_confirms_loss_waits_one_second_and_requires_new_frames(monkeypatch):
    from std_msgs.msg import String
    from mission_control.motion_decision_node import MotionDecisionNode
    from test_wait_refresh_and_fine_settle import LiveInputHarness
    from test_motion_decision_node import send_status

    clock = [10.0]
    monkeypatch.setattr('mission_control.motion_decision_node.time.monotonic', lambda: clock[0])
    node = LiveInputHarness(phase='BALL_APPROACH')
    node.BALL_POST_MOTION_DWELL_SEC = MotionDecisionNode.BALL_POST_MOTION_DWELL_SEC
    receive = MotionDecisionNode._info_callback(node, 'ball')
    receive(String(data=json.dumps(ball_info(camera_center_offset_x_px=200))))
    node.planner.ball_lost_elapsed_sec = 1.0
    for elapsed in (0., .125, .25, .375, .51):
        clock[0] = 10.1 + elapsed
        receive(String(data=json.dumps({
            'detected': False, 'raw_detected': False,
            'rgb_stamp_ns': round(clock[0] * 1e9),
        })))
        MotionDecisionNode._publish_decision(node)
        if elapsed < .5:
            assert not any(m['valid'] for m in node.publisher.messages)
    request = [m for m in node.publisher.messages if m['valid']][-1]
    assert request['action'] == 'BALL_APPROACH_TURN_RIGHT_5'
    assert node.lost_search_turn_limiter.counts['ball'] == 1
    for status in ('RUNNING', 'SUCCEEDED'):
        send_status(node, status=status, action=request['action'],
                    command_id=request['command_id'], event_id=None, dynamics_command=None)
    deadline = node.ball_post_motion_dwell_until
    assert deadline == pytest.approx(clock[0] + 1.0)
    node.publisher.messages.clear()
    for now in (deadline - .001, deadline):
        clock[0] = now
        MotionDecisionNode._publish_decision(node)
        assert not any(m['valid'] for m in node.publisher.messages)
    assert node.latest_info['ball'] is None
    assert not node.general_motion_gate.has_required_fresh_vision()
    # A new visible ball cancels the remembered loss direction.
    clock[0] += .01
    receive(String(data=json.dumps(ball_info(camera_center_offset_x_px=0))))
    MotionDecisionNode._publish_decision(node)
    assert node.last_selected_decision.reason != 'turn_toward_last_seen_ball_side'
