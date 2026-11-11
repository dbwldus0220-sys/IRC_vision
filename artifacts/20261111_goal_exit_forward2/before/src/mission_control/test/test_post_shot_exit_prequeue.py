"""Reserve Line driving from either composite's recovery tail, without motors."""

import json

import pytest
import yaml
from std_msgs.msg import String

from mission_control.motion_decision_node import MotionDecisionNode as Node
from test_continuous_approach import clock
from test_mission_phase_flow import line_info
from test_motion_command_bridge import decoded_messages
from test_pickup_continuous_walk import forward_status
from test_post_ball_goal_handoff import receive
from test_post_shot_continuous_exit import ROOT, start_exit
from test_hurdle_positioning import hurdle


def tail_frames(node, clock, samples=None):
    end = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    for delay, sample in zip((.4, .3, .2), samples or [line_info()] * 3):
        clock[0] = end - delay
        receive(node, clock, 'line', sample)


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [1, 2, 5, 6, 7, 8])
@pytest.mark.parametrize('heading,offset,expected', [
    (0., 0., 'STRAIGHT'),
    (30., .3, 'RECOVER_RIGHT_TURN_RIGHT_4'),
    (-30., -.3, 'RECOVER_LEFT_TURN_LEFT_4'),
])
def test_both_exits_reserve_one_line_motion_and_start_only_after_success(
    clock, section, region, heading, offset, expected,
):
    node, bridge, exiting = start_exit(clock, section, region)
    tail_frames(node, clock, [line_info(heading=heading, offset=offset)] * 3)
    command = node.publisher.messages[-1]
    assert command['action'] == expected
    assert command['source_command']['post_shot_exit_prequeue']
    assert node.mission_phase == 'POST_SHOT_TURN'
    assert node.queued_general_command_id == command['command_id']
    bridge.navigation_command_callback(String(data=json.dumps(command)))
    assert bridge.queued_request_deferred
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] += .01
    receive(node, clock, 'line', line_info(heading=heading, offset=offset))
    assert node.queued_general_command_id == command['command_id']
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    forward_status(node, bridge, exiting, 'SUCCEEDED')
    assert node.mission_phase == ('AUTO' if section == 1 else 'LINE_TRACK')
    assert node.post_shot_dwell_until is None
    assert node.line_motion_min_rgb_stamp_ns is None
    assert bridge.transition_dwell_until is None
    following = decoded_messages(bridge.executor_request_publisher)[-1]
    assert following['command_id'] == command['command_id']
    assert len(bridge.executor_request_publisher.messages) == 2
    forward_status(node, bridge, exiting, 'SUCCEEDED')
    assert len(bridge.executor_request_publisher.messages) == 2
    forward_status(node, bridge, following, 'RUNNING')
    assert node.general_motion_gate.active_command_id == command['command_id']
    assert node.active_general_source == 'line'
    clock[0] += node.active_line_motion_duration_sec
    forward_status(node, bridge, following, 'SUCCEEDED')
    assert not node.general_motion_gate.locked
    assert not node.phase_manager.post_shot_failed


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [1, 2, 5, 8])
@pytest.mark.parametrize('bad', [
    'early', 'duplicate', 'stale', 'lost', 'quality', 'heading',
    'over40', 'under_minus40', 'inconsistent', 'finish', 'hurdle',
])
def test_invalid_or_unaligned_evidence_waits_for_new_line_after_completion(clock, section, region, bad):
    node, bridge, exiting = start_exit(clock, section, region)
    start, duration = node.active_line_motion_started_at, node.active_line_motion_duration_sec
    if bad in {'early', 'duplicate', 'stale'}:
        for i in range(3):
            clock[0] = start + duration * (.4 if bad == 'early' else .85) + i * .03
            stamp = start if bad == 'stale' else start + duration * .85 if bad == 'duplicate' else clock[0]
            Node._info_callback(node, 'line')(String(data=json.dumps({
                **line_info(), 'rgb_stamp_ns': round(stamp * 1e9),
            })))
    else:
        changes = {
            'lost': {'detected': False}, 'quality': {'geometry_quality': .01},
            'heading': {'ground_heading_error_deg': None},
            'over40': {'ground_heading_error_deg': 40.001},
            'under_minus40': {'ground_heading_error_deg': -40.001},
        }.get(bad, {})
        if bad in {'finish', 'hurdle'}:
            clock[0] = start + duration - .41
            receive(node, clock, bad, hurdle(.6) if bad == 'hurdle'
                    else {'detected': True, 'raw_detected': True})
        samples = [{**line_info(), **changes}] * 3
        if bad == 'inconsistent':
            samples[1] = line_info(heading=30., offset=.3)
        tail_frames(node, clock, samples)
    assert node.queued_general_command_id is None
    forward_status(node, bridge, exiting, 'SUCCEEDED')
    assert node.mission_phase == 'POST_SHOT_EXIT_ALIGN'
    assert node.latest_info['line'] is None
    assert len(bridge.executor_request_publisher.messages) == 1
    clock[0] += .01
    receive(node, clock, 'line', line_info())
    Node._publish_decision(node)
    assert node.publisher.messages[-1]['action'] == ('STRAIGHT_1' if bad == 'hurdle' else 'STRAIGHT')


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [1, 2, 5, 8])
@pytest.mark.parametrize('status', ['FAILED', 'TIMEOUT', 'CANCELLED', 'REJECTED'])
def test_composite_failure_discards_reserved_line(clock, section, region, status):
    node, bridge, exiting = start_exit(clock, section, region)
    tail_frames(node, clock)
    bridge.navigation_command_callback(String(data=json.dumps(node.publisher.messages[-1])))
    assert bridge.queued_request_deferred
    forward_status(node, bridge, exiting, status)
    assert node.phase_manager.post_shot_failed
    assert node.queued_general_command_id is None
    assert bridge.queued_request_id is None
    assert len(bridge.executor_request_publisher.messages) == 1


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('heading', [-40., 40.])
@pytest.mark.parametrize('region', [1, 2, 5, 8])
def test_exit_heading_boundary_is_inclusive(clock, section, heading, region):
    node, _, _ = start_exit(clock, section, region)
    tail_frames(node, clock, [line_info(heading=heading, offset=.3 if heading > 0 else -.3)] * 3)
    assert node.queued_general_command_id is not None


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [1, 2, 5, 8])
def test_rejected_reservation_falls_back_without_skipping_exit_alignment(clock, section, region):
    node, bridge, exiting = start_exit(clock, section, region)
    tail_frames(node, clock)
    command = node.publisher.messages[-1]
    node.send_status(command['action'], command['command_id'], 'REJECTED')
    assert node.queued_general_command_id is None
    # A rejected reservation must not authorize reuse after alignment changes.
    clock[0] += .01
    receive(node, clock, 'line', line_info(heading=45.))
    forward_status(node, bridge, exiting, 'SUCCEEDED')
    assert node.mission_phase == 'POST_SHOT_EXIT_ALIGN'
    assert node.latest_info['line'] is None
    clock[0] += .01
    receive(node, clock, 'line', line_info(heading=45.))
    Node._publish_decision(node)
    assert node.last_selected_decision.action.startswith('POST_SHOT_LINE_TURN_RIGHT_')
    assert node.mission_phase == 'POST_SHOT_EXIT_ALIGN'


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('blocked', ['executor_auto_ready', 'decision_started'])
def test_reservation_keeps_execution_readiness_guards(clock, section, blocked, monkeypatch):
    node, _, _ = start_exit(clock, section, 5)
    monkeypatch.setattr(Node, '_poll_start_enter', lambda _: None)
    setattr(node, blocked, False)
    tail_frames(node, clock)
    assert node.queued_general_command_id is None


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [1, 2, 5, 8])
def test_invalid_frame_resets_three_frame_confirmation(clock, section, region):
    node, _, _ = start_exit(clock, section, region)
    tail_frames(node, clock, [line_info(), {'detected': False}, line_info()])
    assert node.queued_general_command_id is None
    clock[0] += .03
    receive(node, clock, 'line', line_info())
    assert node.queued_general_command_id is None
    clock[0] += .03
    receive(node, clock, 'line', line_info())
    assert node.queued_general_command_id is not None


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [1, 2, 5, 8])
def test_recovery_tail_rechecks_direction_before_late_commit(clock, section, region):
    node, _, _ = start_exit(clock, section, region)
    start, duration = node.active_line_motion_started_at, node.active_line_motion_duration_sec
    for fraction in (.72, .75, .78):
        clock[0] = start + duration * fraction
        receive(node, clock, 'line', line_info())
    assert node.queued_general_command_id is None
    # A changed heading must replace the earlier straight decision.
    tail_frames(node, clock, [line_info(heading=30., offset=.3)] * 3)
    assert node.publisher.messages[-1]['action'] == 'RECOVER_RIGHT_TURN_RIGHT_4'
    assert node.queued_general_command_id is not None


def test_composite_capture_timings_match_catalog_and_recovery_tail():
    aliases = yaml.safe_load((ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    catalog = {m['name']: m for m in json.loads((ROOT / 'artifacts/robot_motions_runtime.json').read_text())['motions']}
    for action, (duration, count) in Node.POST_SHOT_EXIT_CAPTURE_CONFIG.items():
        motion = catalog[aliases[action.lower()]]
        turn = catalog[motion['name'].split('+')[0]]
        turn_end = max(f['start_ms'] + f['time_ms'] for f in turn['frames']) / 1000 / motion['playback_speed']
        recovery_start = motion['frames'][len(turn['frames'])]['start_ms'] / 1000 / motion['playback_speed']
        end = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000
        assert duration == pytest.approx(end * motion['repeat_count'] / motion['playback_speed'])
        assert recovery_start - turn_end == pytest.approx(.5)
        assert duration * Node.LINE_MOTION_CAPTURE_START_RATIO > recovery_start
        assert count == 3


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('depth,confirmed,blocked', [
    (1.5, True, False), (1.000001, True, False),
    (.6, False, False), (1., True, True), (.7, True, True), (.45, True, True),
])
def test_exit_reservation_uses_actual_hurdle_entry_conditions(clock, section, depth, confirmed, blocked):
    node, _, _ = start_exit(clock, section, 8)
    clock[0] = node.active_line_motion_started_at + node.active_line_motion_duration_sec - .41
    receive(node, clock, 'hurdle', hurdle(depth, confirmation_confirmed=confirmed,
                                       detected=confirmed, raw_detected=True))
    tail_frames(node, clock)
    assert (node.queued_general_command_id is None) is blocked


@pytest.mark.parametrize('section', [1, 2])
@pytest.mark.parametrize('region', [1, 2, 5, 6, 7, 8])
@pytest.mark.parametrize('reservation', ['not_submitted', 'rejected'])
def test_completion_revalidates_tail_without_waiting_for_another_image(clock, monkeypatch, section, region, reservation):
    node, bridge, exiting = start_exit(clock, section, region)
    end = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    if reservation == 'not_submitted':
        monkeypatch.setattr(Node, '_walking_prequeue_allowed', lambda *args: False)
    tail_frames(node, clock, [line_info(heading=30., offset=.3)] * 3)
    if reservation == 'rejected':
        command = node.publisher.messages[-1]
        assert node.queued_general_command_id == command['command_id']
        node.send_status(command['action'], command['command_id'], 'REJECTED')
    assert node.queued_general_command_id is None
    node.executor_auto_ready = True
    before = len(node.publisher.messages)
    clock[0] = end
    forward_status(node, bridge, exiting, 'SUCCEEDED')
    commands = [c for c in node.publisher.messages[before:] if c['valid']]
    assert len(commands) == 1
    assert commands[0]['action'] == 'RECOVER_RIGHT_TURN_RIGHT_4'
    assert node.mission_phase == ('AUTO' if section == 1 else 'LINE_TRACK')
    assert node.latest_info['line'] is not None
    bridge.navigation_command_callback(String(data=json.dumps(commands[0])))
    assert len(bridge.executor_request_publisher.messages) == 2
    assert bridge.transition_dwell_until is None
    forward_status(node, bridge, exiting, 'SUCCEEDED')
    assert len([c for c in node.publisher.messages[before:] if c['valid']]) == 1


@pytest.mark.parametrize('invalid', [
    'stale', 'future', 'lost', 'heading', 'quality', 'inconsistent',
    'hurdle', 'finish', 'executor', 'failed',
])
def test_completion_reuse_never_bypasses_newer_input_or_safety(clock, monkeypatch, invalid):
    node, bridge, exiting = start_exit(clock, 2, 8)
    end = node.active_line_motion_started_at + node.active_line_motion_duration_sec
    monkeypatch.setattr(Node, '_walking_prequeue_allowed', lambda *args: False)
    samples = [line_info()] * 3
    if invalid == 'inconsistent':
        samples[1] = line_info(heading=30., offset=.3)
    tail_frames(node, clock, samples)
    clock[0] = end - .1
    if invalid in {'lost', 'heading', 'quality'}:
        receive(node, clock, 'line', {**line_info(), **{
            'lost': {'detected': False}, 'heading': {'ground_heading_error_deg': 45.},
            'quality': {'geometry_quality': .01},
        }[invalid]})
    elif invalid == 'future':
        node.latest_info['line']['rgb_stamp_ns'] = round((end + 1.) * 1e9)
    elif invalid in {'hurdle', 'finish'}:
        receive(node, clock, invalid, hurdle(.6) if invalid == 'hurdle'
                else {'detected': True, 'raw_detected': True})
    node.executor_auto_ready = invalid != 'executor'
    before = len(node.publisher.messages)
    clock[0] = end + (1. if invalid == 'stale' else 0.)
    forward_status(node, bridge, exiting, 'FAILED' if invalid == 'failed' else 'SUCCEEDED')
    assert not any(c['valid'] for c in node.publisher.messages[before:])
    assert len(bridge.executor_request_publisher.messages) == 1
