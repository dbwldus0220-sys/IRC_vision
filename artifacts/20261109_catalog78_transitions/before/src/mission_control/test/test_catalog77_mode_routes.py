"""Audit actual bridge dispatch for every affected mission and camera/count."""

import copy
import json
import pytest

from mission_control.motion_command_bridge_node import MotionCommandBridgeNode as Bridge
from test_catalog77_p_motions import ALIASES, ROOT, RUNTIME, SOURCE
from test_motion_command_bridge import (
    FakeBridge, decoded_messages, fine_alignment_message, navigation_message,
)


CASES = []


def add(phase, source, index, action, expected, **command):
    CASES.append((phase, source, index, action, expected, command))


for phase in ('LINE_TRACK', 'LINE_TRACK_AFTER_PICKUP', 'POST_BALL_LINE_ALIGN', 'POST_SHOT_LINE_ALIGN'):
    for action, count in [('STRAIGHT', 6), ('STRAIGHT_1', 2), ('STRAIGHT_2', 4),
                          ('STRAIGHT_3', 6), ('STRAIGHT_4', 8), ('LINE_SPARSE_FORWARD', 2)]:
        add(phase, 'line', 1, action, f'p전진45({count}회)')
    for count in range(1, 7):
        for prefix in ('LINE_OFFSET', 'LINE_HEADING', 'LINE_SPARSE', 'POST_BALL_LINE', 'POST_SHOT_LINE'):
            add(phase, 'line', 1, f'{prefix}_TURN_LEFT_{count}', f'p제자리좌회전45({count}회)')
    for side in ('LEFT', 'RIGHT'):
        add(phase, 'line', 1, f'RECOVER_{side}_TURN_LEFT_4', 'p라인복귀좌회전45도')

for index in (1, 2):
    for action, expected in [('STRAIGHT', 'p전진45(4회)'),
                             ('STRAIGHT_0', 'p미세0도(5회)'),
                             ('BALL_LOST_FORWARD_2', 'p전진45(2회)'),
                             ('BALL_LOST_FORWARD_4', 'p전진45(4회)'),
                             ('BALL_LOST_FINE_FORWARD_3', 'p미세0도(3회)'),
                             ('BALL_APPROACH_FINE_FORWARD_5', 'p미세45도(5회)')]:
        add('BALL_APPROACH', 'ball', index, action, expected)
    for count in range(1, 7):
        for alignment in (False, True):
            add('BALL_APPROACH', 'ball', index, f'BALL_APPROACH_TURN_LEFT_{count}',
                f'p제자리좌회전45({count}회)', ball_fine_alignment=alignment)
    add('BALL_APPROACH', 'ball', index, 'BALL_APPROACH_FINE_FORWARD_5',
        'p미세45도(5회)', ball_fine_alignment=True)
    add('BALL_APPROACH', 'ball', index, 'BALL_APPROACH_RECOVER_LEFT_4',
        'p라인복귀좌회전공용' if index == 1 else 'p라인복귀좌회전45도')
    add('BALL_APPROACH', 'ball', index, 'BALL_APPROACH_RECOVER_RIGHT_4',
        't라인복귀우공용' if index == 2 else 't라인복귀우실전')
    for action, count in [('GOAL_CAMERA_90_FORWARD', 6), ('GOAL_CAMERA_90_FORWARD_1', 2),
                          ('GOAL_CAMERA_90_FORWARD_2', 4)]:
        add('GOAL_APPROACH', 'goal', index, action, f'p전진90도({count}회)')
    for count in range(1, 5):
        add('GOAL_APPROACH', 'goal', index, f'GOAL_CAMERA90_FINE_FORWARD_{count}', f'p미세90도({count}회)')
    for count in range(1, 7):
        add('GOAL_APPROACH', 'goal', index, f'GOAL_CAMERA90_TURN_LEFT_{count}', f'p제자리좌회전90도({count}회)')
    # Goal recovery shares the Line body gait; executor owns the camera override.
    add('GOAL_APPROACH', 'goal', index, 'GOAL_CAMERA90_RECOVER_LEFT_4', 'p라인복귀좌회전45도')

for action, expected in [('STRAIGHT', 'p전진45(4회)'), ('STRAIGHT_0', 'p미세0도(5회)'),
                         ('GO', 'p미세0도(10회)')]:
    add('HURDLE_APPROACH', 'hurdle', 2, action, expected)
for count in range(1, 7):
    add('HURDLE_APPROACH', 'hurdle', 2, 'ALIGN_LEFT', f'p제자리좌회전45({count}회)', turn_count=count)


@pytest.mark.parametrize('phase,source,index,action,expected,command', CASES)
def test_mission_request_reaches_new_catalog(phase, source, index, action, expected, command):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        phase=phase, source=source, action=action, source_command=command,
        mission_progress={'active_ball_mode_index': index, 'completed_ball_index': index,
                          'pickups_completed': index - 1, 'required_pickups': 2}))
    requests = decoded_messages(bridge.executor_request_publisher)
    assert requests, decoded_messages(bridge.motion_status_publisher)
    assert ALIASES[requests[-1]['motion_id']] == expected
    assert expected in RUNTIME


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('stage', ['initial', 'fine'])
@pytest.mark.parametrize('count', range(1, 7))
def test_pickup_checkpoint_uses_all_camera_zero_left_turns(index, stage, count):
    bridge = FakeBridge()
    bridge.navigation_command_callback(navigation_message(
        action='PICKUP_NOW', mission_progress={'active_ball_mode_index': index}))
    if stage == 'fine':
        bridge.pickup_initial_align_waiting = False
        bridge.pickup_fine_align_waiting = True
        bridge.active_motion_id = Bridge.FINE_ALIGN_MARKER
    prefix = 'BALL_PICKUP_CAMERA_DOWN_TURN_LEFT' if stage == 'initial' else 'BALL_PICKUP_FINE_TURN_LEFT'
    bridge.navigation_command_callback(fine_alignment_message(f'{prefix}_{count}'))
    request = decoded_messages(bridge.executor_request_publisher)[-1]
    assert ALIASES[request['motion_id']] == f'p제자리좌회전0도({count}회)'


@pytest.mark.parametrize('camera,counts', [(0, range(1, 7)), (45, range(1, 7)), (90, range(1, 7))])
def test_all_source_camera_variants_and_counts_are_present(camera, counts):
    suffix = str(camera) if camera == 45 else f'{camera}도'
    for count in counts:
        name = f'p제자리좌회전{suffix}({count}회)'
        assert RUNTIME[name]['repeat_count'] == count
        assert RUNTIME[name]['playback_speed'] == pytest.approx(SOURCE[name]['playback_speed'] * .95)
    for count in range(1, 6):
        name = f'p미세{camera}도({count}회)'
        assert RUNTIME[name]['repeat_count'] == count
        assert RUNTIME[name]['playback_speed'] == pytest.approx(SOURCE[name]['playback_speed'] * .95)


def body_frames(name):
    result = copy.deepcopy(RUNTIME[name]['frames'])
    for frame in result:
        for key in ('name', 'frame_id', 'source_frame_id'):
            frame.pop(key, None)
        frame['angles'].pop('0')
    return result


def test_shared_camera_override_routes_preserve_same_body_timing_and_torque():
    assert body_frames('p전진45(4회)') == body_frames('p전진실전0도(4회)')
    assert body_frames('p전진45(4회)') == body_frames('p전진90도(4회)')
    assert body_frames('p라인복귀좌회전45도') == body_frames('p라인복귀좌회전90도')


def test_composite_exit_exceptions_are_explicit_and_unchanged():
    before = json.loads((ROOT / 'artifacts/20261108_catalog77_p_motions/before/artifacts/robot_motions_runtime.json').read_text())
    before = {m['name']: m for m in before['motions']}
    for alias, name in [('pickup_second_backward_turn_left', 't공후 제자리좌회전'),
                        ('post_shot_default_turn_left', 't골대후 제자리좌회전')]:
        assert ALIASES[alias] == name
        assert name not in SOURCE
        assert RUNTIME[name] == before[name]


def test_installed_configuration_points_to_updated_workspace():
    config = ROOT / 'install/irc_step_motion_executor/share/irc_step_motion_executor/config'
    assert (config / 'robot_motions_runtime.json').resolve() == ROOT / 'artifacts/robot_motions_runtime.json'
    assert (config / 'motion_aliases.yaml').resolve() == ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    assert (ROOT / 'build/mission_control/mission_control').resolve() == ROOT / 'src/mission_control/mission_control'
