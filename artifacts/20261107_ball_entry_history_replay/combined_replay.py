"""Hypothetical edge reset through confirmation, decision and simulated bridge.

This is not recovered camera data. Constant distance/ground bearing are assumed
valid inputs; they are not inferred from the synthetic bounding boxes.
"""
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('entry_replay', ROOT / 'artifacts/20261107_ball_entry_replay/replay.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)
import pytest
from std_msgs.msg import String
import mission_control.motion_decision_node as module
from step.temporal_confirmation import TemporalConfirmationFilter

namespace = dict(vars(module))
exec(compile((OUT / 'before_methods.py').read_text(), 'before_methods.py', 'exec'), namespace)


def run(*, edge, history, case='edge_area_change'):
    clock = [10.]
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(module.time, 'monotonic', lambda: clock[0])
        patch.setattr(replay.Node, '_current_ros_time_ns', lambda _: round(clock[0] * 1e9))
        if not history:
            patch.setattr(replay.Node, '_prepare_initial_ball_entry_history', lambda *args: None)
            patch.setattr(replay.Node, '_prepare_pending_ball_entry_walk', namespace['_prepare_pending_ball_entry_walk'])
        node = replay.node_with_entry()
        bridge = replay.FakeBridge(advance_transition_dwells=False)
        _, active = replay.start_line(node, bridge, clock)
        end = node.active_line_motion_started_at + node.active_line_motion_duration_sec
        # Align old 10-hit confirmation (input 13 after a reset at input 4)
        # with the previous synthetic baseline's +134.257ms command time.
        first = end + .134257 - 12 * replay.INTERVAL
        events = [(first + i * replay.INTERVAL, 'ball', i + 1) for i in range(30)]
        events.append((end, 'end', None))
        for interval, kind in ((1/15, 'line'), (.05, 'timer')):
            t = clock[0] + interval
            while t < end + 2.5:
                events.append((t, kind, None))
                t += interval
        f = TemporalConfirmationFilter(window_size=30, required_hits=10,
                                       max_missed_frames=10, edge_area_grace=True)
        trace, confirmed_at, first_command = [], None, None
        published = len(node.publisher.messages)
        for t, kind, index in sorted(events):
            assert t >= clock[0]
            clock[0] = t
            if kind == 'end':
                replay.status(node, bridge, active, 'SUCCEEDED')
            elif kind == 'timer':
                replay.Node._publish_decision(node)
            elif kind == 'line':
                info = {**replay.line_info(), 'rgb_stamp_ns': round((t - replay.AGE) * 1e9)}
                replay.Node._info_callback(node, 'line')(String(data=json.dumps(info)))
            else:
                bbox = [800, 0, 870, 20 if index <= 3 else 80]
                if case == 'position_jump' and index >= 4:
                    bbox = [1100, 0, 1170, 80]
                detected = not (case == 'miss_after_edge' and index == 5)
                confirmation = f.update(detected, bbox=bbox if detected else None,
                    image_width=1280, image_height=720, allow_edge_recovery=edge)
                info = replay.ground_ball_info(depth_m=.86, distance_m=.86, confidence=.9,
                    ground_steering_angle_deg=-28.57, ground_forward_distance_m=.75,
                    depth_age_sec=.15, rgb_stamp_ns=round((t - replay.AGE) * 1e9),
                    bbox=bbox, image_width=1280, image_height=720,
                    center_x=(bbox[0]+bbox[2])/2, center_y=(bbox[1]+bbox[3])/2,
                    bottom_distance_px=719-(bbox[1]+bbox[3])/2,
                    head_down_requested=False, pickup_now=False, pickup_ready=False)
                info.update(confirmation.as_dict())
                info['detected'] = confirmation.confirmed
                if not confirmation.confirmed:
                    info['approach_candidate'] = dict(info)
                    info.update(depth_valid=False, distance_m=None, depth_m=None,
                                note='ball_confirmation_pending' if detected else 'no_ball_detection')
                if confirmation.confirmed and confirmed_at is None:
                    confirmed_at = t
                replay.Node._info_callback(node, 'ball', dispatch_on_ball=True)(String(data=json.dumps(info)))
                trace.append({'input': index, 'relative_ms': (t-end)*1000,
                              **confirmation.as_dict()})
            commands = node.publisher.messages[published:]
            published = len(node.publisher.messages)
            for command in commands:
                if not command.get('valid'):
                    continue
                assert confirmed_at is not None, 'Motion before confirmation'
                assert command['source'] == 'ball', 'Unexpected additional Line motion'
                assert command['action'] == 'RECOVER_LEFT_TURN_LEFT_4'
                request = replay.send(bridge, command)
                queued = t < end and not bridge.queued_request_deferred
                if queued:
                    # Complete the simulated boundary to check status correlation.
                    clock[0] = end
                    replay.status(node, bridge, active, 'SUCCEEDED')
                    replay.status(node, bridge, request, 'RUNNING')
                    assert node.general_motion_gate.active_command_id == command['command_id']
                return dict(case=case, edge_grace=edge, candidate_history=history,
                    confirmed_relative_ms=round((confirmed_at-end)*1000, 3),
                    command_relative_ms=round((t-end)*1000, 3),
                    queued_before_boundary=queued, action=command['action'], trace=trace)
        raise AssertionError('Scenario produced no command')


rows = [run(edge=edge, history=history) for edge, history in
        ((False, False), (False, True), (True, False), (True, True))]
for case in ('position_jump', 'miss_after_edge'):
    rows.extend([run(edge=False, history=False, case=case),
                 run(edge=True, history=True, case=case)])
assert rows[0]['command_relative_ms'] == 134.257 and not rows[0]['queued_before_boundary']
assert rows[1]['command_relative_ms'] == 134.257 and not rows[1]['queued_before_boundary']
assert rows[2]['confirmed_relative_ms'] < 0 and not rows[2]['queued_before_boundary']
assert rows[3]['command_relative_ms'] < 0 and rows[3]['queued_before_boundary']
assert rows[4]['confirmed_relative_ms'] == rows[5]['confirmed_relative_ms']
assert not rows[5]['queued_before_boundary'] and not rows[7]['queued_before_boundary']
(OUT / 'combined_results.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2) + '\n')
for row in rows:
    print({k: v for k, v in row.items() if k != 'trace'})
