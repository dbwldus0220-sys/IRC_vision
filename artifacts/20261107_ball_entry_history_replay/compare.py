"""Synthetic log-timed comparison; no ROS graph, camera, or motor connection."""
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('entry_replay', ROOT / 'artifacts/20261107_ball_entry_replay/replay.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)
import pytest
import mission_control.motion_decision_node as module
from step.temporal_confirmation import TemporalConfirmationFilter

namespace = dict(vars(module))
exec(compile((OUT / 'before_methods.py').read_text(), 'before_methods.py', 'exec'), namespace)
rows = []
for advance in (0., .15, .30, .50, .8):
    for version in ('before', 'after'):
        with pytest.MonkeyPatch.context() as patch:
            if version == 'before':
                patch.setattr(replay.Node, '_prepare_initial_ball_entry_history', lambda *args: None)
                patch.setattr(replay.Node, '_prepare_pending_ball_entry_walk', namespace['_prepare_pending_ball_entry_walk'])
            patch.setattr(replay, 'TemporalConfirmationFilter',
                          lambda **kwargs: TemporalConfirmationFilter(edge_area_grace=version == 'after', **kwargs))
            result = replay.run(10, earlier_sec=advance)
        rows.append({'version': version, **result})
for before, after in zip(rows[::2], rows[1::2]):
    assert before['confirmed_relative_ms'] == after['confirmed_relative_ms']
    assert before['action'] == after['action'] == 'RECOVER_LEFT_TURN_LEFT_4'
    assert after['command_relative_ms'] <= before['command_relative_ms']
assert not rows[1]['queued_before_boundary']
assert rows[3]['queued_before_boundary'] and rows[5]['queued_before_boundary']
(OUT / 'results.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2) + '\n')
for before, after in zip(rows[::2], rows[1::2]):
    print({k: after[k] for k in ('candidate_earlier_ms', 'confirmed_relative_ms')},
          'command:', before['command_relative_ms'], '->', after['command_relative_ms'],
          'queued:', before['queued_before_boundary'], '->', after['queued_before_boundary'])

# Separate edge hypothesis: 3 partial hits, one area change, then stable full boxes.
# This sequence is NOT reconstructed from the recording's raw detections.
edge_results = []
for enabled in (False, True):
    f = TemporalConfirmationFilter(window_size=30, required_hits=10,
                                   max_missed_frames=10, edge_area_grace=enabled)
    trace = []
    for index in range(1, 15):
        result = f.update(True, bbox=[500, 0, 600, 20 if index <= 3 else 80],
                          image_width=1280, image_height=720)
        trace.append({'input': index, **result.as_dict()})
        if result.confirmed:
            break
    edge_results.append({'enabled': enabled, 'confirmed_input': index, 'trace': trace})
assert [r['confirmed_input'] for r in edge_results] == [13, 11]
(OUT / 'edge_hypothesis.json').write_text(json.dumps(edge_results, indent=2) + '\n')
print('Partial-edge hypothesis: confirmation at input 13 -> 11 (two input intervals earlier).')
