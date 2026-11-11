#!/usr/bin/env python3
"""Build goal exits with a half-second hold, optional forward, and line recovery."""

import copy
import json
from pathlib import Path

from upsert_motion_catalog import runtime_motion_from_source


ROOT = Path(__file__).resolve().parents[1]
EXIT_RECOVERIES = (
    # Retained for archived source99/source100 import scripts.
    ('t골대후제자리우회전(5회)', 'n라인복귀좌회전45도'),
    ('p골대후제자리좌회전(3회)', 'n라인복귀우회전45도'),
)
GOAL_EXIT_RECOVERIES = (
    ('m골대후제자리우회전', 'm라인복귀우회전45도', 1),
    ('p골대후제자리좌회전', 'm라인복귀우회전45도', 1),
    ('m골대후제자리우회전(6회)', 'm라인복귀우회전45도', 1),
    ('p골대후제자리좌회전(4회)', 'm라인복귀좌회전45도', 1),
)
# Catalog names, like public command IDs, are retained for existing callers.
FORWARD_EXIT_NAMES = {
    # Archived imports still rebuild the previous t turn data.
    't골대후제자리우회전(6회)': 't골대후제자리우회전(6회)' + '+라인복귀좌회전45도(4회)' * 2,
    'm골대후제자리우회전(6회)': 'm골대후제자리우회전(6회)' + '+라인복귀좌회전45도(4회)' * 2,
    'p골대후제자리좌회전(4회)': 'p골대후제자리좌회전(4회)' + '+라인복귀우회전45도(4회)' * 2,
}
TURN_RECOVERY_DWELL_MS = 500


def build_turn_variant(turn: dict, count: int) -> dict:
    """Keep preparation and complete cycles: source104 right turns use four frames."""
    frames_per_cycle = 4 if turn['name'] == 'm골대후제자리우회전' else 2
    frame_count = 2 + frames_per_cycle * count
    assert count > 0 and turn['repeat_count'] == 1
    assert frame_count <= len(turn['frames'])
    result = copy.deepcopy(turn)
    result['name'] = f"{turn['name']}({count}회)"
    result['frames'] = result['frames'][:frame_count]
    for field in ('angles', 'torques'):
        assert result['frames'][-1][field] == turn['frames'][-1][field]
    result['max_seq_ms'] = max(f['start_ms'] + f['time_ms'] for f in result['frames'])
    return result


def build_exit(turn: dict, recovery: dict, recovery_blocks: int = 1,
               *, forward: dict | None = None) -> dict:
    """Keep turn timing and encode recovery timing at the composite's speed."""
    assert turn['repeat_count'] == 1 and recovery['repeat_count'] == 4
    assert recovery_blocks in (1, 2)
    # Source104 ends with the camera down; the first gait frame moves it to 45.
    # Body targets and every torque flag must still match at the join.
    for field in ('angles', 'torques'):
        turn_end_values = turn['frames'][-1][field]
        recovery_end_values = recovery['frames'][-1][field]
        if field == 'angles' and turn['name'] in {
                'm골대후제자리우회전', 'm골대후제자리우회전(6회)'}:
            assert turn_end_values.keys() == recovery_end_values.keys()
            assert all(value == recovery_end_values[motor]
                       for motor, value in turn_end_values.items() if motor != '0')
        else:
            assert turn_end_values == recovery_end_values
        if forward is not None:
            assert forward['frames'][-1][field] == recovery['frames'][-1][field]
    if forward is not None:
        assert forward['repeat_count'] == 2 and recovery_blocks == 1
    result = copy.deepcopy(turn)
    # Keep composite command names stable when the source recovery is renamed.
    recovery_label = ('라인복귀우회전45도' if recovery['name'] == 't라인복귀우실전'
                      else recovery['name'][1:])
    result['name'] = turn['name'] + ('+' + recovery_label + '(4회)') * recovery_blocks
    # Regions 1-2 now recover right after the full left turn; keep the deployed name.
    if turn['name'] == 'p골대후제자리좌회전' and recovery['name'] == 'm라인복귀우회전45도':
        result['name'] = 'p골대후제자리좌회전+라인복귀좌회전45도(4회)'
    if forward is not None:
        result['name'] = FORWARD_EXIT_NAMES[turn['name']]
    turn_end = max(f['start_ms'] + f['time_ms'] for f in turn['frames'])
    # Frame times are scaled during playback, so encode a real 500 ms hold.
    recovery_start = turn_end + round(TURN_RECOVERY_DWELL_MS * turn['playback_speed'])
    stages = ([(forward, 1)] if forward is not None else []) + [(recovery, recovery_blocks)]
    for stage_index, (gait, blocks) in enumerate(stages):
        cycle_end = max(f['start_ms'] + f['time_ms'] for f in gait['frames'])
        lead_in = gait['frames'][0]['start_ms']
        scale = turn['playback_speed'] / gait['playback_speed']
        if stage_index:
            # Forward and recovery share support; keep the recovery's normal lead-in.
            recovery_start = (result['frames'][-1]['start_ms']
                              + result['frames'][-1]['time_ms'] + round(lead_in * scale))
        for cycle in range(gait['repeat_count'] * blocks):
            for source in gait['frames']:
                frame = copy.deepcopy(source)
                start = recovery_start + round((cycle * cycle_end + source['start_ms'] - lead_in) * scale)
                end = recovery_start + round((cycle * cycle_end + source['start_ms']
                                        + source['time_ms'] - lead_in) * scale)
                frame['start_ms'], frame['time_ms'] = start, end - start
                result['frames'].append(frame)
            result['frames'][-1]['playback_cycle_end'] = True
    result['max_seq_ms'] = result['frames'][-1]['start_ms'] + result['frames'][-1]['time_ms']
    result['end_pose'] = recovery['end_pose']
    return result


def main():
    paths = {kind: ROOT / f'artifacts/robot_motions_{kind}.json'
             for kind in ('pc', 'runtime')}
    catalogs = {kind: json.loads(path.read_text()) for kind, path in paths.items()}
    source = {motion['name']: motion for motion in catalogs['pc']['motions']}
    turns = [build_turn_variant(source[name], count) for name, count in (
        ('m골대후제자리우회전', 5), ('m골대후제자리우회전', 6),
        ('p골대후제자리좌회전', 4),
    )]
    source.update({motion['name']: motion for motion in turns})
    variants = turns + [build_exit(source[turn], source[recovery], blocks,
                                  forward=source['m전진45(2회)'] if turn in FORWARD_EXIT_NAMES else None)
                        for turn, recovery, blocks in GOAL_EXIT_RECOVERIES]
    obsolete = {
        't골대후제자리우회전(5회)+전진4회',
        'p골대후제자리좌회전(3회)+전진4회',
        't골대후제자리우회전(5회)+라인복귀좌회전45도(4회)',
        'p골대후제자리좌회전(3회)+라인복귀우회전45도(4회)',
        't골대후제자리우회전(5회)' + '+라인복귀우회전45도(4회)' * 2,
        'p골대후제자리좌회전(3회)' + '+라인복귀좌회전45도(4회)' * 2,
        't골대후제자리우회전(6회)' + '+라인복귀우회전45도(4회)' * 2,
        'p골대후제자리좌회전(4회)' + '+라인복귀좌회전45도(4회)' * 2,
    }
    for kind, catalog in catalogs.items():
        replacements = {m['name']: m if kind == 'pc' else runtime_motion_from_source(m)
                        for m in variants}
        catalog['motions'] = [m for m in catalog['motions']
                              if m['name'] not in replacements and m['name'] not in obsolete]
        catalog['motions'].extend(replacements.values())
        paths[kind].write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
