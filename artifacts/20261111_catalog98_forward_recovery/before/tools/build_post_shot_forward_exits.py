#!/usr/bin/env python3
"""Append four forward cycles after a half-second hold at the short goal exit."""

import copy
import json
from pathlib import Path

from upsert_motion_catalog import runtime_motion_from_source


ROOT = Path(__file__).resolve().parents[1]
EXIT_NAMES = (
    't골대후제자리우회전(5회)',
    'p골대후제자리좌회전(3회)',
)
TURN_FORWARD_DWELL_MS = 500


def build_exit(turn: dict, forward: dict) -> dict:
    """Keep turn timing and encode forward timing at the composite's speed."""
    assert turn['repeat_count'] == 1 and forward['repeat_count'] == 4
    # Both gaits use this support pose; never introduce a new joint target.
    for field in ('angles', 'torques'):
        assert turn['frames'][-1][field] == forward['frames'][-1][field]
    result = copy.deepcopy(turn)
    result['name'] = turn['name'] + '+전진4회'
    turn_end = max(f['start_ms'] + f['time_ms'] for f in turn['frames'])
    # Frame times are scaled during playback, so encode a real 500 ms hold.
    forward_start = turn_end + round(TURN_FORWARD_DWELL_MS * turn['playback_speed'])
    cycle_end = max(f['start_ms'] + f['time_ms'] for f in forward['frames'])
    lead_in = forward['frames'][0]['start_ms']
    scale = turn['playback_speed'] / forward['playback_speed']
    for cycle in range(4):
        for source in forward['frames']:
            frame = copy.deepcopy(source)
            # Replace the first forward lead-in with the explicit boundary hold.
            start = forward_start + round((cycle * cycle_end + source['start_ms'] - lead_in) * scale)
            end = forward_start + round((cycle * cycle_end + source['start_ms']
                                    + source['time_ms'] - lead_in) * scale)
            frame['start_ms'], frame['time_ms'] = start, end - start
            result['frames'].append(frame)
        result['frames'][-1]['playback_cycle_end'] = True
    result['max_seq_ms'] = result['frames'][-1]['start_ms'] + result['frames'][-1]['time_ms']
    result['end_pose'] = forward['end_pose']
    return result


def main():
    paths = {kind: ROOT / f'artifacts/robot_motions_{kind}.json'
             for kind in ('pc', 'runtime')}
    catalogs = {kind: json.loads(path.read_text()) for kind, path in paths.items()}
    source = {motion['name']: motion for motion in catalogs['pc']['motions']}
    variants = [build_exit(source[name], source['p전진45(4회)']) for name in EXIT_NAMES]
    for kind, catalog in catalogs.items():
        replacements = {m['name']: m if kind == 'pc' else runtime_motion_from_source(m)
                        for m in variants}
        catalog['motions'] = [m for m in catalog['motions'] if m['name'] not in replacements]
        catalog['motions'].extend(replacements.values())
        paths[kind].write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
