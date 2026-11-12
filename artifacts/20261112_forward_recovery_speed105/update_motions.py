"""Set deployed ordinary gaits to 1.05 and rebuild only embedded gait timing."""

import copy
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
UPDATE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source
from build_post_shot_forward_exits import build_exit, GOAL_EXIT_RECOVERIES, FORWARD_EXIT_NAMES


def read(path):
    return json.loads(path.read_text())


def end_ms(motion):
    return max(f['start_ms'] + f['time_ms'] for f in motion['frames'])


def main():
    pc = {m['name']: m for m in read(UPDATE / 'before/artifacts/robot_motions_pc.json')['motions']}
    gaits = {name: copy.deepcopy(motion) for name, motion in pc.items()
             if name.startswith(('m전진', 'm라인복귀'))}
    assert len(gaits) == 18
    for motion in gaits.values():
        motion['playback_speed'] = 1.05
    variants = dict(gaits)
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        motion = build_exit(pc[turn], gaits[recovery], blocks,
                            forward=gaits['m전진45(2회)'] if turn in FORWARD_EXIT_NAMES else None)
        assert motion['name'] in pc
        variants[motion['name']] = motion

    name = 'n공에서 전진하고 후진준비자세'
    prepare = copy.deepcopy(pc[name])
    gait = gaits['m전진0도(2회)']
    old_end = max(f['start_ms'] + f['time_ms'] for f in prepare['frames'][:4])
    frames = copy.deepcopy(gait['frames'])
    scale = prepare['playback_speed'] / gait['playback_speed']
    for frame in frames:
        start = round(frame['start_ms'] * scale)
        end = round((frame['start_ms'] + frame['time_ms']) * scale)
        frame['start_ms'], frame['time_ms'] = start, end - start
    new_end = max(f['start_ms'] + f['time_ms'] for f in frames)
    # Shift the preparation tail without changing its duration, gap, or targets.
    for frame in prepare['frames'][4:]:
        frame['start_ms'] += new_end - old_end
        frames.append(frame)
    prepare['frames'] = frames
    prepare['max_seq_ms'] = end_ms(prepare)
    variants[name] = prepare
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        payload = read(UPDATE / 'before' / relative)
        payload['motions'] = [
            (runtime_motion_from_source(variants[m['name']]) if kind == 'runtime'
             else variants[m['name']]) if m['name'] in variants else m
            for m in payload['motions']
        ]
        (ROOT / relative).write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')
    manifest = {'gaits': list(gaits), 'replaced': list(variants), 'playback_speed': 1.05,
                'duration_seconds': {name: end_ms(m) * m['repeat_count'] / m['playback_speed'] / 1000
                                     for name, m in variants.items()}}
    (UPDATE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
