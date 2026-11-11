"""Import source102 m gaits with stable command counts and composite names."""

import copy
import hashlib
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


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def end_ms(motion):
    return max(f['start_ms'] + f['time_ms'] for f in motion['frames'])


def main():
    source_path = UPDATE / 'source_robot_motions.json'
    raw = read(source_path)['motions']
    source = {m['name']: m for m in raw}
    assert len(source) == len(raw)
    pc = {m['name']: m for m in read(UPDATE / 'before/artifacts/robot_motions_pc.json')['motions']}
    renames = {}
    for camera, counts in [(45, (2, 4, 6, 8)), (90, (1, 2, 3, 4, 6)), (0, (2, 4, 6))]:
        for count in counts:
            old = (f'p전진45({count}회)' if camera == 45 and count != 2
                   else f'p전진실전0도({count}회)' if camera == 0 and count != 2
                   else f'p전진{camera}도({count}회)')
            renames[old] = f'm전진{camera}{"" if camera == 45 else "도"}({count}회)'
    renames.update({
        'p라인복귀좌회전45도': 'm라인복귀좌회전45도',
        'p라인복귀좌회전90도': 'm라인복귀좌회전90도',
        'p라인복귀좌회전공용': 'm라인복귀좌회전공용',
        't라인복귀우실전': 'm라인복귀우회전45도',
        't라인복귀우실전90도': 'm라인복귀우회전90',
        't라인복귀우공용': 'm라인복귀우회전공용',
    })
    variants, source_mapping = {}, {}
    for old, name in renames.items():
        original = 'm전진45(6회)' if name == 'm전진45(8회)' else name
        motion = copy.deepcopy(source[original])
        count = pc[old]['repeat_count']
        assert name == 'm전진45(8회)' or count == motion['repeat_count']
        motion.update(name=name, repeat_count=count)
        if name == 'm라인복귀우회전90':
            # The source alternates camera-down and camera90 targets in this variant.
            for frame, previous in zip(motion['frames'], pc[old]['frames'], strict=True):
                frame['angles']['0'] = previous['angles']['0']
        assert [f['angles']['0'] for f in motion['frames']] == [f['angles']['0'] for f in pc[old]['frames']]
        for field in ('angles', 'torques'):
            assert motion['frames'][-1][field] == pc[old]['frames'][-1][field]
        variants[name] = motion
        source_mapping[name] = original

    name = 'n공에서 전진하고 후진준비자세'
    prepare = copy.deepcopy(pc[name])
    gait = variants['m전진0도(2회)']
    old_end = max(f['start_ms'] + f['time_ms'] for f in prepare['frames'][:4])
    scale = prepare['playback_speed'] / gait['playback_speed']
    frames = copy.deepcopy(gait['frames'])
    for frame in frames:
        start = round(frame['start_ms'] * scale)
        end = round((frame['start_ms'] + frame['time_ms']) * scale)
        frame['start_ms'], frame['time_ms'] = start, end - start
    new_end = frames[-1]['start_ms'] + frames[-1]['time_ms']
    # Retain the preparation gap, joint targets, and effective tail duration.
    for frame in prepare['frames'][4:]:
        frame['start_ms'] += new_end - old_end
        frames.append(frame)
    prepare['frames'] = frames
    prepare['start_pose'] = gait['start_pose']
    prepare['max_seq_ms'] = end_ms(prepare)
    variants[name] = prepare

    for motion in variants.values():
        for frame in motion['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog80)'
        for key in ('start_pose', 'end_pose'):
            if motion[key] == '오뒤무게중심앞':
                motion[key] += '(catalog80)'
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        motion = build_exit(pc[turn], variants[recovery], blocks,
                            forward=variants['m전진45(2회)'] if turn in FORWARD_EXIT_NAMES else None)
        assert motion['name'] in pc
        variants[motion['name']] = motion

    manifest = dict(source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                    renames=renames, source_mapping=source_mapping,
                    camera_overrides={'m라인복귀우회전90': {'motor_id': 0, 'angles': [1.0] * 4}},
                    replaced=list(variants), duration_seconds={})
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        payload = read(UPDATE / 'before' / relative)
        merged = []
        for old in payload['motions']:
            replacement = variants.get(renames.get(old['name'], old['name']))
            motion = old if replacement is None else (
                runtime_motion_from_source(replacement) if kind == 'runtime' else replacement)
            merged.append(motion)
            if replacement is not None and kind == 'runtime':
                manifest['duration_seconds'][motion['name']] = end_ms(motion) * motion['repeat_count'] / motion['playback_speed'] / 1000
        assert len({m['name'] for m in merged}) == len(merged)
        payload['motions'] = merged
        write(ROOT / relative, payload)
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    aliases = (UPDATE / 'before' / relative).read_text()
    for old, new in renames.items():
        aliases = aliases.replace('"' + old + '"', '"' + new + '"')
    aliases = aliases.replace('Source101 p forward names', 'Source102 m forward names')
    aliases = aliases.replace('Source101 p/t recovery names', 'Source102 m recovery names')
    aliases = aliases.replace('then eight opposite recovery cycles.', 'forward two, then same-direction recovery four.')
    (ROOT / relative).write_text(aliases)
    write(UPDATE / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
