"""Import source101 p forward gaits while preserving public command counts."""

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


def main():
    source_path = UPDATE / 'source_robot_motions.json'
    raw = read(source_path)['motions']
    source = {m['name']: m for m in raw}
    assert len(source) == len(raw)
    pc = {m['name']: m for m in read(UPDATE / 'before/artifacts/robot_motions_pc.json')['motions']}
    renames, variants, source_mapping, count_overrides = {}, {}, {}, {}
    for camera, counts in [(45, (2, 4, 6, 8)), (90, (1, 2, 3, 4, 6)), (0, (2, 4, 6))]:
        for count in counts:
            old = f'j전진{camera}{"" if camera == 45 else "도"}({count}회)'
            name = (f'p전진45({count}회)' if camera == 45 and count != 2
                    else f'p전진실전0도({count}회)' if camera == 0 and count != 2
                    else f'p전진{camera}도({count}회)')
            original = 'p전진45(6회)' if count == 8 else name
            motion = copy.deepcopy(source[original])
            assert pc[old]['repeat_count'] == count
            if motion['repeat_count'] != count:
                count_overrides[name] = {'source': motion['repeat_count'], 'command': count}
            motion.update(name=name, repeat_count=count)
            assert [f['angles']['0'] for f in motion['frames']] == [f['angles']['0'] for f in pc[old]['frames']]
            for field in ('angles', 'torques'):
                assert motion['frames'][-1][field] == pc[old]['frames'][-1][field]
            renames[old] = name
            source_mapping[name] = original
            variants[name] = motion
    name = 'n공에서 전진하고 후진준비자세'
    prepare = copy.deepcopy(source[name])
    assert prepare['frames'][4:] == pc[name]['frames'][4:]
    assert prepare['frames'][:4] == variants['p전진0도(2회)']['frames']
    variants[name] = prepare
    source_mapping[name] = name
    for motion in variants.values():
        # Keep the existing shared support-pose identity without changing targets.
        for frame in motion['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog80)'
        for key in ('start_pose', 'end_pose'):
            if motion[key] == '오뒤무게중심앞':
                motion[key] += '(catalog80)'
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        if turn in FORWARD_EXIT_NAMES:
            motion = build_exit(pc[turn], pc[recovery], blocks, forward=variants['p전진45도(2회)'])
            assert motion['name'] in pc
            variants[motion['name']] = motion
    manifest = dict(source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                    renames=renames, source_mapping=source_mapping, count_overrides=count_overrides,
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
                def duration(m):
                    return max(f['start_ms'] + f['time_ms'] for f in m['frames']) * m['repeat_count'] / m['playback_speed'] / 1000
                assert duration(motion) == duration(old)
                manifest['duration_seconds'][motion['name']] = duration(motion)
        assert len({m['name'] for m in merged}) == len(merged)
        payload['motions'] = merged
        write(ROOT / relative, payload)
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    aliases = (UPDATE / 'before' / relative).read_text()
    for old, new in renames.items():
        aliases = aliases.replace('"' + old + '"', '"' + new + '"')
    aliases = aliases.replace('Source102 j forward names', 'Source101 p forward names')
    (ROOT / relative).write_text(aliases)
    write(UPDATE / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
