"""Import the requested m gaits and region-8 shot from immutable snapshots."""

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
    renames = {
        'p라인복귀좌회전45도': 'm라인복귀좌회전45도',
        'p라인복귀좌회전90도': 'm라인복귀좌회전90도',
        'p라인복귀좌회전공용': 'm라인복귀좌회전공용',
        't라인복귀우실전': 'm라인복귀우회전45도',
        't라인복귀우실전90도': 'm라인복귀우회전90',
        't라인복귀우공용': 'm라인복귀우회전공용',
    }
    for camera, counts in [(45, (2, 4, 6, 8)), (90, (1, 2, 3, 4, 6)), (0, (2, 4, 6))]:
        for count in counts:
            old = (f'p전진45({count}회)' if camera == 45 and count != 2
                   else f'p전진실전0도({count}회)' if camera == 0 and count != 2
                   else f'p전진{camera}도({count}회)')
            new = f'm전진{camera}{"" if camera == 45 else "도"}({count}회)'
            renames[old] = new
    replacements = {}
    for name in [*renames.values(), 'r골넣기8번구역']:
        original = 'm전진45(6회)' if name == 'm전진45(8회)' else name
        motion = copy.deepcopy(source[original])
        motion['name'] = name
        if name == 'm전진45(8회)':
            motion['repeat_count'] = 8
        # Preserve the deployed identity for this shared support pose.
        for frame in motion['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog80)'
        for key in ('start_pose', 'end_pose'):
            if motion[key] == '오뒤무게중심앞':
                motion[key] += '(catalog80)'
        replacements[name] = motion
    pc = {m['name']: m for m in read(UPDATE / 'before/artifacts/robot_motions_pc.json')['motions']}
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        motion = build_exit(pc[turn], replacements[recovery], blocks,
                            forward=replacements['m전진45(2회)'] if turn in FORWARD_EXIT_NAMES else None)
        assert motion['name'] in pc
        replacements[motion['name']] = motion
    manifest = dict(source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                    renames=renames, replaced=list(replacements),
                    derived={'m전진45(8회)': ['m전진45(6회)', 8]}, duration_seconds={})
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        catalog = read(UPDATE / 'before' / relative)
        assert set(renames) <= {m['name'] for m in catalog['motions']}
        merged = []
        for old in catalog['motions']:
            name = renames.get(old['name'], old['name'])
            replacement = replacements.get(name)
            motion = old if replacement is None else (
                runtime_motion_from_source(replacement) if kind == 'runtime' else replacement)
            merged.append(motion)
            if replacement is not None and kind == 'runtime':
                manifest['duration_seconds'][name] = (
                    max(f['start_ms'] + f['time_ms'] for f in motion['frames'])
                    * motion['repeat_count'] / motion['playback_speed'] / 1000)
        catalog['motions'] = merged
        assert len({m['name'] for m in merged}) == len(merged)
        write(ROOT / relative, catalog)
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    aliases = (UPDATE / 'before' / relative).read_text()
    for old, new in renames.items():
        aliases = aliases.replace('"' + old + '"', '"' + new + '"')
    aliases = aliases.replace('Source100 p forward names', 'Fast-forward m names')
    aliases = aliases.replace('Source100 p/t recovery names', 'Fast-forward m recovery names')
    (ROOT / relative).write_text(aliases)
    write(UPDATE / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
