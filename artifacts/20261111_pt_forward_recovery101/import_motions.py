"""Import source101 p/t gaits with stable command counts and composite names."""

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
    renames = {
        "m전진45(2회)": "p전진45도(2회)",
        "m전진45(4회)": "p전진45(4회)",
        "m전진45(6회)": "p전진45(6회)",
        "m전진45(8회)": "p전진45(8회)",
        "m전진90도(1회)": "p전진90도(1회)",
        "m전진90도(2회)": "p전진90도(2회)",
        "m전진90도(3회)": "p전진90도(3회)",
        "m전진90도(4회)": "p전진90도(4회)",
        "m전진90도(6회)": "p전진90도(6회)",
        "m전진0도(2회)": "p전진0도(2회)",
        "m전진0도(4회)": "p전진실전0도(4회)",
        "m전진0도(6회)": "p전진실전0도(6회)",
        "m라인복귀좌회전45도": "p라인복귀좌회전45도",
        "m라인복귀좌회전90도": "p라인복귀좌회전90도",
        "m라인복귀좌회전공용": "p라인복귀좌회전공용",
        "m라인복귀우회전45도": "t라인복귀우실전",
        "m라인복귀우회전90": "t라인복귀우실전90도",
        "m라인복귀우회전공용": "t라인복귀우공용"
    }
    variants, source_mapping = {}, {}
    for old, name in renames.items():
        original = 'p전진45(6회)' if name == 'p전진45(8회)' else name
        motion = copy.deepcopy(source[original])
        count = pc[old]['repeat_count']
        assert name in ('p전진45(8회)', 'p전진0도(2회)') or count == motion['repeat_count']
        motion.update(name=name, repeat_count=count)
        assert [f['angles']['0'] for f in motion['frames']] == [f['angles']['0'] for f in pc[old]['frames']]
        for field in ('angles', 'torques'):
            assert motion['frames'][-1][field] == pc[old]['frames'][-1][field]
        variants[name] = motion
        source_mapping[name] = original

    name = 'n공에서 전진하고 후진준비자세'
    prepare = copy.deepcopy(source[name])
    assert prepare['frames'][:4] == variants['p전진0도(2회)']['frames']
    for actual, previous in zip(prepare['frames'][4:], pc[name]['frames'][4:], strict=True):
        for field in ('angles', 'torques', 'time_ms'):
            assert actual[field] == previous[field]
    variants[name] = prepare
    source_mapping[name] = name

    for motion in variants.values():
        for frame in motion['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog80)'
        for key in ('start_pose', 'end_pose'):
            if motion[key] == '오뒤무게중심앞':
                motion[key] += '(catalog80)'
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        motion = build_exit(pc[turn], variants[recovery], blocks,
                            forward=variants['p전진45도(2회)'] if turn in FORWARD_EXIT_NAMES else None)
        assert motion['name'] in pc
        variants[motion['name']] = motion

    manifest = dict(source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                    renames=renames, source_mapping=source_mapping,
                    count_overrides={'p전진0도(2회)': {'source': 3, 'command': 2},
                                     'p전진45(8회)': {'source': 6, 'command': 8}},
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
    aliases = aliases.replace('Source102 m forward names', 'Source101 p forward names')
    aliases = aliases.replace('Source102 m recovery names', 'Source101 p/t recovery names')
    aliases = aliases.replace('then eight opposite recovery cycles.', 'forward two, then same-direction recovery four.')
    (ROOT / relative).write_text(aliases)
    write(UPDATE / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
