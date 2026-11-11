"""Import source101 recovery data and rebuild embedded recovery tails."""

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

RENAMES = {
    'm라인복귀우회전45도': 't라인복귀우실전',
    'm라인복귀우회전90': 't라인복귀우실전90도',
    'm라인복귀우회전공용': 't라인복귀우공용',
    'm라인복귀좌회전45도': 'p라인복귀좌회전45도',
    'm라인복귀좌회전90도': 'p라인복귀좌회전90도',
    'm라인복귀좌회전공용': 'p라인복귀좌회전공용',
}


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    source_path = UPDATE / 'source_robot_motions.json'
    raw = read(source_path)['motions']
    source = {m['name']: m for m in raw}
    assert len(raw) == len(source)
    pc = {m['name']: m for m in read(UPDATE / 'before/artifacts/robot_motions_pc.json')['motions']}
    variants = {}
    for old, name in RENAMES.items():
        motion = copy.deepcopy(source[name])
        assert motion['repeat_count'] == pc[old]['repeat_count'] == 4
        for field in ('angles', 'torques'):
            assert motion['frames'][-1][field] == pc[old]['frames'][-1][field]
        # Retain the deployed shared support-pose identity, with source targets.
        for frame in motion['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog80)'
        for key in ('start_pose', 'end_pose'):
            if motion[key] == '오뒤무게중심앞':
                motion[key] += '(catalog80)'
        variants[name] = motion
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        motion = build_exit(pc[turn], variants[recovery], blocks,
                            forward=pc['j전진45(2회)'] if turn in FORWARD_EXIT_NAMES else None)
        assert motion['name'] in pc
        variants[motion['name']] = motion
    manifest = dict(source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                    renames=RENAMES, replaced=list(variants), duration_seconds={})
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        payload = read(UPDATE / 'before' / relative)
        merged = []
        for old in payload['motions']:
            replacement = variants.get(RENAMES.get(old['name'], old['name']))
            motion = old if replacement is None else (
                runtime_motion_from_source(replacement) if kind == 'runtime' else replacement)
            merged.append(motion)
            if replacement is not None and kind == 'runtime':
                manifest['duration_seconds'][motion['name']] = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000
        assert len({m['name'] for m in merged}) == len(merged)
        payload['motions'] = merged
        write(ROOT / relative, payload)
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    aliases = (UPDATE / 'before' / relative).read_text()
    for old, new in RENAMES.items():
        aliases = aliases.replace('"' + old + '"', '"' + new + '"')
    aliases = aliases.replace('Fast-forward m recovery names', 'Source101 p/t recovery names')
    (ROOT / relative).write_text(aliases)
    write(UPDATE / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
