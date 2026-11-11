"""Replace right exits with source104 and rebuild their complete-cycle variants."""

import copy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
UPDATE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source
from build_post_shot_forward_exits import build_turn_variant, build_exit


def read(path):
    return json.loads(path.read_text())


def main():
    source_path = UPDATE / 'source_robot_motions.json'
    source = {m['name']: m for m in read(source_path)['motions']}
    pc = {m['name']: m for m in read(UPDATE / 'before/artifacts/robot_motions_pc.json')['motions']}
    renames = {'t공후 제자리우회전': 'm공후제자리우회전'}
    renames.update({name: name.replace('t골대후제자리우회전', 'm골대후제자리우회전')
                    for name in pc if name.startswith('t골대후제자리우회전')})
    variants = {}
    for name in ('m공후제자리우회전', 'm골대후제자리우회전'):
        motion = copy.deepcopy(source[name])
        # This source pose has different targets from the deployed shared pose.
        for frame in motion['frames']:
            if frame['name'] == '오뒤무게중심앞':
                frame['name'] += '(catalog104)'
        for key in ('start_pose', 'end_pose'):
            if motion[key] == '오뒤무게중심앞':
                motion[key] += '(catalog104)'
        variants[name] = motion
    for count in (5, 6):
        motion = build_turn_variant(variants['m골대후제자리우회전'], count)
        variants[motion['name']] = motion
    for name in ('m골대후제자리우회전', 'm골대후제자리우회전(6회)'):
        motion = build_exit(variants[name], pc['m라인복귀우회전45도'],
                            forward=pc['m전진45(2회)'] if name.endswith('(6회)') else None)
        variants[motion['name']] = motion
    assert set(variants) == set(renames.values())
    durations = {}
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        payload = read(UPDATE / 'before' / relative)
        merged = []
        for old in payload['motions']:
            motion = variants.get(renames.get(old['name']))
            if motion is None:
                merged.append(old)
                continue
            merged.append(runtime_motion_from_source(motion) if kind == 'runtime' else motion)
            durations[motion['name']] = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) / 1000 / motion['playback_speed']
        assert len({m['name'] for m in merged}) == len(merged)
        payload['motions'] = merged
        (ROOT / relative).write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n')
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    aliases = (UPDATE / 'before' / relative).read_text()
    for old, new in renames.items():
        aliases = aliases.replace('"' + old + '"', '"' + new + '"')
    (ROOT / relative).write_text(aliases)
    manifest = dict(source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                    renames=renames, duration_seconds=durations)
    (UPDATE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
