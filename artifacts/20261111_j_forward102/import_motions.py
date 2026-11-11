"""Derive all ordinary forward gaits from source102, preserving counts and heads."""

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
    source = read(source_path)['motions']
    matches = [m for m in source if m['name'] == 'j전진45(6회)']
    assert len(matches) == 1
    original = matches[0]
    before = read(UPDATE / 'before/artifacts/robot_motions_pc.json')
    pc = {m['name']: m for m in before['motions']}
    renames, variants = {}, {}
    for old in before['motions']:
        if not old['name'].startswith('m전진'):
            continue
        camera = int(old['name'].split('전진')[1].split('(')[0].removesuffix('도'))
        name = 'j' + old['name'][1:]
        motion = copy.deepcopy(original)
        motion.update(name=name, repeat_count=old['repeat_count'])
        for index, frame in enumerate(motion['frames']):
            frame['angles']['0'] = old['frames'][index]['angles']['0']
            if camera == 45:
                if frame['name'] == '오뒤무게중심앞':
                    frame['name'] += '(catalog80)'
            elif index in (0, 2):
                # Camera variants need distinct pose identities for different head angles.
                frame['name'] = f'{original["frames"][index]["name"]}(j전진{camera}도)'
            else:
                frame['name'] = old['frames'][index]['name']
        motion['start_pose'] = motion['frames'][0]['name']
        motion['end_pose'] = motion['frames'][-1]['name']
        assert motion['frames'][-1]['angles'] == old['frames'][-1]['angles']
        renames[old['name']] = name
        variants[name] = motion

    # One forward cycle is embedded before the existing two preparation frames.
    name = 'n공에서 전진하고 후진준비자세'
    prepare = copy.deepcopy(pc[name])
    gait = variants['j전진0도(2회)']
    assert prepare['playback_speed'] == gait['playback_speed']
    assert len(prepare['frames']) == 6
    for old, new in zip(prepare['frames'][:4], gait['frames']):
        assert (old['start_ms'], old['time_ms']) == (new['start_ms'], new['time_ms'])
    prepare['frames'][:4] = copy.deepcopy(gait['frames'])
    prepare['start_pose'] = gait['start_pose']
    variants[name] = prepare
    for turn, recovery, blocks in GOAL_EXIT_RECOVERIES:
        if turn in FORWARD_EXIT_NAMES:
            motion = build_exit(pc[turn], pc[recovery], blocks, forward=variants['j전진45(2회)'])
            assert motion['name'] in pc
            variants[motion['name']] = motion

    manifest = dict(source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),
                    source_motion=original['name'], renames=renames,
                    replaced=list(variants), duration_seconds={})
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        payload = read(UPDATE / 'before' / relative)
        merged = []
        for old in payload['motions']:
            name = renames.get(old['name'], old['name'])
            replacement = variants.get(name)
            motion = old if replacement is None else (
                runtime_motion_from_source(replacement) if kind == 'runtime' else replacement)
            merged.append(motion)
            if replacement is not None and kind == 'runtime':
                manifest['duration_seconds'][name] = max(f['start_ms'] + f['time_ms'] for f in motion['frames']) * motion['repeat_count'] / motion['playback_speed'] / 1000
        assert len({m['name'] for m in merged}) == len(merged)
        payload['motions'] = merged
        write(ROOT / relative, payload)
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    aliases = (UPDATE / 'before' / relative).read_text()
    for old, new in renames.items():
        aliases = aliases.replace('"' + old + '"', '"' + new + '"')
    aliases = aliases.replace('Fast-forward m names', 'Source102 j forward names')
    (ROOT / relative).write_text(aliases)
    write(UPDATE / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
