"""Rebuild source98 walking variants from the saved pre-import catalogs."""

import copy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
UPDATE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source
from build_post_shot_forward_exits import build_exit, EXIT_NAMES


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    source = {m['name']: m for m in read(UPDATE / 'source_robot_motions.json')['motions']}
    renames = {
        **{f'p전진45({n}회)': f'n전진45도({n}회)' for n in (2, 4, 6, 8)},
        **{f'p전진90도({n}회)': f'n전진90도({n}회)' for n in (2, 4, 6)},
        **{f'p전진실전0도({n}회)': f'n전진0도({n}회)' for n in (4, 6)},
        'p라인복귀좌회전45도': 'n라인복귀좌회전45도',
        'p라인복귀좌회전90도': 'n라인복귀좌회전90도',
        'p라인복귀좌회전공용': 'n라인복귀좌회전공용',
        't라인복귀우실전': 'n라인복귀우회전45도',
        't라인복귀우실전90도': 'n라인복귀우회전90도',
        't라인복귀우공용': 'n라인복귀우회전공용',
    }
    variants = {n: copy.deepcopy(m) for n, m in source.items()
                if n.startswith(('n전진', 'n라인복귀'))}
    derived = {}
    # The supplied six-cycle label contains repeat_count=4. Preserve command counts.
    for count in (6, 8):
        name = f'n전진45도({count}회)'
        variants[name] = copy.deepcopy(source['n전진45도(4회)'])
        variants[name].update(name=name, repeat_count=count)
        derived[name] = ['n전진45도(4회)', count]
    # Keep the startup pose unambiguous without changing any supplied joint values.
    pose_renames = {'오뒤무게중심앞': '오뒤무게중심앞(catalog80)'}
    for motion in variants.values():
        for frame in motion['frames']:
            frame['name'] = pose_renames.get(frame['name'], frame['name'])
        for key in ('start_pose', 'end_pose'):
            motion[key] = pose_renames.get(motion[key], motion[key])
    duration = lambda m: max(f['start_ms'] + f['time_ms'] for f in m['frames']) * m['repeat_count'] / m['playback_speed'] / 1000
    manifest = {'source_sha256': hashlib.sha256((UPDATE / 'source_robot_motions.json').read_bytes()).hexdigest(),
                'renames': renames, 'derived': derived, 'pose_renames': pose_renames,
                'imports': list(variants), 'duration_seconds': {}}
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        payload = read(UPDATE / 'before' / relative)
        before = {m['name']: m for m in payload['motions']}
        pc = {m['name']: m for m in read(UPDATE / 'before/artifacts/robot_motions_pc.json')['motions']}
        replacements = dict(variants)
        for name in EXIT_NAMES:
            motion = build_exit(pc[name], variants['n전진45도(4회)'])
            replacements[motion['name']] = motion
        merged = []
        for old in payload['motions']:
            name = renames.get(old['name'], old['name'])
            replacement = replacements.pop(name, None)
            merged.append(old if replacement is None else (
                runtime_motion_from_source(replacement) if kind == 'runtime' else replacement))
        merged.extend(runtime_motion_from_source(m) if kind == 'runtime' else m
                      for m in replacements.values())
        payload['motions'] = merged
        write(ROOT / relative, payload)
        if kind == 'runtime':
            for m in merged:
                old_name = next((old for old, new in renames.items() if new == m['name']), m['name'])
                if m['name'] in variants or m['name'].endswith('+전진4회'):
                    manifest['duration_seconds'][m['name']] = {
                        'before': duration(before[old_name]) if old_name in before else None,
                        'after': duration(m)}
    relative = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
    aliases = (UPDATE / 'before' / relative).read_text()
    for old, new in renames.items():
        aliases = aliases.replace('"' + old + '"', '"' + new + '"')
    aliases = aliases.replace('Catalog77 p forward variants; derived 2/8-cycle', 'Catalog98 n forward variants; derived 6/8-cycle')
    aliases = aliases.replace('Catalog52 R recovery motions', 'Catalog98 n recovery motions')
    (ROOT / relative).write_text(aliases)
    write(UPDATE / 'manifest.json', manifest)


if __name__ == '__main__':
    main()
