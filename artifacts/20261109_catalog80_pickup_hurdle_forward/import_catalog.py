"""Import source80 pickup, hurdle and forward data from the saved baseline."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import yaml

UPDATE = Path(__file__).resolve().parent
ROOT = UPDATE.parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source

DERIVED = {
    'p전진45(2회)': ('p전진45(4회)', 2),
    'p전진45(8회)': ('p전진45(4회)', 8),
    'p전진90도(2회)': ('p전진90도(4회)', 2),
}


def main():
    source_path = UPDATE / 'source_robot_motions.json'
    source = json.loads(source_path.read_text())['motions']
    assert len({m['name'] for m in source}) == len(source)
    imports = {m['name']: copy.deepcopy(m) for m in source
               if m['name'].startswith('p전진')
               or m['name'] in {'r공잡기 400', 'r공잡기500', 't허들찐'}}
    assert len(imports) == 9
    for name, (original, count) in DERIVED.items():
        imports[name] = {**copy.deepcopy(imports[original]),
                         'name': name, 'repeat_count': count}
    baseline = json.loads((UPDATE / 'before/artifacts/robot_motions_runtime.json').read_text())
    retained = [m for m in baseline['motions'] if m['name'] not in imports]
    pose_renames = {}
    for motion in imports.values():
        for frame in runtime_motion_from_source(motion)['frames']:
            if any(f['name'] == frame['name']
                   and (f['angles'], f['torques']) != (frame['angles'], frame['torques'])
                   for m in retained for f in m['frames']):
                pose_renames[frame['name']] = frame['name'] + '(catalog80)'
    durations = {}
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        catalog = json.loads((UPDATE / 'before' / relative).read_text())
        assert set(imports) <= {m['name'] for m in catalog['motions']}
        for index, old in enumerate(catalog['motions']):
            if old['name'] not in imports:
                continue
            motion = copy.deepcopy(imports[old['name']])
            if kind == 'runtime':
                motion = runtime_motion_from_source(motion)
            for frame in motion['frames']:
                frame['name'] = pose_renames.get(frame['name'], frame['name'])
            for key in ('start_pose', 'end_pose'):
                motion[key] = pose_renames.get(motion[key], motion[key])
            catalog['motions'][index] = motion
            if kind == 'runtime':
                seconds = lambda m: max(f['start_ms'] + f['time_ms'] for f in m['frames']) * m['repeat_count'] / m['playback_speed'] / 1000
                durations[motion['name']] = {'before': seconds(old), 'after': seconds(motion)}
        (ROOT / relative).write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')
    alias_path = ROOT / 'src/irc_step_motion_executor/config/motion_aliases.yaml'
    aliases = yaml.safe_load(alias_path.read_text())['motion_aliases']
    assert set(aliases.values()) <= {m['name'] for m in catalog['motions']}
    manifest = {
        'source_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest(),
        'imports': list(imports), 'derived': DERIVED, 'pose_renames': pose_renames,
        'renames': {}, 'changed_aliases': {},
        'affected_aliases': {a: n for a, n in aliases.items() if n in imports},
        'runtime_speed_policy': 'Original speed multiplied by 0.95 once.',
        'duration_seconds': durations,
    }
    (UPDATE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
