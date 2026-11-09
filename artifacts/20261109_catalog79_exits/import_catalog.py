"""Import the three requested source79 motions from a preserved baseline."""

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

RENAMES = {
    't공후 제자리좌회전': 'p공후 제자리좌회전',
    't골대후 제자리좌회전': 'p골대후제자리좌회전',
    'p라인복귀좌회전공용': 'p라인복귀좌회전공용',
}
ALIAS_PATH = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')


def main():
    source_path = UPDATE / 'source_robot_motions.json'
    source = {m['name']: m for m in json.loads(source_path.read_text())['motions']}
    baseline = json.loads((UPDATE / 'before/artifacts/robot_motions_runtime.json').read_text())
    retained = [m for m in baseline['motions'] if m['name'] not in RENAMES]
    pose_renames = {}
    for name in RENAMES.values():
        for frame in source[name]['frames']:
            if any(f['name'] == frame['name']
                   and (f['angles'] != frame['angles'] or f['torques'] != frame['torques'])
                   for m in retained for f in m['frames']):
                pose_renames[frame['name']] = frame['name'] + '(catalog79)'
    for kind in ('pc', 'runtime'):
        relative = Path(f'artifacts/robot_motions_{kind}.json')
        catalog = json.loads((UPDATE / 'before' / relative).read_text())
        for index, old in enumerate(catalog['motions']):
            if old['name'] not in RENAMES:
                continue
            motion = copy.deepcopy(source[RENAMES[old['name']]])
            if kind == 'runtime':
                motion = runtime_motion_from_source(motion)
            for frame in motion['frames']:
                frame['name'] = pose_renames.get(frame['name'], frame['name'])
            for key in ('start_pose', 'end_pose'):
                motion[key] = pose_renames.get(motion[key], motion[key])
            catalog['motions'][index] = motion
        (ROOT / relative).write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')
    alias_text = (UPDATE / 'before' / ALIAS_PATH).read_text()
    previous = yaml.safe_load(alias_text)['motion_aliases']
    for old, new in RENAMES.items():
        alias_text = alias_text.replace(f'"{old}"', f'"{new}"')
    (ROOT / ALIAS_PATH).write_text(alias_text)
    aliases = yaml.safe_load(alias_text)['motion_aliases']
    manifest = {
        'source_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest(),
        'renames': RENAMES, 'pose_renames': pose_renames,
        'changed_aliases': {a: {'before': previous[a], 'after': n}
                            for a, n in aliases.items() if n != previous[a]},
        'runtime_speed_policy': 'Original playback_speed multiplied by 0.95 once.',
        'route_policy': 'First pickup/goal exits remain right; second pickup/goal exits remain left.',
    }
    (UPDATE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
