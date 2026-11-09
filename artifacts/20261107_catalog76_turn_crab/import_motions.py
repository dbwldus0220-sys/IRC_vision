"""Import the requested source76 motions from the preserved pre-update catalog."""

import copy
import hashlib
import json
from pathlib import Path
import sys

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source


def read(path):
    return json.loads(path.read_text())


source = {m['name']: m for m in read(HERE / 'source_robot_motions.json')['motions']}
turns = [f't제자리좌회전{camera}도({count}회)'
         for camera in (0, 45, 90) for count in range(1, 7)]
renames = {
    **{name: name for name in turns},
    '유미세왼옆꽃게0도': 'tt미세왼옆0도',
    'r미세왼옆꽃게90도': 'tt미세왼옆90도',
    'r미세왼옆꽃개0도(2회)': 'tt미세왼옆0도(2회)',
}
shared_entries = {
    'r미세오뒤에서 왼옆0도': 'r미세오뒤에서 오옆0도',
    'r오뒤에서 왼옆0도': 'r오뒤에서 오옆꽃게0도',
    'r미세오뒤에서왼옆90도': 'r미세오뒤에서 오옆 90도',
    'r오뒤에서 왼옆으로90도': 'r오뒤에서 오옆꽃게90도',
}
direction_changes = {f'r{side}옆에서 {other}옆{camera}도'
                     for side, other in [('오', '왼'), ('왼', '오')]
                     for camera in (0, 90)}
removed = set(shared_entries) | direction_changes | {'유미세왼옆꽃게90도'}
replacements = {}
for name in renames.values():
    motion = copy.deepcopy(source['tt미세왼옆0도' if name == 'tt미세왼옆0도(2회)' else name])
    motion['name'] = name
    if name == 'tt미세왼옆0도(2회)':
        motion['repeat_count'] = 2
    if name == 't제자리좌회전45도(6회)':
        motion['playback_speed'] = .90
    replacements[name] = motion

# Pose lookup is global: isolate changed targets without altering their angles.
before = read(HERE / 'before/artifacts/robot_motions_pc.json')
retained = [m for m in before['motions'] if m['name'] not in set(renames) | removed]
pose_renames = {}
for motion in replacements.values():
    for frame in motion['frames']:
        if any(old['name'] == frame['name'] and
               (old['angles'] != frame['angles'] or old['torques'] != frame['torques'])
               for m in retained for old in m['frames']):
            pose_renames[frame['name']] = frame['name'] + '(catalog76)'
for motion in replacements.values():
    for frame in motion['frames']:
        frame['name'] = pose_renames.get(frame['name'], frame['name'])
    for key in ('start_pose', 'end_pose'):
        motion[key] = pose_renames.get(motion[key], motion[key])

for kind in ('pc', 'runtime'):
    relative = Path(f'artifacts/robot_motions_{kind}.json')
    catalog = read(HERE / 'before' / relative)
    motions = []
    for old in catalog['motions']:
        if old['name'] in removed:
            continue
        if old['name'] in renames:
            motion = copy.deepcopy(replacements[renames[old['name']]])
            if kind == 'runtime':
                source_speed = motion['playback_speed']
                motion = runtime_motion_from_source(motion)
                if not motion['name'].startswith('t제자리좌회전'):
                    # The 0.95 multiplier applies only to the requested left turns.
                    motion['playback_speed'] = source_speed
            motions.append(motion)
        else:
            motions.append(old)
    catalog['motions'] = motions
    (ROOT / relative).write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + '\n')

alias_path = Path('src/irc_step_motion_executor/config/motion_aliases.yaml')
alias_text = (HERE / 'before' / alias_path).read_text()
old_aliases = yaml.safe_load(alias_text)['motion_aliases']
deleted_aliases = [alias for alias, name in old_aliases.items() if name in direction_changes]
alias_text = ''.join(line for line in alias_text.splitlines(keepends=True)
                     if not any(line.lstrip().startswith(alias + ':') for alias in deleted_aliases))
for old, new in {**renames, **shared_entries}.items():
    alias_text = alias_text.replace(f'"{old}"', f'"{new}"')
alias_text = alias_text.replace('# Catalog75 t left-turn gait:', '# Catalog76 t left-turn gait:')
alias_text = alias_text.replace('# Catalog63 uses separate right/left support-change motions.',
                                '# Catalog76 left/right crabs share the right-crab entry posture.')
alias_text = alias_text.replace('# Catalog63 directional crab transitions.',
                                '# Preserve directional IDs; both entries use the right-crab motion.')
(ROOT / alias_path).write_text(alias_text)
aliases = yaml.safe_load(alias_text)['motion_aliases']
assert set(aliases.values()) <= {m['name'] for m in motions}
manifest = {
    'source_sha256': hashlib.sha256((HERE / 'source_robot_motions.json').read_bytes()).hexdigest(),
    'renames': renames, 'shared_entries': shared_entries,
    'removed_motions': sorted(removed), 'removed_aliases': deleted_aliases,
    'pose_renames': pose_renames,
    'source_speed_override': {'t제자리좌회전45도(6회)': .9},
    'runtime_turn_speed': .855, 'runtime_left_crab_speed': .95,
    'changed_aliases': {a: n for a, n in aliases.items() if old_aliases.get(a) != n},
}
(HERE / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
