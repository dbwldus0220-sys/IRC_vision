"""Apply the catalog52 R family, preserving existing command repeat counts."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from upsert_motion_catalog import _with_runtime_policy

OUT = Path(__file__).resolve().parent
source_path = ROOT/'motion_imports/robot_motions(52).json'
source = json.loads(source_path.read_text())
r_motions = {m['name']: copy.deepcopy(m) for m in source['motions'] if m['name'].startswith('R')}
assert len(r_motions) == 29
renames = {}
derived = {}


def derive(base, name, count):
    motion = copy.deepcopy(r_motions[base])
    motion.update(name=name, repeat_count=count)
    r_motions[name] = motion
    derived[name] = {'source': base, 'repeat_count': count}


for camera in (45, 90):
    for count in (2, 4, 6, 8):
        name = f'R전진{camera}도({count}회)'
        if name not in r_motions:
            derive(f'R전진{camera}도(4회)', name, count)
        renames[f'전{camera}도({count}회)'] = name
for camera in (45, 0, 90):
    for count in (2, 3, 5, 7, 9):
        suffix = '' if camera == 45 else f'{camera}도'
        name = f'R제자리우회전{suffix}({count}회)'
        if camera == 45 and count == 5:
            name = 'R제자리우회전(5회('
        if count == 3:
            derive(f'R제자리우회전{suffix}(4회)', name, count)
        old = f'찐제자리우회전{camera}도-1({count}회)'
        if camera == 45 and count == 9:
            old = '찐제자리우회전45도(9회)'
        renames[old] = name
renames.update({
    '건라인복귀우회전45도(4회)': 'R라인복귀우회전',
    '건라인복귀좌회전45도(4회)': 'R전진라인복귀좌',
    '건라인복귀좌회전45도(6회)': 'R전진라인복귀좌(6회)',
    '건오뒤카메라90도': 'R오뒤카메라90도',
    '건오뒤에서 기본자세(0도)': 'R오뒤에서기본자세0도',
    '건왼뒤에서 기본자세(0도)': 'R왼뒤에서 기본자세0도',
    '건김오뒤3': 'R오뒤45도',
    '찐오뒤에서 오옆꽃게90도': 'R오뒤에서오옆꽃게90도',
    '찐후진하고 제자리우회전(공)': 'R후진에서제자리우회전(공)',
})
derive('R전진라인복귀좌', 'R전진라인복귀좌(6회)', 6)

alias_path = ROOT/'src/irc_step_motion_executor/config/motion_aliases.yaml'
old_text = (OUT/'motion_aliases.before.yaml').read_text()
old_aliases = yaml.safe_load(old_text)['motion_aliases']
new_aliases = {key: renames.get(value, value) for key, value in old_aliases.items()}
# New variants without an existing autonomous command get explicit manual IDs.
extra_aliases = {
    'r_forward_0_4': 'R전진0도(4회)',
    'r_forward_0_6': 'R전진0도(6회)',
    'r_turn_right_4': 'R제자리우회전(4회)',
    'r_turn_right_90_4': 'R제자리우회전90도(4회)',
    'r_turn_right_0_4': 'R제자리우회전0도(4회)',
    'r_left_back_to_default_0': 'R왼뒤에서 기본자세0도',
    'r_back_pose_45': 'R오뒤45도',
    'r_forward_to_crab_90': 'R오뒤에서오옆꽃게90도',
}
new_aliases.update(extra_aliases)
assert {m['name'] for m in source['motions'] if m['name'].startswith('R')} <= set(new_aliases.values())
outputs = {}
for filename in ('robot_motions_runtime.json', 'robot_motions_pc.json'):
    old = json.loads((OUT/filename.replace('.json', '.before.json')).read_text())
    assert set(renames) <= {m['name'] for m in old['motions']}
    motions = []
    inserted = set()
    for motion in old['motions']:
        name = renames.get(motion['name'])
        if name:
            motions.append(copy.deepcopy(r_motions[name]))
            inserted.add(name)
        else:
            motions.append(copy.deepcopy(motion))
    motions.extend(copy.deepcopy(m) for name, m in r_motions.items() if name not in inserted)
    if filename == 'robot_motions_runtime.json':
        motions = [_with_runtime_policy(m) for m in motions]
    assert len(motions) == len({m['name'] for m in motions})
    assert set(new_aliases.values()) <= {m['name'] for m in motions}
    old['motions'] = motions
    outputs[filename] = old

text = old_text
for before, after in renames.items():
    text = text.replace('"'+before+'"', '"'+after+'"')
text = text.replace('# Re-measure actual yaw after the catalog9 right-turn gait update.',
                    '# Catalog52 R right-turn gait: retain command counts; re-measure actual yaw.')
text = text.replace('# Forward variants use robot_motions(40)(1).json 전.; camera poses and repeats follow names.',
                    '# Catalog52 R forward variants; 2/8-cycle commands retain their existing repeat counts.')
text = text.replace('# Catalog49 four-cycle recovery motions; selected by TURN direction, not line side.',
                    '# Catalog52 R recovery motions; selected by TURN direction, not line side.')
text = text.replace('# Retained six-cycle derivative of the catalog47 left recovery.',
                    '# Six-cycle derivative of the catalog52 R left recovery.')
text += '\n  # Catalog52 R variants available for explicit/manual requests.\n'
text += ''.join(f'  {key}: "{value}"\n' for key, value in extra_aliases.items())
assert yaml.safe_load(text)['motion_aliases'] == new_aliases
# All validation above completes before changing deployed files.
for filename, payload in outputs.items():
    path = ROOT/'artifacts'/filename
    temporary = path.with_suffix('.catalog52.tmp')
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)
alias_path.write_text(text)
manifest = {
    'source_file': str(source_path.relative_to(ROOT)),
    'source_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest(),
    'source_r_motions': [m['name'] for m in source['motions'] if m['name'].startswith('R')],
    'renames': renames, 'derived': derived, 'added_aliases': extra_aliases,
    'changed_aliases': {key: {'before': old_aliases[key], 'after': value}
                        for key, value in new_aliases.items() if key in old_aliases and old_aliases[key] != value},
    'runtime_tolerance_deg': 5.0,
    'motion_count': len(outputs['robot_motions_runtime.json']['motions']),
}
(OUT/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
print(f"R source=29, derived={len(derived)}, replaced={len(renames)}, total={manifest['motion_count']}")
print(f"changed aliases={len(manifest['changed_aliases'])}, added aliases={len(extra_aliases)}")
