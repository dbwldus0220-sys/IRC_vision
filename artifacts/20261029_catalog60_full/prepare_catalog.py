"""Prepare source60 plus existing mission repeat/camera variants, without hardware."""
from pathlib import Path
import copy
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'tools'))
from upsert_motion_catalog import runtime_motion_from_source

source_path = OUT / 'source60.json'
payload = json.loads(source_path.read_text())
source = {m['name']: m for m in payload['motions']}
assert len(source) == len(payload['motions'])
prepared = copy.deepcopy(payload)
derived = {}

def add(name, original, repeat, head=None):
    motion = copy.deepcopy(source[original])
    motion.update(name=name, repeat_count=repeat)
    spec = {'source': original, 'repeat_count': repeat}
    if head is not None:
        spec['head_angle_deg'] = head
        names = {f['name']: f['name'] + '(회전준비90도)' for f in motion['frames']}
        for f in motion['frames']:
            f['angles']['0'] = head
            f['name'] = names[f['name']]
        for key in ('start_pose', 'end_pose'):
            motion[key] = names.get(motion[key], motion[key])
    prepared['motions'].append(motion)
    derived[name] = spec

for camera in ('', '0도', '90도'):
    for repeat in (3, 6):
        add(f'R제자리우회전{camera}({repeat}회)', f'R제자리우회전{camera}(2회)', repeat)
    for repeat in (1, 6):
        add(f'R제자리좌회전{camera}({repeat}회)', f'R제자리좌회전{camera}(2회)', repeat)
for camera, repeats in [('45', (2, 8)), ('90', (2,))]:
    for repeat in repeats:
        add(f'R전진{camera}도({repeat}회)', f'R전진{camera}도(4회)', repeat)
add('r미세0도(허들10회)', 'r미세0도', 10)
add('r미세0도(상단탐색3회)', 'r미세0도', 3)
for repeat in range(1, 5):
    add(f'R미세90도({repeat}회)', 'R미세90도', repeat)
add('R미세오뒤에서 오뒤90도', 'R미세오뒤에서 오뒤45도', 1, head=1.0)

pc_path = ROOT / 'artifacts/robot_motions_pc.json'
runtime_path = ROOT / 'artifacts/robot_motions_runtime.json'
pc_path.write_text(json.dumps(prepared, ensure_ascii=False, indent=2) + '\n')
runtime = {**prepared, 'motions': [runtime_motion_from_source(m) for m in prepared['motions']]}
runtime_path.write_text(json.dumps(runtime, ensure_ascii=False, indent=2) + '\n')
manifest = {'source_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest(),
            'original_motions': list(source), 'derived_motions': derived,
            'runtime_speed_factor': 0.95, 'runtime_position_tolerance_deg': 5.0,
            'original_count': len(source), 'derived_count': len(derived),
            'runtime_count': len(runtime['motions'])}
(OUT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
print(f"Prepared {len(source)} original + {len(derived)} mission variants; speed=source*0.95")
