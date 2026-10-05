import copy
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import yaml

root=Path.cwd()
sys.path.insert(0,str(root/'tools'))
from upsert_motion_catalog import _with_runtime_policy
spec=importlib.util.spec_from_file_location('validator',root/'src/irc_step_motion_executor/scripts/validate_motion_catalog.py')
validator=importlib.util.module_from_spec(spec);spec.loader.exec_module(validator)
source=Path('/home/jet/Downloads/robot_motions(47).json')
runtime=root/'artifacts/robot_motions_runtime.json'
reference=root/'artifacts/robot_motions_pc.json'
previous_reference=root/'artifacts/20261002_common_playback/robot_motions_pc.json'
alias_path=root/'src/irc_step_motion_executor/config/motion_aliases.yaml'
aliases=yaml.safe_load(alias_path.read_text())['motion_aliases']
supplied=json.loads(source.read_text())
original=json.loads(runtime.read_text())
by_name={m['name']:m for m in supplied['motions']}
assert len(by_name)==len(supplied['motions'])
reference_payload=copy.deepcopy(supplied)
derived=[]
for alias,base,count in [
 ('line_recovery_left_6','건라인복귀좌회전45도(4회)',6),
 ('hurdle_fine_forward_10','건미세0도',10),
 *[(f'goal_camera_90_fine_forward_{n}','건미세90도',n) for n in range(1,5)],
]:
 name=aliases[alias]
 assert name not in by_name
 motion=copy.deepcopy(by_name[base]);motion.update(name=name,repeat_count=count)
 reference_payload['motions'].append(motion)
 derived.append({'name':name,'base':base,'repeat_count':count})
base=aliases['fine_to_turn_ready_45'];name=aliases['fine_to_turn_ready_90']
assert name not in by_name
motion=copy.deepcopy(by_name[base]);motion['name']=name
for frame in motion['frames']:
 frame['angles']['0']=1.0
 frame['name']+='(회전준비90도)'
motion['start_pose']=motion['frames'][0]['name'];motion['end_pose']=motion['frames'][-1]['name']
reference_payload['motions'].append(motion)
derived.append({'name':name,'base':base,'camera_motor_0_deg':1.0,'frame_name_suffix':'(회전준비90도)'})
result=copy.deepcopy(reference_payload)
result['motions']=[_with_runtime_policy(m) for m in result['motions']]
for payload in (supplied,reference_payload,result):
 errors=validator.validate_catalog(payload,aliases if payload is not supplied else {})
 assert not errors,errors
 for m in payload['motions']:
  assert isinstance(m['repeat_count'],int) and m['repeat_count']>0
  assert math.isfinite(m['playback_speed']) and m['playback_speed']>0
  for f in m['frames']:
   assert f['start_ms']>=0 and f['time_ms']>0
   assert all(isinstance(v,(float,int)) and math.isfinite(v) for v in f['angles'].values())
   assert all(isinstance(v,bool) for v in f['torques'].values())
for payload in (reference_payload,result):
 frames=[f for m in payload['motions'] for f in m['frames'] if f['name']=='김오뒤3']
 assert frames and all(f['angles']==frames[0]['angles'] for f in frames),'ambiguous startup pose'
 # Player resolves policy reference by frame ID/name and start time.
 for m in payload['motions']:
  for f in m['frames']:
   matches=[g for g in m['frames'] if g['start_ms']==f['start_ms'] and
            ((f.get('frame_id') and g.get('frame_id')==f['frame_id']) or g['name']==f['name'])]
   assert len(matches)==1,(m['name'],f['name'])
stamp=datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')
backup=root/'artifacts'/f'{stamp}_catalog47_full_replace'
backup.mkdir()
shutil.copy2(source,backup/'source_robot_motions47.json')
shutil.copy2(runtime,backup/'robot_motions_runtime.before.json')
shutil.copy2(previous_reference,backup/'robot_motions_pc.before.json')
shutil.copy2(alias_path,backup/'motion_aliases.before.yaml')
shutil.copy2(__file__,backup/'import_catalog.py')
def write_atomic(path,payload):
 fd,tmp=tempfile.mkstemp(dir=path.parent,prefix=path.name+'.')
 try:
  with os.fdopen(fd,'w',encoding='utf-8') as f:
   json.dump(payload,f,ensure_ascii=False,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
  os.chmod(tmp,0o644);os.replace(tmp,path)
 finally:
  if os.path.exists(tmp):os.unlink(tmp)
write_atomic(reference,reference_payload)
write_atomic(runtime,result)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
old={m['name']:m for m in original['motions']};new={m['name']:m for m in result['motions']}
manifest={'source':str(source),'source_sha256':sha(source),'source_motion_count':len(supplied['motions']),
 'before_count':len(old),'runtime_count':len(new),'derived_motions':derived,
 'removed_names':sorted(old.keys()-new.keys()),'updated_names':[n for n in new if old.get(n)!=new[n]],
 'runtime_policy':{'position_tolerance_deg':5.0,'fixed_joint_angles_deg':{'4':18.0,'5':-18.0},
 'exceptions':['찐공잡기리그랩까지 실전','찐골넣기','찐허들']},
 'runtime_path':str(runtime.relative_to(root)),'reference_path':str(reference.relative_to(root)),
 'runtime_sha256':sha(runtime),'reference_sha256':sha(reference),'backup_runtime_sha256':sha(backup/'robot_motions_runtime.before.json'),
 'aliases_changed':False,'validation':{'aliases_resolve':True,'motor_ids':'0..22','startup_pose_consistent':True,'reference_frame_matching':True}}
write_atomic(backup/'manifest.json',manifest)
assert sha(source)==sha(backup/'source_robot_motions47.json')
print('backup='+str(backup))
print('source=75 derived=7 runtime=82 aliases_resolve=true startup_pose_consistent=true')
print('updated='+str(len(manifest['updated_names']))+' removed='+str(manifest['removed_names']))
