#!/usr/bin/env python3
"""Audit the supplied immutable snapshots, runtime catalog and aliases without normalizing them."""
import ast
import csv
import hashlib
import json
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'artifacts/20261006_gui_alignment'
REF=OUT/'reference'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def fingerprint(frames):
    tree=ast.parse((REF/'sdk_gui.py').read_text())
    fn=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body
            if isinstance(n,ast.FunctionDef) and n.name=='sequence_timing_fingerprint')
    fn.decorator_list=[];ns=dict(json=json,hashlib=hashlib)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),'GUI fingerprint','exec'),ns)
    return ns[fn.name](frames)
def main():
    pc={m['name']:m for m in json.loads((REF/'robot_motions(10).json').read_text())['motions']}
    runtime_path=ROOT/'artifacts/robot_motions_runtime.json'
    rt={m['name']:m for m in json.loads(runtime_path.read_text())['motions']}
    state=json.loads((REF/'sdk_gui_state.json').read_text())
    checked=[s for s in state['saved_sequences'] if s.get('export_enabled',False)]
    (OUT/'robot_motions_gui_state.json').write_text(json.dumps({'motions':checked},ensure_ascii=False,indent=2)+'\n')
    state_diff=[]
    for s in checked:
        for field in ('frames','repeat_count','playback_speed'):
            if s.get(field)!=pc.get(s['name'],{}).get(field):state_diff.append([s['name'],field])
    repairs=[]
    for r in json.loads((REF/'sdk_gui_timing_repairs.json').read_text()):
        s=next((s for s in state['saved_sequences'] if s['sequence_id']==r['sequence_id']),None)
        repairs.append(dict(name=s['name'] if s else None,
                            pending=bool(s and fingerprint(s['frames'])==r['before'])))
    diffs=[]
    def walk(a,b,path):
        if isinstance(a,dict) and isinstance(b,dict):
            for k in sorted(a.keys()|b.keys()):walk(a.get(k),b.get(k),path+'/'+str(k))
        elif isinstance(a,list) and isinstance(b,list):
            for i in range(max(len(a),len(b))):walk(a[i] if i<len(a) else None,b[i] if i<len(b) else None,path+'/'+str(i))
        elif a!=b:diffs.append(dict(path=path,gui=a,runtime=b))
    for name in sorted(pc.keys()|rt.keys()):walk(pc.get(name),rt.get(name),name)
    catalog_diffs=diffs.copy()
    diffs.clear()
    for s in checked:walk(s.get('frames'),pc.get(s['name'],{}).get('frames'),s['name']+'/frames')
    state_frame_differences=diffs.copy()
    diffs=catalog_diffs
    aliases=yaml.safe_load((ROOT/'src/irc_step_motion_executor/config/motion_aliases.yaml').read_text())['motion_aliases']
    target='건전진45도(6회)';m=pc[target]
    loaded=next((s for s in state['saved_sequences'] if s['sequence_id']==state['loaded_sequence_id']),None)
    report=dict(source_hashes={p.name:sha(p) for p in REF.iterdir() if p.is_file()},runtime_sha256=sha(runtime_path),
        sdk_hashes={p.name:sha(p) for p in (ROOT/'external_sdk/gui_aligned').iterdir() if p.is_file()},
        counts=dict(gui=len(pc),runtime=len(rt),saved_sequences=len(state['saved_sequences']),checked=len(checked)),
        checked_export_differences=state_diff,state_frame_differences=state_frame_differences,timing_repairs=repairs,
        gui_only=sorted(pc.keys()-rt.keys()),runtime_only=sorted(rt.keys()-pc.keys()),
        aliases_missing_runtime={k:v for k,v in aliases.items() if v not in rt},
        aliases_missing_gui={k:v for k,v in aliases.items() if v not in pc},
        selected=dict(name=target,aliases=[a for a,n in aliases.items() if n==target],
                      frames=m['frames'],playback_speed=m['playback_speed'],repeat_count=m['repeat_count'],
                      timeline_end_ms=max(f['start_ms']+f['time_ms'] for f in m['frames']),
                      max_seq_ms=m['max_seq_ms'],differences=[d for d in diffs if d['path'].startswith(target+'/')]),
        ui_snapshot=dict(loaded_sequence_name=loaded['name'] if loaded else None,
                         timeline_equals_loaded_frames=bool(loaded and state['motion_sequence']==loaded['frames']),
                         runtime_ui_speed_and_repeat='not recorded; saved values used for software comparison'),
        usb_latency=dict(devices=[str(p) for p in Path('/sys/bus/usb-serial/devices').glob('*')],measured_value=None),
        differences=diffs)
    (OUT/'audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    with (OUT/'catalog_diff.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=['path','gui','runtime']);w.writeheader();w.writerows(diffs)
    print(json.dumps({k:report[k] for k in ['counts','checked_export_differences','ui_snapshot']},ensure_ascii=False,indent=2))
    print('selected differences',report['selected']['differences'])
    print('timings',[(f['name'],f['start_ms'],f['time_ms']) for f in m['frames']])
if __name__=='__main__':main()
