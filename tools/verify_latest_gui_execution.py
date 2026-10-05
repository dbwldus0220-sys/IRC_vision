#!/usr/bin/env python3
"""Execute the supplied GUI methods (AST, no Qt/import side effects) against C++ I/O traces."""
import argparse
import ast
import contextlib
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace

METHODS = {
    'normalize_angles', 'shortest_angle_delta', 'interpolate_angle_shortest',
    'frame_motion_progress', 'angle_to_dxl_position', 'dxl_position_to_angle',
    'anim_step', 'scrub_timeline', 'gate_timeline_on_frame_arrival',
    'next_playback_cycle_end', 'restart_playback_cycle', 'playback_time_ms',
    'map_timeline_time', 'resort_motion_sequence', 'start_motion_playback',
    'composer_entry_frames', 'composer_entry_speed', 'composer_timing_segments',
    'build_composed_sequence_frames',
}

def load_gui(path):
    tree = ast.parse(path.read_text())
    ns = dict(math=math, copy=copy, time=SimpleNamespace(), TimelineBlockWidget=object)
    for node in tree.body:
        if isinstance(node, ast.Assign) and all(isinstance(t, ast.Name) and t.id.isupper() for t in node.targets):
            try:
                exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), ns)
            except (NameError, TypeError):
                pass
    methods = [n for c in tree.body if isinstance(c, ast.ClassDef) for n in c.body
               if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    assert {n.name for n in methods} == METHODS
    cls = ast.ClassDef(name='Oracle', bases=[], keywords=[], body=methods, decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), 'exec'), ns)
    return ns

class Dummy:
    def __getattr__(self, name):
        if name == 'findChildren': return lambda *args: []
        if name == 'height': return lambda: 100
        if name == 'horizontalScrollBar': return lambda: self
        return lambda *args, **kwargs: None

def oracle(ns, motion, periods, read_delay, write_delay, queued=None):
    p = ns['Oracle']()
    clock = [1000.0]
    records = []
    initial = {i: i*.25-3 for i in range(23)}
    ns['time'].perf_counter = lambda: clock[0]/1000
    p.online_joints = set(initial)
    p.joints = initial.copy()
    p.start_angles = {}
    p.is_playing = False
    p.robot_sync_enabled = p.execute_on_real_robot = True
    p.playback_context = 'motion'
    p.playback_paused_by_button = False
    p.is_paused = False
    p.SCALE = 1
    for attr in ('anim_timer', 'robot_scrub_timer', 'timeline_container', 'timeline_scroll', 'spin_max_time'):
        setattr(p, attr, Dummy())
    p.update_playback_buttons = lambda: None
    p.refresh_timeline_ui = lambda: None
    p.update_3d_robot = lambda *args: None
    p.set_tracking_target_angles = lambda *args: None
    p.fail_timed_playback = lambda message: (_ for _ in ()).throw(AssertionError(message))
    def read():
        records.append(['R', clock[0]])
        clock[0] += read_delay
        return True
    def sync():
        read()
        p.joints = initial.copy()
        return True
    p.sync_sequence_start_from_robot = sync
    p.read_feedback_and_report_fk = read
    command = initial.copy()
    def write(angles, target_ids=None, **kwargs):
        normalized = p.normalize_angles(angles)
        ids = sorted(normalized if target_ids is None else target_ids)
        records.append(['W', clock[0], {i: (normalized[i], p.angle_to_dxl_position(normalized[i])) for i in ids}])
        command.update(normalized)
        clock[0] += write_delay
        return True
    p.write_goal_positions = write
    def select(m):
        p.motion_sequence = copy.deepcopy(m['frames'])
        p.max_seq_ms = m['max_seq_ms']
        p.playback_repeat_target = m.get('repeat_count', 1)
        p.playback_repeat_current = 1
        p.playback_speed = m.get('playback_speed', 1)
    select(motion)
    complete = [0]
    def stop(**kwargs):
        complete[0] += 1
        records.append(['C', clock[0], complete[0]])
        if queued is not None and complete[0] == 1:
            # Composer-like queued continuation: no new physical start read.
            select(queued)
            p.resort_motion_sequence()
            p.start_angles = command.copy()
            p.anim_duration = max(f['start_ms']+f['time_ms'] for f in p.motion_sequence)/1000
            p.restart_playback_cycle(0, {'angles': command})
        else:
            p.is_playing = False
    p.stop_motion_sequence = stop
    with contextlib.redirect_stdout(io.StringIO()):
        p.start_motion_playback(real_robot=True)
        for tick in range(1000000):
            clock[0] += periods[tick % len(periods)]
            p.anim_step()
            records.append(['T', clock[0], p.current_timeline_ms, p.playback_repeat_current, int(p.is_playing)])
            if not p.is_playing:
                return records
    raise AssertionError('oracle did not finish')

def parse_trace(text):
    records=[]
    for line in text.splitlines():
        fields=line.split('\t'); kind=fields[0]
        if kind not in {'R','W','C','T'}: continue
        if kind=='W':
            values={}
            for value in fields[2:]:
                i,degree,raw=value.split(':'); values[int(i)]=(float(degree),int(raw))
            records.append([kind,float(fields[1]),values])
        else:
            records.append([kind,float(fields[1]),*map(int,fields[2:])])
    return records

def compare(expected, actual, name):
    if len(expected)!=len(actual):
        # Find the earliest differing event rather than only reporting lengths.
        print(name, 'event counts',len(expected),len(actual))
    for index,(a,b) in enumerate(zip(expected,actual)):
        assert a[0]==b[0] and abs(a[1]-b[1])<1e-6, (name,index,a,b)
        if a[0]=='W':
            assert a[2].keys()==b[2].keys(),(name,index,a,b)
            for joint in a[2]:
                assert a[2][joint][1]==b[2][joint][1],(name,index,joint,a,b)
                assert abs(a[2][joint][0]-b[2][joint][0])<1e-9,(name,index,joint,a,b)
        elif a[0]=='C': assert a[2:]==b[2:],(name,index,a,b)
        elif a[0]=='T':
            # Completion/queue clears GUI cursor; compare all running timeline ticks.
            if a[4] and not (index and expected[index-1][0]=='C'):
                assert a[2:]==b[2:],(name,index,a,b)
    assert len(expected)==len(actual),(name,len(expected),len(actual))

def synthetic(ns):
    def frame(name='normal', start=0, duration=100, angles=None, **kw):
        return dict(name=name,start_ms=start,time_ms=duration,angles=angles or {'0':30.25},**kw)
    frames=[frame('오들',20,65,{'0':179.9,'1':-180.0},lift_early_arrival=True),
            frame('오들',85,72,{'0':-179.9},lift_early_arrival=False),
            frame('ordinary',200,50,{'2':-27.125},lift_early_arrival=True,playback_cycle_end=True),
            frame('들기',250,100,{'0':-350.5,'3':180.0},lift_early_arrival=True)]
    motions=[]
    for speed in [.9,1,1.05,2.0]:
        motions.append(dict(name=f'partial_{speed}',frames=copy.deepcopy(frames),max_seq_ms=10000,
                            playback_speed=speed,repeat_count=3,start_pose='link',end_pose='link'))
    motions.append(dict(name='overlap',frames=[frame(start=40,duration=60),frame(start=40,duration=50)],
                        max_seq_ms=80,playback_speed=1,repeat_count=2,start_pose='link',end_pose='link'))
    p=ns['Oracle']()
    p.saved_sequences=[dict(frames=[frame('오들',0,65,lift_early_arrival=True),frame(start=65,duration=65)],playback_speed=.9)]
    p.sequence_composer_entries=[dict(sequence_idx=0,repeat_count=2,gap_before_ms=25,time_ms=285,playback_speed=.9)]
    composed,end=p.build_composed_sequence_frames(apply_source_speed=True)
    assert composed[0]['time_ms']==72
    motions.append(dict(name='saved_composer',frames=composed,max_seq_ms=end+1000,playback_speed=1.05,
                        repeat_count=2,start_pose='link',end_pose='link'))
    for source_speed in [.9,1.0,1.05]:
        for global_speed in [.9,1.0,1.05]:
            if source_speed==.9 and global_speed==1.05: continue
            p.saved_sequences[0]['playback_speed']=source_speed
            p.sequence_composer_entries[0]['playback_speed']=source_speed
            frames,end=p.build_composed_sequence_frames(apply_source_speed=True)
            motions.append(dict(name=f'composer_{source_speed}_{global_speed}',frames=frames,
                                max_seq_ms=end+1000,playback_speed=global_speed,repeat_count=2,
                                start_pose='link',end_pose='link'))
    return {'motions':motions}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--gui',type=Path,required=True)
    ap.add_argument('--probe',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('catalogs',type=Path,nargs='+')
    args=ap.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    ns=load_gui(args.gui)
    fixture=args.output/'synthetic.json';fixture.write_text(json.dumps(synthetic(ns),ensure_ascii=False,indent=2))
    raw_output=subprocess.check_output([str(args.probe),'--raw'],text=True)
    converter=ns['Oracle']()
    for line in raw_output.splitlines():
        angle,raw=line.split('\t')
        assert converter.angle_to_dxl_position(float(angle))==int(raw),(angle,raw)
    summary=[dict(case='raw_rounding_half_steps_and_neighbors',values=len(raw_output.splitlines()))]
    for path in [*args.catalogs,fixture]:
        motions=json.loads(path.read_text())['motions']
        for motion in motions:
            # All catalog motions, regular and delayed I/O; extra jitter/boundary cases on fixtures.
            scenarios=[([5.0],0,0),([5.0,17.0,83.0,6.0],16,1)]
            if path==fixture: scenarios += [([5.0],10,2),([350.0,5.0],0,0)]
            for periods,read_delay,write_delay in scenarios:
                label=f'{path.name}/{motion["name"]}/{periods}/{read_delay}/{write_delay}'
                expected=oracle(ns,motion,periods,read_delay,write_delay)
                output=subprocess.check_output([str(args.probe),str(path),motion['name'],','.join(map(str,periods)),str(read_delay),str(write_delay)],text=True)
                actual=parse_trace(output)
                compare(expected,actual,label)
                row=dict(case=label,events=len(actual),writes=sum(r[0]=='W' for r in actual),
                         reads=sum(r[0]=='R' for r in actual),completed_ms=actual[-1][1]-1000,
                         max_raw_error=0,max_event_time_error_ms=0)
                summary.append(row)
                if motion['name']=='건전진45도(6회)' or path==fixture:
                    tag=hashlib.sha256(label.encode()).hexdigest()[:12]
                    (args.output/f'trace_{tag}.json').write_text(json.dumps({'case':label,'gui':expected,'cpp':actual},ensure_ascii=False))
    # Compare SDK queue directly with the actual GUI's saved composer path.
    # Both source speeds are 1 here so integer-ms export adds no rounding shift.
    a=copy.deepcopy(synthetic(ns)['motions'][1]); b=copy.deepcopy(a)
    a.update(name='queue_a',repeat_count=1); b.update(name='queue_b',repeat_count=1)
    b['frames'][0]['angles']['0']=-100.125
    queue_file=args.output/'queue.json'
    queue_file.write_text(json.dumps({'motions':[a,b]},ensure_ascii=False))
    composer=ns['Oracle']();composer.saved_sequences=[a,b]
    composer.sequence_composer_entries=[dict(sequence_idx=i,repeat_count=1,gap_before_ms=0,
        time_ms=max(f['start_ms']+f['time_ms'] for f in m['frames']),playback_speed=1.0) for i,m in enumerate([a,b])]
    frames,end=composer.build_composed_sequence_frames(apply_source_speed=True)
    combined=dict(name='actual_gui_composer',frames=frames,max_seq_ms=end,playback_speed=1,repeat_count=1)
    expected=oracle(ns,combined,[5,17],16,1)
    output=subprocess.check_output([str(args.probe),str(queue_file),a['name'],'5,17','16','1',b['name']],text=True)
    actual=parse_trace(output)
    compare([r for r in expected if r[0] in ('R','W')],
            [r for r in actual if r[0] in ('R','W')],'queue_vs_actual_gui_composer')
    assert abs(expected[-1][1]-actual[-1][1])<1e-6
    boundary=max(f['start_ms']+f['time_ms'] for f in a['frames'])
    gui_boundary=next(r[1] for r in expected if r[0]=='T' and r[2]==boundary)
    cpp_boundary=next(r[1] for r in actual if r[0]=='C')
    assert abs(gui_boundary-cpp_boundary)<1e-6
    (args.output/'queue_comparison.json').write_text(json.dumps(dict(gui=expected,cpp=actual),ensure_ascii=False))
    summary.append(dict(case='queue_vs_actual_gui_composer',events=len(actual),max_raw_error=0,max_event_time_error_ms=0))
    (args.output/'results.json').write_text(json.dumps({'gui_sha256':hashlib.sha256(args.gui.read_bytes()).hexdigest(),
        'cases':summary,'passed':len(summary)},ensure_ascii=False,indent=2)+'\n')
    print(f'PASS {len(summary)} cases: actual GUI functions vs C++ player, every write/read/raw/event')

if __name__=='__main__':main()
