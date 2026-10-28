#!/usr/bin/env python3
"""Compare every player I/O event with the supplied 2026-10-28 specification.

This is an independent specification oracle, not the missing latest PC GUI.
The archived GUI is used only to generate exported composer fixtures.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import subprocess

from verify_continuous_playback import raw, segments, sample
from verify_latest_gui_execution import load_gui, synthetic, parse_trace, compare


def normalize(motion):
    """The GUI packs overlapping frames; mirror its documented catalog loader."""
    motion = copy.deepcopy(motion)
    frames = motion['frames'] = sorted(motion['frames'], key=lambda f: f['start_ms'])
    end = 0
    for f in frames:
        f['start_ms'] = max(end, f['start_ms'])
        end = f['start_ms'] + f['time_ms']
    if end > motion['max_seq_ms']:
        limit = motion['max_seq_ms']
        for f in reversed(frames):
            f['start_ms'] = min(f['start_ms'], limit-f['time_ms'])
            limit = f['start_ms']
        offset = max(0, -frames[0]['start_ms'])
        for f in frames:
            f['start_ms'] += offset
    return motion


def oracle(motion, periods, read_delay, write_delay, continuous, excluded=False, queued=None):
    clock = 1000.0
    records = [['R', clock]]
    clock += read_delay
    initial = {i: i*.25-3 for i in range(23)}
    if excluded:
        initial.pop(5)
    command = initial.copy()
    repeat, completed, cycle_start, gated, last_read = 1, 0, 0, None, None
    started = clock/1000

    def prepare(m):
        m = normalize(m)
        if excluded:
            for f in m['frames']:
                f['angles'].pop('5', None)
        return m, segments(m, initial)

    motion, parts = prepare(motion)
    running = True
    for tick in range(1000000):
        clock += periods[tick % len(periods)]
        requested = int((clock/1000-started)*1000*motion.get('playback_speed', 1)+1e-7)
        end = max(f['start_ms']+f['time_ms'] for f in motion['frames'])
        boundary = min([end]+[f['start_ms']+f['time_ms'] for f in motion['frames']
                              if f.get('playback_cycle_end') and cycle_start < f['start_ms']+f['time_ms']])
        timeline = min(requested, boundary)
        writes = []
        if gated is not None and timeline >= gated['start_ms']+gated['time_ms']:
            writes.append({int(j): a for j, a in gated['angles'].items()})
            timeline = gated['start_ms']+gated['time_ms']
            gated = None
        at_boundary = timeline >= boundary
        active = next((f for f in motion['frames'] if f['start_ms'] <= timeline < f['start_ms']+f['time_ms']
                       or at_boundary and timeline == f['start_ms']+f['time_ms']), None)
        if active:
            if at_boundary:
                target = {int(j): a for j, a in active['angles'].items()}
            elif continuous:
                all_angles = sample(parts, timeline)
                target = {int(j): all_angles[int(j)] for j in active['angles']}
            else:
                pose = initial.copy()
                for f in motion['frames']:
                    if f['start_ms']+f['time_ms'] <= active['start_ms']:
                        pose.update({int(j): a for j, a in f['angles'].items()})
                lift = active.get('lift_early_arrival') and any(s in active['name'] for s in ('발들', '들기', '오들', '왼들'))
                duration = max(1, active['time_ms']*(.8 if lift else 1))
                u = max(0, min(1, (timeline-active['start_ms'])/duration))
                progress = .5-.5*math.cos(math.pi*u)
                target = {int(j): pose[int(j)]+((a-pose[int(j)]+180)%360-180)*progress
                          for j, a in active['angles'].items()}
            # Cosine endpoints use shortest-angle arithmetic, as the GUI does.
            if at_boundary and not continuous:
                pose = initial.copy()
                for f in motion['frames']:
                    if f['start_ms']+f['time_ms'] <= active['start_ms']:
                        pose.update({int(j): a for j, a in f['angles'].items()})
                target = {int(j): pose[int(j)]+((a-pose[int(j)]+180)%360-180) for j,a in active['angles'].items()}
        else:
            target = initial.copy()
            for f in motion['frames']:
                if f['start_ms']+f['time_ms'] <= timeline:
                    target.update({int(j): a for j,a in f['angles'].items()})
        writes.append(target)
        for values in writes:
            if values:
                records.append(['W', clock, {j: (a, raw(a)) for j,a in sorted(values.items())}])
                command.update(values)
                clock += write_delay
        if at_boundary or last_read is None or clock/1000-last_read >= .010:
            last_read = clock/1000
            records.append(['R', clock])
            clock += read_delay
        if active:
            gated = active
        if at_boundary:
            if boundary < end:
                cycle_start = boundary
                for f in motion['frames']:
                    if f['start_ms']+f['time_ms'] == boundary:
                        initial.update({int(j): a for j,a in f['angles'].items()})
                started = clock/1000-boundary/(1000*motion.get('playback_speed',1))
                gated, last_read = None, None
            elif repeat < motion.get('repeat_count',1):
                for f in motion['frames']:
                    initial.update({int(j): a for j,a in f['angles'].items()})
                parts = segments(motion, initial)
                repeat += 1
                cycle_start, timeline, gated, last_read, started = 0, 0, None, None, clock/1000
            else:
                completed += 1
                records.append(['C', clock, completed])
                if queued is None:
                    running = False
                else:
                    initial = command.copy()
                    motion, parts = prepare(queued)
                    queued = None
                    repeat, cycle_start, timeline, gated, last_read, started = 1, 0, 0, None, None, clock/1000
        records.append(['T', clock, timeline, repeat, int(running)])
        if not running:
            return records
    raise AssertionError('oracle did not terminate')


def fixtures(gui):
    motions = synthetic(load_gui(gui))['motions']
    def frame(name, start, goal, **options):
        return dict(name=name, start_ms=start, time_ms=100, angles=goal, **options)
    base = dict(name='same_direction', max_seq_ms=300, repeat_count=2, playback_speed=1,
                start_pose='link', end_pose='link', frames=[frame('A',0,{'0':10}),frame('B',100,{'0':20})])
    motions.append(base)
    for mode in ('reverse','stationary','landing_a','landing_b','stop','cycle','missing_joint','only_5','gap','wrap'):
        m=copy.deepcopy(base); m['name']=mode
        a,b=m['frames']
        if mode=='reverse': b['angles']['0']=-10
        if mode=='stationary': b['angles']['0']=10
        if mode=='landing_a': a['name']='A[착지]'
        if mode=='landing_b': b['name']='B[착지]'
        if mode=='stop': a['stop_at_end']=True
        if mode=='cycle': a['playback_cycle_end']=True
        if mode=='missing_joint': b['angles']={'1':20}
        if mode=='only_5': b['angles']={'5':45};m['frames'].append(frame('C',200,{'1':15}))
        if mode=='gap': b['start_ms']=150
        if mode=='wrap': a['angles']={'0':179.9};b['angles']={'0':-179.9}
        motions.append(m)
    return {'motions':motions}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--probe',type=Path,required=True)
    ap.add_argument('--archived-gui',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('catalogs',type=Path,nargs='+')
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    fixture=args.output/'spec_fixtures.json'
    fixture.write_text(json.dumps(fixtures(args.archived_gui),ensure_ascii=False,indent=2)+'\n')
    cases=[]
    for path in [*args.catalogs,fixture]:
        motions=json.loads(path.read_text())['motions']
        for motion in motions:
            scenarios=[([5.0],0,0,False,False),([5.,17.,83.,6.],16,1,False,False)]
            if path==fixture:
                scenarios += [([350.,5.],0,0,False,False),([5.,17.],16,1,True,True)]
            for continuous in (False,True):
                for periods,rd,wd,excluded,timeout in scenarios:
                    label=[str(path),motion['name'],continuous,periods,rd,wd,excluded,timeout]
                    expected=oracle(motion,periods,rd,wd,continuous,excluded)
                    command=[str(args.probe),str(path),motion['name'],','.join(map(str,periods)),str(rd),str(wd)]
                    if continuous:command.append('--continuous')
                    if excluded:command.append('--exclude-5')
                    if timeout:command.append('--feedback-timeout')
                    actual=parse_trace(subprocess.check_output(command,text=True))
                    try:compare(expected,actual,label)
                    except AssertionError:
                        (args.output/'first_failure.json').write_text(json.dumps(dict(case=label,expected=expected,actual=actual),ensure_ascii=False))
                        raise
                    cases.append(dict(case=label,events=len(actual),writes=sum(r[0]=='W' for r in actual),raw_mismatches=0))
    for continuous in (False, True):
        motion=fixtures(args.archived_gui)['motions'][-2]
        for excluded in (False, True):
            expected=oracle(motion,[5.,17.,83.],16,1,continuous,excluded,queued=motion)
            command=[str(args.probe),str(fixture),motion['name'],'5,17,83','16','1','--queue',motion['name']]
            if continuous:command.append('--continuous')
            if excluded:command.append('--exclude-5')
            actual=parse_trace(subprocess.check_output(command,text=True))
            compare(expected,actual,['queue',continuous,excluded])
            cases.append(dict(case=['queue',continuous,excluded],events=len(actual),raw_mismatches=0))
    result=dict(oracle='2026-10-28 user specification; not latest GUI source',passed=len(cases),cases=cases)
    (args.output/'results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(f'PASS {len(cases)} cases; every write/read/order/ID/degree/raw/timestamp compared')

if __name__=='__main__':main()
