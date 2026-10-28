#!/usr/bin/env python3
"""User-specified quintic contract oracle; NOT a copy of the unavailable latest PC GUI."""
import argparse
import json
import math
from pathlib import Path
import subprocess


def raw(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return max(0, min(4095, round(2048 + max(-180, min(179.912109375, angle)) * (4096 / 360))))


def segments(motion, initial):
    pose = initial.copy()
    result = []
    for frame in motion['frames']:
        start = pose.copy()
        goals = {int(k): v for k, v in frame['angles'].items()}
        pose.update(goals)
        result.append(dict(frame=frame, start=start, goal=pose.copy(),
                           delta={j: (v-start[j]+180) % 360-180 for j, v in goals.items()},
                           h=frame['time_ms']/1000/motion.get('playback_speed', 1), v0={}, v1={}))
    for a, b in zip(result, result[1:]):
        fa, fb = a['frame'], b['frame']
        if (fa['start_ms']+fa['time_ms'] != fb['start_ms'] or
                fa.get('playback_cycle_end') or fa.get('stop_at_end') or
                '[착지]' in fa['name'] or '[착지]' in fb['name']):
            continue
        for j in a['delta'].keys() & b['delta'].keys():
            da, db = a['delta'][j], b['delta'][j]
            if da*db <= 0:
                continue
            if any(abs(s['goal'][j]-s['start'][j]-s['delta'][j]) > 1e-9 for s in (a, b)):
                continue
            if any(v < -180 or v > 179.912109375 for v in (a['start'][j], a['goal'][j], b['goal'][j])):
                continue
            a['v1'][j] = b['v0'][j] = math.copysign(min(abs(da/a['h']), abs(db/b['h'])), da)
    return result


def sample(parts, t):
    pose = parts[0]['start'].copy()
    for s in parts:
        f = s['frame']
        if t < f['start_ms']:
            return pose
        if t >= f['start_ms']+f['time_ms']:
            pose = s['goal'].copy()
            continue
        u = max(0, min(1, (t-f['start_ms'])/f['time_ms']))
        pose = s['start'].copy()
        for j, d in s['delta'].items():
            m0, m1 = s['v0'].get(j, 0)*s['h'], s['v1'].get(j, 0)*s['h']
            c3, c4, c5 = 10*d-6*m0-4*m1, -15*d+8*m0+7*m1, 6*d-3*m0-3*m1
            pose[j] = s['start'][j]+m0*u+c3*u*u*u+c4*u*u*u*u+c5*u*u*u*u*u
        return pose
    return pose


def compare(probe, catalog):
    motions = {m['name']: m for m in json.loads(catalog.read_text())['motions']}
    count = 0
    largest_error = 0
    proc = subprocess.Popen([str(probe), str(catalog)], stdout=subprocess.PIPE, text=True)
    key = None
    for line in proc.stdout:
        name, repeat, time, *values = line.rstrip().split('\t')
        if (name, repeat) != key:
            initial = {j: j*.25-3 for j in range(23)}
            if int(repeat) > 1:
                for f in motions[name]['frames']:
                    initial.update({int(k): v for k, v in f['angles'].items()})
            parts = segments(motions[name], initial)
            key = name, repeat
        expected = sample(parts, float(time))
        for j, v in enumerate(values):
            angle, goal = v.split(':')
            error = abs(float(angle)-expected[j])
            largest_error = max(largest_error, error)
            assert error < 1e-9, (name, repeat, time, j, angle, expected[j])
            assert int(goal) == raw(expected[j]), (name, repeat, time, j, goal, raw(expected[j]))
            count += 1
    assert proc.wait() == 0
    return dict(motions=len(motions), joint_samples=count, maximum_angle_error_deg=largest_error, raw_mismatches=0)


def compare_execution(probe, catalog):
    """Validate final sample of every real player tick, including repeat boundary writes."""
    motions = json.loads(catalog.read_text())['motions']
    samples = 0
    for motion in motions:
        for periods, read_delay, write_delay in [('5', '0', '0'), ('5,17,83,6', '1', '1')]:
            output = subprocess.check_output([str(probe), str(catalog), motion['name'],
                                              periods, read_delay, write_delay, '--continuous'], text=True)
            repeat = 1
            initial = {j: j*.25-3 for j in range(23)}
            parts = segments(motion, initial)
            writes = []
            for line in output.splitlines():
                fields = line.split('\t')
                if fields[0] == 'W':
                    writes.append({int(j): (float(a), int(r)) for j, a, r in
                                   (field.split(':') for field in fields[2:])})
                if fields[0] != 'T':
                    continue
                t, next_repeat = int(fields[2]), int(fields[3])
                if next_repeat != repeat:
                    t = max(f['start_ms']+f['time_ms'] for f in motion['frames'])
                if writes:
                    expected = sample(parts, t)
                    for j, (angle, goal) in writes[-1].items():
                        assert abs(angle-expected[j]) < 1e-9, (motion['name'], repeat, t, j, angle, expected[j])
                        assert goal == raw(expected[j]), (motion['name'], repeat, t, j, goal, raw(expected[j]))
                        samples += 1
                    writes.clear()
                if next_repeat != repeat:
                    for f in motion['frames']:
                        initial.update({int(j): a for j, a in f['angles'].items()})
                    parts = segments(motion, initial)
                    repeat = next_repeat
    return dict(motions=len(motions), scenarios_per_motion=2, joint_samples=samples, raw_mismatches=0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--raw-probe', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('catalogs', type=Path, nargs='+')
    args = parser.parse_args()
    result = {'oracle': 'user specification, latest PC GUI unavailable', 'catalogs': {}}
    for catalog in args.catalogs:
        result['catalogs'][str(catalog)] = compare(args.probe, catalog)
    output = subprocess.check_output([str(args.raw_probe), '--raw'], text=True)
    for line in output.splitlines():
        angle, goal = line.split('\t')
        assert raw(float(angle)) == int(goal), line
    result['rounding_boundary_cases'] = len(output.splitlines())
    result['real_player_execution'] = {str(c): compare_execution(args.raw_probe, c) for c in args.catalogs}
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
