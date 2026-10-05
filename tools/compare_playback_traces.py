#!/usr/bin/env python3
"""Normalize GUI JSONL/SDK CSV, compare runs, or export a hardware-free replay."""
import argparse
import collections
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import statistics

WRITES = {'sample', 'endpoint', 'write'}


def load_trace(path):
    text = Path(path).read_text(encoding='utf-8')
    if text.lstrip().startswith('{'):
        rows = [json.loads(line) for line in text.splitlines() if line.strip()]
        if not rows or rows[0].get('schema_version') != 2:
            raise ValueError('GUI trace schema 2 required; recapture with trace_sdk_gui.py')
        complete = bool(rows and rows[-1].get('event') == 'trace_end')
        dropped = rows[-1].get('dropped_rows') if complete else None
    else:
        lines = text.splitlines()
        complete = bool(lines and lines[-1].startswith('# dropped_rows='))
        dropped = int(lines[-1].split('=')[1]) if complete else None
        rows = []
        for raw in csv.DictReader(io.StringIO('\n'.join(line for line in lines if not line.startswith('#')))):
            if not raw.get('begin_ns'):
                raise ValueError('SDK trace schema 2 required; rebuild the SDK')
            row = {key: int(raw[key]) for key in ('run', 'event_seq', 'tick', 'begin_ns', 'end_ns',
                                                  'evaluate_ns', 'timeline_ms', 'repeat', 'frame')}
            row.update(event=raw['event'], motion=raw['motion'], success=raw['success'] == '1',
                       speed=float(raw['speed']), repeat_target=int(raw['repeat_target']),
                       frame_start_ms=int(raw['frame_start_ms']), frame_duration_ms=int(raw['frame_duration_ms']),
                       frame_name=raw['frame_name'], lift_early=raw['lift_early'] == '1',
                       cycle_end=raw['cycle_end'] == '1',
                       angles={str(i): float(raw[f'deg_{i}']) for i in range(23) if raw.get(f'deg_{i}')},
                       raw={str(i): int(raw[f'raw_{i}']) for i in range(23) if raw.get(f'raw_{i}')})
            rows.append(row)
    for row in rows:
        if 'begin_ns' in row and row['end_ns'] < row['begin_ns']:
            raise ValueError('negative event duration')
    return dict(rows=rows, complete=complete, dropped_rows=dropped,
                sha256=hashlib.sha256(text.encode()).hexdigest())


def select_run(trace, run):
    rows = [r for r in trace['rows'] if r.get('run') == run]
    starts = [r for r in rows if r['event'] == 'run_start']
    if len(starts) != 1:
        raise ValueError(f'run {run} needs exactly one run_start')
    start = starts[0]
    return sorted(rows, key=lambda r: (r['begin_ns'], r['event_seq'])), start


def definition(rows, start):
    if 'snapshot' in start:
        snapshot = start['snapshot']
        frames = snapshot['frames']
        speed, repeats = snapshot['playback_speed'], snapshot['repeat_count']
    else:
        defs = sorted((r for r in rows if r['event'] == 'definition'), key=lambda r: r['frame'])
        if not defs or any(not r['angles'] for r in defs):
            raise ValueError('effective definition requires motion_trace_goals=true')
        frames = [dict(name=r['frame_name'], start_ms=r['frame_start_ms'], time_ms=r['frame_duration_ms'],
                       angles=r['angles'], lift_early_arrival=r['lift_early'], playback_cycle_end=r['cycle_end'])
                  for r in defs]
        speed, repeats = start['speed'], start['repeat_target']
    # Float normalization makes 18 and 18.0 the same physical target.
    value = dict(playback_speed=float(speed), repeat_count=int(repeats), frames=[
        dict(name=f['name'], start_ms=int(f['start_ms']), time_ms=int(f['time_ms']),
             angles={str(k): float(v) for k, v in f['angles'].items()},
             lift_early_arrival=bool(f.get('lift_early_arrival', False)),
             playback_cycle_end=bool(f.get('playback_cycle_end', False))) for f in frames])
    return value, hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                           allow_nan=False).encode()).hexdigest()


def stats(values):
    if not values:
        return None
    values = sorted(values)
    return dict(count=len(values), median=statistics.median(values),
                p95=values[max(0, math.ceil(len(values) * .95) - 1)], max=values[-1])


def summarize(rows):
    writes = [r for r in rows if r['event'] in WRITES and r['success']]
    current = {}
    errors = []
    for row in rows:
        if row['event'] in WRITES and row['success']:
            current.update({str(k): (v - 2048) * 360 / 4096 for k, v in row.get('raw', {}).items()})
        elif row['event'] == 'feedback' and row['success']:
            errors.extend(abs(float(v) - current[str(k)]) for k, v in row.get('angles', {}).items() if str(k) in current)
    return dict(writes=len(writes),
        write_interval_ms=stats([(b['begin_ns']-a['begin_ns'])/1e6 for a, b in zip(writes, writes[1:])]),
        evaluate_to_write_ms=stats([(r['begin_ns']-r['evaluate_ns'])/1e6 for r in writes if r.get('evaluate_ns')]),
        read_duration_ms=stats([(r['end_ns']-r['begin_ns'])/1e6 for r in rows if r['event'] == 'feedback']),
        observed_tracking_abs_error_deg=stats(errors),
        ui_duration_ms={event: stats([(r['end_ns']-r['begin_ns'])/1e6 for r in rows if r['event'] == event])
                        for event in ('gui_style', 'gui_3d', 'gui_plot_compute', 'gui_plot_paint')},
        failed_events=sum(not r['success'] for r in rows))


def compare(gui, sdk, gui_run, sdk_run):
    a, sa = select_run(gui, gui_run)
    b, sb = select_run(sdk, sdk_run)
    da, ha = definition(a, sa)
    db, hb = definition(b, sb)
    def grouped(rows):
        groups = collections.defaultdict(list)
        for row in rows:
            if row['event'] in WRITES and row['success']:
                groups[(row['repeat'], row['frame'], row['timeline_ms'], row['event'])].append(row)
        return groups
    ga, gb = grouped(a), grouped(b)
    missing_a = missing_b = raw_mismatches = pairs = 0
    schedule_error = []
    origin_a = sa.get('playback_origin_ns', sa['begin_ns'])
    origin_b = sb.get('playback_origin_ns', sb['begin_ns'])
    for key in ga.keys() | gb.keys():
        left, right = ga.get(key, []), gb.get(key, [])
        missing_a += max(0, len(right)-len(left))
        missing_b += max(0, len(left)-len(right))
        for x, y in zip(left, right):
            pairs += 1
            raw_mismatches += x.get('raw') != y.get('raw')
            schedule_error.append(abs((x['begin_ns']-origin_a)-(y['begin_ns']-origin_b))/1e6)
    return dict(schema_version=2, quality={
        'gui': {k: gui[k] for k in ('complete', 'dropped_rows', 'sha256')},
        'sdk': {k: sdk[k] for k in ('complete', 'dropped_rows', 'sha256')}},
        effective_definition_equal=da == db, effective_definition_hashes=[ha, hb],
        initial_angles_equal=sa.get('snapshot', {}).get('initial_deg', sa.get('angles')) ==
                             sb.get('snapshot', {}).get('initial_deg', sb.get('angles')),
        matched_write_pairs=pairs, matched_raw_mismatches=raw_mismatches,
        writes_without_gui_match=missing_a, writes_without_sdk_match=missing_b,
        matched_schedule_abs_error_ms=stats(schedule_error), gui=summarize(a), sdk=summarize(b),
        limitations=['Missing timeline samples are not interpolated into measured data.',
                     'Read-call intervals are observations, not simultaneous joint sample timestamps.',
                     'USB wire timing and physical arrival are not established by API-call timing.',
                     'UI nested durations must not be summed as independent costs.'])


def export_replay(trace, run, directory):
    if not trace['complete'] or trace['dropped_rows'] != 0:
        raise ValueError('replay requires a closed trace with zero dropped rows')
    rows, start = select_run(trace, run)
    if start.get('gui_context') != 'motion' or 'snapshot' not in start:
        raise ValueError('recorded execution replay currently supports standalone GUI motion runs only')
    if start['timeline_ms'] != 0 or start['repeat'] != 1 or not start['success']:
        raise ValueError('replay requires a successful fresh run, not resume/scrub')
    motion, _ = definition(rows, start)
    motion.update(name='recorded_gui', max_seq_ms=start['snapshot']['max_seq_ms'])
    initial = {int(k): float(v) for k, v in start['snapshot']['initial_deg'].items()}
    if set(initial) != set(range(23)) or not all(math.isfinite(v) for v in initial.values()):
        raise ValueError('replay requires finite initial angles for all 23 motors')
    origin = start['playback_origin_ns']
    ticks = sorted((r for r in rows if r['event'] == 'tick' and r.get('evaluate_ns') is not None),
                   key=lambda r: r['tick'])
    if not ticks:
        raise ValueError('no measured playback ticks')
    script = ['I ' + ' '.join(str(initial[i]) for i in range(23))]
    previous_time = -1
    total_writes = 0
    for tick in ticks:
        relative = (tick['evaluate_ns']-origin)/1e6
        if relative < previous_time or relative < 0:
            raise ValueError('non-monotonic playback evaluation clock')
        previous_time = relative
        script.append(f'T {relative:.9f}')
        events = [r for r in rows if r.get('tick') == tick['tick'] and r['event'] in WRITES | {'feedback'}]
        previous_end = tick['evaluate_ns']
        for event in events:
            if event['begin_ns'] < previous_end:
                raise ValueError('overlapping read/write scopes; replay needs non-nested I/O')
            previous_end = event['end_ns']
            write = event['event'] in WRITES
            total_writes += write
            if write and not event['success']:
                raise ValueError('failed writes cannot be used as a reference gait')
            values = event.get('angles', {}) if event['success'] else {}
            entries = []
            for key, value in sorted(values.items(), key=lambda item: int(item[0])):
                if not 0 <= int(key) < 23 or not math.isfinite(float(value)):
                    raise ValueError('invalid recorded motor data')
                raw = event.get('raw', {}).get(str(key), 0)
                entries.extend([str(key), str(value), str(raw)])
            script.append(' '.join(['W' if write else 'R', f"{(event['begin_ns']-origin)/1e6:.9f}",
                f"{(event['end_ns']-origin)/1e6:.9f}", str(int(event['success'])), str(len(values)), *entries]))
    if not total_writes:
        raise ValueError('no actual goal writes recorded')
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory/'catalog.json').write_text(json.dumps({'motions': [motion]}, ensure_ascii=False, indent=2)+'\n')
    (directory/'schedule.txt').write_text('\n'.join(script)+'\n')
    (directory/'source.json').write_text(json.dumps(dict(trace_sha256=trace['sha256'], run=run,
        hardware_free_only=True, note='Measured callback and I/O timings; not a motor command file.'), indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', type=Path, required=True)
    parser.add_argument('--sdk', type=Path)
    parser.add_argument('--gui-run', type=int, default=1)
    parser.add_argument('--sdk-run', type=int, default=1)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--export-replay', type=Path)
    parser.add_argument('--normalized-dir', type=Path)
    args = parser.parse_args()
    gui = load_trace(args.gui)
    sdk = load_trace(args.sdk) if args.sdk else None
    if args.normalized_dir:
        args.normalized_dir.mkdir(parents=True, exist_ok=True)
        for platform, trace in [('gui', gui), ('sdk', sdk)]:
            if trace is None:
                continue
            rows = [dict(event='session', schema_version=2, platform=platform,
                         source_trace_sha256=trace['sha256'])]
            rows.extend(r for r in trace['rows'] if r['event'] not in {'session', 'trace_end'})
            if trace['complete']:
                rows.append(dict(event='trace_end', dropped_rows=trace['dropped_rows']))
            (args.normalized_dir/f'{platform}.jsonl').write_text(
                '\n'.join(json.dumps(r, ensure_ascii=False) for r in rows)+'\n')
    if args.export_replay:
        export_replay(gui, args.gui_run, args.export_replay)
    if args.sdk:
        result = compare(gui, sdk, args.gui_run, args.sdk_run)
        text = json.dumps(result, ensure_ascii=False, indent=2)+'\n'
        if args.output:
            args.output.write_text(text)
        else:
            print(text, end='')
    elif not args.export_replay:
        parser.error('provide --sdk or --export-replay')


if __name__ == '__main__':
    main()
