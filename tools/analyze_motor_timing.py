#!/usr/bin/env python3
"""Summarize recorded TX/RX timing; never access the motor bus."""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def stats(values):
    if not values:
        return None
    values=sorted(values)
    return dict(count=len(values),mean=statistics.mean(values),
                p95=values[math.ceil(.95*len(values))-1],
                p99=values[math.ceil(.99*len(values))-1],max=values[-1])


def motion_transitions(rows):
    """Separate saved leading holds from host gaps at repeat/command boundaries."""
    definitions = {}
    for row in rows:
        if row['event'] == 'definition':
            definitions.setdefault(row['run'], []).append(row)
    result = []
    for index, boundary in enumerate(rows):
        if boundary['event'] not in ('repeat', 'goals_complete'):
            continue
        previous_index = next((i for i in range(index - 1, -1, -1)
                               if rows[i]['event'] in ('sample', 'endpoint')), None)
        previous = rows[previous_index] if previous_index is not None else None
        following = rows[index + 1:]
        # Do not attribute movement from a later, unrelated motion to this boundary.
        end = next((i for i, r in enumerate(following)
                    if r['event'] in ('repeat', 'goals_complete', 'fault', 'cancel', 'emergency_stop')),
                   len(following))
        following = following[:end]
        samples = [r for r in following if r['event'] in ('sample', 'endpoint')]
        if previous is None or not samples:
            continue
        first = samples[0]
        frames = definitions.get(first['run'], [])
        speed = float(first.get('speed') or 1)
        saved_hold = (min(float(r['frame_start_ms']) for r in frames) / speed
                      if frames else None)
        # Body IDs only: an independent head override must not appear as walking.
        def body_changed(row):
            return row['success'] == '1' and any(
                previous.get(f'raw_{motor}') not in (None, '')
                and row.get(f'raw_{motor}') not in (None, '')
                and float(previous[f'raw_{motor}']) != float(row[f'raw_{motor}'])
                for motor in range(6, 23))
        changed = next((r for r in samples if body_changed(r)), None)
        prefix = following[:following.index(first)]
        between_writes = rows[previous_index + 1:index] + prefix
        kind = ('repeat' if boundary['event'] == 'repeat' else
                'queued' if any(r['event'] == 'queue_activate' for r in prefix) else 'restart')
        previous_end = float(previous['end_ms'])
        result.append(dict(
            kind=kind, from_run=boundary['run'], to_run=first['run'],
            motion=boundary['motion'], next_motion=first['motion'],
            boundary_ms=float(boundary['begin_ms']), saved_leading_hold_ms=saved_hold,
            last_tx_end_to_next_tx_start_ms=float(first['begin_ms']) - previous_end,
            last_tx_end_to_first_body_raw_change_ms=(
                float(changed['begin_ms']) - previous_end if changed else None),
            initial_position_reads=sum(r['event'] == 'initial' for r in between_writes),
            feedback_read_duration_before_next_tx_ms=sum(
                float(r['duration_ms']) for r in between_writes if r['event'] == 'feedback'),
        ))
    return result


def summarize(path):
    lines=path.read_text().splitlines()
    footer=[line for line in lines if line.startswith('# dropped_rows=')]
    rows=list(csv.DictReader(line for line in lines if line and not line.startswith('#')))
    tx=[r for r in rows if r['event'] in ('sample','endpoint')]
    rx=[r for r in rows if r['event'] in ('initial','feedback')]
    intervals=[float(b['begin_ms'])-float(a['begin_ms']) for a,b in zip(tx,tx[1:])]
    failure=[]
    for r in rx:
        if r['success']=='0':
            mask=int(r.get('missing_ids_mask') or 0)
            failure.append(dict(time_ms=float(r['begin_ms']),sdk_result=r.get('bus_result'),
                                result_valid=r.get('bus_result_valid'),
                                missing_ids=[i for i in range(23) if mask&(1<<i)]))
    boundaries=[]
    for r in rows:
        if r['event'] not in ('repeat','cycle','queue_accept','queue_activate','goals_complete'):
            continue
        t=float(r['begin_ms'])
        before=next((w for w in reversed(tx) if float(w['begin_ms'])<=t),None)
        after=next((w for w in tx if float(w['begin_ms'])>t),None)
        boundaries.append(dict(event=r['event'],time_ms=t,
            surrounding_tx_gap_ms=float(after['begin_ms'])-float(before['begin_ms']) if before and after else None))
    runs=[]
    for run in sorted({r['run'] for r in rows}):
        subset=[r for r in rows if r['run']==run]
        start=next((r for r in subset if r['event']=='run_start'),None)
        end=next((r for r in reversed(subset) if r['event'] in ('goals_complete','cancel','emergency_stop')),None)
        if start:
            runs.append(dict(run=run,motion=start['motion'],continuous=start.get('continuous'),
                exclude_motor_5=start.get('exclude_motor_5'),
                duration_ms=float(end['end_ms'])-float(start['begin_ms']) if end else None))
    return dict(trace=str(path),dropped_rows=int(footer[-1].split('=')[1]) if footer else None,
        limitations='Host calls only: TX success is not motor receipt or physical arrival. Missing-ID masks mean unavailable responses, not proof of the failed physical device. Callback dispatch and USB latency require separate observations.',
        tx_interval_ms=stats(intervals),syncwrite_duration_ms=stats([float(r['duration_ms']) for r in tx]),
        syncread_duration_ms=stats([float(r['duration_ms']) for r in rx]),
        tx_failures=sum(r['success']=='0' for r in tx),rx_failures=failure,
        boundaries=boundaries,runs=runs,motion_transitions=motion_transitions(rows))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('trace',type=Path);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();result=summarize(args.trace)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({key:result[key] for key in ('tx_interval_ms','syncwrite_duration_ms','syncread_duration_ms')},indent=2))

if __name__=='__main__':main()
