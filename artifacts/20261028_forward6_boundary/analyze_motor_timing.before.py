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
        boundaries=boundaries,runs=runs)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('trace',type=Path);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();result=summarize(args.trace)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({key:result[key] for key in ('tx_interval_ms','syncwrite_duration_ms','syncread_duration_ms')},indent=2))

if __name__=='__main__':main()
