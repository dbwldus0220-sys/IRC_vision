import csv,itertools,json,subprocess,tempfile
from pathlib import Path
root=Path.cwd()
catalog=root/'artifacts/robot_motions_runtime.json'
ms={m['name']:m for m in json.loads(catalog.read_text())['motions']}
pairs=[('R전진라인복귀좌','R전진라인복귀좌'),('R전진라인복귀좌','R전진45도(6회)'),('R라인복귀우회전','R라인복귀우회전')]
results=[]
with tempfile.TemporaryDirectory(prefix='step_late_queue_') as temp:
 for (a,b),continuous,read_delay,transport_delay in itertools.product(pairs,[False,True],[0,16],[0,100]):
  m=ms[a]; duration=max(f['start_ms']+f['time_ms'] for f in m['frames'])*m['repeat_count']/m['playback_speed']
  at=duration-300+transport_delay
  trace=Path(temp)/'trace.csv'
  cmd=['/tmp/step_late_queue_probe',str(catalog),a,'5',str(read_delay),'0','--account-forward-feedback','--queue',b,'--queue-after-ms',str(at),'--trace',str(trace)]
  if continuous: cmd+=['--continuous']
  subprocess.run(cmd,stdout=subprocess.DEVNULL,check=True)
  rows=list(csv.DictReader(l for l in trace.read_text().splitlines() if not l.startswith('#')))
  starts=[r for r in rows if r['event']=='run_start']; ends=[r for r in rows if r['event']=='goals_complete']; accepts=[r for r in rows if r['event']=='queue_accept']
  assert len(starts)==len(ends)==2 and len(accepts)==1
  assert [r['motion'] for r in starts]==[a,b]
  assert float(accepts[0]['begin_ms'])<float(ends[0]['begin_ms'])
  gap=float(starts[1]['begin_ms'])-float(ends[0]['begin_ms'])
  assert gap==0,(a,b,gap)
  assert [int(r['repeat']) for r in ends]==[ms[a]['repeat_count'],ms[b]['repeat_count']]
  results.append(dict(first=a,second=b,queue_after_ms=at,transport_delay_ms=transport_delay,read_delay_ms=read_delay,continuous=continuous,boundary_dispatch_gap_ms=gap))
out=root/'artifacts/20261028_line_late_queue_speed/late_queue_results.json'
out.write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
print(len(results),'virtual SDK cases passed; zero added boundary dispatch gap')
