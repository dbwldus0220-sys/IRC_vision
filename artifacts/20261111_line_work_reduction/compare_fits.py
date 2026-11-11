import importlib.util
import sys
import subprocess
from dataclasses import asdict
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"src/step"))
import numpy as np
from step.yolo_line_analyzer import YoloLineAnalyzer, LinePoint

spec=importlib.util.spec_from_loader('step._old_fit',loader=None)
old=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=old
source=subprocess.check_output(['git','show','3b90819:src/step/step/yolo_line_analyzer.py'],cwd=ROOT,text=True)
exec(compile(source,'baseline_yolo_line_analyzer.py','exec'),old.__dict__)
node=object.__new__(YoloLineAnalyzer)
node.min_points_for_fit=2
node.min_fit_y_span_px=80.
rng=np.random.default_rng(4103)
for i in range(1200):
    count=int(rng.integers(0,31))
    ys=rng.uniform(100,700,count) if i%10 else np.full(count,300.)
    xs=700.+ys*.3+rng.normal(0,(0.,1.,30.,150.)[i%4],count)
    points=[LinePoint(float(x),float(y),float(c)) for x,y,c in zip(xs,ys,rng.uniform(.4,1,count))]
    node.max_fit_iterations=(0,1,2,3,5)[i%5]
    node.outlier_residual_px=(0.,.01,1.,10.,50.)[(i//5)%5]
    before=old.YoloLineAnalyzer._robust_fit(node,points,require_y_span=bool(i%2))
    after=node._robust_fit(points,require_y_span=bool(i%2))
    assert (asdict(before) if before else None)==(asdict(after) if after else None),i
print('1200 randomized/degenerate/outlier fits identical')
