import contextlib, importlib.util, io, json, subprocess, sys
from pathlib import Path
import argparse
parser=argparse.ArgumentParser(description="Compare C++ raw goals with the supplied GUI on fake I/O")
parser.add_argument('--sdk-root', type=Path, required=True)
parser.add_argument('--probe', type=Path, required=True)
parser.add_argument('catalogs', nargs='+', type=Path)
args=parser.parse_args()
root=args.sdk_root
spec=importlib.util.spec_from_file_location('gui_oracle',root/'gui/test_sdk_gui_timed_playback.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
# Run the supplied GUI playback methods, with fake I/O, as the independent oracle.
raw_gui_code=__import__('ast').parse((root/'gui/sdk_gui.py').read_text())
ast=__import__('ast')
fn=next(n for c in raw_gui_code.body if isinstance(c,ast.ClassDef) for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='angle_to_dxl_position')
exec(compile(ast.Module(body=[fn],type_ignores=[]),'gui position conversion','exec'),mod.namespace)
raw=mod.namespace['angle_to_dxl_position']
for path in args.catalogs:
    motions={m['name']:m for m in json.loads(Path(path).read_text())['motions']}
    output=subprocess.check_output([str(args.probe),str(path)],text=True)
    samples=0;commands=0;p=None;last_name=None
    with contextlib.redirect_stdout(io.StringIO()):
        for line in output.splitlines():
            name,t,boundary,*values=line.split('\t');t=int(t)
            if name!=last_name:
                p=mod.player();mod.set_frames(p,motions[name]['frames']);p.online_joints=set(range(23))
                p.start_angles={i:i*.25-3 for i in range(23)};p.joints=p.start_angles.copy();last_name=name
            p.calls.clear();p.scrub_timeline(t,cycle_end=bool(int(boundary)))
            goals=p.calls[-1][1]
            expected=[raw(p,goals[i]) if i in goals else -1 for i in range(23)]
            actual=list(map(int,values))
            if expected!=actual:raise AssertionError((name,t,expected,actual))
            samples+=1;commands+=len(goals)
    print(Path(path).name, 'motions',len(motions),'samples',samples,'joint_commands',commands,'all matched GUI')
