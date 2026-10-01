import contextlib, importlib.util, io, json, subprocess
from pathlib import Path

sdk=Path('/home/jet/IRC/external_sdk/robot_motion_player_sdk_work_20260801/final step')
out=Path('/tmp/step_forward_gui_audit')
import tarfile
with tarfile.open('/home/jet/Downloads/gui_sdk_20260930_135448.tar.gz') as archive:
 out.mkdir(exist_ok=True)
 for suffix,name in [('/gui/sdk_gui.py','handoff_sdk_gui.py'),('/gui/test_sdk_gui_timed_playback.py','test_sdk_gui_timed_playback.py')]:
  member=next(n for n in archive.getnames() if n.endswith(suffix))
  (out/name).write_bytes(archive.extractfile(member).read())
(out/'sdk_gui.py').write_bytes((out/'handoff_sdk_gui.py').read_bytes())
source=(sdk/'robot_motion_player.cpp').read_text()
source=source.replace('namespace irc_step {','extern std::chrono::steady_clock::time_point auditClockNow();\nnamespace irc_step {',1).replace('Clock::now()', 'auditClockNow()')
(out/'player_clock_probe.cpp').write_text(source)
(out/'trace.cpp').write_text(r'''
#include "robot_motion_player.hpp"
#include "mock_motion_hardware.hpp"
#include <iostream>
#include <iomanip>
std::int64_t audit_ms=0;
std::chrono::steady_clock::time_point auditClockNow() {
    return std::chrono::steady_clock::time_point(std::chrono::milliseconds(audit_ms+1000));
}
int main(int argc,char**argv) {
    auto library=irc_step::MotionLibrary::loadGuiJson(argv[1]);
    irc_step::MockMotionHardware hw;
    hw.setPresentPositions(library.motion(argv[2]).frames().back().angles);
    irc_step::RobotMotionPlayer p(argv[1],hw);
    p.initialize(); p.start(argv[2]);
    std::cout<<std::setprecision(17);
    for(audit_ms=0;audit_ms<10000;audit_ms+=std::stoi(argv[3])) {
        p.update();
        std::cout<<"TRACE\t"<<audit_ms;
        for(int j=0;j<23;++j) std::cout<<'\t'<<hw.commandedPositions().at(j);
        std::cout<<'\n';
        if(!p.running()) break;
    }
}
''')
subprocess.run(['g++','-std=c++20','-O0','-I',str(sdk),str(out/'trace.cpp'),str(out/'player_clock_probe.cpp'),str(sdk/'motion_pattern.cpp'),'-o',str(out/'trace')],check=True)
spec=importlib.util.spec_from_file_location('gui_test',out/'test_sdk_gui_timed_playback.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
catalog=Path.cwd()/'artifacts/robot_motions_runtime.json';motions=json.loads(catalog.read_text())['motions']
summary=[]
for m in motions:
 if '전진' not in m['name']:continue
 for period in (5,10,20):
  p=mod.player();mod.set_frames(p,m['frames']);p.online_joints=set(range(23));p.start_angles={int(k):v for k,v in m['frames'][-1]['angles'].items()};p.joints=p.start_angles.copy();p.playback_speed=m['playback_speed'];p.playback_repeat_target=m['repeat_count']
  gui={}
  with contextlib.redirect_stdout(io.StringIO()):
   for ms in range(0,10000,period):
    mod.tick(p,ms/1000);gui[ms]=p.calls[-1][1].copy()
    if not p.is_playing:break
  output=subprocess.check_output([str(out/'trace'),str(catalog),m['name'],str(period)],text=True)
  cpp={int(parts[1]):dict(enumerate(map(float,parts[2:]))) for line in output.splitlines() if (parts:=line.split('\t'))[0]=='TRACE'}
  max_diff=(0,None,None)
  for ms in cpp.keys() & gui.keys():
   for j in range(10,23):
    delta=abs(cpp[ms][j]-gui[ms][j])
    if delta>max_diff[0]:max_diff=(delta,ms,j)
  summary.append({'motion':m['name'],'period_ms':period,'gui_finish_ms':max(gui),'sdk_finish_ms':max(cpp),'max_leg_command_difference_deg':round(max_diff[0],4),'at_ms':max_diff[1],'motor_id':max_diff[2]})
  if m['name']=='찐찐전진45(4회)' and period==5:
   (out/'forward4_command_trace.json').write_text(json.dumps({'gui':gui,'sdk':cpp},ensure_ascii=False))
(out/'timing_results.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
for r in summary:print(json.dumps(r,ensure_ascii=False))
