#include "gui_playback.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace irc_step;
static MotionFrame frame(double angle, int start, int duration=1000) {
    MotionFrame f; f.name="move"; f.start_ms=start; f.time_ms=duration; f.angles={{1,angle}}; return f;
}
static double angle(const GuiContinuousTrajectory& t,double ms) { return t.sample(ms).angles[1]; }
static void boundary(const GuiContinuousTrajectory& t,double ms,double velocity) {
    const double dt=.1;
    const double left=(angle(t,ms)-angle(t,ms-dt))/(dt/1000);
    const double right=(angle(t,ms+dt)-angle(t,ms))/(dt/1000);
    assert(std::abs(left-velocity)<.001 && std::abs(right-velocity)<.001);
    const double al=(angle(t,ms)-2*angle(t,ms-dt)+angle(t,ms-2*dt))/(dt*dt/1e6);
    const double ar=(angle(t,ms+2*dt)-2*angle(t,ms+dt)+angle(t,ms))/(dt*dt/1e6);
    assert(std::abs(al)<.5 && std::abs(ar)<.5);
}
int main() {
    GuiContinuousTrajectory t;
    t.reset({frame(10,0,100),frame(20,100,100)},{{1,0}},1);
    assert(angle(t,50)==3.4375 && angle(t,100)==10 && angle(t,150)==16.5625);
    // 100 ms 경계의 공유속도는 100 deg/s. 양쪽 가속도는 0으로 수렴한다.
    const double epsilon=0.0001;
    assert(std::abs((angle(t,100)-angle(t,100-epsilon))/(epsilon/1000)-100)<1e-5);
    assert(std::abs((angle(t,100+epsilon)-angle(t,100))/(epsilon/1000)-100)<1e-5);
    std::vector<MotionFrame> frames{frame(20,0),frame(60,1000)};
    t.reset(frames,{{1,0},{2,7}},1);
    boundary(t,1000,20);
    assert(angle(t,0)==0 && angle(t,2000)==60);
    assert(t.sample(500).angles[2]==7);
    for(int scenario=0;scenario<7;++scenario) {
        auto f=frames;
        if(scenario==0) f[1].angles[1]=0; // 반전
        if(scenario==1) f[1].angles[1]=20; // 정지
        if(scenario==2) f[1].start_ms=1200; // 대기
        if(scenario==3) f[0].name="[착지]";
        if(scenario==4) f[1].name="[착지]";
        if(scenario==5) f[0].playback_cycle_end=true;
        if(scenario==6) f[0].stop_at_end=true;
        t.reset(f,{{1,0}},1); boundary(t,1000,0);
    }
    frames={frame(20,0),frame(60,1000)};
    frames[1].angles={{2,30}};t.reset(frames,{{1,0},{2,0}},1);boundary(t,1000,0);
    assert(t.sample(1500).angles[1]==20);
    // 래핑 구간 양 끝에서 저장된 각도를 정확히 복원한다.
    frames={frame(-170,0),frame(-150,1000)};t.reset(frames,{{1,170}},1);
    assert(angle(t,0)==170 && angle(t,1000)==-170);
    assert(std::abs((angle(t,1000.1)-angle(t,1000))/.0001)<.001);
    frames={frame(40,0)};frames[0].name="발들기";frames[0].lift_early_arrival=true;
    t.reset(frames,{{1,0}},1);assert(angle(t,800)<40);
    assert(sampleGuiMotion(frames,{{1,0}},800).angles[1]==40);
    assert(guiShortestDelta(170,-170)==20 && guiShortestDelta(-170,170)==-20);
    assert(guiShortestDelta(0,-540)==-180);
    assert(guiPositionRaw(-131.5283203125)==552);
    std::cout<<"PASS velocity/acceleration continuity, stops, gap, partial, wrap, endpoints, lift OFF fallback\n";
}
