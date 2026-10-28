#include "robot_motion_player.hpp"
#include "gui_playback.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace irc_step;
struct Bus : IMotionHardware {
    JointAngles goals, present;
    int reads=0, prepares=0;
    bool initialize() noexcept override { for(int id=0;id<23;++id) present[id]=0; return true; }
    bool ready() const noexcept override { return true; }
    bool prepareImmediatePlayback() noexcept override { ++prepares; return true; }
    bool commandPosition(const JointAngles& g,const std::vector<int>& ids,uint32_t d,uint32_t a) noexcept override {
        assert(d==0 && a==0); for(int id:ids) goals[id]=g.at(id); return true;
    }
    bool readPresentPositions(JointAngles& p) noexcept override {++reads;p=present;return true;}
    bool holdCurrentPosition(uint32_t) noexcept override {return true;}
    bool setTorqueEnabled(bool) noexcept override {return true;}
    std::string_view lastError() const noexcept override {return {};}
};
int main(int argc,char**argv) {
    assert(argc==2);
    const auto path=(std::filesystem::path(argv[1])/"forward_continuation.json").string();
    std::ofstream(path)<<R"({"motions":[
      {"name":"f4","max_seq_ms":300,"repeat_count":2,"continuation_group":"forward_45","frames":[
       {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":10}},
       {"name":"back","start_ms":190,"time_ms":100,"angles":{"1":0}}]},
      {"name":"f6","max_seq_ms":300,"repeat_count":1,"continuation_group":"forward_45","frames":[
       {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":10}},
       {"name":"back","start_ms":190,"time_ms":100,"angles":{"1":0}}]},
      {"name":"other","max_seq_ms":300,"repeat_count":1,"continuation_group":"forward_90","frames":[
       {"name":"different","start_ms":90,"time_ms":200,"angles":{"1":70}}]},
      {"name":"stop","max_seq_ms":300,"repeat_count":2,"continuation_group":"forward_45","frames":[
       {"name":"lift","start_ms":90,"time_ms":100,"angles":{"1":10}},
       {"name":"back","start_ms":190,"time_ms":100,"stop_at_end":true,"angles":{"1":0}}]},
      {"name":"smooth","max_seq_ms":400,"repeat_count":1,"continuation_group":"smooth","frames":[
       {"name":"first","start_ms":90,"time_ms":100,"angles":{"1":10}},
       {"name":"second","start_ms":190,"time_ms":100,"angles":{"1":0}},
       {"name":"last","start_ms":290,"time_ms":100,"angles":{"1":5}}]}
    ]})";
    for(int scenario=0;scenario<5;++scenario) {
        Bus b; int ms=0;
        RobotMotionPlayer p(path,b,[&]{return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));});
        assert(p.initialize());
        if(scenario==4) p.setContinuousInterpolation(false);
        const std::string name=scenario==3 ? "stop" : "f4";
        assert(p.start(name)==StartResult::Accepted);
        if(scenario!=3) assert(p.queueNext("other")==QueueResult::IncompatibleTransition);
        if(scenario==0 || scenario==4) assert(p.queueNext("f6")==QueueResult::Queued);
        if(scenario==2) { assert(p.queueNext("f6")==QueueResult::Queued); p.clearQueuedMotion(); }
        const auto expected=p.motionInfo(name).expected_duration_ms;
        assert(expected==((scenario==3 || scenario==4) ? 580 : 490));
        for(ms=5;ms<=1000 && p.running();ms+=5) {
            p.update();
            if(ms==290 && scenario!=3 && scenario!=4) assert(p.playbackRepeat()==2);
            if(ms==295 && scenario!=3 && scenario!=4) assert(b.goals.at(1)>0); // no repeated 90ms hold
            if(ms==495 && scenario==0) assert(b.goals.at(1)>0); // no queued start hold
        }
        assert(p.succeeded());
        assert(b.prepares==1); // queue does not reinitialize profiles or recapture start
        const int elapsed=ms-5;
        assert(elapsed==(scenario==0 ? 690 : scenario==4 ? 870 : scenario==3 ? 580 : 490));
        assert(b.goals.at(1)==0);
    }
    for (bool late : {false, true}) {
        Bus bus; int ms=0;
        RobotMotionPlayer player(path,bus,[&]{return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));});
        assert(player.initialize());
        assert(player.start("smooth")==StartResult::Accepted);
        ms=late ? 350 : 100;
        player.update();
        const double before=bus.goals.at(1);
        assert(player.queueNext("smooth")==QueueResult::Queued);
        player.update();
        assert(bus.goals.at(1)==before); // queue never changes a segment already playing
        ms=389;player.update();const double left=bus.goals.at(1);
        ms=390;player.update();assert(bus.goals.at(1)==5);
        ms=391;player.update();const double right=bus.goals.at(1);
        assert(right>5); // next motion starts immediately, including late queues
        if (late) {
            assert((5-left)*1000<1 && (right-5)*1000<1);
        } else {
            assert(std::abs((5-left)*1000-50)<1);
            assert(std::abs((right-5)*1000-50)<1);
        }
    }
    // At a same-direction boundary both sides retain 20deg/s, acceleration 0.
    MotionFrame a,b;
    a.name="a";a.time_ms=1000;a.angles={{1,20}};
    b.name="b";b.time_ms=100;b.angles={{1,30}};
    GuiContinuousTrajectory left,right;
    left.reset({a},{{1,0}},1);left.connectNext(a,b,1);
    const auto velocity=left.endVelocities();assert(velocity[1]==20);
    right.reset({b},{{1,20}},1);right.setStartVelocities(velocity);
    const auto x=[&](double t){return t<=1000?left.sample(t).angles[1]:right.sample(t-1000).angles[1];};
    assert(std::abs((x(1000)-x(999.99))/.00001-20)<.01);
    assert(std::abs((x(1000.01)-x(1000))/.00001-20)<.01);
    a.stop_at_end=true;left.reset({a},{{1,0}},1);left.connectNext(a,b,1);assert(left.endVelocities()[1]==0);
    std::cout<<"PASS forward repeat/queue without leading hold; group guard, explicit stop, fallback, velocity continuity\n";
}
