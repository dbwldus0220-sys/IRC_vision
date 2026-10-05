#include "robot_motion_player.hpp"
#include "mock_motion_hardware.hpp"
#include "gui_playback.hpp"
#include <cassert>
#include <fstream>
#include <filesystem>
#include <iostream>
using namespace irc_step;
int main(int argc, char** argv) {
    assert(argc==2);
    const auto path=std::filesystem::path(argv[1])/"alignment_contract.json";
    std::ofstream(path)<<R"({"motions":[{"name":"a","max_seq_ms":5000,"repeat_count":1,
      "start_pose":"same","end_pose":"same","frames":[
        {"name":"a","start_ms":0,"time_ms":20,"angles":{"0":10.25,"1":20}},
        {"name":"b","start_ms":20,"time_ms":20,"angles":{"2":40},"torques":{"2":false}}]}]})";
    for (int scenario=0; scenario<8; ++scenario) {
        MockMotionHardware hw; JointAngles initial;
        for(int id=0; id<23; ++id) initial[id]=1.0;
        hw.setPresentPositions(initial); hw.setAutoReachGoal(false);
        double ms=1000;
        RobotMotionPlayer p(path.string(),hw,[&] {return RobotMotionPlayer::Clock::time_point(
            std::chrono::milliseconds(static_cast<int>(ms)));});
        assert(p.initialize());
        bool rejected_gate=false;
        try { p.setPositionToleranceEnabled(true); } catch(const std::invalid_argument&) { rejected_gate=true; }
        assert(rejected_gate);
        p.configureJointPolicy(path.string(), scenario==6, true);
        if(scenario==0) {
            hw.setReadSuccess(false);
            assert(p.start("a")==StartResult::HardwareNotReady);
            assert(hw.commandCount()==0); continue;
        }
        if(scenario==7) {
            assert(p.startPoseTransition(std::vector<double>(23,15),20));
            ms+=20; p.update(); assert(p.status()==MotionStatus::Settling);
            ms+=100; p.update(); assert(!p.succeeded());
            hw.setPresentPositions(JointAngles{});
            ms+=3100; p.update(); assert(p.result()==MotionError::PositionTimeout);continue;
        }
        assert(p.start("a")==StartResult::Accepted);
        assert(p.start("a")==StartResult::RejectedBusy);
        ms+=5; p.update();
        assert(hw.lastDurationMs()==0 && hw.lastAccelerationMs()==0);
        if(scenario==1) {
            assert(p.queueNext("a")==QueueResult::Queued);
            hw.setCommandSuccess(false); ms+=20; p.update();
            assert(p.result()==MotionError::FrameSendFailed && !p.hasQueuedMotion());continue;
        }
        if(scenario==2) { assert(p.cancel()==CancelResult::Cancelled);assert(hw.holdCount()==1);continue; }
        if(scenario==3) { assert(p.emergencyStop());assert(!p.hardwareReady());continue; }
        if(scenario==4) hw.setReadSuccess(false); // observation failure never adds a position gate
        if(scenario==5) {
            assert(p.queueNext("a")==QueueResult::Queued);
            p.setQueuedTransitionHoldMs(20);
        }
        if(scenario==6) p.setJointOverride(0,-64.25);
        for(int i=0; p.running() && i<100; ++i) {ms+=5; p.update();}
        assert(p.succeeded());assert(!p.completionConfirmsArrival());assert(ms<1200);
        assert(std::abs(hw.commandedPositions().at(1)-20)<1e-9);
        assert(std::abs(hw.commandedPositions().at(2)-40)<1e-9);
        assert(hw.ready()); // natural completion never disables torque
        if(scenario==6) assert(hw.commandedPositions().at(0)==-64.25);
        if(scenario==5) assert(p.completionSequence()==2);
        if(scenario==4) hw.setReadSuccess(true);
        assert(p.start("a")==StartResult::Accepted);
        p.update();
        if(scenario!=6) assert(hw.commandedPositions().at(0)==1.0); // new start reads PP, not old goal
        assert(hw.commandedPositions().at(1)==1.0);
    }
    std::cout<<"PASS start read failure, goal failure/queue purge, cancel, E-stop, feedback failure, queue hold, override, startup settling\n";
}
