#include "dynamixel_motion_hardware.hpp"
#include "robot_motion_player.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
using namespace irc_step;
int main(int argc,char** argv) {
    assert(argc==2);
    const std::string path=std::string(argv[1])+"/continuous_hardware.json";
    std::ofstream(path)<<R"({"motions":[{"name":"test","max_seq_ms":200,"repeat_count":2,"playback_speed":2,"frames":[
      {"name":"발들기","start_ms":0,"time_ms":100,"lift_early_arrival":true,"angles":{"1":20,"5":80}},
      {"name":"partial","start_ms":100,"time_ms":100,"angles":{"2":30}}]}]})";
    for(int scenario=0;scenario<7;++scenario) {
        dynamixel::reset();
        DynamixelMotionHardwareConfig config;
        config.exclude_motor_5=scenario!=0;
        dynamixel::offline=(scenario==2 ? 6 : 5);
        DynamixelMotionHardware hw(config);
        double ms=0;
        RobotMotionPlayer p(path,hw,[&]{return RobotMotionPlayer::Clock::time_point(std::chrono::microseconds(int64_t(ms*1000)));});
        const bool initialized=p.initialize();
        if(scenario==0 || scenario==2) { assert(!initialized); continue; }
        assert(initialized);
        bool seeded=false;
        for(const auto& event:dynamixel::events) {
            if(event.type=='W' && event.address==116) seeded=true;
            if(event.type=='w' && event.address==64 && event.data.begin()->second==1) assert(seeded);
        }
        if(scenario==3) dynamixel::registers[6][64]=0;
        if(scenario==4) dynamixel::torque_read_failure=6;
        if(scenario==5) dynamixel::offline=6;
        const auto start=p.start("test");
        if(scenario>=3 && scenario<=5) { assert(start==StartResult::HardwareNotReady); continue; }
        assert(start==StartResult::Accepted);
        assert(p.update()==MotionStatus::Running);
        // 실측 시작각: ID 1 raw=2049. 중립이나 이전 Goal로 대체하지 않는다.
        assert(dynamixel::registers[1][116]==2049);
        ms=25; p.update();
        assert(dynamixel::registers[1][116]==uint32_t(guiPositionRaw((20+360.0/4096)/2)));
        // 첫 반복에서 직전 부분 프레임에 없는 관절의 최종각도 유지한다.
        ms=100; p.update();
        ms=105; p.update();
        assert(dynamixel::registers[1][116]==uint32_t(guiPositionRaw(20)));
        for(ms=110;p.running() && ms<500;ms+=5) p.update();
        assert(p.succeeded()); assert(ms<=210);
        assert(dynamixel::registers[1][116]==uint32_t(guiPositionRaw(20)));
        assert(dynamixel::registers[2][116]==uint32_t(guiPositionRaw(30)));
        assert(p.startPoseTransition(std::vector<double>(23,0),20));
        for(int id=0;id<23;++id) dynamixel::registers[id][132]=2048;
        for(int i=0;p.running() && i<100;++i) { ms+=5; p.update(); }
        assert(p.succeeded());
        assert(p.emergencyStop());
        int samples=0,off5=0;
        for(const auto& e:dynamixel::events) {
            if(e.data.contains(5)) {
                assert(e.type=='w' && e.address==64 && e.data.at(5)==0); ++off5;
            }
            if(e.type=='W') {
                assert(e.size==4);
                assert(e.address==108 || e.address==112 || e.address==116);
                for(auto [id,value]:e.data) { if(e.address!=116) assert(value==0); else assert(value<=4095); }
                if(e.address==116) ++samples;
            }
            // PID, 전류 제한, Shutdown 쓰기가 없어야 한다.
            assert(e.address!=38 && e.address!=63 && e.address!=80 && e.address!=82 && e.address!=84);
        }
        assert(samples>20 && off5>=2);
    }
    // 제외 관절만 있는 활성 프레임에서는 다른 관절로 대체 전송하지 않는다.
    std::ofstream(path)<<R"({"motions":[{"name":"only5","max_seq_ms":300,"frames":[
      {"name":"A","start_ms":0,"time_ms":100,"angles":{"1":20}},
      {"name":"B","start_ms":100,"time_ms":100,"angles":{"5":80}},
      {"name":"C","start_ms":200,"time_ms":100,"angles":{"2":30}}]}]})";
    for(bool continuous : {false,true}) {
        dynamixel::reset();
        DynamixelMotionHardwareConfig config; config.exclude_motor_5=true;
        dynamixel::offline=5;
        DynamixelMotionHardware hw(config);
        int ms=0;
        RobotMotionPlayer p(path,hw,[&]{return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));});
        p.setContinuousInterpolation(continuous);
        p.configureDiagnostics(5,std::string(argv[1])+(continuous ? "/packet_fault_continuous.csv" : "/packet_fault_cosine.csv"),true);
        assert(p.initialize() && p.start("only5")==StartResult::Accepted);
        p.update(); ms=100; p.update();
        dynamixel::events.clear();
        ms=125; p.update();
        for(const auto& e:dynamixel::events) assert(e.type!='W');
        assert(p.playbackTimelineMs()==125 && p.running());
        // 일반 피드백 누락은 시간을 늘리지 않지만 SDK 오류/누락 ID는 남긴다.
        dynamixel::offline=6;
        ms=150; p.update();
        assert(p.running() && p.playbackTimelineMs()==150);
        assert(hw.busDiagnostics().valid && hw.busDiagnostics().result==-1);
        assert(hw.busDiagnostics().missing_ids==(1u<<6));
        dynamixel::write_failure=true;
        ms=200; p.update();
        assert(p.result()==MotionError::FrameSendFailed);
        assert(hw.busDiagnostics().result==-1001);
    }
    std::cout<<"PASS actual player -> hardware -> Dxl -> fake SDK packets; exclusion, torque checks, seed, profiles, startup, repeat, timeout, write failure, emergency OFF\n";
}
