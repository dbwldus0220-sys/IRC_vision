#include "dynamixel_motion_hardware.hpp"
#include "robot_motion_player.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
using namespace irc_step;

static void testGroupedTorqueVerification() {
    for (int scenario = 0; scenario < 13; ++scenario) {
        dynamixel::reset();
        Dxl dxl;
        if (scenario == 10) dxl.SetExcludeMotor5(true);
        if (scenario == 12) {
            assert(!dxl.VerifyTorqueEnabled(true));
            assert(dynamixel::events.empty());
            continue;
        }
        assert(dxl.Initialize("fake", 4000000));
        for (int id = 0; id < 23; ++id) dynamixel::registers[id][64] = 1;
        if (scenario == 1) dynamixel::registers[20][64] = 0;
        if (scenario == 2) dynamixel::offline = 22;
        if (scenario == 3) dynamixel::torque_read_failure = 20;
        if (scenario == 4) dynamixel::torque_packet_error = 22;
        if (scenario == 5) dynamixel::torque_group_failure = true;
        if (scenario == 6) dynamixel::torque_group_missing = 20;
        if (scenario == 7) dynamixel::torque_group_error_unavailable = 22;
        if (scenario == 8) dynamixel::torque_group_add_failure = 20;
        if (scenario == 9) dynamixel::all_offline = true;
        if (scenario == 10) {
            dynamixel::offline = 5;
        }
        if (scenario == 11)
            for (int id = 0; id < 23; ++id) dynamixel::registers[id][64] = 0;
        const bool expected = scenario == 0 || (scenario >= 5 && scenario <= 7)
            || scenario == 10 || scenario == 11;
        assert(dxl.VerifyTorqueEnabled(scenario != 11) == expected);
        int grouped = 0, individual = 0;
        for (const auto& event : dynamixel::events) {
            assert(event.address == 64 && event.size == 1);
            assert(event.type == 'R' || event.type == 'r'); // Verification never writes torque.
            if (event.type == 'R') {
                ++grouped;
                assert(event.data.size() == (scenario == 10 ? 22u : 23u));
            } else ++individual;
            if (scenario == 10) assert(!event.data.contains(5));
        }
        assert(grouped == (scenario == 8 ? 0 : 1));
        const bool fallback = scenario == 2 || scenario == 3
            || (scenario >= 5 && scenario <= 7) || scenario == 9;
        assert((individual > 0) == fallback);
        if (scenario == 0) {
            // A new check must observe a motor that turned off after the previous check.
            dynamixel::registers[20][64] = 0;
            assert(!dxl.VerifyTorqueEnabled(true));
        }
    }
    std::cout << "PASS grouped torque verification: normal, off, missing, packet error, fallback, exclusion\n";
}

static uint32_t lastSentGoal(int id) {
    for (auto event = dynamixel::events.rbegin(); event != dynamixel::events.rend(); ++event)
        if (event->type == 'W' && event->address == 116 && event->data.contains(id))
            return event->data.at(id);
    assert(false);
    return 0;
}

static void testCompetitionContinuation(const std::string& path) {
    std::ofstream(path) << R"({"motions":[{"name":"continue","repeatable":true,
      "start_pose":"same","end_pose":"same","max_seq_ms":100,"frames":[
      {"name":"frame","start_ms":0,"time_ms":100,"angles":{"1":20,"5":80,"22":40}}]}]})";
    for (bool continuous : {false, true}) for (bool queued : {false, true}) {
        for (int fault = 0; fault < 5; ++fault) {
            dynamixel::reset();
            DynamixelMotionHardware hw;
            int ms = 1000;
            RobotMotionPlayer p(path, hw, [&] {
                return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
            });
            p.setPositionToleranceEnabled(false);
            p.setContinuousInterpolation(continuous);
            assert(p.initialize());
            assert(p.start("continue") == StartResult::Accepted);
            if (queued) assert(p.queueNext("continue") == QueueResult::Queued);
            p.update();
            if (fault == 0) dynamixel::registers[5][64] = 0;
            if (fault == 1) dynamixel::torque_read_failure = 5;
            if (fault == 2) dynamixel::offline = 5;
            if (fault == 3) dynamixel::offline = 22;
            if (fault == 4) dynamixel::position_error = 5;
            for (int tick = 0; p.running() && tick < 100; ++tick) { ms += 5; p.update(); }
            assert(p.succeeded() && !p.completionConfirmsArrival());
            assert(p.completionSequence() == (queued ? 2 : 1));

            // 정상 관절은 새 실측값, 응답 없는 관절만 직전 송신 목표를 사용한다.
            dynamixel::registers[1][132] = 2160;
            dynamixel::events.clear();
            assert(p.start("continue") == StartResult::Accepted);
            p.update();
            assert(lastSentGoal(1) == 2160);
            assert(lastSentGoal(5) == (fault == 2 || fault == 4 ? uint32_t(guiPositionRaw(80)) : 2053));
            assert(lastSentGoal(22) == (fault == 3 ? uint32_t(guiPositionRaw(40)) : 2070));
            for (const auto& event : dynamixel::events)
                assert(!(event.type == 'w' && event.address == 64));
            for (int tick = 0; p.running() && tick < 100; ++tick) { ms += 5; p.update(); }
            assert(p.succeeded());

            // 응답이 돌아오면 이전 목표 대신 다시 실측각에서 시작한다.
            dynamixel::offline = dynamixel::position_error = dynamixel::torque_read_failure = -1;
            dynamixel::registers[5][64] = 1;
            dynamixel::registers[5][132] = 2222;
            assert(p.start("continue") == StartResult::Accepted);
            p.update();
            assert(lastSentGoal(5) == 2222);
            assert(p.emergencyStop());
            assert(p.start("continue") == StartResult::HardwareNotReady);
        }
    }
    for (int scenario = 0; scenario < 6; ++scenario) {
        dynamixel::reset();
        DynamixelMotionHardware hw;
        int ms = 1000;
        RobotMotionPlayer p(path, hw, [&] {
            return RobotMotionPlayer::Clock::time_point(std::chrono::milliseconds(ms));
        });
        p.setPositionToleranceEnabled(false);
        if (scenario == 0) dynamixel::offline = 5;
        if (scenario == 0) { assert(!p.initialize()); continue; }
        assert(p.initialize());
        if (scenario == 1) {
            dynamixel::offline = 5;
            assert(p.start("continue") == StartResult::HardwareNotReady); // 기준각 없음
            continue;
        }
        assert(p.start("continue") == StartResult::Accepted);
        p.update(); ms += 50; p.update();
        if (scenario == 2) {
            assert(p.cancel() == CancelResult::Cancelled);
            dynamixel::registers[5][64] = 0;
            assert(!p.startPoseTransition(std::vector<double>(23, 0), 100));
        } else if (scenario == 3) {
            assert(p.cancel() == CancelResult::Cancelled);
            dynamixel::all_offline = true;
            assert(p.start("continue") == StartResult::HardwareNotReady);
        } else if (scenario == 4) {
            dynamixel::offline = 5;
            assert(p.cancel() == CancelResult::HoldFailed);
        } else {
            const auto last_successful_goal = lastSentGoal(5);
            dynamixel::write_failure = true;
            ms += 50; p.update();
            assert(p.result() == MotionError::FrameSendFailed);
            assert(p.start("continue") == StartResult::HardwareNotReady); // 프로파일 송신 실패
            dynamixel::write_failure = false;
            dynamixel::offline = 5;
            assert(p.start("continue") == StartResult::Accepted);
            p.update();
            assert(lastSentGoal(5) == last_successful_goal); // 실패한 목표로 덮어쓰지 않음
        }
    }
    std::cout << "PASS competition continuation: 20 fault/queue/interpolation cases, "
                 "6 strict-boundary/cache cases; no automatic torque re-enable\n";
}

int main(int argc,char** argv) {
    assert(argc==2);
    testGroupedTorqueVerification();
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
    testCompetitionContinuation(path);
    std::cout<<"PASS actual player -> hardware -> Dxl -> fake SDK packets; exclusion, torque checks, seed, profiles, startup, repeat, timeout, write failure, emergency OFF\n";
}
