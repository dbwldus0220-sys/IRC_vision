#pragma once
// 테스트 전용 전송 기록기. 실제 SDK 대신 별도 테스트 타깃에만 포함한다.
#include <array>
#include <cassert>
#include <cstdint>
#include <map>
#include <vector>
#define COMM_SUCCESS 0
#define DXL_LOBYTE(w) ((w)&0xff)
#define DXL_HIBYTE(w) (((w)>>8)&0xff)
#define DXL_LOWORD(l) ((l)&0xffff)
#define DXL_HIWORD(l) (((l)>>16)&0xffff)
namespace dynamixel {
struct Event { char type; int address, size; std::map<int, uint32_t> data; };
inline std::vector<Event> events;
inline std::array<std::map<int,uint32_t>,23> registers;
inline int offline = -1, torque_read_failure = -1;
inline bool write_failure = false;
inline void reset() {
    events.clear(); offline = torque_read_failure = -1;
    write_failure = false;
    for (int id=0; id<23; ++id) registers[id] = {{6,42},{10,4},{64,0},{132,uint32_t(2048+id)}};
}
class PortHandler {
public:
    static PortHandler* getPortHandler(const char*) { return new PortHandler; }
    bool openPort() { return true; }
    void closePort() {}
    bool setBaudRate(int baud) { assert(baud == 4000000); return true; }
};
class PacketHandler {
public:
    static PacketHandler* getPacketHandler(double protocol) { assert(protocol==2.0); static PacketHandler p; return &p; }
    int read1ByteTxRx(PortHandler*, int id, int address, uint8_t* value, uint8_t* error) {
        events.push_back({'r',address,1,{{id,0}}}); *error=0;
        if (id==offline || (address==64 && id==torque_read_failure)) return -1;
        *value=registers.at(id)[address]; return 0;
    }
    int write1ByteTxRx(PortHandler*, int id, int address, uint8_t value, uint8_t* error) {
        events.push_back({'w',address,1,{{id,value}}}); *error=0;
        if(id==offline) return -1;
        registers.at(id)[address]=value; return 0;
    }
    int write2ByteTxRx(PortHandler*, int id, int address, uint16_t value, uint8_t* error) {
        events.push_back({'w',address,2,{{id,value}}}); *error=0; return id==offline ? -1 : 0;
    }
    int read4ByteTxRx(PortHandler*, int id, int address, uint32_t* value) {
        *value=registers.at(id)[address]; return id==offline ? -1 : 0;
    }
};
class GroupSyncWrite {
    Event event_;
public:
    GroupSyncWrite(PortHandler*,PacketHandler*,int address,int size):event_{'W',address,size,{}} {}
    bool addParam(int id,uint8_t* data) {
        if(event_.data.contains(id)) return false;
        uint32_t value=0; for(int i=0;i<event_.size;++i) value|=uint32_t(data[i])<<(i*8);
        event_.data[id]=value; return true;
    }
    int txPacket() {
        events.push_back(event_);
        if(write_failure) return -1001;
        for(auto [id,value]:event_.data) if(id!=offline) registers.at(id)[event_.address]=value;
        return 0;
    }
    void clearParam() { event_.data.clear(); }
};
class GroupSyncRead {
    Event event_;
public:
    GroupSyncRead(PortHandler*,PacketHandler*,int address,int size):event_{'R',address,size,{}} {}
    bool addParam(int id) { return event_.data.emplace(id,0).second; }
    int txRxPacket() { events.push_back(event_); return event_.data.contains(offline) ? -1 : 0; }
    bool isAvailable(int id,int,int) { return event_.data.contains(id) && id!=offline; }
    uint32_t getData(int id,int address,int) { return registers.at(id)[address]; }
    void clearParam() { event_.data.clear(); }
};
}
