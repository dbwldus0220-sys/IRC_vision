#include "step_dynamixel.hpp"
#include "gui_playback.hpp"
#include <algorithm>
#include <iostream>  
#include <cmath>
#include <stdexcept>

// const char* getAvailableDeviceName()
// {
//     std::vector<const char*> candidates = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};
//     for (auto dev : candidates) {
//         auto portHandler = dynamixel::PortHandler::getPortHandler(dev);
//         if (portHandler->openPort()) {
//             portHandler->setBaudRate(BAUDRATE);
//             auto packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

//             uint16_t model_number;
//             uint8_t dxl_error = 0;
//             int dxl_comm_result = packetHandler->ping(portHandler, 1, &model_number, &dxl_error);
//             // ⚠️ 여기서 1은 네 다이나믹셀 ID. 네 환경에 맞게 바꿔줘야 함.

//             if (dxl_comm_result == COMM_SUCCESS) {
//                 std::cout << "[Info] Dynamixel found on " << dev << std::endl;
//                 portHandler->closePort(); // 다시 열기 위해 닫아줌
//                 return dev;
//             } else {
//                 portHandler->closePort();
//             }
//         }
//     }
//     std::cerr << "[Error] No valid Dynamixel port found" << std::endl;
//     exit(1);
// }


Dxl::Dxl() : portHandler(nullptr), packetHandler(nullptr) { initActuatorValues(); }

bool Dxl::Initialize(const std::string& device, int baud) {
    if (port_opened_) return true;
    if (portHandler == nullptr) {
        portHandler = dynamixel::PortHandler::getPortHandler(device.c_str());
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    }
    if (!portHandler->openPort()) return false;
    port_opened_ = true;
    if (!portHandler->setBaudRate(baud)) {
        portHandler->closePort();
        port_opened_ = false;
        return false;
    }
    // GUI와 동일하게 Operating Mode와 PID는 변경하지 않는다.
    return true;
}

Dxl::~Dxl() {
    // GUI 종료와 동일하게 torque OFF 명령 없이 포트만 닫는다.
    if (port_opened_) portHandler->closePort();
    delete portHandler;
}

bool Dxl::WriteGoalDegrees(const std::vector<double>& angles, const std::vector<int>& ids) {
    if (!port_opened_ || angles.size() != NUMBER_OF_DYNAMIXELS || ids.empty()) return false;
    dynamixel::GroupSyncWrite writer(portHandler, packetHandler, DxlReg_GoalPosition, 4);
    for (const int id : ids) {
        if (id < 0 || id >= NUMBER_OF_DYNAMIXELS || !std::isfinite(angles[id])) return false;
        uint8_t data[4]{};
        getParam(irc_step::guiPositionRaw(angles[id]), data);
        if (!writer.addParam(static_cast<uint8_t>(id), data)) return false;
    }
    return writer.txPacket() == COMM_SUCCESS;
}

bool Dxl::ReadPositionDegrees(std::vector<double>& angles) {
    if (!port_opened_) return false;
    dynamixel::GroupSyncRead reader(portHandler, packetHandler, DxlReg_PresentPosition, 4);
    for (int id = 0; id < NUMBER_OF_DYNAMIXELS; ++id)
        if (!reader.addParam(id)) return false;
    if (reader.txRxPacket() != COMM_SUCCESS) return false;
    for (int id = 0; id < NUMBER_OF_DYNAMIXELS; ++id)
        if (!reader.isAvailable(id, DxlReg_PresentPosition, 4)) return false;
    angles.resize(NUMBER_OF_DYNAMIXELS);
    for (int id = 0; id < NUMBER_OF_DYNAMIXELS; ++id) {
        const auto raw = static_cast<std::int32_t>(reader.getData(id, DxlReg_PresentPosition, 4));
        angles[id] = std::clamp((raw - 2048.0) * (360.0 / 4096.0), -180.0, 179.912109375);
    }
    return true;
}

// ************************************ GETTERS ***************************************** //

//Getter() : 각도 읽기(raw->rad)
bool Dxl::syncReadTheta()
{
    if (!port_opened_) return false;
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentPosition, 4);
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++)
        if (!groupSyncRead.addParam(dxl_id[i])) return false;
    if (groupSyncRead.txRxPacket() != COMM_SUCCESS) return false;
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++)
        if (!groupSyncRead.isAvailable(dxl_id[i], DxlReg_PresentPosition, 4)) return false;
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) {
        position[i] = groupSyncRead.getData(dxl_id[i], DxlReg_PresentPosition, 4);
    }
    groupSyncRead.clearParam();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) th_[i] = convertValue2Radian(position[i]) - PI - zero_manual_offset[i];
    return true;
}

//Getter() : 각도 getter() [rad]
VectorXd Dxl::GetThetaAct()
{
    if (!syncReadTheta())
        throw std::runtime_error("failed to read all DYNAMIXEL positions");
    return th_;
}

//Getter() : velocity 읽기 (raw data)
bool Dxl::syncReadThetaDot()
{
    if (!port_opened_) return false;
    dynamixel::GroupSyncRead groupSyncReadThDot(portHandler, packetHandler, DxlReg_PresentVelocity, 4);
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
        if (!groupSyncReadThDot.addParam(dxl_id[i])) return false;
    if (groupSyncReadThDot.txRxPacket() != COMM_SUCCESS) return false;
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
        if (!groupSyncReadThDot.isAvailable(dxl_id[i], DxlReg_PresentVelocity, 4)) return false;
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) {
        velocity[i] = groupSyncReadThDot.getData(dxl_id[i], DxlReg_PresentVelocity, 4);
    }
    groupSyncReadThDot.clearParam();
    return true;
}

//Getter() : 각속도 getter() [rad/s] 
//0.0239868240
VectorXd Dxl::GetThetaDot()
{
    if (!syncReadThetaDot())
        throw std::runtime_error("failed to read all DYNAMIXEL velocities");
    VectorXd vel_(NUMBER_OF_DYNAMIXELS);
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        const auto signed_velocity = static_cast<std::int32_t>(velocity[i]);
        vel_[i] = signed_velocity * 0.0239808239; // 1 raw = 0.229 rpm = 0.0239808239 rad/s
    }
    return vel_;
}

//Getter() : About dynamixel packet data
void Dxl::getParam(int32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}

//Getter() : 추정계산 (이전 세타값 - 현재 세타값 / 시간) [rad/s]
void Dxl::CalculateEstimatedThetaDot(int dt_us)
{
    if (dt_us <= 0) return;
    th_dot_est_ = (th_ - th_last_) / (dt_us * 1.0e-6);
    th_last_ = th_;
}

//Getter() : 각속도 추정계산 getter() [rad/s] 
VectorXd Dxl::GetThetaDotEstimated()
{
    return th_dot_est_;
}


//Getter() : PID gain getter()
// VectorXd Dxl:: GetPIDGain()
// {

// }

//Getter() : 전류값 [mA] 
void Dxl::SyncReadCurrent()
{
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentCurrent, 2);
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) groupSyncRead.addParam(dxl_id[i]);
    groupSyncRead.txRxPacket();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) current[i] = groupSyncRead.getData(dxl_id[i], DxlReg_PresentCurrent, 2);
    groupSyncRead.clearParam();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) cur_[i] = convertValue2Current(current[i]);
}

VectorXd Dxl::GetCurrent()
{
    SyncReadCurrent();
    return cur_;
}


//Getter() : 현재 모드 getter()
int16_t Dxl::GetPresentMode()
{
    return this->Mode;
}


// **************************** SETTERS ******************************** //

//setter() : 각도 setter() [rad]
bool Dxl::syncWriteTheta()
{
  if (!port_opened_) return false;
  dynamixel::GroupSyncWrite gSyncWriteTh(portHandler, packetHandler, DxlReg_GoalPosition, 4);

  uint8_t parameter[4] = {0};

  for (uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++){
    const int32_t raw = std::clamp(
        static_cast<int32_t>(std::lround(ref_th_[i] * RAD_TO_VALUE)), 0, 4095);
    getParam(raw, parameter);
    if (!gSyncWriteTh.addParam(dxl_id[i], parameter)) return false;
  }
  const int result = gSyncWriteTh.txPacket();
  gSyncWriteTh.clearParam();
  return result == COMM_SUCCESS;
}

bool Dxl::ConfigureTimeBasedProfile()
{
    if (!port_opened_) return false;

    // Drive Mode는 EEPROM이므로 호출 전 토크가 OFF여야 한다. GUI와 같이
    // 펌웨어 V42+ 및 bit2 적용 여부를 모든 관절에서 확인한다.
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; ++i) {
        uint8_t firmware = 0;
        uint8_t drive_mode = 0;
        uint8_t error = 0;
        int result = packetHandler->read1ByteTxRx(
            portHandler, dxl_id[i], DxlReg_FirmwareVersion, &firmware, &error);
        if (result != COMM_SUCCESS || error != 0 || firmware < MIN_TIME_PROFILE_FIRMWARE) {
            std::cerr << "[Error] Time-based profile requires firmware V42+, ID "
                      << int(dxl_id[i]) << " reports V" << int(firmware) << std::endl;
            return false;
        }
        result = packetHandler->read1ByteTxRx(
            portHandler, dxl_id[i], DxlReg_DriveMode, &drive_mode, &error);
        if (result != COMM_SUCCESS || error != 0) return false;
        if (!(drive_mode & DRIVE_MODE_TIME_BASED_BIT)) {
            result = packetHandler->write1ByteTxRx(
                portHandler, dxl_id[i], DxlReg_DriveMode,
                drive_mode | DRIVE_MODE_TIME_BASED_BIT, &error);
            if (result != COMM_SUCCESS || error != 0) {
                std::cerr << "[Error] Failed to enable time-based Drive Mode, ID "
                          << int(dxl_id[i]) << std::endl;
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        uint8_t verified_mode = 0;
        error = 0;
        result = packetHandler->read1ByteTxRx(
            portHandler, dxl_id[i], DxlReg_DriveMode, &verified_mode, &error);
        if (result != COMM_SUCCESS || error != 0
            || !(verified_mode & DRIVE_MODE_TIME_BASED_BIT)) {
            std::cerr << "[Error] Time-based Drive Mode verification failed, ID "
                      << int(dxl_id[i]) << std::endl;
            return false;
        }
    }

    return WriteZeroProfiles();
}

bool Dxl::WriteZeroProfiles() {
    if (!port_opened_) return false;
    dynamixel::GroupSyncWrite acceleration(
        portHandler, packetHandler, DxlReg_ProfileAcceleration, 4);
    dynamixel::GroupSyncWrite velocity(
        portHandler, packetHandler, DxlReg_ProfileVelocity, 4);
    uint8_t zero[4] = {0, 0, 0, 0};
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; ++i) {
        if (!acceleration.addParam(dxl_id[i], zero)
            || !velocity.addParam(dxl_id[i], zero)) return false;
    }
    if (acceleration.txPacket() != COMM_SUCCESS) return false;
    if (velocity.txPacket() != COMM_SUCCESS) return false;
    acceleration.clearParam();
    velocity.clearParam();
    return true;
}

bool Dxl::syncWriteTimeBasedTheta(const VectorXd& theta,
                                  const std::vector<int>& motor_ids,
                                  uint32_t duration_ms,
                                  uint32_t acceleration_ms)
{
    if (!port_opened_ || theta.size() != NUMBER_OF_DYNAMIXELS
        || motor_ids.empty()) return false;
    duration_ms = std::clamp<uint32_t>(duration_ms, 1, MAX_TIME_PROFILE_MS);
    acceleration_ms = std::min<uint32_t>(acceleration_ms, duration_ms / 2);

    dynamixel::GroupSyncWrite acceleration_writer(
        portHandler, packetHandler, DxlReg_ProfileAcceleration, 4);
    dynamixel::GroupSyncWrite duration_writer(
        portHandler, packetHandler, DxlReg_ProfileVelocity, 4);
    dynamixel::GroupSyncWrite goal_writer(
        portHandler, packetHandler, DxlReg_GoalPosition, 4);
    uint8_t duration_param[4] = {0};
    uint8_t acceleration_param[4] = {0};
    getParam(static_cast<int32_t>(duration_ms), duration_param);
    getParam(static_cast<int32_t>(acceleration_ms), acceleration_param);

    for (const int id : motor_ids) {
        if (id < 0 || id >= NUMBER_OF_DYNAMIXELS) return false;
        uint8_t goal_param[4] = {0};
        const double absolute_rad = theta[id] + PI;
        const int32_t raw = std::clamp(
            static_cast<int32_t>(std::lround(absolute_rad * RAD_TO_VALUE)), 0, 4095);
        getParam(raw, goal_param);
        if (!acceleration_writer.addParam(static_cast<uint8_t>(id), acceleration_param)
            || !duration_writer.addParam(static_cast<uint8_t>(id), duration_param)
            || !goal_writer.addParam(static_cast<uint8_t>(id), goal_param)) return false;
    }
    // GUI와 동일하게 Profile Acceleration -> Profile Time -> Goal 순서로 쓴다.
    // 일반 프레임도 0을 써서 직전 [착지] 설정이 남지 않게 한다.
    if (acceleration_writer.txPacket() != COMM_SUCCESS) return false;
    if (duration_writer.txPacket() != COMM_SUCCESS) return false;
    if (goal_writer.txPacket() != COMM_SUCCESS) return false;
    acceleration_writer.clearParam();
    duration_writer.clearParam();
    goal_writer.clearParam();
    return true;
}




//Setter() : 목표 세타값 설정 [rad]
void Dxl::SetThetaRef(const VectorXd& theta)
{
    if (theta.size() != NUMBER_OF_DYNAMIXELS)
        throw std::invalid_argument("theta must contain exactly 23 joints");
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS;i++) 
    {
        ref_th_[i] = theta[i]+PI;
        // std::cout << ref_th_[i] << std::endl;
    }
}

//setter() : 토크 setter() [Nm]
void Dxl::syncWriteTorque()
{
    dynamixel::GroupSyncWrite groupSyncWriter(portHandler, packetHandler, DxlReg_GoalCurrent, 2);
    uint8_t parameter[NUMBER_OF_DYNAMIXELS] = {0};
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        ref_torque_value[i] = torqueToValue(ref_torque_[i], i);
        if(ref_torque_value[i] > 1000) ref_torque_value[i] = 1000; //상한값
        else if(ref_torque_value[i] < -1000) ref_torque_value[i] = -1000; //하한값
    }
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        getParam(ref_torque_value[i], parameter);
        groupSyncWriter.addParam(dxl_id[i], (uint8_t *)&parameter);
    }
    groupSyncWriter.txPacket();
    groupSyncWriter.clearParam();
}

//Setter() : 목표 토크 설정 [Nm]
void Dxl::SetTorqueRef(VectorXd a_torque)
{
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) ref_torque_[i] = a_torque[i];
}

// Setter() : PID gain setter()
void Dxl::SetPIDGain(VectorXd PID_Gain)
{    
    uint8_t dxl_error = 0;
    
    if (PID_Gain.size() != 3)
    {
        std::cerr << "PID_Gain should have exactly 3 elements: P, I, and D gains." << std::endl;
        return;
    }
    
    uint16_t P_gain = static_cast<uint16_t>(PID_Gain(0));
    uint16_t I_gain = static_cast<uint16_t>(PID_Gain(1));
    uint16_t D_gain = static_cast<uint16_t>(PID_Gain(2));

    // P, I, D Gain을 각각의 레지스터에 설정
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        // P Gain 설정
        int result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionPGain, P_gain, &dxl_error);
        if (result != COMM_SUCCESS)
        {
            std::cerr << "Failed to set P gain for DXL ID: " << static_cast<int>(dxl_id[i]) << std::endl;
        }

        // I Gain 설정
        result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionIGain, I_gain, &dxl_error);
        if (result != COMM_SUCCESS)
        {
            std::cerr << "Failed to set I gain for DXL ID: " << static_cast<int>(dxl_id[i]) << std::endl;
        }

        // D Gain 설정
        result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionDGain, D_gain, &dxl_error);
        if (result != COMM_SUCCESS)
        {
            std::cerr << "Failed to set D gain for DXL ID: " << static_cast<int>(dxl_id[i]) << std::endl;
        }
    }
}

//Setter() : 현재 모드 설정
int16_t Dxl::SetPresentMode(int16_t Mode)
{
    if (Mode == Current_Control_Mode)
    {
        this->Mode = Current_Control_Mode;
        return Current_Control_Mode;
    }
    else if (Mode == Position_Control_Mode)
    {
        this->Mode = Position_Control_Mode;
        return Position_Control_Mode;
    }
    else
    {
        std::cerr << "[Error] Invalid operating mode requested." << std::endl;
        return -1;
    }
}

bool Dxl::SetTorqueEnabled(bool enabled)
{
    if (!port_opened_) return false;
    bool success = true;
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; ++i) {
        uint8_t error = 0;
        const int result = packetHandler->write1ByteTxRx(
            portHandler, dxl_id[i], DxlReg_TorqueEnable, enabled ? 1 : 0, &error);
        if (result != COMM_SUCCESS || error != 0) {
            std::cerr << "[Error] Torque " << (enabled ? "ON" : "OFF")
                      << " failed, ID: " << int(dxl_id[i]) << std::endl;
            success = false;
        }
    }
    return success;
}

bool Dxl::IsReady() const
{
    return port_opened_;
}

// **************************** Function ******************************** //

//Torque2Value : 토크 -> 로우 data
int32_t Dxl::torqueToValue(double torque, uint8_t index)
{
    int32_t value_ = int(torque * torque2value[index]); //MX-64
    return value_;
}

//Value2Radian (Raw data -> Radian)
float Dxl::convertValue2Radian(int32_t value)
{
    float radian = value / RAD_TO_VALUE;
    return radian;
}

//Value2Curret (Raw data -> Current)
// 1raw  = 3.36[mA]
// Range = 0 ~ 1941 (raw)
float Dxl::convertValue2Current(int32_t value)
{
    float current_ = value *3.36;
    return current_;
}

//각도(rad), 각속도(rad/s) 읽고, torque(Nm->raw) 쓰기 
//제어 주파수(전류제어 : 300, 위치제어 : ?)
void Dxl::Loop(bool RxTh, bool RxThDot, bool TxTorque)
{
    if(RxTh) syncReadTheta();
    if(RxThDot) syncReadThetaDot();
    if(TxTorque) syncWriteTorque();
    
}

//dxl 초기 세팅
void Dxl::initActuatorValues()
{
    for (int i =0; i< NUMBER_OF_DYNAMIXELS; i++)
    {
        torque2value[i] = TORQUE_TO_VALUE_MX_106;
    }


    
    for (int i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    zero_manual_offset[i] = 0;
}










// portHandler, dxl_id[i], DxlReg_PositionDGain, D_gain, &dxl_error


VectorXd Dxl::read_rad()
{
    VectorXd rdl_(NUMBER_OF_DYNAMIXELS);
    int32_t present_position = 0;
    for (int i =0; i< NUMBER_OF_DYNAMIXELS; i++)
    {
        packetHandler->read4ByteTxRx(portHandler, dxl_id[i], DxlReg_PresentPosition,(uint32_t*)&present_position);
        rdl_[i] = (present_position - 2048) * (2.0 * M_PI / 4096.0);
    }

    return rdl_;
}

void Dxl::MoveToTargetSmoothCos(const VectorXd& theta_goal, int steps, int delay_ms)
{
    VectorXd theta_now = read_rad();

    for (int s = 1; s <= steps; ++s)
    {
        double rate = 0.5 * (1 - cos(M_PI * double(s) / steps));
        VectorXd theta_interp = theta_now + (theta_goal - theta_now) * rate;
        SetThetaRef(theta_interp);
        syncWriteTheta();
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    SetThetaRef(theta_goal);
    syncWriteTheta();
    std::this_thread::sleep_for(std::chrono::seconds(3));
}
