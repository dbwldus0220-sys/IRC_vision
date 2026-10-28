# 2026-10-28 Jetson GUI 실행 정렬 결과

요청서 기준 구현, aarch64 빌드, 설치, 가상 하드웨어 검증을 수행했다. **지정 SHA256의 최신 GUI 원본은 발견하지 못했다. 실제 FTDI 포트는 확인했지만 4Mbps 읽기에서 ID 0~22 모두 응답 timeout이었다.** 따라서 최신 PC GUI와 실기 움직임의 완전 일치 또는 낙상 감소를 입증한 결과는 아니다.

## 1. 수정 전 확인한 차이

기존 미커밋 작업을 포함한 시작 상태를 `before.tar.gz`로 백업했다. 아래의 ‘수정 전’은 git HEAD가 아니라 그 백업 상태다. `changes_from_session_start.patch`에서 이번 작업만 구분할 수 있다.

| 항목 | 수정 전 코드의 동작 | 이번 수정 / 영향 |
|---|---|---|
| 지연 tick | `robot_motion_player.cpp:updateRunning()`의 연속 모드 `while(next_frame_)`가 지나간 모든 종점을 송신 | 직전 활성 프레임 종점만 송신하고 그 tick을 경계 시각으로 제한. 다음 tick은 실제 경과시간 사용 |
| 반복·내부 cycle | 연속 모드는 원래 시계 유지 또는 계획 길이만 더함 | 종점 송신·강제 피드백이 끝난 실제 시각으로 재설정. 통신 지연만큼 총 실행시간이 길어질 수 있음 |
| 부분 프레임 | `GuiContinuousTrajectory::sample()`이 누적 자세 전체의 commanded mask 반환 | 활성 프레임의 ID만 송신. 공백에서는 누적 자세 유지. 5번만 있는 제외 프레임에서는 송신 없이 시간 진행 |
| 정확한 경계 | 내부 cycle 경계에 다음 구간의 샘플을 선택할 수 있음 | 최종점/cycle에서는 종료 프레임의 저장된 최종각과 ID를 반환 |
| 연산 순서 | 전개한 다항식, ARM 최적화의 FMA 결합 | 요청한 `q0+m0*u+u*u*u*(c3+u*(c4+u*c5))` 순서. `-ffp-contract=off`를 하위 실제 전송 타깃에도 전파 |
| 시간 단위 | 궤적 경계속도 계산에 전체 배속을 섞음(기존에는 상쇄됨) | 내보낸 프레임 시간 기준 deg/ms로 계산, 전체 배속은 재생 시계에만 적용 |
| 별도 기준 JSON | 비교 launch의 override OFF가 별도 PC JSON에서 ID 0/4/5를 복원 | 비교 launch는 선택한 motion JSON 자체를 기준으로 사용. 임의 JSON의 목표가 다른 파일로 바뀌지 않음 |
| 단일 자세 | `startPoseTransition()`도 연속 모드 기본값을 따라감 | 단일 자세는 코사인. startup 도착 확인·settling·timeout 유지 |
| 옵션 상태 | ROS parameter 값을 변경해도 생성된 하드웨어 정책에는 미반영 가능 | 보간/5번 제외 parameter를 read-only로 선언. 변경은 노드 재시작 후 재탐색·토크 상태 검사 |
| 진단 | 통신 성공 bool만 기록, 모드/누락 ID 없음 | 모드, 제외 상태, SDK 결과, 응답 누락 mask, 원본/유효 프레임, cancel/fault/E-stop 이벤트 기록 |
| 동시 접근 | 하드웨어 객체의 호출 직렬화가 명시적이지 않음 | production 하드웨어 공개 버스 연산을 재귀 mutex로 직렬화. 별도 프로세스의 접근까지 막는 장치 잠금은 아님 |

프로파일 0 + 중간 Goal 송신, 시작 PP 읽기, 토크 ON 전 Goal 선등록, 정상 완료 시 토크 유지, Python ties-to-even 변환, 5번 제외 시 OFF 시도는 **작업 시작 시 이미 구현되어 있던 부분**이며 이번에 실제 하드웨어 소스를 가짜 SDK와 연결해 검증했다. 과거 시간 프로파일 API의 최소 1ms 제한은 남아 있지만 시퀀스 경로에서 호출하지 않는다. PID/Operating Mode/PWM·전류 제한/Shutdown을 초기화하지 않는다.

### 처음 차이가 발생한 지점

- 백업 소스를 별도로 다시 빌드했다. `delayed_boundary_before_after.json`은 동일 가상 시작각·tick `[50,75,5]`를 주입한 수정 전/후 이벤트다. 경과 125ms, 이벤트 5에서 수정 전은 B 프레임의 125ms 보간값을 보내고, 기준/수정 후는 A의 종점을 확정한 뒤 B의 100ms 시작값을 보낸다. 수정 후 46개 이벤트가 기준과 일치한다.
- 최적화 빌드 검증 중 `건공잡기`, 가상 단조시각 2450ms(시작 1000ms에서 경과 1450ms), 이벤트 712, ID 20에서 **각도는 동일한 -131.5283203125°인데 raw가 Python 552 / C++ 551**이었다. ARM FMA 결합을 끈 뒤 일치했다. 최초 실패 자료는 `first_raw_difference_before_fma_fix.json`이다. 이는 실제 모터 측정이 아니다.

## 2. 실제 실행 경로 및 파일

저장소 루트: `/home/jet/IRC/IRC-STEP/IRC_vision_latest`

| 단계 | 실제 파일 / 함수 |
|---|---|
| 비교용 launch | `src/irc_step_motion_executor/launch/sdk_gui_alignment.launch.py:generate_launch_description()` |
| 설치 launch | `install/irc_step_motion_executor/share/irc_step_motion_executor/launch/sdk_gui_alignment.launch.py` |
| 실행 바이너리 | `install/irc_step_motion_executor/lib/irc_step_motion_executor/sdk_motion_executor` |
| ROS 노드 | `src/irc_step_motion_executor/src/sdk_executor_node.cpp:SdkMotionExecutorNode()`; 5ms timer → `driver_->poll()` |
| 요청 및 alias | `sdk_executor_driver.cpp:handle_request()` → `sdk_executor_core.cpp:handle_request()/poll()`의 `catalog_.resolve()`; `config/motion_aliases.yaml` |
| 예시 alias | `sdk_forward_4` → `R전진45도(4회)`, `forward` → `R전진45도(8회)` |
| 기본 JSON | `install/irc_step_motion_executor/share/irc_step_motion_executor/config/robot_motions_runtime.json`; launch 인수로 명시 변경 가능 |
| backend 선택 | `src/irc_step_motion_executor/src/motion_backend_factory.cpp:create_motion_backend()`; `robot_motion_player`를 명시해야 실제 SDK 경로 선택 |
| 실제 소유 객체 | `production_robot_motion_runtime_factory.cpp:ProductionRobotMotionRuntimeOwner` → `DynamixelMotionHardware`, `RobotMotionPlayer` |
| JSON 로드 | `external_sdk/gui_aligned/motion_pattern.cpp:MotionLibrary::loadGuiJson()` → `parseMotionObject()` → `MotionPattern::validate()` |
| 플레이어 | `robot_motion_player_backend.cpp:poll_status()` → `RobotMotionPlayer::update()` → `updateRunning()` → `sendGuiSample()` / `writeGoals()` |
| 보간 | `external_sdk/gui_aligned/gui_playback.hpp:GuiContinuousTrajectory::reset()/sample()`, OFF는 `sampleGuiMotion()` |
| 하드웨어 | `external_sdk/gui_aligned/dynamixel_motion_hardware.cpp:commandImmediatePosition()/readPresentPositions()` |
| raw/패킷 | `step_dynamixel.cpp:Dxl::WriteGoalDegrees()` → `guiPositionRaw()` → `getParam()` → `dynamixel::GroupSyncWrite(116,4)::txPacket()` |
| 피드백 | `Dxl::ReadPositionDegrees()` → `dynamixel::GroupSyncRead(132,4)::txRxPacket()` |
| 실제 SDK 라이브러리 | `/opt/ros/humble/lib/libdynamixel_sdk.so` |

`build_manifest.json`에 설치/빌드 바이너리 SHA256, 실제 resolved path, CMake cache/link 명령, production compile flags, `ldd`, 링크된 플레이어/하드웨어 함수 심볼을 저장했다. `test_support/dynamixel_sdk`는 별도 테스트 타깃에서만 사용한다. production은 `/opt/ros/humble/include`의 헤더와 실제 SDK 라이브러리를 사용한다. backend 생성 실패가 simulated로 조용히 대체되는 경로는 없다.

`SdkExecutorCore::poll()`은 예약 RUNNING 통지가 보류되어도 `backend_.poll_status()`를 호출한다. 이 동작은 시작 시 이미 반영되어 있었고 테스트로 유지했다. SDK가 호환 자세로 판단한 예약은 내부 연결을 사용한다. 호환되지 않는 모션의 mission bridge 재요청에는 ROS 왕복/새 시작 PP 읽기가 발생하므로 이를 SDK 내부 예약과 동일한 무공백 전환으로 부르지 않는다.

전체 미션 launch `src/mission_control/launch/full_system_robot.launch.py`는 기본 startup 자세 전환과 머리 override가 활성화되어 있다. **그 기본 실행은 격리된 GUI 비교 조건과 다르다.** 이번 작업에서 미션 정책이나 모션 목표를 변경하지 않았다.

## 3. 환경과 모터 설정 비교

| 항목 | GUI 요청 기준 | 현재 Jetson 코드/환경 | 실물 확인 |
|---|---|---|---|
| 환경 | PC Python GUI | aarch64, ROS 2 Humble, L4T R36.5.2 | 로컬 환경에서 확인 |
| 프로토콜/baud | 2.0 / 4,000,000 | 동일, fake SDK도 값 검증 | 실제 4Mbps 읽기에서 전 ID timeout |
| ID | 0~22, 기본 제외 없음 | 동일, 5번 선택 제외 | 전 ID 모델 읽기 timeout(-3001) |
| 포트 | 후보 `/dev/ttyUSB0` | launch에서 명시 입력, 전체 미션 기본 후보 동일 | 호스트 `/dev/ttyUSB0`, FTDI by-id 확인 |
| Torque/Drive Mode | 64 / 10, firmware V42+, bit2 | 코드에서 확인, OFF 상태에서 필요한 bit만 추가하고 재읽기 | 실제 firmware/Drive Mode 미확인 |
| Profile Accel/Velocity | 108/112, 4바이트, 0 | 준비 시 0; 각 tick은 116만 송신 | fake SDK 패킷 검증 |
| Goal/Present | 116/132, 4바이트 | GroupSyncWrite/GroupSyncRead | 실제 wire 캡처 없음 |
| PID/Operating Mode/전류/PWM/Shutdown | GUI가 일괄 재설정하지 않음 | 초기화 경로에서 일괄 재설정하지 않음 | 기존 레지스터값 미확인; PID=850 가정하지 않음 |
| Homing Offset/방향 | 실제 PC 설정과 맞춰야 함 | degree 직접 전송 경로에 수동 offset/부호 반전 없음 | 모터 자체 EEPROM offset/방향 미확인 |
| 목표/피드백 주기 | 약 5ms / 읽기 시작 기준 10ms | 5ms ROS timer, 10ms 관측, 경계 강제 읽기 | 실측 주기 미확인 |
| UI/로그 | 약 30fps / 정상 로그 약 10Hz | Jetson은 GUI 없음. 진단 trace는 별도 비동기 스레드가 모든 이벤트 기록 | GUI 렌더링 지연은 별도 비교 필요 |
| 5번 상태 | 제외/미확인 | heartbeat `excluded_unconfirmed`, 토크 ON으로 간주 안 함 | 제외한 5번 OFF도 시도, 응답 보장 안 함 |

`execution_audit_before.json`은 샌드박스 안의 초기 관측이며 포트가 가려져 있었다. 호스트 읽기 전용 검사 `host_device_audit.json`에서 `/dev/ttyUSB0` 및 `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAKRKTW-if00-port0`, USB latency **16ms**, SDK 실행기/GUI 프로세스 없음, `fuser` 포트 점유 없음까지 확인했다.

`read_motor_settings.py`는 쓰기 함수를 사용하지 않고 모델/설정 레지스터만 읽는 점검 스크립트다. 실제 Protocol 2.0·4Mbps에서 **ID 0~22 모두 첫 모델 읽기(주소 0, 2바이트)가 COMM_RX_TIMEOUT=-3001**로 실패하여 해당 ID의 나머지 설정 읽기를 건너뛰었다. `motor_register_audit.json`과 `motor_register_audit_summary.json`에 단조시각·SDK 결과를 저장했다. 단일 `read2ByteTxRx` timeout 소요시간 평균 **34.907ms**, p95 **35.231ms**, p99/최대 **35.395ms**였다. 이것은 SyncRead 주기나 Goal 송신 주기 측정이 아니다. 응답 없는 읽기의 반환 raw=0을 유효한 모델값으로 해석하면 안 된다. 전원/배선/현재 baud/프로토콜 등 원인은 미확정이다. 모터 쓰기 횟수는 **0**이며 USB latency도 바꾸지 않았다. 전체 읽기 가능한 파일시스템을 검색했지만 `continuous_trajectory.py`는 없었다. 발견한 `sdk_gui.py`:

- `/home/jet/IRC/external_sdk/robot_motion_player_sdk_work_20260801/final step/sdk_gui.py`: `1c3e8e00b7cb90a4a3416572e1bde8c58bb4853956315dea0053801e5888598b`
- 저장소 `artifacts/20261006_gui_alignment/reference/sdk_gui.py`: `ce6b871a14af1187a5e3f221834265649e1e819cb6b529487d75a090c7e2d527`

둘 다 요청한 `9d90a9f79cf5f099229e57b1c737e7208f0567a5c979a08ba94fb142d0252661`과 다르다. 요청한 continuous 파일 해시 `4608bb0d506ef000eb062e7cf85f6207807c51404199d2f39450b65fcc17460d`도 확인할 수 없다. Downloads의 `gui_sdk_20260930_135448.tar.gz`도 검사했으며 내부 sdk_gui.py 해시는 `5f510f76fb1eb7bd2c29f959026994d5365141bdf8f1e1a941cdf90bf5ed3f25`로 다르다. 다른 로컬 백업과 git 기록에서도 지정 파일을 찾지 못했다(`reference_archive_search.json`). 요청에 언급된 최신 PC 테스트 57개를 이 환경에서 실행했다고 주장하지 않는다.

## 4. 검증 결과와 해석

- `contract/results.json`: **892개** 요청서 기반 시나리오, **332,644개 이벤트**. PC/runtime 각각 86개 모션, 연속 ON/OFF, 정상/지연 tick, 합성 경계·부분 관절·정지·반전·착지·cycle·lift·제외·timeout·예약을 포함한다. 모든 W/R 순서·시각·대상 ID·각도·raw와 실행 중 timeline을 대조했다. 최신 GUI 소스 자체를 실행한 결과는 아니다.
- `continuous_parity.json`: 두 카탈로그 합계 **1,281,284개 보간 관절 샘플**, 최대 각도 차이 **0**, raw 불일치 **0**. 실제 C++ 플레이어의 정상/지연 실행 샘플 합계 **1,448,862개**, raw 불일치 **0**. 이 보조 검사는 tick의 마지막 송신값을 검사하며, 전체 송신 순서는 앞의 contract 검사에서 검증한다.
- raw half-step 및 인접 부동소수점/래핑 경계 **12,291개** 일치.
- `test_continuous_hardware`: production 플레이어 → production hardware → Dxl → 가짜 DYNAMIXEL SDK에서 주소·바이트 수·ID·프로파일 0·Goal 선등록·토크 확인·5번 미응답·다른 ID 미응답·제외 프레임 무송신·피드백 실패 지속·쓰기 실패 중단·전체 OFF를 검사한다.
- 단일 자세의 코사인 25% 시점 및 startup settling 유지, 취소 hold, 비상 OFF, 정상 완료 torque 유지도 검사한다.
- 기존 archived GUI의 실제 메서드를 AST로 실행하는 OFF 비교는 구형 소스에 대한 비교다. 부분 관절 반복은 요청된 전체 누적 규칙으로 oracle을 명시 보완한다. 이것을 최신 GUI 그대로의 결과로 해석하지 않는다.
- `python_tests.log`: trace/기록 재생/카탈로그 import 테스트 **12개 통과**. 기록 재생 시 보간 모드 metadata를 넘기고, metadata가 없는 구형 로그는 코사인으로 취급한다.
- `executor_final_tests.log`: 실행기 비네트워크 CTest **33개 통과**. `executor_latest_xunit_result.txt`에는 최신 xUnit **113개, 오류/실패 0**을 별도로 집계했다.
- `ros_launch_tests.log`: simulated/거부 backend ROS 통합 테스트 **5개 통과**. 처음 샌드박스 UDP 제한으로 실패해 localhost 전용 별도 도메인에서 다시 실행했다. 재생 옵션 변경 거부도 실제 parameter service로 확인했다.

확장한 `mission_control` 전체 시험은 **3,550 통과 / 293 실패**다. 이번 변경과 구분하기 위해 백업한 수정 전 source로 다시 실행했으며 **같은 293개 실패 목록**이 재현되었다(`mission_baseline_comparison.json`: only_current/only_baseline 모두 빈 목록). baseline 첫 실행에는 ROS 로그 디렉터리 쓰기 제한 오류 1개가 추가되어 3,549 통과였으며, 해당 1개는 `ROS_LOG_DIR=/tmp/step_alignment_baseline_logs`로 재실행해 통과했다(`mission_baseline_logging_retry.log`). mission 소스 239개 파일이 백업과 동일함도 확인했다. 기존 카탈로그/미션 기대값 충돌과 lint 문제를 이번 모터 실행 정렬 작업에서 임의로 바꾸지 않았다. 전체 저장소 테스트가 모두 통과한 상태는 아니다.

실행기 테스트에서는 현재 alias 107개 및 `R전진45도`를 기존 테스트의 오래된 기대값에 반영했다. alias YAML이나 실제 선택 모션은 변경하지 않았다. 최초 `colcon_test.log` 및 과거 CTest XML에는 초기 실패가 남아 있으므로 최종 상태는 `executor_final_tests.log`, `ros_launch_tests.log`, 최신 xUnit 집계로 구분한다.

숫자 예제(시작 0°, A=10°/100ms, B=20°/100ms):

| 시각 | 연속 ON 각도 | raw |
|---|---:|---:|
| 0ms | 0 | 2048 |
| 50ms | 3.4375 | 2087 |
| 100ms | 10 | 2162 |
| 150ms | 16.5625 | 2236 |
| 200ms | 20 | 2276 |

100ms 좌우 속도는 100°/s. OFF의 50ms 값은 5°. 연속 ON에는 발 들기 80% 단축을 적용하지 않는다.

PC/runtime JSON의 각도·시간·배속·반복은 이 세션에서 변경하지 않았다. 기존 파일 사이 차이는 모든 motion의 completion 정책과 `건공잡기`의 프레임 이름 2개 및 end_pose 이름 suffix다(`catalog_differences.json`). runtime tolerance 5.0과 ID 4/5 원본 목표 보존 정책을 유지한다.

### 조합 시간의 적용 범위

현재 Jetson의 실제 입력은 GUI에서 **저장·내보낸 frames JSON**이다. 이 경로는 원본 배속으로 정수 ms 변환된 프레임 시간을 읽고 전체 `playback_speed`만 적용한다. 보관된 GUI의 `build_composed_sequence_frames(apply_source_speed=True)`로 원본/전체 배속 조합을 만들어 검증했다. 추가 gap은 원본 배속으로 다시 줄이지 않는다.

Jetson에는 GUI 편집기 안의 live `sequence_composer_entries` 실행 API가 없다. 저장 이전의 piecewise `composer_timing_segments()/playback_time_ms()`와 export의 정수 ms 반올림은 동일한 데이터가 아니다. **최신 GUI의 live composer와 직접 대조는 미확인**이며, 비교할 때는 양쪽 모두 같은 내보낸 JSON을 사용해야 한다. 라이브 편집기 데이터를 임의 schema로 해석하는 기능은 추가하지 않았다.

## 5. 빌드·실행·측정 명령

작업 디렉터리에서:

```bash
cd /home/jet/IRC/IRC-STEP/IRC_vision_latest
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j3 colcon build --packages-select irc_step_motion_executor mission_control \
  --cmake-args -DIRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON \
  -DROBOT_MOTION_SDK_DIR="$PWD/external_sdk/gui_aligned"
source install/setup.bash
ros2 pkg prefix irc_step_motion_executor
ldd install/irc_step_motion_executor/lib/irc_step_motion_executor/sdk_motion_executor
```

하드웨어 없는 검증:

```bash
ctest --test-dir build/irc_step_motion_executor --output-on-failure
python3 tools/verify_gui_execution_contract.py \
  --probe build/irc_step_motion_executor/robot_motion_sdk/gui_execution_probe \
  --archived-gui artifacts/20261006_gui_alignment/reference/sdk_gui.py \
  --output /tmp/step_gui_contract \
  artifacts/robot_motions_pc.json artifacts/robot_motions_runtime.json
python3 -m pytest -q tools/test_playback_traces.py tools/test_upsert_motion_catalog.py
```

ROS 배선만 확인하는 기본 실행은 simulated이며 모터 패킷 검증을 대신하지 않는다:

```bash
ROS_LOG_DIR=/tmp/step_ros_logs ros2 launch irc_step_motion_executor sdk_gui_alignment.launch.py \
  backend_type:=simulated continuous_interpolation:=true exclude_motor_5:=false
```

실제 연결 경로는 아래 FTDI by-id로 확인했다. 다만 현재 모든 ID가 읽기 timeout이므로 전원·배선·baud 문제를 확인해 정상 응답을 확보해야 한다. 아래 하드웨어 실행 명령은 이번 세션에서 실행하지 않았다. 하드웨어 초기화는 Goal 선등록 뒤 토크를 켠다. 새 보간/경계 처리는 시뮬레이션과 지지 장치가 있는 시험을 먼저 거쳐야 한다.

```bash
STEP_PORT=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAKRKTW-if00-port0
readlink -f "$STEP_PORT"
fuser -v "$STEP_PORT"
# 다른 포트 사용 프로세스가 없는 것을 확인한 뒤 실행
ROS_LOG_DIR=/tmp/step_ros_logs ros2 launch irc_step_motion_executor sdk_gui_alignment.launch.py \
  backend_type:=robot_motion_player enable_robot_hardware:=true explicit_torque_approval:=true \
  robot_device_path:="$STEP_PORT" \
  motion_json_path:="$PWD/artifacts/robot_motions_pc.json" \
  continuous_interpolation:=true exclude_motor_5:=false \
  motion_trace_path:=/tmp/step_gui_alignment.csv
```

다른 터미널에서 같은 setup을 source한 뒤 한 모션 요청:

```bash
ros2 run irc_step_motion_executor manual_motion_request.py --ros-args \
  -p motion_id:=sdk_forward_4 -p timeout_ms:=60000
```

60초는 이 명령의 외부 watchdog 상한이며 계획 시간을 늘리는 설정이 아니다. 정상 목표 전송 완료는 물리적 도착 확인이 아니다. 기존 취소·쓰기 실패·외부 timeout·비상정지 경로는 유지된다. 노드 종료나 Ctrl-C 자체를 비상 토크 OFF로 가정하지 않는다.

측정 후:

```bash
python3 tools/analyze_motor_timing.py /tmp/step_gui_alignment.csv --output /tmp/step_motor_timing.json
python3 tools/analyze_callback_timing.py /tmp/step_gui_alignment.csv.callbacks.csv --output /tmp/step_callback_timing.json
```

trace는 단조시각, run/repeat/frame/timeline, 연속/제외 옵션, planned/sample/endpoint 목표, raw, feedback 시각, SDK 결과/누락 mask를 남긴다. `source_definition` → `definition`은 프레임 보정, `planned` → `sample`은 실행 중 override를 구분한다. Goal raw는 실제 Dxl 변환과 같은 함수이며 실물 수신 ack가 아니다. `dropped_rows`가 0인지 반드시 확인해야 전송 횟수/간격을 빠짐없이 해석할 수 있다.

`virtual_delayed_trace.csv`/`virtual_delayed_timing.json`은 가상 RX=16ms, TX=1ms, tick=[50,75,5] 예제다. TX 간격 평균 40.0625ms, p95/p99/max=92ms가 나오는 것은 **주입한 지연의 결과**이며 실측값이 아니다. `fake_sdk_timing.json`의 0ms 통신 시간도 가짜 SDK 시계다. USB latency 16ms와 위 단일 레지스터 읽기 timeout만 실측했다. 실제 SyncWrite/SyncRead 평균·p95·p99·max, 반복/예약 공백, 전체 길이는 정상 응답을 복구한 뒤 실물 로그가 있어야 판단할 수 있다.

## 6. 남아 있는 확인 사항

1. 지정 hash의 최신 GUI 두 파일 확보 및 실제 함수와 양쪽 packet trace 직접 대조. 현재 검증은 요청서 oracle + 구형 GUI 일부 함수다.
2. 동일 시작 자세/동일 JSON/배속/반복/연속 ON/OFF/5번 제외 선택을 양쪽에서 명시하고 실제 모터 설정을 읽어 비교. Homing Offset/Drive Mode 방향/Operating Mode/PID/전류·PWM·Shutdown은 미측정이다.
3. USB 장치/latency와 현재 포트 점유 없음은 확인했다. 모든 ID의 읽기 timeout 원인을 해결한 뒤 실제 Goal/SyncRead 간격·반복/예약 공백을 측정해야 한다. 객체 mutex는 다른 프로그램을 차단하지 않는다.
4. SyncRead 실패는 재생 시간을 인위적으로 늘리거나 즉시 중단시키지 않지만, blocking read timeout 자체는 다음 tick을 늦춘다. 시작 자세/토크 확인 실패와 Goal 송신 실패는 계속 실행을 거부·중단한다. 정상 타이밍을 위해 오류 처리를 제거하지 않았다.
5. GUI와 동일한 목표 명령이어도 균형·실제 추종·낙상은 확인되지 않았다. 실제 버스 캡처/로봇 로그와 시뮬레이션·지지 시험이 필요하다.
6. live composer/수동 GUI scrub에는 Jetson의 동일 UI 진입점이 없다. 저장한 시퀀스 실행 및 단일 자세 API와 구분한다.

## 7. 변경 전 복원

로봇 실행기를 정지하고 현재 작업도 별도로 보존한 뒤 실행한다. 백업은 git reset과 달리 **이 작업 시작 전의 미커밋 변경도 포함**한다. `.git`은 건드리지 않는다.

```bash
cd /home/jet/IRC/IRC-STEP/IRC_vision_latest/artifacts/20261028_gui_execution_alignment
sha256sum -c backup.sha256
cd /home/jet/IRC/IRC-STEP/IRC_vision_latest
tar -xzf artifacts/20261028_gui_execution_alignment/before.tar.gz -C "$PWD"
```

새로 추가한 `external_sdk/gui_aligned/bus_diagnostics.hpp`, `tools/verify_gui_execution_contract.py`, `tools/analyze_motor_timing.py`는 tar 복원으로 삭제되지 않는다. 복원 후에는 기존 CMake에서 사용하지 않으며, 이번 추가 파일임을 확인하고 제거할 수 있다. 새 검증 산출물 디렉터리는 증거로 남겨도 된다. 기존 `build/install`도 백업했지만 환경 변경 또는 링크 혼동을 피하려면 복원한 소스로 위 빌드를 다시 실행하고 `source install/setup.bash` 후 package prefix를 확인한다.
