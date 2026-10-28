# Jetson 모션 실행기 변경 기록 — 2026-10-27

연속 5차 보간(기본 ON)과 5번 모터 제외(기본 OFF)를 실제 실행 경로에 반영했다.
실행 노드를 수정 SDK로 재빌드했다. 모터에 명령을 보내거나 실제 보행을 시험하지는 않았다.
최신 PC GUI와 PC의 73개 테스트는 이 환경에서 확인하지 못했다. 아래 ON 검증은 사용자 제공
수식·규칙을 Python으로 구현한 기준과의 대조이며, 최신 GUI 코드 직접 대조와 구분해야 한다.

## 실행 경로와 변경 파일

`src/mission_control/launch/full_system_robot.launch.py`
또는 `src/irc_step_motion_executor/launch/sdk_gui_alignment.launch.py`
→ `sdk_motion_executor` (`src/irc_step_motion_executor/src/sdk_executor_node.cpp`)
→ `ProductionRobotMotionRuntimeFactory` → `RobotMotionPlayerBackend`
→ `external_sdk/gui_aligned/robot_motion_player.cpp`
→ `DynamixelMotionHardware::commandImmediatePosition()`
→ `Dxl::WriteGoalDegrees()` (`step_dynamixel.cpp`)
→ Dynamixel SDK `GroupSyncWrite(116, 4)`.

- `gui_playback.hpp`: 인접 구간 속도와 5차 보간. 구간 정보는 시작/반복 경계에서 생성한다.
- `motion_pattern.hpp/.cpp`: `stop_at_end` 파싱. 모션 파일 자체는 변경하지 않았다.
- `robot_motion_player.hpp/.cpp`: 기본 ON, 실제 시작각, 종점 전송, 반복 목표 누적,
  제외 모터 필터, startup 검사 연결. OFF는 코사인·80% 발 들기 옵션을 유지한다.
- `step_dynamixel.hpp/.cpp`, `dynamixel_motion_hardware.hpp/.cpp`, `motion_hardware.hpp`:
  활성 ID의 읽기·프로파일·Goal·토크 ON/확인, 5번 포함 전체 토크 OFF.
- `robot_motion_runtime_config.hpp`, `production_robot_motion_runtime_factory.cpp`,
  `sdk_executor_node.cpp`, 두 launch 파일: 옵션 전달과 상태 표시.
- `sdk_hardware_preflight_core.cpp`: 별도 사전 점검 CLI에도 `--exclude-motor-5` 지원.
- CMake, 테스트, 비교 도구, README: 빌드와 검증 연결.

정확한 이번 변경 파일·해시는 [changes.json](changes.json)에 있다. 작업 전부터 존재하던
사용자 변경과 구분했으며, `git_diff.before.patch`와 `git_status.before.txt`도 보관했다.

기존 빌드 캐시의 SDK 경로는 저장소 밖
`/home/jet/IRC/external_sdk/robot_motion_player_sdk_work_20260801/final step`이었다.
현재는 `/home/jet/IRC/IRC-STEP/IRC_vision_latest/external_sdk/gui_aligned`이다.
설치된 실행 파일은 `build/irc_step_motion_executor/sdk_motion_executor`로 연결된다.
[실행 파일·SDK 증거](environment.json), [동적 라이브러리](binary_dependencies.txt)를 참조한다.

## 제어 동작과 차이

연속 모드는 같은 관절이 맞닿은 두 프레임에 있고 방향이 같을 때만
`sign(delta) × min(abs(delta_left/h_left), abs(delta_right/h_right))`를 연결한다.
실제 구간 시간 `h = time_ms / (1000 × playback_speed)`를 사용한다.
방향 반전, 이동량 0, 대기, 관절 누락 경계, 시퀀스 시작/끝,
`[착지]` 양쪽 경계, `playback_cycle_end`와 `stop_at_end` 끝은 속도 0이다.
래핑을 거치는 구간에도 속도를 연결하지 않는다. 구간 끝 가속도는 0이다.
시작/끝 각도는 원본 표현을 정확히 사용하고 누락 관절은 직전 목표를 유지한다.

모든 신규 실기 `start()`는 Present Position을 새로 읽는다. 반복은 이전 목표를 이어받는다.
기존 코드에는 마지막 프레임에 있는 관절만 반복 시작각으로 반영하는 문제가 있어,
부분 프레임에서 목표가 옛 시작각으로 되돌아갈 수 있었다. 모든 프레임의 마지막 목표를
누적하도록 고쳤다. **이 수정은 OFF에도 적용**되므로 이 경우에는 구 GUI와 동작 차이가 있다.
또한 하드웨어 설정의 기본 ID 목록에 전달 ID를 추가해 46개가 될 수 있던 문제를
`clear()` 후 복사하도록 수정했다. ID 0~22와 인덱스는 유지한다.

연속 모드는 늦은 tick에서 지나간 종점을 전송한 뒤 현재 시각을 샘플링한다.
통신 지연을 반복/내부 cycle의 기준 시각에 추가하지 않는다. 일반 모션은 실제 도착을
기다리지 않고 전송 완료 시 성공 처리한다. 별도 startup 자세의 기존 도착 검사는 유지한다.
배속·반복은 최상위 `playback_speed`/`repeat_count`만 적용한다. 조합 JSON의 이미 환산된
프레임 시간에 원본 메타데이터의 배속·반복을 다시 적용하지 않는다.

5번 제외 시 하드웨어 읽기/설정/ON/확인/Goal에서 5번만 건너뛴다. Present Position 결과에도
가짜 5번 값을 넣지 않으며 startup 도착 검사에서도 제외한다. 전체 OFF는 5번에도 시도하되,
5번 미응답은 활성 모터 처리의 실패로 간주하지 않는다. 다른 모터 실패는 계속 검사한다.
토크 ON 후와 새 재생 전 활성 모터의 Torque Enable을 읽어 OFF/확인 실패를 차단한다.
5번 상태는 로그의 `EXCLUDED / UNCONFIRMED (제외/미확인)` 및 heartbeat의
`motor_5_status: "excluded_unconfirmed"`이다. 포함 상태의 `included`도 토크 ON 보증은 아니다.

이 옵션은 5번의 실제 토크를 유지하거나 Shutdown을 해제하지 않는다. 기존 초기화의 전체
토크 OFF가 5번에도 전달될 수 있다. PID·전류 제한·Shutdown 쓰기는 추가하지 않았다.
기존 Drive Mode/펌웨어 검사와 현재 위치 Goal 선등록 → 토크 ON 순서는 유지했다.

## 환경·설정·실행

이 환경은 aarch64, ROS 2 Humble이며 설치된 Dynamixel SDK 패키지는
`4.0.3-1jammy.20260307.131019`, Eigen `3.4.0-2ubuntu2`, json-c `0.15-3~ubuntu1.22.04.2`다.
SDK 헤더는 `/opt/ros/humble/include`, 라이브러리는 `/opt/ros/humble/lib/libdynamixel_sdk.so`다.
설정은 Protocol 2.0, 4,000,000bps, ID 0~22, Goal 116/4바이트, Position 132/4바이트,
Profile Acceleration 108/4바이트와 Velocity 112/4바이트를 재생 전 0으로 쓴다.
4096 step/rev, 중립 2048, 최종 raw 0~4095, ties-to-even 반올림이다.

노드의 wall timer는 5ms, 플레이어 경과시간은 `steady_clock`, 피드백은 약 10ms다.
프레임 경계에서는 별도 피드백이 발생할 수 있다. 실제 버스 전송 주기와 지터를 실측한 것은
아니다. 이 작업 환경에서 `/dev/ttyUSB*`와 실행 중인 실기 노드는 보이지 않았다.
전체 시스템의 설정 포트는 `/dev/ttyUSB0`; 실제 연결 포트/독점 사용 여부는 현장에서 확인한다.

재빌드:

```bash
cd /home/jet/IRC/IRC-STEP/IRC_vision_latest
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-select irc_step_motion_executor mission_control --symlink-install \
  --cmake-args -DIRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON \
  -DROBOT_MOTION_SDK_DIR="$PWD/external_sdk/gui_aligned" \
  -DDYNAMIXEL_SDK_INCLUDE_DIR=/opt/ros/humble/include \
  -DDYNAMIXEL_SDK_LIBRARY=/opt/ros/humble/lib/libdynamixel_sdk.so
source install/setup.bash
```

하드웨어 없이 실제 플레이어/전송 구현 검사:

```bash
ctest --test-dir build/irc_step_motion_executor --output-on-failure \
  -R 'test_continuous|test_legacy_cosine|test_gui_alignment|test_sdk_hardware_preflight_core'
```

실기 실행은 먼저 시뮬레이션/가짜 통신 검사 후 로봇을 지지한 상태에서 수행한다.
다음 명령은 **활성 모터의 토크를 ON**하며, 모션 요청 전에는 현재 자세를 Goal로 유지한다.
전체 시스템과 동시에 실행해 동일 포트를 두 프로세스가 열지 않도록 한다.

```bash
ros2 launch irc_step_motion_executor sdk_gui_alignment.launch.py \
  backend_type:=robot_motion_player enable_robot_hardware:=true \
  explicit_torque_approval:=true robot_device_path:=/dev/ttyUSB0 \
  continuous_interpolation:=true exclude_motor_5:=false \
  motion_trace_path:=/tmp/step_continuous.csv
```

5번 제외 시 `exclude_motor_5:=true`, 코사인 복귀 시 `continuous_interpolation:=false`로
노드를 다시 실행한다. 옵션은 시작 시 설정한다. 저장 JSON에서 5번 각도를 삭제하지 않는다.
이 launch는 startup 이동과 머리/어깨 override를 꺼 PC 모션 비교에 사용한다.
검증할 alias를 지정해 별도 터미널에서 한 번 요청한다:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run irc_step_motion_executor manual_motion_request.py --ros-args \
  -p motion_id:=line_forward_2 -p timeout_ms:=15000
```

전진 요청은 지지 장치와 정지 수단을 준비한 실기 단계에서만 실행한다.
전체 시스템에는 아래와 같이 같은 옵션을 전달한다. 이 경로는 기존 startup 이동과
카메라 관절 override 정책을 포함하므로 순수 GUI 각도 비교와 구분한다.

```bash
ros2 launch mission_control full_system_robot.launch.py \
  continuous_interpolation:=true exclude_motor_5:=false
```

## 검증 결과와 한계

- ROS 실행 노드와 mission_control 빌드 성공. 변경 관련 최종 선택 검사 5개 통과: [final_tests.log](final_tests.log).
- 제외/미확인 heartbeat를 확인하는 simulated ROS 통합 검사 통과: [exclusion_status_launch.log](exclusion_status_launch.log).
- SDK 단독 5개 검사 통과: [sdk_tests.log](sdk_tests.log).
- 런타임/PC 카탈로그 각각 83개 모션, 각각 611,133개 보간 목표 비교.
  최대 각도 오차 `2.842170943040401e-14°`, raw 차이 0.
- 실제 `RobotMotionPlayer`를 가짜 하드웨어로 실행: 카탈로그별 83개 × 정상/지연 두 조건,
  각각 686,711개 tick 목표 비교, raw 차이 0. 4개 합성 모션도 검증했다.
- Python/C++ 반올림 경계 및 인접값 12,291개 일치. [parity.json](parity.json).
- 코사인 OFF: 보관된 20261006 GUI 함수와 524개 경우 통과.
  부분 프레임 반복의 전체 목표 유지 수정만 기준 GUI 상태에도 적용했다.
  이를 `--carry-all-repeat-goals`와 결과 메타데이터에 명시했다.
  원본 GUI 파일은 수정하지 않았다. [legacy_comparison.json](legacy_comparison.json).
- 패킷 검사: 5번 미응답 제외 ON 성공/OFF 실패, 다른 ID 미응답·토크 OFF·확인 실패 차단,
  현재 위치 선등록, 108/112=0, 116 중간 Goal, startup/반복/전체 OFF 검증.
- 기존 전체 회귀가 모두 통과한 것은 아니다. Python 검사 `55 passed / 64 failed`는
  변경 전 백업에서도 동일하다(오래된 모션명/카탈로그 기대값 등).
  C++ catalog core 1개, executor core 4개, driver 1개의 기존 실패도 백업에서 재현했다.
  ball head override launch의 heartbeat 단언 실패 역시 변경 전 실행 파일로 재현했다.
  hurdle launch는 소켓 제한 없는 재검사에서 통과했다. 관련 `*_baseline.log`와
  `python_tests.log`, `ros_launch_recheck.log`에 근거를 남겼다.
- 최초 전체 검사/중간 검사 로그에는 수정 중의 결과도 포함된다. 최종 변경 검증은
  `final_tests.log`, `parity.json`, `ros_build.log`를 기준으로 본다.

PC에서 통과했다는 73개 테스트를 여기서 재실행한 것은 아니다.
최신 PC `sdk_gui.py`가 제공되면 같은 시작각/모션/시각을 주입해 직접 대조해야 한다.
코사인과 연속 모드는 구간 중간 각도 및 피크 속도/가속도가 달라진다. 수치·통신 테스트가
통과해도 흔들림 감소, 지면 접촉 안정성, 실제 속도 추종은 보장하지 않는다.

실기에서 추가로 확인할 항목:

1. 지지 상태에서 실제 포트, 모터 응답, 토크 ON 전 Goal≈Present Position 확인.
2. 5번 제외 ON/OFF 각각의 읽기·Goal ID 목록 및 전체 OFF 시도 확인.
   제외한 어깨의 실제 상태가 자세/균형에 미치는 영향도 관찰.
3. 단일 동작/낮은 배속부터 목표·피드백·전송 간격을 기록.
   5ms/10ms의 실제 지터, 통신 오류, 추종 오차, 전류/온도 변화를 비교.
4. 착지/반전/반복 경계의 목표 속도와 실제 속도, 발 접촉, IMU 흔들림을 같은 초기조건에서
   ON/OFF 비교. 단순 완료 상태는 일반 재생의 실제 도착 확인이 아니다.
5. 전신 자동 보행은 위 확인 후 진행. 기존 위치 피드백 실패는 일반 재생에서 관측 실패로
   기록되며, 매 tick 토크/도착 감시로 확대하지 않았다. 시작각·토크 사전 확인과 startup
   도착 실패는 별도로 차단한다.

## 복원

작업 시작 상태 전체를 [before.tar.gz](before.tar.gz)에 보관했다. 코드, 설정, 두 모션 JSON,
기존 build/install을 포함한다. 이번 변경 파일만 되돌리려면 모든 실행 노드를 종료한 뒤:

```bash
cd /home/jet/IRC/IRC-STEP/IRC_vision_latest
python3 artifacts/20261027_continuous_playback/restore.py --with-build
```

`--with-build`는 이전 실행 파일과 빌드/설치 파일도 덮어쓴다. 이후에 수정한 내용은 먼저
별도로 보관해야 한다. 새 파일에 후속 수정이 있으면 스크립트는 삭제 전 중단한다.
소스만 복원하려면 옵션을 생략하고, 실행 전에 반드시 이전 SDK 설정으로 다시 빌드한다.
기존 빌드 캐시도 백업돼 있다. 백업/검증 기록은 복원 후에도 남는다.
소스 복원은 임시 디렉터리에서 원본 해시와 일치함을 확인했다: [restore_validation.txt](restore_validation.txt).

모션 JSON은 이번 작업에서 바뀌지 않았다. 추후 변경 후 당시 JSON까지 되돌려야 할 때:

```bash
tar -xzf artifacts/20261027_continuous_playback/before.tar.gz \
  artifacts/robot_motions_runtime.json artifacts/robot_motions_pc.json
```
