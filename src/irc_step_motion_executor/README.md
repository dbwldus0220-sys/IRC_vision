# IRC STEP Motion Executor C++ Wrapper

이 패키지는 전달받은 C++ `RobotMotionPlayer` SDK를 향후 ROS 2에 연결하기 위한
독립 `ament_cmake` wrapper의 최소 골격이다. 현재 단계는
**catalog-only/mock-safe**이며 production motion executor가 아니다.

## 현재 동작

- `/motion/executor/request`, `/motion/executor/cancel`을
  `std_msgs/msg/String` JSON으로 구독한다.
- `/motion/executor/status`에 기존 JSON String 계약의 필드를 발행한다.
- `config/motion_aliases.yaml`을 읽어 motion alias 존재 여부만 검증한다.
- 알려진 alias의 start 요청도 `REJECTED` / `HARDWARE_NOT_READY`로 반환한다.
- 알려지지 않은 motion은 `REJECTED` / `INVALID_MOTION`으로 반환하며 다른
  motion으로 fallback하지 않는다.
- `RobotMotionPlayer`, Dynamixel backend 및 hardware 객체를 생성하지 않는다.

`forward: "전진"`과 `forward_short: "첫발"`은 실물 검증 전의 **개발 후보**
alias일 뿐 production 확정값이 아니다. `turn_left`, `shoot`, `hurdle` 등
확인되지 않은 motion은 의도적으로 매핑하지 않았다.

## JSON 계약

request는 `action`(string), `command_id`(integer 또는 null),
`event_id`(integer 또는 null), `request_id`(integer),
`motion_id`(string)를 필수 key로 사용한다. 특히 `event_id: null`은 유효하지만
`event_id` key 누락은 `REJECTED` / `INVALID_REQUEST`이다.

status는 `status`, `action`, `command_id`, `event_id`, `request_id`,
`motion_id`, `error_code`, `message`를 항상 포함한다. catalog-only core는
정상 계약 요청의 원래 action과 correlation 값을 terminal `REJECTED`
status까지 보존한다.

## Hardware-independent executor core

`MotionBackend`는 실제 SDK 연결 전 단계의 하드웨어 독립 계약이다.
`start_motion()`, `cancel_motion()`, `poll_status()`만 정의하며 실제 SDK
함수명이나 생성자 정보를 포함하지 않는다. `SdkExecutorCore`는 기존 JSON
request를 검증하고 alias를 resolve한 뒤 이 인터페이스를 통해서만 motion
상태를 처리한다.

core 단위 검증은 test 전용 `FakeMotionBackend`만 사용한다. 실제
`RobotMotionPlayer` adapter와 hardware executor node는 아직 구현하지 않았으며,
core library는 `robot_control` target에 링크하지 않는다. 이 core의 빌드와
단위 테스트 통과는 실물 동작 또는 안전성 검증을 의미하지 않는다.

## Simulated executor node

`sdk_motion_executor`는 현재 `SimulatedMotionBackend`만 사용하는 하드웨어 없는
ROS 2 node이다. 실제 SDK backend는 아직 연결되어 있지 않다.

- subscribe: `/motion/executor/request`
- subscribe: `/motion/executor/cancel`
- publish: `/motion/executor/status`
- parameters: `motion_aliases_file`, `poll_period_ms`(기본 20),
  `running_polls`(기본 2),
  `settling_polls`(기본 1), `force_start_failure`, `force_backend_failure`

```bash
ros2 launch irc_step_motion_executor sdk_executor_simulated.launch.py
```

request는 즉시 status를 만들고 timer poll은 simulated 상태를
`RUNNING → SETTLING → SUCCEEDED`로 진행한다. 외부 status에서 `SETTLING`은
message에 settling을 표시한 `RUNNING`으로 발행된다. cancel은 다음 poll에서
`CANCELLED`가 된다. 전이는 sleep이나 장치 시간이 아닌 poll 횟수만 사용한다.

이 launch에는 실물 로봇이나 serial 장치를 연결하지 않는다. simulated
topic test 통과 역시 실물 안전성 검증을 의미하지 않는다.

## RobotMotionPlayer backend adapter

`RobotMotionPlayerBackend`는
`IRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON`일 때만 빌드되는 SDK opt-in target이다.
실제 `irc_step::RobotMotionPlayer`를 생성하거나 소유하지 않고, 외부에서
주입된 non-owning API wrapper를 `MotionBackend` 상태로 변환한다.

기본 `sdk_motion_executor` node는 계속 `SimulatedMotionBackend`만 사용한다.
real backend 선택, production factory 및 hardware launch는 아직 없으며
`RobotMotionPlayerBackend`가 node에서 활성화되는 경로도 없다. 관절 방향,
영점, limit, 모션 거리 및 torque 안전 조건이 확인되기 전에는 실제 player를
생성·초기화하거나 이 adapter로 motion을 실행하지 않는다.

## 빌드 모드

기본 빌드는 SDK 경로 없이 catalog-only 모드로 동작한다.

```bash
colcon build --packages-select irc_step_motion_executor
```

SDK-enabled 빌드는 명시적으로 option을 켜는 경우에만 구성된다. SDK 소스의
라이선스, 복사 및 사용에 대한 조직 승인이 끝난 뒤에만 승인된 **외부 경로**를
지정해야 한다. 이 저장소에는 SDK를 복사하지 않으며 `vendor/robot_motion_sdk`
디렉터리나 tar.gz 해제 결과를 만들지 않는다.

```bash
colcon build --packages-select irc_step_motion_executor \
  --cmake-args \
  -DIRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON \
  -DROBOT_MOTION_SDK_DIR=/approved/external/sdk/path
```

`IRC_STEP_ENABLE_ROBOT_MOTION_SDK`의 기본값은 `OFF`,
`ROBOT_MOTION_SDK_DIR`의 기본값은 빈 문자열이다. option이 `OFF`이면 SDK를
탐색하거나 `add_subdirectory()` 하지 않고 어떤 SDK library에도 링크하지
않는다.

option이 `ON`이면 지정한 외부 디렉터리에 다음 항목이 모두 있어야 한다.

- `CMakeLists.txt`
- `robot_motion_player.hpp`
- `robot_motion_player.cpp`
- `add_subdirectory()` 이후 생성되는 `robot_control` CMake target

경로 또는 항목이 없으면 configure 단계에서 명확히 실패하며 다른 경로로
fallback하지 않는다. SDK source는 현재 package build tree 아래의 별도 binary
directory에서 `add_subdirectory()`되고 원본 source는 수정하지 않는다.

현재 sdk-enabled 결과물은 `robot_motion_player.hpp` include와
`robot_control` link 적합성만 확인하는 compile probe이다. 실제
`sdk_motion_executor_node`, `RobotMotionPlayer`, Dynamixel 객체 또는 hardware
호출은 포함하지 않는다. 따라서 sdk-enabled build 성공은 실제 로봇에서의
동작 가능성이나 안전성을 의미하지 않는다.

## Launch

```bash
ros2 launch irc_step_motion_executor catalog_only.launch.py
```

launch의 `hardware_enable` 기본값은 `false`, runtime SDK 경로 기본값은 빈
문자열이다. catalog-only node는 두 안전 조건을 강제하며 실제 장치, serial,
torque 또는 motor에 접근하지 않는다. 실물 motion 정보와 안전 조건이 확정되기
전에는 sdk-enabled build 결과를 hardware node로 확장하거나 실물에서 실행하지
않는다. 관절 방향·영점·limit·속도·전류/토크·비상정지 등 calibration 및 안전
정보가 확인되기 전에는 hardware node 사용을 금지한다.

기존 Python `motion_executor_node`, legacy adapter, SDK placeholder,
`full_system.launch.py`는 이 패키지와 별개이며 변경하거나 대체하지 않는다.
