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

## 빌드 모드

기본 빌드는 SDK 경로 없이 catalog-only 모드로 동작한다.

```bash
colcon build --packages-select irc_step_motion_executor
```

외부 SDK 포함을 위한 build guard만 준비되어 있다. SDK 소스의 라이선스 및
재배포 승인이 끝난 뒤에만 SDK를 `vendor/robot_motion_sdk` 같은 승인된 위치에
배치해야 한다. 이 저장소는 SDK 소스를 포함하지 않으며 tar.gz를 자동으로
해제하지 않는다.

```bash
colcon build --packages-select irc_step_motion_executor \
  --cmake-args \
  -DIRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON \
  -DROBOT_MOTION_SDK_DIR=/approved/external/sdk/path
```

`IRC_STEP_ENABLE_ROBOT_MOTION_SDK`의 기본값은 `OFF`,
`ROBOT_MOTION_SDK_DIR`의 기본값은 빈 문자열이다. option이 `OFF`이면 SDK를
`add_subdirectory()` 하지 않고 어떤 SDK library에도 링크하지 않는다.
option이 `ON`이면 유효한 SDK `CMakeLists.txt`가 반드시 필요하다. 이 모드에도
실제 SDK 호출 node/backend 연결은 아직 구현되어 있지 않다.

## Launch

```bash
ros2 launch irc_step_motion_executor catalog_only.launch.py
```

launch의 `hardware_enable` 기본값은 `false`, runtime SDK 경로 기본값은 빈
문자열이다. catalog-only node는 두 안전 조건을 강제하며 실제 장치, serial,
torque 또는 motor에 접근하지 않는다. 실물 motion 정보와 안전 조건이 확정되기
전에는 sdk-enabled 빌드 및 실물 실행을 사용하지 않는다.

기존 Python `motion_executor_node`, legacy adapter, SDK placeholder,
`full_system.launch.py`는 이 패키지와 별개이며 변경하거나 대체하지 않는다.
