# mission_control

`mission_control`은 비전 결과를 받아 현재 수행할 행동 하나를 선택하는
ROS 2 패키지입니다. 카메라 입력, YOLO 추론, 객체별 거리·각도 계산은
`step` 패키지가 담당합니다.

## 역할 경계

```text
step (vision)
  /vision/line_info
  /vision/ball_info
  /vision/goal_info
  /vision/hurdle_info
          |
          v
mission_control (decision)
  /mission/phase 입력
  motion_decision_node
          |
          v
  /navigation/motion_command
          |
          v
  motion_command_bridge_node
          |
          v
  /motion/executor/request
          |
          v
  irc_step_motion_executor/sdk_motion_executor
          |
          v
  /motion/executor/status -> /motion/status
```

- `step`: 보이는 것을 측정하고 객체별 행동 후보를 계산합니다.
- `mission_control`: 후보 간 우선순위, 미션 단계, 단발 모션 요청을
  관리합니다.
- `motion_command_bridge_node`: 현재 catalog에 있는 `forward`와
  `forward_short`만 C++ executor 요청으로 변환합니다. 나머지 action은
  `/motion/status`에 `UNSUPPORTED`를 발행하며 executor로 보내지 않습니다.
- C++ executor: `full_system.launch.py`에서는 `backend_type=simulated`,
  `enable_robot_hardware=false`로만 실행됩니다. SDK, 포트, Dynamixel 및 torque를
  사용하지 않습니다.

`mission_control`은 `step`의 line/ball/goal/hurdle planner 클래스를
재사용하므로 두 패키지를 함께 빌드해야 합니다.

## 빌드와 실행

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
colcon build --packages-select step mission_control --symlink-install
source install/setup.bash
ros2 run mission_control motion_decision_node
```

미션 단계 예시:

```bash
ros2 topic pub --once /mission/phase std_msgs/msg/String "{data: 'AUTO'}"
```

상세 설계와 아직 구현되지 않은 SDK 연동 항목은
[`docs/MOTION_DECISION_SPEC.md`](docs/MOTION_DECISION_SPEC.md)를
참고합니다.
