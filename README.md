# STEP 2026 IRC Production 통합 시스템

이 저장소는 **2026 IRC Humanoid Robot Intelligent Competition**용 STEP 휴머노이드 로봇의 production 통합 코드다. Intel RealSense, YOLO26 객체 탐지, 미션별 통합 비전, line navigation·mission decision, STEP SDK motion executor를 ROS 2로 연결한다. 이 문서는 현재 워킹 트리의 실제 source/launch/config 기준이며 계획 기능은 별도로 표시한다.

## 전체 production 데이터 흐름

```text
Intel RealSense
  ├─ /camera/camera/color/image_raw
  ├─ /camera/camera/aligned_depth_to_color/image_raw
  └─ /camera/camera/color/camera_info
        ↓
yolo26_detector (YOLO26 TensorRT, best.engine)
        ↓ /vision/detections
unified_vision_node
  ├─ YoloLineAnalyzer ──> /vision/line_info
  ├─ BallAnalyzer     ──> /vision/ball_info
  ├─ GoalAnalyzer     ──> /vision/goal_info
  └─ HurdleAnalyzer   ──> /vision/hurdle_info
        ↓
MotionDecisionNode
        ↓ /navigation/motion_command
MotionCommandBridgeNode
        ↓ /motion/executor/request
sdk_motion_executor
        ↓ RobotMotionPlayerBackend
RobotMotionPlayer → DynamixelMotionHardware → Dynamixel

상태: /motion/executor/status → bridge → /motion/status → decision/lock
```

`step`은 detector·analyzer·planner, `mission_control`은 phase·최종 명령·bridge, `irc_step_motion_executor`는 catalog·SDK backend·실기 상태 관리를 담당한다.

## Production 실행

```bash
cd ~/IRC/IRC-STEP/IRC_vision_latest
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mission_control full_system_robot.launch.py
```

일상 production 실행은 긴 launch argument 없이 현재 기본값을 사용한다. 기본값은 RealSense, `best.engine`/TensorRT, display, `AUTO`, 실제 `robot_motion_player`, robot hardware·torque 승인, startup pose 활성화다. 즉 위 명령은 실제 토크와 관절 움직임을 발생시킬 수 있으므로 로봇 고정, 작업 반경 확보, 전원, `/dev/ttyUSB1`, motor ID와 아래 SDK JSON 경로를 먼저 확인해야 한다.

```text
/home/jet/IRC/external_sdk/robot_motion_player_sdk_work_20260801/final step/robot_motions.json
```

## RealSense production remap

RealSense는 `align_depth.enable=true`, gyro/accel 활성화로 launch된다. 현재 `full_system_robot.launch.py`의 실제 remap은 다음과 같다.

| 입력 | source(내부 기본명) | target(RealSense) | 현재 상태 |
|---|---|---|---|
| color image | `/camera/color/image_raw` | `/camera/camera/color/image_raw` | detector와 unified vision에 적용 |
| aligned depth | `/camera/aligned_depth_to_color/image_raw` | `/camera/camera/aligned_depth_to_color/image_raw` | unified vision에 적용 |
| color camera_info | `/camera/color/camera_info` | `/camera/camera/color/camera_info` | unified vision에 적용 |

Production launch에서는 color image, aligned depth, color camera_info의 세 RealSense remap을 모두 유지한다. goal/hurdle 등 depth 기반 analyzer가 camera intrinsics를 정상적으로 받을 수 있도록 camera_info remap도 production 계약으로 관리한다.

## YOLO26 / TensorRT

`step/yolo26_detector`는 `line`, `ball`, `goal`, `backboard`, `hurdle`을 한 번에 탐지해 `/vision/detections`로 발행한다. production launch는 설치된 `share/step/models/best.engine`과 `device=tensorrt`를 사용한다.

`tensorrt_backend.py`는 TensorRT Python binding과 CUDA Runtime을 직접 사용하며 다음 계약을 강제한다.

- 정적 shape, input 1개/output 1개인 engine만 지원한다.
- engine의 input/output dtype·shape로 재사용 GPU buffer를 할당한다.
- detector 입력은 NCHW이며 engine shape와 다르면 거부한다.
- H2D → `execute_async_v3` → D2H → synchronize 후 기존 YOLO 후처리로 보낸다.

`.onnx`를 지정하면 실제 ONNX Runtime 경로가 사용되며 `auto/cpu/cuda/tensorrt` provider 선택이 있다. `.engine`에서 `device=cpu`는 CPU fallback이 아니라 경고 후 GPU TensorRT를 사용한다.

현재 `src/step/models/best.engine`은 워킹 트리에 있으나 **Git untracked**다. clone에 포함된다고 가정하지 말고 대상 Jetson의 TensorRT/CUDA와 호환되는 engine을 별도 준비하고 install 결과를 확인해야 한다.

## Unified vision

`unified_vision_node`는 네 analyzer를 `MultiThreadedExecutor` 한 프로세스에서 실행한다. 별도 analyzer를 동시에 띄우면 동일 topic이 중복 발행될 수 있다.

| analyzer | 출력 | 내용 |
|---|---|---|
| `YoloLineAnalyzer` | `/vision/line_info` | path, heading error, lateral offset, turn, quality |
| `BallAnalyzer` | `/vision/ball_info` | 위치·depth·정렬·pickup 조건 |
| `GoalAnalyzer` | `/vision/goal_info` | goal/backboard 위치·depth·score 조건 |
| `HurdleAnalyzer` | `/vision/hurdle_info` | 거리·각도·정렬·GO 확인 |

decision node는 `/vision/finish_info`도 구독하지만 unified vision에는 finish analyzer/publisher가 없다.

## Line navigation algorithm

`line_navigation_planner.py`는 filtered `heading_error_deg`와 `lateral_offset_norm`을 우선하고 없으면 raw 값을 사용한다. heading/geometry/detection quality의 최솟값이 `0.35` 미만이면 lost로 처리한다.

```text
steering_error = heading_error + 24° × lateral_offset
               + 0.15 × reliable preview turn
```

preview는 `|turn_angle_deg| >= 8°`, consistency `>= 0.55`일 때만 반영한다. 각속도 `0.60 rad/s`, 각가속도 `1.20 rad/s²`로 제한하고 오차와 quality에 따라 전진 속도를 낮춘다.

- `STRAIGHT`: `|offset| <= 0.20`, `|heading| <= 7°`, `|steering| <= 7°`.
- `FINE_LEFT/FINE_RIGHT`: 중간 offset/heading/steering 보정 후보. 이전 FINE 상태에서는 진입 `7°`, 이탈 `4°` hysteresis를 쓴다.
- `LEFT/RIGHT`: offset 진입 임계 `0.28` 초과 또는 heading `18°` 이상인 큰 보정. offset recovery는 `0.20` tolerance까지 돌아와야 풀린다.

현재 `fine_turn_supported=false`다. FINE 후보가 나와도 실제 planner 출력은 `STRAIGHT`, 각속도 0, reason `fine_turn_unavailable_straight_fallback`이며 bridge mapping도 없다. 2/4/6/8/10회 세분화는 구현되지 않았다.

Line lost 시 마지막 valid offset/heading을 기억한다. 첫 lost frame은 `STOP`, 기본 lost-frame threshold 2부터 offset 우선·heading 보조로 `LEFT/RIGHT` recovery를 한다. 방향 이력이 없으면 `line_lost_without_history`, 기본 최대 3회 recovery 후에는 `line_recovery_attempts_exhausted`로 `STOP`한다. line이 중심/heading tolerance로 복귀하면 attempt를 초기화한다.

## Mission decision / phase

`MotionDecisionNode`는 line/ball/goal/hurdle/finish 정보의 freshness를 검사하고 phase에 맞는 planner 하나만 실행한다.

| phase | 현재 흐름 |
|---|---|
| `AUTO` | priority는 현재 line 하나다. |
| `BALL_SEARCH` | line 이동 중 fresh 공이 control range에 오면 `BALL_APPROACH`. |
| `BALL_APPROACH` | 접근·정렬 후 `PICKUP_NOW`; 성공 시 `GOAL_APPROACH`, 실패 시 복귀. |
| `GOAL_SEARCH` | line 이동 중 fresh goal이 control range에 오면 `GOAL_APPROACH`. |
| `GOAL_APPROACH` | 접근·정렬 후 `SHOT`; 성공 시 횟수/ball section 갱신. |
| `HURDLE_APPROACH` | 접근·정렬·확인 후 `GO`; 성공 시 진입 전 phase 복귀. |
| `LINE_TRACK` | line planner 고정. |
| `FINISH/WALK_TO_FINISH` | finish 활성·fresh·confidence 조건 후 `CROSS_FINISH`, 성공 시 `FINISHED`. 현재 vision publisher/motion mapping 없음. |
| `FINISHED` | 실행 명령을 더 내지 않는다. |

`PICKUP_NOW`, `SHOT`, `GO`, `CROSS_FINISH`는 special motion이다. 활성 command/event와 일치하는 terminal status까지 special lock을 유지한다. 나머지 general motion도 별도 gate로 한 번에 하나만 실행한다.

## SDK motion pipeline과 correlation

```text
MotionDecisionNode → MotionCommandBridgeNode → sdk_motion_executor
→ RobotMotionPlayerBackend → RobotMotionPlayer → Dynamixel
```

- `command_id`: decision이 발행한 명령 ID. gate와 bridge 중복 방지에 쓴다.
- `request_id`: executor 요청 ID. bridge는 현재 `request_id = command_id`로 만든다.
- `event_id`: special mission event ID이며 general command는 `null`일 수 있다.

executor는 세 ID를 status에 보존한다. bridge는 request ID가 맞는 상태만 전달하고, decision은 현재 command/event와 맞는 상태만 lock 해제와 phase 갱신에 사용한다.

## Startup pose

production 기본은 motion JSON의 `오뒤307`, `1800 ms`다. catalog에서 23개 관절 각도를 읽고 SDK `startPoseTransition(angles, duration_ms)`를 호출한다. RobotMotionPlayer가 현재 Present Position을 시작 pose로 capture한 뒤 `오뒤307`으로 직접 smooth transition하며 중간 motion alias를 재생하지 않는다.

`updateStartupPose()`의 `Running → Settling → Succeeded` 동안 Present Position 기반 도달 확인이 끝날 때까지 navigation gate가 잠긴다. 성공 후 `[STARTUP POSE] AUTO gate released`가 기록된다. capture/Present Position/transition 실패 시 gate는 error 상태다.

```text
startup 중 너무 이른 request
 → REJECTED / STARTUP_POSE_GATE_LOCKED
 → 원 command_id·event_id·request_id 보존
 → bridge terminal 전달 및 active lock 해제
 → decision general gate 해제, fresh Vision 요구
 → startup 완료 + 새 Vision
 → 새 command_id로 재판단·재시도
```

이 코드는 startup-pose race에서 bridge가 영구 BUSY가 되지 않게 한다. 해당 코드는 transient rejection이며 기본 retry limit은 2다.

## 현재 production motion mapping

`motion_command_bridge_node.py`와 `motion_aliases.yaml`을 결합한 실제 mapping만 표시한다.

| action | bridge motion_id | SDK JSON motion name | 상태 |
|---|---|---|---|
| `STRAIGHT`, `APPROACH` | `forward` | `전진 가장 일직선` | 지원 |
| `LEFT` | `line_turn_left_large` | `좌회전실전(9회)` | 지원 |
| `RIGHT` | `line_turn_right_large` | `우회전 실전(15회)` | 지원 |
| `PICKUP_NOW` | `pickup` | `공잡기리그랩까지 실전` | 지원 |
| `GO` | `hurdle` | `허들넘기 실전` | 지원 |
| `FINE_LEFT/FINE_RIGHT` | 없음 | 없음 | 미지원·미매핑 |
| `TURN_LEFT/TURN_RIGHT` | 없음 | 없음 | 미지원·미매핑 |
| `ALIGN_LEFT/ALIGN_RIGHT` | 없음 | 없음 | 미지원·미매핑 |
| `SHOT`, `CROSS_FINISH` | 없음 | 없음 | 미지원·미매핑 |
| `SLOW_APPROACH`, `FINE_FORWARD_STEP`, `APPROACH_GOAL`, `APPROACH_HURDLE`, `RETREAT_GOAL`, `STOP` | 없음 | 없음 | gate action이나 bridge 미매핑 |

config에는 bridge가 선택하지 않는 `sdk_turn_in_place_*`, `sdk_turn_right_*`, 기본자세 전환 alias도 있다. alias 존재만으로 navigation action이 지원되는 것은 아니다.

## Safety / lock

- General gate는 발행 즉시 잠기며 일치하는 `RUNNING` 후 terminal status로 정상 해제된다.
- Special lock은 pickup/shot/hurdle/finish 완료까지 새 실행을 억제한다.
- bridge는 같은 `command_id`를 `DUPLICATE_COMMAND_ID`, active 중 새 요청을 `BUSY`로 거부한다.
- RobotMotionPlayer busy는 `SDK_BUSY`, startup 중 요청은 `STARTUP_POSE_GATE_LOCKED`다.
- 위 busy 계열은 bounded transient retry 대상이며 영구 rejection/실패는 같은 action 재발행을 막는다.
- terminal 후 Vision generation이 하나 증가해야 다음 명령이 가능하다(fresh Vision requirement).
- `LEFT`, `RIGHT`, `PICKUP_NOW`, `GO`는 source/action이 기본 `0.5 s` 유지되어야 한다(pre-motion settle).
- stale 또는 action/command/event/request correlation이 다른 status는 현재 lock을 풀지 않는다.
- executor heartbeat startup grace/timeout과 safety interlock이 있다.

실기 motion 변경은 simulation/fake backend 후 스탠드 고정 상태에서 검증해야 한다. turn 횟수, pose, torque, motor ID 변경은 낙상·관절 충돌·과토크 위험이 있다.

## 테스트

이번 README 작업에서는 테스트 전체를 실행하지 않았으므로 PASS를 주장하지 않는다. 현재 테스트 범주는 다음과 같다.

- `src/step/test`: line/ball/goal/hurdle planner, temporal confirmation, TensorRT contract, lint
- `src/mission_control/test`: phase/flow, decision, general gate, bridge correlation/mapping, heartbeat/interlock, launch defaults, mock/legacy, production catalog contract
- `src/irc_step_motion_executor/test`: request/timeout/cancel/status, alias/catalog, backend/factory, hardware preflight, startup pose, fake SDK CMake 및 launch

실기 전 관련 단위·launch contract와 fake SDK/backend 테스트를 먼저 수행한다.

## Known limitations / TODO

- FINE 작은 좌/우 회전 SDK motion과 bridge mapping은 미구현이다.
- 2/4/6/8/10회 회전 세분화는 미구현이다. config에서 작은 좌회전 SDK JSON motion도 확인되지 않는다.
- 제자리 `TURN_*`, mission `ALIGN_*`은 production motion에 연결되지 않았다.
- `SHOT`, `CROSS_FINISH`는 미지원이다.
- finish decision 경로는 있으나 `/vision/finish_info` publisher가 없다.
- `best.engine`은 현재 Git tracked asset이 아니다.

## 개발 원칙

- 실제 SDK motion이 없는 action을 임의 motion으로 대체하지 않는다.
- line walking turn과 mission in-place turn/alignment를 구분한다.
- production launch defaults는 운용 계약으로 유지한다.
- RealSense color/depth/camera_info remap을 유지하고 launch test로 검증한다.
- simulation, fake SDK, generated engine/build/install/log artifact를 production source와 구분한다.
- motion·torque·startup 변경은 simulation/fake backend에서 먼저 검증한다.
