# Motion Interface 초안

## 목적

Vision / Mission에서 판단한 행동을 ROS2 Motion Executor가 안정적으로 실행하고,
SDK의 실행 상태와 오류를 상위 ROS Action 결과로 변환하기 위한 인터페이스를
정의한다.

이 문서는 기존 `/navigation/motion_command` 호환 경로와 향후 ROS Action 경로를
함께 설명한다. 실제 모션 데이터와 실물 로봇 검증 결과가 확정될 때까지는 초안으로
관리한다.

## 전체 구조

```text
Vision / Mission
        ↓
ROS2 Motion Executor
        ↓
RobotMotionPlayer
        ↓
IMotionHardware
        ↓
Dynamixel
```

- Vision / Mission은 상황을 판단하고 실행할 `motion_id`를 선택한다.
- ROS2 Motion Executor는 상위 요청, timeout, 재시도와 Action 결과 변환을 담당한다.
- RobotMotionPlayer는 JSON 모션을 읽고 프레임 단위로 재생한다.
- IMotionHardware는 모션 재생기와 실제 하드웨어 구현 사이의 추상화 계층이다.
- Dynamixel 구현은 실제 모터 통신과 하드웨어 오류를 처리한다.

## 역할 분담

### 알고리즘 담당

- ROS2 Motion Executor
- 미션 판단
- 명령 매핑
- 상위 timeout과 재시도
- SDK 상태를 ROS Action 결과로 변환
- SDK mock 통합 테스트

알고리즘 계층은 SDK 내부의 프레임 전환이나 복합 모션 중간 단계를 상위
Vision / Mission에 노출하지 않는다. 완료 결과에는 최초 요청의 `motion_id`를
유지한다.

### SDK 담당

- RobotMotionPlayer
- JSON 파싱과 모션 재생
- Dynamixel 연결 및 오류 처리
- 프레임 시간과 프로파일
- cancel 안전정지
- 최종 자세 도달 판정
- MockMotionHardware

## 기존 호환 구조

현재 `/navigation/motion_command` JSON과 `motion_command_bridge_node`는 새 ROS
Action 경로가 검증될 때까지 유지한다. 새 경로가 준비되었다는 이유만으로 기존
bridge를 즉시 제거하지 않는다.

현재 호환 요청 형식:

```json
{
  "action": "STRAIGHT",
  "angle_deg": 0.0
}
```

향후 권장 요청 형식:

```json
{
  "request_id": 1,
  "action": "WALK_FORWARD",
  "parameters": {
    "angle_deg": 0.0
  }
}
```

향후 권장 결과 형식:

```json
{
  "request_id": 1,
  "action": "WALK_FORWARD",
  "status": "SUCCEEDED",
  "error_code": "",
  "message": ""
}
```

## 지원 motion_id

- `home`
- `forward`
- `forward_short`
- `turn_left`
- `turn_right`
- `adjust_left`
- `adjust_right`
- `backward`
- `pick_ball`
- `shoot`
- `hurdle`
- `recover`
- `head_left`
- `head_right`
- `head_center`

지원 목록에 없는 `motion_id`는 RobotMotionPlayer를 호출하지 않고 거부한다.

## MotionExecutorCore

MotionExecutorCore는 ROS에 의존하지 않는 순수 상태 머신이다. mock 가능한
RobotMotionPlayer 객체를 생성자에 주입하며, ROS2 Motion Executor가 이 core를
감싸서 향후 ROS Action 요청과 결과를 연결한다.

### ExecutorState

- `IDLE`
- `STARTING`
- `RUNNING`
- `SETTLING`
- `SUCCEEDED`
- `FAILED`
- `CANCELLED`
- `TIMEOUT`

`STARTING`, `RUNNING`, `SETTLING`은 실행 중인 상태이며 `busy()`가 `true`이다.
`SUCCEEDED`, `FAILED`, `CANCELLED`, `TIMEOUT`은 terminal 상태이다.

### MotionExecutionResult

결과는 다음 필드를 가진다.

- `motion_id`: 최초 요청한 motion ID
- `final_status`: terminal `ExecutorState`
- `success`: 정상 완료 여부
- `error_code`: 문자열 오류 코드
- `message`: 진단용 설명

`MotionExecutionResult.error_code`는 항상 문자열이어야 한다. SDK enum 객체를
그대로 저장하지 않고 반드시 `enum.name`을 사용한다.

예:

```text
MotionError.COMMUNICATION_ERROR
→ "COMMUNICATION_ERROR"
```

### lifecycle

1. `start_motion(motion_id, timeout_ms)`를 호출한다.
2. 요청이 수락되고 `busy()`가 `true`인 동안 주기적으로 `tick(elapsed_ms)`를
   호출한다.
3. `terminal_result()`로 완료 결과를 확인한다.
4. 상위 계층이 결과를 처리한 뒤 `reset()`을 호출한다.
5. `reset()` 이후 새로운 모션을 받을 수 있다.

실행 중에는 `reset()`을 허용하지 않는다. 실행 중 새 `start_motion()` 요청도
거부하며, 현재 active motion과 기존 결과를 덮어쓰지 않는다.

`timeout_ms`가 0 이하인 요청, 지원하지 않는 `motion_id`, 준비되지 않은
하드웨어는 SDK 실행 전에 실패시킨다.

## SDK 계약

RobotMotionPlayer는 다음 메서드를 제공한다.

- `start(motion_id)`
- `update()`
- `running()`
- `status()`
- `succeeded()`
- `result()`
- `lastError()`
- `cancel()`
- `hardwareReady()`
- `currentMotion()`

### SDK 상태 매핑

| MotionStatus | ExecutorState |
| --- | --- |
| `RUNNING` | `RUNNING` |
| `SETTLING` | `SETTLING` |
| `SUCCEEDED` | `SUCCEEDED` |
| `FAILED` | `FAILED` |
| `CANCELLED` | `CANCELLED` |

`FAILED`에서는 `player.result()`의 `MotionError` enum 이름을
`error_code` 문자열로 저장한다. `player.lastError()`는 사람이 읽는 상세
오류 문자열이므로 `message`에 포함한다.

예:

```text
player.result() == MotionError.COMMUNICATION_ERROR
→ error_code = "COMMUNICATION_ERROR"

player.lastError() == "motor communication failed"
→ message에 포함
```

### StartResult 매핑

| StartResult | 처리 |
| --- | --- |
| `ACCEPTED` | active motion과 timeout을 저장하고 `STARTING`으로 전환 |
| `REJECTED_BUSY` | 요청 거부, 현재 실행 상태 유지 |
| `MOTION_NOT_FOUND` | `FAILED`, 모션 데이터 없음 |
| `HARDWARE_NOT_READY` | `FAILED`, 하드웨어 준비 오류 |
| `INVALID_MOTION` | `FAILED`, 유효하지 않은 모션 |

StartResult가 실패하면 해당 StartResult의 `enum.name`을 `error_code`
문자열로 사용한다. `player.lastError()`가 비어 있지 않으면 그 내용을
`message`에 포함한다.

## timeout 및 cancel 규칙

- 상위 timeout에 도달하면 `player.cancel()`을 정확히 한 번 호출한다.
- timeout 처리 이후에는 `player.update()`를 호출하지 않는다.
- 상위 timeout의 terminal 결과는 `TIMEOUT`이다.
- timeout `error_code`는 `"POSITION_TIMEOUT"`을 사용한다.
- cancel 결과가 `CANCELLED`이면 `CANCELLED` terminal 결과를 생성한다.
- `HOLD_FAILED`는 `FAILED` / `"CANCEL_FAILED"`로 변환한다.
- `HARDWARE_NOT_READY`는 `FAILED` / `"HARDWARE_NOT_READY"`로 변환한다.
- `NOT_RUNNING`은 안전하게 아무 동작도 하지 않고 기존 terminal result를
  덮어쓰지 않는다.

## terminal 결과 규칙

terminal 결과는 정확히 한 번만 생성한다. 한 번 생성된
`MotionExecutionResult`는 이후 `tick()`, `cancel()` 또는 중복 완료 신호로
변경하거나 교체하지 않는다.

상위 계층이 결과를 소비하기 전까지 `terminal_result()`는 같은 결과를 반환한다.
결과 소비가 끝난 뒤에만 `reset()`하여 `IDLE`로 돌아간다.

## Executor 공통 규칙

1. `IDLE`에서 유효한 명령을 받으면 실행을 시작한다.
2. 실행이 시작되면 `RUNNING` 상태를 상위 계층에 전달한다.
3. 실행 중 일반 명령은 중복 실행하지 않는다.
4. 정상 완료 시 `SUCCEEDED`를 한 번만 전달한다.
5. SDK 시작 실패나 통신 오류 시 `FAILED`를 전달한다.
6. 제한 시간을 넘으면 안전 정지를 요청하고 `TIMEOUT`을 전달한다.
7. 안전정지 요청은 일반 명령보다 우선한다.
8. 상태와 결과에는 최초 요청의 `motion_id`를 유지한다.
9. SDK 내부 모션 이름이나 중간 단계를 알고리즘 계층에 노출하지 않는다.

## 현재 mock 기반 MotionExecutorNode

현재 `motion_executor_node`는 실제 RobotMotionPlayer나 Dynamixel을 연결하지
않는다. 프로세스 내부에서 `MockRobotMotionPlayer`를 생성하여
MotionExecutorCore에 주입하는 검증용 ROS2 노드 뼈대이다.

기본 timer 주기는 10ms이며 ROS parameter `tick_period_ms`로 변경할 수 있다.
0 이하의 값이 설정되면 안전한 기본값 10ms를 사용한다.

### 요청 topic

- topic: `/motion/executor/request`
- type: `std_msgs/msg/String`

```json
{
  "request_id": 1,
  "motion_id": "forward",
  "timeout_ms": 5000
}
```

잘못된 JSON, 필수 필드 누락과 유효하지 않은 필드 타입은
`REJECTED` / `"INVALID_REQUEST"`로 응답한다. 지원하지 않는 `motion_id`는
`REJECTED` / `"INVALID_MOTION"`으로 응답한다. 실행 중 새 요청은
`REJECTED` / `"REJECTED_BUSY"`로 응답한다.

### 상태 topic

- topic: `/motion/executor/status`
- type: `std_msgs/msg/String`

```json
{
  "request_id": 1,
  "motion_id": "forward",
  "status": "RUNNING",
  "error_code": "",
  "message": ""
}
```

수락 직후 `RUNNING`을 한 번 발행한다. 이후 terminal 결과
`SUCCEEDED`, `FAILED`, `CANCELLED`, `TIMEOUT` 중 하나를 정확히 한 번
발행하고 core를 `reset()`한다. terminal 결과까지 최초 `request_id`와
`motion_id`를 유지한다.

### cancel topic

- topic: `/motion/executor/cancel`
- type: `std_msgs/msg/String`

```json
{
  "request_id": 10
}
```

cancel의 `request_id`가 현재 실행 중인 최초 요청과 일치할 때만
`MotionExecutorCore.cancel()`을 호출한다. 불일치하면
`REJECTED` / `"REQUEST_ID_MISMATCH"`, 실행 중인 모션이 없으면
`REJECTED` / `"NOT_RUNNING"`을 발행한다. 성공하면 최초 `request_id`와
`motion_id`를 유지한 `CANCELLED`를 정확히 한 번 발행한다.

이 cancel topic은 mock 기반 노드의 개발/검증용 인터페이스이다. 실제 SDK
연동 시에는 ROS Action cancel 처리와 SDK 안전정지 계약에 맞게 교체될 수 있다.

### mock 실패 주입 parameter

- `mock_fail_after_updates` (기본값 `-1`)
  - `-1`: 실패를 주입하지 않는다.
  - `0` 이상: 설정한 update 횟수에 도달하면 mock을 `FAILED`로 전환한다.
- `mock_failure_code` (기본값 `"COMMUNICATION_ERROR"`)
  - `MotionError`의 enum 이름 문자열을 사용한다.

실패가 주입되면 `player.result()`는 지정된 `MotionError` enum을 반환하고,
`player.lastError()`는 사람이 읽을 수 있는 상세 문자열을 반환한다. Executor는
이를 `FAILED`, enum 이름 문자열 `error_code`, 상세 오류가 포함된 `message`로
변환한다. 한 번 `FAILED`가 된 mock은 이후 `SUCCEEDED`로 바뀌지 않는다.

이 parameter들은 실제 고장을 재현하지 않는 개발/검증 전용 기능이다. 실제
RobotMotionPlayer 연동 시 제거되거나 실제 SDK의 오류 주입 방식으로 교체될 수
있으며 Dynamixel 또는 실제 하드웨어에는 접근하지 않는다.

## Legacy Motion Executor Adapter

`legacy_motion_executor_adapter`는 현재 `/navigation/motion_command` JSON을
구독하여 mock 기반 `/motion/executor/request` 형식으로 변환하는 병렬 검증
경로이다.

```text
/navigation/motion_command
        ├─ motion_command_bridge_node → 기존 경로
        └─ legacy_motion_executor_adapter → mock Motion Executor 검증 경로
```

기존 `motion_command_bridge_node`는 삭제하거나 대체하지 않는다. 새 adapter는
mock Executor 통합이 검증되는 동안에만 병렬로 사용하며 실제 SDK 또는
Dynamixel에 접근하지 않는다.

### topic

- 구독: `/navigation/motion_command` (`std_msgs/msg/String`)
- 발행: `/motion/executor/request` (`std_msgs/msg/String`)

입력 예:

```json
{
  "action": "STRAIGHT",
  "angle_deg": 0.0
}
```

출력 예:

```json
{
  "request_id": 1,
  "motion_id": "forward",
  "timeout_ms": 5000
}
```

`request_id`는 adapter 내부에서 1부터 증가한다. `angle_deg`는 호환성을 위해
파싱하지만 현재 Executor 요청에는 포함하지 않는다. 잘못된 JSON, 유효하지 않은
`action`, 지원하지 않는 `action`은 경고만 남기고 발행하지 않는다.

### action 매핑

| legacy action | motion_id |
| --- | --- |
| `STRAIGHT`, `APPROACH`, `GO` | `forward` |
| `SLOW_APPROACH`, `FINE_FORWARD_STEP` | `forward_short` |
| `APPROACH_GOAL`, `APPROACH_HURDLE` | `forward_short` |
| `ALIGN_LEFT` | `adjust_left` |
| `ALIGN_RIGHT` | `adjust_right` |
| `RECOVER_LEFT`, `RECOVER_RIGHT` | `recover` |
| `RETREAT_GOAL` | `backward` |
| `PICKUP_NOW` | `pick_ball` |
| `SHOT` | `shoot` |
| `TURN_LEFT` | `turn_left` |
| `TURN_RIGHT` | `turn_right` |
| `CROSS_FINISH` | `hurdle` |

### timeout

- 일반 이동: 5000ms
- `pick_ball`: 10000ms
- `shoot`: 10000ms
- `hurdle`: 12000ms
- `recover`: 8000ms

## mock 통합 검증 launch

`motion_executor_mock.launch.py`는 mock 기반 Motion Executor 통합 흐름만
검증하기 위한 launch 파일이다. 다음 두 노드만 실행 대상으로 포함한다.

- `mission_control / motion_executor_node`
- `mission_control / legacy_motion_executor_adapter`

이 launch는 `MockRobotMotionPlayer`만 사용한다. 실제 RobotMotionPlayer를
실행하지 않고 Dynamixel에 접근하지 않으며 실물 로봇을 움직이지 않는다.
실제 로봇 운용용 launch로 사용하면 안 된다.

기본 실행:

```bash
ros2 launch mission_control motion_executor_mock.launch.py
```

정상 검증용 예:

```bash
ros2 launch mission_control motion_executor_mock.launch.py \
  tick_period_ms:=500
```

실패 주입용 예:

```bash
ros2 launch mission_control motion_executor_mock.launch.py \
  tick_period_ms:=500 \
  mock_fail_after_updates:=2 \
  mock_failure_code:=COMMUNICATION_ERROR
```

launch argument 기본값:

- `player_backend`: `mock`
- `tick_period_ms`: `100`
- `mock_fail_after_updates`: `-1`
- `mock_failure_code`: `COMMUNICATION_ERROR`

### 기존 mission status 호환 흐름

mock 통합 검증에서는 다음 흐름으로 새 Executor를 기존 mission node에
연결한다.

```text
motion_decision_node
→ /navigation/motion_command
→ legacy_motion_executor_adapter
→ /motion/executor/request
→ motion_executor_node
→ /motion/executor/status
→ legacy_motion_status_adapter
→ /motion/status
→ motion_decision_node
```

`legacy_motion_status_adapter`는 Executor의 `RUNNING`, `SUCCEEDED`,
`FAILED`, `CANCELLED`, `TIMEOUT`, `REJECTED` 값을 그대로 보존하면서 기존
subscriber가 사용하는 `status`, `action`, `command_id`, `event_id`,
`dynamics_command`, `motion_in_progress`, `reason` 필드를 만든다.
Executor의 `request_id`, `motion_id`, `error_code`, `message`도 추가 필드로
보존한다. 잘못된 JSON, 누락되거나 알 수 없는 status는 경고 후 발행하지
않는다.

이 adapter는 mock 검증을 위한 임시 호환 계층이다. 향후 mission node가
`/motion/executor/status`를 직접 사용하면 제거할 수 있다.

## Motion Player backend 선택

MotionExecutorNode는 `player_backend` parameter를 factory에 전달하여 player를
생성한다. 현재 지원하는 backend는 다음 두 가지이다.

- `mock`: 기본값. `MockRobotMotionPlayer`를 사용하며 기존 정상 흐름과 실패 주입
  검증을 지원한다.
- `sdk`: 실제 SDK가 아닌 `SdkMotionPlayerPlaceholder`를 사용한다.

`sdk` placeholder는 향후 연결 지점을 준비하기 위한 안전 객체이며 실제 모션을
실행하지 않는다. SDK, serial 또는 Dynamixel 모듈을 import하지 않고
`hardwareReady()`가 항상 `False`이다. 따라서 `player_backend:=sdk`로 실행해도
모션 시작 요청은 `HARDWARE_NOT_READY`로 거부되며 실물 로봇은 움직이지 않는다.

실제 SDK 연동 단계에서는 MotionExecutorCore나 ROS topic을 변경하지 않고
factory의 `sdk` 생성 구현만 실제 RobotMotionPlayer adapter로 교체할 예정이다.
그 전까지 기본 backend는 반드시 `mock`으로 유지한다.

## 아직 미확정인 항목

- 실제 전체 모션 JSON 데이터
- 모션별 실행시간
- 실제 시작/종료 자세
- 실물 로봇 검증 결과
