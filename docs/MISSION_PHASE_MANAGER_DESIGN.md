# Mission Phase Manager 조사 및 설계

## 0. 문서 목적과 조사 기준

이 문서는 Mission Phase Manager를 구현하기 전에 현재 저장소의 phase,
Vision 관측, action 선택, motion status 처리 책임을 실제 코드 기준으로
정리한 설계 자료이다. 이번 조사에서는 실행 코드와 테스트를 변경하지 않았고
ROS 2 launch 및 실제 장치 관련 프로그램을 실행하지 않았다.

주요 조사 대상은 다음과 같다.

- `src/mission_control/mission_control/motion_decision_node.py`
- `src/mission_control/mission_control/motion_decision_planner.py`
- `src/mission_control/mission_control/motion_command_gate.py`
- `src/mission_control/mission_control/legacy_motion_status_adapter.py`
- `src/step/step/mission_state_estimator.py`
- `src/step/step/unified_vision_node.py`
- line, ball, goal, hurdle analyzer와 navigation planner/controller
- 관련 단위 테스트 및 `docs/MOTION_INTERFACE.md`

이 문서에서 “현재 미구현”은 저장소에 해당 전환 조건 또는 실행 코드가
없다는 뜻이다. 확인되지 않은 경기 규칙이나 영상 조건은 `TBD`로 표시한다.

## 1. 현재 phase 관련 구성요소

### 1.1 토픽과 상태 소유권

| 항목 | 현재 상태 | 실제 위치 |
| --- | --- | --- |
| `/mission/phase` publisher | **실행 노드 publisher 없음**. README의 수동 `ros2 topic pub` 예시만 존재 | `README.md`, `src/mission_control/README.md` |
| `/mission/phase` subscriber | 존재. plain string 또는 `{"phase": "..."}` JSON을 받아 대문자로 저장 | `MotionDecisionNode.__init__`, `_phase_callback()` |
| `/vision/mission_state` publisher | 존재 | `MissionStateEstimator.publisher`, `_publish_state()` |
| `/vision/mission_state` algorithm subscriber | **없음** | mission_control 전체에서 subscriber 없음 |
| `/vision/mission_state` subscriber | 시각화용 subscriber만 존재 | `MissionMapVisualizer` |
| 내부 최종 phase | `MotionDecisionNode.mission_phase` | `MotionDecisionNode.__init__` |
| phase를 읽는 planner | `MotionDecisionPlanner.plan()`과 `source_for_phase()` | `motion_decision_planner.py` |
| motion status subscriber | `/motion/status` | `MotionDecisionNode._motion_status_callback()` |

`full_system.launch.py`는 `initial_mission_phase` 기본값 `AUTO`를
`motion_decision_node`에 넘기지만 `/mission/phase` publisher나
`mission_state_estimator`는 실행하지 않는다.

### 1.2 phase를 직접 변경하는 현재 코드

`motion_decision_node` 안에 두 변경 경로가 동시에 존재한다.

1. `MotionDecisionNode._phase_callback()`
   - 외부 `/mission/phase` 문자열 또는 JSON의 `phase`를 읽는다.
   - 비어 있지 않으면 `self.mission_phase = phase.upper()`로 즉시 덮어쓴다.
   - 허용 phase 목록 검증은 없다.

2. `MotionDecisionNode._motion_status_callback()`
   - 특수 action의 `RUNNING`을 받으면 특수 모션 잠금을 설정한다.
   - 같은 특수 모션의 `SUCCEEDED`, `FAILED`, `TIMEOUT`을 받으면
     `_update_action_progress()`를 호출하고 다음 phase를 계산한다.
   - 계산 후 `self.mission_phase = next_phase`로 직접 변경한다.

planner는 전달받은 phase에 따라 source와 action을 선택할 뿐,
`mission_phase`를 변경하거나 다음 phase를 반환하지 않는다.

### 1.3 진행도 변경 위치

모든 진행도는 현재 `motion_decision_node`가 소유한다.

| 상태 | 초기화 | 변경 위치와 조건 |
| --- | --- | --- |
| `pickups_completed` | 0 | `_update_action_progress()`: `PICKUP_NOW + SUCCEEDED` |
| `shots_completed` | 0 | `_update_action_progress()`: `SHOT + SUCCEEDED` |
| `ball_sections_processed` | 0 | 실패/timeout `PICKUP_NOW`, 또는 성공·실패·timeout 모든 `SHOT` |
| `finish_enabled` | `ball_sections_processed >= required_ball_sections` | `PICKUP_NOW` 또는 `SHOT` 처리 후 같은 식으로 재계산 |
| `mission_complete` | `False` | `CROSS_FINISH + SUCCEEDED`에서 `True`; 실패/timeout에서 `False` |

카운터는 각각 설정된 최대값을 넘지 않도록 `min()`으로 제한된다.
현재 성공한 `PICKUP_NOW` 자체는 `ball_sections_processed`를 증가시키지 않고,
그 뒤 `SHOT` 결과가 해당 section을 처리한 것으로 센다.

## 2. 실제 phase 목록과 의미

### 2.1 코드에서 확인된 phase 문자열

명시적으로 생성, 비교, 설정 또는 테스트되는 phase는 다음과 같다.

- `AUTO`
- `BALL_SEARCH`
- `BALL_APPROACH`
- `GOAL_SEARCH`
- `GOAL_APPROACH`
- `HURDLE_APPROACH`
- `LINE_TRACK`
- `FINISH` (`source_for_phase()`가 정확히 이 문자열을 line으로 매핑)
- `WALK_TO_FINISH`
- `FINISHED`
- `HURDLE_LOCK` (테스트에서 확인되는 lock 예)

또한 실행 중 특수 모션이 있으면 현재 phase 뒤에 동적으로 `_LOCK`을 붙인다.
따라서 `AUTO_LOCK`, `BALL_APPROACH_LOCK`, `GOAL_APPROACH_LOCK`,
`WALK_TO_FINISH_LOCK` 등도 입력 phase에 따라 생성 가능하다.

`source_for_phase()`는 문자열 prefix도 허용한다.

- `BALL*`, `PICK*` → ball
- `GOAL*`, `SHOOT*` → goal
- `HURDLE*`, `JUMP*` → hurdle
- `LINE*`, 정확히 `FINISH` → line
- 알 수 없는 문자열 → source `none`

즉, 현재 phase는 enum으로 제한되지 않는다. 아래 표는 실제로 명시된 phase만
정리하며 임의 prefix 조합은 별도 phase로 만들지 않는다.

### 2.2 phase별 현재 동작

| phase | 사용하는 Vision 입력 | 발행 가능한 action | 진입 조건 | 성공 종료 조건 | 실패/timeout | 다음 phase |
| --- | --- | --- | --- | --- | --- | --- |
| `AUTO` | hurdle, ball, goal, line | 선택 source의 모든 action | 초기 기본값, 외부 입력, 특수 모션 실패/timeout, `SHOT`/`GO` 성공 | `PICKUP_NOW` 성공, `SHOT` 또는 `GO` 성공 | `PICKUP_NOW`/`SHOT` 실패 후 finish가 아직 비활성일 때 `AUTO` 유지 | pickup 성공 → `GOAL_APPROACH`; shot/go 성공 → `AUTO`; pickup/shot 실패이면서 finish 활성 → `WALK_TO_FINISH` |
| `BALL_SEARCH` | ball과 fallback line; confirmed hurdle은 항상 우선 | ball action, line action, hurdle action, `WAIT` | 외부 `/mission/phase`만 확인됨 | `PICKUP_NOW + SUCCEEDED` | 실패/timeout 후 finish 비활성 → `AUTO`; 활성 → `WALK_TO_FINISH` | 성공 → `GOAL_APPROACH` |
| `BALL_APPROACH` | ball; confirmed hurdle은 항상 우선 | ball action 또는 hurdle action | 외부 입력 또는 테스트 fixture에서 사용. 자동 진입 조건은 현재 미구현 | `PICKUP_NOW + SUCCEEDED` | 위와 동일 | 성공 → `GOAL_APPROACH` |
| `GOAL_SEARCH` | goal과 fallback line; confirmed hurdle은 항상 우선 | goal action, line action, hurdle action, `WAIT` | 외부 `/mission/phase`만 확인됨 | `SHOT + SUCCEEDED` | 실패/timeout 후 finish 활성 → `WALK_TO_FINISH`, 아니면 `AUTO` | 성공 후 finish 활성 → `WALK_TO_FINISH`, 아니면 `AUTO` |
| `GOAL_APPROACH` | goal; confirmed hurdle은 항상 우선 | goal action 또는 hurdle action | `PICKUP_NOW + SUCCEEDED` 또는 외부 입력 | `SHOT + SUCCEEDED` | 위와 동일 | finish 활성 여부에 따라 `WALK_TO_FINISH` 또는 `AUTO` |
| `HURDLE_APPROACH` | hurdle | hurdle action | 외부 `/mission/phase`만 확인됨 | `GO + SUCCEEDED` | `GO + FAILED/TIMEOUT` → `AUTO` | `AUTO` |
| `LINE_TRACK` | line; confirmed hurdle은 항상 우선 | line action 또는 hurdle action | 외부 `/mission/phase`; `WALK_TO_FINISH` 내부에서 line planner 호출 시 임시 planning phase로도 사용 | phase 자체의 성공 종료 조건은 현재 미구현 | 현재 미구현 | TBD |
| `FINISH` | line; confirmed hurdle은 항상 우선 | line action 또는 hurdle action | 외부 `/mission/phase`만 확인됨 | 현재 미구현 | 현재 미구현 | TBD. `FINISHED`와 다른 문자열 |
| `WALK_TO_FINISH` | finish와 line; confirmed finish가 최우선 | `CROSS_FINISH`, 아니면 line action | pickup/shot 처리 뒤 `finish_enabled=True`, 또는 외부 입력 | `CROSS_FINISH + SUCCEEDED` | `CROSS_FINISH + FAILED/TIMEOUT`이면 finish target을 재-arm하고 같은 phase 유지 | 성공 → `FINISHED`; 실패/timeout → `WALK_TO_FINISH` |
| `FINISHED` | 없음 | `STOP` | `CROSS_FINISH + SUCCEEDED`, `mission_complete=True`, 또는 외부 입력 | 계속 정지 | 별도 실패 처리 없음 | `FINISHED` 유지 |
| `*_LOCK` | planner는 Vision source를 선택하지 않음 | `WAIT` (`valid=False`) | `special_motion_running=True`일 때 현재 phase 뒤에 동적으로 추가 | matching 특수 terminal status | matching `FAILED/TIMEOUT`도 lock 종료 | 원래 phase가 아니라 status callback이 계산한 phase |

주의할 점:

- `MotionDecisionPlanner._select_source()`는 phase보다 confirmed hurdle을 먼저
  검사한다. 따라서 `BALL_*`, `GOAL_*`, `LINE_TRACK`에서도 hurdle이 확인되면
  hurdle planner가 action을 선택할 수 있다.
- `*_SEARCH`만 대상 미검출 시 line fallback과 기억 기반 recovery를 사용한다.
  `*_APPROACH`는 요청 source를 그대로 선택한다.
- 외부 `/mission/phase` 입력에는 유효성 검사와 현재 특수 모션 lock 보호가 없다.

## 3. 실제 action 목록

### 3.1 요청된 주요 action

| action | 실제 생성/사용 위치 | phase 관계 | terminal 여부 |
| --- | --- | --- | --- |
| `STRAIGHT` | `LineNavigationPlanner` | line이 선택되는 `AUTO`, `*_SEARCH` fallback, `LINE_TRACK`, `WALK_TO_FINISH` | 일반 motion gate 관리 |
| `LEFT` | `LineNavigationPlanner` | 위와 동일 | 일반 motion gate 관리 |
| `RIGHT` | `LineNavigationPlanner` | 위와 동일 | 일반 motion gate 관리 |
| `WAIT` | 통합 planner no-source/lock, goal/hurdle planner의 안전 대기, duplicate terminal 억제 | 모든 phase에서 입력 부재·불안전·lock 시 가능 | 실행 ack 없음 |
| `PICKUP_NOW` | `BallNavigationPlanner` | ball source가 선택된 phase | 특수 terminal, `requires_ack=True` |
| `SHOT` | `GoalNavigationPlanner` | goal source가 선택된 phase | 특수 terminal, `requires_ack=True` |
| `GO` | `HurdleNavigationPlanner` | confirmed hurdle이 있거나 hurdle phase | 특수 terminal, `requires_ack=True` |
| `CROSS_FINISH` | `MotionDecisionNode._select_mission_decision()` | `WALK_TO_FINISH`이고 finish 조건 충족 | 특수 terminal, `requires_ack=True` |

### 3.2 코드에서 추가로 확인된 action

| source | 추가 action |
| --- | --- |
| line | `STOP`, `RECOVER_LEFT`, `RECOVER_RIGHT` |
| ball | `STOP`, `TURN_LEFT`, `TURN_RIGHT`, `APPROACH`, `SLOW_APPROACH`, `FINE_FORWARD_STEP` |
| ball lost recovery | `BALL_LOST_STOP`, `HEAD_SCAN_LEFT`, `HEAD_SCAN_RIGHT`, `HEAD_CENTER` |
| goal | `ALIGN_LEFT`, `ALIGN_RIGHT`, `APPROACH_GOAL`, `RETREAT_GOAL`, `WAIT_SCORE_CONFIRMATION` |
| goal lost recovery | `GOAL_LOST_STOP`, `RECOVER_GOAL_TURN_LEFT`, `RECOVER_GOAL_TURN_RIGHT` |
| hurdle | `ALIGN_LEFT`, `ALIGN_RIGHT`, `APPROACH_HURDLE`, `WAIT_GO_CONFIRMATION` |
| finished node branch | `STOP` |

`MotionDecisionPlanner.TERMINAL_ACTIONS`에는 `PICKUP_NOW`, `SHOT`, `GO`만
있다. `CROSS_FINISH`는 planner가 아니라 node의 finish 우선 분기에서 직접
생성한다.

## 4. 영상처리 출력 사용 현황

아래 “사용 필드”는 analyzer가 발행하는 전체 schema가 아니라
`motion_decision_node`와 그 내부 planner가 실제로 읽는 필드이다.

### 4.1 `/vision/line_info`

실제 사용:

- `detected`
- `filtered_heading_error_deg` (없으면 `heading_error_deg`)
- `filtered_lateral_offset_norm` (없으면 `lateral_offset_norm`)
- `heading_quality`
- `geometry_quality`
- `detection_quality`
- `turn_angle_deg`
- `turn_consistency`

`mission_state_estimator`는 추가로 `missed_line_frames`를 읽는다.

현재 통합 알고리즘에서 미사용:

- `line_count`, `center_points_px`, segment 관련 배열과 개수
- `lateral_offset_px`, near/far heading 상세값
- median 값, filter history 상세값
- `mean_confidence`, `fit_rmse_px`
- `image_width`, `image_height`
- path continuity/debug 필드

### 4.2 `/vision/ball_info`

실제 사용:

- `detected`
- `confidence`
- `bearing_deg`
- `offset_x_norm`
- `depth_m`
- `distance_m`
- `depth_valid`
- `pickup_ready`
- `pickup_now`

현재 미사용:

- `state`, center/bbox/크기/면적
- `offset_y_px`, `offset_y_norm`, `horizontal_direction`, `elevation_deg`
- 3D lateral/vertical/horizontal distance 상세값
- `is_centered`, `is_close`, `approach_ready`, `is_in_pickup_window`
- pickup target/tolerance 값
- candidate와 priority/debug/camera/depth age/note 필드

### 4.3 `/vision/goal_info`

실제 사용:

- `detected`
- `confidence`
- `depth_m`
- `distance_m`
- `depth_valid`
- `bearing_deg`
- `offset_x_norm`
- `score_now`

현재 미사용:

- `state`, `aim_source`, `aim_bbox`
- center/bbox/크기/면적과 y축 offset
- `horizontal_direction`, `elevation_deg`, `lateral_offset_m`
- analyzer의 `is_centered`, `depth_in_score_range`,
  `score_depth_error_m` 값
- depth sample, candidate, image/camera/depth age/note 필드

planner는 중심과 scoring depth를 자체 설정값으로 다시 계산하고,
`score_now`는 준비 geometry가 맞을 때 confirmation gate로 사용한다.

### 4.4 `/vision/hurdle_info`

실제 사용:

- `detected`
- `confirmation_confirmed` (필드가 존재할 때 source 선택 gate)
- `confidence`
- `depth_m`
- `distance_m`
- `depth_valid`
- `ground_gap_m`
- `camera_bottom_gap_m`
- `hurdle_angle_deg`
- `go_now`

현재 미사용:

- `state`, center/bbox/크기/면적, x/y offset
- `horizontal_direction`, `bearing_deg`, `elevation_deg`
- `horizontal_distance_m`, `camera_bottom_gap_px`, `lateral_offset_m`
- estimated width와 left/right depth
- analyzer의 `is_parallel`, `ground_gap_in_go_range`,
  `go_ground_gap_error_m`
- depth sample, candidate, image/camera/depth age/note 필드

planner는 parallel, ground gap, bottom gap 조건을 다시 계산하고 `go_now`를
confirmation gate로 사용한다.

### 4.5 `/vision/mission_state`

`motion_decision_node`와 `MotionDecisionPlanner`는 이 토픽을 구독하지 않으며
어떤 필드도 읽지 않는다. 현재 소비자는 `MissionMapVisualizer`뿐이다.

발행 필드는 다음과 같다.

- `zone`, `mission`, `confidence`, `expected_objects`
- `line_detected`, `line_quality`
- `heading_error_deg`, `lateral_offset_norm`, `missed_line_frames`
- `last_seen_objects`, `state_age_sec`, `note`

따라서 현재 알고리즘 관점에서는 모두 미사용이며, 향후 Phase Manager가
사용하더라도 최종 phase가 아닌 참고 후보로 취급해야 한다.

### 4.6 참고: `/vision/finish_info`

요청된 네 analyzer 외에 현재 node는 finish 입력도 실제로 사용한다.

- `detected`
- `confirmed`
- `confidence`

`WALK_TO_FINISH`, `finish_enabled=True`, confidence가
`finish_min_confidence` 이상일 때만 `CROSS_FINISH`를 선택한다.

## 5. `/vision/mission_state` 분석

### 5.1 estimator 입력

`MissionStateEstimator`는 다음 두 토픽만 구독한다.

- `/vision/line_info`
- `/vision/detections`

ball/goal/hurdle analyzer의 구조화된 `/vision/*_info`는 구독하지 않는다.
raw detections에서는 `line`을 제외하고 confidence가 기준 이상인 각 class의
가장 강한 detection만 보존한다.

### 5.2 실제 발행 가능한 zone/state

`_estimate_zone()`이 실제 반환할 수 있는 `zone` 전체는 다음 7개다.

| zone | 발생 조건 |
| --- | --- |
| `START` | 노드 시작 후 `start_hold_sec` 이내 |
| `SCORE_GOAL_A` | fresh detection에 `goal` 또는 `backboard`가 존재 |
| `PICK_BALL_A` | fresh detection에 `ball`이 존재 |
| `WALK_TO_BALL_A` | fresh `hurdle` 존재, 또는 line 검출과 line quality 기준 충족 |
| `LINE_RECOVERY` | 위 조건들이 아니고 `missed_line_frames`가 기준 이상 |
| `UNKNOWN` | 그 밖의 불안정 상태 |

zone 고유값은 **6개**다:
`START`, `SCORE_GOAL_A`, `PICK_BALL_A`, `WALK_TO_BALL_A`,
`LINE_RECOVERY`, `UNKNOWN`.

`COURSE_ZONES` 상수에는 추가로 `PICK_BALL_B`, `SCORE_GOAL_B`,
`WALK_TO_BALL_B`, `WALK_TO_FINISH`, `FINISH`가 정의되어 있지만
`_estimate_zone()`은 이 값들을 반환하지 않는다. 즉 현재 발행 불가능한
계획/시각화용 정의다.

발행 가능한 `mission` 문자열은 다음과 같다.

- `wait_or_start`
- `score_goal_a`
- `pick_ball_a`
- `follow_line_to_ball_a`
- `recover_line`
- `hold_or_scan`

### 5.3 최종 phase로 사용하기 어려운 이유

현재 estimator 결과는 최종 phase로 사용할 수 없다.

- object가 보이면 course 순서와 완료 이력 없이 바로 Goal A 또는 Ball A로
  분류한다.
- A/B 구분이나 이미 처리한 pickup/shot 횟수를 사용하지 않는다.
- hurdle은 통과 phase가 아니라 `WALK_TO_BALL_A`로 분류한다.
- `PICK_BALL_B`, `SCORE_GOAL_B`, finish 관련 zone은 실제로 발생하지 않는다.
- motion `RUNNING`, `SUCCEEDED`, `FAILED`, `TIMEOUT`을 구독하지 않는다.
- 공을 실제로 집었는지, 슛이 끝났는지, finish crossing이 성공했는지 알 수 없다.

따라서 `zone`, object landmark, line 상태를 참고 후보 신호로 사용하는 것은
적절하지만, Phase Manager가 현재 진행도와 motion status를 함께 검증해야 한다.

motion status 없이는 확정할 수 없는 전환은 최소 다음과 같다.

- ball phase → goal phase (`PICKUP_NOW` 성공 필요)
- goal phase → 다음 course/finish phase (`SHOT` 결과 필요)
- hurdle 실행 후 일반 주행 복귀 (`GO` 결과 필요)
- finish 접근 → `FINISHED` (`CROSS_FINISH` 성공 필요)
- 실패/timeout 뒤 재시도 또는 다음 section 진행 여부

## 6. 책임 중복과 충돌 위험

### 6.1 현재 중복

| 책임 | 현재 소유자 | 중복/문제 |
| --- | --- | --- |
| 외부 phase 지정 | 임의 `/mission/phase` publisher | publisher 구현과 권위 규칙이 없음 |
| 최종 phase 저장 | `motion_decision_node.mission_phase` | 외부 callback과 내부 status callback이 모두 변경 |
| phase별 source 선택 | `MotionDecisionPlanner` | 변경은 하지 않지만 hurdle 전역 우선순위가 phase 경계를 넘음 |
| coarse mission 추정 | `MissionStateEstimator` | ball/goal/line landmark로 phase와 유사한 zone을 별도 판단 |
| 진행도 | `motion_decision_node` | decision과 phase 관리가 한 node에 결합 |
| 특수 motion 결과 전환 | `motion_decision_node` | 향후 Manager 책임과 직접 중복될 영역 |

외부 `/mission/phase` 메시지와 motion status가 가까운 시점에 도착하면 callback
실행 순서에 따라 마지막 대입이 최종 phase가 된다. source authority,
revision, transition validation이 없으므로 잘못된 역행도 허용된다.

Executor 호환 adapter 경로에도 phase 전환에 영향을 주는 lossy mapping이 있다.
`legacy_motion_executor_adapter`는 특수 `GO`를 `motion_id="forward"`로
변환하지만 `legacy_motion_status_adapter`는 `forward`를 `STRAIGHT`로
역변환한다. 따라서 이 경로의 status action은 원래 `GO`로 복원되지 않아
`motion_decision_node`의 특수 `GO` terminal 분기에 들어가지 않는다.
`PICKUP_NOW`, `SHOT`, `CROSS_FINISH`는 각각 `pick_ball`, `shoot`, `hurdle`에서
원래 특수 action으로 복원된다.

또한 특수 terminal 처리에서 active `event_id`가 있으면 event를 비교하고
action도 비교하지만 active/status `command_id`를 직접 비교하지 않는다.
Executor status adapter는 `event_id=null`을 발행하므로 이 경로에서는 사실상
action 일치와 선행 `RUNNING` lock에 의존한다. Phase Manager 구현 시 특수
전환도 원 요청 식별자를 명시적으로 상관시켜야 한다.

### 6.2 기존 navigation controller

각 controller는 analyzer를 직접 구독하고 별도 명령 토픽을 발행한다.

| controller | 출력 |
| --- | --- |
| `LineNavigationController` | `/navigation/line_command` |
| `BallNavigationController` | `/navigation/ball_command` |
| `GoalNavigationController` | `/navigation/goal_command` |
| `HurdleNavigationController` | `/navigation/hurdle_command` |
| `MotionDecisionNode` | `/navigation/motion_command` |

네 controller는 `full_system.launch.py`에는 포함되지 않지만 executable로
설치되어 독립 실행할 수 있다. 토픽 이름은 서로 다르므로 ROS publisher
자체가 동일 토픽에서 충돌하지는 않는다. 그러나 downstream bridge/algorithm이
개별 command와 통합 command를 동시에 소비하면 다음 문제가 생긴다.

- 서로 다른 source가 같은 시점에 상충하는 이동/회전 명령을 냄
- phase와 무관한 `PICKUP_NOW`, `SHOT`, `GO` 후보가 실행될 수 있음
- command ID와 latch가 여러 발행자 기준으로 분리됨
- motion status의 원 명령 소유자를 결정하기 어려움
- 급격한 방향 전환이나 중복 특수 모션 요청으로 실물 안정성이 저하될 수 있음

최종 운용 graph에서는 motion 명령의 유일한 발행자를
`motion_decision_node`로 제한하고 개별 controller 출력은 debug 전용으로
비활성화하거나 downstream 연결을 제거해야 한다.

## 7. 권장 최종 구조

```text
vision analyzers
  ├─ /vision/line_info
  ├─ /vision/ball_info
  ├─ /vision/goal_info
  ├─ /vision/hurdle_info
  ├─ /vision/finish_info
  └─ /vision/mission_state (참고 후보)
          │
          ▼
mission_phase_manager
  ├─ Vision 관측 수신
  ├─ /motion/status 수신
  ├─ current_phase와 진행도 관리
  └─ /mission/phase 발행
          │
          ▼
motion_decision_node
  ├─ /mission/phase 수신
  ├─ /vision/..._info 수신
  ├─ phase에 맞는 action 선택
  └─ /navigation/motion_command 발행
          │
          ▼
motion executor
  └─ /motion/status 반환 ──────> mission_phase_manager
                         └─────> motion_decision_node의 명령 gate
```

### 7.1 권장 상태 소유권

MissionPhaseManager:

- `current_phase`
- `pickups_completed`
- `shots_completed`
- 현재 코드에 존재하는 `ball_sections_processed`
- `finish_enabled`
- `mission_complete`
- active 특수 action/event 식별 정보
- 특수 모션 성공/실패/timeout 후 phase 전환
- `/vision/mission_state` 후보의 수용/거부

motion_decision_node:

- 최신 Vision 관측과 freshness
- 현재 phase에 맞는 source/action 선택
- 일반 모션 중복 차단
- 특수 action 중복 latch
- `command_id`, 필요 시 `event_id` 생성
- `/navigation/motion_command` 발행

영상처리:

- 검출 결과, confidence
- 위치, bearing, depth, distance
- `is_centered`, `pickup_ready`, `pickup_now`, `score_now`, `go_now` 같은
  카메라 기준 후보
- temporal confirmation
- `/vision/mission_state` 참고 신호

planner는 phase를 읽어 action만 반환하고 phase 상태를 변경하지 않는 현재
방식을 유지하는 것이 적절하다.

### 7.2 인터페이스 권장사항

현재 `/mission/phase`는 plain string과 임의 JSON을 모두 받아 검증이 어렵다.
구현 시 최소한 다음 정보가 있는 JSON 계약을 검토해야 한다.

```json
{
  "phase": "GOAL_APPROACH",
  "previous_phase": "BALL_APPROACH",
  "reason": "pickup_succeeded",
  "transition_id": 12,
  "mission_progress": {
    "pickups_completed": 1,
    "shots_completed": 0,
    "finish_enabled": false,
    "mission_complete": false
  }
}
```

필드 확정은 TBD이며, 기존 plain string subscriber 호환 기간도 별도로 정해야
한다. `command_id`/`request_id`와 phase `transition_id`의 역할은 섞지 않는다.

## 8. 상태 전환표

아래 표는 현재 코드의 실제 전환만 담는다.

| 현재 phase | Vision 조건 | 실행 action | 필요한 motion status | 성공 시 다음 phase | 실패/timeout 시 phase | 비고 |
| --- | --- | --- | --- | --- | --- | --- |
| `AUTO` 또는 ball 계열 | ball detected, confidence/depth/alignment 유효, `pickup_now` confirmation | `PICKUP_NOW` | `RUNNING` 후 `SUCCEEDED`/`FAILED`/`TIMEOUT` | `GOAL_APPROACH` | finish 비활성 `AUTO`; 활성 `WALK_TO_FINISH` | 성공 시 pickup count 증가 |
| `GOAL_SEARCH`/`GOAL_APPROACH` 또는 AUTO goal 선택 | goal detected, control range, centered, scoring depth, `score_now` confirmation | `SHOT` | 위와 동일 | finish 비활성 `AUTO`; 활성 `WALK_TO_FINISH` | finish 비활성 `AUTO`; 활성 `WALK_TO_FINISH` | 성공 시 shot count 증가; 결과와 무관하게 section 처리 |
| `HURDLE_APPROACH` 또는 confirmed hurdle 우선 | hurdle detected/confirmed, valid depth, parallel, gap 조건, `go_now` confirmation | `GO` | 위와 동일 | `AUTO` | `AUTO` | phase와 무관하게 confirmed hurdle이 우선될 수 있음 |
| `WALK_TO_FINISH` | fresh finish: detected, confirmed, confidence 기준 충족 | `CROSS_FINISH` | 위와 동일 | `FINISHED` | `WALK_TO_FINISH` | 실패 시 finish action 재-arm |
| `WALK_TO_FINISH` | finish 조건 미충족 | line planner action | 일반 action의 matching status | phase 변화 없음 | phase 변화 없음 | line planning은 내부적으로 `LINE_TRACK` 사용 |
| `FINISHED` | 조건 없음 | `STOP` | 없음 | `FINISHED` | 해당 없음 | `mission_complete_stop` |
| 임의 phase + 특수 motion 실행 중 | Vision과 무관 | `WAIT` | active 특수 terminal | status에 따른 위 전환 | status에 따른 위 전환 | 동적 `*_LOCK` |
| `BALL_SEARCH` | ball이 control range 밖/미검출, recovery도 비활성; line detected | line action | 일반 action status | phase 변화 없음 | phase 변화 없음 | 자동 ball phase 진입/종료는 TBD |
| `GOAL_SEARCH` | goal이 control range 밖/미검출, recovery도 비활성; line detected | line action | 일반 action status | phase 변화 없음 | phase 변화 없음 | 자동 goal phase 진입/종료는 TBD |
| `LINE_TRACK` | line tracking | line action | 일반 action status | TBD | TBD | phase 완료 기준 현재 미구현 |

현재 특수 상태 callback은 `CANCELLED`와 `REJECTED`를 phase 전환 terminal로
처리하지 않는다. 이 두 상태의 Manager 정책은 TBD다.

## 9. 단계적 구현 계획

이번 작업에서는 아래 구현을 수행하지 않는다.

### 1단계: 순수 `MissionPhaseManager` 상태 객체

예상 파일:

- 신규 `src/mission_control/mission_control/mission_phase_manager.py`
- 신규 `src/mission_control/test/test_mission_phase_manager.py`

ROS 의존성 없이 phase, progress, active special event와 transition 결과를
관리한다. 허용 phase와 status를 명시적으로 검증한다.

### 2단계: 단위 테스트

예상 파일:

- `src/mission_control/test/test_mission_phase_manager.py`

pickup/shot/go/cross-finish 성공·실패·timeout, 중복/오래된 status,
최대 카운터, finish 활성화와 역행 방지를 검증한다. `CANCELLED`,
`REJECTED` 정책을 먼저 확정한다.

### 3단계: `motion_decision_node` 내부 phase 변경 이전

예상 파일:

- `src/mission_control/mission_control/motion_decision_node.py`
- `src/mission_control/test/test_motion_decision_node.py`

`_motion_status_callback()`의 phase/progress 변경과
`_update_action_progress()`를 Manager로 옮긴다. decision node에는 일반 gate와
action latch에 필요한 status 처리만 남긴다.

### 4단계: `/mission/phase` publisher 노드 연결

예상 파일:

- 신규 `src/mission_control/mission_control/mission_phase_manager_node.py`
- `src/mission_control/setup.py`
- 관련 launch 파일
- 신규 node 단위 테스트

Manager node가 Vision 후보와 `/motion/status`를 구독하고 권위 있는
`/mission/phase`를 발행한다. 기존 수동 publisher와의 동시 사용을 금지한다.

### 5단계: mock 통합 테스트

예상 파일:

- 신규 mission phase mock input/status helper
- 신규 launch test
- 필요 시 `src/mission_control/launch/mission_motion_mock.launch.py`

실제 카메라나 SDK 없이 Vision → phase → decision → mock executor → status →
다음 phase 전체 경로를 검증한다.

### 6단계: 기존 navigation controller 비활성화 확인

예상 파일:

- 운용 launch 파일
- launch graph 테스트 또는 문서

개별 controller가 운용 launch에 포함되지 않고, downstream이
`/navigation/motion_command`만 소비하는지 확인한다. executable 자체 삭제는
별도 결정 사항이다.

### 7단계: 실제 영상처리 연결

예상 파일:

- `mission_phase_manager_node.py`
- 실제 운용 launch
- parameter/config 및 통합 테스트

먼저 recorded/mock 데이터 또는 시뮬레이션으로 검증한 뒤 실물에 연결한다.
phase 오판은 잘못된 pickup/shot/hurdle motion과 낙상으로 이어질 수 있으므로
실물 시험 전 status timeout, stale Vision, emergency stop 정책이 필요하다.

## 10. 영상처리팀 및 알고리즘팀 확인 TBD

- Ball A/B, Goal A/B를 Vision만으로 구분할 식별 신호가 있는가?
- `/vision/mission_state`의 A/B 및 finish zone을 실제로 발생시키려면 어떤
  landmark와 course 순서 정보가 필요한가?
- `pickup_now`, `score_now`, `go_now`, finish `confirmed`는 후보인지 최종
  실행 허가인지 계약을 명확히 해야 한다.
- analyzer와 planner가 geometry 조건을 각각 다시 계산하는 구조에서 어느
  계층의 threshold를 기준값으로 삼을 것인가?
- `PICKUP_NOW` 성공 뒤 section을 언제 완료로 셀 것인지 현재 정책
  (`SHOT` 결과 시 처리)이 경기 규칙과 맞는가?
- pickup/shot 실패를 section 처리로 셀 조건과 재시도 횟수는 무엇인가?
- `GO` 성공 뒤 정확한 다음 phase가 항상 `AUTO`인지 확인이 필요하다.
- `LINE_TRACK`의 완료 조건과 다음 phase는 무엇인가?
- confirmed hurdle의 전역 최우선 처리가 모든 phase에서 맞는가?
- 특수 motion `CANCELLED`, `REJECTED` 시 phase와 재시도 정책은 무엇인가?
- 외부 운영자 phase override를 허용한다면 Manager 상태와 진행도를 어떻게
  동기화할 것인가?
- finish 검출 토픽/schema와 confidence/confirmation 기준의 최종 소유자는
  누구인가?
- `/mission/phase` JSON schema, QoS, latched/transient-local 필요 여부,
  transition ID 정책을 확정해야 한다.

## 11. 결론

현재 저장소에는 `/mission/phase`의 권위 있는 publisher가 없고,
`motion_decision_node`가 외부 phase 입력을 받으면서 동시에 motion status로
phase와 진행도를 직접 변경한다. `/vision/mission_state`는 coarse landmark
추정 및 시각화 신호이며 motion 완료 이력이 없어 최종 phase로 사용할 수 없다.

권장 구조는 phase와 진행도 및 특수 terminal 전환을
`MissionPhaseManager`가 단독 소유하고, `motion_decision_node`는 발행된
phase와 Vision을 이용해 하나의 action만 선택하는 것이다. 개별 navigation
controller는 최종 운용 명령 경로에서 비활성화하여 명령 발행자를 하나로
유지해야 한다.
