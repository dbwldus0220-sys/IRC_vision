# 2026-09-23 12:11:39 영상: 근접 대기와 잡기 후 반복 회전

제어 코드는 변경하지 않았다. 영상·같은 실행의 ROS 로그·현재 코드 및 모터를 연결하지 않은 함수 재현을 대조했다. 아래 모션의 “완료”는 실행기가 보낸 성공 응답을 뜻하며 실제 관절 이동 완료를 보증하지 않는다.

## 근거 자료

- 영상: `/home/jet/Videos/Screencasts/Screencast from 2026년 09월 23일 12시 11분 39초.webm`
- 판단: `/home/jet/.ros/log/python3_15346_1790133118681.log`
- 브리지: `/home/jet/.ros/log/python3_15348_1790133118745.log`
- 실행기: `/home/jet/.ros/log/sdk_motion_executor_15350_1790133117469.log`
- launch: `/home/jet/.ros/log/2026-09-23-12-11-57-023996-jet-15334/launch.log`
- 모션명: `src/irc_step_motion_executor/config/motion_aliases.yaml`

영상 시각은 대략적인 위치다. WebM PTS, 화면 위 녹화 카운터와 실제 시계 사이에 약 1~2초 차이가 있어 정확한 비교에는 로그의 KST 시각을 사용한다.

## 1. 100px 조건과 1분 부근 대기

100px 이하에서 꽃게걸음을 일괄 차단하지 않는다. `pickup_close_alignment_active`는 잡기 전 제자리 회전을 막는다. 단, 앞서 추가한 근접 공 분실 처리는 공의 재검출까지 `WAIT`를 반환한다. 따라서 미검출 중에는 추가 꽃게걸음과 잡기 전 후진도 진행하지 않는다. 시간 제한은 없다.

실제 흐름:

| KST | 영상 대략 | 관측/명령 |
| --- | --- | --- |
| 12:12:32.329 | 0:53 | `pickup_fine_forward_0` → `미세0도-4` 요청 |
| 12:12:34.322 | 0:55 | 미세걸음 완료 후 3초 dwell |
| 12:12:36 부근 | 0:57 | 화면에 `Robot dx=+84px`, `Bottom dy=45px` 표시 |
| 12:12:37.43~37.89 | 0:58~0:59 | `PICKUP / SIDE STEP RIGHT`, `Motion succeeded: 미세오옆꽃게0도` 확인 |
| 12:12:37.744 | 0:58 | `BALL lost during pickup positioning` 기록 |
| 12:12:37.892~40.943 | 0:59~1:02 | 3초 dwell 후 미세 정렬 확인 단계로 복귀 |
| 12:12:40.943~12:13:14.839 | 1:02~1:35 | 공의 확정 재검출을 기다림. 중간 raw 검출은 있었으나 확정 전에 사라짐 |
| 12:13:14.839 | 1:35 | `BALL confirmation completed` |
| 12:13:14.927~15.948 | 1:35~1:36 | `pickup_pre_backward_camera_down` → `공잡기전후진-2(2회)` |

즉 꽃게걸음 요청은 한 번 있었다. 그 뒤 공을 놓친 약 37초 동안 후속 진행이 지연됐다. “3초 dwell이 끝나지 않은 것”이 아니라 dwell 종료 후 재검출 조건을 충족하지 못한 것이다. 화면도 `NON BLOCKING DWELL`에서 `BALL PICKUP FINE ALIGN CHECK`로 바뀐다.

미세 정렬은 유효한 확정 공 검출과 `Bottom dy <= 300px`, `-30 <= Robot dx <= +55px`를 확인해야 후진으로 넘어간다. 100px는 회전 금지 기준이며 이 300px 정렬 완료 기준과 다르다. `dx=+84px`는 오른쪽 꽃게 보정 대상이다. 1:36의 `dx=-47px` 표시는 후진이 이미 시작된 뒤의 새 영상값이므로 후진을 결정한 프레임의 값으로 사용하면 안 된다.

함수 재현 결과:

```text
Bottom dy=45, dx=+84, detected=true -> BALL_PICKUP_CRAB_RIGHT
같은 픽업에서 detected=false, raw_detected=false -> WAIT
재검출, Bottom dy=45, dx=0 -> BALL_PICKUP_FINE_ALIGN_CONTINUE
```

근거 화면: [꽃게 성공 응답](20260923_121139_review/0059_crab_succeeded.jpg), [공 분실 정렬 대기](20260923_121139_review/0110_ball_lost_wait.jpg), [잡기 전 후진](20260923_121139_review/0136_pre_grasp_backward.jpg).

## 2. 공 잡은 뒤 요청된 모션

| KST | 명령/실제 alias 대상 |
| --- | --- |
| 12:13:18.991 | `pickup` → `공잡기리그랩까지 실전` |
| 12:13:27.946 | `pickup_grasp_check_pose` → `공확인자세` |
| 12:13:28.404~31.449 | 잡힘 검증: 52개 유효 프레임 모두 GRABBED, 최종 GRABBED |
| 12:13:31.443 | `pickup_retreat_2` → `후진실전-2(3회)` |
| 12:13:35.991 | `pickup_first_backward_turn_right` → `후진하고 제자리우회전(공)` |
| 12:13:41.943 | 픽업 시퀀스 성공, `POST_BALL_LINE_ALIGN` 진입 |

픽업의 고정 후진·우회전 이후 라인 정렬용 회전 요청은 총 18회였다.

| 실제 모션명 | 로직의 보정각 | 요청 횟수 |
| --- | --- | --- |
| `제자리우회전45도-1(2회)` | 오른쪽 15° | 8 |
| `제자리좌회전45도-1(1회)` | 왼쪽 30° | 9 |
| `제자리우회전45도-1(3회)` | 오른쪽 30° | 1 |

모션명의 `45도`는 카메라 자세이며 로봇 몸체를 매번 45° 돌린다는 뜻이 아니다. 위 각도는 planner에 설정된 보정값이다.

순서는 첫 전진 전 `우2 → 우2 → 우2 → 좌1 → 좌1 → 우2 → 좌1`(7회), 4회 전진과 8회 전진 사이 `우2 → 좌1 → 좌1 → 우2 → 좌1 → 좌1 → 우2`(7회), 8회 전진 이후 `우3 → 우2 → 좌1 → 좌1`(4회)다. 마지막 좌1은 라인 미검출 탐색이다. 별도의 공 탐색 회전은 이 18회에 포함하지 않았다.

## 3. 라인 정렬 회전이 많은 직접 원인

`motion_decision_planner.py`의 `_plan_post_ball_line_align()`은 다음 둘 중 하나를 매번 고른다.

- `abs(filtered_lateral_offset_norm) <= 0.45`: 라인의 기울기 `heading`으로 회전한다.
- `abs(filtered_lateral_offset_norm) > 0.45`: 화면 아래쪽 라인 점을 향하는 각도로 회전한다.

두 각도는 방향이 반대일 수 있다. 경계에서 기준을 유지하는 상태/여유 구간이 없고, 반복 횟수나 오차 개선 여부로 중단하는 조건도 없다. 가까운 목표점 기준에서는 최소 좌회전 30°/우회전 15° 모션을 반복한다.

| KST | heading | offset | 사용한 기준/각도 | 요청 |
| --- | --- | --- | --- | --- |
| 12:13:49.627 | +18.904° | -0.351 | 라인 기울기 +18.904° | 우2 |
| 12:13:53.426 | +13.656° | -0.756 | 근거리 점 -68.832° | 좌1 |
| 12:13:56.926 | +16.952° | -0.518 | 근거리 점 -52.765° | 좌1 |
| 12:14:00.426 | +20.123° | -0.209 | 라인 기울기 +20.123° | 우2 |

우회전으로 기울기를 줄인 후 화면상 라인이 왼쪽으로 밀리면 반대 기준의 좌회전이 나온다. 다시 offset이 0.45 이내가 되면 heading 기준 우회전으로 돌아간다. 로그 입력으로 함수를 재현해 같은 `우2 → 좌1 → 좌1 → 우2`를 확인했다. 따라서 모든 반복을 모터 무응답으로 설명할 수는 없다. 이 구간에는 영상값 변화와 기준 전환이 실제로 있다.

## 4. 전진실험45도 4회/8회는 누락되지 않았다

| KST 요청~성공 | 영상 대략 | 명령 | command_id |
| --- | --- | --- | --- |
| 12:14:07.730~09.878 | 2:28~2:31 | `post_ball_forward_4` → `전진실험45도(4회)` | 740 |
| 12:14:38.431~42.696 | 2:59~3:04 | `post_ball_forward_8` → `전진실험45도(8회)` | 769 |
| 12:15:10.9 부근~11.762 | 3:32 부근 | `post_ball_camera_90` → `오뒤카메라90도`, 이후 GOAL_APPROACH | 819 |

영상에도 `POST BALL FORWARD 4/8` 배너와 각각의 한국어 `Motion succeeded`가 나온다. 근거: [4회 성공 화면](20260923_121139_review/0235_forward4_succeeded.jpg), [8회 성공 화면](20260923_121139_review/0304_forward8_succeeded.jpg).

현재 순서는 `잡기/후진/고정회전 → 라인 정렬 → 전진4 → 라인 정렬 → 전진8 → 라인 정렬 → 카메라90 → 골대 접근`이다. 따라서 고정 우회전 직후 4회/8회가 연달아 실행되지는 않는다. 반복 정렬 때문에 첫 전진까지 약 26초, 첫 전진 완료 후 다음 전진 시작까지 약 29초 걸렸다.

## 5. 골대 단계에 공 탐색이 끼어드는 별도 버그

12:14:56.417에 `POST_BALL_LINE_ALIGN` 중 공이 0.774m에서 확정 검출되어 `BALL approach entry latched while stationary`가 발생했다. 이후 `BALL_LOST_FORWARD_2`, `BALL_APPROACH_TURN_LEFT_2`가 실행됐다. 12:15:11.763에 GOAL_APPROACH로 바뀐 뒤에도 `BALL_APPROACH_TURN_LEFT_2`가 계속 반복됐다.

원인:

1. `source_for_phase(POST_BALL_LINE_ALIGN)`은 `line`을 반환한다.
2. `_latch_ball_approach_entry()`는 `line` 중 공 접근을 허용하므로, 이미 잡은 공을 운반 중인 라인 정렬 단계에서도 `ball_approach_alignment_pending=True`, `ball_lock_active=True`가 된다.
3. `_select_mission_decision()`은 이 pending 상태를 골대 planner보다 먼저 처리한다.
4. 골대 전환 완료 시 pending 상태를 해제하지 않아 GOAL_APPROACH에서도 공 분실 탐색이 우선한다.

실제 node 메서드에 모의 입력을 넣어 재현했다:

```text
AFTER_BALL_DURING_LINE_ALIGN POST_BALL_LINE_ALIGN pending True lock True
DECISION_DURING_LINE_ALIGN BALL_APPROACH_TURN_LEFT_2
AFTER_TRANSITION GOAL_APPROACH pending True decision BALL_APPROACH_TURN_LEFT_2
```

따라서 골대 이동 문제에는 전진 모션 누락 외에, 미션 단계 사이의 공 접근 상태가 잘못 유지되는 문제가 있다.

## 6. 실행기 통신 오류 증거와 확인 한계

영상 약 3:29~3:30(12:15:07 부근)에는 다음 출력도 확인된다.

```text
[DYNAMIXEL POSITION READ] attempt=1/3 comm_result=-3001
detail=[TxRxResult] There is no status packet!
... attempt=2/3, 3/3 ...
Failed to read trajectory start Present Position; continuing from the last commanded pose
Motion succeeded: 제자리좌회전45도-1(1회)
```

[오류 화면](20260923_121139_review/0330_motor_read_failure.jpg). 실행기 로그의 위치 검사 설정도 DISABLED다. SDK는 시작 위치 읽기가 실패하면 이전 목표 자세를 사용하고, 위치 검사를 끈 경우 시간 재생 종료만으로 성공 처리할 수 있다. 이 자료로 상태 패킷 수신 실패는 확인되지만, 전원·케이블·토크·고의 전원 차단 중 어느 원인인지는 확정할 수 없다. 이 뒤쪽 오류를 앞선 꽃게걸음이나 전진4/8의 미동작 원인으로 소급해 단정할 수도 없다.

## 수정 방향

- 근접 공 분실: 회전 금지는 유지하되, 마지막 유효 정렬값을 제한적으로 활용할지 또는 자세/카메라로 재검출할지 정책을 정해야 한다. 정렬 여부가 확인되지 않은 상태에서 무조건 후진·잡기로 진행시키면 위치 오차를 숨길 수 있다. 재검출 대기 사유도 화면에 명시하는 것이 좋다.
- 라인 정렬: 기울기 정렬과 라인 진입 방향 정렬을 분리하고, 선택 기준이 바뀌는 경계에 여유 구간을 둔다. 횟수 제한만 추가해 억지로 전진하기보다 실제로 오차가 줄어드는지 확인하는 것이 우선이다.
- 골대 이동 중 공 제어: 운반/골대 접근 단계에서는 새 BALL 접근 활성화를 막고, 단계 전환 시 공 접근 pending 상태를 해제한다. 이후 정상적인 두 번째 공 미션에는 영향을 주지 않도록 단계별 조건으로 제한해야 한다.
- 모터 오류: SDK 위치 읽기/전송 실패를 별도 오류로 유지하고, 실제 관절 확인이 없는 성공과 혼동하지 않도록 해야 한다. 하드웨어 수정 검증은 지지 상태에서 진행한다.
