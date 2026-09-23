# 픽업 정렬 후 후진 연결 및 2분 10초 정지 분석

## 변경

- 일반 픽업의 `PICKUP_MOTION_TAIL`에서 마지막 `pickup_fine_forward_0`를 제거했다.
- 접근/꽃게걸음 완료 → 기존 3초 대기 → 새 영상의 미세 정렬 완료 확인 → `pickup_pre_backward_camera_down` 순서다.
- 정렬 완료 기준은 기존대로 `bottom_distance_px <= 300`, `-30 <= offset_x_px <= 55`다. 충분히 접근하지 않았으면 미세전진을 반복하고, 좌우 정렬이 안 됐으면 꽃게걸음을 수행한다.
- 초기 접근, 정렬 보정, 공 분실 탐색 조건과 관절 JSON은 이번 변경에서 수정하지 않았다.
- 별도 `use_no_ball_pickup_path` 경로의 미세전진은 유지한다. 이 경로에는 일반 미세 정렬 완료 확인이 없다.
- 설치된 Python 패키지는 `build/mission_control/mission_control`을 통해 소스를 참조한다. 노드를 재시작하면 수정이 반영된다. 로봇 실행은 하지 않았다.

## 새 영상과 같은 실행의 로그

영상: `/home/jet/Videos/Screencasts/Screencast from 2026년 09월 23일 11시 38분 47초.webm`

- 실행: `/home/jet/.ros/log/2026-09-23-11-38-54-660489-jet-10766/launch.log`
- 판단: `/home/jet/.ros/log/python3_10777_1790131136287.log`
- 브리지: `/home/jet/.ros/log/python3_10781_1790131135996.log`
- 실행기: `/home/jet/.ros/log/sdk_motion_executor_10783_1790131135145.log`

사용자는 2분 10초 부근에 다리 자체가 전혀 움직이지 않았다고 확인했다.

| 실제 시각(KST) | 명령 | 입력 heading | 결과 |
| --- | --- | --- | --- |
| 11:40:52.477 | 525, POST_BALL_LINE_TURN_RIGHT_2 | 21.712° | 11:40:53.126 SUCCEEDED |
| 11:40:56.278 | 526, POST_BALL_LINE_TURN_RIGHT_2 | 21.642° | 11:40:56.931 SUCCEEDED |
| 11:41:00.078 | 527, POST_BALL_LINE_TURN_RIGHT_2 | 21.747° | 11:41:00.726 SUCCEEDED |
| 11:41:34.278 | 536, POST_BALL_LINE_TURN_RIGHT_2 | 21.739° | 11:41:34.925 SUCCEEDED |

관측 나이는 0.003~0.073초였다. 판단은 약 3.8초마다 우회전 명령을 내고 약 0.65초 후 성공 응답을 받았다. 나머지는 모션 후 대기다. 영상에서도 라인 각도가 약 21.7°로 유지된다. 명령 차단 또는 영구적인 dwell 대기가 아니라, 실행기의 성공 보고와 실제 모터 동작이 불일치한 상황이다.

## 성공으로 보고되는 코드 경로

실행기 로그 첫 줄: `Motion position tolerance check: DISABLED`.

빌드가 참조하는 SDK는 `/home/jet/IRC/external_sdk/robot_motion_player_sdk_work_20260801/final step`이다.

`robot_motion_player.cpp`에서:

1. `updateRunning()`은 시작 위치 읽기 실패 시 저장된 목표 자세가 있으면 그 자세를 사용한다.
2. 궤적 Goal 전송이 실패해도 오류를 기록하고 재생 시간을 계속 진행한다.
3. `updateSettling()`은 `position_tolerance_enabled_ == false`이면 실제 위치 확인 없이 `complete_motion(MotionError::None, "")`로 성공 처리한다. 앞선 전송 실패 오류도 이 경로에서 성공으로 덮일 수 있다.

따라서 이 실행에서 `SUCCEEDED`는 실제로 15° 회전했다는 증거가 아니다. 모터가 정지해도 명령/성공/재판단이 반복될 수 있다.

물리적인 최초 정지 원인은 현재 자료로 확정할 수 없다. 저장된 ROS 로그에는 SDK의 표준 오류 출력과 모터별 Present Position, Torque Enable, Hardware Error Status가 없다. 전원/버스 단절, 토크 해제 등을 구분하려면 지지 상태에서 해당 값을 확인하고 SDK stdout/stderr를 함께 기록해야 한다. 이 진단에서는 모터 명령이나 설정 변경을 수행하지 않았다. 위치 검사만 켠다고 전원/통신 문제가 해결되는 것은 아니다.

## 잡기 전 근접 회전

사용자가 지적한 범위는 잡기 후 라인 복귀가 아니라 공을 잡기 전 근접 정렬이다.

현재 미세 정렬은 공이 보일 때 전진 또는 꽃게걸음을 사용한다. 공이 `detected=false`, `raw_detected=false`가 되면 같은 단계에서도 기억한 방향으로 약 45° 탐색 회전을 요청할 수 있다. 근접 접근 완료 이력으로 이 회전을 막는 조건은 없다.

새 영상 실행의 판단 로그에는 11:40:09.087에 `BALL lost during pickup positioning`이 있고, 11:40:10.056에는 다시 검출이 확정됐다고 기록된다. 분실 기록만으로 그 직후 탐색 회전이 실행됐다고 단정할 수는 없다. 노드 로그는 개별 픽업 보정의 action/reason 전체를 저장하지 않는다.

새 영상의 1:13.6 및 1:18.4 부근에서 확인한 근접 보정 배너는 `PICKUP / SIDE STEP LEFT`다. 1:13 부근 입력은 `Robot dx=-117px`, `Bottom dy=126px`로, 현재 초기 정렬의 근접 왼쪽 꽃게걸음 조건과 일치한다. 이 두 장면을 제자리 회전 실행의 근거로 사용할 수는 없다.

## 검증

- 브리지·판단 planner·판단 node: 913개 통과.
- 픽업 mission phase 흐름: 22개 통과.
- 첫 번째/두 번째 픽업의 실제 alias 순서: 2개 통과.
- 수정된 순서는 미세 정렬 완료 직후 후진 요청을 보내며 추가 준비 자세/미세전진을 보내지 않음을 검증했다.
- 이전 접근 횟수 0/2회, 꽃게 보정, 비복구성 보정 실패, 공 미검출 별도 경로, dwell 및 픽업 완료 흐름을 검증했다.
- 확장 검사에서는 골대 기대값과 기존 모션 카탈로그/hash/alias 불일치 15개가 남았다. 변경 전 tail을 메모리에서 복원해도 같은 15개가 실패하므로 이번 수정 범위에서 수정하지 않았다.
- `git diff --check` 통과.

다음 실물 검증에서는 먼저 시뮬레이션/모의 입력을 확인하고, 지지 상태에서 미세걸음 또는 꽃게걸음 종료 자세가 후진 시작 자세로 연결될 때 급격한 관절 변화가 없는지 확인해야 한다.
