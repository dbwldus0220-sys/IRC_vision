# 2026-09-23 14:43:54 녹화: 약 1분 40초의 큰 제자리 우회전

## 결론

공 픽업 뒤 10초 기본 라인 주행 중 일반 `RIGHT` 명령이 나왔고,
이 명령의 고정 매핑 때문에 `제자리우회전45도(9회)`가 실행됐다.
일반 라인 플래너는 앞쪽 코너의 예측 각도를 조향값에 더하지만,
코너까지 남은 거리나 `corner_approach_motion`을 회전 허용 조건으로 쓰지 않는다.
코너가 아직 앞에 있는 상태에서 큰 회전으로 이어질 수 있는 구조다.
제어 코드와 모션 카탈로그는 수정하지 않았다.

## 실제 실행 순서

판단 로그: `/home/jet/.ros/log/python3_4517_1790142246031.log`
영상: `/home/jet/Videos/Screencasts/Screencast from 2026년 09월 23일 14시 43분 54초.webm`

| KST | 사건 |
|---|---|
| 14:45:19.322 | 픽업 후 라인 정렬: heading +26.465°, RIGHT_2 요청 |
| 14:45:23.122 | 픽업 후 라인 정렬: heading +15.386°, RIGHT_2 요청 |
| 14:45:26.944 | 10초 기본 라인 주행 시작 |
| 14:45:30.126 | 첫 STRAIGHT 완료 |
| 14:45:33.430 | 두 번째 STRAIGHT 완료 |
| 14:45:33.521 | 일반 라인 RIGHT 선택, pre-motion settle 0초 |
| 14:45:36.413 | RIGHT 성공 완료 |
| 14:45:36.781 / 37.175 | 라인 미검출 탐색 LINE_LOST_TURN_LEFT 각각 완료 |
| 14:45:37.221 | 10초 주행 종료, GRABBED에 따라 카메라 전환 시작 |

영상 터미널에서 두 STRAIGHT는 `전진실험45도(6회)`, 문제의 RIGHT는
`제자리우회전45도(9회)`의 성공 메시지로 확인된다.
이 RIGHT는 앞선 픽업 후 heading 정렬용 RIGHT_2와 다른 명령이다.

- [회전 전 화면](20260923_144354_review/before_right.png)
- [RIGHT 시작 부근 화면](20260923_144354_review/right_started.png)
- [9회 우회전 완료 및 라인 탐색 화면](20260923_144354_review/right_completed_and_line_search.png)

## 판단과 실제 모션의 연결

[라인 플래너](../src/step/step/line_navigation_planner.py#L374)의 기본 계산:

`steering = heading + 24 × offset_norm + 0.15 × reliable_turn_preview`

시작 부근 화면에는 heading +8.9°, offset -0.256, preview +63.0°,
corner distance 0.74m, Approach STRAIGHT_5가 표시된다.
직전 화면에는 corner distance 0.85m, Approach STRAIGHT가 표시된다.
예측 각도가 신뢰도 조건을 통과했다고 가정하면:

`8.9 + 24 × (-0.256) + 0.15 × 63.0 = 12.206°`

일반 방향 분류는 최초 진입 기준 12° 초과, heading 5° 초과,
기본 3회 판단 확인을 거치면 RIGHT를 선택한다. 회전 상태 유지 기준은 7°다.
3회 확인 상태에는 직전 보행 중 수집한 프레임의 재생 결과도 들어갈 수 있으므로,
정지 후 새 프레임 3장을 반드시 기다린다는 뜻은 아니다.

[브리지](../src/mission_control/mission_control/motion_command_bridge_node.py#L152)는
계산된 조향 각도와 무관하게 `RIGHT → line_turn_right_large`로 고정 매핑한다.
[alias](../src/irc_step_motion_executor/config/motion_aliases.yaml#L20)는
이를 `제자리우회전45도(9회)`로 연결한다.
카탈로그 이름의 45도는 카메라 자세이며, 현재 주석의 9회 회전 보정값은 약 95°다.
이 녹화에서 물리적으로 95° 돌았다는 측정 결과는 아니다.

화면의 `Approach`는 비전 분석 값이다. 실제 라인 플래너는 그 값을 사용하지 않아
화면에 STRAIGHT/STRAIGHT_5가 보이면서 실행 명령은 RIGHT일 수 있다.

## 오프라인 확인 및 한계

실제 플래너와 브리지 함수에 화면의 반올림 수치를 입력해 재현했다.
저장되지 않은 `turn_consistency`는 1.0으로 가정했다.
코너 거리를 0.20 / 0.74 / 2.00m로 바꿔도 모두
`STRAIGHT → STRAIGHT → RIGHT → 제자리우회전45도(9회)`가 나왔다.
preview만 0으로 바꾸면 steering 2.756°, STRAIGHT가 유지됐다.
[재현 결과](20260923_144354_review/offline_reproduction.json)

원본 line_info와 decision_debug 전체가 저장되지 않아 명령 발행 순간의 모든
입력값을 정확히 복원한 것은 아니다. 화면과 명령 배너도 비동기 갱신된다.
영상은 가변 프레임 간격이므로 순차 디코딩한 미디어 시각을 사용했으며,
화면 녹화 타이머와 약 1~2초 차이가 있어 사건 시각은 ROS 로그를 기준으로 삼았다.

## 수정 방향

- 앞쪽 코너 예측과 현재 발밑 라인의 방향 보정을 구분하고,
  코너 진입용 큰 회전에는 도달 조건을 둔다. 접근 중 필요한 작은 방향 보정은 유지한다.
- 일반 RIGHT를 무조건 9회에 연결하는 대신, 실제 방향 오차와 보정된 모션 각도를
  대응시킨다. 계산된 작은 보정값을 큰 코너 회전으로 확대하지 않도록 한다.
- 이 경로는 모든 기본 라인 주행에서 공유하므로, 수정은 공 픽업 후 10초뿐 아니라
  일반 코너 주행에도 영향을 준다. 오프라인 입력 재생/시뮬레이션으로 먼저 확인해야 한다.
