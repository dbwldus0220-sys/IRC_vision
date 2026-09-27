# 2026-09-26 22:45:27 녹화 판단 분석

분석만 수행했으며 제어 코드와 모션 JSON은 변경하지 않았다. 실제 로봇을 실행하지 않았다.

영상: `/home/jet/Videos/Screencasts/Screencast from 2026년 09월 26일 22시 45분 27초.webm`
로그: `/home/jet/.ros/log/python3_13737_1790430347080.log`

시간은 사용자 사진과 일치하도록 화면 상단 녹화 타이머를 기준으로 기술했다. 파일 내부 POS_MSEC는 녹화 타이머보다 약 1~2초 뒤의 값을 보인다. 순차 디코딩한 프레임에서 수치를 읽었으며, 모션 발행에 사용된 원본 vision JSON 전체가 저장된 것은 아니므로 특정 프레임이 정확한 발행 입력이었다고 단정하지 않는다.

## 3분 56초: 골대가 왼쪽에 조금 있지만 아직 멀어 미세 전진

`goal_3m56s.jpg` (파일 시간 238.013초): State=APPROACH / LEFT, Aim source=BACKBOARD, Depth Z=0.70 m, Offset X=-54 px, Offset norm=-0.084, Bearing=-3.4°, Shot now=NO, Planner=NO COMMAND.

- LEFT는 관측된 목표 위치다. goal_analyzer는 offset norm < -0.04이면 LEFT를 표시한다. 실제 좌회전 명령과는 구분해야 한다.
- 골대 방향 보정은 bearing 절댓값이 5° 이하이면 생략한다. -3.4°는 이 범위 안이다.
- 슛 거리 범위는 0.41±0.02 m다. 0.70 m는 아직 멀고, 0.50 < depth <= 0.85 m 분기로 GOAL_CAMERA90_FINE_FORWARD_3가 선택된다.
- 화면 값으로 순수 함수를 실행해 yaw 보정 None, 거리 명령 GOAL_CAMERA90_FINE_FORWARD_3를 재현했다.
- 로그 182~185행: GOAL_CAMERA90_TURN_RIGHT_2 완료(1790430562.8896), GOAL_CAMERA90_FINE_FORWARD_3 완료(1790430567.2219), 동일 미세 전진 완료(1790430571.5268), FINE_FORWARD_1 완료(1790430575.0381). 따라서 해당 구간은 우회전 정렬 이후 미세 접근 단계다.
- NO COMMAND는 화면 표시용 최근 명령이 없거나 0.8초 유효시간을 넘었다는 뜻이다. 실행 중 모션이나 다음 판단이 없다는 뜻은 아니다.

관련 코드: goal_analyzer.py:486, goal_navigation_planner.py:130, motion_decision_planner.py:1579, yolo26_detector.py:991.

## 4분 39초: 영상 기울기 기반 좌복귀를 이미 실행 중

화면 진행:

| 파일 시간 | 표시 | Image head | Offset norm | Ground head | Steering |
| --- | --- | --- | --- | --- | --- |
| 279.004초 | LINE FORWARD 6 | -13.5° | +0.547 | +8.04° | +25.14° |
| 279.541초 | LINE FORWARD 6 | -16.1° | +0.551 | +8.34° | +27.98° |
| 279.779초 | LINE RETURN LEFT 4 | -13.8° | +0.626 | +16.45° | +30.41° |
| 280.079초 | LINE RETURN LEFT 4 | -15.2° | +0.642 | +14.11° | +32.87° |
| 280.501초 | LINE RETURN LEFT 4 / SEARCH | 미검출 | 미검출 | N/A | N/A |

현재 코드의 분기:

1. offset > 0이면 라인은 RIGHT 쪽이다.
2. offset와 heading의 부호가 반대이고 |heading| >= 12°이며 |offset| < 0.55이면 복귀 회전을 생략하는 예외가 있다.
3. 위 예외 범위를 벗어난 뒤 heading <= -10°이면 RECOVER_RIGHT_TURN_LEFT_4가 선택된다.
4. 브리지는 TURN_LEFT_4를 line_recovery_left_4로 매핑하고, alias는 찐라인복귀좌회전45도(4회)를 가리킨다. 화면이 이를 LINE RETURN LEFT 4로 표시한다. 4는 이 모션 이름의 횟수 표기이며 4°라는 의미가 아니다.

화면 값으로 현재 순수 판단 함수를 재현했다:

- offset=+0.547, heading=-13.5° → STRAIGHT
- offset=+0.642, heading=-15.2° → RECOVER_RIGHT_TURN_LEFT_4

로그 195~199행도 우복귀 → 직진 → RECOVER_RIGHT_TURN_LEFT_4 완료(1790430608.4384) 순서를 확인시켜 준다.

### 핵심 원인과 한계

line_navigation_planner.py:309에서 읽는 것은 filtered_heading_error_deg / filtered_lateral_offset_norm이다. 화면 Ground head / Steering은 yolo_line_analyzer.py:192의 진단용 지면 투영에서 나온 값으로 실제 라인 복귀 판단에 사용되지 않는다. 따라서 지면 환산상 우측인 상황에서도 영상 기울기가 음수이면 좌복귀를 선택할 수 있다. 복귀 분기는 일반 heading+offset 조향 합산보다 먼저 반환한다.

원근 때문에 오른쪽의 라인이 영상에서는 먼 쪽으로 갈수록 왼쪽으로 기울 수 있다. 이 영상에서도 영상 기울기와 지면 환산 방향의 부호 불일치가 확인된다. 다만 지면 환산은 고정 호모그래피에 의존하므로, 흔들리는 실제 카메라에서 어느 값이 실제 지면 방향과 정확히 일치하는지는 별도 검증이 필요하다. 작은 Ground RMSE만으로 보정 정확성을 보장할 수 없다.

SEARCH는 현재 검출 상태다. 이미 시작한 일반 모션은 완료까지 잠기며 최신 영상이 사라졌다고 즉시 교체되지 않는다(motion_decision_node.py:2682). 큰 배너는 RUNNING 상태를 따로 표시한다(yolo26_detector.py:1024). 따라서 첨부 사진은 라인을 못 본 순간 새 좌회전을 만든 장면이 아니라, 앞서 결정한 좌복귀 도중 라인이 사라진 장면이다.

### 개선 방향

모션 JSON의 좌우 이름 교체나 회전 부호 일괄 반전으로 해결할 문제가 아니다. 지면 좌표 기반 조향을 실제 판단 입력으로 사용하는 방안과 영상/지면 방향 충돌 시 재관측하는 방안을 검토해야 한다. 지면 보정 유효성, 현재 카메라 자세, 미검출 처리, 0.55 경계 부근 반복 전환을 먼저 오프라인 재생과 시뮬레이션에서 검증해야 한다. 이번에는 변경하지 않았다.
