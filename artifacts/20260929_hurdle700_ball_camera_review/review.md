# 허들 700 mm 변경 및 BALL 카메라 미하향 분석

## 적용한 변경

- 허들 전용 `HURDLE_FINE_DISTANCE_M = 0.700`을 추가했다.
- 허들 분석기의 거리 구간 표시, 허들 플래너의 미세 접근/시퀀스 진입, 미션 제어권 진입, 인식 구간 표시를 같은 기준으로 맞췄다.
- 유효 거리 기준 `700 < depth <= 1000 mm`는 일반 접근, `depth <= 700 mm`는 근거리 구간이다. 정렬·검출·거리 유효성 조건은 계속 적용된다.
- BALL의 기존 550 mm 기준과 카메라 제어 코드는 변경하지 않았다.
- 사진의 STRAIGHT_0/찐미세45도-4 표와 현재 실행 경로는 차이가 있다. 현재 통합 미션은 정렬 조건을 만족한 근거리 허들에 `GO`를 보내 기존 미세 전진/대기/허들 넘기 시퀀스를 실행한다. 브리지는 허들 미세 전진을 `pickup_fine_forward_0`(찐미세0도-4)에 연결한다. 이번에는 해당 모션 연결과 순서를 보존했다.
- 근거리 시퀀스를 시작하는 위치가 최대 150 mm 멀어지므로, 실기 전 시뮬레이션에서 700 mm 전후의 전진 잔여 거리와 넘기 위치를 확인해야 한다.

## 카메라가 내려가지 않은 직접 원인

`src/irc_step_motion_executor/src/sdk_executor_node.cpp`의 `handle_ball_info()`는 아래 조건이 모두 맞아야 접근 중 머리 관절 override를 시작한다.

1. BALL 검출 및 `head_down_requested == true`.
2. 현재 실제 실행 모션이 `ball_head_override_motion_ids`에 포함됨.
3. 머리 override 활성화 상태이며 기존 BALL/GOAL/HURDLE override가 새 요청을 막고 있지 않음.

기본 허용 목록은 `line_forward_2/4/6/8/10`, `sdk_forward_4`, `forward`다. 당시 `BALL_APPROACH_RECOVER_LEFT_4`는 브리지에서 `line_recovery_left_4`(찐라인복귀좌회전45도(4회))로 연결되며 이 목록에 없다. 오른쪽 접근 보정 모션도 포함되어 있지 않다. 따라서 화면의 하단 진입 요청이 ON이어도 접근 보정 회전 중에는 `!ball_head_motion_active()` 때문에 하향 명령을 시작하지 않는다.

화면 `Head down: ON`은 실제 카메라 관절 도달 상태가 아니다. `ball_analyzer.py`가 공 중심과 화면 하단 사이의 거리 `bottom_distance_px <= 120`으로 만든 요청 플래그를 그대로 표시한다. 실행기의 허용 모션 확인이나 실제 하향 완료 여부를 표시하지 않는다.

또한 550 mm는 `BallNavigationPlanner` 및 `_track_ball_pickup_entry()`의 **픽업 진입 기준**이다. 위 C++ 접근 중 머리 하향은 mm 거리를 검사하지 않고 하단 픽셀 요청과 실행 모션을 검사한다. 두 조건을 구분해야 한다.

## 영상 및 로그 근거

원본 영상: `/home/jet/Videos/Screencasts/Screencast from 2026년 09월 29일 17시 26분 40초.webm`

- 공을 놓치기 직전 프레임에서 Depth Z 약 0.61 m, Bottom dy 119 px, Head down ON, Ground steer 약 -22.82도가 보인다. 약 0.64 m/110 px에서도 요청은 ON이었다.
- 실제 카메라 하향 override는 당시 기록되지 않았다. 접근 보정 회전 완료 기록이 반복되고, 17:28:09.927에 공 검출 상실을 저장했다.
- 17:28:10.234에 마지막 `BALL_APPROACH_RECOVER_LEFT_4`가 완료된다. 이후 브리지는 `valid is not true`인 대기 명령을 받는다. 화면의 SEARCH LEFT 문구만으로 회전 명령이 실제 실행되었다고 판단할 수 없다.
- 공을 다시 검출한 뒤 17:28:27.687에 `Ball entered bottom trigger: motor 0 override -64.0 deg latched`가 기록된다. 17:28:29.057에 일반 STRAIGHT가 완료되므로, 이때는 허용된 일반 전진 중 하향이 시작된 것으로 해석된다.
- 17:28:28.730에 `BALL pickup entry latched: distance_m=0.541`, 17:28:29.124에 PICKUP_NOW 진입이 기록된다. 하향 override가 픽업 진입보다 먼저 일어났다는 점도 두 조건이 별개임을 보여준다.

원시 ball_info 전체를 기록한 rosbag은 이번 분석에 사용하지 않았다. 따라서 검출 상실 전 모든 프레임에서 550 mm 이하가 없었다고 단정하지 않는다. 확인한 마지막 거리 표시는 약 610 mm이고, 그 구간의 픽업 진입 저장 로그가 없다는 것이 관찰 근거다. `INFO STALE` 표시도 보이지만 이는 화면 프레임과 분석 결과의 동기화 표시이므로 제어기에서 `depth_age_sec > 0.70`으로 거절됐다는 증거로 취급하지 않았다.

근거 파일:

- `ball_before_loss_depth_610mm.jpg`: 검출 상실 직전 거리/요청 표시.
- `ball_lost.jpg`: 공 검출 상실 상태.
- `timeline.log`: 해당 구간의 판단 노드와 C++ 실행기 로그, 한국 시간 병기.

## 별도로 고려할 BALL 개선

최소 수정 후보는 `line_recovery_left_4`, `line_recovery_right_4`를 하향 허용 목록에 추가하는 것이다. 이 경우 보정 회전 중에도 카메라가 내려가므로 공 시야 유지에는 도움이 될 수 있지만, 카메라 자세가 바뀌는 동안 기존 바닥 투영 기반 조향각의 유효성과 다음 모션 판단을 함께 검증해야 한다. 먼저 시뮬레이션/모의 실행기로 검증해야 하며, 이번 원인 분석에서는 BALL 동작을 수정하지 않았다.

표시 개선 후보는 요청 플래그와 실행기 heartbeat의 `ball_head_override_active`를 구분해서 표시하는 것이다. 후자도 관절의 실제 도달을 검증한 센서값은 아니므로 완료 표시로 사용하면 안 된다.

## 검증

실기 모션은 실행하지 않았다. ROS 환경을 로드한 뒤 소스 패키지 경로를 PYTHONPATH로 지정해 아래 9개 테스트 파일을 실행했다.

- step: test_hurdle_navigation_planner, test_hurdle_analyzer_depth_separation, test_yolo26_detector, test_approach_distance, test_ball_navigation_planner.
- mission_control: test_hurdle_positioning, test_motion_decision_planner, test_motion_command_bridge, test_mission_phase_flow.

결과: **1478 passed, 19 failed**. 허들 700 mm 경계 및 실제 모션 연결 테스트는 통과했다. 남은 실패는 공 미세 정렬 2건과 미션 흐름 17건이며, 변경 전 HEAD의 별도 /tmp 복사본에서도 같은 테스트 ID 19건이 실패함을 확인했다. 이번 변경으로 새로 실패한 테스트는 없다. 기존 실패를 숨기거나 기대값을 변경하지 않았다.

`tests_final.log`, `baseline.log`, `baseline_phase.log`에 상세 결과가 있다. `git diff --check`도 통과했다. 현재 build/step의 해당 Python 소스는 src와 같은 inode로 연결되어 있으며, 실행 중인 노드에는 재시작 후 변경이 반영된다.
