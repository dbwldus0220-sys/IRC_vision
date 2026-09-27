# 23:42:11 영상: 공 인식 대기와 pickup 정렬 기준

분석 대상: `/home/jet/Videos/Screencasts/Screencast from 2026년 09월 26일 23시 42분 11초.webm`

## 확인 결과

공보다 라인이 우선되어 무시한 상황이 아니다. 해당 구간에는 공 후보 검출/미검출이 반복되어 확정을 기다렸으며, 이후 실제 공 접근 회전이 실행되었다. 확정되고 유효한 거리가 1.5 m 이내인 공은 일반 AUTO 선택에서 라인보다 먼저 선택된다. 진행 중인 동작은 공 후보가 생겼다고 중간에 취소하지 않고, 동작 경계에서 확정을 기다린다.

아래 시간은 화면 상단 녹화 타이머 기준 대략값이다. 파일 프레임 PTS와 화면 타이머에는 차이가 있어 파일명 시간과 일치하지 않는다.

| 화면 시간 | 증거 | 해석 |
| --- | --- | --- |
| 약 1:08 | `frame_70.289.jpg`: BALL / WAIT, CANDIDATE 2/12 | 공 후보는 있으나 확정 전 |
| 약 1:11 | `frame_74.197.jpg`: ball 0.51, INFO STALE, BALL / WAIT, RAW | 검출 박스는 생기지만 공 확인은 안정적이지 않음 |
| 약 1:15 | `frame_76.045.jpg`: Analyzer CONFIRMED, TRACK; 현재 박스 confidence 0.35 | 확정 공을 얻은 사실 확인. 현재 검출 박스와 analyzer 결과는 다른 시각 데이터 |
| 약 1:17 | `frame_78.024.jpg`: LINE ALIGN / IN-PLACE TURN LEFT 2 | 실제 BALL_APPROACH_TURN_LEFT_2를 공유 모션 별칭 때문에 LINE ALIGN으로 표시 |

## 코드와 로그 근거

- `src/step/step/yolo26_detector.py`: 공 검출 confidence 기본 문턱 0.20.
- `src/step/step/ball_analyzer.py`: 후보 confidence 기본 문턱 0.45. 최근 20프레임 중 공간적으로 일치하는 후보 12회로 공을 확정한다. 따라서 화면에 박스가 있다고 바로 제어용 공으로 확정되는 것은 아니다. 특히 0.20~0.45 검출은 화면에 보일 수 있지만 analyzer 후보에서 제외된다.
- `src/step/step/temporal_confirmation.py`: 위치/크기 일치 여부와 missed frame을 관리한다. 확정 후에도 현재 프레임에 검출이 없으면 detected를 확정 상태로 내보내지 않는다. 12회는 연속 12회라는 뜻이 아니다.
- `motion_decision_node.py::_update_ball_confirmation_pending`: raw 후보가 있으면 현재 모션 이후 대기; raw가 0.5초 이상 사라지면 대기를 해제한다.
- 실행 로그 `python3_10355_1790433736491.log`: 1790433800.098 raw latch → 3800.806 lost → 3800.905 raw latch → 3804.805 confirmation completed. 다음 3810.222/3810.228에 `BALL_APPROACH_TURN_LEFT_2` 후 3초 대기/성공이 기록된다. 3812.080에는 재확정된다.
- `motion_command_bridge_node.py::ACTION_TO_MOTION_ID`: BALL_APPROACH_TURN_LEFT_2 → post_ball_line_turn_left_2. `yolo26_detector.py::_running_motion_banner`는 이 별칭을 LINE ALIGN으로 표시한다. 공/라인이 회전 모션을 공유해서 생긴 표시 문제이며 실제 소스 우선순위의 증거가 아니다. 이번 변경에서는 표시 코드를 수정하지 않았다.
- `INFO STALE`은 RGB와 analyzer 타임스탬프 차이가 overlay 기본 허용 50 ms를 넘었다는 표시다. 영상에서는 약 133~234 ms 차이가 보인다. 표시용 `_fresh_ball_info` 검사이며, 이 문구만으로 mission planner가 공을 폐기했다고 해석하면 안 된다.

가까워 보이는 것만으로 제어 조건이 충족되지는 않는다. 지속적인 RGB 후보 확정과 유효한 거리값이 모두 필요하다. 영상/로그로 후보 검출의 불안정 및 확정 지연은 확인했지만, 원본 RGB와 프레임별 detection 토픽 기록이 없어 모델 confidence 저하의 원인(가림, 배경, 흔들림 등)을 하나로 단정할 수 없다. 현재 영상에는 사람 손과 받침대가 공에 인접해 있으나 원인으로 확정하지 않는다. 검출/확정 문턱은 이번 요청에서 변경하지 않았다.

## 요청한 수정

- pickup 미세 접근/정렬 완료: 유효한 `Bottom dy <= 60 px` AND `-30 <= offset_x_px <= 55`.
- Bottom dy 기본값을 40 → 60으로 변경: planner, node, 두 full-system launch 및 README.
- 좌우 범위는 이미 요청과 같아서 유지. Bottom dy는 화면 하단부터 **공 중심**까지의 픽셀 간격이고, offset_x_px는 보정된 로봇 중심 기준이다.
- Bottom dy > 60이면 미세전진, 60 이하이지만 좌우 범위를 벗어나면 꽃게걸음, 두 조건을 만족하면 다음 pickup 단계로 진행한다. 무효/미검출 대기와 기존 모션 매핑은 유지했다.
- 40~60px 구간에서 추가 미세전진을 더 일찍 끝낼 수 있으므로 실제 집기 위치는 기존보다 멀어질 수 있다. 시뮬레이션/오프라인 판정 확인 후 지지한 상태에서 실기 검증이 필요하다.

## 검증

ROS 설치 환경과 작업공간 환경을 로드한 뒤 planner, decision node, full_system launch, full_system_robot launch 테스트를 실행했다. **807 passed**. 59/60/61px와 좌우 -31/-30/55/56px의 조합 12개 경계 사례를 포함한다. 실제 모터는 구동하지 않았다.

설치된 두 launch는 수정한 소스의 심볼릭 링크임을 확인했다. 이미 실행 중인 노드는 재시작해야 새 기본값이 적용된다. launch 인자로 별도 값을 지정했다면 해당 값이 기본값보다 우선한다.
