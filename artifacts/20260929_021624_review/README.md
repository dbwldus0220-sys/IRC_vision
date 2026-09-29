# 2026-09-29 02:16:24 녹화 분석

대상: `/home/sy/Downloads/Screencast from 2026년 09월 29일 02시 16분 24초.webm`

아래 초 단위는 파일의 재생 위치다. 영상 상단 녹화 타이머와는 약 1~3초 차이가 있다. 녹화 터미널에는 `IRC_vision_latest` 경로가 보인다. 해당 실행의 소스 버전·ROS 원본 메시지는 제공되지 않았으므로, 영상에서 확인한 사실과 현재 작업 공간 코드에 따른 재현을 구분한다.

## 1. 첫 BALL의 WAIT: 검출 실패보다 바닥 조향각 계산 실패가 핵심

[114초 화면](114_ball_wait_fresh_depth.jpg)에는 공 confidence 약 0.83, Depth Z 1.40m, CENTER 깊이 샘플 81개, RGB–Depth 차이 0.0ms, Camera info OK가 보인다. 그런데 `Ground steer: N/A`, `Steering: N/A`, `Angle source: WAIT CHECK`, `Planner: BALL / WAIT`다. 공이 보이지 않아서 기다린 상황이 아니다.

[116초 화면](116_ball_wait_stale_depth.jpg)에는 STALE 약 200ms도 나타난다. 일부 프레임의 깊이 동기화 지연은 추가 문제지만, 앞의 정상 깊이 프레임에서도 WAIT가 나왔으므로 STALE만으로 설명할 수 없다.

현재 코드 경로:

1. `ball_analyzer.ball_ground_geometry()`는 공 박스 아래쪽 중심을 바닥에 투영한다.
2. 라인과 공이 공유하는 `project_line_points_to_ground()`는 바닥 전방 거리 **0.35~1.05m** 밖의 점을 버린다.
3. 영상 박스를 대략 `[624, 122, 664, 155]` (RGB 1280×720)로 읽어 현재 homography에 넣으면, 범위 필터 전 전방 거리는 **1.141m**다. 따라서 `ground_projection_valid=False`가 된다.
4. 일반 공 접근은 ground 정보가 있으면 유효한 바닥 조향각을 요구한다. 값이 없을 때 이미지 각도 −9°로 대신 진행하지 않는다.
5. 일반 BALL 계획은 `invalid_ball_alignment`, 접근 정렬 체크포인트는 `invalid_ball_approach_alignment_input`으로 WAIT가 된다. 영상에는 두 reason 중 어느 것이 발행됐는지 나오지 않는다.

Depth Z 1.40m와 투영 전방 거리 1.141m는 기준이 다른 값이다. 1.40m를 직접 1.05m 제한과 비교한 결론이 아니다. 위 계산은 화면에서 읽은 근사 박스와 현재 기본 보정값을 사용한 재현이며, 당시 ROS 박스·파라미터의 원본 복원은 아니다.

수정 방향: BALL 제어 범위에 맞는 원거리 바닥 보정을 검증하고, 필요하면 공 접근용 검증 범위를 라인 피팅 범위와 분리한다. 상한만 올리면 검증되지 않은 거리에서 잘못된 조향각을 사용할 수 있다. 이번 작업에서는 주행 파라미터를 변경하지 않았다.

## 2. LINE LOST 후 LEFT: 코너 방향과 소실 탐색 방향이 서로 다름

확인한 진행 순서:

- 라인복귀 우회전 `RECOVER_RIGHT_TURN_RIGHT_4` 실행.
- 이어 라인복귀 좌회전 `RECOVER_LEFT_TURN_LEFT_4` 실행. [196초](196_corner_right_during_left_recovery.jpg)에는 실행 배너가 `LINE RETURN LEFT 4`이면서 분석 패널은 `Corner: RIGHT`다.
- [198초](198_corner_right_invalid_ground.jpg)에도 Corner RIGHT가 있지만 Ground head/Steering이 N/A다.
- [200초](200_corner_right_during_forward.jpg)에는 Corner RIGHT를 보면서 `LINE FORWARD 6` 실행 중이다.
- [202초](202_last_near_point_left.jpg)에는 라인이 화면 하단으로 빠져나간다. `Offset norm`은 양수지만 `NEAR` 점은 화면 중앙보다 왼쪽이다.
- 이후 라인 SEARCH. [208초 터미널](208_line_lost_left_status.jpg)에는 `LINE_LOST_TURN_LEFT`와 `찐제자리좌회전45도-1(2회)` 성공이 확인된다.

현재 소실 탐색 코드는 다음 순서다:

1. `observe_line_for_search()`가 신뢰도 조건을 만족하는 라인을 관측한다.
2. `center_points_px[0]`가 있으면, 일반 offset 대신 **가장 가까운 점의 x − 화면 폭/2**를 기억한다.
3. 라인이 사라지면 `_plan_source('line')`가 일반 코너 계획보다 먼저 그 방향으로 `LINE_LOST_TURN_LEFT/RIGHT`를 만든다.
4. 이 분기에는 `corner_direction=RIGHT`를 기억하거나 우선하는 조건이 없다.

따라서 가까운 점이 왼쪽이고 코너가 오른쪽인 장면에서, 소실 후 LEFT가 나오는 경로는 현재 코드로 재현된다. `Offset norm > 0` 또는 `Corner: RIGHT`여도 이 탐색 방향은 LEFT일 수 있다.

## 3. Corner RIGHT가 실제 RIGHT로 이어지지 않은 이유와 증거의 한계

`Corner: RIGHT`는 분석기의 코너 미리보기다. 실제 제어의 `LEFT/RIGHT` 명령을 그대로 보여 주는 필드가 아니다. 현재 `LineNavigationPlanner.plan()`은 이 문자열을 직접 명령으로 쓰지 않고 다음을 따로 평가한다.

- 유효한 바닥 heading과 라인 품질.
- 회전 미리보기의 크기·일관성(`turn_consistency >= 0.55`).
- heading + 24×offset + 0.15×유효 preview로 얻는 조향 오차.
- 라인복귀 보정이 필요한지, heading과 곡선 방향이 충돌하는지.
- 코너 진입 조향 오차 12° 초과, 같은 방향의 heading 5° 초과.
- 같은 회전 후보를 3회 확인했는지. **확인 중 두 번은 `STRAIGHT`를 반환한다.**

또한 실행 중인 일반 모션은 완료 전 새 회전 명령으로 교체하지 않는다. 그러므로 영상의 우회전 코너 검출이 바로 우회전 실행을 의미하지 않는다. 영상에서 확인되는 직접 경로는 **보정 좌회전 → 전진 6회 → 라인 소실 → 마지막 가까운 점 방향으로 좌회전 탐색**이다.

재현에서는 200초 표시값과 같은 heading +29.45°, offset −0.148, preview +59.3°를 넣고 미리보기가 신뢰된다고 가정했을 때 `STRAIGHT / turn_confirmation_pending` 두 번 뒤에 RIGHT가 나온다. 이는 회전 확정 대기 중에도 긴 전진 모션이 선택될 수 있는 구조를 보여 준다. 하지만 영상에는 정확한 명령 발행 시점의 `turn_consistency`, 입력 프레임, 후보 누적 횟수, decision reason이 없어, 당시 최초 전진을 선택한 조건이 이것이었다고 단정할 수는 없다.

공 잡기 실패 자체가 코너 RIGHT를 비활성화한 것은 아니다. 현재 `complete_post_ball_line_align()`은 GRASPED가 아니면 GOAL 경로 대신 AUTO로 돌아가며 일반 라인 코너 계획을 사용한다.

후속 제어 개선 후보: 코너 확인 대기에서 전진 6회가 바로 실행되는 조건을 줄이고, 충분히 확정한 코너 방향·거리 정보를 소실 직전까지 유지하는 방식이다. 소실 후 RIGHT를 무조건 우선하면 아직 코너에 도달하지 않은 상황에서도 돌아 버릴 수 있으므로, 저장 영상/합성 입력 재생으로 진입 시점과 초기화 조건부터 검증해야 한다.

## 4. 이번 표시 수정

파일: `src/step/step/yolo26_detector.py`

이전에는 실제 RIGHT/LEFT 명령도 실행 중 모션 alias 이름에 덮여 `LINE RETURN RIGHT 4` 또는 `LINE RETURN LEFT 4`로 보일 수 있었다. 이제 실제 코너 action과 해당 alias가 일치하면 실행 중 상단 배너를 `RIGHT` / `LEFT`로 유지한다. 가까운 허들 표시도 이 두 코너 배너를 덮지 않는다.

| 실제 명령 | 유지한 모션 alias | 실행 모션 | 상단 표시 |
|---|---|---|---|
| RIGHT | line_recovery_right_4 | 찐라인복귀우회전45도(4회) | RIGHT |
| LEFT | line_recovery_left_4 | 찐라인복귀좌회전45도(4회) | LEFT |
| RECOVER_RIGHT_TURN_RIGHT_4 | line_recovery_right_4 | 찐라인복귀우회전45도(4회) | LINE RETURN RIGHT 4 |
| LINE_LOST_TURN_LEFT | line_search_left_2 | 찐제자리좌회전45도-1(2회) | LINE SEARCH / IN-PLACE TURN LEFT 2 |

코너 미리보기만으로 실제 명령을 RIGHT로 표시하지 않는다. 일반 라인복귀·BALL 보정과 코너가 같은 alias를 공유하므로 실제 action을 함께 확인한다. 이번 코드는 화면 표시 변경이며 위의 원거리 공 정지나 코너 주행 판단 자체를 변경하지 않는다.

## 검증

- 기존 `src/step/test/test_yolo26_detector.py`: **83 passed**.
- 근사 공 박스 → 투영 범위 초과 → invalid_ball_alignment 재현.
- 오른쪽 곡선 확인 3회와 첫 두 STRAIGHT 재현.
- NEAR는 왼쪽, Corner는 RIGHT인 합성 입력의 LEFT 소실 탐색 재현.
- 실제 RIGHT/LEFT, 일반 복귀, BALL 보정, 소실 탐색, 불일치 alias의 배너 확인.
- 기존 라인복귀 좌우 매핑 유지 확인. 로봇 모션 실행 없음.

재현 코드: [reproduce.py](reproduce.py), 출력: [checks.txt](checks.txt).

실행 예시(프로젝트 루트):

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/src/step:$PWD/src/mission_control:$PYTHONPATH" python artifacts/20260929_021624_review/reproduce.py
```
