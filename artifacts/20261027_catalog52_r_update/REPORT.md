# catalog52 R 모션 적용 기록

R 원본 29개를 모두 반영했다. 기존 32개 모션을 대응 R 모션으로 교체하고 기존 반복 수 유지용 변형 8개를 포함해 총 88개가 됐다. 런타임 허용 오차는 88개 모두 5도다. PC 참조 카탈로그는 원본 completion 값을 보존한다.

원본 각도(4·5번 포함), 프레임 시간, 배속, 토크 값과 프레임 이름을 보존했다. 원본에 없는 전진 2/8회, 우회전 3회, 라인복귀 좌 6회는 해당 R 원본에서 이름과 repeat_count만 바꿨다. 합성 모션은 원본의 repeat_count=1을 유지한다.

`R제자리우회전(5회(`는 원본 이름 그대로 유지하고 5회 명령에 연결했다. 기존 우회전 3회 명령을 4회로 바꾸지 않았다.

R 원본에 대응하지 않는 모션과 자동 판단/회전각 선택 로직은 유지했다. 기존 자동 명령에 대응하지 않는 R 항목에는 아래 수동 ID를 추가했다. 예를 들어 기존 goal_forward_to_crab_right_90의 기본자세 전환 정책은 유지하고, 새 R 꽃게 전환은 r_forward_to_crab_90으로 별도 접근한다.

## 기존 명령 매핑 변경

| 명령 ID | 이전 | 적용 |
|---|---|---|
| `sdk_forward_4` | 전45도(4회) | R전진45도(4회) |
| `line_forward_2` | 전45도(2회) | R전진45도(2회) |
| `line_forward_4` | 전45도(4회) | R전진45도(4회) |
| `line_forward_6` | 전45도(6회) | R전진45도(6회) |
| `line_forward_8` | 전45도(8회) | R전진45도(8회) |
| `forward` | 전45도(8회) | R전진45도(8회) |
| `line_turn_right_2` | 찐제자리우회전45도-1(2회) | R제자리우회전(2회) |
| `line_turn_right_3` | 찐제자리우회전45도-1(3회) | R제자리우회전(3회) |
| `line_turn_right_5` | 찐제자리우회전45도-1(5회) | R제자리우회전(5회( |
| `line_turn_right_7` | 찐제자리우회전45도-1(7회) | R제자리우회전(7회) |
| `line_turn_right_9` | 찐제자리우회전45도(9회) | R제자리우회전(9회) |
| `line_turn_right_large` | 찐제자리우회전45도(9회) | R제자리우회전(9회) |
| `line_recovery_left_4` | 건라인복귀좌회전45도(4회) | R전진라인복귀좌 |
| `line_recovery_left_6` | 건라인복귀좌회전45도(6회) | R전진라인복귀좌(6회) |
| `line_recovery_right_4` | 건라인복귀우회전45도(4회) | R라인복귀우회전 |
| `line_search_right_5` | 찐제자리우회전45도-1(5회) | R제자리우회전(5회( |
| `stationary_turn_right` | 찐제자리우회전45도(9회) | R제자리우회전(9회) |
| `ball_camera_down_forward_2` | 전45도(2회) | R전진45도(2회) |
| `ball_camera_down_forward_4` | 전45도(4회) | R전진45도(4회) |
| `pickup_forward_to_crab_0` | 건오뒤에서 기본자세(0도) | R오뒤에서기본자세0도 |
| `pickup_first_backward_turn_right` | 찐후진하고 제자리우회전(공) | R후진에서제자리우회전(공) |
| `pickup_first_turn_right_7` | 찐제자리우회전45도-1(7회) | R제자리우회전(7회) |
| `pickup_first_turn_right_9` | 찐제자리우회전45도(9회) | R제자리우회전(9회) |
| `pickup_camera_down_turn_right_2` | 찐제자리우회전0도-1(2회) | R제자리우회전0도(2회) |
| `pickup_camera_down_turn_right_3` | 찐제자리우회전0도-1(3회) | R제자리우회전0도(3회) |
| `pickup_camera_down_turn_right_5` | 찐제자리우회전0도-1(5회) | R제자리우회전0도(5회) |
| `pickup_camera_down_turn_right_7` | 찐제자리우회전0도-1(7회) | R제자리우회전0도(7회) |
| `pickup_camera_down_turn_right_9` | 찐제자리우회전0도-1(9회) | R제자리우회전0도(9회) |
| `post_ball_forward_4` | 전45도(4회) | R전진45도(4회) |
| `post_ball_forward_6` | 전45도(6회) | R전진45도(6회) |
| `post_shot_turn_right_9` | 찐제자리우회전45도(9회) | R제자리우회전(9회) |
| `post_ball_forward_8` | 전45도(8회) | R전진45도(8회) |
| `post_ball_camera_90` | 건오뒤카메라90도 | R오뒤카메라90도 |
| `goal_camera_90_forward_2` | 전90도(2회) | R전진90도(2회) |
| `goal_camera_90_forward_4` | 전90도(4회) | R전진90도(4회) |
| `goal_camera_90_forward_6` | 전90도(6회) | R전진90도(6회) |
| `post_ball_line_turn_right_2` | 찐제자리우회전45도-1(2회) | R제자리우회전(2회) |
| `post_ball_line_turn_right_3` | 찐제자리우회전45도-1(3회) | R제자리우회전(3회) |
| `post_ball_line_turn_right_5` | 찐제자리우회전45도-1(5회) | R제자리우회전(5회( |
| `post_ball_line_turn_right_7` | 찐제자리우회전45도-1(7회) | R제자리우회전(7회) |
| `post_ball_line_turn_right_9` | 찐제자리우회전45도(9회) | R제자리우회전(9회) |
| `goal_camera_90_turn_right_2` | 찐제자리우회전90도-1(2회) | R제자리우회전90도(2회) |
| `goal_camera_90_turn_right_3` | 찐제자리우회전90도-1(3회) | R제자리우회전90도(3회) |
| `goal_camera_90_turn_right_5` | 찐제자리우회전90도-1(5회) | R제자리우회전90도(5회) |
| `goal_camera_90_turn_right_7` | 찐제자리우회전90도-1(7회) | R제자리우회전90도(7회) |
| `goal_camera_90_turn_right_9` | 찐제자리우회전90도-1(9회) | R제자리우회전90도(9회) |

## 추가 명령 ID

| ID | R 모션 |
|---|---|
| `r_forward_0_4` | R전진0도(4회) |
| `r_forward_0_6` | R전진0도(6회) |
| `r_turn_right_4` | R제자리우회전(4회) |
| `r_turn_right_90_4` | R제자리우회전90도(4회) |
| `r_turn_right_0_4` | R제자리우회전0도(4회) |
| `r_left_back_to_default_0` | R왼뒤에서 기본자세0도 |
| `r_back_pose_45` | R오뒤45도 |
| `r_forward_to_crab_90` | R오뒤에서오옆꽃게90도 |

## 검증

- 대상 검사 10개 통과: `tests.log`. 과거 고정 카탈로그를 참조하던 어깨 보존 검사는 현재 PC 참조와 전체 필드를 비교하도록 갱신했다.
- 설치 경로에서 startup 자세와 PC 정책 참조를 적용한 실제 플레이어 재생 88개 통과: `runtime_check.log`.
- 전체 모션/매핑/ID 0~22 구조 검사 통과.
- 실제 플레이어 정상/지연 tick의 목표값 723,212개 대조, raw 불일치 0: `playback_parity.json`.
- 실기 재생은 하지 않았다. 이전 전진 배속 0.9와 달리 새 R 전진은 원본 1.0이며 프레임 시간도 달라졌으므로, 시뮬레이션 확인 후 로봇을 지지한 상태에서 실제 보폭·회전량·착지 안정성을 확인해야 한다.

## 적용 및 복원

설치된 JSON/YAML은 수정 파일에 심볼릭 링크로 연결돼 있어 재빌드는 필요 없다. 현재 재생을 안전하게 끝낸 후 실행 노드를 재시작하면 새 데이터를 읽는다. 연속 보간/5번 제외 옵션은 이전 설정 그대로다.

원본은 `motion_imports/robot_motions(52).json`, 변경 내역은 `manifest.json`, 백업은 이 디렉터리의 `*.before.*`에 있다. 복원 시 후속 변경을 먼저 보관하고 노드를 종료한 뒤 저장소 루트에서 실행한다:

```bash
cp artifacts/20261027_catalog52_r_update/robot_motions_runtime.before.json artifacts/robot_motions_runtime.json
cp artifacts/20261027_catalog52_r_update/robot_motions_pc.before.json artifacts/robot_motions_pc.json
cp artifacts/20261027_catalog52_r_update/motion_aliases.before.yaml src/irc_step_motion_executor/config/motion_aliases.yaml
```

테스트 파일까지 되돌리려면 `test_upsert_motion_catalog.before.py`를 `tools/test_upsert_motion_catalog.py`로 복사하고 이번에 추가한 `src/mission_control/test/test_catalog52_r_update.py`를 제거한다.
