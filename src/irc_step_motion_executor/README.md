# IRC STEP Motion Executor C++ Wrapper

## 2026-10-02 공통 재생 경로 수정 및 검증

현재 production은 외부 SDK를 연결해 사용한다. 아래의 과거 catalog-only 단계
설명과 구분한다. 이번 작업은 모터를 연결하거나 launch를 실행하지 않았다.
사용자가 선택한 대로 **기존 runtime 82모션/293프레임을 그대로 유지**했다.
PC 데이터는 비교 및 override OFF 기준 파일로만 별도 설치한다.

### 기준 파일과 전체 차이

- branch: `agent/sdk-gui-frame-gates-20260930`
- HEAD: `0e6b0b486a00f751401dd762f80251555a7147f4` (commit/push 없음)
- 작업 시작 시 git 변경 없음. 잘못된 final-Goal/time-profile 변경도 없었다.
- 최초 host 프로세스 검사에서 실행 중인 full_system/모션 executor 없음.
- PC commit: `4bfdddc86ad34278b22b4f2f8e54edc5fd998192`
- PC GUI SHA256: `5f510f76fb1eb7bd2c29f959026994d5365141bdf8f1e1a941cdf90bf5ed3f25`
- PC JSON SHA256: `b4d228b865cef7f582bdb67f1fe5d20be4c82178a2553810a30fba2e48fd811e`
- 최초 비교 runtime SHA256: `f160ebc8f256f27c9ab699255398bfa53503f11d2424863e0fcbbe8df76c0ecb`
- 최신 runtime SHA256: `b537a60b167503fb7491a87c8106b698aacce9531e837ed28652ebdd167aad2d`

runtime source는 `artifacts/robot_motions_runtime.json`, install은
`install/irc_step_motion_executor/share/irc_step_motion_executor/config/robot_motions_runtime.json`이다.
install은 source로 연결된 symlink이며 둘 다 82/293 및 최신 runtime SHA가 같다.
최초 비교 이후 사용자가 바꾼 `찐라인복귀우회전45 도(4회)`의 `왼뒤407`
ID 10/11/12(-4/2/-2 → -3/0/0)는 보존했다. 이번 경계·시계 수정에서는
runtime JSON을 편집하지 않았다.
PC 파일은 commit의 raw URL에서 다운로드하여 전체 SHA를 검증한
`artifacts/20261002_common_playback/robot_motions_pc.json`이며 install의
`config/robot_motions_pc.json`도 같은 바이트를 참조한다(82/286).
최초 binary는 node source보다 새로웠다. 이번에는 외부 SDK와 wrapper를 다시
빌드했고 최종 binary hash/경로는 아래 verification 파일에 기록했다.

전체 84개 이름의 합집합, 모든 프레임/관절/metadata 차이는
[`catalog_diff.json`](../../artifacts/20261002_common_playback/catalog_diff.json),
최초 runtime 기준 행 단위 표 691개는 [`catalog_diff.csv`](../../artifacts/20261002_common_playback/catalog_diff.csv)에 있다.
변경 전 원본도 `robot_motions_runtime.before.json`에 보존했다.

| 구분 | 확인한 내용 | 처리 |
|---|---|---|
| 의도된 ID 0 정책 | 공통 대응 프레임의 JSON ID 0 값은 PC와 같음. ball/goal/hurdle ROS callback은 runtime head override를 설정함 | 기본 ON 유지 |
| 의도된 ID 4·5 정책 | 대응 프레임 245개에서 각 축 값이 다름. 일반 모션은 18/-18, 명시된 예외 3개는 원래 값 | 기본 ON 유지 |
| 기존 completion 정책 | 공통 80모션 모두 PC 2도, Jetson 5도. settle 설정은 동일 | artifacts 정책대로 5도 유지 |
| 이름 집합 | 양쪽 모두 82개지만 서로 다른 이름이 각각 2개 | 기존 이름 보존 |
| 비정책 관절 | 라인 복귀 ID 10/11/12, 우회전 ID 15/18, 좌회전90·카메라90 ID 17/19, 자세전환 ID 15, 허들 ID 15 등 | 기존 튜닝 보존 |
| 시간·구조 | 좌회전 복합 모션 추가 프레임, 허들 삭제·시각 변경, 0도 자세전환 speed 1.0→1.05 | 기존 데이터 보존 |
| torques | 카메라90/좌회전90의 `오뒤412(90도)` 7프레임에서 PC true, Jetson false | 재생 중 Torque Enable에 사용하지 않음 |
| 기타 | 공통 모션의 repeat_count/repeatable/start_pose, 대응 프레임 lift_early_arrival/playback_cycle_end는 동일. 허들의 end_pose/마지막 frame name은 다름 | 상세 원본 차이 보존 |

비정책 관절의 PC→Jetson 차이는 다음과 같다. 따라서 ID 0·4·5를 제외해도
데이터 차이가 남는다. 이것이 오류라는 뜻은 아니며, 작업자의 추가 의도 확인 없이 고치지 않았다.

| 모션/프레임 | PC → Jetson |
|---|---|
| 좌/우 라인복귀 4회 `왼뒤407` | ID 10: -3→-4, 11: 0→2, 12: 0→-2 |
| 우회전45 2/3/5/7/9회 및 우회전 복합 모션 `제우오들25` | ID 15: -3.548828125→-4, 18: 19→18.80859375 |
| 좌회전90 1~6회 및 `찐오뒤카메라90도`의 `오뒤412(90도)` | ID 17: -24→-26, 19: 67.115234375→68.115234375 |
| `찐오뒤에서 기본자세(카메라45도)` 첫 프레임 | ID 15: -3→-2 |
| `찐허들`의 `허들찐12` / `왼뒤407` | ID 15: -6→0.703125 / ID 10·11·12는 라인복귀와 같은 차이 |

순증 7프레임은 단순한 7개 삽입이 아니다. 아래 합계는 **+4+6-1+4+2-4-4=+7**이다.
시각 표기는 `start_ms/time_ms`, 프레임 index는 JSON 차이표에서 0부터 센다.

| 모션 | PC / Jetson 프레임 수 | 정확한 위치·내용 |
|---|---|---|
| `찐기본자세에서 제자리좌회전(골대)` | 10 / 14 | Jetson index 10~13: `제좌왼들25` 2206/61, `오뒤412` 2267/144, `제좌왼들25` 2545/61, `오뒤412` 2606/144 |
| `찐후진에서 제자리좌회전(공)` | 10 / 16 | 위 4개 + index 14~15 `제좌왼들25` 2884/61, `오뒤412` 2945/144 |
| `찐허들` | 17 / 16 | PC index 13 `허들찐13` 5810/70 삭제. 뒤 `왼뒤407` 5880/300→5810/100, `왼들401` 6818/60→6614/55, 마지막 `오뒤412` 6878/120→`오뒤412(허들)` 6669/120. 마지막 프레임의 비정책 관절은 동일 |
| `찐찐라인복귀좌회전45도(6회)` | 0 / 4 | Jetson 전용: `오들401` 60/70, `왼뒤407` 130/150, `왼들401` 430/45, `오뒤412` 475/90 |
| `찐미세오뒤에서 오뒤(골대 카메라90도)` | 0 / 2 | Jetson 전용: `왼들401(회전준비90도)` 50/67, `오뒤412(회전준비90도)` 117/118 |
| `찐미세오옆꽃게90-1(2회)` | 4 / 0 | PC 전용: `미세오옆꽃게1-1` 80/60, `미세오옆꽃게2` 140/140, `미세오옆꽃게3` 330/60, `오옆4` 390/100 |
| `찐미세오옆꽃게90-1(3회)` | 4 / 0 | 위와 같은 4개 프레임, repeat_count=3 |

### 이름 계약 및 요청 정리

등록 alias 96개의 target은 모두 현재 runtime에서 해석된다. 누락되었던
`line_forward_8`과 `forward`는 기존 8회 전진 의미대로 `찐찐전진45(8회)`,
`goal_camera_90_forward_2`는 `찐찐전진90(2회)`에 연결했다.

사용자 지시에 따라 `line_forward_10`과 이를 요청하는 `STRAIGHT_5`를 제거했다.
bridge, 일반 명령 gate, 라인 관측 시간표, head override 대상에서 제거했으며
거리 분류도 더 이상 STRAIGHT_5를 생성하지 않는다. 0.680m를 초과하는 거리는
기존 범위 밖 처리인 일반 STRAIGHT로 판단한다. 제거된 요청은 다른 모션으로
대체 실행하지 않고 거절한다. 사용되지 않던 `pickup_crab_prepare` 상수와
상태 분기도 제거했다. 실제 미세전진→꽃게 준비 모션은 보존했다.

전체 action map과 모션 상수의 alias 누락 검사가 통과한다.
PC 원본으로 교체하면 여전히 `line_recovery_left_6`와 `fine_to_turn_ready_90`의
대상이 없으므로, 사용자가 선택한 runtime 82모션/293프레임을 유지한다.
전체 alias target과 정적 참조 위치는 `catalog_diff.json`에 기록했다.

### 공 집기 오류 후 진행 정책

SDK의 Goal 송신 실패는 FAILED로 남으며 해당 모션과 SDK queue를 종료한다.
bridge는 그 결과를 SUCCEEDED로 덮어쓰지 않는다. 기존에 허용하던 네 종류의
모터 오류에 대해서는 실패 내역을 `/motion/status`의 `motor_failures` 배열에
남기고 sequence의 다음 단계를 별도로 진행한다. 각 항목은 motion_id,
status=FAILED, error_code, message를 보존한다. 기존 준비 모션 예외와
critical 오류의 AUTO 잠금 정책은 유지한다.

별도의 복구 ping, 대기 또는 도착 gate는 추가하지 않았다. 다음 단독 모션은
기존 SDK start의 profile 설정, Present Position 읽기, Goal 송신 검사를 사용한다.
RUNNING 수신이나 dwell 종료는 통신 회복 성공으로 세지 않는다. 다음 모션도
실패하여 **연속 2개 모션이 실패하면** 현재 pickup sequence를 FAILED로 끝낸다.
이는 이번 수정의 미션 정책이며 SDK 담당자 프롬프트 자체의 요구는 아니다.
성공한 모션이 사이에 있으면 연속 실패 횟수는 초기화한다. 전체 AUTO에 새로운
영구 잠금을 추가하지 않으며 이후 판단은 기존 BALL_APPROACH 재시도 정책을 따른다.

모션 실행 실패는 자세 완료 이력에 기록하지 않으며, 집기 확인 자세가 실패하면
영상 검증 창도 열지 않는다. 이때 공 획득 결과는 UNKNOWN으로 남고 후퇴·복귀를
계속할 수 있다. 정상 검증에서 NOT_GRABBED가 나온 경우도 기존대로 라인 정렬 후
AUTO를 재개한다. sequence 끝의 SUCCEEDED는 묶음 처리가 끝났다는 뜻이며,
생략된 모션의 실패 내역은 종료 상태에도 남는다. 공 획득 성공을 뜻하지 않는다.

### 적용 경로와 Goal writer

| 단계 | 코드 위치 |
|---|---|
| vision/algorithm decision → navigation publish | `mission_control/motion_decision_node.py:3521` |
| bridge 및 sequence/ready gate → motion request | `motion_command_bridge_node.py:1051`, `:1107` |
| request/alias resolve/queue | `src/sdk_executor_driver.cpp:20`, `src/sdk_executor_core.cpp:194` |
| 5ms timer → core poll → SDK update | `src/sdk_executor_node.cpp:250`, `src/sdk_executor_driver.cpp:40`, `src/robot_motion_player_backend.cpp:266` |
| JSON lookup, actual-time timeline, endpoint 보장 | 외부 SDK `robot_motion_player.cpp:523` |
| half-cosine + shortest-angle + lift 80% | 외부 SDK `robot_motion_player.cpp:978`, `:959`, `:990` |
| ID 0·4·5 + correction/override 합성 | 외부 SDK `robot_motion_player.cpp:923` |
| direct Goal 합성 → hardware | 외부 SDK `robot_motion_player.cpp:1033`, `dynamixel_motion_hardware.cpp:144` |
| degree/radian/raw → 한 번의 GroupSyncWrite | 외부 SDK `dynamixel_controller.cpp:72`, `gui_goal_position.hpp:10`, `step_dynamixel.cpp:549` |

외부 SDK 경로는 `/home/jet/IRC/external_sdk/robot_motion_player_sdk_work_20260801/final step`이다.
23축 degree map을 하드웨어의 desired_rad에 합쳐 `SetPosition`으로 전송하고,
raw 변환은 `2048 + degree*4096/360`에 GUI의 wrap/clamp/ties-to-even을 적용한다.

full_system의 일반 Goal writer는 이 executor 하나다. vision은 ROS 정보만
보내며 head callback은 같은 player의 override map을 갱신한다. head용 별도
SyncWrite는 없다. `tools/upsert_motion_catalog.py:21`은 import 시 JSON의
4·5 및 completion 정책을 적용하는 도구이지 bus writer가 아니다.

같은 hardware owner의 startup transition(`robot_motion_player.cpp:363`)과
cancel hold(`dynamixel_motion_hardware.cpp:216`)는 별도 상황에서
`commandPosition → SetTimeBasedPosition → syncWriteTimeBasedTheta`를 사용한다.
일반 재생의 profile 0 설정과 혼동하지 않는다. SDK의 legacy
`MoveToTargetSmoothCos`도 Goal Position write를 호출할 수 있으나 현재
full_system 호출 경로에는 없다. `Dxl::Loop`의 선택적 write는 Goal Torque이며
일반 모션 재생 경로에서 호출하지 않는다. 별도 실행되는 임의 프로그램까지 막는
OS 수준의 port lock을 이번에 추가한 것은 아니다.

### 변경한 공통 동작과 영향

| 항목 | 이전 | 변경 후 |
|---|---|---|
| 프레임 경계 | endpoint 후 같은 callback에서 다음 sample 가능 | endpoint 성공을 확인하고 callback 종료. 다음 callback부터 진행 |
| 여러 경계를 넘긴 tick | 샘플과 경계 명령 중첩 가능 | 밀린 endpoint만 callback당 하나. 놓친 5ms sample 몰아쓰기 없음 |
| queue | 마지막 sample과 queued activation이 같은 callback일 수 있음 | 마지막 final 성공 이후 callback에서만 activation. 첫 queued Goal은 그 다음 callback |
| ROS queue 알림 | RUNNING 알림 때문에 SDK update 한 번 생략 | 알림 callback에서도 SDK poll 유지. 오류를 즉시 보고 |
| 실패 | 전송 실패 후 timeline 진행 / 최종 read 실패도 best effort 성공 가능 | Goal 실패 또는 read 실패는 FAILED, queue 해제 |
| 위치 도착 | 마지막 모션의 최대 100ms best effort 확인 | 일반 재생 기본값 false. 최종 Goal 성공 시 도착 검사 없이 완료. 시작 자세 검사는 별도 유지 |
| 반복 시계 | 마지막 Goal 다음 callback에서 재설정 | 최종 송신을 마친 callback에서 새 시각으로 재설정. 다음 callback부터 경과시간 반영 |
| 내부 반복 | playback_cycle_end를 실행 경계로 사용하지 않음 | 내부 종점 성공 후 시계 재설정, 초과 지연 버림. 마지막 플래그는 전체 종료·반복 경로만 사용 |
| 시작 자세 | 단독 실행 PP read 실패 시 과거 Goal fallback 가능. queue에서 PP 재읽기 | 단독 시작은 PP read 필수. 반복/queue는 직전 성공한 final Goal 사용 |
| lift | 이름만 검사, 최소 duration 10ms | flag=true **그리고** 키워드일 때 80%, 최소 1ms. 현재 catalog에서 실효 곡선 차이는 없음 |
| 누락 관절 | 일부 sample이 partial map | 이전 자세를 유지한 full map 합성 |

`playback_speed`는 elapsed timeline에 한 번만 곱한다. 반복 끝은 마지막
`start_ms+time_ms`이며 `max_seq_ms`가 아니다. JSON `torques`는 하드웨어
Torque 전환에 사용하지 않는다. `playback_cycle_end`는 단독 재생에도 적용하는
시간 경계이며 관절 도착 조건이 아니다.

최신 SDK 담당자 답변에 따라 PC 목표각·경계·반복 시계를 따르되, callback당
Goal SyncWrite 최대 1회는 Jetson의 명시적 정책으로 유지한다. PC는 경계에서
종점과 경계 sample을 두 번 보낼 수 있다. Jetson은 인접 부분 관절 프레임까지
합성해 한 번 보낸다. 일반 경계는 기존 시계를 유지하고, 내부·전체 반복은
성공한 송신 처리가 끝난 뒤 새 시각으로 시계를 갱신한다. 송신 실패 시 갱신하지 않는다.
PC의 관측용 100Hz 피드백은 추가하지 않았다. 계산 규칙의 일치가 실제 버스 부하나
송신 간격의 일치를 뜻하지는 않는다.
위 순서 보장은 실제 관절 도착 보장이 아니다. `SyncWrite::txPacket` 성공은
송신 측 성공이며 각 모터의 수신·이동 또는 hardware shutdown 해제를 보장하지 않는다.
Hardware Error Status/Shutdown 레지스터는 현재 재생 경로에서 polling하지 않는다.
일반 실패 시 자동 Torque OFF를 추가하지 않았다. 기존 emergencyStop/shutdown의
Torque OFF 경로는 유지하며, 이번 테스트에서는 fake hardware만 호출했다.

수정은 외부 SDK 6개 파일과 ROS wrapper/launch에 걸친다. SDK 변경 전체는
[`sdk_common_playback.patch`](../../tools/sdk_common_playback.patch), 전후 hash는
[`sdk_common_playback_manifest.json`](../../tools/sdk_common_playback_manifest.json)에 있다.
`tools/apply_common_playback_sdk.py --sdk '.../final step'`는 읽기 검증만 하고,
`--apply`일 때만 정확한 원본 또는 직전 적용본 hash에 패치를 적용한다.
이번 답변에 따른 추가 변경만 담은 파일은
[`sdk_common_playback_pc_boundary.patch`](../../tools/sdk_common_playback_pc_boundary.patch)다.
다른 변경이 있으면 거절한다.
실제 설정된 SDK에는 최종 패치를 적용했고 해당 SDK로 빌드했다.

### 옵션 및 나중의 A/B 시험

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| enable_head_override | true | false이면 PC 원본 ID 0 사용, head ROS override 차단 |
| enable_shoulder_override | true | false이면 PC 원본 ID 4·5 사용 |
| policy_reference_json_path | install/config/robot_motions_pc.json | OFF일 때 참조하는 검증된 PC 원본 |
| queued_transition_hold_ms | 0 | 성공한 마지막 Goal 이후 activation까지 최소 대기시간 |
| position_tolerance_enabled | false | 일반 모션 종료 검사만 제어. 시작 자세 위치 검사·안정화·AUTO 전 대기에는 영향 없음 |
| motion_trace_enabled | false | bounded trace를 종료 후 비동기 파일 기록 |
| motion_trace_path | 빈 문자열 | trace 활성화 시 필수 출력 경로 |
| motion_trace_goals | false | 상세 23축 degree Goal 추가 |

일반 모션의 shoulder ON 정책은 18/-18이며 예외는 정확히
`찐공잡기리그랩까지 실전`, `찐골넣기`, `찐허들` 세 이름이다.
head ON은 기존 JSON과 ball/goal/hurdle runtime policy를 유지한다.
OFF일 때는 frame_id 또는 name+start_ms로 PC 프레임을 확인하고 해당 축만
복원한다. 매칭 불가·모호한 데이터는 추측하지 않고 실행 전 거절한다.
현재 원본이 없는 위 Jetson 전용 2모션은 OFF 시험을 할 수 없다.
따라서 전체 경기 실행을 OFF로 바꾸는 대신 PC 대응이 있는 대표 모션을 사용한다.
startup pose에도 같은 ON/OFF 축 선택을 적용하며, catalog 오류는 hardware
backend 생성 전에 검출한다(`startup_pose_catalog.cpp:89`, `sdk_executor_node.cpp:172`).

나중에 로봇을 지지한 상태에서 동일 시작 자세·전원·모션으로 기본 ON/ON을
기록한 뒤, head와 shoulder를 각각 하나씩 OFF로 바꿔 비교한다. 그 다음
정책을 고정하고 queue hold만 0/20/50/100ms로 바꾼다. 이번에는 실시하지 않았다.
0ms도 다음 callback에서 activation하며 첫 Goal은 한 callback 더 뒤다.
5ms 주기가 정확하다면 final→activation→첫 Goal은 약 5ms→5ms이다.
hold 값은 최솟값이며 timer jitter와 송신 시간만큼 길어질 수 있다.

### Timer 및 telemetry 해석

`sdk_executor_node.cpp:673`의 `rclcpp::spin`은 single-thread executor다.
head/status/request callback, startup/PP read, blocking SyncWrite가 timer를
지연시킬 수 있다. vision은 별도 process이고 자체 `MultiThreadedExecutor(4)`를
사용한다(`step/unified_vision_node.py:38`). vision callback 자체가 같은 executor에
올라가는 구조는 아니지만 CPU/USB/OS scheduling 부하는 공유한다.
실제 5ms 주기, 송신 간격, baud 또는 firmware 상태는 실기로 측정하지 않았다.

파일 I/O는 선택한 trace 파일을 초기화할 때와 worker thread에서만 수행한다.
제어 루프는 기본적으로 카운터만, trace를 켜면 최대 8192 tick을 메모리에
저장한다. 이후 tick은 dropped_trace로 집계하며 worker queue도 최대 4 batch다.
기본 trace는 비활성화. 모션 종료 요약은 한 번 출력한다.
긴 모션의 전체 tick이 필요하면 trace 한도를 고려해야 한다.

trace는 CSV row type으로 구분한다. `summary` 열은 motion, ticks,
expected_tick_ms, mean/max_interval_ms, over_7.5/10/20 counts,
final_corrections, goal_failures, queue_delay_ms, elapsed_ms, dropped_trace이다.
`tick` 열은 motion, monotonic_ms, expected_tick_ms, actual_interval_ms,
repeat, frame_index, frame_name, timeline_ms, eased_progress, final_goal,
queued, queue_activated, queue_delay_ms, sent, success, write_ms,
write_started_ms, goal_interval_ms, motor_count, head_override,
shoulder_override, [옵션 Goal 0..22] 순서다. 값이 없는 Goal은 NaN이다.
head_override는 동적 ID 0 override 활성 상태, shoulder_override는
예외를 제외한 shoulder 정책 활성 상태다. elapsed/queue delay와 progress는
제어 clock, write_started/goal_interval/write_ms는 실제 steady clock을 사용한다.

### 오프라인 검증 결과와 한계

- `irc_step_motion_executor`, `mission_control`, `step` 빌드 성공. launch/full_system 실행 없음.
- `test_common_playback`: 전진/후진/좌우회전/꽃게/공/복구/허들을 포함한 **82개 전체** fake 실행 성공.
  half-cosine, shortest-angle, lift flag, speed 1회, max_seq 무시, 늦은 경계,
  frame final/queue 실패, hold 0/20/50/100, repeat, gap, 누락 관절, cancel/emergency,
  4개 ON/OFF 조합, bounded trace 및 비동기 flush를 검증했다.
- `찐찐전진45(4회)`는 repeat=4, speed=1.05, 실제 주기=555ms,
  프레임 시각 80/68, 148/132, 360/70, 430/125를 고정 회귀 벡터로 검사했다.
- `test_queue_control_tick`: 전환 알림 callback에서도 backend poll, 즉시 실패 전달 2개 통과.
- startup policy, runtime config, backend/factory, parameter type, JSON validator 및
  SDK CMake guard/mock build 검사 통과.
- CTest 20종 검사 중 첫 실행 19종 통과. 마지막 옛 허들 이름 기대값을 현재
  alias로 수정한 뒤 해당 executor core 검사를 재실행해 **20종 모두 통과**했다.
  launch 5종은 실행하지 않았다. 기존 4종 실패의 원인이던 누락 forward alias와
  오래된 모션 이름 기대값을 해결했다.
- Python bridge/gate/decision/safety/phase 및 거리·개별 planner 검사 **1,226개 통과**.
  단발성 오류 후 계속 진행, 연속 2회 실패 중단, 중간 성공 시 횟수 초기화,
  실패한 집기 확인 자세의 UNKNOWN 보존, 종료 실패 기록을 검증했다.
- 현재 alias 계약/target 존재/5도 completion 검사 3개와 import 정책 검사 5개 통과.
- 미션 전체 흐름 검사 214개 중 193개 통과, **21개는 수정 전 HEAD에서도 같은
  이름으로 실패**한다. `/tmp`의 독립 HEAD 소스로 재현했고 새 실패는 없다.
  `phase_flow_baseline.json`에 전후 결과를 남겼으며, 이번 작업에서 별도 골대·허들
  판단 정책을 바꿔 기존 실패를 맞추지는 않았다.
- `baseline_test_failures.json`은 최초 C++ 실패 이력,
  `verification.json`은 최초 공통 재생 수정 당시의 검사 요약이다.
  SDK 담당자의 경계·반복 시계 정정을 반영한 최신 검증은
  [`verification_pc_boundary.json`](../../artifacts/20261002_common_playback/verification_pc_boundary.json)에 기록한다.
  추가 가상 시계 검사는 200ms 종료 후 205ms의 19.9384417도/raw 2275,
  8ms 송신 지연 후 208ms 재시작, 내부 경계 125→130ms의 timeline 105ms,
  일반 경계의 timeline 130ms, 배속, 부분 관절 경계, 송신 실패,
  일반 종료 검사 생략과 시작 자세 검사·80ms 안정화 유지를 포함한다.
  이번 재검증은 3개 패키지 빌드, CTest 20종, Python 1,241개 및 catalog 계약
  3개가 통과했다. Python에는 노드를 실행하지 않는 launch 설정 검사 10개도 포함한다.

관절 도착/전류/과열/전압 저하/모터 shutdown, 실제 5ms timer jitter와
실제 Goal 송신 간격은 아직 미측정이다. 공통 코드의 명령 순서 보장을
실제 걷기 안정성 검증과 동일시하지 않는다. 오류 후 다음 모션을 시작할 수
있는 실제 자세인지, shutdown된 모터가 있는지는 이 오프라인 검사로 판정할 수 없다.

## Production startup pose contract

`full_system_robot.launch.py`는 fail-closed startup gate를 기본 활성화한다.
`RobotMotionPlayer::initialize()`가 끝난 뒤 executor는 `motion_json_path`에서
`startup_pose_name`과 정확히 일치하는 유일한 frame을 찾아 motor ID 0..22의
유한한 각도 23개를 읽는다. 이어 production SDK 확장 API
`startPoseTransition(const std::vector<double>&, int64_t)`를 호출하고
`updateStartupPose()`가 `Succeeded`를 반환할 때까지 poll한다.

SDK 구현은 `DynamixelMotionHardware::initialize()`에서 저장한 Present Position을
시작 벡터로 사용하고, 기존 time-based/shortest-angle 처리와 hardware write를
재사용해야 한다. JSON/pose 오류, API 미지원, write 실패, settling timeout은
모두 AUTO gate를 영구 잠금 상태로 유지하며 STRAIGHT로 fallback하지 않는다.

이 패키지는 전달받은 C++ `RobotMotionPlayer` SDK를 향후 ROS 2에 연결하기 위한
독립 `ament_cmake` wrapper의 최소 골격이다. 현재 단계는
**catalog-only/mock-safe**이며 production motion executor가 아니다.

## 현재 동작

- `/motion/executor/request`, `/motion/executor/cancel`을
  `std_msgs/msg/String` JSON으로 구독한다.
- `/motion/executor/status`에 기존 JSON String 계약의 필드를 발행한다.
- `config/motion_aliases.yaml`을 읽어 motion alias 존재 여부만 검증한다.
- 알려진 alias의 start 요청도 `REJECTED` / `HARDWARE_NOT_READY`로 반환한다.
- 알려지지 않은 motion은 `REJECTED` / `INVALID_MOTION`으로 반환하며 다른
  motion으로 fallback하지 않는다.
- `RobotMotionPlayer`, Dynamixel backend 및 hardware 객체를 생성하지 않는다.

최신 SDK의 top-level motion 10개는 `sdk_*` alias로 각각 수동 시험할 수 있다.
production canonical alias는 현재 확인된 `forward`, `pickup`, `hurdle`만 둔다.
짧은 전진, shoot, 좌·우 회전 canonical alias는 실물 검증 전이므로 만들지
않았으며, 알 수 없는 motion으로 fallback하지 않는다.

## 단일 motion smoke test

executor만 안전한 simulated backend로 시작한다. 이 명령 자체로 motion request가
발행되지는 않는다.

```bash
ros2 launch irc_step_motion_executor sdk_motion_test.launch.py
```

다른 터미널에서 alias 하나를 반드시 지정해 정확히 한 번 요청한다. 기본 timeout은
15초이며 긴 반복 motion은 사용자가 명시적으로 늘린다.

```bash
ros2 run irc_step_motion_executor manual_motion_request.py --ros-args \
  -p motion_id:=sdk_pickup -p timeout_ms:=15000
```

helper는 자신의 `request_id`에 해당하는 `RUNNING`, `SUCCEEDED`, `FAILED`,
`CANCELLED`, `REJECTED`와 `error_code`/`message`를 출력한다. 전체 catalog를
순차 실행하거나 프로그램 시작만으로 자동 재생하지 않는다.

실물에서는 로봇 지지, 비상정지 수단, 관절 영점·방향·limit를 먼저 확인하고
SDK-enabled 빌드에서만 다음 값을 모두 명시한다. 아래 명령은 torque ON과 실제
동작을 허용하므로 반드시 개별 motion을 simulated로 먼저 검증해야 한다.

```bash
ros2 launch irc_step_motion_executor sdk_motion_test.launch.py \
  backend_type:=robot_motion_player \
  enable_robot_hardware:=true explicit_torque_approval:=true \
  motion_json_path:=/approved/sdk/robot_motions.json \
  robot_device_path:=/dev/ttyUSB0 robot_baud_rate:=4000000 \
  robot_motor_ids:="[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22]"
```

실행 중 취소는 기존 executor API만 사용한다. helper가 출력한 `request_id`와
동일한 값이어야 하며, 이것은 별도의 emergency stop을 의미하지 않는다.

```bash
ros2 topic pub --once /motion/executor/cancel std_msgs/msg/String \
  "{data: '{\"request_id\": 123456789}'}"
```

## SDK JSON/catalog 사전 검증

다음 검사는 serial, SDK, torque 또는 모터에 접근하지 않는다.

```bash
ros2 run irc_step_motion_executor validate_motion_catalog.py \
  /approved/sdk/robot_motions.json \
  --aliases install/irc_step_motion_executor/share/irc_step_motion_executor/config/motion_aliases.yaml
```

JSON의 top-level `motions`, 중복 motion 이름, 각 alias target, 모든 frame의
angles/torques motor ID `0..22`, 그리고 motion별 `completion`, `start_pose`,
`end_pose` 존재 여부를 검사한다.

## JSON 계약

request는 `action`(string), `command_id`(integer 또는 null),
`event_id`(integer 또는 null), `request_id`(integer),
`motion_id`(string)를 필수 key로 사용한다. 특히 `event_id: null`은 유효하지만
`event_id` key 누락은 `REJECTED` / `INVALID_REQUEST`이다.

status는 `status`, `action`, `command_id`, `event_id`, `request_id`,
`motion_id`, `error_code`, `message`를 항상 포함한다. catalog-only core는
정상 계약 요청의 원래 action과 correlation 값을 terminal `REJECTED`
status까지 보존한다.

## Hardware-independent executor core

`MotionBackend`는 실제 SDK 연결 전 단계의 하드웨어 독립 계약이다.
`start_motion()`, `cancel_motion()`, `poll_status()`만 정의하며 실제 SDK
함수명이나 생성자 정보를 포함하지 않는다. `SdkExecutorCore`는 기존 JSON
request를 검증하고 alias를 resolve한 뒤 이 인터페이스를 통해서만 motion
상태를 처리한다.

core 단위 검증은 test 전용 `FakeMotionBackend`만 사용한다. 실제
`RobotMotionPlayer` adapter와 hardware executor node는 아직 구현하지 않았으며,
core library는 `robot_control` target에 링크하지 않는다. 이 core의 빌드와
단위 테스트 통과는 실물 동작 또는 안전성 검증을 의미하지 않는다.

## Simulated executor node

`sdk_motion_executor`는 현재 `SimulatedMotionBackend`만 사용하는 하드웨어 없는
ROS 2 node이다. 실제 SDK backend는 아직 연결되어 있지 않다.

- subscribe: `/motion/executor/request`
- subscribe: `/motion/executor/cancel`
- publish: `/motion/executor/status`
- parameters: `backend_type`(기본 `simulated`),
  `enable_robot_hardware`(기본 `false`), `motion_json_path`(기본 빈 값),
  `motion_aliases_file`,
  `poll_period_ms`(기본 5; production trajectory control 200Hz),
  `running_polls`(기본 2),
  `settling_polls`(기본 1), `force_start_failure`, `force_backend_failure`

```bash
ros2 launch irc_step_motion_executor sdk_executor_simulated.launch.py
```

request는 즉시 status를 만들고 timer poll은 simulated 상태를
`RUNNING → SETTLING → SUCCEEDED`로 진행한다. 외부 status에서 `SETTLING`은
message에 settling을 표시한 `RUNNING`으로 발행된다. cancel은 다음 poll에서
`CANCELLED`가 된다. 전이는 sleep이나 장치 시간이 아닌 poll 횟수만 사용한다.

이 launch에는 실물 로봇이나 serial 장치를 연결하지 않는다. simulated
topic test 통과 역시 실물 안전성 검증을 의미하지 않는다.

`MotionBackend` 생성은 node가 아니라 factory가 담당한다. parameter를
생략하거나 `backend_type=simulated`를 지정하면 simulated backend만 생성되며
실제 motion을 실행하지 않는다. `robot_motion_player`를 요청하면 simulated로
fallback하지 않고 guard 순서에 따라 node 시작을 중단한다.

- `enable_robot_hardware=false`: `ROBOT_HARDWARE_NOT_ENABLED`
- hardware enabled + SDK OFF: `ROBOT_MOTION_PLAYER_BACKEND_NOT_BUILT`
- hardware enabled + SDK ON + runtime factory를 주입하지 않음:
  `ROBOT_MOTION_PLAYER_RUNTIME_NOT_CONFIGURED`

SDK adapter와 production runtime factory target은 존재하지만 node runtime
wiring은 아직 비활성 상태다. 지원하지 않는 backend 이름도 허용값을 포함한
`UNSUPPORTED_BACKEND_TYPE` 오류로 거부한다.

Real runtime은 아직 비활성이다. `backend_type=robot_motion_player`만
지정하면 `ROBOT_HARDWARE_NOT_ENABLED`로 안전하게 시작을 거부한다.
향후 real runtime에는 `enable_robot_hardware=true`와 명시적인
`motion_json_path`가 모두 필요하지만, 기본 node에는 production runtime factory가
연결되지 않았다. real 요청은
simulated로 fallback하지 않는다.

승인된 SDK 작업 복사본에서는 `Dxl` constructor의 port, baud rate, torque 및
operating-mode 접근을 제거하고 명시적인 `Dxl::Initialize()` 단계로 옮겼다.
SDK ON production factory는 `DynamixelMotionHardware`, 주입형 2인자
`RobotMotionPlayer`, borrowed API와 backend 객체를 생성하고 소유권만 구성한다.
검증된 ROS 2 runtime config의 device path, baud rate와 motor IDs는 SDK의
`DynamixelMotionHardwareConfig`로 변환되어 hardware 생성자까지 전달된다.
Production factory는 hardware policy가 모두 통과한 경우에만 owner를 만들고
`RobotMotionPlayer::initialize()`를 정확히 한 번 호출한다. 성공한 경우에만
backend를 반환하며 자동 simulated fallback은 없다. Production backend 선택은
실제 serial port, operating mode 및 torque 접근을 일으킬 수 있다.

Production factory의 `preflight()`는 진단 전용 경로다. fixed SDK hardware
profile을 검증한 뒤 torque-OFF 통신 및 모터 응답 확인만 수행한다.
motion JSON을 읽거나 `RobotMotionPlayer`를 생성하지 않으며, explicit torque
approval를 요구하지 않고 backend도 반환하지 않는다.
반면 `create()`와 그 내부 initialize 경로는 explicit torque approval가 반드시
필요하고, torque ON까지 성공한 경우에만 motion backend를 반환한다. 따라서
preflight 성공은 motion-ready 또는 모션 실행 가능 상태를 뜻하지 않는다.

SDK-enabled 빌드의 `sdk_hardware_preflight`는 일반
`sdk_motion_executor`와 분리된 일회성 diagnostics CLI다. ROS 2 node, topic 또는
지속 실행 loop를 만들지 않으며 `--device`, `--baud`, `--motor-ids`와
`--confirm-hardware-access PREFLIGHT_ONLY_TORQUE_OFF`를 모두 명시해야 한다.
이 확인 문자열은 하드웨어 포트 접근만 승인하며 torque ON 승인이 아니다.
CLI는 `explicit_torque_approval=false`를 유지하고 `initialize()`나 모션 실행을
호출하지 않는다. 성공 메시지가 출력되어도 torque는 OFF로 유지되며
motion-ready 상태가 아니다. 이 도구의 실제 실행은 로봇 지지·비상정지 등
하드웨어 점검 절차를 준비하고 사용자가 연결 상태를 판단한 뒤에만 수행해야 한다.

Production runtime 객체 생성과 hardware initialization 승인은 서로 다른
단계다. 초기화 policy의 기본값은 `enable_robot_hardware=false`이며, JSON 경로,
device path, baud rate, motor ID 목록·중복·범위와 explicit torque approval를
모두 검증한다. validation 성공은 설정 사전조건이 충족됐다는 뜻일 뿐 실제
모터 초기화나 torque enable을 의미하지 않는다. 실제 initialize 호출 구현은
후속 단계에서 별도의 hardware 승인과 함께 추가해야 한다.

`sdk_motion_executor`는 같은 ROS parameter 경로에서 다음 값을 읽어 runtime
config에 복사한다.

- `enable_robot_hardware`: `false`
- `robot_device_path`: 빈 문자열
- `robot_baud_rate`: `0`
- `robot_motor_ids`: 빈 정수 배열
- `explicit_torque_approval`: `false`
- `motion_json_path`: 빈 문자열

각 기본값은 hardware initialization을 허용하지 않는다. 기본 backend도 계속
`simulated`이며, simulated 선택에서는 이 parameter를 지정해도 hardware
policy나 initialize를 실행하지 않는다. ROS parameter는 bool, string, integer,
integer array 타입으로 선언되어 잘못된 타입은 node 구성 단계에서 거부된다.

현재 외부 SDK는 device `/dev/ttyUSB0`, baud rate `4000000`, motor ID `0..22`를
legacy 기본 profile로 사용한다. runtime config의 `device_path`, `baud_rate`,
`motor_ids`는 이 SDK profile과 정확히 일치하는지
확인하는 safety assertion이다. motor ID 순서는 무시하지만 `0..22` 전체 집합이
필요하다. 값은 SDK config 생성자에 전달되고 production factory의 명시적
initialize 단계에서 실제 hardware 접근에 사용된다.

현재 SDK API에서 runtime 생성자에 전달할 수 있는 설정은 motion JSON
경로와 hardware config다. protocol `2.0`은 외부 SDK 저수준 소스에 고정되어
있다. calibration, joint limit 및 실물 안전 정보가 검증되기
전에는 hardware를 활성화하면 안 된다. build/test 성공은 실물 안전성
검증을 의미하지 않는다.

`RobotMotionRuntime`은 SDK runtime 소유자를 backend보다 오래 유지한다.
멤버 소멸 순서상 backend가 먼저 파괴되고 runtime 소유자가 나중에
파괴되므로, 참조 기반 `RobotMotionPlayerBackend`의 dangling reference를
방지할 수 있다. 이 계약은 initialize 횟수를 기록하는 fake SDK ownership
test로 검증한다.

## RobotMotionPlayer backend adapter

`RobotMotionPlayerBackend`는
`IRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON`일 때만 빌드되는 SDK opt-in target이다.
실제 `irc_step::RobotMotionPlayer`를 생성하거나 소유하지 않고, 외부에서
주입된 non-owning API wrapper를 `MotionBackend` 상태로 변환한다.

기본 `sdk_motion_executor` node는 계속 `SimulatedMotionBackend`를 사용한다.
production factory는 SDK ON library로만 제공되며 real backend wiring 및 hardware
launch는 아직 없다. `RobotMotionPlayerBackend`가 node에서 활성화되는 경로도
없다. 관절 방향,
영점, limit, 모션 거리 및 torque 안전 조건이 확인되기 전에는 실제 player를
생성·초기화하거나 이 adapter로 motion을 실행하지 않는다.

## 빌드 모드

기본 빌드는 SDK 경로 없이 catalog-only 모드로 동작한다.

```bash
colcon build --packages-select irc_step_motion_executor
```

SDK-enabled 빌드는 명시적으로 option을 켜는 경우에만 구성된다. SDK 소스의
라이선스, 복사 및 사용에 대한 조직 승인이 끝난 뒤에만 승인된 **외부 경로**를
지정해야 한다. 이 저장소에는 SDK를 복사하지 않으며 `vendor/robot_motion_sdk`
디렉터리나 tar.gz 해제 결과를 만들지 않는다.

```bash
colcon build --packages-select irc_step_motion_executor \
  --cmake-args \
  -DIRC_STEP_ENABLE_ROBOT_MOTION_SDK=ON \
  -DROBOT_MOTION_SDK_DIR=/approved/external/sdk/path
```

`IRC_STEP_ENABLE_ROBOT_MOTION_SDK`의 기본값은 `OFF`,
`ROBOT_MOTION_SDK_DIR`의 기본값은 빈 문자열이다. option이 `OFF`이면 SDK를
탐색하거나 `add_subdirectory()` 하지 않고 어떤 SDK library에도 링크하지
않는다.

option이 `ON`이면 지정한 외부 디렉터리에 다음 항목이 모두 있어야 한다.

- `CMakeLists.txt`
- `robot_motion_player.hpp`
- `robot_motion_player.cpp`
- `add_subdirectory()` 이후 생성되는 `robot_control` CMake target

경로 또는 항목이 없으면 configure 단계에서 명확히 실패하며 다른 경로로
fallback하지 않는다. SDK source는 현재 package build tree 아래의 별도 binary
directory에서 `add_subdirectory()`되고 원본 source는 수정하지 않는다.

SDK-enabled build는 compile probe와 production runtime ownership library를
빌드한다. production factory가 만드는 객체는 메모리상 ownership만 구성하며
hardware initialize나 port/torque 접근을 수행하지 않는다. 따라서 SDK-enabled
build 성공은 실제 로봇에서의 동작 가능성이나 안전성을 의미하지 않는다.

## Launch

```bash
ros2 launch irc_step_motion_executor catalog_only.launch.py
```

launch의 `hardware_enable` 기본값은 `false`, runtime SDK 경로 기본값은 빈
문자열이다. catalog-only node는 두 안전 조건을 강제하며 실제 장치, serial,
torque 또는 motor에 접근하지 않는다. 실물 motion 정보와 안전 조건이 확정되기
전에는 sdk-enabled build 결과를 hardware node로 확장하거나 실물에서 실행하지
않는다. 관절 방향·영점·limit·속도·전류/토크·비상정지 등 calibration 및 안전
정보가 확인되기 전에는 hardware node 사용을 금지한다.

기존 Python `motion_executor_node`, legacy adapter, SDK placeholder,
`full_system.launch.py`는 이 패키지와 별개이며 변경하거나 대체하지 않는다.

## 2026-10-01 현재 Jetson SDK의 전진 재생 정렬

현재 설치 빌드는
`/home/jet/IRC/external_sdk/robot_motion_player_sdk_work_20260801/final step`을
참조한다. 아래 9월 30일 절의 별도 검증 작업본과 구분해야 한다.
이 경로에 적용하는 변경은 `tools/sdk_active_gui_alignment.patch`에 보관한다.

프레임 종료 호출에서 경계 자세와 반복 종료를 함께 처리해 반복마다 한 tick씩
추가되던 지연을 제거했다. GUI와 동일하게 배속 적용 후 정수 ms로 시각을 계산하고,
시작 자세 읽기가 끝난 시점부터 시간을 센다. 직접 실행하는 노드의 기본 주기도
통합 launch 및 GUI와 같은 5ms다. 실제 USB 송신 주기를 보장하는 값은 아니다.

초기화는 기존 PID와 Operating Mode를 덮어쓰지 않는다. 요청한 위치 제어 모드와
실제 모드가 다르거나 읽지 못하면 시작을 거부한다. 이미 모터에 저장된 PID를
과거 값으로 복원하지는 않는다. 모터 4·5번 보정, 모션 JSON의 5도 허용 오차와
기존 최종 완료 판정, 시작 자세 및 카메라 보정 정책은 유지한다.

가짜 하드웨어 검증은 아래 명령으로 재현한다. 같은 실행용 JSON, 시작 자세,
배속, 호출 간격을 양쪽에 주입해 23개 목표각과 종료 시각을 비교한다. 제어 주기
5/10/20ms 및 지터, 서로 다른 시작 자세와 최초 읽기 지연을 포함한다.

```bash
python3 tools/verify_sdk_forward_gui.py \
  --sdk "/home/jet/IRC/external_sdk/robot_motion_player_sdk_work_20260801/final step" \
  --gui-archive /home/jet/Downloads/gui_sdk_20260930_135448.tar.gz \
  --catalog artifacts/robot_motions_runtime.json
```

모터 통신이나 실물 보행을 실행하는 검사가 아니다. 실물 비교 전 같은 PID와
시작 자세인지 확인하고, 시뮬레이션 및 지지 장치가 있는 조건에서 먼저 검증한다.

## 2026-09-30 별도 GUI 호환 SDK 검증 작업본

당시 검토한 외부 SDK는 담당자 전달본
`gui_sdk_20260930_135448.tar.gz`에 `tools/sdk_gui_compat.patch`를 적용한 것이다.
아래 절은 위의 초기 scaffold 설명보다 최신이며, 운영 모션 JSON과 기존 ROS
실행 기본값을 바꾸지 않은 상태에서 검증했다. 실제 모터에서는 실행하지 않았다.

재현용 작업본은 저장소 밖의 **새 폴더**에 준비한다.

```bash
python3 tools/prepare_gui_sdk.py /path/to/gui_sdk_20260930_135448.tar.gz \
  --output /path/to/new-step-sdk
```

준비 도구는 전달 압축파일과 파일별 해시를 검증하고 C++ 패치만 적용한다.
패키지 설치, 로봇 실행, runtime JSON 교체는 수행하지 않는다.
GUI 소스와 저장 상태는 담당자의 원본이며, STEP runtime 보정은
`tools/upsert_motion_catalog.py`가 별도로 적용한다.

C++ 작업본은 PC half-cosine 보간, 발 들기 80%/유지 20%, 현재각 시작,
5ms 호출 기준의 중간 Goal 송신, 10ms 조건의 관측 피드백, 조합 내부 반복
경계를 지원한다. 모터에는 초기화 시 Profile Acceleration/Velocity=0을
설정하고 이후 Goal Position만 보낸다. 생성자에서는 장치에 접근하지 않으며
Operating Mode/PID를 덮어쓰지 않는다. GUI와 같이 종료 때 토크를 자동 해제하지
않으므로 토크 해제는 명시적 emergencyStop 경로로 구분한다.

ROS 호환을 위해 hardware config/preflight, 시작 자세 전환, joint override와
기존 queue/completionSequence API를 유지·보완했다. 시작 자세는 기존 STEP의
실제 코드의 4도/80ms/3000ms 도착 검사와 AUTO 전 2초 대기를 유지한다. GUI에는 없는
STEP 통합 기능이므로 이것까지 GUI와 동일하다는 뜻은 아니다.

2026-09-30의 프레임별 도착 대기 정책은 최신 PC 기준에 따른 2026-10-02
공통 재생 정책으로 대체했다. SDK와 두 full_system launch 모두
`position_tolerance_enabled=false`가 기본값이다. 일반 프레임·반복·queue는
위치 도착을 기다리지 않는다. 예약이 없으면 마지막 Goal 송신 성공 callback에서
완료한다. `true`를 명시하면 기존 모션 끝의 최대 100ms best-effort 검사만
활성화하며, 프레임마다 3000ms 기다리는 옛 동작을 복원하지 않는다.
시작 자세의 별도 위치 검사·80ms 안정화·AUTO 전 2초 대기는 유지한다.
완료는 계획한 Goal 송신이 끝났다는 뜻이며 실제 관절 도착을 증명하지 않는다.
검증은 `test_common_playback`과 startup gate의 가짜 하드웨어 검사로 수행한다.

GUI 전용 가상환경은 ROS Python 경로와 사용자 패키지를 배제해 실행해야 한다.
관측된 버전은 Python 3.10.12, PyQt5 5.15.6, dynamixel-sdk 4.0.5,
pyserial 3.5, numpy 1.21.5다. ROS/vision 환경의 numpy를 일괄 변경하지 않는다.
GUI 창 생성은 모터 탐색/토크 OFF를 수행할 수 있으므로 검증에서는 import와
제공된 가짜 통신 테스트만 실행했다.

```bash
python3 -m venv --system-site-packages /path/to/new-step-sdk/.venv
env -u PYTHONPATH PYTHONNOUSERSITE=1 /path/to/new-step-sdk/.venv/bin/python \
  -m pip install -r /path/to/new-step-sdk/gui/requirements-observed.txt
```

외부 ROBOTIS C++ 라이브러리의 담당자 버전은 전달본에도 특정되지 않았다.
이 PC에서는 `/opt/ros/humble/lib/libdynamixel_sdk.so`로 컴파일·링크했다.
USB latency, 실제 모터 PID/Operating Mode/영점, 실측 전송 주기는 일치 검증을
하지 않았다. 따라서 패키지 버전과 소프트웨어 궤적 일치가 실물 동작의 일치를
보장하지 않는다.

독립 GUI 비교는 원본 Python 재생 메서드를 가짜 I/O에서 실행하고 C++의
`gui_trace_probe`가 계산한 모터 raw 값과 대조한다. SDK의 CTest는 가상 시계로
반복 지연, 시작각, 예약, 취소, 오류, override, 시작 자세 게이트를 확인한다.

```bash
python3 tools/verify_gui_playback.py --sdk-root /path/to/new-step-sdk \
  --probe /path/to/sdk-build/gui_trace_probe artifacts/robot_motions_runtime.json
```

현재 runtime에는 STEP 보정과 추가 모션이 있으므로 새 GUI export를 통째로
덮어쓰지 않는다. ID 4=18도/5=-18도 규칙과 세 예외 및 completion 5도 정책은
`artifacts/AGENTS.md`를 따른다. 9월 30일 새 `robot_motions(3).json` 비교는
`artifacts/20260930_sdk_gui_alignment/motion3_comparison.json`에 보관했다.
