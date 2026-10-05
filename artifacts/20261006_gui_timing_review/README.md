# STEP GUI와 Jetson 재생 시각 비교

이 문서는 1차 조사 당시 상태를 보존한다. 이후 사용자 요청에 따라 실제 코드와 로컬 설치본을 수정한 내용은 [구현 및 검증 결과](IMPLEMENTATION.md)에 있다. 아래의 설치본 86개 불일치는 로컬에서 해소했으며, 당시 비교 증거인 `input_audit.json`은 덮어쓰지 않았다.

동일한 모션 JSON과 보간식만으로 실제 움직임이 같아지는 것은 아니다. 목표를 계산한 타임라인 시각, 모터 쓰기 호출 시각, 관절의 실제 응답을 나누어 확인해야 한다. 이번 조사에서는 GUI 화면 처리 지연을 이 구분에 반영했다. 먼저 로컬 원본과 설치본의 데이터 불일치가 확인됐으므로, 화면 지연 하나로 Jetson의 흔들림이나 넘어짐 원인을 확정할 수 없다.

사용자가 알려 준 실행 경로는 `full_system_robot.launch.py`다. 아래 실행 경로 분석은 현재 저장소의 해당 launch와 C++ SDK에 대한 것이다. Jetson에 설치되어 실행 중인 바이너리·파라미터·모션 JSON과, 이번 GUI 실험에 사용한 파일 경로는 아직 확인되지 않았다. 하드웨어를 연결하거나 로봇을 움직이지 않았다. 제어 로직, GUI, 모션 JSON, 기존 설치본은 수정하지 않았다.

**1. 확인 수준과 화면 처리 지연**

| 근거 | 확인한 내용 | 해석 범위 |
| --- | --- | --- |
| 사용자 제공 Qt 모의 통신 실험 | 32프레임 그래프 숨김 약 5ms, 표시·갱신 약 57ms, 같은 재생 경로에서 그래프 갱신 정지 약 5ms | 목표각 쓰기 함수 호출 간격의 중앙값. USB 패킷이나 실제 움직임 측정이 아님 |
| 사용자 제공 Qt 모의 통신 실험 | 그래프 제거 후 블록 스타일 유지 약 18ms, 스타일 생략 약 5ms | 해당 PC·모션·화면 조건에 해당 |
| 사용자 제공 Qt 모의 통신 실험 | 원본 4프레임 8회 반복에서 그래프 제거 후 두 페이지 모두 약 5ms | 모든 모션·실행이 57ms 또는 18ms 지연을 갖는다는 주장을 지지하지 않음 |
| 저장소 GUI 참조 코드 | 5ms Qt 타이머, 같은 GUI 스레드의 화면 처리·동기식 통신, 궤적 재계산과 스타일 재적용 경로 | 제공 실험의 구조적 설명과 일치. 이 파일이 실험 당시 실행 파일과 같은지는 미확인 |
| 이번 가상 시계 검사 | GUI의 실제 Python 메서드와 C++ 재생기 비교 386건 통과 | Qt 이벤트 루프·paintEvent·실제 USB·관절 응답 검증은 아님 |
| 실제 로봇 | 양 환경의 대응 실측 기록 없음 | 흔들림 원인, 실제 전송 간격, 추종 성능은 아직 판정 불가 |

참조 GUI의 `anim_step()`은 화면 갱신을 약 30fps로 제한하지만, 이는 한 번의 화면 처리 비용이 5ms보다 짧다는 보장이 아니다. `scrub_timeline()`은 조합 페이지 재생에서도 모션 페이지의 `TimelineBlockWidget` 스타일을 갱신한다. 따라서 조합 페이지를 화면 처리 지연이 없는 기준으로 사용하지 않는다.

`JointTrajectoryWidget.paintEvent()`가 `trajectory_samples()`를 통해 전체 궤적을 다시 계산한다. `trajectory_graph.update()`는 그리기 요청이므로, 이 함수 호출 시간만 재면 나중에 이벤트 루프가 처리하는 실제 그리기 비용을 놓친다. 스타일·3D 갱신은 목표 계산/쓰기 앞에 실행될 수 있고, paintEvent는 다음 tick을 늦출 수 있다. 따라서 쓰기 간격뿐 아니라 **타임라인 평가 시각부터 쓰기 시작까지의 지연**도 기록해야 한다.

근거: [GUI 그래프](../20261006_gui_alignment/reference/sdk_gui.py#L839), [재생 tick](../20261006_gui_alignment/reference/sdk_gui.py#L5087), [스타일과 쓰기 순서](../20261006_gui_alignment/reference/sdk_gui.py#L5187). 사용자가 복원했다고 알려 준 그래프·갱신 코드는 변경하지 않았다.

**2. 동일한 최신 데이터를 사용하는지**

아래는 이번 조사 시점의 로컬 파일이다. 전체 SHA-256, 모든 차이, 별칭 해석 결과는 [input_audit.json](input_audit.json)에 있다.

| 입력 | 모션 수 | SHA-256 앞 12자리 |
| --- | ---: | --- |
| `artifacts/robot_motions_runtime.json` | 82 | `7c63d8a34c0a` |
| `artifacts/robot_motions_pc.json` | 82 | `efeaa05b3430` |
| 로컬 설치 패키지의 runtime JSON | 86 | `ce7184c2b871` |
| 기존 GUI 참조 `robot_motions(10).json` | 75 | `fd90be1f68ea` |

현재 PC 원본과 runtime 원본은 모션 이름, 프레임 시간·시작 시각, 배속, 반복, 타임라인 공백, `lift_early_arrival`, `playback_cycle_end`가 일치한다. 전체 필드 비교에서 차이는 관절 4·5 목표각 각각 239곳과 `completion.position_tolerance_deg` 82곳뿐이다. runtime은 저장소 정책에 따라 관절 4=18도, 5=-18도와 허용오차 5도를 적용한다. 따라서 두 파일은 목표각까지 완전히 같은 입력은 아니다. 이 정책은 임의로 해제하지 않는다. 일반 재생은 completion 도착 대기를 사용하지 않으므로 허용오차 값 차이를 곧바로 일반 보행의 대기시간 차이로 해석하지 않는다.

현재 runtime 원본과 설치본은 공통 모션 중 20개의 내용이 다르다. 시간 관련 비교에는 프레임 구성과 반복 경계 플래그도 포함하며, 6개 모션이 다르다. 설치본에만 있는 모션도 4개다. 최신 원본을 편집했더라도 기본 launch가 설치본을 읽으면 이전 데이터를 사용할 수 있다. 이것은 로컬에서 확인한 문제이며 Jetson의 동일 문제는 아직 가설이다.

예를 들어 `건전진45도(6회)`는 첫 프레임에서 아래와 같다. 프레임 번호는 여기서 1부터 센다.

| 관절 | 현재 runtime 원본 | 설치본·이전 GUI 참조 |
| --- | ---: | ---: |
| 14 | 57도 | 56도 |
| 16 | 6도 | 5도 |
| 19 | 61도 | 58도 |

세 번째 프레임의 관절 16도 최신 5.955078125도와 이전 4.955078125도로 다르다. 이 차이는 목표 궤적 자체의 차이이며, 화면 처리 지연과 별도 원인 후보다.

이 모션의 최신 시간은 `(시작, 길이)`가 `(90,60), (150,105), (339,62), (403,110)`ms, 앞·중간 공백은 `90,0,84,2`ms, 배속 1.0, 반복 6회다. 한 회의 타임라인 끝은 513ms이고 명목상 총 시간은 3078ms다. `max_seq_ms=11522`는 이 재생 경로에서 마지막 프레임 이후의 추가 대기를 뜻하지 않는다. 실제 종료 시각은 tick·통신·반복 경계 처리에 따라 더 길 수 있다.

추가 대기는 세 종류로 구분한다.

- JSON의 첫 프레임 시작 시각과 프레임 사이 공백: 재생 타임라인에 포함된다.
- GUI 조합의 `gap_before_ms`: 조합을 저장·내보낼 때 프레임 시각으로 펼쳐진다. GUI의 실시간 조합은 원본별 배속을 구간 시계에 적용하며, 저장 시 공유 시작·종료 경계를 정수 ms로 반올림한다. 실시간 조합과 내보낸 JSON도 해당 유효 타임라인을 비교해야 한다. GUI 메모리의 조합 설정·현재 배속·반복은 파일만으로 확정할 수 없다.
- 알고리즘의 연결 대기: SDK의 `queued_transition_hold_ms`와 상위 bridge의 조건부 대기가 있다. JSON만 비교해서는 이 대기를 확인할 수 없다.

**3. full_system_robot 실행 경로**

실행 흐름은 `full_system_robot.launch.py → sdk_motion_executor → SdkMotionExecutorNode의 poll_timer → SdkExecutorDriver::poll → SdkExecutorCore::poll → RobotMotionPlayerBackend::poll_status → RobotMotionPlayer::update → DynamixelMotionHardware → Dxl`이다. launch의 backend는 `robot_motion_player`, 기본 baud rate는 4,000,000, motor ID는 0..22다.

`motion_json_path` 기본값은 `FindPackageShare('irc_step_motion_executor')/config/robot_motions_runtime.json`이다. 현재 작업 폴더의 `artifacts` 파일을 직접 읽는 설정이 아니다. 또한 ROS overlay에 따라 어떤 설치 prefix가 선택되는지도 달라질 수 있다. 로컬 CMakeCache의 SDK 경로는 `external_sdk/gui_aligned`이며 SDK 파일 해시와 설치 실행파일 해시는 이전 build_manifest와 일치했다. 다만 로컬 빌드는 x86_64이므로 이것으로 Jetson 배포를 확인한 것은 아니다.

| 항목 | 현재 코드의 동작 | 확인해야 할 실제 기록 |
| --- | --- | --- |
| 목표 갱신 | ROS wall timer 설정 5ms, `steady_clock` 경과시간 × 배속으로 타임라인 계산 | 실제 callback 시작 간격과 타임라인 평가→쓰기 지연. 5ms 전송 보장 아님 |
| 통신 | Goal Position SyncWrite 후 조건에 따라 동기식 Present Position SyncRead | 각 함수와 패킷 API의 시작·종료 및 실패 |
| 피드백 | 첫 sample, 읽기 시작 기준 10ms 이상 경과, cycle/모션 종점에서 읽기 | 100Hz 고정 실측이 아님. read가 10ms보다 길면 사실상 매 tick에서 읽을 수 있음 |
| 시작 | 새 `start()`는 프로파일 0 설정 후 실제각을 읽고 보간 시작각으로 사용 | 시작 읽기와 첫 쓰기 사이 시간·초기 자세 |
| 일반 프레임 | 이전 tick에서 활성화한 프레임의 종점을 쓴 뒤 같은 tick에서 경계 sample 전송 가능. 일반 경계에서 시작 시계는 유지 | endpoint와 sample을 합치거나 중복 제거하지 않음 |
| 지연된 tick | 모든 누락 중간 목표를 재전송하지 않음. 종점 보장은 추적 중인 활성 프레임 대상 | 큰 지연으로 활성화되지 않은 프레임을 건너뛰는지도 기록 |
| 내부 cycle | `playback_cycle_end`에서 경계 sample·읽기 뒤 시계 재설정 | 경계 전후 대기와 `cycle` 이벤트 |
| 반복 | 마지막 목표 일부를 시작각에 누적하고 읽기 완료 뒤 다음 반복 시계 시작 | 반복 번호·누적 실제 경과시간·repeat 이벤트 |
| SDK 예약 연결 | 동일 repeatable 모션 또는 end_pose/start_pose 호환 시 예약 가능. 이전 논리 목표를 누적하고 새 실제 시작각 읽기 없이 전환 | queue 요청·수락·전환·다음 첫 쓰기 모두 필요 |
| 예약 대기 | 기본 `queued_transition_hold_ms=0`. 0이어도 다음 모션 첫 sample은 다음 update에서 발생. 양수이면 경계 이후 유지 대기 | 설정값과 실측 경계 간격을 구분 |
| 완료 | 일반 모션 성공은 목표 전송 절차 완료. 실제 관절 도착 확인 아님 | `goals_complete`와 측정각을 별도로 비교 |
| 시작 자세 | full_system_robot은 startup 자세 사용이 기본. 별도의 실제각 정착 검사를 수행 | startup 완료 뒤 첫 보행의 실제 시작 자세 |

프로파일 0 중간 목표 경로는 [sendGuiSample](../../external_sdk/gui_aligned/robot_motion_player.cpp#L271), 경계 처리는 [updateRunning](../../external_sdk/gui_aligned/robot_motion_player.cpp#L307), 실제 SyncRead는 [ReadPositionDegrees](../../external_sdk/gui_aligned/step_dynamixel.cpp#L73)에 있다. 동기식 읽기는 동일 제어 callback을 점유한다. 실측 읽기 시간은 현재 알 수 없으며 과거 가정값 16ms를 측정값으로 사용하지 않는다.

ROS 노드는 `rclcpp::spin()`으로 실행된다. 동일 노드의 콜백 작업과 동기식 통신이 poll 처리를 지연시킬 수 있다. 독립 실행용 `playBlocking()`의 `update(); sleep(5ms)`는 이번 ROS 경로에서 사용하지 않는다. `configureDiagnostics(expected_tick_ms, ...)`의 tick 인자도 재생 주기를 만드는 코드가 아니다.

full_system_robot은 기본으로 머리 override와 어깨 정책을 사용한다. 관절 0은 공·허들·골 상황에 따라 재생 도중 바뀔 수 있으므로 JSON 목표와 실제 쓰기 목표를 함께 기록해야 한다. 상위 [motion_command_bridge_node.py](../../src/mission_control/mission_control/motion_command_bridge_node.py#L1166)에는 자세 전환 전 1초, 골대 게걸음 전 1초, 허들 특정 단계 전 1초 등의 조건부 대기가 있으며 대기 확인 timer는 50ms다. 모든 보행 프레임에 적용되는 대기는 아니다. 예약하지 않고 완료 통지 뒤 새 start를 보내면 메시지·timer 지연과 시작각 읽기도 다시 발생한다.

**4. 같은 형식으로 기록하는 방법**

공통 출력은 `session.json`과 `events.jsonl`로 제안한다. 아래는 새 기록 규격의 설계이며 현재 logger가 전부 구현한다는 뜻은 아니다. 제어 루프에서는 고정 크기 버퍼에 기록하고, 별도 스레드가 직렬화한다. 로그를 켜고 끈 조건도 비교해 기록 자체의 교란을 확인한다. JSON 직렬화·파일 쓰기·해시 계산을 매 tick에서 하지 않는다. Python Queue는 lock·할당 비용이 있으므로 C++ ring buffer와 같은 실시간 특성을 가정하지 않는다.

| 필드 | 의미 |
| --- | --- |
| `schema_version, session_id, platform, process_id` | 기록 규격과 프로세스 구분 |
| `catalog_path, catalog_sha256, gui_or_binary_sha256, sdk_sha256, aliases_sha256` | 파일·실행 코드 식별, resolved path 포함 |
| `effective_motion_id` | 재생 직전 정렬·조합·정책·보정을 반영한 frames/배속/반복/추가 대기 snapshot의 식별값. 같은 정규화 절차로 오프라인 계산 |
| `run_id, motion_instance_id, request_id, motion_name` | 같은 모션을 재예약해도 실행 인스턴스 구분 |
| `event_id, parent_event_id, tick_id, thread_id, event` | 중첩·지연 paint·통신 인과관계 |
| `begin_mono_ns, end_mono_ns, run_origin_mono_ns` | Python `perf_counter_ns`, C++ `steady_clock`의 정수 ns. 동기화되지 않은 PC·Jetson 절대값끼리 빼지 않음 |
| `frame_index, frame_id, repeat_index, cycle_index, timeline_ms` | 프레임 index는 0부터, 반복은 1부터. 시작 전은 null. sample 평가 시점 값을 고정해서 기록 |
| `evaluate_mono_ns, requested_timeline_ms` | tick의 원래 평가 시각과 경계 제한 전 요청 시각. 실제 쓰기 때 목표가 얼마나 오래됐는지 확인 |
| `command_ids, planned_deg, sent_deg, sent_raw` | JSON 보간 목표와 override/양자화 후 전송 목표 구분. 대상 ID 목록 포함 |
| `measured_deg, measured_raw, valid_ids, success, error_code` | 읽기에 성공한 측정값만 새 샘플로 취급. 실패 시 이전 feedback을 새 측정으로 기록하지 않음 |
| `gui_context, graph_visible, graph_joint, block_count` | 화면 비용 비교 조건 |
| `dropped_rows, writer_error` | 기록 유실 및 파일 쓰기 실패. 0인지 확인하기 전에는 완전한 재생 기록으로 취급하지 않음 |

이벤트는 `run_start, tick, evaluate, write, read, frame_endpoint, cycle_end, repeat_end, queue_request, queue_accept, queue_activate, run_end, hold_begin, hold_end, gui_style, gui_3d, gui_plot_paint`로 맞춘다. 경계 이벤트에는 이전/다음 프레임과 반복도 포함한다. 콜백 실행 후 바뀐 메타데이터만 붙이면 이전 반복의 마지막 쓰기가 새 반복으로 잘못 분류될 수 있으므로, 시작 시 context와 전환 후 context를 구분한다.

쓰기·읽기는 두 계층으로 구분해 기록한다. `hardware_call`은 목표 준비·변환 등을 포함한 공통 하드웨어 API 구간, `packet_api`는 GUI `txPacket/txRxPacket`과 C++ 대응 SDK 호출 구간이다. SDK 함수 반환 시각은 실제 USB 선로 전송 완료·모터 적용·관절 도착 시각과 같지 않다. 선로 시각까지 필요하면 별도 USB/시리얼 측정과 시계 대응이 필요하다. SyncRead 측정각도 모든 관절이 동시에 샘플링됐다고 가정하지 않고 읽기 구간 안의 관측으로 취급한다.

GUI는 `anim_step`, 타임라인 평가, 스타일 루프, `update_3d_robot`, `JointTrajectoryWidget.paintEvent` 및 그 안의 궤적 계산을 계측한다. `update()` 호출과 실제 `paintEvent`를 별도 이벤트로 둔다. 중첩된 paint/trajectory와 read/FK 시간을 단순 합산하지 않고, 부모 구간 또는 시간 구간의 합집합으로 계산한다. Qt timer 지연에는 미계측 이벤트·OS 스케줄링도 있으므로 모든 잔여 시간을 그래프 비용으로 단정하지 않는다.

기존 [trace_sdk_gui.py](../../tools/trace_sdk_gui.py)와 [playback_trace.hpp](../../external_sdk/gui_aligned/playback_trace.hpp)는 활용할 수 있지만 다음 보완이 필요하다.

- GUI 기록은 JSONL 초 단위, SDK 기록은 CSV ms 단위이며 이벤트 이름도 다르다. 공통 adapter 또는 exporter를 추가한다. 단위 변환만으로 서로 다른 컴퓨터의 단조 시계 원점이 같아지지는 않는다.
- GUI wrapper에 모션명·프레임 번호·유효 데이터 해시·실제 paint/style 시간이 없다. 시작 데이터 snapshot은 일부 있지만 목표 쓰기 이벤트와 직접 연결하기 부족하다. read 실패 후 기존 `feedback_angles` 복사를 새 측정으로 해석하면 안 된다.
- SDK는 `planned/sample/endpoint/feedback/cycle/repeat/goals_complete`와 시작·종료 시간을 이미 기록하지만 데이터 해시·tick 이벤트·명시적 예약 활성화 이벤트가 없다. `trace_run_`은 queue 전환에서 증가하지 않아 같은 이름을 예약하면 인스턴스 구분이 모호하다. `initial`은 runtime 상태 reset 이전에 기록되어 이전 프레임/반복 값이 남을 수 있다.
- SDK CSV의 `raw_*`는 각도에서 다시 계산한 값이다. 특히 측정 raw는 현재 hardware API에서 clamp 후 각도로 반환되어 원본 raw가 보존되지 않는다. 정확한 raw 측정을 원하면 읽기 경계에서 원본 값을 남긴다. startup 정착 읽기, idle override, hold/실패 경로도 필요하면 계측 범위에 추가한다.

**5. 기록 비교와 현재 판정**

비교는 아래 순서로 한다. 동일한 이벤트 개수부터 가정하거나 반복별로 시간을 다시 0으로 만들어 경계 지연을 지우면 안 된다.

| 비교 | 방법 | 해석 |
| --- | --- | --- |
| 목표 궤적 | 같은 유효 데이터·초기각·timeline에서 `planned_deg` 및 양자화 목표 비교. 이어서 override 전후 비교 | 다르면 JSON/정렬/보간/배속/정책 차이. 현재 설치본의 다리 목표각 차이는 이 단계에서 확인됨 |
| 목표 전송 시각 | run 시작 기준 쓰기 시각, `write_begin[i+1]-write_begin[i]`, 평가→쓰기 지연, 프레임별 sample 수, endpoint 순서, 반복/예약 경계 간격 비교 | 궤적이 같아도 다르면 전달 시각·중간 목표 개수 차이. GUI 화면 비용, 읽기 시간, scheduling과 대조 |
| 관절 추종 | 성공한 읽기의 각도를 직전 유효 전송 목표와 비교하고, GUI·Jetson의 측정각 곡선을 실제 경과시간 기준으로 겹침 | 같은 명령열·시각인데 측정각이 다르면 추종·초기조건·하중 등 추가 조사. 목표 전송만으로 도착 판정하지 않음 |

목표→측정 오차와 모션 타임라인상의 계획각→측정 오차를 별도로 계산한다. 명령은 다음 명령까지 유지되는 계단형 입력으로 표시하되 모터 내부 움직임이 계단형이라고 가정하지 않는다. 읽기가 드문 구간의 실제각을 임의 보간해 순간 도착이나 지연을 확정하지 않는다. 관절별 오차와 지연의 중앙값·p95·최댓값, sample 수, 실패·누락 수, 읽기 소요시간 분포를 함께 본다. 최대 오차만으로 넘어짐의 원인을 확정하지 않는다.

이번 [386건 결과](comparison/results.json)는 현재 runtime 82개와 PC 82개를 각각 동일 입력으로 GUI/C++에 주입한 검사다. 서로 다른 두 JSON의 궤적이 같다는 검사가 아니다. 참조 GUI 메서드를 AST로 가져오고 화면 함수를 Dummy로 대체하며 Qt timer도 실행하지 않는다.

`건전진45도(6회)`의 합성 입력 예시는 다음과 같다. 아래 16ms는 테스트에서 주입한 읽기 시간이며 실측 USB 지연이 아니다.

| 합성 조건 | 쓰기 횟수 | 가상 완료시간 |
| --- | ---: | ---: |
| update마다 5ms 진행, read/write 지연 0 | 642 | 3090ms |
| update 전 시계 증가 5/17/83/6ms 반복, read 16ms, write 1ms | 90 | 3256ms |

각 조건에서 GUI와 C++의 목표 raw, 쓰기·읽기 시각, 검사 대상 경계 결과는 일치했다. 두 조건 사이에는 쓰기 횟수가 크게 달랐다. 이 실험은 전송 시각이 별도 입력이라는 점을 보여 주지만 사용자 제공 57ms·18ms 현상의 재측정이나 Jetson 흔들림의 재현은 아니다. 관절 추종은 양 환경의 실측 기록이 없으므로 현재 구분 판정할 수 없다.

**6. 최소 수정안과 기록 기반 재현 설계**

첫 단계는 입력과 배포를 맞추는 것이다. Jetson에서 실제 node 파라미터의 JSON 경로·파일 해시·바이너리·별칭·override·추가 대기를 확인한다. 다음 검증 실행에서는 검토한 JSON을 `motion_json_path`로 명시하거나 설치본을 일치시킨다. 원본과 설치본을 읽는 프로그램은 실행 중 파일 교체만으로 라이브러리가 갱신되지 않으므로, 재시작 시 로드한 snapshot을 식별해야 한다. GUI도 재생 직전 유효 snapshot을 남긴다. 승인된 어깨 정책이 GUI 검증 데이터와 다르면 그 차이를 남기고 해당 목표로 별도 검증한다.

두 번째 단계는 위 기록 계측만 추가하는 것이다. 변경 후보는 `tools/trace_sdk_gui.py`, `external_sdk/gui_aligned/playback_trace.hpp`, `robot_motion_player.cpp`의 이벤트 기록 지점 및 통신 경계다. 처음에는 보간·반복·예약·피드백 주기를 그대로 두고 실패·누락 없는 대응 로그를 수집한다. 일반 보행에 프레임 도착 대기를 추가하거나 매 프레임 보간 시작을 실제각으로 바꾸지 않는다. 이들은 지연 계측과 별개의 제어 변경이며 보행 연속성과 주기를 바꿀 수 있다.

세 번째 단계는 실측 명령열에서만 선택적 재현 모드를 설계하는 것이다. 검증된 GUI 실행의 성공한 write마다 `(실행 시작 기준 실제 호출 시각, 타임라인 평가 시각, 대상 ID, raw 목표, 프레임/반복/경계)`를 저장한다. 아직 이 기록이 없으므로 재현 스케줄 수치나 허용 lateness를 정하지 않는다.

- 먼저 오프라인에서 GUI의 실제 평가 시각·경계 입력을 C++에 주입하고 목표 raw와 이벤트 순서를 대조한다. 스타일 처리로 목표가 평가 후 늦게 쓰인 구간은 평가 시각과 쓰기 시각을 따로 재현해야 한다. 쓰기 시각에 현재 타임라인을 다시 계산하는 것만으로는 같은 명령이 나오지 않을 수 있다.
- 정확한 명령열 비교가 필요하면 기록된 목표 raw를 기록된 상대 시각에 보내는 선택 모드를 검토한다. 같은 시각의 endpoint/sample도 순서를 보존한다. 상대 sleep의 누적 대신 시작 원점에 대한 절대 마감시각을 사용한다. 이는 새 모션의 일반 제어 규칙이 아니라 특정 검증 실행 재현 모드다.
- Jetson 동기식 읽기가 다음 기록 시각을 침범하는지 먼저 측정한다. 침범하면 그 환경에서 해당 스케줄을 정확히 재현할 수 없다. 누락 목표를 몰아서 보내거나 임의로 읽기를 비동기화하지 않는다. 기존 단일 버스 소유권을 유지하고, 실측에 근거한 실패/중단 기준과 별도의 읽기 스케줄 검증이 필요하다.
- 초기 자세·목표 policy·모션 hash·모터 설정·부하 조건을 함께 맞춘다. 기록된 GUI 목표열은 초기 자세가 다른 로봇에서 같은 움직임을 보장하지 않는다. 기록의 단조 증가, 유실, 마지막 목표, 반복/예약 경계를 확인한 뒤 새 재현 코드는 먼저 모의 하드웨어·시뮬레이션에서 검증한다. 실제 보행 검증은 그 이후 단계다.

57ms나 18ms 고정 sleep을 넣지 않는다. 전체 배속을 임의로 낮추지 않는다. 배속 변경은 궤적의 시간축 전체를 바꾸지만 불규칙한 GUI 지연은 평가·쓰기 시각, 누락 sample, 경계 시계 재시작에 서로 다르게 영향을 준다. GUI 최적화도 기존에 조정한 보행의 명령 시각을 바꿀 수 있으므로 전후 실측 없이 동작 보존으로 간주하지 않는다.

**7. 기록 수집과 재검증 명령**

이미 실행 중인 Jetson에서 다음 명령은 상태 확인용이다. 로봇 launch를 새로 실행하는 명령은 아니다.

```bash
ros2 pkg prefix irc_step_motion_executor
ros2 param get /sdk_motion_executor motion_json_path
ros2 param dump /sdk_motion_executor
```

위에서 확인한 실제 JSON 경로와 실행파일에 `sha256sum`을 적용해 [input_audit.json](input_audit.json)의 전체 해시와 대조한다. 이후 예정된 실측 실행에는 기존 launch 인자에 `motion_trace_enabled:=true motion_trace_goals:=true motion_trace_path:=<실행별 CSV 경로>`를 추가할 수 있다. 이는 현재 SDK CSV 항목만 수집하며 GUI paint나 새 공통 규격을 구현하지 않는다. 같은 로그 경로를 재사용하면 덮어쓸 수 있으므로 실행별 이름을 사용한다.

이번 로컬 검증은 다음과 같이 재현할 수 있다. 모터 하드웨어를 사용하지 않는다.

```bash
python3 artifacts/20261006_gui_timing_review/audit_inputs.py
g++ -std=c++20 -O2 -pthread -I external_sdk/gui_aligned \
  external_sdk/gui_aligned/gui_execution_probe.cpp \
  external_sdk/gui_aligned/robot_motion_player.cpp \
  external_sdk/gui_aligned/motion_pattern.cpp \
  -o /tmp/step_gui_delay_execution_probe
python3 tools/verify_latest_gui_execution.py \
  --gui artifacts/20261006_gui_alignment/reference/sdk_gui.py \
  --probe /tmp/step_gui_delay_execution_probe \
  --output artifacts/20261006_gui_timing_review/comparison \
  artifacts/robot_motions_runtime.json artifacts/robot_motions_pc.json
```

재실행하면 이 조사 폴더의 evidence 파일을 갱신한다. 모션 데이터에는 쓰지 않는다. GUI 실험의 실행 파일·재생 직전 데이터 snapshot과 Jetson 실측 로그가 확보되면, 위 세 단계 비교로 처음 차이가 생기는 위치를 판정할 수 있다.
