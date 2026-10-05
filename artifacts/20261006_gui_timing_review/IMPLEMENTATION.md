# STEP GUI 재생 비교 기능 구현 결과

사용자의 후속 요청에 따라 조사에 그치지 않고 GUI 기록 도구, C++ 실행 기록, 공통 비교 도구, 기록 시각으로 C++ 재생기를 실행하는 검증기를 구현했다. 로컬 SDK 실행파일을 다시 빌드·설치하고 설치 패키지의 모션 JSON을 최신 원본과 일치시켰다. 이 작업은 x86_64 개발 환경에 대한 것이며 Jetson 원격 배포나 로봇 구동을 수행한 것은 아니다.

원본 `artifacts/robot_motions_runtime.json`과 `robot_motions_pc.json`은 바이트 단위로 보존했다. 설치본은 기존 86개에서 원본과 동일한 82개로 갱신했다. 따라서 이전 설치본으로 실행하던 경우에는 이미 원본에 반영돼 있던 최신 목표각·타이밍이 적용된다. 이를 제어 로직 변경 효과와 혼동하지 않아야 한다. 이전 설치 데이터는 `deployment_before/`에 보존했다.

**구현한 처리**

| 파일 | 변경 내용 |
| --- | --- |
| `tools/trace_sdk_gui.py` | 실제 `paintEvent`, 궤적 계산, 블록 스타일, 3D 갱신 시간 기록. 목표 평가 시각, 종점/sample 구분, 재생 직전 데이터 snapshot과 해시 추가. 실패한 읽기는 이전 각도를 새 측정값으로 내보내지 않음 |
| 같은 GUI 도구의 bus proxy | 실제 SyncWrite에 성공적으로 준비된 ID/raw와 SDK packet API 구간 기록. 일부 `addParam` 실패 시 요청각 전체를 전송된 값으로 표시하지 않음. SyncRead 원본 raw와 packet API 구간도 별도 기록 |
| `external_sdk/gui_aligned/playback_trace.hpp` | 기존 CSV 컬럼을 유지하며 단조 ns 시각·tick·이벤트 번호·유효 프레임 정보 추가. CSV 인용부호 escape 수정 |
| `external_sdk/gui_aligned/robot_motion_player.cpp` | 유효 프레임 정의와 tick 기록. 초기 읽기의 이전 프레임/반복 값 제거. startup과 같은 이름의 예약 모션에도 실행 ID를 따로 부여. queue 수락/활성화 기록 |
| `tools/compare_playback_traces.py` | GUI JSONL와 SDK CSV를 공통 ns 기반 JSONL로 변환. 유효 목표 데이터, 같은 timeline의 raw 목표, 실제 호출 간격, 평가→쓰기 지연, 읽기 시간, 관측된 추종 오차를 분리해 비교 |
| `external_sdk/gui_aligned/recorded_execution_probe.cpp` | GUI 기록의 평가 시각과 I/O 구간을 주입해 실제 `RobotMotionPlayer`를 가짜 하드웨어에서 실행. 목표 raw·I/O 순서·실행 지연·완료 여부 검사. Dynamixel/ROS와 링크하지 않음 |

실제 GUI 파일은 수정하지 않으며 wrapper로 실행할 때 계측한다. 그래프와 스타일 갱신을 제거하거나 timer를 변경하지 않았다. 기존 C++의 보간, 프레임 종점 처리, 반복, 예약 연결 정책도 유지했다. 기록을 켜면 계측 비용은 추가되므로 실기에서 기록 ON/OFF 비교가 필요하다. 무하드웨어 검사에서 명령 결과가 동일하다는 사실이 실제 부하까지 0임을 의미하지 않는다.

유효 데이터 해시는 양쪽 로그에서 프레임 이름·정렬된 시작/길이·목표각·조기 도달/반복 경계 flag·배속·반복을 정규화해서 오프라인으로 만든다. 초기각은 별도로 비교한다. 동적인 관절 override는 같은 타임라인의 실제 write raw 차이로 드러난다. 입력 파일 자체의 해시와 이 유효 데이터 해시는 서로 다른 식별값이다.

**실제로 남아 있는 부분**

GUI의 57ms·18ms 중앙값을 제어 코드의 sleep으로 넣지 않았다. 실측된 시각별 목표열이 없으므로 로봇에 적용할 지연 스케줄도 아직 정하지 않았다. 새 검증기는 **기록이 있을 때 C++ 알고리즘이 같은 호출 조건에서 무엇을 보내는지 확인하는 기능**이다. 실제 모터에 기록된 목표를 재송신하는 모드는 아니다.

현재 기록 시각 재현은 처음부터 시작한 GUI 모션 페이지의 단독 모션을 지원한다. 반복과 불규칙한 tick·read/write 지연은 지원하지만 GUI live composer의 구간별 시계, 일시정지 후 재개는 명시적으로 거절한다. 두 페이지 모두 기록·비교는 가능하다. GUI live composer의 원본 타임라인을 저장 JSON의 정수 ms로 펼치는 과정은 별도 검증이 필요하며, 그 차이를 무시해 재현 성공으로 표시하지 않는다.

원점·목표 평가 시각·read/write 구간은 기록대로 주입하지만, 반복 경계에서 GUI가 통신 이후 별도 UI 처리에 소비한 시간 등 C++에 없는 처리는 자동으로 맞추지 않는다. 이 때문에 불일치가 나오면 실패 결과를 보존하고 그 지점을 다음 알고리즘 수정의 근거로 사용한다. 로그를 강제로 통과시키도록 시간을 보정하지 않는다.

SDK feedback CSV의 raw는 여전히 반환된 각도로부터 복원한 값이다. GUI packet_read에는 원본 raw가 있다. C++ 저수준 packet API·원본 raw의 직접 계측과 실제 USB 선로 시각은 이번 변경의 범위에 포함되지 않는다. 비교기의 추종 오차는 읽기 구간에서 관측된 위치와 마지막 전송 목표 사이의 오차이며 관절 도착 보장이 아니다.

**사용 방법**

GUI를 계측해서 실행하려면 실제 사용하는 GUI 경로를 지정한다. 기존 로그 덮어쓰기는 거절한다. 미저장 모션이나 조합은 `--motion-name`으로 이름을 명시할 수 있다.

```bash
python3 tools/trace_sdk_gui.py \
  --gui /path/to/sdk_gui.py \
  --output /tmp/step_gui_run1.jsonl \
  --motion-name '건전진45도(6회)'
```

Jetson에서 변경된 SDK를 빌드·설치한 후, 예정된 `full_system_robot.launch.py` 실행에 아래 인자를 추가하면 SDK 기록을 수집한다. 기존 ROS 파라미터 인터페이스를 그대로 사용한다. 개발 PC에서 만든 x86_64 실행파일을 Jetson용 빌드로 간주하면 안 된다.

```text
motion_trace_enabled:=true
motion_trace_goals:=true
motion_trace_path:=/tmp/step_sdk_run1.csv
```

startup과 각 예약 모션에 run ID가 따로 있다. 비교할 실제 모션의 run을 지정한다. 아래의 SDK run 2는 예시이며 로그의 `run_start`와 `motion`을 보고 선택한다.

```bash
python3 tools/compare_playback_traces.py \
  --gui /tmp/step_gui_run1.jsonl --gui-run 1 \
  --sdk /tmp/step_sdk_run1.csv --sdk-run 2 \
  --normalized-dir /tmp/step_common_trace \
  --output /tmp/step_trace_comparison.json
```

`quality`에서 양쪽 기록의 정상 종료와 dropped_rows=0을 먼저 확인한다. `effective_definition_equal`과 `initial_angles_equal`이 다르면 같은 초기 조건의 전송 시각 비교로 해석하지 않는다. 같은 timeline의 write가 없으면 누락된 비교점으로 표시하며 목표를 보간해서 가짜 측정값을 만들지 않는다. 중첩된 그래프 계산/paint 시간은 단순 합산하지 않는다.

기록 시각으로 C++를 검증하는 아래 명령은 하드웨어를 사용하지 않는다. 검증기가 보고하는 값은 동작 성공률이 아니라 기록과의 불일치 수다.

```bash
python3 tools/compare_playback_traces.py \
  --gui /tmp/step_gui_run1.jsonl --gui-run 1 \
  --export-replay /tmp/step_recorded_run
build/irc_step_motion_executor/robot_motion_sdk/recorded_execution_probe \
  /tmp/step_recorded_run/catalog.json /tmp/step_recorded_run/schedule.txt
```

로그가 잘렸거나 유실됐거나 초기각 23개가 없거나 write가 실패한 경우 기준 동작으로 내보내지 않는다. 검증기의 종료 코드는 0=일치, 1=목표/I/O/시각/완료 불일치, 2=입력 또는 실행 준비 오류다.

**검증 결과**

- GUI wrapper가 원래 명령 인자와 모션 데이터를 보존하는지 검사했다. 실패한 feedback과 일부 motor 파라미터 준비 실패도 검사했다.
- SDK trace ON/OFF에서 write/read/raw/timeline 출력이 완전히 같음을 검사했다. 같은 이름의 예약 모션이 서로 다른 실행으로 기록되는지도 확인했다.
- 실제 GUI 메서드로 만든 합성 기록을 C++ 재생기로 재현했다. 목표각을 일부러 바꾸면 불일치를 검출했다. 이 기록은 실측 로봇 기록이 아니다.
- 새 검사·기존 데이터 정책 검사 12건, SDK CTest 3건 통과. 전체 현재 모션을 대상으로 한 GUI/C++ 가상 시계 비교 386건 통과.
- 모션 원본 해시는 조사 당시와 같고, 로컬 설치본 runtime/PC JSON은 각각 원본과 바이트 단위로 일치한다. 세부 해시와 빌드 정보는 `implementation_validation.json`에 있다.
