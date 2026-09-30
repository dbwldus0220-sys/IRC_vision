2026-09-30 01:11:11 영상 분석

제어 소스는 변경하지 않았다. 영상, 촬영 당시 ROS 로그, 현재 src/build 코드를 확인했다. 공 분석기·검출기·허들 플래너·미션 플래너의 src/build 파일 내용은 일치한다. 전체 ROS 토픽 녹화와 프레임별 명령 입력은 없어 개별 발행 순간의 모든 수치는 복원할 수 없다. 재현 스크립트의 입력은 합성 데이터이며 실제 토픽 재생이 아니다.

프레임 파일 숫자는 컨테이너 PTS 기준 요청 초이다. 화면 오른쪽 위 녹화 타이머는 약 1~3초 차이가 있으므로 아래 표는 화면 표시 시각을 사용한다. 이미지는 원본 해상도의 추출 프레임이다.

| 화면 표시 시각 | 파일 | 관찰 |
|---|---|---|
| 1:03 | frame_065.jpg | 흰 양말/발 부위 ball 0.32, INFO STALE. 공 확정이나 접근 증거는 아니다. |
| 1:55 | frame_117.jpg | LINE FORWARD 6, hurdle CANDIDATE 1/15. |
| 2:04 | frame_125.jpg | HURDLE / FINE FORWARD, Depth Z 0.48m, 좌우 깊이 약 0.51m, Parallel err +0.8deg. |
| 2:17 | frame_138.jpg | LINE RETURN LEFT 4, Ground head -9.61deg. |
| 2:19 | detail_139.75.jpg | LINE RETURN RIGHT 4, Ground head +14.36deg, offset -0.349. 실행 중 화면이며 명령 발행 입력과 같다는 보장은 없다. |
| 2:20 | frame_142.jpg | RIGHT 4 실행 중 Ground head -11.70deg, 오른쪽 코너 기억 로그. |
| 2:21 | frame_144.jpg | RECOVER_LEFT_TURN_RIGHT_4 성공 로그. |
| 2:28 | frame_150.jpg | 공 RAW/SEARCH, Frame sync STALE 166.8ms. |
| 2:33 | frame_155.jpg | 공 신뢰도 0.93, Frame sync STALE 200.1ms. |
| 2:38 | frame_160.jpg | 공 0.98, Analyzer CONFIRMED / APPROACH, 동시에 INFO STALE 133.4ms. |

라인 복귀

실행 로그 순서: GO 완료 → RECOVER_LEFT_TURN_LEFT_4 → RECOVER_LEFT_TURN_RIGHT_4 → RECOVER_LEFT_TURN_LEFT_4. RECOVER_LEFT는 라인의 위치, TURN_RIGHT는 회전 방향이다. 배너는 실제 motion_id의 RECOVERY를 RETURN으로 바꾼 표시다. 라인 복귀 분기는 화면의 Image head나 표시 Steering이 아니라 ground_heading_error_deg와 filtered_lateral_offset_norm을 사용하며, 일반 조향 계산보다 먼저 실행된다.

방향 조건은 단순히 라인이 왼쪽이면 왼쪽 회전이 아니다. 기본 방향 임계값 10도, 바깥쪽을 향할 때 3도, 코너/중앙 복귀 예외가 있다. 기본 설정에서는 offset=-0.349, heading=10~12도 미만에서 오른쪽 복귀를 선택하지만 12도 이상이면 중앙을 향하는 예외로 복귀를 생략한다. 합성 재현에서 이 불연속을 확인했다. 따라서 +14.36도 화면 하나로 그 값이 우회전 발행 원인이라고 단정할 수 없다. 회전 중 수치가 바뀌어도 이미 실행 중인 모션의 배너는 유지된다. 발에 의한 가림, 보행/목 자세 변화, 고정 homography의 오차 가능성을 확인할 필요가 있다. 작은 fitting RMSE만으로 보정 행렬의 정확성이 보장되지는 않는다.

공 판단

현재 기본값: YOLO ball 클래스 출력 >=0.20 → ball_analyzer 후보 >=0.30 → 최근 40개 검출 콜백 중 같은 후보 18회 → 제어 시 >=0.35와 유효 깊이/제어 범위 등 추가 조건. 여기서 0.35는 35% 점수 임계값이며 3.5가 아니다. 점수는 실제 정답 확률로 보정된 수치라고 볼 수 없다.

같은 후보 조건: 중심 이동/영상 대각선 <=0.18, min(이전면적,현재면적)/max(...) >=0.40. 불일치하면 새 후보로 초기화. 11회 연속 미검출이면 초기화. 현재 검출이 없으면 그 프레임 confirmed=false지만 짧은 끊김 뒤 같은 후보는 확정 상태를 회복할 수 있다. 모든 후보를 독립 추적하지 않고 confidence·중앙성·깊이·면적 점수가 가장 높은 하나를 누적한다. 모양/색상/발 제외 규칙은 없다. 깊이 자체는 RGB 확정의 필수 조건이 아니며 제어 진입에는 필요하다. RGB-depth 허용 차이 기본 50ms, 최소 유효 픽셀 5개, depth timeout 0.7초.

INFO STALE는 화면 RGB와 ball_info RGB stamp 차이가 기본 50ms보다 클 때도 발생한다. 확인한 장면의 133~200ms는 이 제한을 넘는다. 이는 박스 인식 실패 또는 내부 미확정과 동의어가 아니다. 화면 FPS는 추론 전후 구간 처리시간 역수로 계산하며, 표시·대기·분석 등 전체 처리량을 뜻하지 않는다. 해당 로그의 effective_fps는 대체로 11~16 수준이어서 연속 18회도 약 1.1~1.6초가 필요하다.

1.4m가 줄자로 잰 지면 거리인지 Depth Z인지 구분해야 한다. 코드 제어 거리 상한은 Depth Z 1.5m이며 RGB 확정은 거리와 별개다. 이 영상만으로 1.4m에서의 검출률이나 이전 모델 대비 성능 저하를 정량 확정할 수 없다. 2번째 실행은 거리 1.142m에서 BALL 진입, 이후 confirmation completed 로그가 존재한다.

허들 직진 원인

motion_decision_planner.py의 실제 허들 호출은 항상 positioning=True다. 이 경로는 path_reference를 무시하고 중심 픽셀 방향 atan2(dx, bottom_distance_px)을 사용한다. 이 값은 실제 몸체 yaw가 아니다. 회전 임계값은 70도이며 center_turn_min_angle_deg=15는 이 분기에 사용되지 않는다. 평행 오차 8도는 계산되지만 최종 시퀀스 허가에 강제되지 않는다. bottom_distance<=100px이면 회전 차단이 유지되고 방향 오차가 남아도 최종 시퀀스를 허용할 수 있다.

촬영 로그: depth 0.691m에서 positioning 진입 → depth 0.536m, center_steering -42.5218deg에서 최종 시퀀스 확정. GO는 남은 미세 전진 2회, 대기, 허들 모션으로 구성되어 이후 각도 재정렬이 없다. 합성 재현에서도 중심 -42.52도, 허들 평행 오차 30도일 때 0.9m STRAIGHT, 0.6m STRAIGHT_0, 0.536m GO가 나온다.

허들 각도 추정 자체도 보강 필요: 현재 box 높이 55%의 수평선에서 폭 15/32.5/50/67.5/85% 지점 depth를 읽는다. 기울어진 허들은 일부 샘플이 바닥일 수 있다. 각도는 atan2(right_depth-left_depth, box_width*center_depth/fx)로 근사하므로 실제 샘플 간격(기본 box 폭의 70%)과 분모도 다르다. 정확한 yaw는 두 샘플의 역투영 3D 좌표와 카메라-몸체 변환이 필요하다.

개선 우선순위 제안

1. 화면에 raw/confirmed/제어가능/프레임 나이를 분리하고 stamp가 맞는 RGB와 분석 결과를 표시한다. 단순히 stale 제한을 풀어 오래된 위치를 새 위치처럼 표시하지 않는다.
2. 같은 원본 카메라 입력으로 거리별 검출률, 최초 확정 시간, 후보 초기화 횟수, 프레임 지연, 발 오검출을 기록한다. 스크린캐스트는 UI가 합성되어 있어 학습 원본으로 적합하지 않다.
3. 공 1~1.8m의 작은 영상, 실제 조명·보행 블러·가림과 발/양말/신발의 오검출 사례를 기존 데이터에 추가해 재학습한다. 같은 영상의 인접 프레임이 train/val 양쪽에 섞이지 않게 촬영 단위로 나눈다. 임계값만 낮추면 발 후보가 늘 수 있다.
4. 허들은 통과 위치(라인과 허들 교차점 또는 허들 유효 폭 안의 목표점)와 정면 각도(허들 가로축에 수직인 몸체 방향)를 별도로 제어한다. box는 ROI로만 쓰고 내부 색/경계/마스크에서 실제 허들 표면을 찾는다. 여러 표면 depth 점을 역투영하고 몸체 좌표계에서 강건하게 직선을 맞춘다.
5. 충분히 떨어진 곳에서 짧은 회전→정지→새 관측으로 보정한다. 예시 초기 후보는 회전 시작 10도, 종료 5도, 최종 8도 이내 및 연속 3~5회 유효 관측이다. 이는 검증 전 수치로 실제 최소 회전량/정밀도에 맞춰야 한다. 근접 회전 차단은 방향 미정렬 상태의 GO 허가로 이어지지 않도록 WAIT/재관측을 둔다. 먼저 오프라인/시뮬레이션에서 검증한다.

참고 공식 문서

- https://docs.ultralytics.com/yolov5/tutorials/tips-for-best-training-results/ (다양한 이미지와 배경 샘플)
- https://docs.ultralytics.com/tasks/obb/ (회전 박스는 일반 검출 박스와 별도 기능)
- https://dev.realsenseai.com/docs/projection-texture-mapping-and-occlusion-with-intel-realsense-depth-cameras/ (Depth Z와 3D 거리, 역투영)
