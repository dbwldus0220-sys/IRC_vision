# IRC 로봇 주행 비전

ROS 2와 Intel RealSense D435i 영상에서 YOLO26 객체를 탐지하고, 라인 주행 정보와 경기장 미니맵 상태를 계산하는 IRC 휴머노이드 로봇 대회용 비전 프로젝트입니다.

## 발표용 핵심 요약

이 프로젝트의 목표는 카메라 영상에서 `line`, `ball`, `goal`, `backboard`, `hurdle`을 한 번의 YOLO 추론으로 탐지하고, 각 미션에 필요한 기하 정보와 행동 후보를 ROS 2 토픽으로 제공하는 것입니다.

개발은 다음 순서로 진행했습니다.

1. OpenCV 색상·윤곽선 기반 바닥/방향 탐색 프로토타입을 제작했습니다.
2. 조명과 배경 변화에 더 잘 대응하도록 YOLO26 다중 객체 탐지로 전환했습니다.
3. 여러 개의 `line` 검출 중심점을 가까운 순서로 연결해 경로, heading, lateral offset, curve, quality를 계산했습니다.
4. RealSense aligned depth와 camera intrinsics를 결합해 공·골대·허들의 거리와 좌우 위치를 계산했습니다.
5. 인식과 행동 판단을 분리하기 위해 각 미션을 `analyzer → planner → controller` 구조로 구성했습니다.
6. 네 analyzer를 한 프로세스에서 실행하는 `step/unified_vision_node`와, 네 planner 중 하나만 선택하는 `mission_control/motion_decision_node`를 추가했습니다.
7. YOLO 화면이 실제 선택된 planner를 따라 line 전체 경로 또는 객체별 metrics를 자동 표시하도록 통합했습니다.

현재까지 완성된 범위는 **비전 인식, 기하 정보 계산, 추상 행동 판단, 통합 토픽 발행, 화면 시각화**입니다. 실제 STEP SDK 모션 번호와 C++ 모션 완료/실패 신호 연결은 다음 단계입니다.

## 패키지 역할 분리

- `step`: 카메라·YOLO·analyzer와 line/ball/goal/hurdle별 planner의 원본
- `mission_control`: `step`의 planner를 import하여 미션 우선순위와 최종 명령 하나를 선택

의존 방향은 `mission_control → step` 한쪽입니다. 따라서 `step`은 단독으로
계속 빌드하고 시험할 수 있으며, planner의 경로·클래스명·입출력 계약을
바꾸지 않는 내부 로직 수정은 `mission_control`에도 그대로 반영됩니다.
두 패키지는 별도 Git 저장소가 아니라 같은 `my_cv` 저장소에서 함께
커밋하고 push합니다.

단위 규칙은 다음과 같습니다.

| 종류 | 내부 ROS/계산 단위 | 설명 |
|---|---:|---|
| 화면 좌표와 간격 | `px` | bbox, 중심점, 화면 중심 오차 |
| 정규화 화면 오차 | 무단위 | 화면 반폭 또는 높이 기준 비율 |
| 실제 거리 | `m` | RealSense depth와 3D 위치; SDK 경계에서 필요하면 `×1000`하여 mm로 변환 |
| 각도 | `deg` | 화면 기준 bearing, heading, hurdle angle |
| 속도 | `m/s`, `rad/s` | 현재 planner의 하드웨어 독립 목표값 |

발표 자료용 압축 정리는 [`docs/PRESENTATION_HANDOFF.md`](docs/PRESENTATION_HANDOFF.md)에 별도로 작성했습니다.

## 주요 기능

- RealSense RGB 이미지 토픽 구독
- YOLO26 ONNX 기반 `line`, `ball`, `goal`, `backboard`, `hurdle` 탐지
- YOLO `line` 중심점 기반 경로 분석
- 오검출 line 점 제거용 path continuity filter
- heading, lateral offset, curve, quality 정보 계산
- 직진/좌회전/우회전, 선속도, 각속도, 이동량 주행 명령 계산
- line debug monitor와 path visualizer 제공
- 경기장 ㄹ자 미니맵과 mission state 시각화
- RealSense D435i IMU gyro와 line 정보를 이용한 실시간 robot pose 추정
- 공 탐지 결과와 aligned depth를 이용한 `/vision/ball_info` 발행
- 허들 탐지 결과와 aligned depth를 이용한 `/vision/hurdle_info` 및 `GO` 후보 발행
- RGB-D 특징점 추적과 RANSAC 기반 `/vision/visual_odom` 실험 노드 제공

## 실행 환경

- ROS 2
- Python 3
- rclpy
- sensor_msgs
- cv_bridge
- OpenCV
- NumPy
- ONNX Runtime
- 컬러 이미지 토픽을 제공하는 카메라 노드

기본 구독 토픽은 `/camera/color/image_raw`입니다.

## 빌드

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
colcon build --packages-select step mission_control --symlink-install
source ~/my_cv/install/setup.bash
```

현재 개발 환경은 ROS 2 Humble 기준입니다.

### 다른 PC에서 설치

YOLO ONNX 모델은 `src/step/models/best.onnx`에 포함되어 있고 빌드 시 `share/step/models/best.onnx`로 설치됩니다. 따라서 기존 PC의 `/home/geonwoo/Desktop/...` 경로가 없어도 기본 실행할 수 있습니다.

```bash
git clone https://github.com/geonwoo0407/IRC_vision.git my_cv
cd ~/my_cv
source /opt/ros/humble/setup.bash
colcon build --packages-select step mission_control --symlink-install
source ~/my_cv/install/setup.bash
```

다른 PC에도 ROS 2 Humble, RealSense ROS driver, OpenCV, NumPy, cv_bridge, ONNX Runtime이 설치되어 있어야 합니다. 자세한 환경 설정은 [`docs/PC_SETUP.md`](docs/PC_SETUP.md)와 [`docs/JETSON_SETUP.md`](docs/JETSON_SETUP.md)를 참고합니다.

## 실행

기본 실행은 RealSense, YOLO26 detector, line analyzer, visualizer, mission state, minimap을 나누어 실행합니다.

전체 경기용 노드는 다음 launch 명령 하나로 실행할 수 있습니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mission_control full_system.launch.py
```

카메라를 별도로 실행 중이면 `enable_camera:=false`를 지정합니다.

```bash
ros2 launch mission_control full_system.launch.py enable_camera:=false
```

### 경기용 통합 실행

기존 line, ball, goal, hurdle 파일은 유지하면서 네 analyzer를 한 프로세스에서 실행하고, 네 navigation controller 대신 하나의 motion decision node가 기존 planner를 직접 호출합니다.

```text
yolo26_detector
    ↓ /vision/detections
unified_vision_node (한 프로세스)
    ├── yolo_line_analyzer
    ├── ball_analyzer
    ├── goal_analyzer
    └── hurdle_analyzer
    ↓ /vision/*_info
mission_control/motion_decision_node (단일 판단 노드)
    ↓ /navigation/motion_command
SDK/C++ motion node
```

통합 모드에서는 기존 `yolo_line_analyzer`, `ball_analyzer`, `goal_analyzer`, `hurdle_analyzer`를 별도로 실행하지 않습니다. 기존 `line_navigation_controller`, `ball_navigation_controller`, `goal_navigation_controller`, `hurdle_navigation_controller`도 함께 실행하지 않습니다. 동일 토픽이 중복 발행될 수 있기 때문입니다.

RealSense와 YOLO를 실행한 다음 아래 두 명령을 각각 새 터미널에서 실행합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step unified_vision_node
```

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run mission_control motion_decision_node
```

미션 단계는 `/mission/phase`로 지정합니다. `AUTO`는 시험용이며 실제 경기에서는 명시적인 단계를 보내는 것이 안전합니다.

`*_SEARCH` 단계는 대상이 보이기 전까지 line planner를 사용하고, 대상이 검출되면 해당 planner로 전환합니다. `PICK_LOCK`, `SHOOT_LOCK`, `HURDLE_LOCK` 같은 `*_LOCK` 단계에서는 C++ 모션 완료 상태를 기다리며 새 이동 명령을 내지 않습니다.

```bash
ros2 topic pub --once /mission/phase std_msgs/msg/String "{data: 'BALL_SEARCH'}"
ros2 topic pub --once /mission/phase std_msgs/msg/String "{data: 'GOAL_SEARCH'}"
ros2 topic pub --once /mission/phase std_msgs/msg/String "{data: 'HURDLE_APPROACH'}"
ros2 topic pub --once /mission/phase std_msgs/msg/String "{data: 'LINE_TRACK'}"
```

통합 출력은 아래에서 확인합니다.

```bash
ros2 topic echo /navigation/motion_command
```

YOLO의 `metrics_mode:=auto` 화면은 `/navigation/motion_command`의 실제 `source`를 우선 표시합니다. line planner가 선택되면 `LINE METRICS`와 현재 `STRAIGHT`, `LEFT`, `RIGHT` 등의 action이 표시됩니다.

`PICKUP_NOW`, `SHOT`, `GO`는 조건에 처음 진입할 때만 `sdk_motion_requested: true`가 한 번 발생합니다. 현재 `sdk_motion_id`는 `null`이며 알고리즘/SDK 담당자와 모션 번호 계약을 확정한 뒤 매핑해야 합니다.

터미널 1에서 RealSense를 실행합니다.
IMU 기반 위치추정을 같이 테스트하려면 `enable_gyro:=true`, `enable_accel:=true`를 함께 켭니다.

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_gyro:=true \
  enable_accel:=true
```

터미널 2에서 YOLO26 detector를 실행합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step yolo26_detector --ros-args \
  -p device:=cpu \
  -p display:=false \
  -p publish_annotated_image:=false \
  -p max_fps:=30.0
```

화면을 보면서 RGB 카메라 값을 실시간으로 조정하려면 `display`를 켭니다. 별도 설정 프로그램 없이 기존 YOLO 창 아래의 2열 카메라 패널을 조작하면 RealSense 카메라 노드의 파라미터가 즉시 갱신됩니다.

```bash
ros2 run step yolo26_detector --ros-args \
  -p device:=cpu \
  -p display:=true \
  -p metrics_mode:=auto
```

조정 가능한 항목은 `Auto Exposure`, `Exposure`, `Gain`, `Brightness`, `Contrast`, `Saturation`, `Sharpness`, `Auto White Balance`, `White Balance`, `Power Line`입니다. 두 자동 항목은 긴 슬라이더가 아닌 `ON/OFF` 버튼이며, `Power Line`은 `OFF/50/60/AUTO` 네 버튼 중 하나를 선택합니다. 나머지 연속값만 짧고 동일한 길이의 슬라이더로 표시합니다. `Exposure` 또는 `Gain`을 직접 움직이면 자동 노출이 꺼지고, `White Balance`를 움직이면 자동 화이트밸런스가 꺼집니다. 국내 60 Hz 환경에서는 `Power Line`을 보통 `60` 또는 `AUTO`로 사용합니다. YOLO 창에 포커스를 둔 채 `R` 키를 누르면 기본값으로 돌아갑니다.

카메라 조정 패널이 필요 없으면 다음 파라미터를 추가합니다.

```bash
-p show_camera_controls:=false
```

기본 RealSense 노드 이름은 `/camera/camera`입니다. 카메라 노드 이름을 바꿔 실행했다면 YOLO에도 `-p camera_node_name:=/변경한/노드이름`을 전달해야 합니다. 이 패널은 탐지 영상의 밝기와 색을 보정하는 RGB 센서 설정만 다루며, 거리값을 만드는 depth 센서의 노출·레이저 출력은 변경하지 않습니다.

터미널 3에서 Line Analyzer를 실행합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step yolo_line_analyzer
```

터미널 4에서 Line Path Visualizer를 실행합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step line_path_visualizer
```

알고리즘 파트에 라인 주행 명령을 보내려면 별도 터미널에서 Line Navigation Controller를 실행합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step line_navigation_controller
```

출력은 `/navigation/line_command`의 JSON이며 실제 모터를 직접 구동하지 않습니다. 알고리즘/보행 노드가 이 토픽을 구독하고 미션 상태와 로봇별 보행 한계를 확인한 뒤 실행해야 합니다.

터미널 5에서 Ball Analyzer를 실행합니다.
공까지의 거리를 쓰려면 터미널 1에서 `align_depth.enable:=true`가 켜져 있어야 합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step ball_analyzer
```

허들 미션을 시험할 때는 별도 터미널에서 analyzer와 controller를 실행합니다.
화면 패널을 허들로 고정하려면 YOLO 실행 명령에 `-p metrics_mode:=hurdle`을 추가합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step hurdle_analyzer
```

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step hurdle_navigation_controller
```

현재 임시 `GO` 조건은 카메라 바로 아래 바닥점과 허들 사이 거리 `0.10m ±0.10m`, 화면 하단과 허들 bbox 하단의 depth 환산 간격 `0.05m 이하`, 카메라와 허들의 평행 오차 `±8°`입니다. 허들은 화면 중앙으로 맞출 필요가 없습니다. 바닥 거리는 카메라 높이 `0.70m`, 허들 depth 측정점 높이 `0.10m`와 depth 기반 3D 직선거리를 피타고라스 정리에 적용해 계산합니다. 실제 치수와 모션 위치가 정해지면 `camera_height_m`, `hurdle_reference_height_m`, `go_target_ground_gap_m`, `go_ground_gap_tolerance_m`, `go_max_camera_bottom_gap_m`, `go_angle_tolerance_deg` 파라미터로 조정합니다.

터미널 6에서 RGB-D Visual Odometry 실험 노드를 실행합니다.
이 노드는 공/골대 같은 고정 객체가 안 보이는 구간에서 주변 특징점으로 상대 이동을 추정하는 테스트용입니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step rgbd_visual_odometry
```

터미널 7에서 IMU + Line Pose Estimator를 실행합니다.
처음 1.5초 정도는 gyro bias 보정을 위해 카메라/로봇을 가만히 두는 것이 좋습니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step imu_line_pose_estimator
```

터미널 8에서 Mission State Estimator를 실행합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step mission_state_estimator
```

터미널 9에서 Mission Map Visualizer를 실행합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step mission_map_visualizer
```

이미 RealSense, YOLO26 detector, line analyzer, line path visualizer, IMU pose estimator, mission state estimator를 켜둔 상태에서 맵 창만 추가로 열고 싶다면 새 터미널 하나를 더 열고 아래만 실행하면 됩니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step mission_map_visualizer
```

선택으로 터미널 디버그 모니터를 실행할 수 있습니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step line_debug_monitor
```

## 노드 설명

- `yolo26_detector`: RealSense RGB 영상에서 YOLO26 객체를 탐지하고 `/vision/detections` 발행
- `yolo_line_analyzer`: `/vision/detections`에서 `line`만 분석하여 `/vision/line_info` 발행
- `ball_analyzer`: `/vision/detections`의 `ball`과 aligned depth를 분석하여 `/vision/ball_info` 발행
- `rgbd_visual_odometry`: RGB-D 특징점 추적과 RANSAC으로 `/vision/visual_odom` 발행
- `line_debug_monitor`: `/vision/line_info`를 터미널에서 읽기 쉽게 표시
- `line_path_visualizer`: line 경로, heading, offset, quality를 OpenCV 창에 표시
- `line_navigation_controller`: line 정보를 직진/좌회전/우회전과 속도/이동량 명령으로 변환
- `ball_navigation_controller`: ball 정보를 정렬/접근/집기 후보 명령으로 변환
- `goal_analyzer`: goal 탐지와 aligned depth를 `/vision/goal_info`로 정리
- `goal_navigation_controller`: goal 정보를 SDK 골넣기 행동 후보로 변환
- `hurdle_analyzer`: hurdle과 aligned depth를 점프 준비 정보로 정리
- `hurdle_navigation_controller`: hurdle 정보를 SDK 점프 행동 후보로 변환
- `unified_vision_node`: 기존 네 analyzer를 한 프로세스에서 실행
- `mission_control/motion_decision_node`: 네 planner 중 현재 미션에 맞는 하나를 선택해 통합 명령 발행
- `imu_line_pose_estimator`: RealSense gyro와 line 정보를 이용해 `/vision/robot_pose` 발행
- `mission_state_estimator`: line/object 정보를 이용해 현재 mission state를 `/vision/mission_state`로 발행
- `mission_map_visualizer`: ㄹ자 경기장 미니맵, 공/골대 위치, start/finish, robot pose, mission flow를 표시

## 조정 항목

조명, 카메라 높이와 각도에 따라 다음 값을 조정할 수 있습니다.

- YOLO confidence threshold
- Line analyzer ROI 범위
- Path continuity filter threshold
- Temporal filter window와 EMA alpha
- Mission state 전환 조건
- YOLO 화면 아래 RGB 카메라 실시간 조정 패널

## 주의

- 현재 코드는 비전 상태를 계산하고 시각화합니다.
- `/navigation/line_command`와 `/navigation/ball_command`는 추상 목표값이며 로봇 구동부를 직접 움직이지 않습니다.
- 실행 전에 카메라 토픽이 정상적으로 발행되는지 확인하세요.
- `mission_state_estimator`는 현재 기본 FSM 뼈대이며, 실제 경기장 테스트 후 전환 조건을 더 구체화해야 합니다.

## 포함 파일

- `src/step/step/yolo26_detector.py`: YOLO26 ONNX 객체 탐지와 ROS 토픽 발행
- `src/step/step/yolo_line_analyzer.py`: YOLO26 `line` 탐지 결과를 경로와 방향 정보로 정리
- `src/step/step/ball_analyzer.py`: YOLO26 `ball` 탐지와 RealSense aligned depth를 공 접근/집기 정보로 정리
- `src/step/step/rgbd_visual_odometry.py`: RGB-D 특징점 추적 기반 상대 위치추정 실험 노드
- `src/step/step/line_debug_monitor.py`: `/vision/line_info`를 터미널에서 요약 표시
- `src/step/step/line_path_visualizer.py`: `/vision/line_info`와 카메라 이미지를 시각화
- `src/step/step/line_navigation_planner.py`: line 주행 목표를 계산하는 하드웨어 독립 알고리즘
- `src/step/step/line_navigation_controller.py`: line 정보를 알고리즘용 주행 명령으로 변환
- `src/step/step/ball_navigation_planner.py`: ball 정렬과 접근 목표를 계산하는 하드웨어 독립 알고리즘
- `src/step/step/ball_navigation_controller.py`: ball 정보를 알고리즘용 접근 명령으로 변환
- `src/step/step/goal_analyzer.py`: goal 위치와 depth 및 골넣기 조건 분석
- `src/step/step/goal_navigation_planner.py`: SDK용 골대 정렬/거리/득점 행동 판단
- `src/step/step/goal_navigation_controller.py`: `/navigation/goal_command` 발행
- `src/step/step/hurdle_analyzer.py`: hurdle 위치, depth, 폭과 기울기 분석
- `src/step/step/hurdle_navigation_planner.py`: SDK용 허들 정렬/거리/GO 판단
- `src/step/step/hurdle_navigation_controller.py`: `/navigation/hurdle_command` 발행
- `src/step/step/unified_vision_node.py`: 기존 analyzer 파일을 한 프로세스로 구성
- `src/mission_control/mission_control/motion_decision_planner.py`: 미션 단계별 planner 선택과 명령 정규화
- `src/mission_control/mission_control/motion_decision_node.py`: `/navigation/motion_command` 단일 발행
- `src/mission_control/docs/MOTION_DECISION_SPEC.md`: SDK 연동을 포함한 판단 FSM 구현 범위와 후속 계약
- `src/step/step/imu_line_pose_estimator.py`: RealSense gyro와 line 기반 실시간 위치추정
- `src/step/step/mission_state_estimator.py`: 현재 mission state 추정
- `src/step/step/mission_map_visualizer.py`: 경기장 미니맵과 mission flow 시각화
- `src/step/setup.py`: ROS 2 Python 노드 등록
- `src/step/package.xml`: ROS 2 패키지 정보와 의존성

## YOLO26 비전 파이프라인

현재 추가로 구현한 YOLO26 기반 비전 파이프라인은 RealSense RGB 영상에서 객체를 탐지하고, `line` 객체만 다시 분석해서 주행 알고리즘이 사용할 수 있는 선 정보로 정리합니다.

YOLO26 클래스는 다음과 같습니다.

```text
line
ball
goal
backboard
hurdle
```

데이터 흐름은 다음과 같습니다.

```text
RealSense D435i
    ↓
/camera/color/image_raw
    ↓
yolo26_detector
    ↓
/vision/detections
    ├── yolo_line_analyzer
    └── ball_analyzer
            ↓
       /vision/ball_info
            ↓
       ball_navigation_controller → /navigation/ball_command

    └── goal_analyzer
            ↓
       /vision/goal_info
            ↓
       goal_navigation_controller → /navigation/goal_command

    └── hurdle_analyzer
            ↓
       /vision/hurdle_info
            ↓
       hurdle_navigation_controller → /navigation/hurdle_command

yolo_line_analyzer
    ↓
/vision/line_info
    ├── line_debug_monitor
    ├── line_path_visualizer
    ├── line_navigation_controller → /navigation/line_command
    ├── imu_line_pose_estimator
    └── mission_state_estimator
            ↓
       /vision/mission_state
            ↓
       mission_map_visualizer

RealSense D435i RGB-D
    ↓
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
    ↓
rgbd_visual_odometry
    ↓
/vision/visual_odom

RealSense D435i IMU
    ↓
/camera/camera/gyro/sample
    ↓
imu_line_pose_estimator
    ↓
/vision/robot_pose
    ↓
mission_map_visualizer
```

## 라인 주행 명령

`line_navigation_controller`는 `/vision/line_info`의 필터링된 heading, lateral offset, 먼 경로의 turn angle과 세 가지 quality를 사용합니다. 출력 `/navigation/line_command`의 주요 필드는 다음과 같습니다.

```text
valid                       명령 사용 가능 여부
motion                      STOP / STRAIGHT / LEFT / RIGHT / RECOVER_LEFT / RECOVER_RIGHT
reason                      명령 또는 정지 이유
linear_speed_mps             목표 선속도
lateral_speed_mps            라인 복귀용 좌우 속도(+오른쪽, -왼쪽)
angular_speed_rad_s          목표 각속도(+ 우회전, - 좌회전)
angular_accel_rad_s2         제한된 각가속도
command_duration_sec         이 명령의 권장 유지시간
travel_distance_m            유지시간 동안의 예상 직진 이동량
lateral_travel_distance_m    유지시간 동안의 예상 좌우 이동량
target_heading_change_deg    유지시간 동안의 예상 회전량
steering_error_deg           heading/offset/preview를 합친 조향 오차
line_quality                 사용된 최소 line quality
valid_for_sec                수신 측 watchdog 유효시간
```

라인 미검출, 낮은 quality, 잘못된 값 또는 입력 timeout이면 `valid=false`, `motion=STOP`을 발행합니다. 기본 최대 선속도는 기존 휴머노이드 nominal speed에 맞춘 `0.05m/s`이며, 급커브일수록 자동 감속합니다. 로봇 실측에 맞춰 다음처럼 파라미터를 조정할 수 있습니다.

라인 중심 offset 절댓값이 기본 `0.28` 이상이면 일반 회전과 구분하여 `RECOVER_LEFT` 또는 `RECOVER_RIGHT`를 발행합니다. 이때 전진과 커브 선행 보정은 멈추고 `lateral_speed_mps`만 사용합니다. offset이 `0.16` 안쪽으로 복귀할 때까지 복귀 상태를 유지하므로 경계에서 명령이 흔들리지 않습니다. 먼 커브 보정은 turn angle이 `8도` 이상이고 consistency가 `0.55` 이상일 때만 적용합니다.

```bash
ros2 run step line_navigation_controller --ros-args \
  -p max_linear_speed_mps:=0.04 \
  -p max_angular_speed_rad_s:=0.45 \
  -p max_angular_accel_rad_s2:=0.8 \
  -p command_duration_sec:=0.5
```

방향 부호는 기존 line analyzer와 같습니다. 화면상 경로가 오른쪽이면 heading/offset이 양수이고 `RIGHT` 및 양의 각속도가 출력됩니다. 보행 알고리즘에서는 `command_id`가 새로 들어올 때 기존 목표를 새 값으로 교체해야 하며, `travel_distance_m`을 매 메시지마다 큐에 누적하면 안 됩니다. 또한 `valid_for_sec` 안에 새 명령이 없으면 자체적으로도 정지시키는 watchdog을 두는 것이 좋습니다.

`line_path_visualizer`는 기본적으로 최종 방향, 속도, 회전속도, 다음 이동량과 조향 계산식만 간단히 표시합니다. 기존의 모든 디버그 값과 점 번호가 필요할 때만 다음 옵션을 켭니다.

```bash
ros2 run step line_path_visualizer --ros-args \
  -p show_all_metrics:=true \
  -p show_point_numbers:=true \
  -p show_geometry_labels:=true
```


## 공 탐지와 depth 정보

`ball_analyzer`는 `yolo26_detector`가 발행한 `/vision/detections`에서 `ball`만 골라내고, RealSense aligned depth 이미지에서 공 중심 주변의 median depth를 계산합니다.

`yolo26_detector`의 기본 영상 창 오른쪽 위에는 `ball_analyzer`가
발행한 거리, 수평 오차, 방향, 각도와 좌우 거리가 표시됩니다. 노란색
세로선은 화면 중심이고 주황색 화살표는 화면 중심에서 공 중심까지의
수평 오차입니다. 이 패널은 `show_ball_metrics:=false`로 끌 수 있습니다.

입력 토픽은 다음과 같습니다.

```text
/vision/detections
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

출력 토픽은 다음과 같습니다.

```text
/vision/ball_info
```

`/vision/ball_info`에는 다음 정보가 들어갑니다.

```text
detected
state                 SEARCH / NO_DEPTH / FAR / TRACK / APPROACH / PICKUP_READY / PICKUP_NOW
confidence
center_x, center_y
offset_x_px, offset_y_px
offset_x_norm, offset_y_norm
horizontal_direction      LEFT / CENTER / RIGHT
bearing_deg               카메라 정면 대비 좌우 각도(+오른쪽, -왼쪽)
elevation_deg             카메라 정면 대비 상하 각도(+아래, -위)
depth_m                   카메라 정면축(z) 방향 거리
lateral_offset_m          카메라 중심축에서 좌우 거리(+오른쪽, -왼쪽)
vertical_offset_m         카메라 중심축에서 상하 거리(+아래, -위)
horizontal_distance_m     수평면에서 카메라와 공 사이 거리
distance_m                카메라에서 공까지 3차원 직선거리
camera_info_ready         각도와 3D 좌표 계산 가능 여부
is_centered
is_close
approach_ready
pickup_ready
pickup_now
is_in_pickup_window
pickup_target_x_ratio, pickup_target_y_ratio
pickup_x_tolerance_norm, pickup_y_tolerance_ratio
target_priority_score
candidate_count
```

초기 거리 기준은 다음처럼 잡았습니다.

```text
detect_depth_m        3.0m 이하이면 추적 기억을 시작
approach_depth_m      0.90m 이하이면 공 제어 전환 후보
pickup_ready_depth_m  0.9m 이하 + 화면 중앙이면 집기 자세 준비
pickup_now_depth_m    공 줍기 기준 depth 0.80m
pickup_depth_tolerance_m depth 양방향 허용오차 ±0.05m
pickup_center_tolerance_norm 화면 중심 허용오차 ±0.08
pickup_target_y_ratio 화면 위에서 아래로 0.82 지점
pickup_y_tolerance_ratio 세로 목표 허용오차 ±0.12
horizontal_deadband_px 화면 중심 ±20px는 방향을 CENTER로 판단
center_tolerance_px   화면 중심 ±140px 이내이면 중앙 정렬로 판단
```

공이 보이더라도 `pickup_ready`는 가까운 depth와 화면 내부의 집기 목표 범위 조건이 맞아야만 `true`가 됩니다. 계산상 임시 목표점은 화면 가로 중앙, 화면 높이의 82% 지점이지만 일반 실행 화면에는 별도의 `PICKUP TARGET` 사각형을 표시하지 않습니다. `pickup_now`는 depth가 `0.80m ±0.05m`, 수평 오차가 `±0.08`, 세로 위치가 `0.82 ±0.12`에 모두 들어올 때만 `true`가 되고, 이때 `PICK UP BALL` 문구가 표시됩니다. 카메라 목 각도와 실제 집기 자세가 확정되면 세로 목표값은 반드시 실측 조정해야 합니다.

다만 최종 모션 실행 여부는 이 노드가 아니라 mission/behavior 쪽에서 결정해야 합니다. 예를 들어 현재 미션이 `SCORE_GOAL_A`이면 `ball_info.pickup_ready`가 true여도 공줍기 명령을 무시하고, 현재 미션이 `PICK_BALL_A` 또는 `PICK_BALL_B`일 때만 공 모션을 허용하는 방식이 안전합니다.

실행 명령어는 다음과 같습니다.

```bash
ros2 run step ball_analyzer
```

공을 너무 멀리서부터 접근 대상으로 잡으면 `approach_depth_m`을 줄입니다.

```bash
ros2 run step ball_analyzer --ros-args \
  -p approach_depth_m:=0.9
```

공을 집기 직전 판정이 너무 빡빡하면 목표 depth와 허용오차, 수평 중심 허용오차를 조정합니다.

```bash
ros2 run step ball_analyzer --ros-args \
  -p pickup_ready_depth_m:=0.9 \
  -p pickup_now_depth_m:=0.8 \
  -p pickup_depth_tolerance_m:=0.05 \
  -p pickup_center_tolerance_norm:=0.08 \
  -p pickup_target_y_ratio:=0.82 \
  -p pickup_y_tolerance_ratio:=0.12
```

### 공 접근 명령

라인의 `line_navigation_planner.py`와
`line_navigation_controller.py`에 대응하도록 공도 두 계층으로 분리했습니다.

```text
/vision/ball_info
    ↓
ball_navigation_planner       순수 계산 함수, ROS/하드웨어 의존 없음
    ↓
ball_navigation_controller    timeout과 주기 발행 담당
    ↓
/navigation/ball_command
```

주요 명령은 다음과 같습니다.

```text
valid                       명령 사용 가능 여부
motion                      STOP / TURN_LEFT / TURN_RIGHT / APPROACH /
                            SLOW_APPROACH / FINE_FORWARD_STEP / PICKUP_NOW
reason                      명령 또는 정지 이유
linear_speed_mps             목표 전진속도
lateral_speed_mps            현재 0.0, 향후 옆걸음 확장용
angular_speed_rad_s          목표 각속도(+오른쪽, -왼쪽)
angular_accel_rad_s2         제한된 각가속도
command_duration_sec         명령 권장 유지시간
travel_distance_m            유지시간 동안 예상 전진 이동량
target_heading_change_deg    유지시간 동안 예상 회전량
bearing_error_deg            공 방향 오차(+오른쪽, -왼쪽)
offset_x_norm                화면 중심 기준 정규화 오차
depth_m                      카메라 정면축 거리
distance_error_m             집기 기준까지 남은 전방 거리
pickup_ready, pickup_now     집기 상태
valid_for_sec                수신 측 watchdog 유효시간
```

실행과 확인은 다음과 같습니다.

```bash
ros2 run step ball_navigation_controller
ros2 topic echo /navigation/ball_command
```

기본 동작은 공이 0.90m 안으로 들어온 뒤에만 시작합니다. 공이 좌우로
벗어나면 `TURN_LEFT/RIGHT`로 제자리 정렬하고, 정렬 후 `APPROACH`로
직진하며, 1.0m 안쪽에서 감속합니다. 0.95m 안쪽에서는 연속 보행 대신
`FINE_FORWARD_STEP` 잔발걸음 후보를 내고, 화면 목표 영역과
`0.80m ±0.05m` 조건을 모두 만족하면 전진을 멈추고 `PICKUP_NOW`
후보를 냅니다. 이 출력은 로봇을 직접 움직이지 않으며,
추후 behavior/FSM이 공 미션일 때만 선택해야 합니다. 매핑 및 위치추정
코드와는 연결하지 않았습니다.

### 공 거리 우선순위와 비활성화된 분실 복구

통합 `motion_decision_node`는 공을 처음 보았다는 이유만으로 라인보다
공을 우선하지 않습니다. 현재 거리 기반 전환 순서는 다음과 같습니다.

```text
공 미검출                       → line 주행
공 검출, 거리 3.0m 초과         → line 주행, 추적 시작 안 함
공 검출, Depth > 0.90m         → line 주행 유지
공 검출, Depth <= 0.90m        → ball planner로 전환
공이 화면에서 사라짐             → 즉시 line 판단으로 복귀
```

0.90m 공 제어 전환은 화면의 `Depth Z`와 동일한 `depth_m`을 기준으로
판단합니다. 테스트 중 공이 화면에서 사라질 때마다 제자리 탐색하는 것을
막기 위해 `enable_ball_lost_recovery=false`가 기본값입니다. 나중에
복구 전략이 확정되면 `true`로 다시 켤 수 있으며 관련 설정은
`motion_decision_node`의
`ball_tracking_range_m`, `ball_control_range_m`, `ball_lost_stop_sec`,
`ball_recovery_timeout_sec`, `ball_recovery_turn_rad_s`,
`ball_reacquire_center_deg`, `ball_reacquire_center_norm` 파라미터로
조정할 수 있습니다.

## 골대 분석과 SDK 행동 신호

골대도 공과 같은 세 계층으로 구성합니다.

```text
/vision/detections + aligned depth + camera info
    ↓
goal_analyzer
    ↓ /vision/goal_info
goal_navigation_planner
    ↓
goal_navigation_controller
    ↓ /navigation/goal_command
SDK behavior/FSM
```

`goal_analyzer`는 `class_name=goal`만 선택합니다. 골대 중앙이 빈 공간일
수 있으므로 `backboard`가 goal bbox 안에 함께 검출되면 backboard의
중심과 bbox를 정렬 및 depth 기준으로 사용합니다. backboard가 없을
때만 goal 중심으로 대체합니다. 기준 bbox 내부 5개 지점의 depth를
측정하고 유효한 지점들의 중앙값을 사용합니다. 주요 출력은 다음과
같습니다.

```text
detected, state, confidence
aim_source, aim_bbox        backboard 우선, 없으면 goal
center_x, center_y, bbox
offset_x_px, offset_x_norm
horizontal_direction       LEFT / CENTER / RIGHT
bearing_deg                +오른쪽, -왼쪽
depth_m                    카메라 광축 Z 거리
distance_m                 카메라와 측정 지점 사이 직선거리
lateral_offset_m           카메라 중심축 기준 좌우거리
depth_sample_count         depth를 얻은 유효 샘플 지점 수
is_centered
depth_in_score_range
score_depth_error_m
score_now
```

실제 골넣기 모션이 정해지기 전까지 임시 조건은 다음과 같습니다.

```text
goal_tracking_range_m          3.0m 이내에서 backboard 위치 기억
control_start_depth_m          0.50m 이내에서 골대 보행/정렬 우선
score_target_depth_m          0.25m
score_depth_tolerance_m       ±0.05m
score_center_tolerance_norm   ±0.10
```

거리와 중심 조건이 모두 맞으면 `score_now=true`가 되고 최종 명령은
`action=SHOT`, `sdk_motion_requested=true`가 됩니다. 그 외에는
`ALIGN_LEFT`, `ALIGN_RIGHT`, `APPROACH_GOAL`, `RETREAT_GOAL`, `WAIT` 중
하나를 내지만 속도값은 만들지 않습니다.

통합 판단의 골대 상태 흐름은 다음과 같습니다.

```text
Depth Z > 3.0m               → 골대 기억 안 함, line 주행
0.50m < Depth Z <= 3.0m     → backboard 좌우 위치 기억, line 주행 유지
Depth Z <= 0.50m            → goal planner 우선
중앙에서 벗어남              → ALIGN_LEFT / ALIGN_RIGHT
중앙이고 0.30m보다 멂        → APPROACH_GOAL 보행모션 후보
중앙이고 0.25m ±0.05m       → SHOT 단발 모션 요청
기억한 골대 분실 직후         → GOAL_LOST_STOP, 선속도 0
0.35초 후에도 미검출          → RECOVER_GOAL_TURN_LEFT / RIGHT
골대 재검출, 중앙 밖          → 중앙 정렬까지 제자리 회전 유지
8초 동안 재검출 실패          → 골대 기억 해제 후 line 복귀
```

기억과 정렬은 큰 `goal` bbox가 아니라 가능하면 내부 `backboard`의
`depth_m`, `bearing_deg`, `offset_x_norm`을 사용합니다. 통합 출력의
`goal_tracking`에서 활성 여부, 분실 시간과 마지막 방향을 확인할 수
있습니다. 공을 집은 뒤 골대 단계에서는 `/mission/phase`를
`GOAL_SEARCH`로 지정해야 이전 공 기억보다 골대 탐색을 확실하게
우선할 수 있습니다.

```bash
ros2 run step goal_analyzer
ros2 run step goal_navigation_controller
ros2 topic echo /vision/goal_info
ros2 topic echo /navigation/goal_command
```

이 값은 SDK 모션을 직접 실행하지 않는 행동 후보입니다. 실제 SDK 호출은
현재 미션이 골넣기 단계이고 `sdk_motion_requested=true`일 때 behavior/FSM
한 곳에서만 수행해야 합니다. 매핑 및 위치추정 코드에는 연결하지
않았습니다.

## 허들 분석과 넘기 준비 신호

허들은 화면 중앙 위치를 정렬 기준으로 사용하지 않습니다. 가로로 긴 bbox의 좌우 depth 차이로 카메라와 허들의 평행 오차를 계산하고, 이 각도와 대표 depth만으로 접근 및 넘기 준비를 판단합니다.

```text
/vision/detections + aligned depth + camera info
    ↓
hurdle_analyzer
    ↓ /vision/hurdle_info
hurdle_navigation_planner
    ↓
hurdle_navigation_controller 또는 motion_decision_node
    ↓ /navigation/hurdle_command 또는 /navigation/motion_command
SDK behavior/FSM
```

`hurdle_analyzer`는 hurdle bbox 높이의 약 55% 지점에서 가로 방향 5개 위치의 depth patch를 측정합니다. 각 patch는 유효 depth의 median을 사용하고, 전체 대표 depth도 유효한 5개 값의 median으로 정해 일부 구멍이나 반사 노이즈 영향을 줄입니다.

주요 출력값은 다음과 같습니다.

```text
detected, state, confidence
center_x, center_y, bbox
depth_m                         카메라 광축 Z 거리
distance_m                      depth와 픽셀 좌표로 복원한 3D 직선거리
ground_gap_m                    카메라 아래 바닥점과 허들 사이 거리
camera_bottom_gap_px            화면 하단과 hurdle bbox 하단의 픽셀 간격
camera_bottom_gap_m             위 픽셀 간격을 depth로 환산한 길이
estimated_width_m               bbox 폭과 depth로 계산한 허들 추정 폭
left_depth_m, right_depth_m      허들 좌우 대표 depth
hurdle_angle_deg                좌우 depth 차이로 계산한 평행 오차
depth_sample_count              유효한 가로 depth 측정점 개수
is_parallel
ground_gap_in_go_range
go_ground_gap_error_m
go_now
```

현재 임시 넘기 준비 조건은 다음과 같습니다.

```text
camera_height_m                  0.70m
hurdle_reference_height_m        0.10m
go_target_ground_gap_m           0.10m
go_ground_gap_tolerance_m        ±0.10m → 허용 범위 0.00~0.20m
go_max_camera_bottom_gap_m       0.05m
go_angle_tolerance_deg           ±8.0°
```

카메라–허들 3D 직선거리를 빗변, `camera_height_m - hurdle_reference_height_m`를 세로변으로 두고 피타고라스 정리로 `ground_gap_m`을 계산합니다. 빗변이 세로변보다 짧게 측정되는 불가능한 기하 상태는 절대 0m로 자르지 않고 `N/A`로 처리합니다. 이때도 잘못된 조기 `GO`를 막기 위해 화면 하단부터 허들 bbox 하단까지의 픽셀 간격을 `depth / fy` 비율로 실제 길이로 환산한 `camera_bottom_gap_m`이 반드시 0.05m 이하인지 함께 검사합니다. 허들이 카메라와 평행하고 거리 조건들이 모두 맞아야 `state=GO_READY`, `go_now=true`, planner의 `action=GO`가 됩니다. 평행하지 않으면 `ALIGN_LEFT/ALIGN_RIGHT`, 하단 간격 또는 바닥 간격이 크면 `APPROACH_HURDLE`, 필수 depth·하단 간격·좌우 각도를 계산할 수 없으면 `WAIT`를 사용합니다. 허들의 화면 좌우 위치는 이 판단에 영향을 주지 않습니다.

통합 `motion_decision_node`는 `GO` 조건이 여러 프레임 유지되더라도 `sdk_motion_requested=true`를 최초 진입 시 한 번만 발행합니다. 실제 모션 ID는 아직 정해지지 않아 `sdk_motion_id=null`입니다.

```bash
ros2 run step hurdle_analyzer
ros2 run step hurdle_navigation_controller
ros2 topic echo /vision/hurdle_info
ros2 topic echo /navigation/hurdle_command
```

현재 바닥 간격 `0.10m ±0.10m` 조건과 카메라·허들 높이는 임시값입니다. 실제 장착 높이와 최종 허들 모션의 시작 위치가 정해지면 파라미터를 실측 보정해야 합니다. 모션이 느리게 접근해 허들에 완전히 붙은 뒤 시작하는 방식으로 정해지면 다음 로직을 추가할 수 있습니다.

```text
ALIGN
→ 일정 depth 이내에서 LOOK_DOWN 요청
→ 카메라 하향 자세 안정화
→ 저속 CREEP 접근
→ 화면 하단과 hurdle bbox 하단의 bottom_gap_px 확인
→ 중심/depth/bottom gap이 연속 시간 동안 안정되면 GO
```

`bottom_gap_px`는 영상상의 픽셀 기준이고, RealSense 실제 거리는 `m`로 유지합니다. 카메라 하향 각도와 로봇 자세가 고정되어야 픽셀 간격을 반복 가능한 최종 접근 기준으로 사용할 수 있습니다.

## 통합 비전 실행과 미션 의사결정

### `unified_vision_node`

기존 analyzer 파일을 삭제하거나 한 파일로 복사하지 않고 다음 네 클래스를 한 Python 프로세스의 `MultiThreadedExecutor`에서 실행합니다.

```text
YoloLineAnalyzer
BallAnalyzer
GoalAnalyzer
HurdleAnalyzer
```

따라서 알고리즘 파일은 개별 테스트에 사용할 수 있고, 경기 실행 시에는 `ros2 run step unified_vision_node` 한 명령만 사용합니다. 현재 방식은 **한 프로세스 안의 네 ROS Node 구성**이며, 완전히 하나의 ROS Node로 바꾸려면 각 analyzer에서 ROS 구독부와 순수 계산부를 `Core` 클래스로 추가 분리해야 합니다.

### `mission_control/motion_decision_node`

`motion_decision_node`는 `/vision/line_info`, `/vision/ball_info`, `/vision/goal_info`, `/vision/hurdle_info`를 한 곳에서 받고 현재 `/mission/phase`에 해당하는 planner 하나만 실행합니다.

| phase 형태 | 동작 |
|---|---|
| `AUTO` | 공은 0.90m, 골대는 0.50m 안에서만 우선; 공이 사라지면 line으로 즉시 복귀 |
| `BALL_SEARCH` | 0.90m 밖에서는 line 주행, 0.90m 안에서 ball planner 전환; 기본 분실 회전 없음 |
| `GOAL_SEARCH` | 0.50~3m backboard는 기억하면서 line 주행, 0.50m 안에서 goal planner 전환, 분실 시 마지막 방향 회전 |
| `HURDLE_SEARCH` | 목표가 보이면 hurdle planner, 아직 안 보이면 line planner로 주행하며 탐색 |
| `BALL_APPROACH`, `GOAL_APPROACH`, `HURDLE_APPROACH` | 해당 객체 planner에 집중 |
| `LINE_TRACK` | line planner만 사용 |
| `PICK_LOCK`, `SHOOT_LOCK`, `HURDLE_LOCK` | C++ SDK 모션이 끝날 때까지 `WAIT`; 새 이동 판단 차단 |

확정된 허들이 보이면 화면 중앙 여부와 관계없이 공·골대보다 hurdle
planner를 먼저 선택합니다. 허들을 화면 중앙으로 옮기는 대신 좌우 depth
차이로 계산한 `hurdle_angle_deg`가 허용 범위에 들도록 제자리 회전하고,
그 다음 depth를 맞춥니다. SDK 모션이 이미 실행 중인 `*_LOCK` 상태는
이 우선순위보다 먼저 적용됩니다.

모든 입력은 기본 0.5초 timeout을 사용합니다. 오래된 정보는 선택 대상에서 제외하므로 멈춘 analyzer의 마지막 검출값으로 계속 움직이는 것을 방지합니다.

통합 출력 `/navigation/motion_command`의 공통 구조는 다음과 같습니다.

```text
phase                       현재 미션 단계
source                      none / line / ball / goal / hurdle
action                      선택된 planner의 정규화 명령
valid, reason               명령 유효 여부와 판단 이유
source_command              기존 planner의 상세 원본 출력
command_id                  매 주기 증가하는 메시지 번호
event_id                    PICKUP/SCORE/GO 같은 단발 이벤트 번호
sdk_motion_requested        단발 SDK 요청; 조건 진입 첫 프레임만 true
request_latched             단발 이벤트 조건이 현재 유지 중인지 여부
sdk_motion_id               현재 null; C++ SDK 계약 후 설정
input_age_sec               네 입력 토픽의 최신 데이터 나이
ball_tracking               현재 기본 비활성; 향후 분실 복구용 상태
goal_tracking               3m backboard 기억, 분실 시간, 마지막 좌우 방향
```

현재 통합 노드는 친구가 설계한 전체 `motion_decision`의 입력/선택 뼈대입니다. 아래 항목은 SDK 인터페이스가 확정된 뒤 추가해야 합니다.

- 실제 모션 번호 매핑과 `angle`, `custom_param` 필드
- 최초 강제 전진과 비상 정지
- `total_fail_count` 기반 무한 루프 탈출
- C++ `busy/end/fail` 응답 기반 mission lock 해제
- 공 줍기 실패 후 목 스캔과 가변 후진
- 슛 준비/투척/팔 회수 시퀀스
- 허들 착지 damping과 이전 yaw 복귀

### 통합 화면

YOLO 화면의 `metrics_mode=auto`는 `/navigation/motion_command`의 실제 `source`를 우선합니다.

- line 선택: line bbox 글자를 숨기고 가까운 점부터 먼 점까지 초록 경로, 점 번호, `NEAR/FAR`, 카메라 중심선, heading, lateral offset을 표시. 실제 planner action인 `STRAIGHT`, `LEFT`, `RIGHT`, `RECOVER LINE LEFT/RIGHT`, `STOP`은 화면 위쪽의 큰 청록 배너로도 표시
- ball 선택: 거리, depth, 화면 중심 오차를 표시. 실제 planner action인 `BALL TURN LEFT/RIGHT`, `BALL APPROACH`, `BALL SLOW APPROACH`, `BALL FINE STEP`, `BALL STOP`은 큰 주황 배너, 최종 `PICK UP BALL`은 `GO!`, `SHOT`과 같은 초록 실행 배너로 표시. 공 분실 복구 중에는 BALL 패널을 유지하면서 검출된 line 경로를 배경에 함께 표시
- goal 선택: backboard 우선 중심, 거리, depth, 정렬 상태와 `SHOT` 표시
- hurdle 선택: Depth Z, 계산된 바닥 간격, 화면 하단-허들 간격, 좌우 depth, 폭, 카메라-허들 평행 오차와 `GO!` 표시. 화면 중심 오차와 좌우 위치는 표시하지 않음

### 공통 연속 프레임 객체 확정

단일 프레임의 오검출이 planner 또는 SDK 모션 요청으로 이어지지 않도록
`temporal_confirmation.py`의 `TemporalConfirmationFilter`를 line, ball,
goal, hurdle analyzer가 공통으로 사용합니다.

```text
line             최근 3프레임 중 2회 유효 경로
ball             최근 5프레임 중 3회 같은 위치·크기의 bbox
goal/backboard   최근 5프레임 중 3회 같은 위치·크기의 bbox
hurdle           최근 7프레임 중 5회 같은 위치·크기의 bbox
PICKUP_NOW       최근 5프레임 중 3회 조건 유지
SHOT             최근 5프레임 중 3회 조건 유지
GO               최근 7프레임 중 5회 조건 유지
```

공·골대·허들은 bbox 중심 이동량과 면적 비율도 비교하므로 전혀 다른
위치나 크기의 오검출은 같은 객체의 누적으로 계산하지 않습니다. 짧은
미검출은 기본 2프레임까지 기록을 보존하지만, 현재 프레임에 객체가 없을
때는 stale 위치로 움직이지 않도록 `detected=false`를 발행합니다.

YOLO 원본 bbox에는 확정 단계를 다음처럼 표시합니다.

```text
RAW             analyzer confidence/형상 조건을 아직 통과하지 못함
CANDIDATE 2/5   연속 프레임 확인 진행 중
CONFIRMED       planner 입력으로 사용할 수 있는 확정 객체
```

공통 판정 알고리즘은 `src/step/step/temporal_confirmation.py` 한 파일에서
관리합니다. 대상별 강도는 각 analyzer의 다음 ROS 파라미터로 실행 중
설정할 수 있습니다.

```text
confirmation_window_size
confirmation_required_hits
confirmation_max_missed_frames
confirmation_max_center_shift_norm   # line 제외
confirmation_min_area_ratio          # line 제외
```

최종 모션 유지 프레임 수는 `pickup_confirmation_*`,
`score_confirmation_*`, `go_confirmation_*` 파라미터로 별도 조정합니다.

이 시각화는 기존 analyzer/planner 결과를 그리기만 하며 별도의 판단을 만들지 않습니다. OpenCV 선과 글자 렌더링 비용은 YOLO ONNX 추론보다 작고, 통합 모드에서는 별도 `line_path_visualizer` 창을 켜지 않아도 됩니다.


## RGB-D Visual Odometry 실험

`rgbd_visual_odometry`는 공/골대처럼 이름이 붙은 객체가 안 보이는 구간에서도 주변 특징점을 이용해 카메라의 상대 이동을 추정하기 위한 테스트 노드입니다.

이 노드는 완성된 SLAM이 아니라 lightweight visual odometry 실험입니다.

처리 흐름은 다음과 같습니다.

```text
RGB image
    ↓
Good Features 추출
    ↓
Optical Flow로 다음 프레임 추적
    ↓
Aligned depth로 2D 점을 3D 점으로 변환
    ↓
RANSAC estimateAffine3D로 정적 배경 motion 추정
    ↓
동적 물체/심판 발/사람 움직임 같은 outlier 제거
    ↓
/vision/visual_odom 발행
```

출력 토픽은 다음과 같습니다.

```text
/vision/visual_odom
```

주요 출력값은 다음과 같습니다.

```text
tracking_ok
update_used
x_m, y_m
yaw_deg
total_distance_m
delta_forward_m
delta_lateral_m
delta_yaw_deg
feature_count
tracked_count
depth_pair_count
inlier_count
inlier_ratio
confidence
note
```

`update_used`가 `false`이면 이번 프레임의 이동 추정은 버린 것입니다. 보통 특징점이 부족하거나, depth가 부족하거나, RANSAC inlier 비율이 낮거나, 한 프레임 이동량이 너무 커서 이상치로 판단한 경우입니다.

실행 명령어는 다음과 같습니다.

```bash
ros2 run step rgbd_visual_odometry
```

GUI 디버그 창 없이 토픽만 보고 싶으면 아래처럼 실행합니다.

```bash
ros2 run step rgbd_visual_odometry --ros-args \
  -p display:=false
```

특징점이 너무 적으면 ROI를 넓히거나 feature 조건을 완화합니다.

```bash
ros2 run step rgbd_visual_odometry --ros-args \
  -p roi_y_min_ratio:=0.05 \
  -p max_corners:=700 \
  -p quality_level:=0.005
```

사람 발이나 움직이는 물체 때문에 위치가 튀면 RANSAC 기준을 더 엄격하게 합니다.

```bash
ros2 run step rgbd_visual_odometry --ros-args \
  -p min_inlier_ratio:=0.6 \
  -p ransac_threshold_m:=0.03 \
  -p max_translation_per_frame_m:=0.12
```

주의: 이 노드는 정확한 경기장 절대좌표를 바로 주는 SLAM이 아닙니다. 먼저 “가만히 있으면 거의 안 움직이는지”, “전진/후진/좌우 이동 시 `/vision/visual_odom` 값이 그럴듯하게 변하는지” 확인하는 용도입니다. 안정성이 확인되면 이후 `mission_map_visualizer` 또는 별도 localization 노드에 연결합니다.

## IMU 기반 위치추정

`imu_line_pose_estimator`는 RealSense D435i의 gyro/accel, `/vision/line_info`, `/vision/detections`를 함께 사용해 경기장 미니맵 위의 로봇 위치를 추정합니다.

현재 방식은 완전한 SLAM이 아니라 대회 맵 테스트용 lightweight estimator입니다.

- gyro z축을 적분해서 짧은 시간의 yaw 변화를 추정합니다.
- accel/gyro 변화량으로 실제 카메라 또는 로봇이 움직였는지 판단합니다.
- 기본 실행에서는 line quality와 일정 시간 이상 지속되는 IMU 움직임을 이용해 맵 경로 위 진행도를 추정합니다.
- 짧은 손떨림은 무시하기 위해 `motion_start_sec`, `motion_score_start_threshold`, `motion_score_stop_threshold`로 움직임 판단을 디바운스합니다.
- 실제 이동 거리는 `nominal_forward_speed_mps` 파라미터로 임시 추정합니다.
- line lateral offset을 이용해 점선 경로 중심에서 좌우로 벗어난 추정 위치를 미니맵에 표시합니다.
- `ball`, `goal`, `backboard` landmark 보정은 기본값으로 꺼져 있으며, 필요할 때만 켭니다.
- 결과는 `/vision/robot_pose` JSON 토픽으로 발행됩니다.
- `mission_map_visualizer`는 `/vision/robot_pose`가 들어오면 미니맵 위에 실시간 로봇 위치와 heading 화살표를 표시합니다.

실행 명령어는 다음과 같습니다.

```bash
ros2 run step imu_line_pose_estimator
```

RealSense IMU 토픽 이름이 다르면 `imu_topic`을 직접 지정합니다.

```bash
ros2 run step imu_line_pose_estimator --ros-args \
  -p imu_topic:=/camera/camera/gyro/sample
```

accel 토픽 이름이 다르면 `accel_topic`도 지정합니다.

```bash
ros2 run step imu_line_pose_estimator --ros-args \
  -p imu_topic:=/camera/camera/gyro/sample \
  -p accel_topic:=/camera/camera/accel/sample
```

로봇의 실제 보행 속도에 맞춰 미니맵 진행 속도를 바꾸고 싶으면 아래처럼 조정합니다.

```bash
ros2 run step imu_line_pose_estimator --ros-args \
  -p nominal_forward_speed_mps:=0.05 \
  -p max_estimated_speed_mps:=0.08
```

카메라를 가만히 두었는데도 맵이 전진하면 motion threshold를 올리거나 추정 속도를 더 낮춥니다.

```bash
ros2 run step imu_line_pose_estimator --ros-args \
  -p accel_motion_threshold_mps2:=0.6 \
  -p gyro_motion_threshold_rad_s:=0.12 \
  -p motion_start_sec:=0.5 \
  -p motion_score_start_threshold:=0.7 \
  -p nominal_forward_speed_mps:=0.03 \
  -p max_estimated_speed_mps:=0.05
```

맵 진행을 완전히 멈추고 lateral offset과 heading만 보고 싶으면 아래처럼 실행합니다.

```bash
ros2 run step imu_line_pose_estimator --ros-args \
  -p enable_route_progress:=false
```

주의: IMU만으로는 시간이 지날수록 위치 오차가 쌓입니다. 최종 대회용으로는 공/골대 depth 거리, 발걸음/보행 상태, 체크포인트 통과 조건을 함께 사용해 보정해야 합니다.

현재 미니맵의 `x_m`, `y_m`은 실제 절대 위치가 아니라 route progress와 line lateral offset을 합친 추정 위치입니다. 점선 밖으로 벗어나는지는 `lateral_offset_m`과 `path_deviation_status`로 확인합니다.

라인 기준 좌우가 실제와 반대로 보이면 `lateral_offset_sign`을 바꿉니다.

```bash
ros2 run step imu_line_pose_estimator --ros-args \
  -p lateral_offset_sign:=1.0
```

미니맵의 시선 화살표가 실제 카메라 방향과 반대로 꺾이면 `line_heading_sign`을 바꿉니다.

```bash
ros2 run step imu_line_pose_estimator --ros-args \
  -p line_heading_sign:=1.0
```

현재 landmark 보정은 기본값으로 꺼져 있습니다. `enable_landmark_correction:=true`를 켜면 객체가 보였을 때 맵상의 고정된 공/골대 위치 근처로 `route_progress_m`을 조금씩 당깁니다. depth 없이 켜면 공/골대가 보이는 순간 위치가 튈 수 있으므로, 실제 주행 전에는 꺼둔 상태로 테스트하는 것을 권장합니다.

## Fake motion step 테스트

아직 실제 알고리즘/모션 파트가 없을 때는 `step_motion_pose_test`로 가짜 보행 피드백을 흉내낼 수 있습니다.

이 노드는 `/vision/line_info`를 보고 라인이 안정적으로 보이면, 알고리즘과 모션 파트가 `walk_forward` 명령을 수행했고 한 걸음이 완료되었다고 가정합니다.

- 기본 한 걸음 보폭은 `0.15m`입니다.
- 기본 fake step 주기는 `0.8초`입니다.
- 한 걸음이 완료될 때마다 `/vision/robot_pose`의 `route_progress_m`이 `0.15m` 증가합니다.
- 나중에 실제 모션 파트가 생기면 fake step 대신 실제 step feedback 토픽으로 교체하면 됩니다.

실행할 때는 `imu_line_pose_estimator` 대신 아래 노드를 실행합니다.

```bash
ros2 run step step_motion_pose_test
```

보폭이나 걸음 주기를 바꾸고 싶으면 아래처럼 실행합니다.

```bash
ros2 run step step_motion_pose_test --ros-args \
  -p fake_step_length_m:=0.15 \
  -p fake_step_period_sec:=0.8
```

## 미니맵과 미션 상태

현재 경기장 미니맵은 1칸을 `1m x 1m`로 보는 격자 기반입니다.

- 공 위치는 `BALL A`, `BALL B`로 표시합니다.
- 골대 위치는 `GOAL A`, `GOAL B`로 표시합니다.
- 시작선과 도착선은 각 끝선에서 0.5m 떨어진 보라색 선으로 표시합니다.
- Mission flow는 다음 순서를 기본으로 둡니다.

```text
START
-> WALK_TO_BALL_A
-> PICK_BALL_A
-> SCORE_GOAL_A
-> WALK_TO_BALL_B
-> PICK_BALL_B
-> SCORE_GOAL_B
-> WALK_TO_FINISH
-> FINISH
```

현재는 line/object detection을 이용한 기본 상태 추정만 구현되어 있습니다. 실제 로봇 주행 알고리즘과 연결할 때는 deadband, hysteresis, command smoothing을 적용해야 합니다.

## YOLO26 실행 참고

현재 PC 테스트는 `CPUExecutionProvider` 기준입니다.
`device:=auto`는 TensorRT, CUDA, CPU 순서로 ONNX Runtime 실행 장치를 선택합니다.

현재 `yolo26_detector.py`의 기본 모델은 ROS 패키지와 함께 설치됩니다.

```text
src/step/models/best.onnx
→ install/step/share/step/models/best.onnx
```

다른 모델을 시험할 때만 `model_path` 파라미터로 경로를 직접 지정합니다.

```bash
ros2 run step yolo26_detector --ros-args \
  -p model_path:=/absolute/path/to/best.onnx \
  -p device:=cpu
```

Jetson Orin Nano에서는 추후 `cuda` 또는 `tensorrt` 실행을 테스트할 예정입니다.
