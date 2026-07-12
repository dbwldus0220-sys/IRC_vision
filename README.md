# IRC 로봇 주행 비전

ROS 2와 Intel RealSense D435i 영상에서 YOLO26 객체를 탐지하고, 라인 주행 정보와 경기장 미니맵 상태를 계산하는 IRC 휴머노이드 로봇 대회용 비전 프로젝트입니다.

## 주요 기능

- RealSense RGB 이미지 토픽 구독
- YOLO26 ONNX 기반 `line`, `ball`, `goal`, `backboard`, `hurdle` 탐지
- YOLO `line` 중심점 기반 경로 분석
- 오검출 line 점 제거용 path continuity filter
- heading, lateral offset, curve, quality 정보 계산
- line debug monitor와 path visualizer 제공
- 경기장 ㄹ자 미니맵과 mission state 시각화

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

기본 구독 토픽은 `/camera/camera/color/image_raw`입니다.

## 빌드

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
colcon build --packages-select step --symlink-install
source ~/my_cv/install/setup.bash
```

현재 개발 환경은 ROS 2 Humble 기준입니다.

## 실행

기본 실행은 RealSense, YOLO26 detector, line analyzer, visualizer, mission state, minimap을 나누어 실행합니다.

터미널 1에서 RealSense를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py
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

터미널 5에서 Mission State Estimator를 실행합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step mission_state_estimator
```

터미널 6에서 Mission Map Visualizer를 실행합니다.

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
- `line_debug_monitor`: `/vision/line_info`를 터미널에서 읽기 쉽게 표시
- `line_path_visualizer`: line 경로, heading, offset, quality를 OpenCV 창에 표시
- `mission_state_estimator`: line/object 정보를 이용해 현재 mission state를 `/vision/mission_state`로 발행
- `mission_map_visualizer`: ㄹ자 경기장 미니맵, 공/골대 위치, start/finish, mission flow를 표시

## 조정 항목

조명, 카메라 높이와 각도에 따라 다음 값을 조정할 수 있습니다.

- YOLO confidence threshold
- Line analyzer ROI 범위
- Path continuity filter threshold
- Temporal filter window와 EMA alpha
- Mission state 전환 조건

## 주의

- 현재 코드는 비전 상태를 계산하고 시각화합니다.
- 로봇 구동부에 실제 제어 명령을 발행하는 기능은 아직 포함되어 있지 않습니다.
- 실행 전에 카메라 토픽이 정상적으로 발행되는지 확인하세요.
- `mission_state_estimator`는 현재 기본 FSM 뼈대이며, 실제 경기장 테스트 후 전환 조건을 더 구체화해야 합니다.

## 포함 파일

- `src/step/step/yolo26_detector.py`: YOLO26 ONNX 객체 탐지와 ROS 토픽 발행
- `src/step/step/yolo_line_analyzer.py`: YOLO26 `line` 탐지 결과를 경로와 방향 정보로 정리
- `src/step/step/line_debug_monitor.py`: `/vision/line_info`를 터미널에서 요약 표시
- `src/step/step/line_path_visualizer.py`: `/vision/line_info`와 카메라 이미지를 시각화
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
/camera/camera/color/image_raw
    ↓
yolo26_detector
    ↓
/vision/detections
    ↓
yolo_line_analyzer
    ↓
/vision/line_info
    ├── line_debug_monitor
    ├── line_path_visualizer
    └── mission_state_estimator
            ↓
       /vision/mission_state
            ↓
       mission_map_visualizer
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

현재 `yolo26_detector.py`의 기본 모델 경로는 다음과 같습니다.

```text
/home/geonwoo/Desktop/realsense/dataset/best.onnx
```

다른 PC나 Jetson에서 실행할 때는 필요하면 `model_path` 파라미터로 경로를 직접 지정합니다.

```bash
ros2 run step yolo26_detector --ros-args \
  -p model_path:=/absolute/path/to/best.onnx \
  -p device:=cpu
```

Jetson Orin Nano에서는 추후 `cuda` 또는 `tensorrt` 실행을 테스트할 예정입니다.
