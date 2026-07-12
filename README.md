# IRC 로봇 주행 비전

ROS 2 카메라 영상에서 바닥의 흰색 테이프를 검출하고 로봇의 진행 방향과 조향 각도를 계산하는 OpenCV 기반 비전 프로젝트입니다.

## 주요 기능

- ROS 2 컬러 이미지 토픽 구독
- 관심 영역(ROI) 기반 영상 처리
- 이진화와 모폴로지 연산을 이용한 노이즈 제거
- 흰색 테이프의 윤곽선, 중심점과 각도 검출
- 여러 테이프 중심을 이용한 이동 경로 계산
- 좌회전, 직진, 우회전 방향 시각화

## 실행 환경

- ROS 2
- Python 3
- rclpy
- sensor_msgs
- cv_bridge
- OpenCV
- NumPy
- 컬러 이미지 토픽을 제공하는 카메라 노드

기본 구독 토픽은 `/camera/camera/color/image_raw`입니다.

## 빌드

```bash
source /opt/ros/<ros-distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`<ros-distro>`를 설치된 ROS 2 배포판 이름으로 변경하세요.

## 실행

먼저 RealSense 등의 카메라 노드를 실행한 뒤, 다른 터미널에서 원하는 비전 노드를 실행합니다.

```bash
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
ros2 run step find_ddirect
```

사용 가능한 실행 노드는 다음과 같습니다.

```bash
ros2 run step look_ground
ros2 run step look_gground
ros2 run step find_direct
ros2 run step find_ddirect
```

## 노드 설명

- `look_ground`: 전체 영상에서 테이프 중심과 각도 검출
- `look_gground`: 동적 ROI를 적용한 테이프 검출
- `find_direct`: 여러 테이프 중심의 평균을 이용한 진행 방향 계산
- `find_ddirect`: 테이프 중심 경로와 상대 각도를 이용한 진행 방향 계산

## 조정 항목

조명, 카메라 높이와 각도에 따라 각 Python 파일의 다음 값을 조정할 수 있습니다.

- 이진화 임계값
- ROI 범위
- 윤곽선 최소/최대 면적
- 테이프 종횡비 범위
- 좌회전, 직진, 우회전 판정 각도

## 주의

- 현재 코드는 검출 결과와 방향을 OpenCV 창에 표시합니다.
- 로봇 구동부에 실제 제어 명령을 발행하는 기능은 포함되어 있지 않습니다.
- 실행 전에 카메라 토픽이 정상적으로 발행되는지 확인하세요.

## 포함 파일

- `src/step/step/look_ground.py`: 기본 테이프 검출
- `src/step/step/look_gground.py`: ROI 기반 테이프 검출
- `src/step/step/find_direct.py`: 평균 중심 기반 진행 방향 계산
- `src/step/step/find_ddirect.py`: 경로 상대 각도 기반 진행 방향 계산
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
    └── line_path_visualizer
```

## YOLO26 실행 순서

현재 YOLO26 파이프라인은 5개 터미널로 실행합니다.

- 터미널 1 = RealSense
- 터미널 2 = YOLO26 detector
- 터미널 3 = Line Analyzer
- 터미널 4 = Line Debug Monitor
- 터미널 5 = Line Path Visualizer

먼저 빌드합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
colcon build --packages-select step --symlink-install
source ~/my_cv/install/setup.bash
```

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
ros2 run step yolo26_detector --ros-args -p device:=cpu
```

터미널 3에서 Line Analyzer를 실행합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step yolo_line_analyzer
```

터미널 4에서 Line Debug Monitor를 실행합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step line_debug_monitor
```

터미널 5에서 Line Path Visualizer를 실행합니다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step line_path_visualizer
```

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

## YOLO26 관련 파일

- `src/step/step/yolo26_detector.py`: YOLO26 ONNX 객체 탐지와 ROS 토픽 발행
- `src/step/step/yolo_line_analyzer.py`: YOLO26 `line` 탐지 결과를 경로와 방향 정보로 정리
- `src/step/step/line_debug_monitor.py`: `/vision/line_info`를 터미널에서 요약 표시
- `src/step/step/line_path_visualizer.py`: `/vision/line_info`와 카메라 이미지를 시각화
