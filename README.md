# IRC 로봇 주행 비전

ROS 2와 Intel RealSense 영상에서 YOLO26 객체를 탐지하고, 바닥의 흰색 테이프를 분석하는 로봇 비전 프로젝트입니다.

현재 기본 실행 흐름은 다음과 같습니다.

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

## 주요 기능

- ROS 2 컬러 이미지 토픽 구독
- YOLO26 ONNX 기반 `line`, `ball`, `goal`, `backboard`, `hurdle` 탐지
- TensorRT, CUDA, CPU 순서의 ONNX Runtime 실행 장치 선택
- JSON 탐지 결과와 주석 이미지 ROS 토픽 발행
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
- ONNX Runtime
- 컬러 이미지 토픽을 제공하는 카메라 노드

기본 구독 토픽은 `/camera/camera/color/image_raw`입니다.

## 빌드

```bash
source /opt/ros/<ros-distro>/setup.bash
colcon build --packages-select step --symlink-install
source ~/my_cv/install/setup.bash
```

현재 개발 환경은 ROS 2 Humble이므로 `<ros-distro>`에는 `humble`을 사용합니다. ROS 2 Humble은 Python 3.10을 사용하므로 Conda Python 3.13이 활성화된 터미널에서는 먼저 `conda deactivate`를 실행하세요.

## 실행

현재 기본 시연은 5개 터미널로 실행한다.

1. 터미널 1 = RealSense
2. 터미널 2 = YOLO26 detector
3. 터미널 3 = Line Analyzer
4. 터미널 4 = Line Debug Monitor
5. 터미널 5 = Line Path Visualizer

### 실행 순서

1. 먼저 빌드한다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
colcon build --packages-select step --symlink-install
source ~/my_cv/install/setup.bash
```

2. 터미널 1에서 RealSense를 실행한다.

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

3. 터미널 2에서 YOLO26 detector를 실행한다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step yolo26_detector --ros-args -p device:=cpu
```

4. 터미널 3에서 Line Analyzer를 실행한다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step yolo_line_analyzer
```

5. 터미널 4에서 Line Debug Monitor를 실행한다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step line_debug_monitor
```

6. 터미널 5에서 Line Path Visualizer를 실행한다.

```bash
cd ~/my_cv
source /opt/ros/humble/setup.bash
source ~/my_cv/install/setup.bash
ros2 run step line_path_visualizer
```

### 실행 방법

YOLO26 detector는 기본적으로 `/camera/camera/color/image_raw`를 구독한다.
`yolo_line_analyzer`는 `/vision/detections`를 받아 `/vision/line_info`를 만든다.
`line_debug_monitor`와 `line_path_visualizer`는 이 `/vision/line_info`를 함께 사용한다.

현재 `yolo26_detector.py`의 기본 모델 경로는 코드 기준으로 다음 값이다.

```text
/home/geonwoo/Desktop/realsense/dataset/best.onnx
```

따라서 다른 PC나 Jetson에서 실행할 때는 필요하면 `model_path` 파라미터로 경로를 직접 지정한다.

```bash
ros2 run step yolo26_detector --ros-args \
  -p model_path:=/absolute/path/to/best.onnx \
  -p device:=cpu
```

`device:=cpu`는 현재 개발 PC에서 가장 안정적인 실행 방식이다. Jetson으로 옮긴 뒤에는 `cuda` 또는 `tensorrt`로 바꿔볼 수 있다.

### 토픽 흐름

- `/camera/camera/color/image_raw` → YOLO26 detector 입력
- `/vision/detections` → line analyzer 입력
- `/vision/line_info` → debug monitor 입력
- `/vision/line_info` + 카메라 이미지 → path visualizer 입력

### 보조 노드

현재 기본 실행 흐름은 위 5개 터미널이다.
아래 파일들은 예전 실험용 또는 보조용 노드이므로 기본 실행 절차에서는 제외한다.

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
- `yolo26_detector`: RealSense RGB 영상에서 YOLO26 객체 탐지

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
- `device:=auto`는 TensorRT, CUDA, CPU 순으로 사용 가능한 ONNX Runtime 실행 장치를 선택합니다.
- 현재 노드는 RGB 객체 탐지 단계이며 RealSense Depth 거리 계산은 다음 단계에서 추가합니다.

## 포함 파일

- `src/step/step/look_ground.py`: 기본 테이프 검출
- `src/step/step/look_gground.py`: ROI 기반 테이프 검출
- `src/step/step/find_direct.py`: 평균 중심 기반 진행 방향 계산
- `src/step/step/find_ddirect.py`: 경로 상대 각도 기반 진행 방향 계산
- `src/step/step/yolo26_detector.py`: YOLO26 ONNX 객체 탐지와 ROS 토픽 발행
- `src/step/step/yolo_line_analyzer.py`: YOLO26 line 탐지 결과를 경로/방향 정보로 정리
- `src/step/step/line_debug_monitor.py`: `/vision/line_info`를 텍스트로 요약 표시
- `src/step/step/line_path_visualizer.py`: `/vision/line_info`와 카메라 이미지를 시각화
- `src/step/setup.py`: ROS 2 Python 노드 등록
- `src/step/package.xml`: ROS 2 패키지 정보와 의존성
