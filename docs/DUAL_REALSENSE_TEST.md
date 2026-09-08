# Dual RealSense Stage 1 테스트

## 목적과 범위

이 문서는 production single-camera pipeline을 변경하지 않고 다음 구조의
가능성과 Jetson 부하를 확인하기 위한 수동 hardware integration test 절차다.

```text
Front RealSense RGB --+
                      +--> RGB selector --> YOLO26 x 1
Down RealSense RGB ---+
```

두 센서는 계속 stream하고 selector가 활성 RGB만 전달한다. Stage 1은 RGB와
YOLO detection만 다룬다. Depth, CameraInfo 기반 거리/3D 계산,
`unified_vision_node`, localization, mission control, motion executor와 실제 로봇
구동은 실행하지 않는다.

기존 detector를 test topic override로 재사용하므로 class 계약도 production과
같은 6개다.

```text
0 line
1 ball
2 goal
3 backboard
4 hurdle
5 grab
```

## production과 다른 점

- `full_system.launch.py`와 `full_system_robot.launch.py`는 실행하거나 수정하지
  않는다.
- 모든 실험 node와 topic은 기본적으로 `/vision_test` 아래에 있다.
- 두 RealSense 모두 color만 켜고 depth, IR, IMU, RGBD, pointcloud, depth align,
  TF publish를 끈다.
- YOLO display와 annotated image publish는 기본적으로 꺼 불필요한 그리기와
  image copy를 줄인다. 시각 확인이 필요할 때만 켠다.
- selector는 `cv_bridge` 변환 없이 선택된 ROS `Image` 메시지를 전달한다.
- YOLO node는 launch 파일에 하나만 존재한다.

예정한 물리 역할은 다음과 같다.

- Front: 정면/중장거리 line, corner, 먼 hurdle/ball, goal, backboard,
  landmark/localization
- Down: 로봇 앞 근거리 line, 가까운 hurdle/ball, pickup/grab 확인, 발 앞 위치

## 설치된 RealSense launch와 topic 규칙

이 branch를 만든 환경의 ROS 2 Humble
`/opt/ros/humble/share/realsense2_camera/launch/rs_launch.py`에서 실제로 확인한
인자는 `camera_namespace`, `camera_name`, `serial_no`,
`rgb_camera.color_profile`, `enable_depth`, `enable_gyro`,
`align_depth.enable`, `pointcloud.enable`, `publish_tf` 등이다.

현재 설치본의 `global_settings.yaml`은 lifecycle node를 사용하지 않는다. 전용
launch는 중첩 include 때 발생하는 잘못된 상위 인자 경고를 피하기 위해, 확인한
동일 parameter로 `realsense2_camera_node` 두 개를 직접 실행한다.

RealSense node의 namespace와 name을 함께 적용한 기본 topic은 다음과 같다.

```text
/vision_test/front_camera/color/image_raw
/vision_test/down_camera/color/image_raw
```

현재 설치된 드라이버가 다르면 실행 전에 그 장치에서 다시 확인한다.

```bash
ros2 launch realsense2_camera rs_launch.py --show-args
```

## 준비와 빌드

두 카메라의 serial을 기록한다. launch가 숫자로만 된 serial도 문자열로
RealSense driver에 전달하므로 명령에 별도 중첩 따옴표는 필요하지 않다.

```bash
rs-enumerate-devices -s
rs-enumerate-devices | rg -i 'Name|Serial Number|USB Type Descriptor'
```

Front와 Down 장착 위치에 해당하는 serial을 물리적으로 확인한다. 두 serial이
같거나, 활성화한 카메라의 serial이 비어 있으면 launch는 즉시 실패한다.

```bash
cd ~/IRC_vision
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select step
source install/setup.bash
```

실제 저장소 경로가 다르면 `cd` 경로만 바꾼다.

## 실행

Jetson TensorRT 기본 실행:

```bash
ros2 launch step dual_realsense_test.launch.py \
  front_serial_no:=FRONT_SERIAL \
  down_serial_no:=DOWN_SERIAL
```

Down을 초기 source로 선택하려면 다음 인자를 추가한다.

```bash
initial_active_camera:=down
```

기본 profile은 production color 입력과 같은 `1280,720,30`, detector cap은
`30.0` FPS다. 필요할 때만 다음처럼 바꾼다.

```bash
color_profile:=640,480,30 max_fps:=15.0
```

PC에서 ONNX CPU로 graph만 확인하는 예:

```bash
ros2 launch step dual_realsense_test.launch.py \
  front_serial_no:=FRONT_SERIAL \
  down_serial_no:=DOWN_SERIAL \
  model_path:="$(ros2 pkg prefix --share step)/models/best.onnx" \
  device:=cpu max_fps:=15.0
```

시각 확인을 짧게 수행할 때만 다음 옵션을 사용한다. 성능 측정 시에는 둘 다
`false`로 되돌린다.

```bash
display:=true publish_annotated_image:=true
```

## Topic과 node 확인

```bash
ros2 topic list | rg '^/vision_test/'
ros2 node list | rg '^/vision_test/'
```

핵심 topic은 다음과 같다.

```text
/vision_test/front_camera/color/image_raw
/vision_test/down_camera/color/image_raw
/vision_test/active/color/image_raw
/vision_test/camera_select
/vision_test/active_camera
/vision_test/detections
/vision_test/detections/image       # 옵션을 켠 경우
```

YOLO가 하나뿐인지 확인한다. 출력이 한 줄이어야 한다.

```bash
ros2 node list | rg '/vision_test/yolo26_detector$'
```

`/vision_test/line_info`, `ball_info`, `goal_info`, `hurdle_info`는 detector의
test 전용 입력 이름일 뿐 Stage 1 launch는 analyzer를 실행하지 않는다.

## Front/Down 전환

현재 source 확인:

```bash
ros2 topic echo /vision_test/active_camera
```

Down으로 전환:

```bash
ros2 topic pub --once /vision_test/camera_select \
  std_msgs/msg/String "{data: down}"
```

Front로 전환:

```bash
ros2 topic pub --once /vision_test/camera_select \
  std_msgs/msg/String "{data: front}"
```

카메라 node를 stop/start하지 않는다. 잘못된 문자열은 selector가 거부하고 기존
source를 유지한다. 여러 번 왕복하면서 selector log의 전환 시점과 forwarded
frame count, active RGB와 detection의 끊김을 확인한다.

## FPS와 switching 측정

각 명령은 가능하면 별도 terminal에서 60초 이상 측정한다.

```bash
ros2 topic hz /vision_test/front_camera/color/image_raw
ros2 topic hz /vision_test/down_camera/color/image_raw
ros2 topic hz /vision_test/active/color/image_raw
ros2 topic hz /vision_test/detections
```

메시지 timestamp 지연도 함께 확인할 수 있다.

```bash
ros2 topic delay /vision_test/active/color/image_raw
```

다음 순서로 최소 20회 전환한다.

```text
front -> down -> front -> down ...
```

각 전환에서 확인할 항목:

- active RGB가 다음 선택 source의 frame으로 바뀌는 시간
- `ros2 topic hz`가 보고하는 최소/최대 간격과 일시적인 frame gap
- detection 재개 시간과 detection FPS
- crash, 오래된 source frame, camera reconnect 여부

## 공정한 single/dual 부하 비교

같은 branch, model, resolution, detector FPS cap, 전원 모드에서 순서대로
측정한다. 각 case는 1분 warm-up 뒤 최소 5분 기록한다.

1대 기준선은 같은 launch에서 Down만 끈다.

```bash
ros2 launch step dual_realsense_test.launch.py \
  front_serial_no:=FRONT_SERIAL \
  enable_down_camera:=false
```

그 다음 두 카메라를 켠 기본 명령으로 같은 측정을 반복한다. 비교할 값은 다음과
같다.

```text
front RGB average/min FPS
down RGB average/min FPS
active RGB average/min FPS
detection average/min FPS
CPU utilization
GPU utilization/frequency
RAM and swap usage
power draw and temperature
USB errors/reconnects
switching maximum frame gap
```

PC에서는 다음 도구를 사용할 수 있다.

```bash
htop
watch -n 1 "ps -eo pid,pcpu,pmem,rss,cmd | rg \
  'realsense2_camera_node|yolo26_detector|dual_realsense'"
nvidia-smi dmon -d 1
```

Jetson에서는 동일한 전원 모드와 clock 조건을 유지하고 다음을 기록한다.

```bash
sudo tegrastats --interval 1000
```

테스트 결과 표:

| 항목 | 1대 기준선 | 2대/front | 2대/down | 판정 |
|---|---:|---:|---:|---|
| RGB FPS |  |  |  |  |
| active RGB FPS |  |  |  |  |
| detection FPS |  |  |  |  |
| CPU |  |  |  |  |
| GPU |  |  |  |  |
| RAM |  |  |  |  |
| power/temperature |  |  |  |  |
| 최대 switch gap | N/A |  |  |  |
| USB error/reconnect |  |  |  |  |

## USB bandwidth 확인

실행 전후에 topology를 확인한다.

```bash
lsusb -t
```

두 RealSense가 `5000M` 이상의 USB 3.x 링크인지, 같은 저속 hub에 묶이지
않았는지 확인한다. 가능하면 서로 다른 root port/controller에 연결한다. 장시간
실행 중 kernel log도 확인한다.

```bash
sudo dmesg --follow | rg -i 'usb|uvc|realsense|disconnect|reset|bandwidth'
```

다음 현상은 실패로 기록한다.

- `Device or resource busy`, bandwidth 부족 또는 UVC 오류
- USB reset/disconnect/reconnect
- 한 카메라만 반복적으로 FPS가 떨어짐
- color topic 중단 또는 RealSense node 종료

## Stage 1 성공 기준

- 두 serial이 항상 같은 Front/Down 역할로 연결된다.
- 두 RGB topic 모두 목표 30 FPS 근처를 장시간 유지한다.
- selector와 detection이 20회 이상 왕복 전환 후에도 crash하지 않는다.
- Front/Down 영상이 선택과 일치하고 YOLO process는 정확히 하나다.
- detection FPS와 Jetson CPU/GPU/RAM/power가 실제 운용 허용 범위다.
- USB error/reconnect가 없다.
- production 파일과 production launch에는 변경이 없다.

절대적인 “허용 부하”는 Jetson 전원 모드, thermal 상태와 나머지 production
node 부하에 따라 달라진다. 따라서 Stage 1 통과는 dual RGB + single YOLO의
필요조건이며, `full_system_robot` 전체 성능 통과를 대신하지 않는다.

## 알려진 제한과 Stage 2

Stage 1은 RGB source만 바꾸므로 depth나 3D geometry를 active RGB와 함께
사용하면 안 된다. 선택 후 `Image.header.frame_id`는 해당 물리 카메라 frame을
그대로 유지한다.

Stage 2에서는 별도 test graph에서 다음을 함께 구현하고 검증해야 한다.

- 같은 source의 RGB, aligned depth, color CameraInfo를 원자적으로 선택
- 전환 직후 이전 카메라의 depth/cache를 폐기하고 timestamp 동기화 확인
- Front/Down 각각의 intrinsic과 설치 extrinsic/calibration 적용
- `robot_center_offset_px`, `camera_height_m`, `camera_pitch_down_deg`,
  `camera_forward_offset_m` 등 카메라별 값 전환
- test topic override로 `unified_vision_node`를 재사용할 수 있는지 검증
- motion hardware를 끈 상태에서 나머지 full-system node까지 포함한 Jetson
  부하 비교

Front와 Down의 설치 각도와 위치가 다르므로 active camera가 바뀌었는데 하나의
extrinsic을 계속 사용하는 구조는 허용하지 않는다. Stage 2 결과가 확인되기
전에는 이 branch를 production branch에 merge하지 않는다.

## 2026-09-09 개발 PC smoke test 기록

Jetson이 아닌 x86_64 Ubuntu 개발 PC에서 짧은 기능 검증을 수행했다. 이 결과는
Jetson 연산량 또는 장시간 USB 안정성 판정이 아니다.

```text
RealSense ROS: 4.58.3
librealsense: 2.58.3
Device 1: D435I 948522070995, firmware 5.17.3.10
Device 2: D435I 148522071908, firmware 5.17.0.10
USB: 두 장치 모두 3.2, 같은 5000M hub 아래
Profile: RGB8 1280x720@30, depth/IR/IMU/TF off
YOLO: best.onnx, CPUExecutionProvider, max_fps=5.0
```

두 serial의 Front/Down 지정은 기능 확인을 위한 임시 매핑이었으며 실제 장착
방향은 로봇에서 다시 확인해야 한다. 측정 결과는 다음과 같다.

```text
Front RGB: 약 29.7 FPS
Down RGB: 약 30.0 FPS
Active RGB: 약 29.1 FPS
Detection: 약 4.5 FPS (CPU 5 FPS cap)
자동 전환: 20회, crash 없음, 최종 상태 front
Frame ID: front_camera_color_optical_frame / down_camera_color_optical_frame
종료: camera 2개, selector, detector 모두 clean exit
```

두 카메라의 firmware가 다르고 USB root 경로를 공유하므로 Jetson 장시간 시험
기록에서 이를 변수로 남긴다. 호환성을 확인하지 않은 firmware 변경은 하지
않는다. Jetson에서는 TensorRT `best.engine`, production과 같은 전원/clock 및
30 FPS cap으로 1대 기준선과 2대 결과를 별도로 측정해야 한다.
