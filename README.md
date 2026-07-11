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
