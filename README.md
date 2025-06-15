## TurtleBot3-based-Autonomous-Search-Robot-for-Missing-Target
# 실행 관련 지침 (Instruction)
## 목차
- [실행환경](#실행환경)
- [Phase 1: 기본 환경 구성](#phase-1-기본-환경-구성)
- [Phase 2: 우리 패키지 블록들 실행](#Phase-2-우리-패키지-블록들-실행)
- [Phase 3: 탐색시작 + 타이머](#Phase-3-탐색-시작--타이머)
- [Expected Result](#Expected-Result-Post-condition)
# 실행환경
UBUNTU 22.04<br>
ROS2 HUMBLE

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rviz2 \
  ros-humble-tf2-tools```
slam-toolbox: 지도 생성을 위한 SLAM 패키지

navigation2, nav2-bringup: 자율 주행을 위한 내비게이션 스택

rviz2: 3D 시각화 도구

tf2-tools: 좌표계 변환(TF) 디버깅 도구

2. 시뮬레이션 전용 패키지
Gazebo 가상 환경에서 테스트할 경우에만 추가로 설치하는 패키지입니다.

bash
sudo apt install -y ros-humble-gazebo-*
ros-humble-gazebo-*: Gazebo 시뮬레이터 및 관련 ROS 2 패키지 전체

3. 실제 로봇 전용 패키지
실제 TurtleBot3 하드웨어로 구동할 경우에만 추가로 설치하는 패키지입니다.

bash
sudo apt install -y \
  ros-humble-dynamixel-sdk \
  ros-humble-turtlebot3-msgs \
  ros-humble-hls-lfcd-lds-driver \
  libudev-dev
dynamixel-sdk: 다이나믹셀 모터 제어용 SDK

turtlebot3-msgs: TurtleBot3 전용 메시지 타입

hls-lfcd-lds-driver: 구형 LDS-01 라이다 센서 드라이버

libudev-dev: USB 장치 규칙(udev) 관리를 위한 라이브러리
