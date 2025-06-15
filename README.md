## TurtleBot3-based-Autonomous-Search-Robot-for-Missing-Target
# 실행 관련 지침 (Instruction)
## 목차
1. [설치목록](#1-설치목록)  
   - [기본 세팅](#1-1-시스템-패키지-설치)  
   - [시뮬레이션 설치](#2-시뮬레이션-설치)  
   - [터틀봇3 설치 (설정)](#3-터틀봇3-설치-설정)  
   - [YOLO 설치](#4-yolo-설치)  
2. [실행법](#2-실행법)  
   - [시뮬레이션 실행](#1-시뮬레이션-실행)  
   - [터틀봇3 실행](#2-터틀봇3-실행)  
   - [YOLO 실행](#4-yolo-설치)

###### (태그를 누르면 해당하는 목차로 이동합니다.)
# 실행환경
UBUNTU 22.04<br>
ROS2 HUMBLE<br>
Simulation GPU : GTX1060 3GB
## 1. 설치목록

### [1] 기본 세팅

시뮬레이션과 실제 로봇 환경 **모두에** 반드시 설치해야 하는 공통 패키지 및 소스 코드.

#### **1-1. 시스템 패키지 설치**
```bash
sudo apt update
sudo apt install -y
  ros-humble-slam-toolbox
  ros-humble-navigation2
  ros-humble-nav2-bringup
  ros-humble-rviz2
  ros-humble-tf2-tools
```

#### **1-2. 소스 코드 다운로드**
(GitHub 저장소에 `turtlebot3`, `DynamixelSDK` 등이 이미 포함되어 있으므로, 해당 저장소만 클론.)
```bash
# ROS 2 작업 환경 생성 및 이동
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 프로젝트 저장소를 클론합니다.
git clone https://github.com/lmwde57/TurtleBot3-based-Autonomous-Search-Robot-for-Missing-Target.git

# 다운로드된 폴더 안의 실제 패키지들을 src 폴더로 이동시킵니다.
mv TurtleBot3-based-Autonomous-Search-Robot-for-Missing-Target/my_turtlebot_ws/src/* .
rm -rf TurtleBot3-based-Autonomous-Search-Robot-for-Missing-Target
```
---

### [2] 시뮬레이션 설치

**가상 환경(Gazebo)에서만** 프로젝트를 실행할 경우 추가로 필요한 설정입니다.

#### **2-1. 시뮬레이터 패키지 설치**
```bash
sudo apt install -y ros-humble-gazebo-*
```

#### **2-2. 시뮬레이션용 소스 코드 다운로드**
```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
---

### [3] 터틀봇3 설치 (설정)

**실제 TurtleBot3 하드웨어로** 구동할 경우 추가로 필요한 설정입니다.

#### **3-1. 하드웨어 드라이버 및 라이브러리 설치**
```bash
sudo apt install -y \
  ros-humble-dynamixel-sdk \
  libudev-dev
```
#### **3-2. 라이다(LiDAR) 센서 설정**
```bash
# 최신 LDS-02 모델을 사용하는 경우 드라이버 소스 코드 다운로드
cd ~/ros2_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/ld08_driver.git

# 터미널 환경에 사용할 라이다 모델을 지정 (LDS-02 사용 시)
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
```

#### **3-3. USB 포트 권한 설정**
```bash
sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
---

#### ※ 실행 전 최종 빌드 (필수)

위의 모든 설치가 끝난 후, 터미널을 열고 아래 명령어를 실행하여 전체 패키지를 빌드하고 환경을 설정합니다.

```bash
cd ~/ros2_ws/
colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### [4] YOLO 설치
#### 4-1.
```bash
pip install torch torchvision opencv-python numpy
```

#### 4-2. YOLOv5 레포 clone
```bash
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
```

---
## 2. 실행법
### [1] 시뮬레이션 실행
앞선 설치 관련 내용을 **반드시** 준수하여야 함.
#### 1-1. 가제보 시뮬레이션 실행
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
#### 1-2. SLAM 실행
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True slam_params_file:=/home/yoon/my_turtlebot_ws/param/my_slam_params.yaml
```
#### 1-3. Nav2 실행
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/home/yoon/my_turtlebot_ws/param/my_nav2_params.yaml
```
#### 1-4. 자동 탐색 패키지 실행
```bash
ros2 run explore_lite explore --ros-args --params-file /home/yoon/my_turtlebot_ws/param/explore_params.yaml
```
#### 1-5. RViz 실행
```bash
ros2 launch nav2_bringup rviz_launch.py
```
#### 1-6. 미션 매니저 노드 실행
```bash
ros2 run mission_control mission_manager
```
### [2] 터틀봇3 실행

#### 2-1. 로봇 시작
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

#### 2-2. 지도 SLAM
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/pi/my_turtlebot_ws/param/my_slam_params.yaml use_sim_time:=False
```
#### 2-3. 이동 NAV2
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=/home/pi/my_turtlebot_ws/param/my_nav2_params.yaml
```

#### 2-4. 제어
```bash
ros2 run mission_control mission_manager
```
#### 2-5. 탐색
```bash
ros2 run explore_lite explore --ros-args --params-file /home/pi/my_turtlebot_ws/param/explore_params.yaml -p use_sim_time:=False
```
### 3. YOLO 실행
#### 3-1. rasp.py 이동
**rasp.py** 파일을 라즈베리파이의 홈 폴더로 옮긴 후 진행 

#### 3-2. 카메라 설정법
라즈베리파이 (ssh 접속) <br>
ros2 launch rasp.py # 카메라 실행 <br>
컴퓨터 <br>
python3 yoloserv.py # 카메라 이미지에 대해 yolo 추론 실행, 이후 결과 bool형의 /doll_detected 반환 <br>
ros2 run rqt_image_view rqt_image_view /yolov5/image_annotated #이미지 추론 확인용, 꼭 실행할 필요는 없음 <br>
 
#### 3-3. 참고: 오류 발생시 설치할 것
```bash
pip install torch torchvision
pip install opencv-python
sudo apt install ros-humble-cv-bridge
pip install numpy
```
