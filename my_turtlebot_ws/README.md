## TurtleBot3-based-Autonomous-Search-Robot-for-Missing-Target
# 전체 테스트 순서 (8개 터미널)
## 목차
- [실행환경](#실행환경)
- [Phase 1: 기본 환경 구성](#phase-1-기본-환경-구성)
- [Phase 2: 우리 패키지 블록들 실행](#Phase-2-우리-패키지-블록들-실행)
- [Phase 3: 탐색시작 + 타이머](#Phase-3-탐색-시작--타이머)
- [Expected Result](#Expected-Result-Post-condition)
# 실행환경
UBUNTU 22.04<br>
ROS2 HUMBLE

# Phase 1: 기본 환경 구성
<터미널 1: Gazebo 시뮬레이션 실행><br>
```bash
cd ~/my_turtlebot_ws/ && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

<터미널 2: SLAM Toolbox 실행>
```bash
cd ~/my_turtlebot_ws/ && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

<터미널 3: Navigation2 실행>
```bash
cd ~/my_turtlebot_ws/ && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

# Phase 2: 우리 패키지 블록들 실행
<터미널 4: 블록 1 (초기 위치 저장) 실행><br>
```bash
cd ~/my_turtlebot_ws/ && source install/setup.bash && ros2 run auto_explore_controller position_manager
```
<터미널 5: 블록 3 (복귀 대기) 실행><br>
```bash
cd ~/my_turtlebot_ws/ && source install/setup.bash && ros2 run auto_explore_controller navigation_controller
```
<터미널 6: 블록 4 (탐색 제어 허브) 실행><br>
```bash
cd ~/my_turtlebot_ws/ && source install/setup.bash && ros2 run auto_explore_controller exploration_controller
```
# Phase 3: 탐색 시작 + 타이머
<터미널 7: m-explore-ros2 탐색 패키지 실행>
```bash
cd ~/my_turtlebot_ws/ && source install/setup.bash && ros2 launch explore_lite explore.launch.py
```
<터미널 8: 블록 2 (15초 타이머) 실행 - 빠르게!>
```bash
cd ~/my_turtlebot_ws/ && source install/setup.bash && ros2 run auto_explore_controller timer_controller
```
실제 환경에서는 목표물을 찾고 즉시 복귀하지만, 해당 실행 환경에선 목표물 설정 대신 임의로 15초 후 찾았다는 가정 하에 진행.

# Expected Result (Post condition)
3초 후: 블록 4에서 "탐색 시작" 메시지

즉시: explore_lite가 실제 탐색 시작

15초 카운트다운: 블록 2에서 타이머 진행

15초 후: "타이머 완료" → "탐색 중단" → "복귀 시작"

복귀 완료: "✅ 전체 시퀀스 성공!"
