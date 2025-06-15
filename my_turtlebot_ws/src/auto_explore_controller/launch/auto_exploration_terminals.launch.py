from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
import os

def generate_launch_description():

    # 작업 경로 설정
    workspace_path = os.path.expanduser('~/my_turtlebot_ws')

    # 공통 환경 설정
    base_cmd = f'cd {workspace_path} && source install/setup.bash && export TURTLEBOT3_MODEL=burger'

    # 1. SLAM (즉시 실행)
    slam_process = ExecuteProcess(
        cmd=['gnome-terminal', '--title=1-SLAM', '--', 'bash', '-c',
             f'{base_cmd} && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False; exec bash'],
        output='screen'
    )

    # 2. Navigation2 (10초 후)
    nav2_process = ExecuteProcess(
        cmd=['gnome-terminal', '--title=2-Nav2', '--', 'bash', '-c',
             f'{base_cmd} && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False; exec bash'],
        output='screen'
    )

    # 3. RViz (20초 후)
    rviz_process = ExecuteProcess(
        cmd=['gnome-terminal', '--title=3-RViz', '--', 'bash', '-c',
             f'{base_cmd} && ros2 launch nav2_bringup rviz_launch.py; exec bash'],
        output='screen'
    )

    # 4. 블록 1 (30초 후)
    block1_process = ExecuteProcess(
        cmd=['gnome-terminal', '--title=4-Block1-Position', '--', 'bash', '-c',
             f'{base_cmd} && ros2 run auto_explore_controller position_manager; exec bash'],
        output='screen'
    )

    # 5. 블록 3 (40초 후)
    block3_process = ExecuteProcess(
        cmd=['gnome-terminal', '--title=5-Block3-Navigation', '--', 'bash', '-c',
             f'{base_cmd} && ros2 run auto_explore_controller navigation_controller; exec bash'],
        output='screen'
    )

    # 6. 블록 4 (50초 후)
    block4_process = ExecuteProcess(
        cmd=['gnome-terminal', '--title=6-Block4-Exploration', '--', 'bash', '-c',
             f'{base_cmd} && ros2 run auto_explore_controller exploration_controller; exec bash'],
        output='screen'
    )

    # 7. m-explore-ros2 (60초 후)
    explore_process = ExecuteProcess(
        cmd=['gnome-terminal', '--title=7-Explore', '--', 'bash', '-c',
             f'{base_cmd} && ros2 launch explore_lite explore.launch.py; exec bash'],
        output='screen'
    )

    # 8. 블록 2 타이머 (70초 후)
    timer_process = ExecuteProcess(
        cmd=['gnome-terminal', '--title=8-Block2-Timer', '--', 'bash', '-c',
             f'{base_cmd} && ros2 run auto_explore_controller timer_controller; exec bash'],
        output='screen'
    )

    return LaunchDescription([
        # 순차적 실행 (10초 간격)
        LogInfo(msg='🚀 자동 탐색 시스템 시작 - SLAM 실행'),
        TimerAction(
            period=0.0,
            actions=[LogInfo(msg='📍 SLAM 시작...'), slam_process]
        ),

        TimerAction(
            period=15.0,
            actions=[LogInfo(msg='📍 Navigation2 시작...'), nav2_process]
        ),

        TimerAction(
            period=25.0,
            actions=[LogInfo(msg='📍 RViz 시작...'), rviz_process]
        ),

        TimerAction(
            period=35.0,
            actions=[LogInfo(msg='📍 블록 1 (위치 저장) 시작...'), block1_process]
        ),

        TimerAction(
            period=50.0,
            actions=[LogInfo(msg='📍 블록 3 (복귀 대기) 시작...'), block3_process]
        ),

        TimerAction(
            period=65.0,
            actions=[LogInfo(msg='📍 블록 4 (탐색 제어) 시작...'), block4_process]
        ),

        TimerAction(
            period=80.0,
            actions=[LogInfo(msg='📍 탐색 패키지 시작...'), explore_process]
        ),

        TimerAction(
            period=70.0,
            actions=[LogInfo(msg='📍 타이머 시작 - 15초 후 복귀!'), timer_process]
        ),
    ])
