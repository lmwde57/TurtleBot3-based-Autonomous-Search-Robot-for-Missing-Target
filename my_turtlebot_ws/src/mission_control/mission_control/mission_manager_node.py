# 파일 경로: ~/my_turtlebot_ws/src/mission_control/mission_control/mission_manager_node.py

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import random
import time
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class MissionManager(Node):
    """
    완전 자동화 미션 매니저 노드 (최종 안정화 버전)
    """

    def __init__(self):
        super().__init__('mission_manager_node')
        self.navigator = BasicNavigator()

        # 상태 변수: INITIALIZING, EXPLORING, PATROLLING, RETURNING_HOME, IDLE
        self.state = 'INITIALIZING'
        self.initial_pose_stamped = None
        self.costmap = None
        self.last_frontier_time = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.explore_publisher = self.create_publisher(Bool, '/explore/resume', 10)

        self.doll_detected_sub = self.create_subscription(
            Bool, '/doll_detected', self._doll_detected_callback, 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._costmap_callback,
            rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.frontier_sub = self.create_subscription(
            Path, '/explore/frontiers', self._frontier_callback, 10)

        self.mission_timer = self.create_timer(1.0, self._mission_loop)
        self.get_logger().info("완전 자동화 미션 매니저 시작. SLAM 준비 대기 중...")

    def _save_initial_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation
            self.initial_pose_stamped = pose
            self.get_logger().info(f"자동으로 초기 위치 저장 완료!")
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("SLAM TF 변환 대기 중... 아직 초기 위치를 저장할 수 없습니다.")
            return False

    def _start_exploration(self):
        self.get_logger().info("자동 탐색을 시작합니다...")
        msg = Bool(); msg.data = True
        self.explore_publisher.publish(msg)
        self.last_frontier_time = time.time()

    def _stop_exploration(self):
        self.get_logger().info("탐색을 중단합니다...")
        msg = Bool(); msg.data = False
        self.explore_publisher.publish(msg)

    def _frontier_callback(self, msg):
        if self.state == 'EXPLORING':
            self.last_frontier_time = time.time()

    def _costmap_callback(self, msg):
        self.costmap = msg

    def _doll_detected_callback(self, msg):
        ### --- 핵심 개선 사항 1 --- ###
        # 이 콜백은 오직 '중단' 신호만 보냅니다. 복귀 명령은 여기서 내리지 않습니다.
        if msg.data and self.state in ['EXPLORING', 'PATROLLING']:
            self.get_logger().info(f"인형 발견! 현재 '{self.state}' 상태를 중단합니다.")

            if self.state == 'EXPLORING':
                self._stop_exploration()

            # Nav2에 현재 진행중인 모든 작업을 취소하라고 명령
            self.navigator.cancelTask()
            # 상태를 '복귀 중'으로 변경. 실제 복귀 명령은 mission_loop에서 처리.
            self.state = 'RETURNING_HOME'

    def _mission_loop(self):
        ### --- 핵심 개선 사항 2 --- ###
        # 중앙 관제탑으로서, 현재 상태를 보고 순차적으로 다음 명령을 내립니다.
        if self.state == 'INITIALIZING':
            if self._save_initial_position():
                self._start_exploration()
                self.state = 'EXPLORING'

        elif self.state == 'EXPLORING':
            if self.last_frontier_time and (time.time() - self.last_frontier_time > 15.0):
                self.get_logger().info("탐색 완료. 무작위 순찰 모드로 전환합니다.")
                self._stop_exploration()
                self.state = 'PATROLLING'

        elif self.state == 'PATROLLING':
            if self.navigator.isTaskComplete():
                random_goal = self.get_random_goal()
                if random_goal:
                    self.navigator.goToPose(random_goal)

        elif self.state == 'RETURNING_HOME':
            # Nav2가 이전 작업(탐색 또는 순찰)을 완전히 취소하고 '할 일 없음' 상태가 되면,
            if self.navigator.isTaskComplete():
                # 그때 비로소 최종 복귀 명령을 내립니다.
                self.get_logger().info("이전 작업 종료 확인. 초기 위치로 최종 복귀를 시작합니다...")
                if self.initial_pose_stamped:
                    self.navigator.goToPose(self.initial_pose_stamped)
                # 복귀 명령을 내렸으니, 상태를 변경하여 다시 명령하지 않도록 함
                self.state = 'FINAL_RETURN_IN_PROGRESS'

        elif self.state == 'FINAL_RETURN_IN_PROGRESS':
            # 최종 복귀가 완료되었는지 확인
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("최종 복귀 완료. 모든 임무를 종료합니다.")
                else:
                    self.get_logger().error(f"최종 복귀 실패: {result}")

                self.state = 'IDLE'
                self.mission_timer.cancel() # 모든 임무가 끝났으므로 타이머 정지

    def get_random_goal(self):
        # (이전과 동일한 '진짜' 무작위 순찰 로직)
        if self.costmap is None: return None
        width = self.costmap.info.width; height = self.costmap.info.height
        data = np.array(self.costmap.data).reshape((height, width))
        free_space_indices = np.where(data == 0)

        if free_space_indices[0].size == 0: return None
        random_index = np.random.choice(len(free_space_indices[0]))
        y_idx, x_idx = free_space_indices[0][random_index], free_space_indices[1][random_index]

        resolution = self.costmap.info.resolution
        origin_x, origin_y = self.costmap.info.origin.position.x, self.costmap.info.origin.position.y
        goal_x = origin_x + (x_idx + 0.5) * resolution
        goal_y = origin_y + (y_idx + 0.5) * resolution

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x; goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation.w = 1.0
        return goal_pose

def main(args=None):
    rclpy.init(args=args)
    mission_manager = MissionManager()
    rclpy.spin(mission_manager)
    mission_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
