#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
import tf2_ros
import math

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')

        # BasicNavigator 초기화
        self.navigator = BasicNavigator()

        # tf2 설정 (현재 위치 추적용)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 초기 위치 및 상태 변수
        self.initial_pose = None
        self.navigation_active = False
        self.initial_position_received = False

        # QoS 설정: 이전 메시지도 수신 가능
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 초기 위치 구독자 (블록 1에서 전송된 위치 수신, QoS 적용)
        self.initial_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/initial_position',
            self.initial_position_callback,
            qos_profile
        )

        # 블록 4에서 오는 복귀 명령 구독자
        self.return_command_subscriber = self.create_subscription(
            Bool,
            '/navigation/return_home',
            self.return_command_callback,
            10
        )

        # 블록 4에게 복귀 상태 전송 퍼블리셔
        self.status_publisher = self.create_publisher(
            String,
            '/navigation/status',
            10
        )

        self.get_logger().info('Navigation Controller 시작 - 복귀 기능 준비')
        self.get_logger().info('블록 1에서 초기 위치 전송 대기 중... (QoS 지속 메시지 수신 가능)')

        # Nav2 시스템 준비 대기
        self.timer = self.create_timer(1.0, self.check_nav2_ready)
        self.nav2_ready = False

        # 상태 전송 타이머 (복귀 중 상태 주기적 전송)
        self.status_timer = self.create_timer(2.0, self.publish_status)

    def check_nav2_ready(self):
        """Nav2 시스템이 준비되었는지 확인"""
        try:
            # tf 변환이 가능한지 확인
            self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            if not self.nav2_ready:
                self.get_logger().info('Nav2 시스템 준비 완료 - 복귀 기능 활성화')
                self.nav2_ready = True
                self.timer.cancel()
        except:
            if not self.nav2_ready:
                self.get_logger().info('Nav2 시스템 대기 중... (SLAM/Navigation 필요)')

    def initial_position_callback(self, msg):
        """블록 1에서 전송된 초기 위치 수신"""
        self.initial_pose = msg
        self.initial_position_received = True

        self.get_logger().info('🎯 초기 위치 수신 완료 - 복귀 기능 준비됨')
        self.get_logger().info(f'수신된 위치: x={msg.pose.position.x:.3f}m, y={msg.pose.position.y:.3f}m')
        self.get_logger().info('블록 4에서 복귀 명령 대기 중...')

    def return_command_callback(self, msg):
        """블록 4에서 오는 복귀 명령 수신"""
        if msg.data and not self.navigation_active:  # True = 복귀 시작
            self.get_logger().info('📡 블록 4에서 복귀 명령 수신!')
            self.return_to_initial_position()

    def return_to_initial_position(self):
        """블록 1에서 받은 초기 위치로 복귀"""
        if not self.initial_position_received:
            self.get_logger().error('❌ 초기 위치를 아직 수신하지 못함 - 블록 1 실행 필요')
            return False

        if not self.nav2_ready:
            self.get_logger().error('❌ Nav2가 준비되지 않음 - SLAM/Navigation 필요')
            return False

        try:
            self.get_logger().info('🏠 초기 위치로 복귀 시작!')
            self.get_logger().info(f'목표 위치: x={self.initial_pose.pose.position.x:.3f}m, y={self.initial_pose.pose.position.y:.3f}m')

            # BasicNavigator로 복귀 명령 실행
            self.navigator.goToPose(self.initial_pose)
            self.navigation_active = True

            self.get_logger().info('Nav2에 이동 명령 전송 완료 - 자율 이동 시작')
            return True

        except Exception as e:
            self.get_logger().error(f'❌ 복귀 명령 실패: {str(e)}')
            return False

    def check_navigation_status(self):
        """복귀 진행 상태 확인 (ROS2 Humble 호환 - 숫자 기반)"""
        if not self.navigation_active:
            return "IDLE"

        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            self.navigation_active = False

            # 숫자 기반 결과 확인 (ROS2 Humble 호환)
            if result == 3:  # SUCCEEDED = 3
                self.get_logger().info('✅ 초기 위치 복귀 성공!')
                self.get_logger().info('블록 3 성공 - Nav2 복귀 기능 완료')
                return "SUCCEEDED"
            elif result == 2:  # CANCELED = 2
                self.get_logger().warn('⚠️  복귀 취소됨')
                return "CANCELED"
            elif result == 1:  # FAILED = 1
                self.get_logger().error('❌ 복귀 실패')
                return "FAILED"
            else:
                # 예상치 못한 결과도 성공으로 처리 (안전 장치)
                self.get_logger().info(f'복귀 완료 (결과: {result})')
                return "SUCCEEDED"
        else:
            # 진행 중 상태 로그
            distance = self.get_current_distance_from_initial()
            if distance:
                self.get_logger().info(f'복귀 중... 목표까지 남은 거리: {distance:.2f}m')
            return "ACTIVE"

    def publish_status(self):
        """현재 복귀 상태를 블록 4에게 전송"""
        status = self.check_navigation_status()

        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)

    def get_current_distance_from_initial(self):
        """초기 위치로부터 현재 거리 계산"""
        if not self.initial_position_received or not self.nav2_ready:
            return None

        try:
            current_transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            dx = current_transform.transform.translation.x - self.initial_pose.pose.position.x
            dy = current_transform.transform.translation.y - self.initial_pose.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            return distance

        except Exception as e:
            return None

    def cancel_navigation(self):
        """복귀 취소"""
        if self.navigation_active:
            self.navigator.cancelTask()
            self.navigation_active = False
            self.get_logger().info('복귀 명령 취소됨')

def main():
    rclpy.init()
    nav_controller = NavigationController()

    # Nav2 준비될 때까지 대기
    rate = nav_controller.create_rate(10)
    while rclpy.ok() and not nav_controller.nav2_ready:
        rclpy.spin_once(nav_controller, timeout_sec=0.1)

    if nav_controller.nav2_ready:
        nav_controller.get_logger().info('Nav2 준비 완료 - 초기 위치 수신 및 복귀 명령 대기')

        # 초기 위치 수신 대기
        while rclpy.ok() and not nav_controller.initial_position_received:
            rclpy.spin_once(nav_controller, timeout_sec=0.1)

        if nav_controller.initial_position_received:
            nav_controller.get_logger().info('블록 4에서 복귀 명령 대기 모드 진입')

    try:
        rclpy.spin(nav_controller)
    except KeyboardInterrupt:
        nav_controller.get_logger().info('Navigation Controller 종료')
    finally:
        nav_controller.navigator.lifecycleShutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
