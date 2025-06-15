#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time

class ExplorationController(Node):
    def __init__(self):
        super().__init__('exploration_controller')

        # m-explore-ros2 탐색 제어 퍼블리셔 (탐색 시작/중단)
        self.explore_publisher = self.create_publisher(
            Bool,
            '/explore/resume',
            10
        )

        # [수정됨] YOLO 노드에서 오는 인형 발견 신호 구독자 (기존 타이머 신호 대신)
        self.doll_found_subscriber = self.create_subscription(
            Bool,
            'yolov5/object_detected',  # YOLO 노드가 발행할 토픽 이름
            self.doll_found_callback,
            10
        )

        # 블록 3에게 복귀 명령 전송 퍼블리셔
        self.return_command_publisher = self.create_publisher(
            Bool,
            '/navigation/return_home',
            10
        )

        # 블록 3에서 오는 복귀 상태 구독자
        self.return_status_subscriber = self.create_subscription(
            String,
            '/navigation/status',
            self.return_status_callback,
            10
        )

        # 상태 변수
        self.is_exploring = False
        self.return_requested = False

        self.get_logger().info('Exploration Controller 시작 - 탐색 제어 및 신호 수신 대기')

    def start_exploration(self):
        """m-explore-ros2 탐색 시작"""
        if not self.is_exploring:
            self.is_exploring = True

            msg = Bool()
            msg.data = True
            self.explore_publisher.publish(msg)

            self.get_logger().info('🔍 탐색 시작 - m-explore-ros2 활성화')
            self.get_logger().info('인형 발견 신호 대기 중...') # [수정됨] 로그 메시지 변경

    def doll_found_callback(self, msg):
        """[수정됨] YOLO 노드에서 오는 인형 발견 신호 수신 콜백"""
        if msg.data and self.is_exploring:  # True = 인형 발견 신호
            self.get_logger().info('🧸 인형 발견 신호 수신! 탐색을 중단하고 복귀합니다.')
            self.stop_exploration_and_return()

    def stop_exploration_and_return(self):
        """탐색 중단 후 복귀 시작"""
        if self.is_exploring:
            self.is_exploring = False
            stop_msg = Bool()
            stop_msg.data = False
            self.explore_publisher.publish(stop_msg)
            self.get_logger().info('🛑 탐색 중단 - m-explore-ros2 정지')

        if not self.return_requested:
            self.return_requested = True
            return_msg = Bool()
            return_msg.data = True
            self.return_command_publisher.publish(return_msg)
            self.get_logger().info('🏠 복귀 명령 전송 - 블록 3에게 복귀 요청')

    def return_status_callback(self, msg):
        """블록 3에서 오는 복귀 상태 수신"""
        status = msg.data
        if status == "SUCCEEDED":
            self.get_logger().info('✅ 복귀 완료! - 전체 시퀀스 성공')
            self.return_requested = False
        elif status == "ACTIVE":
            self.get_logger().info('🚶 복귀 진행 중...')
        elif status in ["FAILED", "CANCELED", "ABORTED"]:
            self.get_logger().error(f'❌ 복귀 실패: {status}')
            self.return_requested = False
    
    # --- 아래 main 함수는 테스트용이므로 그대로 둡니다 ---
def main():
    rclpy.init()
    exploration_controller = ExplorationController()
    
    # 런치 파일에서 실행할 때는 이 부분이 직접 호출되지 않습니다.
    # 하지만 단독 테스트를 위해 유지합니다.
    # exploration_controller.get_logger().info('3초 후 자동 탐색 시작...')
    # time.sleep(3)
    # exploration_controller.start_exploration()

    try:
        rclpy.spin(exploration_controller)
    except KeyboardInterrupt:
        exploration_controller.get_logger().info('Exploration Controller 종료')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

