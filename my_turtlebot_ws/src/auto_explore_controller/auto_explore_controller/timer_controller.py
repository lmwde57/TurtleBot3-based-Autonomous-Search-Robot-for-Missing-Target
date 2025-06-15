#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class TimerController(Node):
    def __init__(self):
        super().__init__('timer_controller')

        # 블록 4에게 타이머 완료 신호를 보내는 퍼블리셔
        self.timer_signal_publisher = self.create_publisher(
            Bool,
            '/timer/stop_signal',
            10
        )

        # 15초 타이머 설정
        self.timer_duration = 15.0  # 15초
        self.countdown_timer = self.create_timer(1.0, self.countdown_callback)
        self.remaining_time = self.timer_duration

        # 타이머 상태
        self.timer_completed = False

        self.get_logger().info('Timer Controller 시작 - 15초 후 블록 4에게 중단 신호 전송')
        self.get_logger().info(f'카운트다운 시작: {self.remaining_time:.0f}초')

    def countdown_callback(self):
        """1초마다 호출되는 카운트다운 콜백"""

        if self.remaining_time > 0:
            # 카운트다운 진행
            self.get_logger().info(f'탐색 중단까지 남은 시간: {self.remaining_time:.0f}초')
            self.remaining_time -= 1.0

        elif not self.timer_completed:
            # 15초 완료 - 블록 4에게 중단 신호 발송
            self.send_stop_signal()

    def send_stop_signal(self):
        """블록 4에게 탐색 중단 신호 발송"""
        if not self.timer_completed:
            msg = Bool()
            msg.data = False  # False = 중단 신호
            self.timer_signal_publisher.publish(msg)
            self.timer_completed = True

            self.get_logger().info('🚨 15초 타이머 완료!')
            self.get_logger().info('블록 4에게 중단 신호 전송 - 탐색 중단 및 복귀 요청')
            self.get_logger().info('블록 2 성공 - 타이머 기반 신호 전송 완료')

            # 타이머 정지
            self.countdown_timer.cancel()

    def manual_stop(self):
        """수동 중단 신호 발송 (테스트용)"""
        self.remaining_time = 0
        self.send_stop_signal()

    def get_remaining_time(self):
        """남은 시간 반환"""
        return self.remaining_time

    def is_timer_completed(self):
        """타이머가 완료되었는지 확인"""
        return self.timer_completed

def main():
    rclpy.init()
    timer_controller = TimerController()

    try:
        # 15초 타이머 동작
        rclpy.spin(timer_controller)
    except KeyboardInterrupt:
        timer_controller.get_logger().info('Timer Controller 종료')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
