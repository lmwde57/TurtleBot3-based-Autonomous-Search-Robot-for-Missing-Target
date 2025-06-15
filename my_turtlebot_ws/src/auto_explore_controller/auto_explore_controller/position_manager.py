#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

class PositionManager(Node):
    def __init__(self):
        super().__init__('position_manager')
        self.navigator = BasicNavigator()
        self.initial_pose = None

        # tf2 설정 (SLAM 좌표 추적용)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Position Manager 시작 - SLAM 지도 좌표 기반 (Gazebo 좌표 무시)')

        # SLAM 준비 확인
        self.timer = self.create_timer(1.0, self.check_slam_ready)
        self.slam_ready = False

        # QoS 설정: 늦게 구독하는 노드도 메시지 수신 가능
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 초기 위치 퍼블리셔 (QoS 적용)
        self.initial_pose_publisher = self.create_publisher(
            PoseStamped,
            '/initial_position',
            qos_profile
        )

    def check_slam_ready(self):
        """SLAM이 준비되었는지 확인"""
        try:
            # SLAM이 map -> base_link transform을 제공하는지 확인
            self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            if not self.slam_ready:
                self.get_logger().info('SLAM 시스템 준비 완료 - 지도 좌표계 활성화')
                self.slam_ready = True
                self.timer.cancel()
        except:
            if not self.slam_ready:
                self.get_logger().info('SLAM 시스템 대기 중... (지도 좌표계 생성 중)')

    def save_initial_position(self):
        """SLAM 지도 좌표 기반 초기 위치 저장 (Gazebo 좌표 무시)"""
        if not self.slam_ready:
            self.get_logger().warn('SLAM이 아직 준비되지 않음')
            return False

        try:
            # SLAM이 생성한 지도에서 현재 위치 가져오기
            current_transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # SLAM 지도 좌표계에서의 실제 위치 저장
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            # tf에서 가져온 실제 SLAM 좌표 사용
            pose.pose.position.x = current_transform.transform.translation.x
            pose.pose.position.y = current_transform.transform.translation.y
            pose.pose.position.z = current_transform.transform.translation.z
            pose.pose.orientation = current_transform.transform.rotation

            self.initial_pose = pose

            # 초기 위치 퍼블리시 (QoS로 늦게 구독해도 수신 가능)
            self.initial_pose_publisher.publish(pose)
            self.get_logger().info('초기 위치를 다른 노드들에게 전송 완료 (QoS 지속)')

            self.get_logger().info('SLAM 지도 좌표로 초기 위치 저장 완료 (Gazebo 좌표 무시)')
            self.get_logger().info(f'저장된 SLAM 좌표: x={pose.pose.position.x:.3f}m, y={pose.pose.position.y:.3f}m')

            return True

        except Exception as e:
            self.get_logger().error(f'SLAM 좌표 저장 실패: {str(e)}')
            return False

    def get_initial_position(self):
        """저장된 초기 위치 반환"""
        return self.initial_pose

    def get_current_distance_from_initial(self):
        """초기 위치로부터 현재 거리 계산 (SLAM 좌표 기반)"""
        if not self.initial_pose or not self.slam_ready:
            return None

        try:
            # 현재 SLAM 좌표 가져오기
            current_transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 저장된 초기 위치와 현재 위치 간 거리 계산
            dx = current_transform.transform.translation.x - self.initial_pose.pose.position.x
            dy = current_transform.transform.translation.y - self.initial_pose.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)

            return distance

        except Exception as e:
            self.get_logger().error(f'거리 계산 실패: {str(e)}')
            return None

    def is_position_saved(self):
        """초기 위치가 저장되었는지 확인"""
        return self.initial_pose is not None

def main():
    rclpy.init()
    position_manager = PositionManager()

    # SLAM 준비될 때까지 대기
    rate = position_manager.create_rate(10)
    while rclpy.ok() and not position_manager.slam_ready:
        rclpy.spin_once(position_manager, timeout_sec=0.1)

    if position_manager.slam_ready:
        # SLAM 지도 좌표로 초기 위치 저장
        if position_manager.save_initial_position():
            position_manager.get_logger().info('블록 1 성공 - SLAM 기반 위치 관리자 준비 완료')
            position_manager.get_logger().info('다음 단계: 탐색 시작 및 15초 후 SLAM 좌표로 정확한 복귀')
        else:
            position_manager.get_logger().error('블록 1 실패')

    try:
        rclpy.spin(position_manager)
    except KeyboardInterrupt:
        position_manager.get_logger().info('Position Manager 종료')
    finally:
        position_manager.navigator.lifecycleShutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
