#!/usr/bin/env python3
# localization_monitor_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from std_msgs.msg import Bool, String


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')

        # ===== 임계값 =====
        self.cov_xy_threshold = 0.5
        self.cov_yaw_threshold = 0.5
        self.lost_grace_sec = 3.0
        self.amcl_stale_sec = 2.0       # AMCL 끊김 판정 시간
        self.startup_grace_sec = 5.0    # 부팅 직후 NO_AMCL 경고 보류 시간

        # ===== 상태 머신 =====
        # 'init' → 'no_amcl' / 'ok' / 'stale' / 'uncertain' / 'lost'
        self.state = 'init'

        # 측정 데이터
        self.last_cov_x = None
        self.last_cov_y = None
        self.last_cov_yaw = None
        self.last_amcl_time = None
        self.first_uncertain_time = None

        self.start_time = self.get_clock().now()

        # ===== 토픽/서비스 =====
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)

        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop/localization', 10)
        self.sos_pub = self.create_publisher(String, '/sos_trigger', 10)
        self.status_pub = self.create_publisher(String, '/localization_status', 10)

        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')

        # 상태 토픽 주기 발행 (2Hz)
        self.create_timer(0.5, self.publish_state)
        # 상태 머신 평가 (1Hz, 상태 전이 시에만 이벤트 처리)
        self.create_timer(1.0, self.evaluate_state)

        self.get_logger().info(
            f'localization_monitor 시작 (공분산 임계값: xy={self.cov_xy_threshold}, '
            f'yaw={self.cov_yaw_threshold}, 유예시간: {self.lost_grace_sec}초)'
        )

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        """AMCL 메시지 수신 시 공분산 데이터 갱신"""
        covariance = msg.pose.covariance
        self.last_cov_x = covariance[0]
        self.last_cov_y = covariance[7]
        self.last_cov_yaw = covariance[35]
        self.last_amcl_time = self.get_clock().now()

    def evaluate_state(self):
        """1Hz 주기로 위치 추정 상태를 평가하여 전이 발생 시 이벤트 처리"""
        current_time = self.get_clock().now()
        new_state = self._compute_state(current_time)

        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state
        self._log_transition(old_state, new_state, current_time)
        self._handle_transition(new_state)

    def _compute_state(self, current_time):
        # 1) AMCL 초기 수신 대기
        if self.last_amcl_time is None:
            elapsed_boot = (current_time - self.start_time).nanoseconds / 1e9
            return 'init' if elapsed_boot < self.startup_grace_sec else 'no_amcl'

        # 2) AMCL 수신 타임아웃
        elapsed_since_amcl = (current_time - self.last_amcl_time).nanoseconds / 1e9
        if elapsed_since_amcl > self.amcl_stale_sec:
            return 'stale'

        # 3) 공분산 유효성 검사
        if (self.last_cov_x is None or
                self.last_cov_y is None or
                self.last_cov_yaw is None):
            return 'init'

        is_covariance_diverged = (self.last_cov_x > self.cov_xy_threshold or
                                  self.last_cov_y > self.cov_xy_threshold or
                                  self.last_cov_yaw > self.cov_yaw_threshold)

        if not is_covariance_diverged:
            self.first_uncertain_time = None
            return 'ok'

        if self.first_uncertain_time is None:
            self.first_uncertain_time = current_time

        elapsed = (current_time - self.first_uncertain_time).nanoseconds / 1e9
        return 'lost' if elapsed > self.lost_grace_sec else 'uncertain'

    def _log_transition(self, old_state, new_state, current_time):
        cov_str = ''
        if self.last_cov_x is not None:
            cov_str = (f' | cov_x={self.last_cov_x:.3f}, '
                       f'cov_y={self.last_cov_y:.3f}, '
                       f'cov_yaw={self.last_cov_yaw:.3f}')

        if new_state == 'ok':
            if old_state in ('init', 'no_amcl'):
                self.get_logger().info(f'위치 추적 시작{cov_str}')
            elif old_state == 'stale':
                self.get_logger().info(f'AMCL 수신 복귀{cov_str}')
            elif old_state == 'lost':
                self.get_logger().info(f'위치 재추정 성공{cov_str}')
            elif old_state == 'uncertain':
                self.get_logger().info(f'위치 정상 복귀{cov_str}')

        elif new_state == 'no_amcl':
            self.get_logger().warn(
                'AMCL 포즈 미수신 (/amcl_pose 대기 중)')

        elif new_state == 'stale':
            elapsed_sec = (current_time - self.last_amcl_time).nanoseconds / 1e9
            self.get_logger().warn(f'AMCL 수신 지연 ({elapsed_sec:.1f}초 경과)')

        elif new_state == 'uncertain':
            self.get_logger().warn(f'위치 공분산 초과 (불확실){cov_str}')

        elif new_state == 'lost':
            self.get_logger().error(
                f'위치 추적 분실{cov_str} - 재위치추정 요청')

    def _handle_transition(self, new_state):
        if new_state == 'lost':
            self.sos_pub.publish(String(data='localization_lost'))
            self._trigger_global_relocalization()

    def _trigger_global_relocalization(self):
        if self.global_loc_client.wait_for_service(timeout_sec=1.0):
            self.global_loc_client.call_async(Empty.Request())
            self.get_logger().info('AMCL 글로벌 재위치추정 요청 전송')
        else:
            self.get_logger().warn('reinitialize_global_localization 서비스 응답 없음')

    def publish_state(self):
        """외부 노드용 위치 상태 토픽 발행"""
        is_lost = (self.state == 'lost')
        self.emergency_pub.publish(Bool(data=is_lost))

        if self.state == 'lost':
            status = 'lost'
        elif self.state == 'uncertain':
            status = 'uncertain'
        else:
            status = 'ok'
        self.status_pub.publish(String(data=status))


def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()