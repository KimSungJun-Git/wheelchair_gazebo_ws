#!/usr/bin/env python3
# imu_safety_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
import math


class ImuSafetyNode(Node):
    def __init__(self):
        super().__init__('imu_safety_node')
        
        # IMU에 sensor_data QoS 적용 (드라이버와 매칭)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos_profile_sensor_data)

        # 비상정지 신호 및 상세 상태 토픽 발행
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop/imu', 10)
        self.sos_pub = self.create_publisher(String, '/sos_trigger', 10)

        # 임계값 설정
        self.tilt_threshold_deg = 45.0       # 기울기 한계 (roll/pitch) [deg]
        self.impact_threshold = 45.0         # 충격 가속도 한계 [m/s^2]
        self.recovery_time_sec = 2.0         # 정상 복귀 판정 대기 시간 [s]

        # 상태 변수
        self.in_emergency = False
        self.last_trigger_time = None

        self.create_timer(0.1, self.publish_state)
        self.get_logger().info(
            f'imu_safety_node 시작 (기울기 임계값: {self.tilt_threshold_deg} deg, '
            f'충격 임계값: {self.impact_threshold} m/s^2)'
        )

    def imu_cb(self, msg: Imu):
        orientation = msg.orientation
        sin_roll = 2.0 * (orientation.w * orientation.x + orientation.y * orientation.z)
        cos_roll = 1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y)
        roll = math.degrees(math.atan2(sin_roll, cos_roll))

        sin_pitch = 2.0 * (orientation.w * orientation.y - orientation.z * orientation.x)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sin_pitch))))

        # 센서 장착 각도 보정 (평균 raw roll: 3.16 deg, raw pitch: 48.40 deg)
        roll = roll - 3.16
        pitch = pitch - 48.40

        spin_rate = abs(msg.angular_velocity.z)
        is_spinning = spin_rate > 0.3 

        # 선형 가속도 크기 계산
        linear_accel = msg.linear_acceleration
        accel_magnitude = math.sqrt(linear_accel.x**2 + linear_accel.y**2 + linear_accel.z**2)

        # 위험 상태 판정 (선회 중 기울기 오감지 방지)
        if is_spinning:
            is_tilt_danger = False
        else:
            is_tilt_danger = abs(roll) > self.tilt_threshold_deg or abs(pitch) > self.tilt_threshold_deg

        self.get_logger().info(
            f'Roll: {roll:.2f} deg, Pitch: {pitch:.2f} deg | 회전 중: {is_spinning}',
            throttle_duration_sec=1.0)
        
        is_impact_danger = accel_magnitude > self.impact_threshold

        if is_tilt_danger or is_impact_danger:
            if not self.in_emergency:
                reason = '기울기' if is_tilt_danger else '충격'
                detail = f'roll={roll:.1f}° pitch={pitch:.1f}°' if is_tilt_danger else f'accel={accel_magnitude:.1f}m/s²'
                self.get_logger().error(f'[imu_safety] 위험 감지 ({reason}): {detail}')

                self.in_emergency = True
                self.last_trigger_time = self.get_clock().now()

                self.sos_pub.publish(String(data=f'imu_{reason}:{detail}'))
        else:
            # 정상 상태 지속 시 비상 해제
            if self.in_emergency and self.last_trigger_time is not None:
                elapsed = (self.get_clock().now() - self.last_trigger_time).nanoseconds / 1e9
                if elapsed > self.recovery_time_sec:
                    self.in_emergency = False
                    self.get_logger().info('[imu_safety] 정상 상태 복귀')

    def publish_state(self):
        self.emergency_pub.publish(Bool(data=self.in_emergency))


def main():
    rclpy.init()
    node = ImuSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()