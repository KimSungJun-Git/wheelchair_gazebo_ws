#!/usr/bin/env python3
# safety_stop_node.py
import math
import json
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool


class SafetyStopNode(Node):
    def __init__(self):
        super().__init__('safety_stop_node')
        self.create_subscription(Twist, '/cmd_vel_nav', self.nav_cmd_callback, 10)
        # 초음파 (RELIABLE QoS, 일반 publisher와 매칭)
        self.create_subscription(Range, '/ultrasonic/range', self.front_callback, 10)
        self.create_subscription(Range, '/ultrasonic/left', self.left_callback, 10)
        self.create_subscription(Range, '/ultrasonic/right', self.right_callback, 10)
        # 라이다, IMU, odom은 sensor_data 끊김 감지 위해 타임스탬프 기록
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
        # CPA 기반 동적 위협 스캔 토픽 구독
        self.create_subscription(LaserScan, '/scan_threat', self.scan_threat_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # 외부 비상 신호 — IMU와 Localization으로 분리
        self.create_subscription(Bool, '/emergency_stop/imu', self.imu_emergency_cb, 10)
        self.create_subscription(Bool, '/emergency_stop/localization', self.localization_emergency_cb, 10)

        self.nav_cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        
        self.zone_pub = self.create_publisher(String, '/current_zone', 10)
        self.avoid_pub = self.create_publisher(String, '/avoidance_direction', 10)
        self.alert_pub = self.create_publisher(String, '/safety_alert', 10)
        self.action_pub = self.create_publisher(String, '/safety_action', 10)
        self.health_pub = self.create_publisher(String, '/sensor_health', 10)

        # ===== 설정값 ===== (config/safety.yaml 에서 조정)
        self.declare_parameter('danger_distance_m', 0.20)   # 전방 정지 거리
        self.declare_parameter('clearance_min_m', 0.20)     # 회피 가능 판단 임계값
        self.danger_distance_m = self.get_parameter('danger_distance_m').get_parameter_value().double_value
        self.clearance_min_m = self.get_parameter('clearance_min_m').get_parameter_value().double_value

        # 동적 장애물(CPA 위협) 액추에이션 설정
        self.declare_parameter('threat_stop_dist_m', 1.00)   # 정지 대기 전방 거리 (1.2m -> 1.0m 단축)
        self.declare_parameter('threat_stop_width_m', 0.70)  # 정지 대기 폴리곤 폭 (±0.35m)
        self.declare_parameter('threat_slow_dist_m', 2.00)   # 서행 전방 거리
        self.declare_parameter('threat_slow_width_m', 0.90)  # 서행 폴리곤 폭 (±0.45m)
        self.declare_parameter('threat_min_points', 2)       # 위협 판정 최소 빔 수
        self.declare_parameter('emergency_scan_dist_m', 0.25)   # 원본 스캔 초근접 긴급 정지 거리 (회피 시 오발동 방지)
        self.declare_parameter('emergency_scan_width_m', 0.45)  # 원본 스캔 초근접 긴급 정지 폭

        self.threat_stop_dist_m = self.get_parameter('threat_stop_dist_m').get_parameter_value().double_value
        self.threat_stop_width_m = self.get_parameter('threat_stop_width_m').get_parameter_value().double_value
        self.threat_slow_dist_m = self.get_parameter('threat_slow_dist_m').get_parameter_value().double_value
        self.threat_slow_width_m = self.get_parameter('threat_slow_width_m').get_parameter_value().double_value
        self.threat_min_points = self.get_parameter('threat_min_points').get_parameter_value().integer_value
        self.emergency_scan_dist_m = self.get_parameter('emergency_scan_dist_m').get_parameter_value().double_value
        self.emergency_scan_width_m = self.get_parameter('emergency_scan_width_m').get_parameter_value().double_value

        # ===== 동적 위협 상태 및 정면 고정(Heading Lock) 제어 =====
        self.threat_state = 'CLEAR'  # 'STOP', 'SLOW', 'CLEAR'
        self.threat_nearest_dist = 99.0
        self.current_yaw = 0.0
        self.locked_yaw = None
        self.threat_clear_time = None

        # ===== 거리 데이터 =====
        self.dist_front = 9.9
        self.dist_left = 9.9
        self.dist_right = 9.9
        self.was_in_danger = False

        # ===== 센서 위험 상태 (센서 이름 → 위험 여부) =====
        # 하나라도 True면 비상 정지. 각 센서는 독립적으로 갱신됨.
        self.sensor_danger = {
            'imu_emergency': False,           # IMU 노드의 위험 신호 (기울기/충격)
            'localization_emergency': False,  # Localization 노드의 위험 신호 (위치 분실)
            'lidar_lost': False,              # 라이다 끊김
            'imu_lost': False,                # IMU 끊김
            'ultrasonic_lost': False,         # 초음파 끊김
            'odom_lost': False,               # 모터(오도메트리) 끊김
            'lidar_emergency': False,         # 라이다 0.35m 이내 초근접 긴급 충돌 위험
        }
        # 현재 위험 상태인 센서 목록과 종합 비상 여부 (set_reason에서 자동 갱신)
        self.dangerous_sensors = []
        self.is_emergency = False

        # 헬스 체크용 타임스탬프 o
        # None은 "한 번도 메시지가 안 옴"을 의미
        self.last_seen: Dict[str, Optional[Time]] = {
            'lidar': None,
            'imu': None,
            'odom': None,
            'ultrasonic_front': None,
            'ultrasonic_left': None,
            'ultrasonic_right': None,
        }
        # 센서별 타임아웃 (초) — 이 시간 안에 메시지가 안 오면 끊김 판정
        self.timeout_sec = {
            'lidar': 1.0,                # 일반적으로 5~10Hz
            'imu': 1.0,                  # 일반적으로 50~100Hz
            'odom': 1.0,                 # 일반적으로 20~50Hz
            'ultrasonic_front': 2.0,     # 초음파는 느림
            'ultrasonic_left': 2.0,
            'ultrasonic_right': 2.0,
        }
        # 부팅 직후 false positive 방지용 grace time
        self.start_time = self.get_clock().now()
        self.startup_grace_sec = 5.0

        # ===== /safety_action 발행 상태 =====
        # 예전에는 /cmd_vel_nav 콜백에서 바로 발행해 Nav2 주기·부하에 따라
        # 0Hz(정지 중) ~ 40Hz(주행 중)로 요동쳤다. 이제 상태가 바뀐 순간은
        # 즉시, 그 외에는 1Hz로만 내보내 로그의 시간축을 균일하게 만든다.
        self.last_action = None            # (action, reason)
        self.last_cmd_time = None          # 마지막 /cmd_vel_nav 수신 시각
        # 이 시간 넘게 명령이 없으면 idle로 기록. mode_switch의 cmd_timeout_sec과
        # 같은 현상을 보는 값이므로 함께 조정한다.
        self.declare_parameter('cmd_idle_timeout_sec', 1.0)
        self.cmd_idle_timeout_sec = self.get_parameter('cmd_idle_timeout_sec').get_parameter_value().double_value

        # 타이머
        self.create_timer(1.0, self.publish_zone)
        self.create_timer(0.1, self.check_danger_state)
        self.create_timer(0.5, self.check_sensor_health)
        self.create_timer(1.0, self.publish_action_heartbeat)

        self.get_logger().info(
            f'Safety Stop Node 시작 | 전방 정지: {self.danger_distance_m*100:.0f}cm | '
            f'헬스체크 활성화 (라이다/IMU/초음파/모터) | '
            f'입력: /cmd_vel_nav')
    #  센서 위험 상태 갱신
    def imu_emergency_cb(self, msg: Bool):
        self.set_reason('imu_emergency', msg.data)

    def localization_emergency_cb(self, msg: Bool):
        self.set_reason('localization_emergency', msg.data)
    
    def set_reason(self, key, active):
        """센서 위험 상태 변경 시 엣지 트리거 로그 (같은 상태면 로그 안 찍힘)"""
        prev = self.sensor_danger.get(key, False)
        if prev == active:
            return

        # 1) 상태 갱신
        self.sensor_danger[key] = active
        # 2) 파생 변수 즉시 갱신
        self.dangerous_sensors = [k for k, v in self.sensor_danger.items() if v]
        self.is_emergency = any(self.sensor_danger.values())

        # 3) 로그
        if active:
            self.get_logger().error(f'🛑 [{key}] 위험 상태 → 위험 센서: {self.dangerous_sensors}')
        else:
            if self.dangerous_sensors:
                self.get_logger().warn(f'✅ [{key}] 정상 복귀 → 남은 위험 센서: {self.dangerous_sensors}')
            else:
                self.get_logger().info(f'✅ [{key}] 정상 복귀 → 모든 센서 정상')

    #  센서 콜백 (마지막 수신 시각 기록)
    def lidar_callback(self, msg: LaserScan):
        self.last_seen['lidar'] = self.get_clock().now()

        # 원본 /scan에서 초근접(0.35m 이내) 긴급 충돌 위험 검사 (Fail-safe 긴급 정지)
        cnt_emerg = 0
        angle = msg.angle_min
        emerg_half_w = self.emergency_scan_width_m / 2.0

        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                if 0.05 < x <= self.emergency_scan_dist_m and abs(y) <= emerg_half_w:
                    cnt_emerg += 1
            angle += msg.angle_increment

        self.set_reason('lidar_emergency', cnt_emerg >= 3)

    def scan_threat_callback(self, msg: LaserScan):
        """CPA 기반 위협 스캔(/scan_threat)에서 정지 및 감속 영역 내 포인트 검사"""
        cnt_stop = 0
        cnt_slow = 0
        min_dist = 99.0

        angle = msg.angle_min
        stop_half_w = self.threat_stop_width_m / 2.0
        slow_half_w = self.threat_slow_width_m / 2.0

        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                # 전방 영역(x > 0.05m)만 검사
                if x > 0.05:
                    if x <= self.threat_stop_dist_m and abs(y) <= stop_half_w:
                        cnt_stop += 1
                        if r < min_dist:
                            min_dist = r
                    elif x <= self.threat_slow_dist_m and abs(y) <= slow_half_w:
                        cnt_slow += 1
                        if r < min_dist:
                            min_dist = r

            angle += msg.angle_increment

        self.threat_nearest_dist = min_dist
        prev_state = self.threat_state
        if cnt_stop >= self.threat_min_points:
            self.threat_state = 'STOP'
            if self.locked_yaw is None:
                self.locked_yaw = self.current_yaw
            self.threat_clear_time = None
        elif cnt_slow >= self.threat_min_points:
            self.threat_state = 'SLOW'
            if self.locked_yaw is None:
                self.locked_yaw = self.current_yaw
            self.threat_clear_time = None
        else:
            self.threat_state = 'CLEAR'
            if prev_state in ('STOP', 'SLOW'):
                self.threat_clear_time = self.get_clock().now()

        if prev_state != self.threat_state:
            if self.threat_state == 'STOP':
                self.get_logger().warn(
                    f'🛑 [Dynamic Threat] 보행자 접근 감지 -> 정지 대기 (거리: {min_dist:.2f}m, 점: {cnt_stop}개, 정면 헤딩 고정)'
                )
            elif self.threat_state == 'SLOW':
                self.get_logger().info(
                    f'⚠️ [Dynamic Threat] 보행자 전방 감지 -> 감속 서행 (거리: {min_dist:.2f}m, 점: {cnt_slow}개, 정면 유지)'
                )
            else:
                self.get_logger().info('✅ [Dynamic Threat] 보행자 통과/해소 -> 정상 주행 복귀 (급회전 방지 완충 적용)')

    def imu_callback(self, msg: Imu):
        self.last_seen['imu'] = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.last_seen['odom'] = self.get_clock().now()
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def front_callback(self, msg: Range):
        self.dist_front = msg.range
        self.last_seen['ultrasonic_front'] = self.get_clock().now()

    def left_callback(self, msg: Range):
        self.dist_left = msg.range
        self.last_seen['ultrasonic_left'] = self.get_clock().now()

    def right_callback(self, msg: Range):
        self.dist_right = msg.range
        self.last_seen['ultrasonic_right'] = self.get_clock().now()

    #  센서 상태 체크 (0.5초마다)
    def check_sensor_health(self):
        now = self.get_clock().now()

        # 부팅 직후 grace 시간 동안은 체크 스킵
        if (now - self.start_time).nanoseconds / 1e9 < self.startup_grace_sec:
            return

        groups = {
            'lidar_lost': ['lidar'],
            'imu_lost': ['imu'],
            'odom_lost': ['odom'],
            # 초음파가 안 들어오면 여기서 계속 emergency 처리돼서 로봇이 멈춰버림.
            # Gazebo 시뮬레이션엔 초음파 센서가 없어서 비활성화. (실제 로봇에서는 다시 켤 것)
            # 'ultrasonic_lost': ['ultrasonic_front', 'ultrasonic_left', 'ultrasonic_right'],
        }

        health_status = {}
        for danger_key, sensor_keys in groups.items():
            any_lost = False
            for sk in sensor_keys:
                last = self.last_seen[sk]
                timeout = self.timeout_sec[sk]
                if last is None:
                    health_status[sk] = 'never'
                    any_lost = True
                else:
                    elapsed = (now - last).nanoseconds / 1e9
                    if elapsed > timeout:
                        health_status[sk] = f'lost({elapsed:.1f}s)'
                        any_lost = True
                    else:
                        health_status[sk] = 'ok'

            self.set_reason(danger_key, any_lost)

        msg = String()
        msg.data = json.dumps(health_status, ensure_ascii=False)
        self.health_pub.publish(msg)

    #  Cmd Vel 게이트웨이
    def nav_cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.process_and_publish(msg, "Navigation", self.nav_cmd_pub)

    def process_and_publish(self, msg, source, publisher):
        """Nav2 cmd_vel을 공통 안전 필터로 검사한 뒤 지정된 토픽으로 출력"""

        # 1) 비상 정지 (IMU 충격, 위치 분실, 라이다 초근접 0.35m 등)
        if self.is_emergency:
            publisher.publish(Twist())
            self.publish_action(source, 'blocked', ','.join(self.dangerous_sensors))
            return
        
        # 2) 전진 명령 필터링
        safe_msg = Twist()
        safe_msg.linear.x = msg.linear.x
        safe_msg.linear.y = msg.linear.y
        safe_msg.linear.z = msg.linear.z
        safe_msg.angular.x = msg.angular.x
        safe_msg.angular.y = msg.angular.y
        safe_msg.angular.z = msg.angular.z

        # 2-1) 동적 장애물 위협(CPA 기반) 정지: 보행자가 다가오면 완전 정지 (전진 및 회전 완전 차단, 정면 고정)
        if self.threat_state == 'STOP':
            safe_msg.linear.x = 0.0
            safe_msg.angular.z = 0.0  # 회전 속도 완전 차단 (정면 고정)
            publisher.publish(safe_msg)
            self.publish_action(source, 'modified', 'threat_wait')
            return

        # 2-2) 동적 장애물 감속: 전방 접근 시 감속 서행 (회전 차단하여 직진 정면 유지)
        if self.threat_state == 'SLOW':
            safe_msg.linear.x = min(msg.linear.x * 0.4, 0.08) if msg.linear.x > 0.0 else 0.0
            safe_msg.angular.z = 0.0  # 감속 서행 중에도 회피 시도로 인한 90도 회전 차단 (정면 고정)
            publisher.publish(safe_msg)
            self.publish_action(source, 'modified', 'threat_slow')
            return

        # 2-3) 보행자 통과 직후 복귀 완충 (1.5초 동안 제자리 급회전 차단 및 정면 안정화)
        if self.threat_clear_time is not None:
            elapsed = (self.get_clock().now() - self.threat_clear_time).nanoseconds / 1e9
            if elapsed < 1.5:
                # 보행자가 지나간 직후 Nav2 controller가 내보내는 90도 제자리 회전 명령 원천 차단
                if abs(safe_msg.linear.x) < 0.05:
                    safe_msg.angular.z = 0.0
                else:
                    # 완만한 직진 주행 유도 (급격한 회전 방지)
                    safe_msg.angular.z = max(min(safe_msg.angular.z, 0.15), -0.15)
            else:
                self.threat_clear_time = None
                self.locked_yaw = None

        # 2-4) 초음파 전방 장애물 (하드웨어/실차 호환)
        if self.dist_front <= self.danger_distance_m:
            safe_msg.linear.x = 0.0
            safe_msg.angular.z = 0.0
            publisher.publish(safe_msg)
            self.publish_action(source, 'modified', 'obstacle_front')
            return

        # 3) 정상 통과
        publisher.publish(safe_msg)
        self.publish_action(source, 'allowed', '')

    def publish_action(self, source, action, reason, force=False):
        """상태가 바뀐 순간에만 발행. 같은 상태 반복은 heartbeat가 1Hz로 담당."""
        state = (action, reason)
        if not force and state == self.last_action:
            return
        self.last_action = state

        msg = String()
        msg.data = json.dumps({
            'source': source,
            'action': action,
            'reason': reason,
        }, ensure_ascii=False)
        self.action_pub.publish(msg)

    def publish_action_heartbeat(self):
        """현재 안전 상태를 1Hz로 재발행 — 주행 중이든 정지 중이든 시간축이 끊기지 않는다."""
        idle = (
            self.last_cmd_time is None or
            (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            > self.cmd_idle_timeout_sec
        )
        if idle:
            # 명령이 끊긴 구간도 기록돼야 궤적을 복원할 수 있다
            self.publish_action('Navigation', 'idle',
                                ','.join(self.dangerous_sensors), force=True)
        elif self.last_action is not None:
            self.publish_action('Navigation', *self.last_action, force=True)

    #  회피 방향 결정
    def check_danger_state(self):
        in_danger_now = self.dist_front <= self.danger_distance_m

        if in_danger_now and not self.was_in_danger:
            direction = self.decide_avoidance()
            self.get_logger().error(
                f'⚠️ 전방 장애물 진입! F:{self.dist_front*100:.1f}cm '
                f'L:{self.dist_left*100:.1f}cm R:{self.dist_right*100:.1f}cm '
                f'권장 회피: {direction}')
            self.alert_pub.publish(String(data='obstacle_too_close'))
            self.avoid_pub.publish(String(data=direction))
        elif not in_danger_now and self.was_in_danger:
            self.get_logger().info(f'✅ 전방 장애물 해소. F:{self.dist_front*100:.1f}cm')
            self.alert_pub.publish(String(data='obstacle_cleared'))
            # 해소를 알리지 않으면 수집기에 옛 회피 판단이 계속 붙어 있게 된다
            self.avoid_pub.publish(String(data='none'))

        self.was_in_danger = in_danger_now

    def decide_avoidance(self):
        if self.dist_left < self.clearance_min_m and \
           self.dist_right < self.clearance_min_m:
            return 'blocked'
        if self.dist_left > self.dist_right:
            return 'left'
        elif self.dist_right > self.dist_left:
            return 'right'
        else:
            return 'left'

    def publish_zone(self):
        zone_msg = String()
        if self.is_emergency:
            zone_msg.data = f'비상정지 | 위험 센서: {",".join(self.dangerous_sensors)}'
        elif self.threat_state == 'STOP':
            zone_msg.data = f'동적대기(보행자접근) | 전방 {self.threat_nearest_dist:.2f}m'
        elif self.threat_state == 'SLOW':
            zone_msg.data = f'서행구역(보행자감지) | 전방 {self.threat_nearest_dist:.2f}m'
        elif self.dist_front <= self.danger_distance_m:
            zone_msg.data = (
                f'위험구역(정지) | F:{self.dist_front*100:.1f}cm '
                f'L:{self.dist_left*100:.1f}cm R:{self.dist_right*100:.1f}cm')
        else:
            zone_msg.data = (
                f'일반구역 | F:{self.dist_front*100:.1f}cm '
                f'L:{self.dist_left*100:.1f}cm R:{self.dist_right*100:.1f}cm')
        self.zone_pub.publish(zone_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()