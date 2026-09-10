#!/usr/bin/env python3
#mode_switch_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String, Empty
from action_msgs.srv import CancelGoal
from typing import Optional
from collections import deque
import sys
import termios
import tty
import threading
import time
import math
import numpy as np

def quaternion_from_yaw(yaw):
    """yaw(rad) → (x, y, z, w) quaternion"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def yaw_from_quaternion(x, y, z, w):
    """(x, y, z, w) quaternion → yaw(rad)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class ModeSwitchNode(Node):
    def __init__(self):
        super().__init__('mode_switch_node')
        
        # 목적지 이름으로 이동 명령
        self.dest_sub = self.create_subscription(String, '/destination', self.destination_callback, 10)
        # 홈 귀환 명령 (웹 UI → Empty 메시지)
        self.home_sub = self.create_subscription(Empty, '/go_home', self.go_home_callback, 10)
        # 3. RViz 등에서 클릭한 목적지 좌표 수신 (RViz2는 Volatile QoS)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        # 4. Nav2가 생성한 전체 이동 경로 수신
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_cb, 10)
        
        # 1. 외부에서 모드 전환 명령 수신 (수동 <-> 자율)
        self.mode_cmd_sub = self.create_subscription(String, '/mode_switch', self.mode_cmd_callback, 10)
        # 안전 알림 (safety_stop_node → 금지구역 진입 시 goal 취소)
        self.safety_sub = self.create_subscription(String, '/safety_alert', self.safety_alert_callback, 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)####

        # ===== 토픽 구독 =====
        self.nav_cmd_sub = self.create_subscription(Twist, '/cmd_vel_safe', self.nav_cmd_callback, 10)
        self.teleop_cmd_sub = self.create_subscription(Twist, '/cmd_vel_teleop', self.teleop_cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ===== Nav2 Action Client =====
        # 1. 목적지로 이동을 지시하는 Action Client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # ===== Nav2 Cancel Service =====
        # 2. 진행 중인 이동을 강제 취소하는 Service Client
        self._cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        # ===== 모드 상태 =====
        self.declare_parameter('default_mode', 'manual')
        self.mode = self.get_parameter('default_mode').get_parameter_value().string_value
        self.last_goal: Optional[PoseStamped] = None
        self._goal_handle = None
        self._goal_locked = False

        # ===== [기술보고서 v9 Phase 1] Goal & Path 보존 =====
        self.preserved_goal: Optional[PoseStamped] = None
        self.preserved_path: Optional[list] = None  # [{'x': ..., 'y': ...}, ...]
        preserved_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.preserved_goal_pub = self.create_publisher(PoseStamped, '/preserved_goal', preserved_qos)
        self.preserved_path_pub = self.create_publisher(Path, '/preserved_path', preserved_qos)

        # ===== [기술보고서 v9 Phase 1~2] Fallback & AMCL Re-seeding =====
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.loc_status_sub = self.create_subscription(String, '/localization_status', self.loc_status_cb, 10)

        # AMCL 자동 재배치 (/initialpose) 퍼블리셔
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 2초 링 버퍼 (10Hz * 2s = 20개 샘플) — map, odom, imu 전체 상태 스냅샷 보존
        self.pose_buffer = deque(maxlen=20)
        self.last_valid_map = None   # {'x': ..., 'y': ..., 'yaw': ...}
        self.last_valid_odom = None  # {'x': ..., 'y': ..., 'yaw': ..., 'v_lin': ..., 'v_ang': ...}
        self.last_valid_imu = None   # {'yaw': ..., 'ang_vel_z': ..., 'acc_x': ..., 'acc_y': ...}

        # Fallback 제어 상태 변수
        self.fallback_active = False
        self.anchor_map = None       # 튀기 전 정상 map 좌표
        self.anchor_odom = None      # 튀기 전 odom 좌표
        self.anchor_imu = None       # 튀기 전 imu 좌표/헤딩
        self.target_distance = 0.0   # 목적지까지 총 직선거리
        self.target_global_yaw = 0.0 # 목적지 절대 방향각
        self.fallback_start_time = 0.0
        self._reseed_timer = None
        self._last_fallback_log_time = 0.0
        self._last_fallback_tick_time = 0.0

        # 실시간 센서 캐시 (전체 필드 보존)
        self.current_odom = None     # {'x': ..., 'y': ..., 'yaw': ..., 'v_lin': ..., 'v_ang': ...}
        self.current_imu = None      # {'yaw': ..., 'ang_vel_z': ..., 'acc_x': ..., 'acc_y': ...}
        self.current_scan = None
        self.latest_cov_x = 0.0
        self.latest_cov_y = 0.0
        self.loc_status = 'ok'

        # 대기소(홈) 좌표 — SLAM 맵에서 확인 후 수정 
        self.destinations = {
            'home':       {'x': 1.815,  'y': 1.179,  'yaw': 0.0},   # 시작지점 (대기소)
            'room_101':   {'x': 2.146,  'y': 0.003,  'yaw': 0.0},   # 101호
            'room_102':   {'x': -0.338, 'y': -0.910, 'yaw': 0.0},   # 102호
            'emergency':  {'x': -0.751, 'y': 0.272,  'yaw': 0.0},   # 응급실
        }
        self.home_pose = self.destinations['home']
        
        # ===== 최신 명령 저장 =====
        self.latest_nav_cmd = Twist()
        self.latest_teleop_cmd = Twist()
        self.last_teleop_time = 0.0
        self.last_nav_time = 0.0
        # 이 시간 넘게 명령이 끊기면 정지. Nav2 controller_frequency보다 넉넉해야
        # 경로 재계획·제자리 회전 구간에서 주행이 끊기지 않는다.
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.cmd_timeout_sec = self.get_parameter('cmd_timeout_sec').get_parameter_value().double_value

        # ===== 6.3.1절 RANSAC 벽면 상대각 기반 헤딩 드리프트 억제 파라미터 =====
        self.declare_parameter('wall_heading_constraint_enabled', True)
        self.declare_parameter('parallel_wall_tol_deg', 6.0)
        self.declare_parameter('wall_ransac_min_inliers', 12)
        self.declare_parameter('wall_meas_noise_r', 0.05)
        self.declare_parameter('steering_cooldown_sec', 1.5)

        self.wall_heading_constraint_enabled = self.get_parameter('wall_heading_constraint_enabled').get_parameter_value().bool_value
        self.parallel_wall_tol_deg = self.get_parameter('parallel_wall_tol_deg').get_parameter_value().double_value
        self.wall_ransac_min_inliers = self.get_parameter('wall_ransac_min_inliers').get_parameter_value().integer_value
        self.wall_meas_noise_r = self.get_parameter('wall_meas_noise_r').get_parameter_value().double_value
        self.steering_cooldown_sec = self.get_parameter('steering_cooldown_sec').get_parameter_value().double_value

        # RANSAC 평행벽 구속 런타임 상태 변수
        self.theta_wall_odom = None          # 복도 진입 시점 odom 기준 벽면 절대각 Anchor
        self.last_steer_time = 0.0           # 마지막 급조향 발생 시각
        self.parallel_wall_active = False    # 현재 평행벽 락 활성화 여부
        self.last_wall_innov_deg = 0.0       # 마지막 측정된 innovation (deg)
        # ===== 타이머 =====
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.2, self.publish_mode)

        # ===== 키보드 입력 쓰레드 =====
        # ros2 launch로 띄우면 stdin이 tty가 아니라 termios가 예외를 던진다.
        if sys.stdin.isatty():
            self.key_thread = threading.Thread(target=self.key_listener, daemon=True)
            self.key_thread.start()
        else:
            self.get_logger().info('stdin이 tty가 아님 — 키보드 입력 비활성화 (/mode_switch 토픽 사용)')
        
        # ===== SOS 처리 =====
        self.sos_sub = self.create_subscription(
            String, '/sos_trigger', self.sos_callback, 10)
        self.sos_count = 0
        
        self._current_destination = None # 현재 이동 중인 목적지 이름 저장용 변수

        self.get_logger().info(
            f'Mode Switch Node 시작 - 현재 모드: {self.mode}\n'
            f'  [m] 모드 전환 (manual <-> auto)\n'
            f'  [h] 홈 귀환\n'
            f'  [q] 종료\n'
            f'  홈 좌표: x={self.home_pose["x"]}, y={self.home_pose["y"]}')

    # ===== 목적지 수신 =====
    def destination_callback(self, msg):
        """웹 UI외부에서 목적지 이름 받아서 이동"""
        name = msg.data.strip().lower()

        if name not in self.destinations:
            self.get_logger().warn(
                f'알 수 없는 목적지: "{name}" | 사용 가능: {list(self.destinations.keys())}')
            return

        self.get_logger().info(f'목적지 수신: {name}')
        self._send_destination_goal(name)

    # ===== 통합 목적지 이동 (홈 귀환 포함) =====
    def _send_destination_goal(self, name):
        """지정된 이름의 목적지로 NavigateToPose 전송"""
        if name not in self.destinations:
            self.get_logger().warn(f'알 수 없는 목적지: "{name}"')
            return

        dest = self.destinations[name]

        # 자율주행 모드로 전환
        #if self.mode == 'manual':
        if self.mode != 'auto':
            self.reset_cmds()
            self.mode = 'auto'
            self.get_logger().info(f'>>> {name}(으)로 이동을 위해 자율주행 모드로 전환')

        # 기존 goal 취소
        self.cancel_nav()

        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('navigate_to_pose 서버 연결 실패')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = dest['x']
        goal.pose.pose.position.y = dest['y']

        q = quaternion_from_yaw(dest['yaw'])
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        # [기술보고서 v9 Phase 1] 목적지 좌표 보존 및 발행
        self.last_goal = goal.pose
        self.preserved_goal = goal.pose
        self.preserved_goal_pub.publish(self.preserved_goal)

        self.get_logger().info(f'{name}(으)로 이동: x={dest["x"]:.2f}, y={dest["y"]:.2f} (목적지 보존 완료)')

        # 현재 진행 중인 목적지 이름 저장 (도착 콜백에서 사용)
        self._current_destination = name

        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._dest_feedback_cb)
        send_future.add_done_callback(self._dest_response_cb)

    # 콜백들 (이름 변경: home → dest) 
    def _dest_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'{self._current_destination} Goal 거부됨!')
            return
        self.get_logger().info(f'{self._current_destination} Goal 수락 - 이동 중')
        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dest_result_cb)

    def _dest_feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'{self._current_destination}(으)로 이동 중... 남은 거리: {remaining:.2f}m')

    def _dest_result_cb(self, future):
        self._goal_locked = False
        self.get_logger().info(
            f'{self._current_destination} 도착 완료 → 수동 모드로 전환')
        self.mode = 'manual'
        self._goal_handle = None
        self._current_destination = None
        #self.cmd_pub.publish(Twist())
        self.reset_cmds()
    # ===== 홈 귀환 =====
    def go_home_callback(self, msg):
        self.get_logger().info('홈 귀환 명령 수신')
        self._send_home_goal()

    def _send_home_goal(self):
        """대기소 좌표로 NavigateToPose 전송"""
        # 자율주행 모드로 전환
        #if self.mode == 'manual':
        if self.mode != 'auto':
            self.reset_cmds()
            self.mode = 'auto'
            self.get_logger().info('>>> 홈 귀환을 위해 자율주행 모드로 전환')

        # 기존 goal 취소
        self.cancel_nav()

        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('navigate_to_pose 서버 연결 실패')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.home_pose['x']
        goal.pose.pose.position.y = self.home_pose['y']

        q = quaternion_from_yaw(self.home_pose['yaw'])
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        # [기술보고서 v9 Phase 1] 목적지 좌표 보존 및 발행
        self.last_goal = goal.pose
        self.preserved_goal = goal.pose
        self.preserved_goal_pub.publish(self.preserved_goal)

        self.get_logger().info(
            f'홈 좌표로 이동: x={self.home_pose["x"]}, y={self.home_pose["y"]} (목적지 보존 완료)')

        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._home_feedback_cb)
        send_future.add_done_callback(self._home_response_cb)
    
    def _home_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('홈 귀환 Goal 거부됨!')
            return
        self.get_logger().info('홈 귀환 Goal 수락 - 이동 중')
        self._goal_handle = goal_handle

        # 도착 완료 콜백
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._home_result_cb)

    def _home_feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'귀환 중... 남은 거리: {remaining:.2f}m')

    def _home_result_cb(self, future):
        self._goal_locked = False
        self.get_logger().info('홈 도착 완료 → 수동 모드로 전환')
        self.mode = 'manual'
        self._goal_handle = None
        #self.cmd_pub.publish(Twist())
        self.reset_cmds()
    # ===== 안전 알림 처리 =====
    def safety_alert_callback(self, msg):
        if msg.data == 'keepout_violation':
            #self.get_logger().error('🚫 금지구역 진입 → Nav2 goal 취소 + 수동 전환')
            #self.cancel_nav()
            #self.mode = 'manual'
            #self.cmd_pub.publish(Twist())
            self.get_logger().error('🚫 금지구역 진입 → 주행 goal 취소 + 수동 전환')
            self.set_mode('manual')
    
        elif msg.data == 'obstacle_too_close':
            # 자율 모드 중일 때만 처리 (수동 모드면 이미 사용자가 조작 중)
            #if self.mode == 'auto':
            # 자동 계열 모드 중일 때만 강제 수동 전환
            if self.mode == 'auto':
                self.get_logger().error(
                    '⚠️ 전방 장애물 감지 → 주행 모드 해제 + 수동 모드 전환\n'
                    '    탑승자: 직접 회피 후 [m] 키로 주행 재시작 가능')
                self.set_mode('manual')
            else:
                self.get_logger().warn(
                    '⚠️ 전방 장애물 (수동 모드 중) — 직접 조작으로 회피하세요')
    
        elif msg.data == 'obstacle_cleared':
            # 정보용 로그만, 자동 재시작 안 함 (사용자가 결정)
            self.get_logger().info(
                '✅ 전방 장애물 해소. 자율주행은 [m] 키로 재시작')

    # ===== [기술보고서 v9 Phase 1~2] Fallback & Resilient Navigation =====
    def imu_cb(self, msg: Imu):
        o = msg.orientation
        yaw = yaw_from_quaternion(o.x, o.y, o.z, o.w)
        self.current_imu = {
            'yaw': yaw,
            'ang_vel_z': msg.angular_velocity.z,
            'acc_x': msg.linear_acceleration.x,
            'acc_y': msg.linear_acceleration.y,
            'stamp': msg.header.stamp
        }

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(o.x, o.y, o.z, o.w)
        self.current_odom = {
            'x': p.x,
            'y': p.y,
            'z': p.z,
            'yaw': yaw,
            'v_lin': msg.twist.twist.linear.x,
            'v_ang': msg.twist.twist.angular.z,
            'stamp': msg.header.stamp
        }

        # [핵심 내결함성] Fallback 모드일 때: /clock 정지 또는 타이머 지연과 무관하게
        # 실시간 Odom 센서 수신(30Hz)에 맞춰 즉시 Fallback 주행 제어기를 구동 (최대 20Hz 보장)
        if self.mode == 'fallback' and self.fallback_active:
            now = time.time()
            if now - self._last_fallback_tick_time >= 0.045:
                self._last_fallback_tick_time = now
                self.fallback_control_tick()

    def scan_cb(self, msg: LaserScan):
        self.current_scan = msg

    # ===== 6.3.1절 RANSAC 벽면 검출 알고리즘 =====
    def _ransac_line_fit(self, pts, max_iterations=25, dist_thresh=0.06):
        """2D 점 군집에서 RANSAC으로 최적 직선 피팅 (base_link 기준 상대각 산출)"""
        n = len(pts)
        if n < self.wall_ransac_min_inliers:
            return 0, 0.0

        best_inliers = 0
        best_angle = 0.0
        for _ in range(max_iterations):
            idx1, idx2 = np.random.choice(n, size=2, replace=False)
            p1 = pts[idx1]
            p2 = pts[idx2]

            dx = float(p2[0] - p1[0])
            dy = float(p2[1] - p1[1])
            seg_len = math.hypot(dx, dy)
            if seg_len < 0.25:
                continue

            nx = -dy / seg_len
            ny = dx / seg_len

            dists = np.abs(nx * (pts[:, 0] - p1[0]) + ny * (pts[:, 1] - p1[1]))
            inliers = int(np.sum(dists < dist_thresh))

            if inliers > best_inliers:
                best_inliers = inliers
                angle = math.atan2(dy, dx)
                # 벽면은 차체 전후축(X)과 나란하므로 각도를 [-pi/2, pi/2]로 정규화
                if angle > math.pi / 2.0:
                    angle -= math.pi
                elif angle < -math.pi / 2.0:
                    angle += math.pi
                best_angle = angle

        return best_inliers, best_angle

    def detect_parallel_walls(self, scan_msg: LaserScan):
        """LiDAR 스캔에서 좌/우측 벽면을 검출하고 상호 평행성(Corridor) 판정"""
        if scan_msg is None or len(scan_msg.ranges) == 0:
            return False, 0.0, 0, 0, 0.0, 0.0

        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        n = len(ranges)
        angles = scan_msg.angle_min + np.arange(n) * scan_msg.angle_increment

        # 유효 범위 필터: 0.3m ~ 3.5m
        valid = np.isfinite(ranges) & (ranges >= 0.30) & (ranges <= 3.50)
        valid_ranges = ranges[valid]
        valid_angles = angles[valid]

        xs = valid_ranges * np.cos(valid_angles)
        ys = valid_ranges * np.sin(valid_angles)

        # 좌측 영역: y > 0.15, x in [-1.0, 2.5], 각도 20도 ~ 115도
        left_mask = (ys > 0.15) & (xs >= -1.0) & (xs <= 2.5) & (valid_angles >= math.radians(20.0)) & (valid_angles <= math.radians(115.0))
        # 우측 영역: y < -0.15, x in [-1.0, 2.5], 각도 -115도 ~ -20도
        right_mask = (ys < -0.15) & (xs >= -1.0) & (xs <= 2.5) & (valid_angles <= math.radians(-20.0)) & (valid_angles >= math.radians(-115.0))

        left_pts = np.column_stack((xs[left_mask], ys[left_mask]))
        right_pts = np.column_stack((xs[right_mask], ys[right_mask]))

        inliers_l, ang_l = self._ransac_line_fit(left_pts)
        inliers_r, ang_r = self._ransac_line_fit(right_pts)

        if inliers_l >= self.wall_ransac_min_inliers and inliers_r >= self.wall_ransac_min_inliers:
            angle_diff = abs(ang_l - ang_r)
            if angle_diff <= math.radians(self.parallel_wall_tol_deg):
                avg_wall_rel = (ang_l + ang_r) / 2.0
                return True, avg_wall_rel, inliers_l, inliers_r, ang_l, ang_r

        return False, 0.0, inliers_l, inliers_r, ang_l, ang_r

    def loc_status_cb(self, msg: String):
        st = msg.data.strip()
        self.loc_status = st
        # 위치추정이 uncertain 또는 lost로 전이될 때, auto 모드이면 즉시 Fallback 돌파 트리거
        if st in ('uncertain', 'lost') and self.mode == 'auto' and not self.fallback_active:
            if self.last_valid_map is not None:
                self.get_logger().warn(f'⚠️ 위치추정 상태 저하 ({st}) ➔ Fallback Dead-reckoning 모드 진입!')
                self.enter_fallback()

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(o.x, o.y, o.z, o.w)
        cov = msg.pose.covariance
        cov_x = float(cov[0])
        cov_y = float(cov[7])
        cov_yaw = float(cov[35])
        self.latest_cov_x = cov_x
        self.latest_cov_y = cov_y

        now = time.time()
        # 공분산이 0.5 미만으로 정상일 때만 2초 링 버퍼에 map, odom, imu 전체 상태 기록
        if cov_x < 0.5 and cov_y < 0.5 and self.current_odom is not None:
            sample = {
                'time': now,
                'map': {'x': p.x, 'y': p.y, 'yaw': yaw},
                'odom': dict(self.current_odom),
                'imu': dict(self.current_imu) if self.current_imu is not None else {'yaw': yaw, 'ang_vel_z': 0.0, 'acc_x': 0.0, 'acc_y': 0.0},
                'cov_x': cov_x,
                'cov_y': cov_y,
                'cov_yaw': cov_yaw
            }
            self.pose_buffer.append(sample)

            # 버퍼에서 약 1.5초~2.0초 전의 가장 안정적인 샘플을 last_valid로 고정
            if len(self.pose_buffer) >= 5:
                clean_sample = self.pose_buffer[0]
                self.last_valid_map = clean_sample['map']
                self.last_valid_odom = clean_sample['odom']
                self.last_valid_imu = clean_sample['imu']
            else:
                self.last_valid_map = sample['map']
                self.last_valid_odom = sample['odom']
                self.last_valid_imu = sample['imu']
        else:
            # 공분산이 0.5를 초과한 경우 Fallback 진입 검사
            if self.mode == 'auto' and not self.fallback_active:
                if self.last_valid_map is not None:
                    self.get_logger().warn(
                        f'⚠️ AMCL 공분산 폭증 (cov_x={cov_x:.2f}, cov_y={cov_y:.2f}) ➔ Fallback 모드 진입!')
                    self.enter_fallback()

    def enter_fallback(self):
        if self.fallback_active:
            return
        if self.last_valid_map is None or self.last_valid_odom is None:
            self.get_logger().warn('Fallback 진입 불가: 기준 좌표 없음 ➔ manual 정지')
            self.set_mode('manual')
            return

        # [안전 인터락 (Fail-Safe)] 보존된 목적지와 경로가 모두 없으면 맹목적 전진 금지 ➔ 즉시 안전 정지
        if self.preserved_goal is None and not self.preserved_path:
            self.get_logger().error('🚨 [위치 튕김] 보존된 경로/목적이 없어 전진할 수 없습니다 ➔ 안전 정지 및 수동 전환')
            self.cancel_nav()
            self.set_mode('manual')
            return

        self.fallback_active = True
        self.mode = 'fallback'
        self.publish_mode()

        # Nav2 주행 액션 안전 취소 (맵 밖으로 튀어버린 경로 차단)
        self.cancel_nav()
        self.reset_cmds()

        # 2초 전 정상 좌표를 Anchor로 확정 고정 (map, odom, imu 전체 상태 락)
        self.anchor_map = dict(self.last_valid_map)
        self.anchor_odom = dict(self.last_valid_odom)
        self.anchor_imu = dict(self.last_valid_imu) if self.last_valid_imu is not None else None

        # RANSAC 평행벽 앵커 초기화
        self.theta_wall_odom = None
        self.parallel_wall_active = False
        self.last_wall_innov_deg = 0.0
        self.last_steer_time = 0.0

        if self.preserved_path and len(self.preserved_path) > 0:
            final_pt = self.preserved_path[-1]
            dx = final_pt['x'] - self.anchor_map['x']
            dy = final_pt['y'] - self.anchor_map['y']
            self.target_distance = math.hypot(dx, dy)
            self.target_global_yaw = math.atan2(dy, dx)
            path_info = f'보존된 경로: {len(self.preserved_path)}개 웨이포인트 Lookahead(0.9m) Pure Pursuit 추종 시작'
        elif self.preserved_goal is not None:
            gx = self.preserved_goal.pose.position.x
            gy = self.preserved_goal.pose.position.y
            dx = gx - self.anchor_map['x']
            dy = gy - self.anchor_map['y']
            self.target_distance = math.hypot(dx, dy)
            self.target_global_yaw = math.atan2(dy, dx)
            path_info = f'보존된 목적지: x={gx:.2f}, y={gy:.2f} (직선거리: {self.target_distance:.2f}m)'
        else:
            path_info = '경로/목표 없음'

        self.fallback_start_time = time.time()
        self._last_fallback_log_time = self.fallback_start_time

        imu_info = f"IMU yaw: {math.degrees(self.anchor_imu['yaw']):.1f}°" if self.anchor_imu else "IMU N/A"
        self.get_logger().warn(
            f'🚀🚀 [FALLBACK CONTROLLER 기동] 🚀🚀\n'
            f'  - 2초 전 정상 위치: x={self.anchor_map["x"]:.2f}, y={self.anchor_map["y"]:.2f} ({imu_info})\n'
            f'  - {path_info}\n'
            f'  - IMU+Odom 융합 Lookahead Pure Pursuit 경로 추종 돌파 시작!')

        # 진입 즉시 1차 제어 연산 실행 (정지 지연 원천 차단)
        self.fallback_control_tick()

    def fallback_control_tick(self):
        if not self.fallback_active:
            return
        if self.current_odom is None or self.anchor_odom is None:
            return
        if self.anchor_map is None:
            return

        anchor_map = self.anchor_map
        anchor_odom = self.anchor_odom
        current_odom = self.current_odom
        preserved_goal = self.preserved_goal
        preserved_path = self.preserved_path

        # 1) Odom 기준 누적 이동거리 및 IMU/Odom 기반 회전량 계산
        dx_odom = current_odom['x'] - anchor_odom['x']
        dy_odom = current_odom['y'] - anchor_odom['y']
        d_traveled = math.sqrt(dx_odom * dx_odom + dy_odom * dy_odom)

        # 차체 회전각: IMU가 있으면 휠 슬립 없는 IMU 우선 사용, 없으면 Odom yaw 사용
        if self.current_imu is not None and self.anchor_imu is not None:
            dyaw = self.current_imu['yaw'] - self.anchor_imu['yaw']
        else:
            dyaw = current_odom['yaw'] - anchor_odom['yaw']

        # 2) 회전 변환으로 실시간 맵 상의 예상 위치(P_est) 계산
        # theta0: Odom 프레임의 변위를 Map 프레임으로 회전시키는 상대 오프셋 각도
        theta0 = anchor_map['yaw'] - anchor_odom['yaw']
        dx_map = dx_odom * math.cos(theta0) - dy_odom * math.sin(theta0)
        dy_map = dx_odom * math.sin(theta0) + dy_odom * math.cos(theta0)
        est_map_x = anchor_map['x'] + dx_map
        est_map_y = anchor_map['y'] + dy_map
        est_map_yaw = anchor_map['yaw'] + dyaw

        # ===== 6.3.1절 RANSAC 벽면 상대각 기반 헤딩 드리프트 억제 =====
        wall_lock_active = False
        wall_innov_deg = 0.0
        if self.wall_heading_constraint_enabled and self.current_scan is not None:
            # 1) 게이팅: 의도적 급조향 후 쿨다운 검사
            time_since_steer = time.time() - self.last_steer_time
            if time_since_steer >= self.steering_cooldown_sec:
                # 2) RANSAC 좌우 평행벽 검출
                is_parallel, alpha_rel_meas, inl_l, inl_r, ang_l, ang_r = self.detect_parallel_walls(self.current_scan)
                if is_parallel:
                    psi_odom = current_odom['yaw']
                    # 복도 최초 진입 시 벽면 odom 절대각 앵커 고정
                    if self.theta_wall_odom is None:
                        self.theta_wall_odom = psi_odom + alpha_rel_meas
                        self.get_logger().info(
                            f'🔒 [RANSAC 복도 앵커 고정] θ_wall_odom={math.degrees(self.theta_wall_odom):.1f}° '
                            f'(α_rel={math.degrees(alpha_rel_meas):.1f}°, L:{inl_l}개/R:{inl_r}개)')
                    else:
                        # 매 주기 예측값 및 innovation 산출
                        alpha_rel_pred = self.theta_wall_odom - psi_odom
                        innov = alpha_rel_meas - alpha_rel_pred
                        innov = math.atan2(math.sin(innov), math.cos(innov))

                        # 3) 게이팅: 이상치 필터 (15도 이내일 때만 보정)
                        if abs(innov) <= math.radians(15.0):
                            k_gain = 0.04 / (0.04 + self.wall_meas_noise_r)
                            # est_map_yaw에 직접 혁신량 피드백 보정 적용 (상대각 고정)
                            est_map_yaw += k_gain * innov
                            wall_lock_active = True
                            wall_innov_deg = math.degrees(innov)
                            self.last_wall_innov_deg = wall_innov_deg

        self.parallel_wall_active = wall_lock_active

        # 3) 안전 한계(Drift Budget) 검사: 2분(120초) 초과 또는 30m 초과 시 FAIL_SAFE 정지
        elapsed = time.time() - self.fallback_start_time
        if elapsed > 120.0 or d_traveled > 30.0:
            self.get_logger().error(f'🚨 [FAIL_SAFE] Fallback 한계 초과 (경과: {elapsed:.1f}s, 이동: {d_traveled:.1f}m) ➔ 수동 모드 전환')
            self.set_mode('manual')
            return

        # 4) 탈출 판정 (벽면 재발견 또는 목표 거리 도달)
        # 최소 1.5m 이상 진행한 후 벽면 재발견 검사 (진입 직후 오발동 방지)
        is_wall_detected = False
        min_front_dist = 9.9
        if self.current_scan is not None:
            ranges = self.current_scan.ranges
            num_points = len(ranges)
            if num_points > 0:
                # LDS-01 / 시뮬레이션 라이다: 0도가 로봇 정면! (전방 ±20도 = 0°~20° 및 340°~360°)
                span20 = max(1, int(num_points * (20.0 / 360.0)))
                front_slice = list(ranges[:span20]) + list(ranges[-span20:])
                for r in front_slice:
                    if 0.05 < r < min_front_dist:
                        min_front_dist = r

                # 전방 ±60도(300°~60°) 넓은 시야에서 벽면 포인트(0.25m ~ 3.2m) 감지
                span60 = max(1, int(num_points * (60.0 / 360.0)))
                wall_slice = list(ranges[:span60]) + list(ranges[-span60:])
                valid_wall_pts = sum(1 for r in wall_slice if 0.25 < r < 3.2)
                if valid_wall_pts >= 15:
                    is_wall_detected = True

        if preserved_path and len(preserved_path) > 0:
            final_pt = preserved_path[-1]
            dist_to_final = math.hypot(final_pt['x'] - est_map_x, final_pt['y'] - est_map_y)
            goal_reached = (dist_to_final <= 0.60)
        elif preserved_goal is not None:
            goal_reached = (d_traveled >= max(self.target_distance - 0.4, 0.5))
        else:
            goal_reached = False

        if (is_wall_detected and d_traveled >= 1.5) or goal_reached:
            reason = "벽면 재발견" if is_wall_detected else "목표 거리 도달"
            self.get_logger().info(
                f'🎉 [Fallback 탈출 성공: {reason}] 이동거리: {d_traveled:.2f}m ➔ AMCL Reseeding 실행!')
            self.exit_fallback_and_reseed(est_map_x, est_map_y, est_map_yaw)
            return

        # 5) 주행 제어: Lookahead Pure Pursuit 경로 추종 (코너/곡선 완벽 추종)
        lookahead_dist = 0.9  # 전방 주시 거리 0.9m
        if preserved_path and len(preserved_path) > 0:
            # 1단계: 현재 추정 위치에서 가장 가까운 경로점 인덱스 탐색
            min_dist = 999.0
            closest_idx = 0
            for idx, pt in enumerate(preserved_path):
                dist = math.hypot(pt['x'] - est_map_x, pt['y'] - est_map_y)
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = idx

            # 2단계: closest_idx부터 전방 lookahead_dist 이상 떨어진 타겟 웨이포인트 탐색
            target_pt = preserved_path[-1]
            for pt in preserved_path[closest_idx:]:
                dist = math.hypot(pt['x'] - est_map_x, pt['y'] - est_map_y)
                if dist >= lookahead_dist:
                    target_pt = pt
                    break

            cur_target_yaw = math.atan2(target_pt['y'] - est_map_y, target_pt['x'] - est_map_x)
        elif preserved_goal is not None:
            gx = preserved_goal.pose.position.x
            gy = preserved_goal.pose.position.y
            cur_target_yaw = math.atan2(gy - est_map_y, gx - est_map_x)
        else:
            cur_target_yaw = self.target_global_yaw

        yaw_err = cur_target_yaw - est_map_yaw
        # 각도 정규화 (-pi ~ pi)
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        cmd = Twist()
        # 전방 0.25m 이내 장애물 감지 시 안전 일시 정지
        if min_front_dist < 0.25:
            self.get_logger().warn(f'⚠️ Fallback 중 전방 장애물 근접 ({min_front_dist:.2f}m) ➔ 일시 정지')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # 코너 구간(각도 오차 0.45rad/~25도 이상)에서는 감속 부드러운 코너링
            if abs(yaw_err) > 0.45:
                cmd.linear.x = 0.08  # 저속 코너 회전
                cmd.angular.z = max(min(yaw_err * 1.0, 0.40), -0.40)
            else:
                cmd.linear.x = 0.20  # 정상 궤적 안정 신속 주행
                cmd.angular.z = max(min(yaw_err * 0.8, 0.30), -0.30)

        self.cmd_pub.publish(cmd)

        # 급격한 조향 명령 발생 시 쿨다운 타이머 갱신 (벽면 상대각 오판 방지)
        if abs(cmd.angular.z) > 0.20:
            self.last_steer_time = time.time()

        # 1초에 한 번씩 Fallback 주행 상태 출력 (정확한 Wall-clock 기준)
        now = time.time()
        if now - self._last_fallback_log_time >= 1.0:
            self._last_fallback_log_time = now
            wall_info = f" | RANSAC벽면락: ON (innov:{self.last_wall_innov_deg:+.1f}°)" if self.parallel_wall_active else " | RANSAC벽면락: OFF"
            self.get_logger().info(
                f'🚀 [Fallback 주행 중] 이동: {d_traveled:.2f}m | 전방장애물: {min_front_dist:.2f}m | '
                f'yaw_err: {math.degrees(yaw_err):.1f}°{wall_info} | cmd: v={cmd.linear.x:.2f}m/s, w={cmd.angular.z:.2f}rad/s')

    def exit_fallback_and_reseed(self, est_x, est_y, est_yaw):
        self.fallback_active = False
        self.theta_wall_odom = None
        self.parallel_wall_active = False
        self.last_wall_innov_deg = 0.0
        self.reset_cmds()

        # AMCL /initialpose 메시지 작성 및 발행
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.pose.pose.position.x = est_x
        init_pose.pose.pose.position.y = est_y
        q = quaternion_from_yaw(est_yaw)
        init_pose.pose.pose.orientation.z = q[2]
        init_pose.pose.pose.orientation.w = q[3]

        # 적절한 분산값 (파티클이 이 위치 중심으로 모이도록 설정)
        init_pose.pose.covariance[0] = 0.25   # x 분산
        init_pose.pose.covariance[7] = 0.25   # y 분산
        init_pose.pose.covariance[35] = 0.06  # yaw 분산 (~14도)

        self.initial_pose_pub.publish(init_pose)
        self.get_logger().info(
            f'🎯 [AMCL Reseeding 발행 완료] map 좌표: ({est_x:.2f}, {est_y:.2f}, {math.degrees(est_yaw):.1f}°)\n'
            f'    ➔ 1.5초간 AMCL 파티클 수렴 대기 후 Nav2 자율주행 자동 재개')

        # 1.5초 후 Nav2 주행 자동 재개 타이머 (Wall-clock 기준 threading.Timer 사용)
        if self._reseed_timer is not None:
            self._reseed_timer.cancel()
        self._reseed_timer = threading.Timer(1.5, self._reseed_recovery_timer_cb)
        self._reseed_timer.daemon = True
        self._reseed_timer.start()

    def _reseed_recovery_timer_cb(self):
        self._reseed_timer = None

        if self.mode != 'manual':
            self.mode = 'auto'
            self.get_logger().info('>>> AMCL 수렴 완료 ➔ 보존된 목표(preserved_goal)로 Nav2 자율주행 재개!')
            self.resume_nav()

    # ===== 목적지 및 네비게이션 콜백 =====
    def goal_cb(self, msg: PoseStamped):
        self.last_goal = msg
        self._goal_locked = True
        
        # [기술보고서 v9] 수신된 목적지 좌표 영구 보존 및 토픽 발행
        self.preserved_goal = msg
        self.preserved_goal_pub.publish(self.preserved_goal)

        self.get_logger().info(
            f'🎯 [Goal 보존 완료] map 좌표: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f} '
            f'➔ /preserved_goal 토픽 발행')
        if self.mode != 'auto':
            self.mode = 'auto'
            self.publish_mode()
            self.get_logger().info('>>> 2D Goal Pose 수신으로 자율주행(auto) 모드 자동 전환')

    def plan_cb(self, msg: Path):
        if len(msg.poses) > 0:
            # 전체 경로를 웨이포인트 리스트로 상시 보존
            self.preserved_path = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]
            self.preserved_path_pub.publish(msg)

            goal_pose = PoseStamped()
            goal_pose.header = msg.header
            goal_pose.pose = msg.poses[-1].pose
            self.last_goal = goal_pose
            if self.preserved_goal is None:
                self.preserved_goal = goal_pose
                self.preserved_goal_pub.publish(self.preserved_goal)
                self.get_logger().info(
                    f'🎯 [Plan 경로 기반 Goal 자동 보존] x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}')

    def nav_cmd_callback(self, msg):
        self.latest_nav_cmd = msg
        self.last_nav_time = time.time()
        # auto 모드일 때 지연 없이 즉시 모터로 속도 명령 전달 (/clock 정지 시에도 동작 보장)
        if self.mode == 'auto':
            self.cmd_pub.publish(msg)

    def teleop_cmd_callback(self, msg):
        self.latest_teleop_cmd = msg
        self.last_teleop_time = time.time()

        # 사용자가 방향키를 눌러서 속도 값이 0이 아닌 경우
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            if self.mode in ('auto', 'fallback'):
                self.get_logger().warn('>>> 사용자의 수동 조작 감지! 현재 주행 모드를 해제하고 수동으로 전환합니다.')
                self.fallback_active = False
                self.set_mode('manual')
            elif self.mode == 'manual':
                self.cmd_pub.publish(msg)
                
    def sos_callback(self, msg):
        """SOS 신호 수신 → Fallback 진입 또는 즉시 정지"""
        source = msg.data if msg.data else 'unknown'

        # [핵심] 이미 Fallback 모드로 빈 공간을 돌파 중인 경우,
        # localization_monitor_node가 뒤늦게 쏜 SOS에 의해 모터가 꺼지지 않도록 주행 유지!
        if self.mode == 'fallback' or self.fallback_active:
            self.get_logger().info(f'🛡️ [SOS 감지: {source}] 이미 Fallback 돌파 주행 중이므로 모드 유지')
            return

        self.sos_count += 1
        self.get_logger().error(
            f'🚨🚨🚨 SOS #{self.sos_count} 발신: source="{source}" 🚨🚨🚨'
        )

        # 1) Nav2 작업 즉시 취소
        self.cancel_nav()

        # [기술보고서 v9 Phase 1~2] 위치 분실 SOS이고 기준 좌표가 있으면 Fallback 돌파 시도!
        if (self.mode == 'auto' or not self.fallback_active) and (self.preserved_goal is not None or self.preserved_path) and self.last_valid_map is not None:
            self.get_logger().warn(
                f'🛡️ [SOS 감지] 정지하지 않고 Fallback Controller로 빈 공간 통과 시도!')
            self.enter_fallback()
            return

        # 2) 그 외: 모드를 수동으로 강제 전환 (사용자가 직접 통제 / 즉시 정지)
        self.fallback_active = False
        self.mode = 'manual'
        self.reset_cmds()

        # [기술보고서 v9] Nav2가 취소되어도 보존된 목적지는 안전하게 유지됨을 확인
        if self.preserved_goal is not None:
            gp = self.preserved_goal.pose.position
            self.get_logger().info(
                f'🛡️ [Nav2 취소 완료] 보존된 목적지(preserved_goal) 안전 유지: x={gp.x:.2f}, y={gp.y:.2f}')
        # 3) SOS 이벤트를 파일에 기록 (사후 분석용)
        try:
            import os
            log_path = os.path.expanduser('~/wheelchair_sos.log')
            now = self.get_clock().now().to_msg()
            last_x = (f'{self.last_goal.pose.position.x:.3f}'
                      if self.last_goal else 'N/A')
            last_y = (f'{self.last_goal.pose.position.y:.3f}'
                      if self.last_goal else 'N/A')
            with open(log_path, 'a') as f:
                f.write(
                    f'{now.sec}.{now.nanosec:09d},'
                    f'source={source},'
                    f'last_goal_x={last_x},'
                    f'last_goal_y={last_y},'
                    f'count={self.sos_count}\n')
            self.get_logger().info(f'SOS 이벤트 기록: {log_path}')
        except Exception as e:
            self.get_logger().warn(f'SOS 로그 기록 실패: {e}')

    def mode_cmd_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'm':
            self.switch_mode()
        elif cmd == 'a':              
            self.set_mode('auto')
        elif cmd == 'home':
            self._send_home_goal()
        elif cmd in ('manual', 'auto', 'fallback'):
            self.set_mode(cmd)
        else:
            self.get_logger().warn(f'알 수 없는 모드 명령: {cmd}')
            
    def control_loop(self):
        if self.mode == 'auto':
            # Nav2나 safety_stop이 죽으면 /cmd_vel_safe가 끊긴다. 그때 마지막 속도를
            # 계속 내보내면 휠체어가 멈추지 않는다 — teleop와 같은 워치독을 건다.
            if time.time() - self.last_nav_time < self.cmd_timeout_sec:
                self.cmd_pub.publish(self.latest_nav_cmd)
            else:
                if self.latest_nav_cmd != Twist():
                    self.get_logger().error(
                        f'🚨 /cmd_vel_safe 끊김 {self.cmd_timeout_sec}s 초과 → 정지')
                    self.latest_nav_cmd = Twist()
                self.cmd_pub.publish(Twist())
        elif self.mode == 'fallback':
            self.fallback_control_tick()
        else:
            if time.time() - self.last_teleop_time < self.cmd_timeout_sec:
                self.cmd_pub.publish(self.latest_teleop_cmd)
            else:
                self.cmd_pub.publish(Twist())

    def publish_mode(self):
        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)

        # [기술보고서 v9] 보존된 목적지가 존재할 경우 1Hz로 상시 발행 유지
        if self.preserved_goal is not None:
            self.preserved_goal_pub.publish(self.preserved_goal)

    #def switch_mode(self):
    def reset_cmds(self):
        self.cmd_pub.publish(Twist())
        self.latest_nav_cmd = Twist()
        self.latest_teleop_cmd = Twist()

    def set_mode(self, new_mode):
        if new_mode not in ('manual', 'auto', 'fallback'):
            self.get_logger().warn(f'알 수 없는 모드: {new_mode}')
            return

        if self.mode == new_mode:
            self.get_logger().info(f'이미 {new_mode} 모드입니다.')
            return

        self.reset_cmds()

        if new_mode == 'auto':
            self.fallback_active = False
            self.theta_wall_odom = None
            self.parallel_wall_active = False
            self.last_wall_innov_deg = 0.0
            self.mode = 'auto'
            self.get_logger().info('>>> 자율주행 모드로 전환')
            self.resume_nav()
        elif new_mode == 'manual':
            self.fallback_active = False
            self.theta_wall_odom = None
            self.parallel_wall_active = False
            self.last_wall_innov_deg = 0.0
            self.cancel_nav()
            self.mode = 'manual'
            self.get_logger().info('>>> 수동 조작 모드로 전환')
        elif new_mode == 'fallback':
            self.mode = 'fallback'
            self.get_logger().info('>>> Fallback Dead-reckoning 주행 모드로 전환')


    def switch_mode(self):
        if self.mode == 'manual':
            self.set_mode('auto')
        else:
            self.set_mode('manual')

    def cancel_nav(self):
        try:
            if self._cancel_client.service_is_ready():
                request = CancelGoal.Request()
                self._cancel_client.call_async(request)
                self.get_logger().info('Nav2 goal 취소 완료')
            else:
                self.get_logger().warn('Nav2 cancel 서버 없음')
        except Exception as e:
            self.get_logger().warn(f'Nav2 취소 중 오류: {e}')
        self._goal_handle = None

    def resume_nav(self):
        target_goal = self.preserved_goal if self.preserved_goal is not None else self.last_goal
        if target_goal is None:
            self.get_logger().warn('저장된 목적지 없음 - RViz에서 목적지를 찍어주세요')
            return

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose 서버 연결 실패')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = target_goal.header.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = target_goal.pose

        self.get_logger().info(
            f'목적지 재전송 (보존된 목표 복원): x={target_goal.pose.position.x:.2f}, '
            f'y={target_goal.pose.position.y:.2f}')

        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal 거부됨!')
            return
        self.get_logger().info('Goal 수락됨 - 네비게이션 재개')
        self._goal_handle = goal_handle

    def key_listener(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key in ('m', 'M'):
                    self.switch_mode()
                elif key in ('h', 'H'):
                    self._send_home_goal()
                elif key in ('q', 'Q'):
                    self.get_logger().info('종료합니다.')
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitchNode()
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