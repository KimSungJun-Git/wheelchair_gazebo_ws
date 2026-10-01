#!/usr/bin/env python3
# mode_switch_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Empty
from action_msgs.srv import CancelGoal
from typing import Optional
import sys
import termios
import tty
import threading
import time
import math


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
        
        # 토픽 구독 및 발행
        self.dest_sub = self.create_subscription(String, '/destination', self.destination_callback, 10)
        self.home_sub = self.create_subscription(Empty, '/go_home', self.go_home_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.mode_cmd_sub = self.create_subscription(String, '/mode_switch', self.mode_cmd_callback, 10)
        self.safety_sub = self.create_subscription(String, '/safety_alert', self.safety_alert_callback, 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)

        # 속도 제어 토픽
        self.nav_cmd_sub = self.create_subscription(Twist, '/cmd_vel_safe', self.nav_cmd_callback, 10)
        self.teleop_cmd_sub = self.create_subscription(Twist, '/cmd_vel_teleop', self.teleop_cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Nav2 액션 및 서비스 클라이언트
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        # 모드 상태 변수
        self.declare_parameter('default_mode', 'manual')
        self.mode = self.get_parameter('default_mode').get_parameter_value().string_value
        self.last_goal: Optional[PoseStamped] = None
        self._goal_handle = None
        self._goal_locked = False

        # 목표 위치 및 경로 보존
        self.preserved_goal: Optional[PoseStamped] = None
        self.preserved_path: Optional[list] = None
        preserved_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.preserved_goal_pub = self.create_publisher(PoseStamped, '/preserved_goal', preserved_qos)
        self.preserved_path_pub = self.create_publisher(Path, '/preserved_path', preserved_qos)

        # 대기소(홈) 및 주요 위치 좌표
        self.destinations = {
            'home':       {'x': 1.815,  'y': 1.179,  'yaw': 0.0},
            'room_101':   {'x': 2.146,  'y': 0.003,  'yaw': 0.0},
            'room_102':   {'x': -0.338, 'y': -0.910, 'yaw': 0.0},
            'emergency':  {'x': -0.751, 'y': 0.272,  'yaw': 0.0},
        }
        self.home_pose = self.destinations['home']
        
        # 명령 타임아웃 관리
        self.latest_nav_cmd = Twist()
        self.latest_teleop_cmd = Twist()
        self.last_teleop_time = 0.0
        self.last_nav_time = 0.0
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.cmd_timeout_sec = self.get_parameter('cmd_timeout_sec').get_parameter_value().double_value

        # RANSAC 벽면 상대각 기반 헤딩 보정 파라미터
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

        # 평행벽 구속 런타임 상태
        self.theta_wall_odom = None
        self.last_steer_time = 0.0
        self.parallel_wall_active = False
        self.last_wall_innov_deg = 0.0

        # 주기 타이머
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.2, self.publish_mode)

        # 키보드 입력 쓰레드
        if sys.stdin.isatty():
            self.key_thread = threading.Thread(target=self.key_listener, daemon=True)
            self.key_thread.start()
        else:
            self.get_logger().info('stdin이 tty가 아님 - 키보드 입력 비활성화 (/mode_switch 토픽 사용)')
        
        # 비상 상황(SOS) 처리
        self.sos_sub = self.create_subscription(
            String, '/sos_trigger', self.sos_callback, 10)
        self.sos_count = 0
        
        self._current_destination = None

        self.get_logger().info(
            f'mode_switch_node 시작 (현재 모드: {self.mode}, '
            f'홈 좌표: x={self.home_pose["x"]}, y={self.home_pose["y"]})'
        )

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
            self.get_logger().info(f'{name} 이동을 위해 자율주행 모드로 전환')

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

        orientation_quat = quaternion_from_yaw(dest['yaw'])
        goal.pose.pose.orientation.z = orientation_quat[2]
        goal.pose.pose.orientation.w = orientation_quat[3]

        # 목적지 좌표 저장 및 발행
        self.last_goal = goal.pose
        self.preserved_goal = goal.pose
        self.preserved_goal_pub.publish(self.preserved_goal)

        self.get_logger().info(f'{name} 이동 목표 설정: x={dest["x"]:.2f}, y={dest["y"]:.2f}')

        # 현재 진행 중인 목적지 이름 저장
        self._current_destination = name

        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._dest_feedback_cb)
        send_future.add_done_callback(self._dest_response_cb)

    def _dest_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'{self._current_destination} 목표가 거부되었습니다.')
            return
        self.get_logger().info(f'{self._current_destination} 목표 수락 - 이동 시작')
        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dest_result_cb)

    def _dest_feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'{self._current_destination} 이동 중 (남은 거리: {remaining:.2f}m)')

    def _dest_result_cb(self, future):
        self._goal_locked = False
        self.get_logger().info(
            f'{self._current_destination} 도착 완료 - 수동 모드로 전환')
        self.mode = 'manual'
        self._goal_handle = None
        self._current_destination = None
        self.reset_cmds()

    # ===== 홈 귀환 =====
    def go_home_callback(self, msg):
        self.get_logger().info('홈 귀환 명령 수신')
        self._send_home_goal()

    def _send_home_goal(self):
        """대기소 좌표로 NavigateToPose 전송"""
        if self.mode != 'auto':
            self.reset_cmds()
            self.mode = 'auto'
            self.get_logger().info('홈 귀환을 위해 자율주행 모드로 전환')

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

        orientation_quat = quaternion_from_yaw(self.home_pose['yaw'])
        goal.pose.pose.orientation.z = orientation_quat[2]
        goal.pose.pose.orientation.w = orientation_quat[3]

        # 목적지 좌표 저장 및 발행
        self.last_goal = goal.pose
        self.preserved_goal = goal.pose
        self.preserved_goal_pub.publish(self.preserved_goal)

        self.get_logger().info(
            f'홈 좌표로 이동 목표 설정: x={self.home_pose["x"]}, y={self.home_pose["y"]}')

        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._home_feedback_cb)
        send_future.add_done_callback(self._home_response_cb)
    
    def _home_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('홈 귀환 목표가 거부되었습니다.')
            return
        self.get_logger().info('홈 귀환 목표 수락 - 이동 시작')
        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._home_result_cb)

    def _home_feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'홈 복귀 중 (남은 거리: {remaining:.2f}m)')

    def _home_result_cb(self, future):
        self._goal_locked = False
        self.get_logger().info('홈 도착 완료 - 수동 모드로 전환')
        self.mode = 'manual'
        self._goal_handle = None
        self.reset_cmds()

    # ===== 안전 알림 처리 =====
    def safety_alert_callback(self, msg):
        if msg.data == 'keepout_violation':
            self.get_logger().error('금지구역 진입 감지 - 주행 목표 취소 및 수동 모드 전환')
            self.set_mode('manual')
        elif msg.data == 'obstacle_too_close':
            if self.mode == 'auto':
                self.get_logger().warn('전방 장애물 감지 - 자율주행 모드 해제 및 수동 전환')
                self.set_mode('manual')
            else:
                self.get_logger().warn('전방 장애물 감지 (수동 모드)')
        elif msg.data == 'obstacle_cleared':
            self.get_logger().info('전방 장애물 해소')

    # ===== 목적지 및 네비게이션 콜백 =====
    def goal_cb(self, msg: PoseStamped):
        if not msg.header.frame_id:
            msg.header.frame_id = 'map'
        self.last_goal = msg
        self._goal_locked = True
        
        # 수신된 목적지 좌표 저장 및 토픽 발행
        self.preserved_goal = msg
        self.preserved_goal_pub.publish(self.preserved_goal)

        self.get_logger().info(
            f'목표 위치 수신 (map): x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        if self.mode != 'auto':
            self.mode = 'auto'
            self.publish_mode()
            self.get_logger().info('목표 위치 수신으로 자율주행 모드 전환')

    def plan_cb(self, msg: Path):
        if len(msg.poses) > 0:
            # 전체 경로를 웨이포인트 리스트로 보존
            self.preserved_path = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]
            self.preserved_path_pub.publish(msg)

            goal_pose = PoseStamped()
            goal_pose.header = msg.header
            if not goal_pose.header.frame_id:
                goal_pose.header.frame_id = 'map'
            goal_pose.pose = msg.poses[-1].pose
            self.last_goal = goal_pose

            # 목적지 갱신 판단
            has_goal_changed = False
            if self.preserved_goal is None:
                has_goal_changed = True
            else:
                prev_position = self.preserved_goal.pose.position
                current_position = goal_pose.pose.position
                position_distance = math.hypot(current_position.x - prev_position.x, current_position.y - prev_position.y)

                prev_orientation = self.preserved_goal.pose.orientation
                current_orientation = goal_pose.pose.orientation
                orientation_similarity = abs(
                    prev_orientation.x * current_orientation.x +
                    prev_orientation.y * current_orientation.y +
                    prev_orientation.z * current_orientation.z +
                    prev_orientation.w * current_orientation.w
                )

                if position_distance > 0.15 or orientation_similarity < 0.98:
                    has_goal_changed = True

            # 좌표 정보 갱신
            self.preserved_goal = goal_pose

            if has_goal_changed:
                self.preserved_goal_pub.publish(self.preserved_goal)
                self.get_logger().info(
                    f'목표 위치 갱신 (map): x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}')

    def nav_cmd_callback(self, msg):
        self.latest_nav_cmd = msg
        self.last_nav_time = time.time()
        if self.mode == 'auto':
            self.cmd_pub.publish(msg)

    def teleop_cmd_callback(self, msg):
        self.latest_teleop_cmd = msg
        self.last_teleop_time = time.time()

        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            if self.mode == 'auto':
                self.get_logger().warn('수동 조작 입력 감지 - 수동 모드로 전환')
                self.set_mode('manual')
            elif self.mode == 'manual':
                self.cmd_pub.publish(msg)

    def sos_callback(self, msg):
        """SOS 신호 수신 시 즉시 정지 및 수동 모드 전환"""
        source = msg.data if msg.data else 'unknown'

        self.sos_count += 1
        self.get_logger().error(
            f'SOS 신호 수신 (발신원: {source}, 누적: {self.sos_count})'
        )

        # 1) Nav2 작업 취소
        self.cancel_nav()

        # 2) 수동 모드 전환 및 속도 0 명령
        self.mode = 'manual'
        self.reset_cmds()

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
                    f'count={self.sos_count}' + chr(10))
            self.get_logger().info(f'SOS 이벤트 기록 완료: {log_path}')
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
        elif cmd in ('manual', 'auto'):
            self.set_mode(cmd)
        else:
            self.get_logger().warn(f'알 수 없는 모드 명령: {cmd}')

    def control_loop(self):
        if self.mode == 'auto':
            if time.time() - self.last_nav_time < self.cmd_timeout_sec:
                self.cmd_pub.publish(self.latest_nav_cmd)
            else:
                if self.latest_nav_cmd != Twist():
                    self.get_logger().error(
                        f'안전 속도 명령(/cmd_vel_safe) 수신 중단 ({self.cmd_timeout_sec}초 초과) - 정지')
                    self.latest_nav_cmd = Twist()
                self.cmd_pub.publish(Twist())
        else:
            if time.time() - self.last_teleop_time < self.cmd_timeout_sec:
                self.cmd_pub.publish(self.latest_teleop_cmd)
            else:
                self.cmd_pub.publish(Twist())

    def publish_mode(self):
        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)

        if self.preserved_goal is not None:
            self.preserved_goal_pub.publish(self.preserved_goal)

    def reset_cmds(self):
        self.cmd_pub.publish(Twist())
        self.latest_nav_cmd = Twist()
        self.latest_teleop_cmd = Twist()

    def set_mode(self, new_mode):
        if new_mode not in ('manual', 'auto'):
            self.get_logger().warn(f'알 수 없는 모드: {new_mode}')
            return

        if self.mode == new_mode:
            self.get_logger().info(f'이미 {new_mode} 모드입니다.')
            return

        self.reset_cmds()

        if new_mode == 'auto':
            self.mode = 'auto'
            self.get_logger().info('자율주행 모드로 전환')
            self.resume_nav()
        elif new_mode == 'manual':
            self.cancel_nav()
            self.mode = 'manual'
            self.get_logger().info('수동 조작 모드로 전환')

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
                self.get_logger().info('Nav2 주행 목표 취소 요청 전송')
            else:
                self.get_logger().warn('Nav2 cancel 서비스 이용 불가')
        except Exception as e:
            self.get_logger().warn(f'Nav2 취소 중 오류: {e}')
        self._goal_handle = None

    def resume_nav(self):
        target_goal = self.preserved_goal if self.preserved_goal is not None else self.last_goal
        if target_goal is None:
            self.get_logger().warn('저장된 목적지가 없습니다.')
            return

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose 액션 서버 연결 실패')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        frame_id = target_goal.header.frame_id if (hasattr(target_goal, 'header') and target_goal.header.frame_id) else 'map'
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = target_goal.pose

        self.get_logger().info(
            f'주행 목표 재전송: x={target_goal.pose.position.x:.2f}, '
            f'y={target_goal.pose.position.y:.2f} (좌표계: {frame_id})')

        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('주행 재개 목표가 거부되었습니다.')
            return
        self.get_logger().info('주행 재개 목표 수락 - 주행 시작')
        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._resume_result_cb)

    def _resume_result_cb(self, future):
        self._goal_locked = False
        self.get_logger().info('목적지 도착 완료 - 수동 모드로 전환')
        self.mode = 'manual'
        self._goal_handle = None
        self.reset_cmds()

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
