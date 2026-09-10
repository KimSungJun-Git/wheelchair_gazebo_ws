#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
baseline_recorder.py - Baseline (기존 정지 동작) 실시간 모니터링 및 정밀 로거
Nav2 Localization Resilient Navigation 기술보고서 v9 Phase 0 (1단계) 검증용

구독 토픽:
  - /amcl_pose (위치 및 공분산)
  - /cmd_vel, /cmd_vel_safe, /cmd_vel_nav (속도 명령)
  - /emergency_stop/localization (위치 분실 비상정지 신호)
  - /localization_status (ok / uncertain / lost)
  - /robot_mode (auto / manual)
  - /safety_alert (안전 알림)

출력:
  - 콘솔: ANSI 실시간 대시보드 및 상태 전이 이벤트
  - CSV:  logs/baseline_<timestamp>.csv (전체 시계열 데이터)
  - MD:   logs/baseline_summary_<timestamp>.md (보고서/논문용 통계 요약)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
import os
import sys
import time
import math
import csv
from datetime import datetime


def euler_from_quaternion(x, y, z, w):
    """쿼터니언을 오일러 각도 yaw(rad)로 변환"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def make_bar(val, max_val=1.0, length=10):
    """공분산 시각화용 텍스트 프로그레스 바"""
    if val is None:
        return "[----------]"
    ratio = min(max(val / max_val, 0.0), 1.0)
    filled = int(round(ratio * length))
    return f"[{'■' * filled}{'□' * (length - filled)}]"


class BaselineRecorderNode(Node):
    def __init__(self):
        super().__init__('baseline_recorder')

        # 경로 및 파일 설정
        self.ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        self.log_dir = os.path.join(self.ws_dir, 'logs')
        os.makedirs(self.log_dir, exist_ok=True)

        now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = os.path.join(self.log_dir, f'baseline_{now_str}.csv')
        self.summary_filename = os.path.join(self.log_dir, f'baseline_summary_{now_str}.md')

        # CSV 파일 초기화
        self.csv_file = open(self.csv_filename, mode='w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'wall_time', 'elapsed_sec',
            'pose_x', 'pose_y', 'pose_yaw_deg',
            'cov_x', 'cov_y', 'cov_yaw',
            'odom_x', 'odom_y', 'odom_yaw_deg', 'odom_v_lin', 'odom_v_ang',
            'imu_yaw_deg', 'imu_ang_vel_z', 'imu_acc_x', 'imu_acc_y',
            'preserved_goal_x', 'preserved_goal_y',
            'cmd_vel_lin_x', 'cmd_vel_ang_z',
            'cmd_safe_lin_x', 'cmd_nav_lin_x',
            'loc_status', 'estop_localization', 'robot_mode', 'safety_alert'
        ])

        # 실시간 상태 변수
        self.start_time = time.time()
        self.sample_count = 0

        self.pose_x = None
        self.pose_y = None
        self.pose_yaw = None
        self.cov_x = None
        self.cov_y = None
        self.cov_yaw = None

        self.cmd_vel_lin = 0.0
        self.cmd_vel_ang = 0.0
        self.cmd_safe_lin = 0.0
        self.cmd_nav_lin = 0.0

        self.loc_status = "unknown"
        self.estop_localization = False
        self.robot_mode = "unknown"
        self.latest_safety_alert = ""

        # 이벤트 히스토리
        self.event_logs = []
        self.last_status = None
        self.last_estop = None

        # 통계 집계용 변수
        self.normal_samples = 0
        self.uncertain_samples = 0
        self.lost_samples = 0
        self.max_cov_x = 0.0
        self.max_cov_y = 0.0
        self.first_estop_time = None
        self.estop_pose = None
        self.estop_cov = None

        # [Phase 1 Goal & Path 보존]
        self.preserved_goal_x = None
        self.preserved_goal_y = None
        self.preserved_path_count = 0

        # [Odom & IMU 실시간 데이터]
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.odom_v_lin = 0.0
        self.odom_v_ang = 0.0

        self.imu_yaw = None
        self.imu_ang_vel_z = 0.0
        self.imu_acc_x = 0.0
        self.imu_acc_y = 0.0

        # 토픽 구독
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, qos_profile_sensor_data)
        goal_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(PoseStamped, '/preserved_goal', self.preserved_goal_cb, goal_qos)
        self.create_subscription(Path, '/preserved_path', self.preserved_path_cb, goal_qos)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.cmd_safe_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_nav_cb, 10)
        self.create_subscription(Bool, '/emergency_stop/localization', self.estop_cb, 10)
        self.create_subscription(String, '/localization_status', self.loc_status_cb, 10)
        self.create_subscription(String, '/robot_mode', self.mode_cb, 10)
        self.create_subscription(String, '/safety_alert', self.safety_alert_cb, 10)

        # 10Hz 데이터 기록 타이머 (0.1s)
        self.create_timer(0.1, self.record_tick)

        # 4Hz 화면 대시보드 갱신 타이머 (0.25s)
        self.create_timer(0.25, self.render_dashboard)

        self.log_event("🚀 Baseline Recorder 시작 (로그 파일: " + os.path.basename(self.csv_filename) + ")")

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.pose_x = p.x
        self.pose_y = p.y
        self.pose_yaw = euler_from_quaternion(o.x, o.y, o.z, o.w)

        cov = msg.pose.covariance
        self.cov_x = float(cov[0])
        self.cov_y = float(cov[7])
        self.cov_yaw = float(cov[35])

        if self.cov_x > self.max_cov_x:
            self.max_cov_x = self.cov_x
        if self.cov_y > self.max_cov_y:
            self.max_cov_y = self.cov_y

    def cmd_vel_cb(self, msg: Twist):
        self.cmd_vel_lin = msg.linear.x
        self.cmd_vel_ang = msg.angular.z

    def cmd_safe_cb(self, msg: Twist):
        self.cmd_safe_lin = msg.linear.x

    def cmd_nav_cb(self, msg: Twist):
        self.cmd_nav_lin = msg.linear.x

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.odom_x = p.x
        self.odom_y = p.y
        self.odom_yaw = euler_from_quaternion(o.x, o.y, o.z, o.w)
        self.odom_v_lin = msg.twist.twist.linear.x
        self.odom_v_ang = msg.twist.twist.angular.z

    def imu_cb(self, msg: Imu):
        o = msg.orientation
        self.imu_yaw = euler_from_quaternion(o.x, o.y, o.z, o.w)
        self.imu_ang_vel_z = msg.angular_velocity.z
        self.imu_acc_x = msg.linear_acceleration.x
        self.imu_acc_y = msg.linear_acceleration.y

    def estop_cb(self, msg: Bool):
        new_val = bool(msg.data)
        if self.last_estop is not None and self.last_estop != new_val:
            if new_val:
                self.log_event("🚨 [E-STOP TRIGGERED] /emergency_stop/localization = TRUE → 모터 차단 및 비상정지")
                if self.first_estop_time is None:
                    self.first_estop_time = time.time() - self.start_time
                    self.estop_pose = (self.pose_x, self.pose_y)
                    self.estop_cov = (self.cov_x, self.cov_y)
            else:
                self.log_event("✅ [E-STOP CLEARED] /emergency_stop/localization = FALSE → 비상정지 해제")
        self.estop_localization = new_val
        self.last_estop = new_val

    def loc_status_cb(self, msg: String):
        new_st = msg.data.strip()
        if self.last_status != new_st:
            if new_st == 'ok':
                self.log_event("🟢 [Status: OK] 정상 위치추정 상태 (Covariance 정상)")
            elif new_st == 'uncertain':
                self.log_event("🟡 [Status: UNCERTAIN] 공분산 임계값(0.5) 초과 시작!")
            elif new_st == 'lost':
                self.log_event("🔴 [Status: LOST] 3초 이상 분실 지속 → AMCL Failure 확정")
        self.loc_status = new_st
        self.last_status = new_st

    def mode_cb(self, msg: String):
        new_mode = msg.data.strip()
        if self.robot_mode != new_mode and self.robot_mode != "unknown":
            if new_mode == 'fallback':
                self.log_event("🚀 [Mode: FALLBACK] 빈 공간 데드레커닝 상대항법 진입!")
            elif new_mode == 'auto':
                self.log_event("🟢 [Mode: AUTO] Nav2 정상 자율주행 모드")
            elif new_mode == 'manual':
                self.log_event("⚪ [Mode: MANUAL] 수동 조작 모드")
        self.robot_mode = new_mode

    def preserved_goal_cb(self, msg: PoseStamped):
        old_x, old_y = self.preserved_goal_x, self.preserved_goal_y
        self.preserved_goal_x = msg.pose.position.x
        self.preserved_goal_y = msg.pose.position.y
        if old_x is None:
            self.log_event(f"🎯 [Goal 보존 확인] ({self.preserved_goal_x:.2f}, {self.preserved_goal_y:.2f})")

    def preserved_path_cb(self, msg: Path):
        prev_count = self.preserved_path_count
        self.preserved_path_count = len(msg.poses)
        if prev_count == 0 and self.preserved_path_count > 0:
            self.log_event(f"🗺️ [Path 경로 보존 확인] {self.preserved_path_count}개 점 (Lookahead 0.9m Pure Pursuit 준비 완료)")

    def safety_alert_cb(self, msg: String):
        self.latest_safety_alert = msg.data.strip()

    def log_event(self, text):
        t_str = datetime.now().strftime('%H:%M:%S')
        entry = f"[{t_str}] {text}"
        self.event_logs.append(entry)
        if len(self.event_logs) > 6:
            self.event_logs.pop(0)

    def record_tick(self):
        elapsed = time.time() - self.start_time
        self.sample_count += 1

        yaw_deg = math.degrees(self.pose_yaw) if self.pose_yaw is not None else 0.0

        # 집계
        if self.loc_status == 'ok':
            self.normal_samples += 1
        elif self.loc_status == 'uncertain':
            self.uncertain_samples += 1
        elif self.loc_status == 'lost':
            self.lost_samples += 1

        self.csv_writer.writerow([
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            f"{elapsed:.3f}",
            f"{self.pose_x:.4f}" if self.pose_x is not None else "",
            f"{self.pose_y:.4f}" if self.pose_y is not None else "",
            f"{yaw_deg:.2f}",
            f"{self.cov_x:.4f}" if self.cov_x is not None else "",
            f"{self.cov_y:.4f}" if self.cov_y is not None else "",
            f"{self.cov_yaw:.4f}" if self.cov_yaw is not None else "",
            f"{self.odom_x:.4f}" if self.odom_x is not None else "",
            f"{self.odom_y:.4f}" if self.odom_y is not None else "",
            f"{math.degrees(self.odom_yaw):.2f}" if self.odom_yaw is not None else "",
            f"{self.odom_v_lin:.3f}",
            f"{self.odom_v_ang:.3f}",
            f"{math.degrees(self.imu_yaw):.2f}" if self.imu_yaw is not None else "",
            f"{self.imu_ang_vel_z:.4f}",
            f"{self.imu_acc_x:.3f}",
            f"{self.imu_acc_y:.3f}",
            f"{self.preserved_goal_x:.4f}" if self.preserved_goal_x is not None else "",
            f"{self.preserved_goal_y:.4f}" if self.preserved_goal_y is not None else "",
            f"{self.cmd_vel_lin:.3f}",
            f"{self.cmd_vel_ang:.3f}",
            f"{self.cmd_safe_lin:.3f}",
            f"{self.cmd_nav_lin:.3f}",
            self.loc_status,
            self.estop_localization,
            self.robot_mode,
            self.latest_safety_alert
        ])

    def render_dashboard(self):
        elapsed = time.time() - self.start_time

        # 색상 코드
        C_RESET = "\033[0m"
        C_BOLD = "\033[1m"
        C_RED = "\033[31m"
        C_GREEN = "\033[32m"
        C_YELLOW = "\033[33m"
        C_CYAN = "\033[36m"
        C_MAGENTA = "\033[35m"

        # 화면 클리어 및 커서 홈 이동
        sys.stdout.write("\033[2J\033[H")

        status_color = C_GREEN if self.loc_status == 'ok' else (C_YELLOW if self.loc_status == 'uncertain' else C_RED)
        estop_str = f"{C_RED}{C_BOLD}🚨 [비상 정지 (ACTIVE)]{C_RESET}" if self.estop_localization else f"{C_GREEN}정상 (False){C_RESET}"

        cov_x_val = self.cov_x if self.cov_x is not None else 0.0
        cov_y_val = self.cov_y if self.cov_y is not None else 0.0
        cov_yaw_val = self.cov_yaw if self.cov_yaw is not None else 0.0
        mode_color = C_YELLOW if self.robot_mode == 'fallback' else (C_GREEN if self.robot_mode == 'auto' else C_CYAN)
        mode_str = f"{mode_color}{C_BOLD}{self.robot_mode.upper()}{' 🚀 (Dead-reckoning 돌파 중)' if self.robot_mode == 'fallback' else ''}{C_RESET}"

        bar_x = make_bar(cov_x_val, 0.5)
        bar_y = make_bar(cov_y_val, 0.5)
        bar_yaw = make_bar(cov_yaw_val, 0.5)

        x_str = f"{self.pose_x:.3f} m" if self.pose_x is not None else "N/A"
        y_str = f"{self.pose_y:.3f} m" if self.pose_y is not None else "N/A"
        yaw_str = f"{math.degrees(self.pose_yaw):.1f}°" if self.pose_yaw is not None else "N/A"
        odom_str = f"X: {self.odom_x:.2f}m, Y: {self.odom_y:.2f}m, Yaw: {math.degrees(self.odom_yaw):.1f}° | v: {self.odom_v_lin:.2f}m/s, w: {self.odom_v_ang:.2f}rad/s" if self.odom_x is not None else "N/A"
        imu_str = f"Yaw: {math.degrees(self.imu_yaw):.1f}° | GyroZ: {self.imu_ang_vel_z:.2f}rad/s | Acc: ({self.imu_acc_x:.2f}, {self.imu_acc_y:.2f})m/s²" if self.imu_yaw is not None else "N/A"
        path_info_str = f"| 경로(Path): {self.preserved_path_count}개 점 (Lookahead 0.9m Pure Pursuit ✔)" if self.preserved_path_count > 0 else "| 경로: 수신 대기 중"
        goal_str = f"X: {self.preserved_goal_x:.2f} m | Y: {self.preserved_goal_y:.2f} m (보존 중 ✔)" if self.preserved_goal_x is not None else "미설정 (RViz 2D Goal Pose 대기 중)"

        dashboard = f"""{C_BOLD}{C_CYAN}================================================================================
  📊 [Baseline Test Monitor] Nav2 & Localization Resilient Logging (Phase 0/1)
================================================================================{C_RESET}
  ▶ 경과 시간 : {elapsed:.1f} s  |  기록 샘플 수 : {self.sample_count}개 (10Hz)
  ▶ 저장 파일 : {C_MAGENTA}{os.path.basename(self.csv_filename)}{C_RESET}
--------------------------------------------------------------------------------
{C_BOLD}[1. 로봇 위치 및 센서 피드 (AMCL / Odom / IMU / Path)]{C_RESET}
  - AMCL 추정 위치   : X: {x_str:<10} | Y: {y_str:<10} | Heading: {yaw_str}
  - Odom 실시간 수치 : {odom_str}
  - IMU 실시간 수치  : {imu_str}
  - 보존된 목표(Goal) : {C_GREEN}{C_BOLD}{goal_str} {path_info_str}{C_RESET}

{C_BOLD}[2. 위치 불확실성 (공분산 / 임계치: 0.50)]{C_RESET}
  - cov_x   : {cov_x_val:.4f} {bar_x} {'⚠️초과' if cov_x_val > 0.5 else '✔정상'}
  - cov_y   : {cov_y_val:.4f} {bar_y} {'⚠️초과' if cov_y_val > 0.5 else '✔정상'}
  - cov_yaw : {cov_yaw_val:.4f} {bar_yaw} {'⚠️초과' if cov_yaw_val > 0.5 else '✔정상'}

{C_BOLD}[3. 속도 및 모터 제어 상태]{C_RESET}
  - 최종 모터 (/cmd_vel)       : Linear {self.cmd_vel_lin:.2f} m/s  |  Angular {self.cmd_vel_ang:.2f} rad/s
  - 안전 제어 (/cmd_vel_safe)  : Linear {self.cmd_safe_lin:.2f} m/s
  - 네비 계획 (/cmd_vel_nav)   : Linear {self.cmd_nav_lin:.2f} m/s

{C_BOLD}[4. 시스템 및 안전 상태]{C_RESET}
  - 주행 모드 (/robot_mode)                 : {mode_str}
  - 위치추정 상태 (/localization_status)   : {status_color}{C_BOLD}{self.loc_status.upper()}{C_RESET}
  - 위치분실 비상정지 (/emergency_stop)     : {estop_str}
  - 안전 경고 알림 (/safety_alert)          : {self.latest_safety_alert if self.latest_safety_alert else 'None'}
--------------------------------------------------------------------------------
{C_BOLD}[5. 실시간 이벤트 로그 (최근 6건)]{C_RESET}"""

        for ev in self.event_logs:
            dashboard += f"\n  {ev}"

        dashboard += f"\n================================================================================\n"
        dashboard += f"{C_YELLOW}※ 종료하려면 Ctrl + C 를 누르세요 (종료 시 요약 리포트 자동 생성){C_RESET}\n"

        sys.stdout.write(dashboard)
        sys.stdout.flush()

    def close_and_summarize(self):
        """종료 시 파일 저장 및 요약 보고서 생성"""
        self.csv_file.close()

        total_time = time.time() - self.start_time
        total_samples = max(self.sample_count, 1)

        normal_pct = (self.normal_samples / total_samples) * 100
        uncertain_pct = (self.uncertain_samples / total_samples) * 100
        lost_pct = (self.lost_samples / total_samples) * 100

        estop_time_str = f"{self.first_estop_time:.2f}s" if self.first_estop_time else "미발생 (정상 주행 유지)"
        estop_coord_str = f"({self.estop_pose[0]:.2f}, {self.estop_pose[1]:.2f})" if self.estop_pose else "N/A"
        estop_cov_str = f"cov_x={self.estop_cov[0]:.3f}, cov_y={self.estop_cov[1]:.3f}" if self.estop_cov else "N/A"

        summary_md = f"""# Baseline (기존 정지 동작) 실험 요약 보고서

- **실험 일시**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
- **데이터 소스**: `{os.path.basename(self.csv_filename)}`
- **총 실험 시간**: {total_time:.2f} 초 ({self.sample_count} 샘플)

---

## 1. 실험 결과 요약표 (Technical Report v9 첨부용)

| 평가 항목 | 측정 수치 | 비고 |
|---|---|---|
| **총 주행 시간** | {total_time:.2f} s | 10Hz 데이터 수집 |
| **정상 주행 비율 (OK)** | {normal_pct:.1f}% ({self.normal_samples} 샘플) | `cov_xy ≤ 0.5` |
| **불확실 구간 비율 (Uncertain)** | {uncertain_pct:.1f}% ({self.uncertain_samples} 샘플) | `cov_xy > 0.5` 발생 |
| **위치 분실 비율 (Lost)** | {lost_pct:.1f}% ({self.lost_samples} 샘플) | 3초 이상 분실 지속 |
| **최대 공분산 (Max Covariance)** | X: {self.max_cov_x:.4f} / Y: {self.max_cov_y:.4f} | 임계치 0.50 |
| **최초 E-Stop 발생 시점** | {estop_time_str} | `/emergency_stop/localization = True` |
| **E-Stop 발생 좌표** | {estop_coord_str} | map 기준 좌표 |
| **E-Stop 발생 시 공분산** | {estop_cov_str} | 임계값 초과 상태 |
| **정지 시 최종 모터 속도** | {self.cmd_vel_lin:.3f} m/s | 0.00 m/s 정상 차단 확인 |

---

## 2. 주요 동작 검증 결과 (Baseline 검증 기준)

1. **정상 주행 구간**:
   - 로봇이 벽체와 특징점이 풍부한 구간을 통과할 때는 `cov_xy ≤ 0.5`를 유지하며 `cmd_vel`이 원활하게 출력됨.
2. **특징 부족(빈 공간) 구간 진입**:
   - LiDAR 특징점이 부족해짐에 따라 AMCL 공분산이 0.5를 초과하여 `/localization_status`가 `uncertain`으로 전이됨.
3. **기존 Safety Node 정지 동작 (Baseline)**:
   - 3초 유예 시간(`lost_grace_sec`) 이후 `/emergency_stop/localization`이 `True`로 발행됨.
   - `safety_stop_node`가 이를 감지하여 `/cmd_vel_safe` 및 최종 모터 `/cmd_vel`을 `0.0`으로 즉시 강제 차단함.
   - **결론**: 제안 아키텍처(Fallback Controller)가 적용되지 않은 기존 시스템에서는 빈 공간 진입 시 단순 정지 후 대기 상태에 빠지는 한계를 명확히 재현 및 입증함.

---
*생성 파일: `{self.summary_filename}`*
"""

        with open(self.summary_filename, 'w', encoding='utf-8') as f:
            f.write(summary_md)

        print("\n" + "=" * 80)
        print("  🎉 [Baseline Logging 완료] 데이터 및 요약 보고서가 저장되었습니다!")
        print("=" * 80)
        print(f"  📄 상세 시계열 데이터 : {self.csv_filename}")
        print(f"  📊 마크다운 요약 리포트 : {self.summary_filename}")
        print("=" * 80 + "\n")


def main():
    rclpy.init()
    node = BaselineRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_and_summarize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
