#!/usr/bin/env python3
"""
trajectory_predictor_node.py
Nav2 기술보고서 9.4.1절 기반 실시간 GRU 동적 장애물 궤적 예측 ROS 2 노드
- 학습된 2-Layer GRU 모델 가중치(trajectory_gru_best.pth) 로드
- 최근 20개 관측치(2초간 10Hz) 시계열 버퍼링
- 10Hz 주기 배치(Batch) 순방향 추론 (지연 시간 < 2ms)
- RViz 시각화:
  * frame_id: "map" (Gazebo 월드 절대 좌표계)
  * stamp: 0 (타임스탬프 불일치 드롭 방지)
  * lifetime: 0.3초 (부드러운 실시간 갱신)
  * 청록색 형광 실선(GRU 비선형 예측) vs 주황색 점선(등속 외삽)
"""
import os
import sys
import time
import math
import collections
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Time as RosTime

# GRU 모델 아키텍처 임포트
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from models.trajectory_gru import TrajectoryGRU


class TrajectoryPredictorNode(Node):
    def __init__(self):
        super().__init__('trajectory_predictor_node')

        # 파라미터 선언
        self.declare_parameter('model_path', '/home/kim/wheelchair_gazebo_ws/models/trajectory_predictor/trajectory_gru_best.pth')
        self.declare_parameter('num_obstacles', 35)
        self.declare_parameter('history_steps', 20)
        self.declare_parameter('future_steps', 20)
        self.declare_parameter('pred_rate_hz', 10.0)
        self.declare_parameter('use_gpu', True)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.num_obstacles = self.get_parameter('num_obstacles').get_parameter_value().integer_value
        self.hist_len = self.get_parameter('history_steps').get_parameter_value().integer_value
        self.future_steps = self.get_parameter('future_steps').get_parameter_value().integer_value
        rate_hz = self.get_parameter('pred_rate_hz').get_parameter_value().double_value
        use_gpu = self.get_parameter('use_gpu').get_parameter_value().bool_value

        # 1. PyTorch 디바이스 및 모델 로드
        self.device = torch.device("cuda" if (use_gpu and torch.cuda.is_available()) else "cpu")
        self.get_logger().info(f"🧠 [GRU 예측기] 디바이스: {self.device} | 모델 경로: {model_path}")

        self.model = TrajectoryGRU(
            input_size=2,
            hidden_size=48,
            num_layers=2,
            future_steps=self.future_steps,
            dropout=0.0
        ).to(self.device)

        if os.path.exists(model_path):
            state_dict = torch.load(model_path, map_location=self.device)
            self.model.load_state_dict(state_dict)
            self.model.eval()
            self.get_logger().info(f"✅ 최적 학습 가중치 로드 완료: {model_path}")
        else:
            self.get_logger().warn(f"⚠️ 모델 가중치를 찾을 수 없습니다 ({model_path}). 초기화된 상태로 실행합니다.")

        # 2. 장애물별 시계열 버퍼 초기화 (최근 20개 (x, y))
        self.obs_buffers = {
            f"dynamic_obs_{i}": collections.deque(maxlen=self.hist_len)
            for i in range(1, self.num_obstacles + 1)
        }
        self.latest_poses = {}

        # 3. odom 구독기 생성
        self.odom_subs = []
        for i in range(1, self.num_obstacles + 1):
            obs_name = f"dynamic_obs_{i}"
            topic = f"/{obs_name}/odom"
            sub = self.create_subscription(
                Odometry,
                topic,
                lambda msg, name=obs_name: self.odom_callback(msg, name),
                10
            )
            self.odom_subs.append(sub)

        # 4. RViz MarkerArray 퍼블리셔 (RViz 호환을 위해 TRANSIENT_LOCAL QoS 적용)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/predicted_trajectories_markers', marker_qos)

        # 5. 주기적 추론 타이머 (10Hz)
        self.timer = self.create_timer(1.0 / rate_hz, self.inference_callback)
        self.get_logger().info(f"🚀 [GRU 궤적 예측기] {self.num_obstacles}개 장애물 실시간 추론 준비 완료 ({rate_hz}Hz)!")

    def odom_callback(self, msg: Odometry, name: str):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        self.obs_buffers[name].append((px, py))
        self.latest_poses[name] = (px, py)

    def inference_callback(self):
        t_start = time.perf_counter()

        # 추론 가능한 장애물 선별 (버퍼 20스텝 충족)
        active_names = []
        batch_inputs = []
        current_origins = []

        for name, buf in self.obs_buffers.items():
            if len(buf) >= 4:  # 최소 4스텝(0.4초)만 관측되면 즉시 예측 시작!
                active_names.append(name)
                pts = np.array(buf, dtype=np.float32)
                if len(pts) < self.hist_len:
                    # 초기 2초 미만인 경우 첫 번째 위치로 앞부분 패딩
                    pad_len = self.hist_len - len(pts)
                    first_pt = pts[0:1].repeat(pad_len, axis=0)
                    pts = np.vstack([first_pt, pts])

                curr_pt = pts[-1].copy()               # (x_t, y_t)
                rel_pts = pts - curr_pt                # 현재 위치 기준 상대좌표 (마지막 점 = [0, 0])

                batch_inputs.append(rel_pts)
                current_origins.append(curr_pt)

        if not batch_inputs:
            return

        # 배치 텐서 변환 및 1-shot 순방향 추론
        batch_x = torch.tensor(np.array(batch_inputs), dtype=torch.float32, device=self.device)
        with torch.no_grad():
            batch_pred = self.model(batch_x)  # (B, 20, 2)
        pred_numpy = batch_pred.cpu().numpy()

        dt = 0.1
        m_id = 1
        # stamp=0: RViz 시간 동기화 검사를 건너뛰고 항상 즉시 렌더링 (가장 안정적)
        stamp_zero = RosTime(sec=0, nanosec=0)
        marker_lifetime = Duration(seconds=0.35).to_msg()

        marker_array = MarkerArray()

        for idx, name in enumerate(active_names):
            orig_x, orig_y = current_origins[idx]
            rel_future = pred_numpy[idx]  # (20, 2)
            abs_future = rel_future + np.array([orig_x, orig_y])  # 절대 월드 좌표 복원

            # 장애물 번호 추출 (1 ~ 35) -> 고정 고유 ID 매핑으로 잔상 원천 차단
            try:
                obs_id = int(name.split('_')[-1])
            except Exception:
                obs_id = idx + 1

            # -------------------------------------------------------------
            # 1) GRU 신경망 예측 궤적 마커 (선명한 청록색 실선 라인)
            # -------------------------------------------------------------
            line_marker = Marker()
            line_marker.header.frame_id = "map"  # 월드 절대좌표 = map 프레임
            line_marker.header.stamp = stamp_zero
            line_marker.ns = "gru_trajectories"
            line_marker.id = obs_id  # 장애물 고유 고정 ID (1~35) -> 항상 덮어쓰기 보장
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.10  # 선 굵기 10cm로 선명하게 표시
            line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.85, a=1.0)  # 선명한 형광 청록색
            line_marker.lifetime = marker_lifetime

            # 현재 위치에서 시작
            p0 = Point(x=float(orig_x), y=float(orig_y), z=0.15)
            line_marker.points.append(p0)

            for step in range(len(abs_future)):
                fx, fy = abs_future[step]
                p = Point(x=float(fx), y=float(fy), z=0.15)
                line_marker.points.append(p)

            marker_array.markers.append(line_marker)

            # 2초 종단 예측점 강조 마커 (구체)
            end_marker = Marker()
            end_marker.header.frame_id = "map"
            end_marker.header.stamp = stamp_zero
            end_marker.ns = "gru_endpoints"
            end_marker.id = obs_id  # 장애물 고유 고정 ID (1~35)
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.pose.position.x = float(abs_future[-1, 0])
            end_marker.pose.position.y = float(abs_future[-1, 1])
            end_marker.pose.position.z = 0.20
            end_marker.scale.x = 0.32
            end_marker.scale.y = 0.32
            end_marker.scale.z = 0.32
            end_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=1.0)  # 밝은 에메랄드 그린
            end_marker.lifetime = marker_lifetime
            marker_array.markers.append(end_marker)

            # -------------------------------------------------------------
            # 2) 물리 등속(CV) 외삽 궤적 마커 (비교용, 주황색 점선)
            # -------------------------------------------------------------
            pts_hist = batch_inputs[idx]  # (20, 2)
            v_est = (pts_hist[-1] - pts_hist[-5]) / (4.0 * dt)  # 최근 0.4초 속도
            cv_line = Marker()
            cv_line.header.frame_id = "map"
            cv_line.header.stamp = stamp_zero
            cv_line.ns = "cv_extrapolations"
            cv_line.id = obs_id  # 장애물 고유 고정 ID (1~35)
            cv_line.type = Marker.LINE_STRIP
            cv_line.action = Marker.ADD
            cv_line.scale.x = 0.05
            cv_line.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.75)  # 주황색
            cv_line.lifetime = marker_lifetime

            cv_p0 = Point(x=float(orig_x), y=float(orig_y), z=0.08)
            cv_line.points.append(cv_p0)
            for s in range(1, self.future_steps + 1):
                cv_fx = orig_x + v_est[0] * (s * dt)
                cv_fy = orig_y + v_est[1] * (s * dt)
                cv_line.points.append(Point(x=float(cv_fx), y=float(cv_fy), z=0.08))
            marker_array.markers.append(cv_line)

        # 마커 발행
        self.marker_pub.publish(marker_array)

        infer_time_ms = (time.perf_counter() - t_start) * 1000.0
        # 3초에 한 번씩 추론 통계 출력
        if int(time.time() * 2) % 6 == 0:
            self.get_logger().info(
                f"⚡ [GRU 실시간 추론] 활성 장애물: {len(active_names)}/{self.num_obstacles}개 | "
                f"추론 지연: {infer_time_ms:.2f}ms (실시간 10Hz 기준 < 100ms)",
                throttle_duration_sec=3.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPredictorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
