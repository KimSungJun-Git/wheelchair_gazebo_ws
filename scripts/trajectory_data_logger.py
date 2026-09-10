#!/usr/bin/env python3
"""
trajectory_data_logger.py
Gazebo 동적 장애물 궤적 데이터 로거 및 GRU 학습용 데이터셋 생성기
- 1단계: Ground Truth(/gazebo/model_states 또는 /<name>/odom) 실시간 동기 수집
- 2단계: 과거 20스텝(2.0s, 10Hz) 관측치 X + 미래 20스텝(2.0s) 정답 궤적 Y 추출
- 마지막 관측점(t=19) 기준 원점 상대 좌표 변환 자동 적용
- .npz 압축 포맷으로 에피소드 데이터셋 저장
"""
import os
import sys
import time
import math
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class TrajectoryDataLogger(Node):
    def __init__(self, output_dir: str, duration_sec: float = 120.0, noise_std: float = 0.03):
        super().__init__('trajectory_data_logger')
        self.output_dir = output_dir
        self.duration_sec = duration_sec
        self.noise_std = noise_std
        os.makedirs(self.output_dir, exist_ok=True)

        self.start_time = time.time()
        self.target_names = [f"dynamic_obs_{i}" for i in range(1, 36)]

        # 각 장애물별 시계열 버퍼: {name: [(t, x_gt, y_gt, x_obs, y_obs), ...]}
        self.history = {name: [] for name in self.target_names}
        self.latest_positions = {}
        # YAML에서 장애물별 시작 위치 로드
        self.yaml_path = "/home/kim/wheelchair_gazebo_ws/config/obstacles_Local_0904.yaml"
        self.start_positions = {}
        if os.path.exists(self.yaml_path):
            import yaml
            with open(self.yaml_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f)
                for d in cfg.get('dynamic_obstacles', []):
                    name = d['name']
                    wps = d.get('waypoints', [])
                    if wps:
                        self.start_positions[name] = (float(wps[0][0]), float(wps[0][1]))

        # 1. 각 장애물별 /<name>/odom 구독 (50Hz 고주파 Ground Truth 직접 수신)
        self.odom_subs = []
        for name in self.target_names:
            def make_cb(obs_name):
                return lambda msg: self.odom_cb(obs_name, msg)
            sub = self.create_subscription(Odometry, f"/{name}/odom", make_cb(name), 10)
            self.odom_subs.append(sub)

        # 2. 백업용 ModelStates 구독
        self.sub_model_states = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_cb,
            10
        )

        # 3. 10Hz (dt=0.1s) 정주기 데이터 레코딩 타이머
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.record_cb)

        self.total_samples_saved = 0
        self.get_logger().info(
            f"🎬 [TrajectoryDataLogger] 시작됨 | 대상: {len(self.target_names)}개 장애물 odom | "
            f"주기: 10Hz | 관측 노이즈: {self.noise_std}m | 저장 경로: {self.output_dir}"
        )

    def odom_cb(self, name: str, msg: Odometry):
        # planar_move의 odom은 Gazebo 월드 절대 좌표
        self.latest_positions[name] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def model_states_cb(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            if name in self.target_names:
                self.latest_positions[name] = (pose.position.x, pose.position.y)

    def record_cb(self):
        now = time.time()
        elapsed = now - self.start_time

        # 10개 장애물 현재 위치 샘플링 및 노이즈 주입
        for name in self.target_names:
            if name in self.latest_positions:
                gx, gy = self.latest_positions[name]
                # 라이다/트래커 관측 오차를 모사한 Gaussian Noise 추가
                ox = gx + np.random.normal(0.0, self.noise_std)
                oy = gy + np.random.normal(0.0, self.noise_std)
                self.history[name].append((now, gx, gy, ox, oy))

        # 진행 상황 출력 (10초마다)
        if int(elapsed) > 0 and int(elapsed) % 10 == 0 and len(self.history[self.target_names[0]]) % 100 == 0:
            pts = len(self.history[self.target_names[0]])
            self.get_logger().info(
                f"⏱️ [수집 중] 경과: {elapsed:.1f}/{self.duration_sec:.1f}s | "
                f"장애물당 포인트 수: {pts}개 (약 {pts//40}개 시퀀스 후보)"
            )

        # 수집 시간 도달 시 데이터셋 빌드 및 종료
        if elapsed >= self.duration_sec:
            self.get_logger().info("🏁 목표 수집 시간에 도달하여 데이터셋을 빌드 및 저장합니다...")
            self.build_and_save_dataset()
            rclpy.shutdown()

    def build_and_save_dataset(self):
        """
        슬라이딩 윈도우(윈도우 크기 40 = 과거 20스텝 + 미래 20스텝)로 데이터셋 생성:
        - X (과거 20스텝, 관측치): shape (N, 20, 2)
        - Y (미래 20스텝, Ground Truth): shape (N, 20, 2)
        - 원점 정규화: t=19 (입력 마지막 점)을 (0,0)으로 상대 변위화
        """
        X_list = []
        Y_list = []

        window_size = 40  # 과거 20 + 미래 20
        step_stride = 5   # 슬라이딩 보폭 5스텝 (0.5초)

        for name, records in self.history.items():
            if len(records) < window_size:
                continue

            for start_idx in range(0, len(records) - window_size + 1, step_stride):
                sub = records[start_idx : start_idx + window_size]
                
                # 과거 20스텝 관측치 (ox, oy)
                past_obs = np.array([[pt[3], pt[4]] for pt in sub[:20]], dtype=np.float32)
                # 미래 20스텝 정답 (gx, gy)
                future_gt = np.array([[pt[1], pt[2]] for pt in sub[20:]], dtype=np.float32)

                # 유의미한 이동 여부 확인 (전체 40스텝 동안 이동 거리가 0.15m 미만이면 제외)
                total_disp = np.linalg.norm(future_gt[-1] - past_obs[0])
                if total_disp < 0.15:
                    continue

                # 상대 좌표계 변환: 입력의 마지막 점 past_obs[-1]이 원점 (0,0)
                origin = past_obs[-1].copy()
                past_rel = past_obs - origin
                future_rel = future_gt - origin

                X_list.append(past_rel)
                Y_list.append(future_rel)

        if len(X_list) == 0:
            self.get_logger().warn("⚠️ 유효한 이동 시퀀스가 수집되지 않았습니다.")
            return

        X_arr = np.array(X_list, dtype=np.float32)  # shape: (N, 20, 2)
        Y_arr = np.array(Y_list, dtype=np.float32)  # shape: (N, 20, 2)

        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        out_filepath = os.path.join(self.output_dir, f"trajectory_dataset_{timestamp_str}.npz")
        np.savez_compressed(out_filepath, X=X_arr, Y=Y_arr)

        self.get_logger().info(
            f"🎉 [성공] 데이터셋 저장 완료!\n"
            f"   - 파일: {out_filepath}\n"
            f"   - 샘플 수: {len(X_arr)}개 시퀀스\n"
            f"   - 입력 X 형상: {X_arr.shape} (과거 20스텝 relative xy)\n"
            f"   - 정답 Y 형상: {Y_arr.shape} (미래 20스텝 relative xy)"
        )


def main():
    parser = argparse.ArgumentParser(description="동적 장애물 궤적 데이터 로거")
    parser.add_argument("--duration", type=float, default=60.0, help="데이터 수집 지속 시간 (초)")
    parser.add_argument("--noise", type=float, default=0.03, help="관측 가우시안 노이즈 표준편차 (m)")
    parser.add_argument("--outdir", type=str, default="/home/kim/wheelchair_gazebo_ws/dataset", help="저장 디렉토리")
    cli_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    logger_node = TrajectoryDataLogger(
        output_dir=cli_args.outdir,
        duration_sec=cli_args.duration,
        noise_std=cli_args.noise
    )

    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        logger_node.get_logger().info("사용자 중단: 수집된 데이터로 즉시 데이터셋을 빌드합니다.")
        logger_node.build_and_save_dataset()
    finally:
        logger_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
