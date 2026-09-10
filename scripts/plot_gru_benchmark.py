#!/usr/bin/env python3
"""
plot_gru_benchmark.py
GRU 동적 장애물 궤적 예측기 vs Baseline 등속(CV) 외삽 정량 벤치마크 플롯 생성
- ADE / FDE 막대 비교
- 미래 예측 시간(t=0.1~2.0s) 경과에 따른 오차 누적 곡선
- 대표 곡선 궤적 예측 시각화 비교 (Ground Truth vs GRU vs CV)
"""
import os
import sys
import glob
import math
import numpy as np
import matplotlib.pyplot as plt
import torch

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from models.trajectory_gru import TrajectoryGRU


def main():
    # 한글 및 스타일 설정
    plt.rcParams['font.family'] = 'DejaVu Sans'
    plt.rcParams['axes.unicode_minus'] = False

    ws_dir = "/home/kim/wheelchair_gazebo_ws"
    model_path = os.path.join(ws_dir, "models/trajectory_predictor/trajectory_gru_best.pth")
    data_files = sorted(glob.glob(os.path.join(ws_dir, "dataset/trajectory_dataset_*.npz")))
    if not data_files:
        print("데이터셋 파일이 없습니다.")
        return

    data = np.load(data_files[-1])
    X = data['X']  # (N, 20, 2)
    Y = data['Y']  # (N, 20, 2)
    N = len(X)

    # Held-out Test Set (마지막 20%)
    test_idx = int(N * 0.8)
    X_test = torch.tensor(X[test_idx:], dtype=torch.float32)
    Y_test = torch.tensor(Y[test_idx:], dtype=torch.float32)
    N_test = len(X_test)

    # 1. 모델 로드
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = TrajectoryGRU(input_size=2, hidden_size=48, num_layers=2, future_steps=20, dropout=0.0).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()

    # 2. GRU 예측 수행
    with torch.no_grad():
        pred_gru = model(X_test.to(device)).cpu()  # (N_test, 20, 2)

    # 3. CV 외삽 수행
    dt = 0.1
    v_est = (X_test[:, -1] - X_test[:, -5]) / (4.0 * dt)
    time_steps = torch.arange(1, 21, dtype=torch.float32).unsqueeze(0).unsqueeze(-1) * dt
    pred_cv = v_est.unsqueeze(1) * time_steps  # (N_test, 20, 2)

    # 4. 시간 스텝별 오차 곡선 (t=0.1s ~ 2.0s)
    err_gru = torch.norm(pred_gru - Y_test, dim=-1).numpy()  # (N_test, 20)
    err_cv = torch.norm(pred_cv - Y_test, dim=-1).numpy()    # (N_test, 20)

    mean_err_gru = np.mean(err_gru, axis=0)  # (20,)
    mean_err_cv = np.mean(err_cv, axis=0)    # (20,)

    ade_gru = float(np.mean(mean_err_gru))
    fde_gru = float(mean_err_gru[-1])
    ade_cv = float(np.mean(mean_err_cv))
    fde_cv = float(mean_err_cv[-1])

    # 5. 플롯 생성 (1x3 서브플롯)
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    times = np.arange(0.1, 2.05, 0.1)

    # [Subplot 1] 시간에 따른 오차 누적 곡선
    ax1 = axes[0]
    ax1.plot(times, mean_err_cv, 'o--', color='#d9534f', linewidth=2.2, label=f'Constant Velocity (ADE={ade_cv:.3f}m)')
    ax1.plot(times, mean_err_gru, 's-', color='#0275d8', linewidth=2.5, label=f'Proposed 2-Layer GRU (ADE={ade_gru:.3f}m)')
    ax1.set_xlabel('Prediction Horizon t (seconds)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Mean Displacement Error (meters)', fontsize=12, fontweight='bold')
    ax1.set_title('Displacement Error vs Horizon (t=0.1~2.0s)', fontsize=13, fontweight='bold')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='upper left', fontsize=10)

    # [Subplot 2] ADE 및 FDE 막대 비교
    ax2 = axes[1]
    labels = ['ADE (Avg Error)', 'FDE (Final Error at 2s)']
    cv_bars = [ade_cv, fde_cv]
    gru_bars = [ade_gru, fde_gru]
    x = np.arange(len(labels))
    width = 0.32

    rects1 = ax2.bar(x - width/2, cv_bars, width, label='Constant Velocity', color='#e06666', edgecolor='black')
    rects2 = ax2.bar(x + width/2, gru_bars, width, label='Proposed 2-Layer GRU', color='#45818e', edgecolor='black')

    ax2.set_ylabel('Error (meters)', fontsize=12, fontweight='bold')
    ax2.set_title('ADE & FDE Quantitative Benchmark', fontsize=13, fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, fontsize=11, fontweight='bold')
    ax2.grid(True, axis='y', linestyle='--', alpha=0.6)
    ax2.legend(loc='upper left', fontsize=10)

    # 막대 위 수치 라벨링
    for r in rects1:
        h = r.get_height()
        ax2.annotate(f'{h:.3f}m', xy=(r.get_x() + r.get_width() / 2, h), xytext=(0, 3),
                     textcoords="offset points", ha='center', va='bottom', fontsize=10, fontweight='bold')
    for r in rects2:
        h = r.get_height()
        ax2.annotate(f'{h:.3f}m', xy=(r.get_x() + r.get_width() / 2, h), xytext=(0, 3),
                     textcoords="offset points", ha='center', va='bottom', fontsize=10, fontweight='bold')

    # [Subplot 3] 대표 시나리오 궤적 형상 비교 (Turning / Non-linear behavior)
    ax3 = axes[2]
    # 곡선/방향전환 성분이 큰 샘플 탐색 (FDE 차이가 큰 샘플)
    diff_fde = err_cv[:, -1] - err_gru[:, -1]
    best_sample_idx = int(np.argmax(diff_fde))

    hist = X_test[best_sample_idx].numpy()
    gt = Y_test[best_sample_idx].numpy()
    pred_g = pred_gru[best_sample_idx].numpy()
    pred_c = pred_cv[best_sample_idx].numpy()

    ax3.plot(hist[:, 0], hist[:, 1], 'k.--', markersize=6, alpha=0.5, label='Past 2s History')
    ax3.plot(0, 0, 'ko', markersize=8, label='Current (t=0)')
    ax3.plot(gt[:, 0], gt[:, 1], 'g.-', linewidth=2.5, markersize=7, label='Ground Truth (Actual)')
    ax3.plot(pred_g[:, 0], pred_g[:, 1], 'c.-', linewidth=2.2, markersize=6, label='GRU Prediction')
    ax3.plot(pred_c[:, 0], pred_c[:, 1], 'r.--', linewidth=1.8, markersize=5, label='CV Extrapolation')

    ax3.set_xlabel('Relative X (m)', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Relative Y (m)', fontsize=12, fontweight='bold')
    ax3.set_title('Trajectory Comparison: Non-linear Turning', fontsize=13, fontweight='bold')
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(loc='best', fontsize=9)
    ax3.axis('equal')

    plt.tight_layout()

    out_dir = os.path.join(ws_dir, "evaluation")
    os.makedirs(out_dir, exist_ok=True)
    out_file = os.path.join(out_dir, "fig5_gru_trajectory_prediction_benchmark.png")
    plt.savefig(out_file, dpi=200)
    print(f"📊 벤치마크 플롯 저장 완료: {out_file}")

    # 아티팩트 디렉터리에도 복사
    art_dir = "/home/kim/.gemini/antigravity-ide/brain/44d35ccb-80a4-4b23-b4d7-cfafc05ceb46/figures"
    os.makedirs(art_dir, exist_ok=True)
    art_file = os.path.join(art_dir, "fig5_gru_trajectory_prediction_benchmark.png")
    plt.savefig(art_file, dpi=200)
    print(f"🖼️ 아티팩트 플롯 복사 완료: {art_file}")


if __name__ == '__main__':
    main()
