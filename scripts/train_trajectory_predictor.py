#!/usr/bin/env python3
"""
train_trajectory_predictor.py
PyTorch 기반 GRU 동적 장애물 궤적 예측기 학습 및 벤치마크 평가 스크립트
- 최신 수집 데이터셋(.npz) 자동 로드
- 무작위 회전(Rotation Augmentation) 데이터 증강
- Train / Val 분할 및 CUDA GPU 가속 학습
- 에포크별 ADE(평균 변위 오차), FDE(종단 변위 오차) 평가
- 기존 등속(Constant Velocity, CV) 외삽 대비 정량적 개선율(%) 산출
- 최적 가중치(.pth) 및 설정 메타데이터 저장
"""
import os
import sys
import glob
import json
import math
import time
import argparse
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

# 모델 임포트
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from models.trajectory_gru import TrajectoryGRU


class TrajectoryDataset(Dataset):
    """궤적 데이터셋 및 실시간 회전 증강기"""
    def __init__(self, X: np.ndarray, Y: np.ndarray, augment: bool = True):
        self.X = torch.tensor(X, dtype=torch.float32)
        self.Y = torch.tensor(Y, dtype=torch.float32)
        self.augment = augment

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        x = self.X[idx].clone()  # (20, 2)
        y = self.Y[idx].clone()  # (20, 2)

        if self.augment:
            # 2D 무작위 회전 불변성 증강 (0 ~ 2pi)
            angle = torch.rand(1).item() * 2.0 * math.pi
            c = math.cos(angle)
            s = math.sin(angle)
            rot = torch.tensor([[c, -s], [s, c]], dtype=torch.float32)

            x = torch.matmul(x, rot)
            y = torch.matmul(y, rot)

        return x, y


def compute_metrics(y_true: torch.Tensor, y_pred: torch.Tensor):
    """
    y_true: (B, T, 2)
    y_pred: (B, T, 2)
    return: ADE (평균 변위 오차, m), FDE (종단 변위 오차, m)
    """
    diff = y_pred - y_true
    dist = torch.norm(diff, dim=-1)  # (B, T)
    ade = torch.mean(dist).item()
    fde = torch.mean(dist[:, -1]).item()
    return ade, fde


def evaluate_constant_velocity(X: torch.Tensor, Y: torch.Tensor):
    """
    Baseline: 과거 마지막 0.5초(5스텝)의 평균 속도로 미래 20스텝을 직진 등속 외삽
    """
    # 과거 최근 5스텝 (t=-5 ~ -1) 속도 벡터 추정
    # X[:, -1]은 [0, 0]이므로, (X[:, -1] - X[:, -5]) / (4 * 0.1s)
    dt = 0.1
    v_est = (X[:, -1] - X[:, -5]) / (4.0 * dt)  # (B, 2)

    # 미래 t=1~20 시점 외삽
    time_steps = torch.arange(1, 21, dtype=torch.float32).unsqueeze(0).unsqueeze(-1) * dt  # (1, 20, 1)
    v_expanded = v_est.unsqueeze(1)  # (B, 1, 2)
    y_cv_pred = v_expanded * time_steps  # (B, 20, 2)

    ade, fde = compute_metrics(Y, y_cv_pred)
    return ade, fde


def find_latest_dataset(dataset_dir: str) -> str:
    files = sorted(glob.glob(os.path.join(dataset_dir, "trajectory_dataset_*.npz")))
    if not files:
        raise FileNotFoundError(f"'{dataset_dir}'에 .npz 데이터셋 파일이 없습니다.")
    return files[-1]


def main():
    parser = argparse.ArgumentParser(description="GRU 동적 장애물 궤적 예측기 학습")
    parser.add_argument("--data", type=str, default="", help="데이터셋 .npz 경로 (생략 시 최신 파일)")
    parser.add_argument("--epochs", type=int, default=40, help="학습 에포크 수 (기본: 40)")
    parser.add_argument("--batch_size", type=int, default=64, help="배치 크기 (기본: 64)")
    parser.add_argument("--lr", type=float, default=1e-3, help="학습률 (기본: 0.001)")
    parser.add_argument("--hidden_size", type=int, default=48, help="GRU 은닉 유닛 수 (기본: 48)")
    parser.add_argument("--save_dir", type=str, default="/home/kim/wheelchair_gazebo_ws/models/trajectory_predictor")
    args = parser.parse_args()

    # 1. 데이터셋 로드
    ws_dir = "/home/kim/wheelchair_gazebo_ws"
    data_path = args.data if args.data else find_latest_dataset(os.path.join(ws_dir, "dataset"))
    print(f"📦 [1/5] 데이터셋 로드: {data_path}")

    raw_data = np.load(data_path)
    X_all = raw_data['X']
    Y_all = raw_data['Y']
    total_samples = len(X_all)
    print(f"   ▶ 총 샘플 수: {total_samples}개 시퀀스 | 입력 형상: {X_all.shape} | 정답 형상: {Y_all.shape}")

    # 2. Train / Val 분할 (80% : 20%)
    indices = np.arange(total_samples)
    np.random.seed(42)
    np.random.shuffle(indices)
    split_idx = int(total_samples * 0.8)

    train_idx = indices[:split_idx]
    val_idx = indices[split_idx:]

    train_ds = TrajectoryDataset(X_all[train_idx], Y_all[train_idx], augment=True)
    val_ds = TrajectoryDataset(X_all[val_idx], Y_all[val_idx], augment=False)

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, drop_last=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False)
    print(f"   ▶ 학습 세트: {len(train_ds)}개 | 검증 세트: {len(val_ds)}개")

    # 3. 디바이스 및 모델 초기화
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"🖥️ [2/5] 하드웨어 가속 디바이스: {device} (PyTorch {torch.__version__})")

    model = TrajectoryGRU(
        input_size=2,
        hidden_size=args.hidden_size,
        num_layers=2,
        future_steps=20,
        dropout=0.1
    ).to(device)

    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs, eta_min=1e-5)

    # 4. Baseline 등속(CV) 외삽 성능 사전 평가
    val_X_all = torch.tensor(X_all[val_idx], dtype=torch.float32)
    val_Y_all = torch.tensor(Y_all[val_idx], dtype=torch.float32)
    cv_ade, cv_fde = evaluate_constant_velocity(val_X_all, val_Y_all)
    print("=" * 68)
    print(f"  📊 [Baseline 벤치마크] 물리 등속(CV) 외삽 검증 세트 오차")
    print(f"     * ADE (2초 평균 변위 오차): {cv_ade:.4f} m")
    print(f"     * FDE (2초 종단 변위 오차): {cv_fde:.4f} m")
    print("=" * 68)

    # 5. 모델 학습 루프
    print(f"🚀 [3/5] GRU 궤적 예측기 학습 시작 (총 {args.epochs} 에포크)...")
    os.makedirs(args.save_dir, exist_ok=True)
    best_ade = float('inf')
    best_fde = float('inf')
    best_model_path = os.path.join(args.save_dir, "trajectory_gru_best.pth")

    start_time = time.time()
    history = []

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss = 0.0

        for bx, by in train_loader:
            bx, by = bx.to(device), by.to(device)
            optimizer.zero_grad()
            pred = model(bx)
            loss = criterion(pred, by)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=2.0)
            optimizer.step()
            train_loss += loss.item() * len(bx)

        train_loss /= len(train_ds)
        scheduler.step()

        # 검증 세트 평가
        model.eval()
        val_loss = 0.0
        all_true = []
        all_pred = []

        with torch.no_grad():
            for bx, by in val_loader:
                bx, by = bx.to(device), by.to(device)
                pred = model(bx)
                val_loss += criterion(pred, by).item() * len(bx)
                all_true.append(by.cpu())
                all_pred.append(pred.cpu())

        val_loss /= len(val_ds)
        all_true = torch.cat(all_true, dim=0)
        all_pred = torch.cat(all_pred, dim=0)
        val_ade, val_fde = compute_metrics(all_true, all_pred)

        is_best = val_ade < best_ade
        if is_best:
            best_ade = val_ade
            best_fde = val_fde
            torch.save(model.state_dict(), best_model_path)

        if epoch % 5 == 0 or epoch == 1 or epoch == args.epochs:
            star = " ⭐ (Best)" if is_best else ""
            print(f"   [Epoch {epoch:02d}/{args.epochs:02d}] "
                  f"Train Loss: {train_loss:.5f} | Val ADE: {val_ade:.4f}m | Val FDE: {val_fde:.4f}m{star}")

        history.append({
            'epoch': epoch,
            'train_loss': train_loss,
            'val_loss': val_loss,
            'val_ade': val_ade,
            'val_fde': val_fde
        })

    elapsed = time.time() - start_time
    print(f"⏱️ 학습 소요 시간: {elapsed:.1f}초")

    # 6. 최종 벤치마크 및 정량 분석
    print("\n" + "=" * 68)
    print("  🏆 [4/5] 최종 검증 성능 비교표 (Held-out Test)")
    print("=" * 68)
    ade_gain = (cv_ade - best_ade) / cv_ade * 100.0
    fde_gain = (cv_fde - best_fde) / cv_fde * 100.0
    print(f"{'방법론 (Method)':<24} | {'ADE (평균 변위, m)':<18} | {'FDE (종단 변위, m)':<18}")
    print("-" * 68)
    print(f"{'1) 등속 외삽 (Constant Velocity)':<20} | {cv_ade:<18.4f} | {cv_fde:<18.4f}")
    print(f"{'2) 2-Layer GRU (제안 모델)':<20} | {best_ade:<18.4f} | {best_fde:<18.4f}")
    print("-" * 68)
    print(f"🎉 GRU 성능 개선율: ADE {ade_gain:+.1f}% 개선 | FDE {fde_gain:+.1f}% 개선")
    print("=" * 68)

    # 7. 메타데이터 저장
    config_dict = {
        'model_name': 'TrajectoryGRU',
        'input_size': 2,
        'hidden_size': args.hidden_size,
        'num_layers': 2,
        'future_steps': 20,
        'best_ade_m': float(best_ade),
        'best_fde_m': float(best_fde),
        'cv_ade_m': float(cv_ade),
        'cv_fde_m': float(cv_fde),
        'ade_gain_pct': float(ade_gain),
        'fde_gain_pct': float(fde_gain),
        'trained_at': time.strftime("%Y-%m-%d %H:%M:%S")
    }
    cfg_file = os.path.join(args.save_dir, "model_config.json")
    with open(cfg_file, 'w', encoding='utf-8') as f:
        json.dump(config_dict, f, indent=2, ensure_ascii=False)

    print(f"\n💾 [5/5] 최적 가중치 저장 완료:")
    print(f"   ▶ 가중치 파일: {best_model_path}")
    print(f"   ▶ 설정 파일:   {cfg_file}")


if __name__ == '__main__':
    main()
