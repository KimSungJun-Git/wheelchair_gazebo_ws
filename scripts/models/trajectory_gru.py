#!/usr/bin/env python3
"""
trajectory_gru.py
Nav2 기술보고서 9.4.1절 규격 기반 2-Layer GRU 비선형 동적 장애물 궤적 예측 모델
- 입력: 과거 hist_steps(20스텝, 2.0s) 상대좌표 (B, 20, 2)
- 은닉층: 2-Layer GRU (hidden_size: 48)
- 출력: 미래 future_steps(20스텝, 2.0s) 상대좌표 (B, 20, 2)
- 추론 방식: 1-shot 순방향 추론 (< 1ms 지연 시간 보장)
"""
import torch
import torch.nn as nn


class TrajectoryGRU(nn.Module):
    def __init__(self, input_size: int = 2, hidden_size: int = 48, num_layers: int = 2,
                 future_steps: int = 20, dropout: float = 0.1):
        super(TrajectoryGRU, self).__init__()
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.future_steps = future_steps

        # 1. 시계열 인코더 (2-Layer GRU)
        self.encoder = nn.GRU(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0
        )

        # 2. 1-shot 고속 궤적 디코더 (MLP)
        self.decoder = nn.Sequential(
            nn.Linear(hidden_size, 128),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, future_steps * 2)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        x: (B, hist_steps=20, 2) - 과거 2초간 10Hz 상대좌표
        return: (B, future_steps=20, 2) - 미래 2초간 10Hz 상대좌표
        """
        batch_size = x.size(0)

        # GRU 인코딩: 마지막 은닉 상태 추출
        out, h_n = self.encoder(x)  # h_n: (num_layers, B, hidden_size)
        last_hidden = h_n[-1]       # 최상위 레이어의 마지막 은닉 상태: (B, hidden_size)

        # 1-shot 디코딩: 20스텝 동시 생성
        pred_flat = self.decoder(last_hidden)  # (B, 20 * 2)
        pred = pred_flat.view(batch_size, self.future_steps, 2)

        return pred
