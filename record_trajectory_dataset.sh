#!/usr/bin/env bash
# record_trajectory_dataset.sh
# Gazebo 동적 장애물 주행 중 궤적 데이터셋 자동 수집 스크립트

WS=/home/kim/wheelchair_gazebo_ws
DURATION="${1:-60}"  # 기본 수집 시간 60초
NOISE="${2:-0.03}"    # 관측 가우시안 노이즈 표준편차 0.03m

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

echo "========================================================================="
echo "  🎬 동적 장애물 궤적 데이터 로거 시작"
echo "  ▶ 수집 지속 시간: ${DURATION}초"
echo "  ▶ 관측 노이즈 편차: ${NOISE}m"
echo "  ▶ 데이터셋 저장소: $WS/dataset"
echo "========================================================================="

python3 "$WS/scripts/trajectory_data_logger.py" --duration "$DURATION" --noise "$NOISE" --outdir "$WS/dataset"
