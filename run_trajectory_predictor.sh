#!/usr/bin/env bash
# run_trajectory_predictor.sh
# 실시간 GRU 동적 장애물 궤적 예측 ROS 2 노드 실행 스크립트

WS="/home/kim/wheelchair_gazebo_ws"

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

echo "========================================================================="
echo "  🧠 실시간 GRU 동적 장애물 궤적 예측 노드 시작"
echo "  ▶ 가중치: $WS/models/trajectory_predictor/trajectory_gru_best.pth"
echo "  ▶ 토픽:   /predicted_trajectories_markers (RViz 청록색 실선)"
echo "========================================================================="

python3 "$WS/scripts/trajectory_predictor_node.py"
