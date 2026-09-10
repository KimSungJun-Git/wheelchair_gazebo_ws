#!/usr/bin/env bash
# run_gazebo_obstacle.sh
# Gazebo 정적/동적 장애물 자동 스폰 및 웨이포인트 폐루프 제어 스크립트

WS=/home/kim/wheelchair_gazebo_ws
CONFIG_FILE="${1:-$WS/config/obstacles_Local_0904.yaml}"

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

echo "========================================================================="
echo "  🚀 Gazebo 장애물 매니저 시작"
echo "  ▶ 설정 파일: $CONFIG_FILE"
echo "========================================================================="

python3 "$WS/scripts/waypoint_obstacle_manager.py" "$CONFIG_FILE"
