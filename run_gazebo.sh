#!/usr/bin/env bash
# run_gazebo.sh - ROS 2 연동 Gazebo 시뮬레이션 실행 스크립트
set -e

WS="/home/kim/wheelchair_gazebo_ws"
WORLD="${1:-$WS/world/Local_0904.world}"

# 기존 남아있는 가제보 프로세스 정리
killall -9 gzserver gzclient 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH="$WS/models:$GAZEBO_MODEL_PATH"

echo "========================================================================="
echo "  🚀 Gazebo ROS 2 시뮬레이션 시작 (Local_0904 월드)"
echo "  ▶ 월드 파일: $WORLD"
echo "========================================================================="

ros2 launch gazebo_ros gazebo.launch.py world:="$WORLD"
