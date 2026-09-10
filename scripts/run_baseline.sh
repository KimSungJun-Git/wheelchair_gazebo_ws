#!/usr/bin/env bash
# run_baseline.sh - Baseline 정지 동작 실험 및 실시간 로거 원클릭 실행 스크립트
set -e

WS="/home/kim/wheelchair_gazebo_ws"

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

echo "========================================================================="
echo "  🚀 [Phase 0] Baseline 정지 동작 실험 & 데이터 수집 환경 준비"
echo "========================================================================="

# 1. localization_monitor_node 실행 여부 확인 및 자동 기동
MONITOR_PID=""
if ! ros2 node list 2>/dev/null | grep -q "/localization_monitor"; then
    echo "  ▶ [1/2] localization_monitor_node 시작 중..."
    ros2 run wheelchair_robot_control localization_monitor_node > /dev/null 2>&1 &
    MONITOR_PID=$!
    echo "  ✔ localization_monitor_node 기동 완료 (PID: $MONITOR_PID)"
    sleep 1
else
    echo "  ✔ [1/2] localization_monitor_node 가 이미 실행 중입니다."
fi

# 스크립트 종료 시 서브프로세스 정리
cleanup() {
    if [ -n "$MONITOR_PID" ]; then
        echo "  🧹 백그라운드 localization_monitor_node 정리 중..."
        kill "$MONITOR_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "  ▶ [2/2] Baseline 실시간 대시보드 및 로거 시작..."
echo "========================================================================="

# 2. baseline_recorder.py 실행
python3 "$WS/scripts/baseline_recorder.py" "$@"
