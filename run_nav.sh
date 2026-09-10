#!/usr/bin/env bash
# 네비게이션 실행 — 원격 PC용. 라즈베리 센서 4종은 미리 띄워둘 것.
#   imu: stella_ahrs / lidar: ydlidar_ros2_driver / odom: stella_md / 초음파: uul.py
set -u

WS=~/wheelchair_ws
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# 터미널을 여러 개 띄운다. gnome-terminal이 없으면 tmux로 넘어간다.
run() {   # run <탭이름> <명령>
    local title="$1"; shift
    if command -v gnome-terminal >/dev/null; then
        gnome-terminal --tab --title="$title" -- bash -c \
            "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; $*; exec bash"
    else
        tmux new-window -n "$title" \
            "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; $*; exec bash"
    fi
}

if ! command -v gnome-terminal >/dev/null && ! command -v tmux >/dev/null; then
    echo "gnome-terminal도 tmux도 없다. 수동으로 띄워라." >&2
    exit 1
fi
if ! command -v gnome-terminal >/dev/null; then
    tmux has-session -t nav 2>/dev/null && { echo "nav 세션이 이미 있다: tmux attach -t nav"; exit 1; }
    tmux new-session -d -s nav
fi

run "1-bringup"   "ros2 launch wheelchair_robot_control bringup.launch.py"
sleep 3   # TF와 EKF가 먼저 올라와야 Nav2가 붙는다
run "2-nav2"      "ros2 launch wheelchair_robot_navigation2 navigation2.launch.py"
sleep 3
run "3-mode"      "ros2 run wheelchair_robot_control mode_switch_node"
# teleop은 수동 조작용이라 /cmd_vel_teleop으로 remap — 이걸 빼면 안전 필터를 건너뛴다
run "4-teleop"    "ros2 run wheelchair_robot_teleop teleop_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop"
run "5-imu"       "ros2 run wheelchair_robot_control imu_safety_node"
run "6-loc"       "ros2 run wheelchair_robot_control localization_monitor_node"
run "7-ui"        "ros2 launch wheelchair_robot_ui web_ui.launch.py"

command -v gnome-terminal >/dev/null || tmux attach -t nav

cat <<'MSG'

  사용자 화면: http://localhost:8000/Wheelchair_SLAM_UI.html
  관리자 화면: http://localhost:8000/admin/Dashboard.html

  확인: ros2 node list | grep ekf   → 1개만 나와야 정상
MSG
