#!/usr/bin/env bash
# 맵핑 실행 — 원격 PC용. 라즈베리 센서 4종은 미리 띄워둘 것.
# cartographer.launch.py가 TF+EKF(bringup)를 포함하므로 bringup을 따로 켜지 않는다.
set -u

WS=~/wheelchair_ws
MAP_DIR="$WS/src/wheelchair_robot/wheelchair_robot_navigation2/map"
MAP_NAME="${1:-wheelchair_robot_world}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

run() {
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
    tmux has-session -t mapping 2>/dev/null && { echo "mapping 세션이 이미 있다: tmux attach -t mapping"; exit 1; }
    tmux new-session -d -s mapping
fi

run "1-cartographer" "ros2 launch wheelchair_robot_cartographer cartographer.launch.py"
sleep 4
# 맵핑 teleop은 remap 없이 /cmd_vel 직접 — safety_stop_node를 거치지 않는다
run "2-teleop"       "ros2 run wheelchair_robot_teleop teleop_keyboard"

command -v gnome-terminal >/dev/null || tmux attach -t mapping

cat <<MSG

  맵을 다 그렸으면 저장:
    ros2 run nav2_map_server map_saver_cli -f $MAP_DIR/$MAP_NAME

  ⚠️ 맵핑 중에는 safety_stop_node가 없다 — 장애물 정지가 동작하지 않으니 직접 피해라.
MSG
