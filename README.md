# Wheelchair Robot

ROS 2 Humble에서 동작하는 실내 자율주행 전동 휠체어 프로젝트입니다.  
탑승자는 터치 화면에서 목적지(병실, 응급실, 대기실 등)를 고르거나 수동 조작과 SOS 호출을 직접 실행합니다. 휠체어는 2D LiDAR, IMU, 휠 오도메트리, 3방향 초음파 센서를 확장 칼만 필터(EKF)와 Nav2에 묶어 장애물을 피하며 주행하고, 보호자와 관리자는 관제 대시보드에서 현재 위치와 안전 상태를 실시간으로 확인합니다.

## 주행 시연 영상
- [실내 자율주행 휠체어 주행 영상 보기 (Google Drive)](https://docs.google.com/videos/d/1K7URg8u6nNYTwtFxxSr6Geb8zvsFcgvLhU9YpToEdCE/edit?usp=drive_link)

---

## 주요 기능

### 센서 융합 측위와 SLAM
`imu_relay_node`가 AHRS 센서의 빈 프레임(`imu_link`)을 채워 `/imu/data_fixed`로 넘기면, `robot_localization`(`ekf_filter_node`)이 휠 오도메트리(`/odom`)와 IMU 데이터를 합쳐 `/odometry/filtered`와 `odom → base_footprint` TF를 내보냅니다. 바퀴가 미끄러져도 자세 추정치가 크게 흔들리지 않으며, 실내 2D 격자 지도(`world.pgm`, `world.yaml`)는 Cartographer로 그립니다.

### 동적 장애물 분리와 주행 복구
`dynamic_scan_filter`는 2D LiDAR 스캔(`/scan`)을 등속 칼만 필터로 추적해 최근접 접근점(CPA)을 계산합니다. 다가오는 보행자처럼 부딪힐 위험이 있는 빔은 `/scan_threat`로, 벽이나 기둥 같은 고정 장애물 빔은 `/scan_static`으로 나누어 발행합니다.

Nav2 행동 트리(`navigate_w_replanning_and_recovery.xml`)에는 C++ 조건 플러그인 `IsThreatPresent`(`is_threat_present_condition_bt_node.cpp`)를 붙였습니다. 전방에 동적 위협(`/scan_threat`)이 있을 때는 무리하게 경로를 틀거나 후진하지 않고 제자리에서 2초간 멈춰 기다리고(`Wait`), 정적 장애물에 갇혔을 때만 로컬 코스트맵을 비운 뒤 20cm 후진(`BackUp`)합니다. 계단이나 출입 제한 구역은 Keepout Filter(`keepout_mask.pgm`, `keepout_mask.yaml`)로 막아 경로 계획에서 제외합니다.

### 다중 안전 감시와 제어권 전환
주행 중 안전 제어는 네 노드가 나누어 맡습니다.
- `safety_stop_node`: 전방·좌·우 초음파 거리와 `/scan_threat`, 원본 라이다 근접 거리를 감시해 위험 시 정지하거나 감속합니다. 양옆 여유 공간을 비교해 회피 가능 방향(`/avoidance_direction`)과 센서 통신 상태(`/sensor_health`)도 함께 알립니다.
- `imu_safety_node`: 차체가 기준 각도 이상 기울어지거나 충격을 받으면 `/emergency_stop/imu`를 발행합니다.
- `localization_monitor_node`: AMCL 위치 공분산(`cov_x`, `cov_y`, `cov_yaw`)과 토픽 수신 주기를 감시합니다. 위치를 잃어버리면 `/emergency_stop/localization`으로 휠체어를 세우고 글로벌 재측위 서비스를 호출합니다.
- `mode_switch_node`: 자율주행 안전 속도(`/cmd_vel_safe`)와 수동 조작 속도(`/cmd_vel_teleop`) 중 현재 모드(`auto` / `manual`)에 맞는 쪽만 골라 모터 드라이버(`/cmd_vel`)로 내보냅니다. 수동 조작이나 SOS가 들어오면 진행 중인 Nav2 목표를 즉시 취소합니다.

### 탑승자 웹 UI와 관제 대시보드
`web_ui.launch.py` 하나로 `rosbridge_websocket`(9090 포트), 정적 웹 서버(8000 포트), FastAPI 관제 서버(`admin_server.py`, 8090 포트)가 함께 뜹니다.
- 탑승자 화면 (`Wheelchair_SLAM_UI.html`): 실시간 SLAM 지도와 이동 경로, 5방향 장애물 거리를 보여줍니다. 목적지 호출, 수동/자율 전환, SOS 호출과 한/영 전환을 지원합니다.
- 관제 대시보드 (`admin/Dashboard.html`): 시간대·원인별 안전 이벤트 통계와 SLAM 좌표 히트맵을 집계하고, 실시간 위치 추적과 원격 긴급 정지 기능을 제공합니다.

---

## 패키지 구조

```text
wheelchair_gazebo_ws/
└── src/wheelchair_robot/
    ├── wheelchair_description/         # 로봇 URDF(robot.urdf), 3D 메쉬(robot_n2.stl), RViz 설정
    ├── wheelchair_cartographer/        # Cartographer 2D SLAM 설정(cartographer.lua) 및 런치
    ├── wheelchair_navigation2/         # Nav2 파라미터(navigation2.yaml), 맵(world.pgm), Keepout 마스크(keepout_mask.pgm),
    │                                   # 커스텀 BT XML 및 C++ 조건 노드(is_threat_present_condition_bt_node.cpp)
    ├── wheelchair_control/             # 제어 및 안전 패키지
    │   ├── config/                     # ekf.yaml, safety.yaml
    │   ├── launch/                     # bringup.launch.py (TF + IMU 릴레이 + EKF), safety.launch.py
    │   └── wheelchair_control/         # dynamic_scan_filter, safety_stop_node, mode_switch_node,
    │                                   # imu_safety_node, localization_monitor_node, imu_relay_node
    ├── wheelchair_teleop/              # 키보드 수동 조작 노드 (teleop_keyboard.py)
    └── wheelchair_ui/                  # 탑승자 웹 UI + 관제 대시보드 + FastAPI 서버(admin_server.py)
```

---

## 데이터 흐름

```text
[라즈베리파이 센서 계층]
  ├─ stella_ahrs (/imu/data) ──► [imu_relay_node] ──► /imu/data_fixed ─┐
  ├─ stella_md   (/odom)     ──────────────────────────────────────────┼─► [ekf_filter_node] ──► /odometry/filtered (TF: odom→base_footprint)
  ├─ ydlidar     (/scan)     ──► [dynamic_scan_filter] ──► /scan_static (Nav2 코스트맵) & /scan_threat (동적 위협)
  └─ uul.py      (/ultrasonic/range, /left, /right)

[자율주행 및 안전 제어 계층]
  [Nav2 (BT + Keepout Filter)]
        │  (/cmd_vel_nav)
        ▼
  [safety_stop_node] ◄── /scan, /scan_threat, /ultrasonic/*, /emergency_stop/{imu,localization}
        │  (/cmd_vel_safe, /safety_action, /safety_alert, /sensor_health)
        ▼
  [mode_switch_node] ◄── /cmd_vel_teleop (수동 조작), /destination, /mode_switch
        │
        ▼
    /cmd_vel (모터 드라이버 구동)

[웹 UI 및 관제 계층]
  /safety_action, /sos_trigger, /amcl_pose
        ├──► [admin_server (FastAPI :8090)] ◄──► 관제 대시보드 (http://localhost:8000/admin/Dashboard.html)
        └──► [rosbridge_websocket (:9090)]  ◄──► 탑승자 웹 UI  (http://localhost:8000/Wheelchair_SLAM_UI.html)
```

---

## 빌드 및 실행 방법

### 워크스페이스 빌드
```bash
pip3 install fastapi uvicorn
colcon build --symlink-install
source install/setup.bash
```

### 라즈베리파이 (센서 구동 및 네트워크 설정)
```bash
# imu
ros2 launch stella_ahrs stella_ahrs_launch.py

# lidar
ros2 launch ydlidar_ros2_driver ydlidar_launch.py

# odom
ros2 launch stella_md stella_md_launch.py

# ultrasonic
cd ~/colcon_ws/src/
python3 uul.py
```

와이파이 절전모드로 인한 토픽 지연을 막기 위해 절전모드를 끕니다.
```bash
# 절전모드 확인 (라즈베리파이)
iwconfig wlan0 | grep "Power Management"
iw dev wlan0 get power_save

# 절전모드 off (라즈베리파이)
sudo iwconfig wlan0 power off
sudo iw dev wlan0 set power_save off

# 원격 PC 절전모드 off 및 FastRTPS 설정 초기화
sudo iwconfig wlo1 power off
unset FASTRTPS_DEFAULT_PROFILES_FILE
```

### 원격 PC (네비게이션 실행)
```bash
# 터미널 1: TF + EKF 및 안전 필터(동적 스캔 필터 + 안전 정지)
ros2 launch wheelchair_control bringup.launch.py
ros2 launch wheelchair_control safety.launch.py

# 터미널 2: Nav2 네비게이션
ros2 launch wheelchair_navigation2 navigation2.launch.py

# 터미널 3: 모드 전환 (키보드 m/h/q 제어 — 단독 실행)
ros2 run wheelchair_control mode_switch_node

# 터미널 4: 조이스틱/키보드
ros2 run wheelchair_teleop teleop_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop

# 터미널 5: 휠체어 기울기 상태
ros2 run wheelchair_control imu_safety_node

# 터미널 6: 위치추정 2차 감시
ros2 run wheelchair_control localization_monitor_node

# 터미널 7: WEB UI
ros2 launch wheelchair_ui web_ui.launch.py
```

### 원격 PC (맵핑 실행)
```bash
# 터미널 1: TF + EKF + Cartographer + Occupancy + RViz (전부 포함)
ros2 launch wheelchair_cartographer cartographer.launch.py

# 터미널 2: 키보드 (remap 없이)
ros2 run wheelchair_teleop teleop_keyboard

# 맵 저장
ros2 run nav2_map_server map_saver_cli -f ~/wheelchair_gazebo_ws/src/wheelchair_robot/wheelchair_navigation2/map/world
```

### 웹 UI 접속 주소
- 사용자: `http://localhost:8000/Wheelchair_SLAM_UI.html`
- 관리자: `http://localhost:8000/admin/Dashboard.html`

---

## 유지보수 정보

- Maintainer: 김성준 (`ksjun100848@naver.com`)
- ROS 환경: ROS 2 Humble Hawksbill (Ubuntu 22.04)
- License: Apache License 2.0
