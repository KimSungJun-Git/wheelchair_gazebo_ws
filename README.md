# Wheelchair Robot — 실내 자율주행 스마트 휠체어 시스템

ROS 2 (Humble) 기반의 실내 자율주행 전동 휠체어 플랫폼입니다.  
탑승자는 터치 웹 UI에서 목적지(병실, 응급실, 대기실 등)를 선택하거나 수동 조이스틱 조작 및 SOS 호출을 수행할 수 있으며, 휠체어는 다중 센서(2D LiDAR, IMU, 휠 오도메트리, 3방향 초음파)와 확장 칼만 필터(EKF), Nav2를 결합해 동적·정적 장애물을 안전하게 회피하며 주행합니다. 보호자 및 관리자는 통합 관제 대시보드에서 실시간 주행 상태와 LLM 기반 AI 주행 분석 리포트를 확인할 수 있습니다.

---

## 1. 주요 기능

### 1) 센서 융합 측위 및 SLAM (`wheelchair_control` · `wheelchair_cartographer`)
* **IMU 프레임 보정 및 EKF 센서 융합**: `imu_relay_node`가 AHRS 센서의 프레임(`imu_link`)을 보정하여 `/imu/data_fixed`로 중계하고, `robot_localization`(`ekf_filter_node`)이 휠 오도메트리(`/odom`)와 IMU를 융합해 바퀴 슬립에 강인한 `/odometry/filtered` 및 `odom → base_footprint` TF를 발행합니다.
* **2D SLAM 맵핑**: Cartographer를 통해 실내 격자 지도(`world.pgm`, `world.yaml`)를 생성합니다.

### 2) CPA 기반 동적 장애물 분리 및 지능형 주행 (`wheelchair_control` · `wheelchair_navigation2`)
* **동적/정적 스캔 분리 (`dynamic_scan_filter`)**: 2D LiDAR 스캔(`/scan`)을 등속(CV) 칼만 필터로 추적하고 CPA(Closest Point of Approach, 최근접 접근점)를 계산하여, 충돌 위협이 있는 동적 보행자 빔은 `/scan_threat`로, 정적 배경 빔은 `/scan_static`으로 실시간 분리 발행합니다.
* **커스텀 행동 트리 플러그인 (`IsThreatPresent`)**: C++ 커스텀 BT 조건 노드(`is_threat_present_condition_bt_node.cpp`)와 `navigate_w_replanning_and_recovery.xml`을 통해 복구 동작을 이원화합니다.
  * 동적 위협(`/scan_threat`) 감지 시: 무리한 후진이나 경로 이탈 대신 제자리 대기(`Wait 2.0s`) 수행
  * 정적 장애물 고립 시: 로컬 코스트맵 초기화 후 안전 후진(`BackUp 0.20m`) 수행
* **구역 필터링**: Keepout Filter(`keepout_mask.yaml`) 및 Speed Filter(`speed_mask.yaml`)를 적용해 계단·위험 구역 진입을 원천 차단하고 서행 구역에서 자동 감속합니다.

### 3) 다중 계층 안전 중재 및 제어권 전환 (`wheelchair_control`)
* **다중 위험 감시 (`safety_stop_node`)**:
  * 전방·좌·우 3방향 초음파 센서 근접 감지 및 회피 가능 방향(`/avoidance_direction`) 판단
  * CPA 동적 위협(`/scan_threat`) 및 원본 라이다 초근접 장애물 긴급 정지/감속 제어
  * 센서 통신 두절(Timeout) 실시간 감시 (`/sensor_health`)
* **차체 기울기·충격 감시 (`imu_safety_node`)**: 전복 위험 각도 또는 급격한 충격 감지 시 `/emergency_stop/imu` 즉시 발행.
* **위치 추정 이상 감시 (`localization_monitor_node`)**: AMCL 공분산(`cov_x`, `cov_y`, `cov_yaw`) 및 토픽 주기를 상태 머신(`init` → `ok` / `stale` / `uncertain` / `lost`)으로 감시하여 위치 상실 시 `/emergency_stop/localization` 발행 및 글로벌 재측위 요청.
* **수동/자율 제어권 중재 (`mode_switch_node`)**: 안전 검증을 통과한 자율주행 속도(`/cmd_vel_safe`)와 수동 조작 속도(`/cmd_vel_teleop`)를 현재 모드(`auto` / `manual`)에 따라 중재하여 최종 모터 명령(`/cmd_vel`)으로 출력합니다. 수동 개입이나 SOS 발생 시 즉시 자율주행 목표를 취소합니다.

### 4) 탑승자 웹 UI & 관리자 관제 대시보드 (`wheelchair_ui`)
* 단일 런치(`web_ui.launch.py`)로 `rosbridge_websocket`(9090), 정적 웹 서버(8000), FastAPI 관제 백엔드(`admin_server.py`, 8090), 주행 로거(`log_collector_node`)를 통합 기동합니다.
* **탑승자 화면 (`Wheelchair_SLAM_UI.html`)**: 실시간 SLAM 맵·경로·5방향 장애물 거리 시각화, 목적지 원탭 호출, 수동/자율 전환, SOS 긴급 호출, 한/영 다국어 지원.
* **관리자 대시보드 (`admin/Dashboard.html`)**: 시간대별·원인별 안전 이벤트 통계 및 SLAM 좌표 히트맵(개요), 실시간 위치·이벤트 스트림 모니터링 및 원격 긴급 정지(실시간 모니터), 세션별 AI 진단 리포트 열람(보고서).

### 5) 주행 로그 수집 및 로컬 LLM 사후 분석 (`wheelchair_ai`)
* **`log_collector_node`**: 주행 중 발생하는 모든 안전 개입(`blocked`, `modified`, `sos`, `allowed`)과 센서 상태, 로봇 좌표를 중복 제거하여 `~/wheelchair_ws/driving_data/*.json`에 기록합니다.
* **`agent_analyzer`**: 주행 종료 후 로컬 LLM(Ollama `qwen2.5:7b`)을 통해 통계 기반 한국어 분석 보고서(`*_report.md`)를 자동 생성합니다.

---

## 2. 패키지 및 디렉토리 구조

```text
wheelchair_gazebo_ws/
├── run_nav.sh                          # 자율주행 + 안전제어 + 웹 UI 원클릭 통합 실행 스크립트
├── run_mapping.sh                      # Cartographer SLAM 맵핑 원클릭 실행 스크립트
├── scripts/
│   ├── run_all.sh                      # 주행 로그 수집 및 AI 분석 연속 실행 스크립트
│   ├── run_logger.sh                   # 주행 로그 수집기 단독 실행
│   └── run_analyzer.sh                 # AI 주행 분석기 단독 실행
└── src/wheelchair_robot/
    ├── wheelchair_description/         # 로봇 URDF 모델(robot.urdf), 3D 메쉬(robot_n2.stl), RViz 설정
    ├── wheelchair_cartographer/        # Cartographer 2D SLAM 설정(cartographer.lua) 및 런치 파일
    ├── wheelchair_navigation2/         # Nav2 파라미터(navigation2.yaml), 맵, Keepout/Speed 마스크,
    │                                   # 커스텀 BT XML 및 C++ 조건 노드(is_threat_present_condition_bt_node.cpp)
    ├── wheelchair_control/             # 핵심 제어 및 안전 패키지
    │   ├── config/                     # ekf.yaml, safety.yaml
    │   ├── launch/                     # bringup.launch.py (TF+IMU릴레이+EKF), safety.launch.py
    │   └── wheelchair_control/         # dynamic_scan_filter, safety_stop_node, mode_switch_node,
    │                                   # imu_safety_node, localization_monitor_node, imu_relay_node
    ├── wheelchair_teleop/              # 키보드 수동 조작 노드 (teleop_keyboard.py)
    ├── wheelchair_ai/                  # 주행 이벤트 로거(log_collector_node) 및 LLM 분석기(agent_analyzer)
    └── wheelchair_ui/                  # 탑승자용 React 웹 UI + 관리자 대시보드 + FastAPI 백엔드(admin_server.py)
```

---

## 3. 시스템 데이터 흐름

```text
[라즈베리파이 센서 계층]
  ├─ stella_ahrs (/imu/data) ──► [imu_relay_node] ──► /imu/data_fixed ─┐
  ├─ stella_md   (/odom)     ──────────────────────────────────────────┼─► [ekf_filter_node] ──► /odometry/filtered (TF: odom→base_footprint)
  ├─ ydlidar     (/scan)     ──► [dynamic_scan_filter] ──► /scan_static (Nav2 코스트맵) & /scan_threat (동적 위협)
  └─ uul.py      (/ultrasonic/range, /left, /right)

[자율주행 및 안전 제어 계층]
  [Nav2 (BT + Keepout/Speed Filter)]
        │  (/cmd_vel_nav)
        ▼
  [safety_stop_node] ◄── /scan, /scan_threat, /ultrasonic/*, /emergency_stop/{imu,localization}
        │  (/cmd_vel_safe, /safety_action, /safety_alert, /sensor_health)
        ▼
  [mode_switch_node] ◄── /cmd_vel_teleop (키보드/웹 수동 조작), /destination, /mode_switch
        │
        ▼
    /cmd_vel (모터 드라이버 구동)

[웹 UI 및 AI 로깅 계층]
  /safety_action, /sos_trigger, /amcl_pose
        ├──► [log_collector_node] ──► driving_data/*.json ──► [agent_analyzer (LLM)] ──► *_report.md
        ├──► [admin_server (FastAPI :8090)] ◄──► 관제 대시보드 (http://<host>:8000/admin/Dashboard.html)
        └──► [rosbridge_websocket (:9090)]  ◄──► 탑승자 웹 UI  (http://<host>:8000/Wheelchair_SLAM_UI.html)
```

---

## 4. 빌드 및 실행 방법

### 1) 워크스페이스 빌드
```bash
# 의존성 설치 (최초 1회)
pip3 install fastapi uvicorn

# 전체 패키지 빌드
colcon build --symlink-install
source install/setup.bash
```

### 2) 라즈베리파이 (하드웨어 센서 구동)
실차 주행 전 라즈베리파이에서 4종의 센서 드라이버를 먼저 실행합니다.
```bash
# 1. IMU 센서
ros2 launch stella_ahrs stella_ahrs_launch.py

# 2. 2D LiDAR 센서
ros2 launch ydlidar_ros2_driver ydlidar_launch.py

# 3. 모터 드라이버 및 휠 오도메트리
ros2 launch stella_md stella_md_launch.py

# 4. 3방향 초음파 센서
cd ~/colcon_ws/src/ && python3 uul.py
```

### 3) 원격 PC — 자율주행 통합 실행 (원클릭 스크립트)
```bash
./run_nav.sh
```
`run_nav.sh` 실행 시 아래 7개 프로세스가 멀티 탭(또는 `tmux` 세션)으로 순차 기동됩니다:
1. `ros2 launch wheelchair_control bringup.launch.py` (URDF TF + `imu_relay_node` + `ekf_filter_node`)
2. `ros2 launch wheelchair_navigation2 navigation2.launch.py` (Nav2 + Keepout/Speed 마스크 + `safety.launch.py`[동적 스캔 필터 & 안전 정지 노드] + RViz2)
3. `ros2 run wheelchair_control mode_switch_node` (수동/자율 모드 중재기)
4. `ros2 run wheelchair_teleop teleop_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop` (수동 키보드 제어)
5. `ros2 run wheelchair_control imu_safety_node` (기울기/충격 감시)
6. `ros2 run wheelchair_control localization_monitor_node` (AMCL 공분산/위치 상실 감시)
7. `ros2 launch wheelchair_ui web_ui.launch.py` (웹 서버 + rosbridge + 관제 API + 주행 로거)

### 4) 원격 PC — SLAM 맵핑 실행 (새 지도 작성 시)
```bash
./run_mapping.sh [저장할_맵_이름]
```
맵핑 완료 후 아래 명령으로 지도를 저장합니다:
```bash
ros2 run nav2_map_server map_saver_cli -f src/wheelchair_robot/wheelchair_navigation2/map/world
```

### 5) 웹 UI 접속 주소
| 화면 구분 | 접속 URL | 비고 |
|---|---|---|
| **랜딩 페이지** | `http://localhost:8000/` | 탑승자 UI / 관제 대시보드 선택 화면 |
| **탑승자 UI** | `http://localhost:8000/Wheelchair_SLAM_UI.html` | 목적지 선택, 수동 조작, SOS 호출 |
| **관리자 대시보드** | `http://localhost:8000/admin/Dashboard.html` | 실시간 관제, 통계, AI 리포트 (`smac` / `0000`) |

---

## 5. 유지보수 정보

* **Maintainer**: 김성준 (`ksjun100848@naver.com`)
* **ROS Distro**: ROS 2 Humble Hawksbill (Ubuntu 22.04)
* **License**: Apache License 2.0
