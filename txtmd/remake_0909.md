STVL 파라미터 검증 결과 확인했습니다. 그럼 본론으로.

## 결론: 됩니다. 이게 정석에 가까운 구조입니다

Collision Monitor는 `sensor_msgs/LaserScan`이면 토픽 이름을 가리지 않습니다. `/scan_cl`을 넣으면 그대로 동작합니다.

핵심 분담:
- **판단(무엇이 위험한가)** → 필터 노드가 100% 담당
- **실행(감속/정지)** → Collision Monitor가 담당 (memoryless, 폴리곤 안 점 개수만 셈)

## 판단 기준: 3분류 대신 CPA 하나

각 트랙에 대해 **최근접 접근점(Closest Point of Approach)** 하나만 계산하면 원하시는 동작이 전부 나옵니다.

```
p = 장애물 상대위치 (base_link, x=전방 y=좌측)
v = 상대속도 = v_장애물 - v_로봇   (둘 다 odom 프레임에서 구한 뒤 base_link로 회전)

t_cpa = -(p·v) / |v|²
d_cpa = |p + v·t_cpa|
```

**통과(로봇이 반응) 조건**: `0 ≤ t_cpa ≤ 4.0` **AND** `d_cpa < 0.5m`
그 외는 전부 `/scan_cl`에서 삭제.

이 하나로 정리되는 케이스들:

| 상황 | 결과 |
|---|---|
| 이미 지나간 사람 | `t_cpa < 0` → **즉시 삭제, 로봇 바로 재개** |
| 앞을 스쳐 지나갈 사람 | `d_cpa` 큼 → 삭제 (정지 안 함) |
| 정면으로 오는 사람 | `t_cpa` 작고 `d_cpa` 작음 → 통과 → 정지 |
| 옆 벽 | `d_cpa` 큼 → 삭제 |
| 앞에 서 있는 사람 | 로봇이 접근 중이면 `t_cpa` 작음 → 통과 → 정지 |

각도 경계도, 다수결도, Crossing/Head-on 구분도 필요 없습니다. **단조 함수라 플리커링이 구조적으로 안 생깁니다.**

## 반드시 지켜야 할 안전 규칙 3개

**① `|v|` ≈ 0일 때 예외 처리 (안 하면 사고납니다)**

로봇이 멈춰 있고 사람도 서 있으면 `|v|²≈0`이라 `t_cpa`가 발산합니다. 이때 `d_cpa = |p|`로 쓰면 1.5m 앞에 선 사람이 `d_cpa=1.5 > 0.5`로 **삭제되어 로봇이 다시 출발해 들이받습니다.**

```python
if norm(v) < 0.1:
    t_cpa, d_cpa = 0.0, abs(p_y)   # |p|가 아니라 횡방향 오프셋
```
이러면 정면 1.5m에 선 사람은 `p_y≈0` → 통과 → 계속 정지 유지. 이후 데드락 타이머가 받습니다.

**② 신규 트랙은 무조건 통과 (fail-safe)**

속도 추정에 3~5프레임 필요합니다. 그 사이 트랙을 삭제하면 **코너에서 갑자기 나타난 사람이 0.4초간 투명인간**이 됩니다.

```python
if track.age < 3:  pass_through()   # 판단 불가 = 위험으로 간주
```

**③ Collision Monitor를 2단으로 체인**

`/scan_cl`만 보면 **정적 장애물 보호가 통째로 사라집니다.** Humble의 Collision Monitor는 폴리곤별 소스 지정(`sources_names`)이 없어서 한 노드에 두 소스를 섞으면 모든 폴리곤이 모든 소스를 봅니다. 노드를 두 개 띄우세요.

```
controller_server → cmd_vel_smoothed
   → [monitor_dynamic]  (/scan_cl, 넓은 폴리곤, slowdown+stop)  → cmd_vel_stage2
   → [monitor_emergency] (/scan 원본, 좁은 폴리곤, stop only)   → cmd_vel
```

## YAML

```yaml
collision_monitor_dynamic:
  ros__parameters:
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel_stage2"
    transform_tolerance: 0.2
    source_timeout: 0.5
    base_shift_correction: True
    polygons: ["DynSlow", "DynStop"]
    DynSlow:
      type: "polygon"
      points: "[[2.2, 0.45], [2.2, -0.45], [0.0, -0.45], [0.0, 0.45]]"
      action_type: "slowdown"
      slowdown_ratio: 0.35
      min_points: 3
      visualize: True
      polygon_pub_topic: "dyn_slow"
    DynStop:
      type: "polygon"
      points: "[[1.1, 0.35], [1.1, -0.35], [0.0, -0.35], [0.0, 0.35]]"
      action_type: "stop"
      min_points: 3
      visualize: True
      polygon_pub_topic: "dyn_stop"
    observation_sources: ["scan_cl"]
    scan_cl:
      type: "scan"
      topic: "scan_cl"

collision_monitor_emergency:
  ros__parameters:
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_stage2"
    cmd_vel_out_topic: "cmd_vel"
    transform_tolerance: 0.2
    source_timeout: 0.5
    polygons: ["EmergStop"]
    EmergStop:
      type: "polygon"
      points: "[[0.55, 0.30], [0.55, -0.30], [-0.15, -0.30], [-0.15, 0.30]]"
      action_type: "stop"
      min_points: 3
      visualize: True
      polygon_pub_topic: "emerg_stop"
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "scan"
```

> Humble 파싱 주의: `points`가 문자열이 아닌 flat 배열이거나, `min_points` 대신 `max_points`(= min_points−1)일 수 있습니다. 파라미터 에러 나면 이 둘부터 확인하세요.

## 필터 노드 구현 시 함정

- **`/scan_cl`은 원본과 배열 구조가 완전히 동일해야 합니다.** `header`(stamp/frame_id), `angle_min/max`, `angle_increment`, `ranges` 길이 전부 유지. 삭제할 빔만 `float('inf')`로 덮어쓰기. 배열을 재구성하면 각도 매핑이 깨집니다.
- **트래킹은 `odom` 프레임에서.** `map`으로 하면 AMCL 점프가 가짜 속도로 들어옵니다.
- **로봇 속도는 `cmd_vel`이 아니라 `/odom`의 twist에서.** EKF 융합값이라 슬립이 일부 보정됩니다.
- **로컬 코스트맵은 원본 `/scan` 유지.** 정적 장애물 + inflation이 컨트롤러 충돌검사에 필요합니다. 대신 RPP가 사람 때문에 경로를 abort하지 않도록 `max_allowed_time_to_collision_up_to_carrot: 1.0` 정도로 짧게.
- **글로벌 코스트맵의 `stvl_layer`는 제거하셨나요?** 이게 남아 있으면 `voxel_decay`를 아무리 줄여도 우회는 계속 생깁니다. 마킹되는 순간 이미 replan이 돌기 때문입니다.

## 이 구조가 못 하는 것

- 코너 급출현은 여전히 못 풉니다 → SpeedFilter 상시 감속 존으로 완화만
- 복도를 영구 차단한 물체(주차된 침대)면 영원히 섭니다 → 데드락 폴백 필요 (정지 20초 → 음성, 40초 → global costmap obstacle 일시 활성화 후 replan)
- Gazebo 더미 보행자는 양보를 안 하므로 정면 케이스가 전부 데드락으로만 끝납니다. 시나리오 스크립트를 따로 짜야 검증에 의미가 생깁니다.

## 순서

1. 글로벌 코스트맵 `stvl_layer` 제거 → 우회 사라지는지 확인
2. 필터 노드 작성 + RViz에서 `/scan_cl` 원본과 겹쳐보기 (**액추에이션 없이 눈으로만**)
3. Collision Monitor 2단 연결
4. 데드락 폴백

2번 필터 노드(`dynamic_scan_filter.py`) 바로 작성해드릴까요? CPA 판정 + 트래킹 + 마커 퍼블리시 포함해서 드리겠습니다.ROS2 노드로 작성하겠습니다. 앞서 정리한 CPA 판정 + 트래킹 + fail-safe 규칙을 모두 반영합니다.`src/wheelchair_robot/wheelchair_robot_ai/wheelchair_robot_ai/` 아래에 넣고 `setup.py`의 `console_scripts`에 등록하세요.

**실행 파라미터 (Gazebo면 `use_sim_time` 필수)**

```bash
ros2 run wheelchair_robot_ai dynamic_scan_filter --ros-args \
  -p use_sim_time:=true \
  -p tcpa_max:=4.0 -p dcpa_max:=0.5
```

**단계 2 검증 방법 (액추에이션 연결 전)**

Collision Monitor를 아직 붙이지 마세요. RViz에 LaserScan 두 개를 띄우고 비교합니다.

- `/scan` → 흰색, size 0.02
- `/scan_cl` → 빨강, size 0.06
- `/dynamic_scan_filter/tracks` → MarkerArray

확인할 것:

| 상황 | 기대 |
|---|---|
| 벽만 있을 때 | `/scan_cl` 완전히 빔. 빨간 점 0개 |
| 정면으로 사람 접근 | 사람 위치에만 빨간 점, 라벨 `threat` |
| 사람이 로봇 옆을 지나침 | 접근 중엔 `clear`(파랑), 통과 후 `passed`(회색) |
| **사람이 지나간 직후** | **빨간 점이 즉시 사라짐** ← 이게 핵심 지표 |
| 사람이 앞에 서 있고 로봇 접근 | `threat` 유지 |
| 코너에서 갑자기 등장 | 첫 0.3초 `new` (무조건 빨강) |

**튜닝 포인트**

- 정지가 너무 잦다 → `dcpa_max` 0.5 → 0.4
- 늦게 반응한다 → `tcpa_max` 4.0 → 5.0
- 한 사람이 두 ID로 잡힌다 → `merge_dist` 0.6 → 0.75
- 벽 일부가 트랙이 된다 → `max_cluster_width` 0.8 → 0.6
- 속도 화살표가 심하게 떤다 → `r_std` 0.10 → 0.15

**알려진 한계**

- 문틀·소화전 같은 작은 돌출물은 사람과 구분이 안 됩니다. 경로 중심 ±0.5m 안에 있으면 감속이 걸립니다. 병원에서는 오히려 바람직하지만, 거슬리면 `DynSlow` 폴리곤 폭을 ±0.45 → ±0.35로 좁히세요.
- 로봇이 빠르게 회전 중일 때 CPA 정확도가 떨어집니다. 복도 직진 위주라 실사용엔 문제없습니다.
- 이 노드가 죽으면 `/scan_cl`이 끊기고, Collision Monitor의 `source_timeout: 0.5`가 로봇을 정지시킵니다. 의도된 fail-safe입니다.

RViz에서 위 표대로 나오는지 먼저 보시고, 결과 알려주시면 Collision Monitor 연결 단계로 넘어가겠습니다.

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dynamic_scan_filter.py

2D LiDAR 스캔에서 "지금 로봇에게 위협이 되는 대상"의 빔만 남기고
나머지를 전부 inf 로 지운 /scan_cl 을 발행한다.

이 토픽은 Collision Monitor(dynamic 단) 의 입력으로만 쓴다.
정적 장애물 보호는 원본 /scan 을 보는 emergency 단이 별도로 담당한다.

판정은 CPA(Closest Point of Approach) 스칼라 2개로만 한다.
  통과(=위협) 조건 :  0 <= t_cpa <= tcpa_max   AND   d_cpa <= dcpa_max
  그 외는 삭제.

  - 이미 지나간 사람      -> t_cpa < 0        -> 삭제 (로봇 즉시 재개)
  - 앞을 스쳐 지나갈 사람  -> d_cpa 큼        -> 삭제 (정지 안 함)
  - 정면으로 오는 사람    -> t_cpa/d_cpa 작음 -> 통과 -> 감속/정지
  - 경로 위에 서 있는 사람 -> 로봇이 접근중    -> 통과 -> 정지

Fail-safe 규칙 3가지 (반드시 유지할 것):
  1) age < min_age_frames 인 신규 트랙은 무조건 통과. (속도 추정 전 = 판단 불가 = 위험)
  2) |v_rel| ~ 0 인 경우 t_cpa 발산 -> t_cpa=0, d_cpa=|횡방향 오프셋| 로 대체.
     (|p| 를 쓰면 정면 1.5m 에 선 사람이 삭제되어 들이받는다)
  3) tf/odom 이 없어도 스캔은 반드시 매 프레임 발행. 발행이 끊기면
     Collision Monitor 의 source_timeout 이 로봇을 정지시킨다.

Author: for wheelchair_robot_navigation2
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import tf2_ros


# --------------------------------------------------------------------------- #
#  Track : 등속(CV) 모델 4상태 칼만 필터  (x, y, vx, vy)  @ odom frame
# --------------------------------------------------------------------------- #
class Track:
    _next_id = 1

    def __init__(self, x, y, stamp):
        self.id = Track._next_id
        Track._next_id += 1
        if Track._next_id > 9999:
            Track._next_id = 1

        self.X = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        self.P = np.diag([0.25, 0.25, 4.0, 4.0]).astype(np.float64)

        self.last_t = stamp
        self.age = 1          # 업데이트 횟수
        self.misses = 0.0     # 미매칭 지속 시간 [s]

        # 프레임별 결과
        self.beams = []       # 이번 프레임에서 이 트랙에 속한 빔 인덱스
        self.threat = True    # fail-safe: 기본 통과
        self.t_cpa = 0.0
        self.d_cpa = 0.0
        self.reason = 'new'

    @property
    def pos(self):
        return self.X[0:2]

    @property
    def vel(self):
        return self.X[2:4]

    def predict(self, t, q_accel):
        dt = t - self.last_t
        if dt <= 0.0:
            dt = 1e-3
        if dt > 1.0:
            dt = 1.0

        F = np.eye(4)
        F[0, 2] = dt
        F[1, 3] = dt

        q = q_accel ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        Qb = np.array([[dt4 / 4.0, dt3 / 2.0],
                       [dt3 / 2.0, dt2]], dtype=np.float64) * q
        Q = np.zeros((4, 4))
        Q[np.ix_([0, 2], [0, 2])] = Qb
        Q[np.ix_([1, 3], [1, 3])] = Qb

        self.X = F @ self.X
        self.P = F @ self.P @ F.T + Q
        self.last_t = t

    def update(self, z, r_std):
        H = np.zeros((2, 4))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        R = np.eye(2) * (r_std ** 2)

        y = z - H @ self.X
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.X = self.X + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

        self.age += 1
        self.misses = 0.0


# --------------------------------------------------------------------------- #
#  Node
# --------------------------------------------------------------------------- #
class DynamicScanFilter(Node):

    def __init__(self):
        super().__init__('dynamic_scan_filter')

        # ---------------- parameters ---------------- #
        d = self.declare_parameter
        d('scan_topic', '/scan')
        d('odom_topic', '/odom')
        d('out_topic', '/scan_cl')
        d('odom_frame', 'odom')

        # 세그멘테이션
        d('max_consider_range', 5.0)    # 이 거리 밖은 아예 무시
        d('seg_max_gap', 0.15)          # 인접 점 간격 임계 [m]
        d('min_seg_points', 2)          # 세그먼트 최소 점 개수
        d('max_cluster_width', 0.80)    # 이보다 넓으면 벽으로 간주 -> 버림
        d('merge_dist', 0.60)           # 다리 가위질/카트 병합 거리 [m]
        d('max_merged_width', 1.30)     # 병합 후 상한

        # 트래킹
        d('gate_radius', 0.50)          # 데이터 연관 게이팅 [m]
        d('track_timeout', 0.40)        # 미매칭 지속 시 트랙 삭제 [s]
        d('q_accel', 0.60)              # 프로세스 노이즈 (보행자 가속 std) [m/s^2]
        d('r_std', 0.10)                # 측정 노이즈 (중심점 지터 std) [m]

        # CPA 판정
        d('min_age_frames', 3)          # 이 미만이면 무조건 통과 (fail-safe #1)
        d('v_static_thresh', 0.10)      # |v_rel| 이 미만이면 축퇴 처리 (fail-safe #2)
        d('tcpa_max', 4.0)              # [s]
        d('dcpa_max', 0.50)             # [m]

        d('publish_markers', True)

        g = lambda n: self.get_parameter(n).value
        self.scan_topic = g('scan_topic')
        self.odom_topic = g('odom_topic')
        self.out_topic = g('out_topic')
        self.odom_frame = g('odom_frame')

        self.max_range = float(g('max_consider_range'))
        self.seg_gap = float(g('seg_max_gap'))
        self.min_seg_pts = int(g('min_seg_points'))
        self.max_width = float(g('max_cluster_width'))
        self.merge_dist = float(g('merge_dist'))
        self.max_merged_width = float(g('max_merged_width'))

        self.gate = float(g('gate_radius'))
        self.track_timeout = float(g('track_timeout'))
        self.q_accel = float(g('q_accel'))
        self.r_std = float(g('r_std'))

        self.min_age = int(g('min_age_frames'))
        self.v_static = float(g('v_static_thresh'))
        self.tcpa_max = float(g('tcpa_max'))
        self.dcpa_max = float(g('dcpa_max'))

        self.do_markers = bool(g('publish_markers'))

        # ---------------- state ---------------- #
        self.tracks = []
        self.robot_v_base = np.array([0.0, 0.0])   # base_link 기준 로봇 선속도
        self.have_odom = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- pub / sub ---------------- #
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        self.pub_scan = self.create_publisher(LaserScan, self.out_topic, sensor_qos)
        self.pub_marker = self.create_publisher(MarkerArray, '~/tracks', 1)

        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, sensor_qos)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.get_logger().info(
            f'dynamic_scan_filter : {self.scan_topic} -> {self.out_topic} '
            f'(tcpa<={self.tcpa_max}s, dcpa<={self.dcpa_max}m)')

    # ------------------------------------------------------------------ #
    def on_odom(self, msg):
        # /odom 의 twist 는 child_frame(base_link) 기준.
        # cmd_vel 이 아니라 여기서 가져와야 EKF 융합값(슬립 일부 보정)을 쓴다.
        self.robot_v_base = np.array([msg.twist.twist.linear.x,
                                      msg.twist.twist.linear.y])
        self.have_odom = True

    # ------------------------------------------------------------------ #
    def on_scan(self, scan):
        now = self.stamp_to_sec(scan.header.stamp)

        clusters = self.segment_and_cluster(scan)

        T = self.lookup_odom_tf(scan.header.frame_id, scan.header.stamp)
        if T is None:
            # tf 없으면 판단 불가 -> 원본 그대로 통과시켜 보수적으로 동작
            self.publish_scan(scan, None, passthrough_all=True)
            return
        Rm, tvec, yaw = T

        # 클러스터 중심점을 odom 프레임으로
        meas = []
        for c in clusters:
            p_odom = Rm @ c['centroid'] + tvec
            meas.append({'z': p_odom, 'beams': c['beams']})

        self.track_step(meas, now)
        self.evaluate_threat(yaw)
        self.publish_scan(scan, self.tracks)

        if self.do_markers:
            self.publish_markers(scan.header.stamp, Rm, tvec)

    # ------------------------------------------------------------------ #
    #  1) 세그멘테이션 + 클러스터링                                       #
    # ------------------------------------------------------------------ #
    def segment_and_cluster(self, scan):
        n = len(scan.ranges)
        if n == 0:
            return []

        r = np.asarray(scan.ranges, dtype=np.float64)
        ang = scan.angle_min + np.arange(n) * scan.angle_increment

        rmin = max(scan.range_min, 0.05)
        rmax = min(scan.range_max, self.max_range)
        valid = np.isfinite(r) & (r > rmin) & (r < rmax)

        xs = np.where(valid, r * np.cos(ang), 0.0)
        ys = np.where(valid, r * np.sin(ang), 0.0)

        # --- 인접점 간격 기반 분할 ---
        segs = []
        cur = []
        for i in range(n):
            if not valid[i]:
                if len(cur) >= self.min_seg_pts:
                    segs.append(cur)
                cur = []
                continue
            if not cur:
                cur = [i]
            else:
                j = cur[-1]
                if math.hypot(xs[i] - xs[j], ys[i] - ys[j]) <= self.seg_gap:
                    cur.append(i)
                else:
                    if len(cur) >= self.min_seg_pts:
                        segs.append(cur)
                    cur = [i]
        if len(cur) >= self.min_seg_pts:
            segs.append(cur)

        # --- 360도 스캔 wrap 병합 ---
        span = scan.angle_max - scan.angle_min
        if len(segs) >= 2 and abs(abs(span) - 2.0 * math.pi) < 0.15:
            a, b = segs[0], segs[-1]
            if a[0] == 0 and b[-1] == n - 1:
                if math.hypot(xs[a[0]] - xs[b[-1]], ys[a[0]] - ys[b[-1]]) <= self.seg_gap:
                    segs[0] = b + a
                    segs.pop()

        # --- 폭 필터 + 중심점 ---
        raw = []
        for s in segs:
            px, py = xs[s], ys[s]
            width = math.hypot(px[0] - px[-1], py[0] - py[-1])
            if width > self.max_width:
                continue                       # 벽 / 긴 구조물
            raw.append({'beams': list(s),
                        'centroid': np.array([px.mean(), py.mean()]),
                        'width': width})

        return self.merge_clusters(raw, xs, ys)

    def merge_clusters(self, raw, xs, ys):
        """다리 가위질(twin cluster) / 사람+카트 를 하나로 묶는다.
        중심점 단일 연결(single-linkage), 병합 후 폭 상한 초과분은 폐기."""
        m = len(raw)
        if m == 0:
            return []

        parent = list(range(m))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        for i in range(m):
            for j in range(i + 1, m):
                if np.linalg.norm(raw[i]['centroid'] - raw[j]['centroid']) <= self.merge_dist:
                    ri, rj = find(i), find(j)
                    if ri != rj:
                        parent[ri] = rj

        groups = {}
        for i in range(m):
            groups.setdefault(find(i), []).append(i)

        out = []
        for members in groups.values():
            beams = []
            for i in members:
                beams.extend(raw[i]['beams'])
            px = xs[beams]
            py = ys[beams]
            width = math.hypot(px.max() - px.min(), py.max() - py.min())
            if width > self.max_merged_width:
                continue                       # 병합 결과가 벽 크기면 폐기
            out.append({'beams': beams,
                        'centroid': np.array([px.mean(), py.mean()]),
                        'width': width})
        return out

    # ------------------------------------------------------------------ #
    #  2) 데이터 연관 + 칼만 필터                                          #
    # ------------------------------------------------------------------ #
    def track_step(self, meas, now):
        for t in self.tracks:
            dt_prev = now - t.last_t
            t.predict(now, self.q_accel)
            t.beams = []
            t.misses += max(dt_prev, 0.0)

        # greedy nearest-neighbor + gating
        pairs = []
        for ti, t in enumerate(self.tracks):
            for mi, mz in enumerate(meas):
                dist = np.linalg.norm(t.pos - mz['z'])
                if dist <= self.gate:
                    pairs.append((dist, ti, mi))
        pairs.sort(key=lambda p: p[0])

        used_t, used_m = set(), set()
        for dist, ti, mi in pairs:
            if ti in used_t or mi in used_m:
                continue
            t = self.tracks[ti]
            t.update(meas[mi]['z'], self.r_std)
            t.beams = meas[mi]['beams']
            used_t.add(ti)
            used_m.add(mi)

        # 신규 트랙
        for mi, mz in enumerate(meas):
            if mi in used_m:
                continue
            nt = Track(mz['z'][0], mz['z'][1], now)
            nt.beams = mz['beams']
            self.tracks.append(nt)

        # 소멸
        self.tracks = [t for t in self.tracks if t.misses <= self.track_timeout]

    # ------------------------------------------------------------------ #
    #  3) CPA 판정                                                        #
    # ------------------------------------------------------------------ #
    def evaluate_threat(self, robot_yaw):
        """CPA 는 관성계(odom)에서 계산한다. t_cpa / d_cpa 는 프레임 무관.
        축퇴(|v_rel|~0) 처리에만 base_link 횡방향 오프셋이 필요하다."""
        c, s = math.cos(robot_yaw), math.sin(robot_yaw)
        R_wb = np.array([[c, -s], [s, c]])          # base -> odom
        R_bw = R_wb.T                               # odom -> base

        v_robot_odom = R_wb @ self.robot_v_base if self.have_odom else np.zeros(2)
        p_robot_odom = self.robot_p_odom

        for t in self.tracks:
            # --- fail-safe #1 : 속도 추정 전이면 무조건 위협 ---
            if t.age < self.min_age:
                t.threat = True
                t.t_cpa, t.d_cpa = 0.0, 0.0
                t.reason = 'new'
                continue

            p_rel = t.pos - p_robot_odom            # odom frame
            v_rel = t.vel - v_robot_odom
            vv = float(v_rel @ v_rel)

            if vv < self.v_static ** 2:
                # --- fail-safe #2 : 상대속도 0 -> t_cpa 발산 ---
                # |p_rel| 을 쓰면 정면 1.5m 의 정지 인물이 삭제된다.
                p_base = R_bw @ p_rel
                t.t_cpa = 0.0
                t.d_cpa = abs(float(p_base[1]))     # 횡방향 오프셋만
                t.threat = (t.d_cpa <= self.dcpa_max)
                t.reason = 'static' if t.threat else 'static-clear'
                continue

            t_cpa = -float(p_rel @ v_rel) / vv
            if t_cpa < 0.0:
                # 이미 최근접점을 지났다 -> 멀어지는 중
                t.t_cpa, t.d_cpa = t_cpa, float(np.linalg.norm(p_rel))
                t.threat = False
                t.reason = 'passed'
                continue

            d_cpa = float(np.linalg.norm(p_rel + v_rel * t_cpa))
            t.t_cpa, t.d_cpa = t_cpa, d_cpa
            t.threat = (t_cpa <= self.tcpa_max) and (d_cpa <= self.dcpa_max)
            t.reason = 'threat' if t.threat else 'clear'

    # ------------------------------------------------------------------ #
    #  4) /scan_cl 발행                                                   #
    # ------------------------------------------------------------------ #
    def publish_scan(self, scan, tracks, passthrough_all=False):
        """원본과 배열 구조를 100% 동일하게 유지하고, 지울 빔만 inf 로 덮는다.
        배열을 재구성하면 각도 매핑이 깨진다."""
        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max

        if passthrough_all:
            out.ranges = list(scan.ranges)
            out.intensities = list(scan.intensities)
            self.pub_scan.publish(out)
            return

        n = len(scan.ranges)
        keep = np.full(n, np.inf, dtype=np.float32)
        src = np.asarray(scan.ranges, dtype=np.float32)

        if tracks:
            for t in tracks:
                if t.threat and t.beams:
                    idx = np.asarray(t.beams, dtype=np.int64)
                    keep[idx] = src[idx]

        out.ranges = keep.tolist()
        out.intensities = []
        self.pub_scan.publish(out)

    # ------------------------------------------------------------------ #
    #  tf helpers                                                         #
    # ------------------------------------------------------------------ #
    def lookup_odom_tf(self, laser_frame, stamp):
        """laser -> odom 2D 변환과, odom 기준 로봇 위치/헤딩을 함께 구한다."""
        try:
            tf_l = self.tf_buffer.lookup_transform(
                self.odom_frame, laser_frame, stamp, timeout=Duration(seconds=0.03))
        except Exception:
            try:
                tf_l = self.tf_buffer.lookup_transform(
                    self.odom_frame, laser_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(f'tf {self.odom_frame}<-{laser_frame} 실패: {e}',
                                       throttle_duration_sec=2.0)
                return None

        try:
            tf_b = self.tf_buffer.lookup_transform(
                self.odom_frame, 'base_link', rclpy.time.Time())
        except Exception:
            tf_b = tf_l   # 최후 폴백: 라이다 위치를 로봇 위치로 간주

        yaw = self.quat_to_yaw(tf_l.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        Rm = np.array([[c, -s], [s, c]])
        tvec = np.array([tf_l.transform.translation.x,
                         tf_l.transform.translation.y])

        self.robot_p_odom = np.array([tf_b.transform.translation.x,
                                      tf_b.transform.translation.y])
        robot_yaw = self.quat_to_yaw(tf_b.transform.rotation)
        return Rm, tvec, robot_yaw

    @staticmethod
    def quat_to_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    # ------------------------------------------------------------------ #
    #  RViz markers                                                       #
    # ------------------------------------------------------------------ #
    def publish_markers(self, stamp, Rm, tvec):
        arr = MarkerArray()
        clr = Marker()
        clr.action = Marker.DELETEALL
        arr.markers.append(clr)

        for t in self.tracks:
            if t.threat:
                col = (1.0, 0.25, 0.15)   # 빨강 = 통과(위협)
            elif t.reason == 'passed':
                col = (0.35, 0.35, 0.35)  # 회색 = 이미 지나감
            else:
                col = (0.25, 0.7, 1.0)    # 파랑 = 무시

            m = Marker()
            m.header.frame_id = self.odom_frame
            m.header.stamp = stamp
            m.ns = 'body'
            m.id = t.id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(t.X[0])
            m.pose.position.y = float(t.X[1])
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.40
            m.scale.z = 0.60
            m.color.r, m.color.g, m.color.b = col
            m.color.a = 0.55
            arr.markers.append(m)

            a = Marker()
            a.header = m.header
            a.ns = 'vel'
            a.id = t.id
            a.type = Marker.ARROW
            a.action = Marker.ADD
            p0 = Point(x=float(t.X[0]), y=float(t.X[1]), z=0.65)
            p1 = Point(x=float(t.X[0] + t.X[2]), y=float(t.X[1] + t.X[3]), z=0.65)
            a.points = [p0, p1]
            a.scale.x, a.scale.y, a.scale.z = 0.05, 0.10, 0.12
            a.color.r, a.color.g, a.color.b = col
            a.color.a = 0.95
            arr.markers.append(a)

            txt = Marker()
            txt.header = m.header
            txt.ns = 'label'
            txt.id = t.id
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(t.X[0])
            txt.pose.position.y = float(t.X[1])
            txt.pose.position.z = 1.05
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.16
            txt.color.r = txt.color.g = txt.color.b = 1.0
            txt.color.a = 0.95
            spd = float(np.linalg.norm(t.vel))
            txt.text = (f'#{t.id} {t.reason}\n'
                        f'|v|={spd:.2f} t={t.t_cpa:.1f} d={t.d_cpa:.2f}')
            arr.markers.append(txt)

        self.pub_marker.publish(arr)


# --------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = DynamicScanFilter()
    node.robot_p_odom = np.zeros(2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()