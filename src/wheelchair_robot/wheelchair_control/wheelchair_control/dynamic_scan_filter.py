#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dynamic_scan_filter.py

2D LiDAR 스캔에서 CPA(Closest Point of Approach) 기반으로
충돌 위험이 있는 동적 장애물 빔을 선별하여 /scan_threat 토픽으로 발행하고,
정적 장애물 빔은 /scan_static으로 분리 발행한다.

Package: wheelchair_control
"""

import math
from typing import List, Dict, Any, Tuple, Optional, Set

import numpy as np
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
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
    _next_id: int = 1

    def __init__(self, x: float, y: float, stamp: float) -> None:
        self.id: int = Track._next_id
        Track._next_id += 1
        if Track._next_id > 9999:
            Track._next_id = 1

        self.X: np.ndarray = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        self.P: np.ndarray = np.diag([0.25, 0.25, 4.0, 4.0]).astype(np.float64)

        self.last_t: float = stamp
        self.age: int = 1          # 업데이트 횟수
        self.misses: float = 0.0   # 미매칭 지속 시간 [s]

        # 프레임별 결과
        self.beams: List[int] = [] # 이번 프레임에서 이 트랙에 속한 빔 인덱스
        self.threat: bool = False  # 동적 충돌 위협 여부
        self.is_static: bool = True # 정적 장애물 여부
        self.is_dynamic: bool = False # 동적 보행자 여부
        self.v_abs: float = 0.0    # 월드 절대 속도 크기 [m/s]
        self.t_cpa: float = 0.0
        self.d_cpa: float = 0.0
        self.reason: str = 'new'

    @property
    def pos(self) -> np.ndarray:
        return self.X[0:2]

    @property
    def vel(self) -> np.ndarray:
        return self.X[2:4]

    def predict(self, t: float, q_accel: float) -> None:
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
        Q = np.zeros((4, 4), dtype=np.float64)
        Q[np.ix_([0, 2], [0, 2])] = Qb
        Q[np.ix_([1, 3], [1, 3])] = Qb

        self.X = F @ self.X
        self.P = F @ self.P @ F.T + Q
        self.last_t = t

    def update(self, z: np.ndarray, r_std: float) -> None:
        H = np.zeros((2, 4), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        R = np.eye(2, dtype=np.float64) * (r_std ** 2)

        y = z - H @ self.X
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.X = self.X + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

        self.age += 1
        self.misses = 0.0


# --------------------------------------------------------------------------- #
#  Node : DynamicScanFilter
# --------------------------------------------------------------------------- #
class DynamicScanFilter(Node):

    def __init__(self) -> None:
        super().__init__('dynamic_scan_filter')

        # ---------------- parameters ---------------- #
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('out_topic', '/scan_threat')
        self.declare_parameter('out_static_topic', '/scan_static')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # 세그멘테이션 파라미터
        self.declare_parameter('max_consider_range', 5.0)
        self.declare_parameter('seg_max_gap', 0.15)
        self.declare_parameter('min_seg_points', 2)
        self.declare_parameter('max_cluster_width', 0.80)
        self.declare_parameter('merge_dist', 0.60)
        self.declare_parameter('max_merged_width', 1.30)

        # 트래킹 파라미터
        self.declare_parameter('gate_radius', 0.50)
        self.declare_parameter('track_timeout', 0.40)
        self.declare_parameter('q_accel', 0.60)
        self.declare_parameter('r_std', 0.10)

        # CPA 및 정적/동적 판정 파라미터
        self.declare_parameter('min_age_frames', 3)
        self.declare_parameter('v_dynamic_thresh', 0.30)
        self.declare_parameter('v_static_thresh', 0.10)
        self.declare_parameter('tcpa_max', 4.0)
        self.declare_parameter('dcpa_max', 0.50)
        self.declare_parameter('publish_markers', True)

        self.scan_topic: str = str(self.get_parameter('scan_topic').value)
        self.odom_topic: str = str(self.get_parameter('odom_topic').value)
        self.out_topic: str = str(self.get_parameter('out_topic').value)
        self.out_static_topic: str = str(self.get_parameter('out_static_topic').value)
        self.odom_frame: str = str(self.get_parameter('odom_frame').value)
        self.base_frame: str = str(self.get_parameter('base_frame').value)

        self.max_range: float = float(self.get_parameter('max_consider_range').value)
        self.seg_gap: float = float(self.get_parameter('seg_max_gap').value)
        self.min_seg_pts: int = int(self.get_parameter('min_seg_points').value)
        self.max_width: float = float(self.get_parameter('max_cluster_width').value)
        self.merge_dist: float = float(self.get_parameter('merge_dist').value)
        self.max_merged_width: float = float(self.get_parameter('max_merged_width').value)

        self.gate: float = float(self.get_parameter('gate_radius').value)
        self.track_timeout: float = float(self.get_parameter('track_timeout').value)
        self.q_accel: float = float(self.get_parameter('q_accel').value)
        self.r_std: float = float(self.get_parameter('r_std').value)

        self.min_age: int = int(self.get_parameter('min_age_frames').value)
        self.v_dynamic_thresh: float = float(self.get_parameter('v_dynamic_thresh').value)
        self.v_static: float = float(self.get_parameter('v_static_thresh').value)
        self.tcpa_max: float = float(self.get_parameter('tcpa_max').value)
        self.dcpa_max: float = float(self.get_parameter('dcpa_max').value)

        self.publish_markers_enabled: bool = bool(self.get_parameter('publish_markers').value)

        # ---------------- state ---------------- #
        self.tracks: List[Track] = []
        self.robot_v_base: np.ndarray = np.array([0.0, 0.0], dtype=np.float64)
        self.robot_p_odom: np.ndarray = np.array([0.0, 0.0], dtype=np.float64)
        self.have_odom: bool = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- pub / sub ---------------- #
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        self.pub_scan = self.create_publisher(LaserScan, self.out_topic, sensor_qos)
        self.pub_scan_static = self.create_publisher(LaserScan, self.out_static_topic, sensor_qos)
        self.pub_marker = self.create_publisher(MarkerArray, '~/tracks', 1)

        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, sensor_qos)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.get_logger().info(
            f"[dynamic_scan_filter] 시작: {self.scan_topic} -> "
            f"동적위협: {self.out_topic}, 정적장애물: {self.out_static_topic} "
            f"(v_dyn >= {self.v_dynamic_thresh} m/s, tcpa <= {self.tcpa_max} s, dcpa <= {self.dcpa_max} m)"
        )

    # ------------------------------------------------------------------ #
    def on_odom(self, msg: Odometry) -> None:
        # /odometry/filtered의 twist는 base_link 기준 EKF 융합 선속도
        self.robot_v_base = np.array([msg.twist.twist.linear.x,
                                      msg.twist.twist.linear.y], dtype=np.float64)
        self.have_odom = True

    # ------------------------------------------------------------------ #
    def on_scan(self, scan: LaserScan) -> None:
        now = self.stamp_to_sec(scan.header.stamp)

        clusters = self.segment_and_cluster(scan)

        tf_data = self.lookup_odom_tf(scan.header.frame_id, scan.header.stamp)
        if tf_data is None:
            # TF 조회 실패 시 원본 스캔 전달 (안전 폴백)
            self.publish_scan(scan, None, passthrough_all=True)
            return
        rot_matrix, translation, yaw = tf_data

        # 클러스터 중심점을 odom 프레임으로 변환
        measurements = []
        for cluster in clusters:
            p_odom = rot_matrix @ cluster['centroid'] + translation
            measurements.append({'z': p_odom, 'beams': cluster['beams']})

        self.track_step(measurements, now)
        self.evaluate_threat(yaw)
        self.publish_scan(scan, self.tracks)

        if self.publish_markers_enabled:
            self.publish_markers(scan.header.stamp)

    # ------------------------------------------------------------------ #
    #  1) 세그멘테이션 + 클러스터링                                       #
    # ------------------------------------------------------------------ #
    def segment_and_cluster(self, scan: LaserScan) -> List[Dict[str, Any]]:
        n = len(scan.ranges)
        if n == 0:
            return []

        r = np.asarray(scan.ranges, dtype=np.float64)
        ang = scan.angle_min + np.arange(n) * scan.angle_increment

        rmin = max(scan.range_min, 0.05)
        rmax = min(scan.range_max, self.max_range)
        valid = np.isfinite(r) & (r > rmin) & (r < rmax)

        r_safe = np.where(valid, r, 0.0)
        xs = r_safe * np.cos(ang)
        ys = r_safe * np.sin(ang)

        # 인접 점 간격 기반 분할
        segs: List[List[int]] = []
        cur: List[int] = []
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

        # 360도 스캔 wrap 병합
        span = scan.angle_max - scan.angle_min
        if len(segs) >= 2 and abs(abs(span) - 2.0 * math.pi) < 0.15:
            a, b = segs[0], segs[-1]
            if a[0] == 0 and b[-1] == n - 1:
                if math.hypot(xs[a[0]] - xs[b[-1]], ys[a[0]] - ys[b[-1]]) <= self.seg_gap:
                    segs[0] = b + a
                    segs.pop()

        # 폭 필터 및 중심점 산출
        raw: List[Dict[str, Any]] = []
        for s in segs:
            px, py = xs[s], ys[s]
            width = math.hypot(px[0] - px[-1], py[0] - py[-1])
            if width > self.max_width:
                continue                       # 벽 / 긴 구조물 제거
            raw.append({'beams': list(s),
                        'centroid': np.array([px.mean(), py.mean()]),
                        'width': width})

        return self.merge_clusters(raw, xs, ys)

    def merge_clusters(self, raw: List[Dict[str, Any]], xs: np.ndarray, ys: np.ndarray) -> List[Dict[str, Any]]:
        """인접한 클러스터를 단일 객체로 병합"""
        m = len(raw)
        if m == 0:
            return []

        parent = list(range(m))

        def find(a: int) -> int:
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

        groups: Dict[int, List[int]] = {}
        for i in range(m):
            groups.setdefault(find(i), []).append(i)

        out: List[Dict[str, Any]] = []
        for members in groups.values():
            beams: List[int] = []
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
    def track_step(self, meas: List[Dict[str, Any]], now: float) -> None:
        for t in self.tracks:
            dt_prev = now - t.last_t
            t.predict(now, self.q_accel)
            t.beams = []
            t.misses += max(dt_prev, 0.0)

        # greedy nearest-neighbor + gating
        pairs = []
        for ti, t in enumerate(self.tracks):
            for mi, mz in enumerate(meas):
                dist = float(np.linalg.norm(t.pos - mz['z']))
                if dist <= self.gate:
                    pairs.append((dist, ti, mi))
        pairs.sort(key=lambda p: p[0])

        used_t: Set[int] = set()
        used_m: Set[int] = set()
        for dist, ti, mi in pairs:
            if ti in used_t or mi in used_m:
                continue
            t = self.tracks[ti]
            t.update(meas[mi]['z'], self.r_std)
            t.beams = meas[mi]['beams']
            used_t.add(ti)
            used_m.add(mi)

        # 신규 트랙 생성
        for mi, mz in enumerate(meas):
            if mi in used_m:
                continue
            nt = Track(mz['z'][0], mz['z'][1], now)
            nt.beams = mz['beams']
            self.tracks.append(nt)

        # 오래된 미관측 트랙 소멸
        self.tracks = [t for t in self.tracks if t.misses <= self.track_timeout]

    # ------------------------------------------------------------------ #
    #  3) CPA(최근접 접근점) 위협 판정                                    #
    # ------------------------------------------------------------------ #
    def evaluate_threat(self, robot_yaw: float) -> None:
        """CPA는 관성계(odom)에서 계산 (t_cpa / d_cpa는 프레임 무관)"""
        c, s = math.cos(robot_yaw), math.sin(robot_yaw)
        R_wb = np.array([[c, -s], [s, c]])          # base -> odom
        R_bw = R_wb.T                               # odom -> base

        v_robot_odom = R_wb @ self.robot_v_base if self.have_odom else np.zeros(2)
        p_robot_odom = self.robot_p_odom

        for t in self.tracks:
            p_rel = t.pos - p_robot_odom            # odom frame 상대 위치
            v_rel = t.vel - v_robot_odom            # 상대 속도
            vv = float(v_rel @ v_rel)

            # 트랙의 월드 좌표계(odom) 기준 절대 속도 크기
            v_obs = float(np.linalg.norm(t.vel))
            t.v_abs = v_obs

            # --- 1) 신규 트랙 (속도 추정 수렴 전: 3프레임 미만) ---
            if t.age < self.min_age:
                dist_to_robot = float(np.linalg.norm(p_rel))
                if dist_to_robot <= 0.40:
                    # 0.40m 이내 초근접 시 안전을 위해 즉시 위협 통과 (긴급 충돌 마진)
                    t.threat = True
                    t.is_dynamic = True
                    t.is_static = False
                    t.reason = 'new-close'
                else:
                    # 0.40m 이상이면 정적 장애물(벽/모서리)일 가능성을 고려하여 속도 추정 수렴 대기 (오정지 차단)
                    t.threat = False
                    t.is_dynamic = False
                    t.is_static = False
                    t.reason = 'new-wait'
                continue

            # --- 2) 정적 장애물 판정 (절대 속도 < v_dynamic_thresh) ---
            if v_obs < self.v_dynamic_thresh:
                t.is_dynamic = False
                t.is_static = True
                t.threat = False
                t.reason = 'static'
                continue

            # --- 3) 동적 장애물 판정 (절대 속도 >= v_dynamic_thresh) ---
            t.is_dynamic = True
            t.is_static = False

            # 상대속도가 0에 근접한 경우 (동일 속도 주행 등)
            if vv < self.v_static ** 2:
                p_base = R_bw @ p_rel
                t.t_cpa = 0.0
                t.d_cpa = abs(float(p_base[1]))     # base 기준 횡방향 오프셋
                t.threat = (t.d_cpa <= self.dcpa_max)
                t.reason = 'dyn-parallel' if t.threat else 'dyn-clear'
                continue

            t_cpa = -float(p_rel @ v_rel) / vv
            if t_cpa < 0.0:
                # 이미 최근접점을 지나 멀어지는 경우
                t.t_cpa, t.d_cpa = t_cpa, float(np.linalg.norm(p_rel))
                t.threat = False
                t.reason = 'passed'
                continue

            d_cpa = float(np.linalg.norm(p_rel + v_rel * t_cpa))
            t.t_cpa, t.d_cpa = t_cpa, d_cpa
            t.threat = (t_cpa <= self.tcpa_max) and (d_cpa <= self.dcpa_max)
            t.reason = 'threat' if t.threat else 'clear'

    # ------------------------------------------------------------------ #
    #  4) /scan_threat 및 /scan_static 분리 발행                         #
    # ------------------------------------------------------------------ #
    def publish_scan(self, scan: LaserScan, tracks: Optional[List[Track]], passthrough_all: bool = False) -> None:
        """
        - /scan_threat: 동적 장애물 위협 빔 발행
        - /scan_static: 동적 장애물이 제외된 정적 장애물 빔 발행
        """
        out_threat = LaserScan()
        out_threat.header = scan.header
        out_threat.angle_min = scan.angle_min
        out_threat.angle_max = scan.angle_max
        out_threat.angle_increment = scan.angle_increment
        out_threat.time_increment = scan.time_increment
        out_threat.scan_time = scan.scan_time
        out_threat.range_min = scan.range_min
        out_threat.range_max = scan.range_max

        out_static = LaserScan()
        out_static.header = scan.header
        out_static.angle_min = scan.angle_min
        out_static.angle_max = scan.angle_max
        out_static.angle_increment = scan.angle_increment
        out_static.time_increment = scan.time_increment
        out_static.scan_time = scan.scan_time
        out_static.range_min = scan.range_min
        out_static.range_max = scan.range_max

        if passthrough_all:
            out_threat.ranges = list(scan.ranges)
            out_static.ranges = list(scan.ranges)
            self.pub_scan.publish(out_threat)
            self.pub_scan_static.publish(out_static)
            return

        n = len(scan.ranges)
        keep_threat = np.full(n, np.inf, dtype=np.float32)
        keep_static = np.asarray(scan.ranges, dtype=np.float32).copy()
        src = np.asarray(scan.ranges, dtype=np.float32)

        if tracks:
            for t in tracks:
                if t.beams:
                    idx = np.asarray(t.beams, dtype=np.int64)
                    # 1) 동적 위협 빔: /scan_threat 에 채움
                    if t.threat:
                        keep_threat[idx] = src[idx]
                    # 2) 움직이는 보행자 빔: /scan_static 에서 inf로 완전히 삭제 (코스트맵 우회 재탐색 차단)
                    if t.is_dynamic:
                        keep_static[idx] = np.inf

        out_threat.ranges = keep_threat.tolist()
        out_threat.intensities = []
        self.pub_scan.publish(out_threat)

        out_static.ranges = keep_static.tolist()
        out_static.intensities = []
        self.pub_scan_static.publish(out_static)

    # ------------------------------------------------------------------ #
    #  TF 변환 도우미                                                     #
    # ------------------------------------------------------------------ #
    def lookup_odom_tf(self, laser_frame: str, stamp: Any) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        try:
            tf_l = self.tf_buffer.lookup_transform(
                self.odom_frame, laser_frame, stamp, timeout=Duration(seconds=0.3))
        except Exception:
            try:
                tf_l = self.tf_buffer.lookup_transform(
                    self.odom_frame, laser_frame, Time())
            except Exception:
                try:
                    tf_l = self.tf_buffer.lookup_transform(
                        self.odom_frame, self.base_frame, Time())
                except Exception:
                    self.get_logger().warn(f'TF 대기 중: {self.odom_frame} <- {laser_frame}',
                                           throttle_duration_sec=3.0)
                    return None

        # 로봇 베이스 프레임 위치 탐색 (base_footprint 우선, 없으면 base_link)
        try:
            tf_b = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time())
        except Exception:
            try:
                tf_b = self.tf_buffer.lookup_transform(
                    self.odom_frame, 'base_link', Time())
            except Exception:
                tf_b = tf_l   # 최후 폴백

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
    def quat_to_yaw(q: Any) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def stamp_to_sec(stamp: Any) -> float:
        return float(stamp.sec + stamp.nanosec * 1e-9)

    # ------------------------------------------------------------------ #
    #  RViz 시각화 마커 발행                                              #
    # ------------------------------------------------------------------ #
    def publish_markers(self, stamp: Any) -> None:
        arr = MarkerArray()
        clr = Marker()
        clr.action = Marker.DELETEALL
        arr.markers.append(clr)

        for t in self.tracks:
            if t.threat:
                col = (1.0, 0.25, 0.15)   # 빨강: 위협
            elif t.reason == 'passed':
                col = (0.35, 0.35, 0.35)  # 회색: 통과 완료
            else:
                col = (0.25, 0.70, 1.00)  # 파랑: 비위협

            # 1) 원기둥 Bounding Box
            m = Marker()
            m.header.frame_id = self.odom_frame
            m.header.stamp = stamp
            m.ns = 'body'
            m.id = t.id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(t.X[0])
            m.pose.position.y = float(t.X[1])
            m.pose.position.z = 0.30
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.40
            m.scale.z = 0.60
            m.color.r, m.color.g, m.color.b = col
            m.color.a = 0.55
            arr.markers.append(m)

            # 2) 속도 방향 화살표
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

            # 3) 상태 및 CPA 정보 텍스트
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
def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicScanFilter()
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
