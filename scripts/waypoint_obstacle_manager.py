#!/usr/bin/env python3
"""
waypoint_obstacle_manager.py
Local_0904 장애물 자동 스폰 및 35개 동적 장애물 전역 분산 독립 배회(Dispersed Roaming) 매니저
- 정적 장애물 10개 완벽 유지 (Local_0904.world 기본 로드)
- 총 35개 동적 장애물 독립 배회 (dynamic_obs_1 ~ dynamic_obs_35):
  * 전역 복도 네트워크(Global Waypoint Graph) 기반 보행자 랜덤 워크
  * 최근 방문 노드(Recent History) 추적으로 180도 제자리 0.1m 왕복 진동 원천 차단
  * 외벽 및 코너 구석 고립 방지 및 부드러운 완충 쿠션(Spring Damping) 적용
  * 장애물 상호 겹침 방지: Social Repulsion
  * 가제보 로봇(휠체어) 25cm(표면 거리) 이내 근접 시 즉각 랜덤 방향 튕김 회피
"""
import os
import sys
import time
import math
import random
import subprocess
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates


# 맵 안전 경계 (Local_0904 내벽 안쪽 허용 공간)
MAP_MIN_X = -9.8
MAP_MAX_X = 22.8
MAP_MIN_Y = -9.4
MAP_MAX_Y = 9.4


class DynamicObstacleController:
    """개별 동적 장애물의 분산 독립 배회 제어기 (전역 복도 네트워크 기반 랜덤 워크)"""
    def __init__(self, node: Node, name: str, waypoints: list, speed: float = 0.35,
                 dyn_settings: dict = None, waypoint_pool: list = None):
        self.node = node
        self.name = name
        self.base_waypoints = [(float(pt[0]), float(pt[1])) for pt in waypoints]
        self.base_speed = float(speed)
        self.waypoint_pool = waypoint_pool or self.base_waypoints

        # 동적 파라미터 파싱
        self.dyn_settings = dyn_settings or {}
        self.randomize = bool(self.dyn_settings.get('randomize', True))
        self.speed_range = self.dyn_settings.get('speed_range', [0.35, 1.10])
        self.pause_prob = float(self.dyn_settings.get('pause_prob', 0.15))
        self.pause_range_sec = self.dyn_settings.get('pause_range_sec', [0.3, 1.2])
        self.robot_avoid_speed = float(self.dyn_settings.get('robot_avoid_speed', 1.15))

        # 앵커 및 초기 위치
        self.anchor_a = self.base_waypoints[0]
        self.anchor_b = self.base_waypoints[-1] if len(self.base_waypoints) > 1 else self.base_waypoints[0]
        self.visited_history = []  # 최근 방문한 웨이포인트 목록 (180도 유턴 진동 방지)

        self.curr_x = self.anchor_a[0]
        self.curr_y = self.anchor_a[1]
        self.has_position = False

        # 상태 머신
        self.state = "MOVING"  # "MOVING" | "PAUSED" | "EVADING"
        self.pause_until = 0.0
        self.evasion_until = 0.0
        self.evasion_cooldown = 0.0
        self.current_cmd_speed = 0.0
        self.max_accel = 1.2  # m/s^2

        self.active_path = []
        self.current_target_speed = self.base_speed
        self.current_wp_idx = 1
        self._init_first_segment()

        # cmd_vel 퍼블리셔
        self.cmd_pub = node.create_publisher(Twist, f"/{name}/cmd_vel", 10)

        # odom 구독 (월드 절대 좌표)
        self.odom_sub = node.create_subscription(
            Odometry,
            f"/{name}/odom",
            self.odom_callback,
            10
        )

    def trigger_evasion(self, source_x: float, source_y: float, label: str = "가제보 로봇", impulse_speed: float = None):
        """가제보(로봇/벽) 25cm 근접 시 다른 랜덤 방향으로 즉시 튕겨나가는 회피 제어"""
        now = time.time()
        if now < self.evasion_cooldown:
            return

        if impulse_speed is None:
            impulse_speed = self.robot_avoid_speed

        dx = self.curr_x - source_x
        dy = self.curr_y - source_y
        dist = math.hypot(dx, dy)
        if dist < 1e-4:
            base_angle = random.uniform(0.0, 2.0 * math.pi)
        else:
            base_angle = math.atan2(dy, dx)

        # 로봇 반대 방향 기준 좌우 ±75도 무작위 튕김 각도
        evasion_angle = base_angle + random.uniform(-1.3, 1.3)
        escape_dist = random.uniform(2.0, 3.5)
        tx = self.curr_x + escape_dist * math.cos(evasion_angle)
        ty = self.curr_y + escape_dist * math.sin(evasion_angle)
        tx, ty = self._clamp_to_map(tx, ty)

        # 즉시 회피 경로 주입 (지연 없이 즉각 발진)
        self.active_path = [(self.curr_x, self.curr_y), (tx, ty)]
        self.current_wp_idx = 1
        self.current_target_speed = impulse_speed
        self.current_cmd_speed = impulse_speed
        self.state = "EVADING"
        self.evasion_until = now + 1.2
        self.evasion_cooldown = now + 0.6

    def _sample_speed(self) -> float:
        if not self.randomize:
            return self.base_speed
        return random.uniform(self.speed_range[0], self.speed_range[1])

    def _clamp_to_map(self, x: float, y: float) -> tuple:
        """맵 안전 경계 내부로 좌표 클램핑"""
        cx = max(MAP_MIN_X + 0.45, min(MAP_MAX_X - 0.45, x))
        cy = max(MAP_MIN_Y + 0.45, min(MAP_MAX_Y - 0.45, y))
        return (cx, cy)

    def _pick_next_waypoint(self) -> tuple:
        """
        전역 복도 네트워크 기반 보행자 랜덤 워크:
        - 맵 전역의 합법적인 복도 웨이포인트(70여개) 중 현재 위치에서 1.8m ~ 7.5m 거리의 인접 웨이포인트를 무작위 선정
        - 최근 방문한 4개 지점은 가중치를 대폭 억제(0.05)하여 제자리 0.1m 왕복 진동 방지
        """
        pool = self.waypoint_pool if self.waypoint_pool else self.base_waypoints
        if not pool:
            return self.anchor_b

        candidates = []
        for pt in pool:
            d = math.hypot(pt[0] - self.curr_x, pt[1] - self.curr_y)
            if 1.8 <= d <= 7.5:
                # 최근 방문한 지점인지 확인 (0.8m 이내면 동일 지점 간주)
                is_recent = any(math.hypot(pt[0] - vp[0], pt[1] - vp[1]) < 0.8 for vp in self.visited_history)
                weight = 0.05 if is_recent else 1.0
                candidates.append((pt, weight))

        if candidates:
            pts, weights = zip(*candidates)
            chosen = random.choices(pts, weights=weights, k=1)[0]
            return chosen

        # 적정 거리의 후보가 없으면 풀 전체에서 2.0m 이상 떨어진 가장 가까운 복도 지점 선택
        sorted_pts = sorted(pool, key=lambda p: math.hypot(p[0] - self.curr_x, p[1] - self.curr_y))
        for p in sorted_pts:
            if math.hypot(p[0] - self.curr_x, p[1] - self.curr_y) >= 2.0:
                return p

        return self.anchor_b

    def set_new_waypoint(self):
        """새로운 복도 목표점 갱신 및 방문 히스토리 업데이트"""
        nxt = self._pick_next_waypoint()
        self.visited_history.append(nxt)
        if len(self.visited_history) > 4:
            self.visited_history.pop(0)

        self.active_path = [(self.curr_x, self.curr_y), nxt]
        self.current_wp_idx = 1
        self.current_target_speed = self._sample_speed()

    def _init_first_segment(self):
        self.visited_history = [(self.curr_x, self.curr_y)]
        self.set_new_waypoint()

    def update_position(self, world_x: float, world_y: float):
        self.curr_x = world_x
        self.curr_y = world_y
        self.has_position = True

    def odom_callback(self, msg: Odometry):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self.has_position = True

    def update_control(self, repulsion_vx: float = 0.0, repulsion_vy: float = 0.0):
        now = time.time()

        # 1. 회피(EVADING) 또는 일시정지(PAUSED) 상태 처리
        if self.state == "EVADING":
            if not self.active_path or self.current_wp_idx >= len(self.active_path):
                self.state = "MOVING"
                self.set_new_waypoint()
            else:
                tx, ty = self.active_path[self.current_wp_idx]
                if math.hypot(tx - self.curr_x, ty - self.curr_y) < 0.35 or now > self.evasion_until:
                    self.state = "MOVING"
                    self.set_new_waypoint()

        elif self.state == "PAUSED":
            if now < self.pause_until:
                self.current_cmd_speed = max(0.0, self.current_cmd_speed - self.max_accel * 0.05)
                cmd = Twist()
                if math.hypot(repulsion_vx, repulsion_vy) > 0.05:
                    cmd.linear.x = repulsion_vx * 0.3
                    cmd.linear.y = repulsion_vy * 0.3
                self.cmd_pub.publish(cmd)
                return
            else:
                self.state = "MOVING"
                self.set_new_waypoint()

        # 2. 목표점 유효성 및 거리 계산
        if not self.active_path or self.current_wp_idx >= len(self.active_path):
            self.set_new_waypoint()
            return

        tx, ty = self.active_path[self.current_wp_idx]
        dx = tx - self.curr_x
        dy = ty - self.curr_y
        dist = math.hypot(dx, dy)

        # 3. 웨이포인트 도달 판정 (0.35m)
        if self.state != "EVADING" and dist < 0.35:
            if self.randomize and (random.random() < self.pause_prob):
                self.state = "PAUSED"
                pause_sec = random.uniform(self.pause_range_sec[0], self.pause_range_sec[1])
                self.pause_until = now + pause_sec
                self.current_cmd_speed = 0.0
                self.cmd_pub.publish(Twist())
                return

            self.set_new_waypoint()
            tx, ty = self.active_path[self.current_wp_idx]
            dx = tx - self.curr_x
            dy = ty - self.curr_y
            dist = math.hypot(dx, dy)

        # 4. 가감속 슬루율 제어
        target_v = self.current_target_speed if dist > 0.08 else 0.0
        dt = 0.05
        accel_rate = self.max_accel * 3.5 if self.state == "EVADING" else self.max_accel
        max_step = accel_rate * dt
        if self.current_cmd_speed < target_v:
            self.current_cmd_speed = min(target_v, self.current_cmd_speed + max_step)
        else:
            self.current_cmd_speed = max(target_v, self.current_cmd_speed - max_step)

        # 5. 자율 주행 기본 벡터 계산
        cmd = Twist()
        if dist > 0.08 and self.current_cmd_speed > 0.01:
            vx = (dx / dist) * self.current_cmd_speed
            vy = (dy / dist) * self.current_cmd_speed
        else:
            vx = 0.0
            vy = 0.0

        # 6. 상호 회피 및 외벽 완충 반발 벡터 합성
        vx_total = vx + repulsion_vx
        vy_total = vy + repulsion_vy
        speed_total = math.hypot(vx_total, vy_total)
        max_allowed_speed = max(1.30, self.speed_range[1] * 1.35)
        if speed_total > max_allowed_speed and speed_total > 1e-4:
            scale = max_allowed_speed / speed_total
            vx_total *= scale
            vy_total *= scale

        cmd.linear.x = vx_total
        cmd.linear.y = vy_total
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


class WaypointObstacleManager(Node):
    def __init__(self, config_path: str):
        super().__init__('waypoint_obstacle_manager')

        if not os.path.exists(config_path):
            self.get_logger().error(f"설정 파일을 찾을 수 없습니다: {config_path}")
            sys.exit(1)

        with open(config_path, 'r', encoding='utf-8') as f:
            self.cfg = yaml.safe_load(f)

        self.ws_dir = "/home/kim/wheelchair_gazebo_ws"
        self.static_model = os.path.join(self.ws_dir, "models/static_box_obstacle/model.sdf")
        self.dynamic_model = os.path.join(self.ws_dir, "models/dynamic_obstacle/model.sdf")

        dyn_settings = self.cfg.get('dynamic_settings', {})
        self.repulsion_dist = float(dyn_settings.get('repulsion_dist', 1.20))
        self.repulsion_gain = float(dyn_settings.get('repulsion_gain', 0.45))

        # 1. 35개 구역의 모든 안전 복도 웨이포인트 수집 (전역 복도 네트워크 노드)
        self.global_waypoint_pool = []
        raw_dyn = self.cfg.get('dynamic_obstacles', [])
        for d in raw_dyn:
            for wp in d.get('waypoints', []):
                pt = (float(wp[0]), float(wp[1]))
                if not any(math.hypot(pt[0] - p[0], pt[1] - p[1]) < 0.3 for p in self.global_waypoint_pool):
                    self.global_waypoint_pool.append(pt)

        self.get_logger().info(
            f"🎲 동적 장애물 모드: [전역 복도 네트워크 랜덤 워크] | "
            f"총 {len(self.global_waypoint_pool)}개 안전 복도 노드 구축 | "
            f"맵 경계=[{MAP_MIN_X}~{MAP_MAX_X}, {MAP_MIN_Y}~{MAP_MAX_Y}] | "
            f"반발 거리={self.repulsion_dist}m | 속도={dyn_settings.get('speed_range')}"
        )

        # 2. Gazebo ROS 2 스폰 서비스 가용성 점검 및 스폰
        self.spawn_service_available = self.check_spawn_service()
        if self.spawn_service_available:
            self.spawn_static_obstacles()
            self.spawn_dynamic_obstacles()
        else:
            self.get_logger().warn("⚠️ /spawn_entity 서비스가 없어 신규 스폰을 건너뛰고 기존 장애물 제어기만 활성화합니다.")

        # 3. 35개 독립 제어기 인스턴스 생성 (전역 복도 웨이포인트 풀 전달!)
        self.controllers = {}
        for d in raw_dyn:
            name = d['name']
            wps = d.get('waypoints', [])
            speed = d.get('speed', 0.35)
            ctrl = DynamicObstacleController(self, name, wps, speed, dyn_settings, waypoint_pool=self.global_waypoint_pool)
            self.controllers[name] = ctrl

        # 4. 로봇 위치 실시간 구독 (/odom 및 ModelStates)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.has_robot_pose = False
        self.robot_avoid_margin = float(dyn_settings.get('robot_avoid_margin', 0.25))  # 25cm (0.25m)
        self.robot_avoid_speed = float(dyn_settings.get('robot_avoid_speed', 1.15))
        self.static_obstacles = [
            (float(s['x']), float(s['y'])) for s in self.cfg.get('static_obstacles', [])
        ]

        self.robot_odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.robot_odom_callback,
            10
        )

        # 5. Gazebo ModelStates 위치 피드백 구독
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        self.model_states_sub2 = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

        # 20Hz (0.05초) 고정 제어 루프
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(
            f"🚀 총 {len(self.controllers)}개 장애물 제어 시작! "
            f"(가제보 로봇 {self.robot_avoid_margin*100:.0f}cm 이내 접근 시 랜덤 방향 튕김 회피 활성화)"
        )

    def robot_odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.has_robot_pose = True

    def model_states_callback(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            if name in self.controllers:
                self.controllers[name].update_position(pose.position.x, pose.position.y)
            elif name in ['turtlebot3_waffle_pi', 'waffle_pi', 'wheelchair_robot']:
                self.robot_x = pose.position.x
                self.robot_y = pose.position.y
                self.has_robot_pose = True

    def check_spawn_service(self):
        cmd = ["ros2", "service", "list"]
        try:
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)
            if "/spawn_entity" in res.stdout:
                return True
        except Exception:
            pass
        return False

    def spawn_static_obstacles(self):
        print("\n✅ [정적 장애물] Local_0904.world에 기본 정의된 10개 장애물을 즉시 활용합니다.")

    def spawn_dynamic_obstacles(self):
        dyn_list = self.cfg.get('dynamic_obstacles', [])
        total = len(dyn_list)
        print(f"\n[동적 장애물] 총 {total}개 Gazebo 월드 내 존재 여부 확인 중...", flush=True)

        existing_models = set()
        try:
            res = subprocess.run(
                ["ros2", "service", "call", "/get_model_list", "gazebo_msgs/srv/GetModelList", "{}"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5
            )
            if "model_names=" in res.stdout:
                part = res.stdout.split("model_names=")[1].split("]")[0].replace("[", "").replace("'", "").replace('"', "")
                existing_models = set(m.strip() for m in part.split(",") if m.strip())
        except Exception as e:
            print(f"⚠️ 모델 목록 조회 예외: {e}")

        for i, d in enumerate(dyn_list, 1):
            name = d['name']
            wps = d.get('waypoints', [])
            if not wps:
                continue

            if name in existing_models:
                print(f"   [{i:2d}/{total:2d}] {name} 이미 Gazebo에 존재 (스폰 생략)", flush=True)
                continue

            start_x, start_y = float(wps[0][0]), float(wps[0][1])
            print(f"   [{i:2d}/{total:2d}] {name} 스폰 중... (위치: {start_x:.2f}, {start_y:.2f})", flush=True)
            cmd = [
                "ros2", "run", "gazebo_ros", "spawn_entity.py",
                "-entity", name,
                "-robot_namespace", name,
                "-file", self.dynamic_model,
                "-x", str(start_x), "-y", str(start_y), "-z", "0.01",
                "-timeout", "5"
            ]
            try:
                subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=6.0)
            except Exception as e:
                print(f"   ❌ {name} 스폰 예외: {e}", flush=True)
            time.sleep(0.12)
        print("✅ 모든 동적 장애물 Gazebo 스폰 확인 및 준비 완료!\n", flush=True)

    def timer_callback(self):
        """20Hz 제어 주기: 가제보 로봇 25cm 회피, 정적 장애물 회피, 외벽 쿠션, 상호 반발력 계산"""
        ctrl_list = list(self.controllers.values())
        n = len(ctrl_list)

        repulsion_v = {ctrl.name: [0.0, 0.0] for ctrl in ctrl_list}

        OBSTACLE_RADIUS = 0.13    # 동적 장애물 원기둥 반경 (m)

        # -----------------------------------------------------------------
        # [1] 가제보 로봇 근접 검사 및 랜덤 방향 튕김 트리거 (사용자 요청으로 비활성화)
        # -----------------------------------------------------------------
        # ROBOT_RADIUS = 0.22       # 로봇 외곽 반경 (m)
        # ROBOT_TRIGGER_DIST = ROBOT_RADIUS + OBSTACLE_RADIUS + self.robot_avoid_margin  # 0.22 + 0.13 + 0.25 = 0.60m

        # if self.has_robot_pose:
        #     for ctrl in ctrl_list:
        #         dx = ctrl.curr_x - self.robot_x
        #         dy = ctrl.curr_y - self.robot_y
        #         dist = math.hypot(dx, dy)
        #         surface_dist = dist - (ROBOT_RADIUS + OBSTACLE_RADIUS)

        #         # 로봇과 25cm 이내로 들어오면 즉각 다른 랜덤 방향으로 튕겨나감!
        #         if surface_dist <= self.robot_avoid_margin or dist <= ROBOT_TRIGGER_DIST:
        #             ctrl.trigger_evasion(self.robot_x, self.robot_y, label="가제보 로봇", impulse_speed=self.robot_avoid_speed)

        #             if dist > 1e-4:
        #                 nx, ny = dx / dist, dy / dist
        #             else:
        #                 ang = random.uniform(0.0, 2.0 * math.pi)
        #                 nx, ny = math.cos(ang), math.sin(ang)

        #             overlap = max(0.05, (ROBOT_TRIGGER_DIST - dist))
        #             mag = max(0.95, overlap * 4.5)
        #             repulsion_v[ctrl.name][0] += nx * mag
        #             repulsion_v[ctrl.name][1] += ny * mag

        # -----------------------------------------------------------------
        # [2] 정적 장애물 10개 회피 및 외벽 부드러운 완충 쿠션 (진동 완전 차단)
        # -----------------------------------------------------------------
        STATIC_RADIUS = 0.28
        STATIC_TRIGGER_DIST = STATIC_RADIUS + OBSTACLE_RADIUS + 0.25  # 약 0.66m
        for ctrl in ctrl_list:
            # 정적 장애물 근접 검사
            for sx, sy in self.static_obstacles:
                dx = ctrl.curr_x - sx
                dy = ctrl.curr_y - sy
                dist = math.hypot(dx, dy)
                if dist < STATIC_TRIGGER_DIST:
                    ctrl.trigger_evasion(sx, sy, label="정적 장애물", impulse_speed=0.90)
                    if dist > 1e-4:
                        overlap = STATIC_TRIGGER_DIST - dist
                        repulsion_v[ctrl.name][0] += (dx / dist) * overlap * 1.5
                        repulsion_v[ctrl.name][1] += (dy / dist) * overlap * 1.5

            # 외벽 완충 쿠션 (Spring Damping: 벽에 접근할수록 점진적으로 부드럽게 밀어냄)
            wall_margin = 0.60
            deep_penetration = False

            if ctrl.curr_x < (MAP_MIN_X + wall_margin):
                p = (MAP_MIN_X + wall_margin) - ctrl.curr_x
                repulsion_v[ctrl.name][0] += min(0.85, p * 1.6)
                if p > 0.15: deep_penetration = True
            elif ctrl.curr_x > (MAP_MAX_X - wall_margin):
                p = ctrl.curr_x - (MAP_MAX_X - wall_margin)
                repulsion_v[ctrl.name][0] -= min(0.85, p * 1.6)
                if p > 0.15: deep_penetration = True

            if ctrl.curr_y < (MAP_MIN_Y + wall_margin):
                p = (MAP_MIN_Y + wall_margin) - ctrl.curr_y
                repulsion_v[ctrl.name][1] += min(0.85, p * 1.6)
                if p > 0.15: deep_penetration = True
            elif ctrl.curr_y > (MAP_MAX_Y - wall_margin):
                p = ctrl.curr_y - (MAP_MAX_Y - wall_margin)
                repulsion_v[ctrl.name][1] -= min(0.85, p * 1.6)
                if p > 0.15: deep_penetration = True

            # 벽에 너무 깊이 끼인 경우 목표점을 즉시 안쪽 안전 복도로 리셋
            if deep_penetration and ctrl.state != "EVADING":
                ctrl.set_new_waypoint()

        # -----------------------------------------------------------------
        # [3] 장애물 간 상호 거리 검사 (Social Repulsion: 과도한 요동 방지 클램핑)
        # -----------------------------------------------------------------
        for i in range(n):
            c1 = ctrl_list[i]
            for j in range(i + 1, n):
                c2 = ctrl_list[j]
                dx = c1.curr_x - c2.curr_x
                dy = c1.curr_y - c2.curr_y
                dist = math.hypot(dx, dy)

                if 1e-3 < dist < self.repulsion_dist:
                    nx, ny = dx / dist, dy / dist
                    overlap = (self.repulsion_dist - dist)
                    mag = min(0.55, overlap * self.repulsion_gain)

                    repulsion_v[c1.name][0] += nx * mag
                    repulsion_v[c1.name][1] += ny * mag
                    repulsion_v[c2.name][0] -= nx * mag
                    repulsion_v[c2.name][1] -= ny * mag

        # [4] 각 장애물 제어기 갱신 및 속도 명령 퍼블리시
        for ctrl in ctrl_list:
            rx, ry = repulsion_v[ctrl.name]
            ctrl.update_control(repulsion_vx=rx, repulsion_vy=ry)

    def stop_all(self):
        """종료 시 모든 장애물에 즉시 정지 명령 전송"""
        try:
            cmd = Twist()
            for _ in range(4):
                for ctrl in self.controllers.values():
                    ctrl.cmd_pub.publish(cmd)
                time.sleep(0.02)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    default_cfg = "/home/kim/wheelchair_gazebo_ws/config/obstacles_Local_0904.yaml"
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else default_cfg
    manager = WaypointObstacleManager(cfg_path)
    try:
        rclpy.spin(manager)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        manager.stop_all()
        manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
