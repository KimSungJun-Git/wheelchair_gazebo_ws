import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from datetime import datetime
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math

import time
from geometry_msgs.msg import Twist
from collections import deque, Counter

class LogCollectorNode(Node):
    def __init__(self):
        super().__init__('log_collector_node')
        self.log_buffer = deque(maxlen=1000)
        self.declare_parameter('save_dir', os.path.expanduser('~/wheelchair_ws/driving_data'))
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.latest_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.latest_velocity = {"linear": 0.0, "angular": 0.0}
        self.latest_zone = None
        self.latest_health = None
        self.latest_avoid = None
        self.latest_mode = None

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(String, '/current_zone', self.zone_callback, 10)
        self.create_subscription(String, '/sensor_health', self.health_callback, 10)
        self.create_subscription(String, '/avoidance_direction', self.avoid_callback, 10)
        self.create_subscription(String, '/robot_mode', self.mode_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.create_subscription(String, '/safety_action', self.log_callback, 10)
        self.create_subscription(String, '/sos_trigger', self.sos_callback, 10) 
        
        self.create_subscription(String, '/request_log_rotation', self.rotation_callback, 10)

        self.last_snapshot_time = {}

        # /safety_action은 타이머가 아니라 /cmd_vel_nav 콜백에 물려 있어
        # 0Hz(정지 중) ~ 40Hz(주행 중)로 발행 주기가 요동친다.
        # 상태가 바뀌는 순간(edge)은 항상 남기고, 같은 상태 반복은
        # throttle 간격에 1건만 남겨 시간축을 균일하게 만든다.
        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('snapshot_cooldown_sec', 5.0)
        self.throttle_sec = self.get_parameter('log_throttle_sec').get_parameter_value().double_value
        self.snapshot_cooldown_sec = self.get_parameter('snapshot_cooldown_sec').get_parameter_value().double_value

        self.last_state = None      # (action, reason)
        self.last_log_time = 0.0
        self.skipped_count = 0      # throttle로 버린 동일 상태 건수
        self.last_flushed_ts = 0.0  # 디스크에 이미 기록된 마지막 로그 시각

        # safety_stop_node가 죽거나 수동 조작 중이면 /safety_action이 아예 안 온다.
        # 그 구간의 위치·속도를 채워 궤적이 끊기지 않게 한다.
        self.declare_parameter('heartbeat_sec', 1.0)
        self.heartbeat_sec = self.get_parameter('heartbeat_sec').get_parameter_value().double_value
        if self.heartbeat_sec > 0:
            self.create_timer(self.heartbeat_sec, self.heartbeat)

        # 초기 세션 생성
        self.create_new_session()
        self.get_logger().info(f"🟢 [스마트 블랙박스] 링 버퍼 활성화 대기 중...")
        
    def create_new_session(self):
        session_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.session_filename = f"[주행로그]_{session_time}.json"
        self.session_filepath = os.path.join(self.save_dir, self.session_filename)
        self.session_event_counter = Counter()
        self.last_flushed_ts = 0.0   # 새 파일이므로 직전 30초 맥락을 다시 채운다

        # 세션 시작 마커를 즉시 디스크에 기록 → 파일이 항상 존재 보장
        with open(self.session_filepath, 'w', encoding='utf-8') as f:
            marker = {
                "_session_marker": True,
                "event_type": "session_start",
                "timestamp": time.time(),
                "session_name": self.session_filename,
            }
            f.write(json.dumps(marker, ensure_ascii=False) + '\n')

        self.get_logger().info(f"🟢 새 세션 시작 → {self.session_filename}")
        
    def rotation_callback(self, msg):
        self.get_logger().info("🔄 UI 요약 요청 수신")
        # event_name을 매번 unique하게 → 쿨다운 우회
        self.save_snapshot(seconds=30, event_name=f"summary_{int(time.time())}")
        self.create_new_session()
        
    def amcl_callback(self, msg):
        """맵 기준 절대 위치 (SLAM 사용 중)"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest_pose = {
            "x": round(p.x, 3),
            "y": round(p.y, 3),
            "yaw": round(yaw, 3),
        }
                
    def cmd_callback(self, msg):
        """현재 명령 속도 = 로봇이 실제로 움직이려는 속도"""
        self.latest_velocity = {
            "linear": round(msg.linear.x, 3),
            "angular": round(msg.angular.z, 3),
        }
    
    
    def zone_callback(self, msg):
        """현재 안전 구역 갱신"""
        self.latest_zone = msg.data

    def health_callback(self, msg):
        """센서별 ok/lost 상태. 정상 센서는 버리고 이상한 것만 남긴다."""
        try:
            health = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        bad = {k: v for k, v in health.items() if v != 'ok'}
        self.latest_health = bad or None

    def avoid_callback(self, msg):
        """장애물 진입 시 좌/우/blocked 판단. 정지가 불가피했는지 판별하는 근거."""
        self.latest_avoid = msg.data if msg.data != 'none' else None

    def mode_callback(self, msg):
        """auto/manual — 사건이 자율주행 실패인지 수동 조작 중 발생인지 가른다."""
        self.latest_mode = msg.data


    def sos_callback(self, msg):
        """SOS는 단순 문자열로 옴 (위치 분실 등)"""
        data = {
            "source": "sos_trigger",
            "action": "sos",
            "reason": msg.data,
            "timestamp": time.time(),
        }
        self.log_buffer.append(data)
        self.get_logger().error(f"🆘 SOS 감지({msg.data})! 전후 30초 데이터 캡처 중...")
        self.save_snapshot(seconds=30, event_name="sos")
        
    def log_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        current_time = time.time()
        # 센서 이상·회피 판단·모드도 상태에 포함 — 바뀌는 순간을 throttle이 삼키지 않게
        state = (data.get("action"), data.get("reason"),
                 tuple(sorted(self.latest_health)) if self.latest_health else (),
                 self.latest_avoid, self.latest_mode)
        is_edge = state != self.last_state

        # 같은 상태가 계속 흘러들어오면 throttle 간격에 1건만 남긴다.
        if not is_edge and current_time - self.last_log_time < self.throttle_sec:
            self.skipped_count += 1
            return

        data['timestamp'] = current_time
        data['pose'] = self.latest_pose.copy()
        data['velocity'] = self.latest_velocity.copy()
        data['zone'] = self.latest_zone
        data['mode'] = self.latest_mode      # admin_server /api/live가 읽는 필드
        if self.latest_health:
            data['sensor_health'] = self.latest_health   # 이상 센서만
        if self.latest_avoid:
            data['avoidance'] = self.latest_avoid
        if self.skipped_count:
            data['repeated'] = self.skipped_count   # 생략된 동일 상태 건수
        self.skipped_count = 0
        self.last_state = state
        self.last_log_time = current_time

        self.log_buffer.append(data)

        # 스냅샷은 상태가 '바뀐 순간'에만 — 지속 중엔 다시 뜨지 않는다.
        if not is_edge:
            return

        action = data.get("action", "unknown")
        if action == "modified":
            self.get_logger().warn("⚠️ 명령 수정 감지! 전후 30초 데이터 캡처 중...")
            self.save_snapshot(seconds=30, event_name="modified")
        elif action == "blocked":
            self.get_logger().error("🚨 비상정지 감지! 전후 30초 데이터 캡처 중...")
            self.save_snapshot(seconds=30, event_name="blocked")

    def heartbeat(self):
        """직전 heartbeat 간격 안에 아무 로그도 안 쌓였을 때만 위치를 남긴다(중복 기록 방지)."""
        now = time.time()
        if now - self.last_log_time < self.heartbeat_sec:
            return
        self.last_log_time = now
        record = {
            "source": "log_collector",
            "action": "heartbeat",
            "reason": "",
            "timestamp": now,
            "pose": self.latest_pose.copy(),
            "velocity": self.latest_velocity.copy(),
            "zone": self.latest_zone,
            "mode": self.latest_mode,
        }
        if self.latest_health:
            record["sensor_health"] = self.latest_health
        if self.latest_avoid:
            record["avoidance"] = self.latest_avoid
        self.log_buffer.append(record)

    def save_snapshot(self, seconds, event_name):
        now = time.time()
        
        last_time = self.last_snapshot_time.get(event_name, 0)
        if now - last_time < self.snapshot_cooldown_sec:
            return
        self.last_snapshot_time[event_name] = now
        
        # 이미 디스크에 쓴 로그는 다시 쓰지 않는다.
        # (기존에는 이벤트마다 과거 30초를 통째로 재기록해 파일의 8.7%가 중복이었다)
        cutoff_time = max(now - seconds, self.last_flushed_ts)
        snapshot = [log for log in self.log_buffer if log['timestamp'] > cutoff_time]
        if snapshot:
            self.last_flushed_ts = snapshot[-1]['timestamp']

        label_map = {"blocked": "비상정지", "modified": "명령수정", "sos": "SOS",
                     "session_end": "세션종료"}
        label = label_map.get(event_name, event_name)
        
        with open(self.session_filepath, 'a', encoding='utf-8') as f:
            header = {
                "_event_marker": True,
                "event_type": event_name,
                "event_label": label,
                "timestamp": now,
                "snapshot_size": len(snapshot),
            }
            f.write(json.dumps(header, ensure_ascii=False) + '\n')
            for log in snapshot:
                f.write(json.dumps(log, ensure_ascii=False) + '\n')
        
        self.session_event_counter[event_name] += 1
        self.get_logger().info(
            f"💾 [{label}] 캡처 → {self.session_filename} "
            f"(누적: {dict(self.session_event_counter)})"
        )
def main(args=None):
    rclpy.init(args=args)
    node = LogCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("수집 종료.")
    finally:
        # 사건이 한 번도 없으면 링 버퍼가 통째로 사라진다.
        # 종료 시점에 남은 걸 전부 흘려보내야 무사고 주행 궤적이 남는다.
        # (버퍼가 비어 있으면 쓰지 않는다 — web_ui launch의 빈 세션 정리를 살려둔다)
        if node.log_buffer:
            node.save_snapshot(seconds=float('inf'), event_name="session_end")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()