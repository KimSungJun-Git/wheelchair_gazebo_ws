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
        self.get_logger().info("로그 수집 버퍼 활성화 완료")
        
    def create_new_session(self):
        session_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.session_filename = f"[주행로그]_{session_time}.json"
        self.session_filepath = os.path.join(self.save_dir, self.session_filename)
        self.session_event_counter = Counter()
        self.last_flushed_ts = 0.0

        # 세션 시작 마커 기록
        with open(self.session_filepath, 'w', encoding='utf-8') as f:
            marker = {
                "_session_marker": True,
                "event_type": "session_start",
                "timestamp": time.time(),
                "session_name": self.session_filename,
            }
            f.write(json.dumps(marker, ensure_ascii=False) + '\n')

        self.get_logger().info(f"새 세션 시작: {self.session_filename}")
        
    def rotation_callback(self, msg):
        self.get_logger().info("로그 분할(세션 회전) 요청 수신")
        self.save_snapshot(seconds=30, event_name=f"rotation_{int(time.time())}")
        self.create_new_session()
        
    def amcl_callback(self, msg):
        """맵 기준 절대 위치 (SLAM 사용 중)"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                         1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
        self.latest_pose = {
            "x": round(position.x, 3),
            "y": round(position.y, 3),
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
        """센서별 ok/lost 상태. 정상 센서는 제외하고 비정상 상태만 기록"""
        try:
            health_status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        unhealthy_sensors = {sensor: status for sensor, status in health_status.items() if status != 'ok'}
        self.latest_health = unhealthy_sensors or None

    def avoid_callback(self, msg):
        """장애물 진입 시 회피 가능 방향(좌/우/차단) 상태"""
        self.latest_avoid = msg.data if msg.data != 'none' else None

    def mode_callback(self, msg):
        """주행 모드(auto/manual) 상태 기록"""
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
        self.get_logger().error(f"SOS 이벤트 감지 ({msg.data}) - 데이터 스냅샷 저장 중")
        self.save_snapshot(seconds=30, event_name="sos")
        
    def log_callback(self, msg):
        try:
            event_data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        current_time = time.time()
        # 센서 이상·회피 판단·모드도 상태 튜플에 포함
        current_state = (event_data.get("action"), event_data.get("reason"),
                         tuple(sorted(self.latest_health)) if self.latest_health else (),
                         self.latest_avoid, self.latest_mode)
        is_state_changed = current_state != self.last_state

        # 동일 상태 지속 시 throttle 간격에 1건만 기록
        if not is_state_changed and current_time - self.last_log_time < self.throttle_sec:
            self.skipped_count += 1
            return

        event_data['timestamp'] = current_time
        event_data['pose'] = self.latest_pose.copy()
        event_data['velocity'] = self.latest_velocity.copy()
        event_data['zone'] = self.latest_zone
        event_data['mode'] = self.latest_mode
        if self.latest_health:
            event_data['sensor_health'] = self.latest_health
        if self.latest_avoid:
            event_data['avoidance'] = self.latest_avoid
        if self.skipped_count:
            event_data['repeated'] = self.skipped_count
        self.skipped_count = 0
        self.last_state = current_state
        self.last_log_time = current_time

        self.log_buffer.append(event_data)

        # 상태 변화 시에만 스냅샷 생성
        if not is_state_changed:
            return

        action = event_data.get("action", "unknown")
        if action == "modified":
            self.get_logger().warn("속도 명령 수정 감지 - 데이터 스냅샷 저장 중")
            self.save_snapshot(seconds=30, event_name="modified")
        elif action == "blocked":
            self.get_logger().error("비상 정지 감지 - 데이터 스냅샷 저장 중")
            self.save_snapshot(seconds=30, event_name="blocked")

    def heartbeat(self):
        """하트비트 간격 동안 추가 로그가 없을 경우 위치 및 상태 기록"""
        current_time = time.time()
        if current_time - self.last_log_time < self.heartbeat_sec:
            return
        self.last_log_time = current_time
        heartbeat_record = {
            "source": "log_collector",
            "action": "heartbeat",
            "reason": "",
            "timestamp": current_time,
            "pose": self.latest_pose.copy(),
            "velocity": self.latest_velocity.copy(),
            "zone": self.latest_zone,
            "mode": self.latest_mode,
        }
        if self.latest_health:
            heartbeat_record["sensor_health"] = self.latest_health
        if self.latest_avoid:
            heartbeat_record["avoidance"] = self.latest_avoid
        self.log_buffer.append(heartbeat_record)

    def save_snapshot(self, seconds, event_name):
        current_time = time.time()
        
        last_snapshot_time = self.last_snapshot_time.get(event_name, 0)
        if current_time - last_snapshot_time < self.snapshot_cooldown_sec:
            return
        self.last_snapshot_time[event_name] = current_time
        
        cutoff_timestamp = max(current_time - seconds, self.last_flushed_ts)
        snapshot = [log for log in self.log_buffer if log['timestamp'] > cutoff_timestamp]
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
                "timestamp": current_time,
                "snapshot_size": len(snapshot),
            }
            f.write(json.dumps(header, ensure_ascii=False) + '\n')
            for log in snapshot:
                f.write(json.dumps(log, ensure_ascii=False) + '\n')
        
        self.session_event_counter[event_name] += 1
        self.get_logger().info(
            f"[{label}] 스냅샷 저장: {self.session_filename} "
            f"(누적: {dict(self.session_event_counter)})"
        )
def main(args=None):
    rclpy.init(args=args)
    node = LogCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("로그 수집 종료")
    finally:
        # 사건이 한 번도 없으면 링 버퍼가 통째로 사라진다.
        # 종료 시점에 남은 걸 전부 흘려보내야 무사고 주행 궤적이 남는다.
        # (버퍼가 비어 있으면 쓰지 않는다 — web_ui launch의 빈 세션 정리를 살려둔다)
        if node.log_buffer:
            node.save_snapshot(seconds=float('inf'), event_name="session_end")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()