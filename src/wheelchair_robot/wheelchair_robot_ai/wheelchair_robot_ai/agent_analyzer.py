#!/usr/bin/env python3
# agent_analyzer.py
import os
import json
import glob
from collections import Counter
from datetime import datetime

LOG_DIR = os.path.expanduser('~/wheelchair_ws/driving_data')

ACTION_LABEL: dict[str, str] = {
    "blocked":  "🚨 비상정지",
    "modified": "⚠️ 명령수정",
    "sos":      "🆘 SOS",
    "allowed":  "✅ 정상통과",
    "idle":     "⏸️ 대기(명령없음)",
    "heartbeat": "💓 상태기록",
}

REASON_LABEL: dict[str, str] = {
    "imu_lost":               "IMU 연결 끊김",
    "imu_emergency":          "IMU 비상(기울기·충격)",
    "imu_기울기":              "IMU 기울기 초과",
    "ultrasonic_lost":        "초음파 연결 끊김",
    "lidar_lost":             "라이다 연결 끊김",
    "odom_lost":              "오도메트리 끊김",
    "localization_emergency": "위치 추정 실패",
    "obstacle_front":         "전방 장애물",
}

# 정지가 불가피했는지, 피할 길이 있었는데 멈춘 건지 구분
AVOID_LABEL: dict[str, str] = {
    "left":    " (좌측 회피 가능했음)",
    "right":   " (우측 회피 가능했음)",
    "blocked": " (양측 모두 막힘 — 정지 불가피)",
}

# 자율주행 실패인지, 수동 조작 중 발생인지
MODE_LABEL: dict[str, str] = {
    "auto":   " [자율]",
    "manual": " [수동]",
}

DEDUP_WINDOW_SEC = 5.0

def split_reasons(reason_str):
    """콤마로 합쳐진 reason을 분리. 'key:상세값' 형태면 key만 추출."""
    if not reason_str:
        return tuple()

    normalized = []
    for r in reason_str.split(","):
        r = r.strip()
        if not r:
            continue

        key = r.split(":")[0].strip()
        if key:
            normalized.append(key)

    return tuple(sorted(set(normalized)))


def deduplicate_messages(all_logs):
    """
    log_collector가 스냅샷마다 과거 30초 데이터를 중복 저장하는 문제 해결.
    timestamp + source + action + reason 조합이 같으면 같은 메시지로 간주.
    """
    seen = set()
    deduped = []
    for log in all_logs:
        key = (
            log.get("timestamp"),
            log.get("source"),
            log.get("action"),
            log.get("reason"),
        )
        if key in seen:
            continue
        seen.add(key)
        deduped.append(log)
    return deduped


def deduplicate_events(event_logs):
    """같은 종류의 사건이 5초 이내 연속되면 1건으로 묶음"""
    unique = []
    last_key = None
    last_time = 0
    
    for log in event_logs:
        action = log.get("action")
        reasons = split_reasons(log.get("reason", ""))
        ts = log.get("timestamp", 0)
        
        key = (action, reasons)
        if key == last_key and (ts - last_time) < DEDUP_WINDOW_SEC:
            continue
        
        pose = log.get("pose") or {}
        unique.append({
            "action": action,
            "reasons": list(reasons),
            "x": pose.get("x"),
            "y": pose.get("y"),
            "timestamp": ts,
            "avoidance": log.get("avoidance"),
            "mode": log.get("mode"),
            "raw": log,
        })
        last_key = key
        last_time = ts
    
    return unique


def build_summary(all_logs):
    """전체 로그에서 룰 기반 통계 요약 생성 (LLM 거치지 않음)"""
    total = len(all_logs)
    
    action_counter = Counter(log.get("action", "unknown") for log in all_logs)
    
    timestamps = [log.get("timestamp") for log in all_logs if log.get("timestamp")]
    duration = ""
    if timestamps:
        start = datetime.fromtimestamp(min(timestamps)).strftime("%H:%M:%S")
        end = datetime.fromtimestamp(max(timestamps)).strftime("%H:%M:%S")
        duration = f" ({start} ~ {end})"
    
    # 사건 단위 디듀플
    event_logs = [l for l in all_logs if l.get("action") in ("blocked", "modified", "sos")]
    unique_events = deduplicate_events(event_logs)
    
    # 센서별 이상 카운트
    sensor_counter: Counter = Counter()
    for evt in unique_events:
        for r in evt["reasons"]:
            sensor_counter[r] += 1
    
    lines = []
    lines_health: list[tuple] = []
    lines.append(f"## 📋 이벤트 요약{duration}")
    lines.append("")
    lines.append(f"- **총 메시지: {total}개** (중복 제거 후)")
    
    for action, count in action_counter.most_common():
        label = ACTION_LABEL.get(action, action)
        lines.append(f"- {label}: {count}개")
    
    if sensor_counter:
        lines.append("")
        lines.append(f"### 🔧 감지된 이상 (실제 사건 {len(unique_events)}건)")
        for reason, count in sensor_counter.most_common():
            label = REASON_LABEL.get(reason, reason)
            lines.append(f"- {label} (`{reason}`): {count}건")
    
    # 센서 끊김 구간 — "왜 멈췄나"를 이 로그만으로 답할 수 있게
    health_spans: dict[str, list] = {}
    for log in all_logs:
        ts = log.get("timestamp")
        for sensor, status in (log.get("sensor_health") or {}).items():
            span = health_spans.get(sensor)
            if span and ts - span[1] < DEDUP_WINDOW_SEC:
                span[1] = ts
            else:
                health_spans[sensor] = [ts, ts]
                lines_health.append((ts, sensor, status))

    if lines_health:
        lines.append("")
        lines.append(f"### 🩺 센서 이상 구간 ({len(lines_health)}회)")
        for ts, sensor, status in lines_health:
            time_str = datetime.fromtimestamp(ts).strftime("%H:%M:%S")
            # ultrasonic_front/left/right → ultrasonic_lost 로 라벨을 재사용
            base = sensor.rsplit("_", 1)[0] if sensor.startswith("ultrasonic") else sensor
            label = REASON_LABEL.get(f"{base}_lost", sensor)
            lines.append(f"- {time_str} {label} (`{sensor}`) — {status}")

    if unique_events:
        lines.append("")
        lines.append(f"### 📍 사건 발생 위치 (전체 {len(unique_events)}건)")
        for evt in unique_events:
            label = ACTION_LABEL.get(evt["action"], evt["action"])
            time_str = datetime.fromtimestamp(evt["timestamp"]).strftime("%H:%M:%S")
            reason_labels: list[str] = [
                REASON_LABEL.get(r, r) for r in evt["reasons"]
                if isinstance(r, str) and r
            ]
            reason_str = ", ".join(reason_labels) if reason_labels else "원인불명"
            xy = f"x={evt['x']}, y={evt['y']}" if evt['x'] is not None else "위치불명"
            avoid = AVOID_LABEL.get(evt.get("avoidance"), "")
            mode = MODE_LABEL.get(evt.get("mode"), "")
            lines.append(f"- {time_str} {label}{mode} @ `{xy}` — {reason_str}{avoid}")
    
    return "\n".join(lines), unique_events


def read_logs(latest_file):
    """로그 파일 한 개를 읽어 (요약 마크다운, 사건 목록) 반환"""
    raw_logs_list = []

    with open(latest_file, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                log = json.loads(line)
                if log.get("_event_marker"):
                    continue
                raw_logs_list.append(log)
            except json.JSONDecodeError:
                pass
    
    if not raw_logs_list:
        return "## 📋 이벤트 요약\n\n- 수집된 로그가 없습니다.", []

    before_count = len(raw_logs_list)
    all_logs = deduplicate_messages(raw_logs_list)
    after_count = len(all_logs)

    if before_count != after_count:
        print(f"\033[90m🔁 중복 메시지 {before_count - after_count}개 제거 ({before_count} → {after_count})\033[0m")

    return build_summary(all_logs)


def main(args=None):
    log_files = glob.glob(f'{LOG_DIR}/*.json')
    if not log_files:
        print("\033[93m\n[알림] 분석할 로그 파일이 없습니다.\033[0m")
        return

    latest_file = max(log_files, key=os.path.getmtime)
    print(f"\033[90m📄 분석 대상: {os.path.basename(latest_file)}\033[0m")

    try:
        summary, unique_events = read_logs(latest_file)
    except OSError as e:
        print(f"\033[91m⚠️ 로그 읽기 실패: {e}\033[0m")
        return

    print("\033[96m" + summary + "\033[0m\n")
    if not unique_events:
        print("\033[92m🟢 [완벽] 분석할 사건이 없습니다. 안전 주행!\033[0m")

    report_path = latest_file.replace('.json', '_report.md')
    try:
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(summary + "\n")
        print(f"\033[92m💾 분석 리포트 저장: {os.path.basename(report_path)}\033[0m")
    except OSError as e:
        print(f"\033[91m⚠️ 리포트 저장 실패: {e}\033[0m")


if __name__ == "__main__":
    main()