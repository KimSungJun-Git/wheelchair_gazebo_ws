#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import re
import time
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import uvicorn
import subprocess


WORKSPACE_DIR = Path(os.environ.get(
    "WHEELCHAIR_WS",
    os.path.expanduser("~/wheelchair_ws"),
))
DATA_DIR = Path(os.environ.get(
    "WHEELCHAIR_DATA_DIR",
    str(WORKSPACE_DIR / "driving_data"),
))
PORT = int(os.environ.get("PORT", "8090"))

REASON_LABEL: dict[str, str] = {
    "imu_lost": "IMU 연결 끊김",
    "imu_emergency": "IMU 비상(기울기·충격)",
    "imu_기울기": "IMU 기울기 초과",
    "ultrasonic_lost": "초음파 연결 끊김",
    "lidar_lost": "라이다 연결 끊김",
    "odom_lost": "오도메트리 끊김",
    "localization_emergency": "위치 추정 실패",
    "obstacle_front": "전방 장애물",
}
ACTION_SEVERITY = {
    "sos": "critical",
    "blocked": "critical",
    "modified": "warning",
    "allowed": "info",
}
DEDUP_WINDOW_SEC = 5.0



def list_session_files() -> list[Path]:
    if not DATA_DIR.exists():
        return []
    files = list(DATA_DIR.glob("*.json"))
    files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return files


def session_id(path: Path) -> str:
    return path.stem


def parse_jsonl(path: Path) -> tuple[list[dict], list[dict]]:
    """Return (markers, logs). Tolerates partial / malformed lines."""
    markers, logs = [], []
    try:
        with path.open("r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if obj.get("_event_marker"):
                    markers.append(obj)
                else:
                    logs.append(obj)
    except OSError:
        pass
    return markers, logs


def normalize_reason(raw: str) -> tuple[str, str]:
    raw = (raw or "").strip()
    if not raw:
        return "", ""
    first = raw.split(",")[0].strip()
    key = first.split(":")[0].strip()
    return key, raw


def dedup_messages(logs: list[dict]) -> list[dict]:
    seen: set = set()
    out: list[dict] = []
    for l in logs:
        k = (l.get("timestamp"), l.get("source"), l.get("action"), l.get("reason"))
        if k in seen:
            continue
        seen.add(k)
        out.append(l)
    return out


def extract_events(logs: list[dict]) -> list[dict]:
    events: list[dict] = []
    last_key: tuple | None = None
    last_timestamp = 0.0
    for log_entry in logs:
        action = log_entry.get("action")
        if action not in ("blocked", "modified", "sos"):
            continue
        reason_key, raw_reason = normalize_reason(log_entry.get("reason", ""))
        timestamp = float(log_entry.get("timestamp") or 0.0)
        event_signature = (action, reason_key)
        if event_signature == last_key and (timestamp - last_timestamp) < DEDUP_WINDOW_SEC:
            continue
        pose = log_entry.get("pose") or {}
        zone_raw = log_entry.get("zone") or ""
        events.append({
            "ts": timestamp,
            "action": action,
            "severity": ACTION_SEVERITY.get(action, "info"),
            "reason_key": reason_key,
            "reason_label": REASON_LABEL.get(reason_key, reason_key or "원인불명"),
            "reason_raw": raw_reason,
            "pose": {
                "x": pose.get("x"),
                "y": pose.get("y"),
                "yaw": pose.get("yaw"),
            },
            "zone": (zone_raw.split("|")[0].strip() or None) if zone_raw else None,
            "source": log_entry.get("source"),
        })
        last_key = event_signature
        last_timestamp = timestamp
    return events


def session_summary(path: Path, with_events: bool = False, with_raw: bool = False) -> dict:
    _markers, raw_logs = parse_jsonl(path)
    logs = dedup_messages(raw_logs)

    if not logs:
        return {
            "id": session_id(path),
            "filename": path.name,
            "started_at": None,
            "ended_at": None,
            "duration_sec": 0,
            "total": 0,
            "counts": {"blocked": 0, "modified": 0, "sos": 0, "allowed": 0},
        }

    timestamps = [float(ts) for entry in logs if (ts := entry.get("timestamp"))]
    started = min(timestamps) if timestamps else None
    ended = max(timestamps) if timestamps else None

    counts = Counter(entry.get("action") for entry in logs)
    events = extract_events(logs) if with_events else []
    reasons = Counter(e["reason_key"] for e in events if e["reason_key"])

    out: dict[str, Any] = {
        "id": session_id(path),
        "filename": path.name,
        "started_at": started,
        "ended_at": ended,
        "duration_sec": (ended - started) if started and ended else 0,
        "total": len(logs),
        "counts": {
            "blocked": counts.get("blocked", 0),
            "modified": counts.get("modified", 0),
            "sos": counts.get("sos", 0),
            "allowed": counts.get("allowed", 0),
        },
    }
    if with_events:
        out["events"] = events
        out["reasons"] = dict(reasons)
    if with_raw:
        out["raw_lines"] = logs[:2000]
        out["raw_truncated"] = len(logs) > 2000
    return out



app = FastAPI(title="Wheelchair Admin API", version="1.0.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
def health():
    files = list_session_files()
    return {
        "ok": True,
        "mode": "live",
        "data_dir": str(DATA_DIR),
        "exists": DATA_DIR.exists(),
        "session_count": len(files),
        "latest": files[0].name if files else None,
        "server_time": time.time(),
    }


@app.get("/api/reports")
def reports():
    files = list_session_files()
    out = [session_summary(p, with_events=False, with_raw=False) for p in files]
    return {"reports": out, "count": len(out)}


@app.get("/api/events")
def events(hours: int = 24):
    files = list_session_files()
    sessions: list[dict] = []
    all_events: list[dict] = []
    for p in files:
        s = session_summary(p, with_events=True, with_raw=False)
        sessions.append({
            "id": s["id"],
            "started_at": s["started_at"],
            "duration_sec": s["duration_sec"],
            "counts": s["counts"],
        })
        for e in s.get("events", []):
            all_events.append({**e, "session_id": s["id"]})

    cutoff = time.time() - hours * 3600
    recent = [e for e in all_events if e["ts"] >= cutoff]

    by_day: dict[str, dict] = {}
    by_hour: dict[str, dict] = {}
    reason_c: Counter = Counter()
    sev_c = Counter()

    for e in all_events:
        sev_c[e["severity"]] += 1
        if e.get("reason_key"):
            reason_c[e["reason_key"]] += 1
        d = datetime.fromtimestamp(e["ts"])
        day_key = d.strftime("%m/%d")
        hour_key = d.strftime("%m/%d %H:00")
        by_day.setdefault(day_key, {"day": day_key, "critical": 0, "warning": 0})
        if e["severity"] in ("critical", "warning"):
            by_day[day_key][e["severity"]] += 1
        by_hour.setdefault(hour_key, {"hour": hour_key, "critical": 0, "warning": 0, "info": 0})
        by_hour[hour_key][e["severity"]] = by_hour[hour_key].get(e["severity"], 0) + 1

    return {
        "window_hours": hours,
        "by_day": sorted(by_day.values(), key=lambda r: r["day"]),
        "by_hour": sorted(by_hour.values(), key=lambda r: r["hour"]),
        "by_severity": dict(sev_c),
        "reasons": [
            {"key": k, "label": REASON_LABEL.get(k, k), "count": v}
            for k, v in reason_c.most_common()
        ],
        "total_events": len(all_events),
        "recent_count": len(recent),
        "events": all_events,
        "sessions": sessions,
        "session_count": len(sessions),
    }


@app.get("/api/live")
def live():
    files = list_session_files()
    if not files:
        return {"connected": False, "events": [], "pose": None}
    p = files[0]
    s = session_summary(p, with_events=True, with_raw=True)
    last = (s.get("raw_lines") or [{}])[-1]
    pose = last.get("pose") or {}
    zone_raw = last.get("zone") or ""
    last_ts = last.get("timestamp") or 0.0
    connected = (time.time() - last_ts) < 30.0 
    return {
        "connected": connected,
        "session_id": s["id"],
        "pose": {"x": pose.get("x"), "y": pose.get("y"), "yaw": pose.get("yaw")},
        "velocity": last.get("velocity") or {},
        "zone_raw": zone_raw,
        "zone": (zone_raw.split("|")[0].strip() or None) if zone_raw else None,
        "mode": last.get("mode") or "auto",
        "events": list(reversed(s.get("events", [])[-30:])),
        "last_log_ts": last_ts,
    }


@app.get("/")
def root():
    return JSONResponse({
        "service": "wheelchair-admin-api",
        "data_dir": str(DATA_DIR),
        "endpoints": [
            "/api/health",
            "/api/reports",
            "/api/events?hours=24",
            "/api/live",
            "/api/remote_stop",
        ],
    })




@app.post("/api/remote_stop")
def remote_stop():
    """대시보드 원격 정지 — rosbridge가 끊겼을 때의 폴백 경로.

    발행이 실제로 성공했을 때만 ok를 돌려준다. Popen으로 던져놓고
    무조건 성공을 반환하면 정지되지 않았는데 정지됐다고 표시된다.
    """
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "ros2 topic pub --once /sos_trigger std_msgs/String \"{data: 'manual_stop'}\""
    )
    try:
        result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=10)
    except subprocess.TimeoutExpired:
        raise HTTPException(504, "정지 명령 전송 시간 초과 — 휠체어가 계속 주행 중일 수 있습니다.")
    except Exception as e:
        raise HTTPException(500, f"명령 전송 실패: {str(e)}")
    if result.returncode != 0:
        raise HTTPException(500, f"명령 전송 실패: {result.stderr[-300:]}")
    return {"ok": True, "message": "정지 명령 전송 완료"}

def main(args=None):
    global DATA_DIR
    parser = argparse.ArgumentParser(description="Wheelchair Admin Dashboard API")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--data-dir", default=str(DATA_DIR))
    parser.add_argument("--reload", action="store_true")
    parsed, _ = parser.parse_known_args(args)

    DATA_DIR = Path(os.path.expanduser(parsed.data_dir))
    print(f"[AdminServer] data_dir = {DATA_DIR} ({'OK' if DATA_DIR.exists() else '없음'})")
    print(f"[AdminServer] API address: http://{parsed.host}:{parsed.port}/api/health")
    
    app_target = "wheelchair_ui.admin_server:app" if parsed.reload else app
    uvicorn.run(app_target, host=parsed.host, port=parsed.port, reload=parsed.reload)


if __name__ == "__main__":
    main()