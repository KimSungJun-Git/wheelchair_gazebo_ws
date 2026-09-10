// 관제 대시보드 API 클라이언트 + rosbridge 연결.
//
// 이 파일이 Dashboard.html에서 가장 먼저 로드되므로, 모든 .jsx가 공유하는
// 전역 선언(React 훅, dict 접근자)도 여기서 한 번만 한다.
//   ⚠️ 각 <script type="text/babel">는 같은 전역 스코프에서 실행된다.
//      여러 파일에서 `const useState = ...`를 각각 선언하면
//      "Identifier 'useState' has already been declared"로 뒤 파일이 통째로 죽는다.
const { useState, useEffect, useMemo, useRef } = React;

// 백엔드/rosbridge는 이 페이지를 서빙한 호스트에 있다.
// (localhost 하드코딩 시 태블릿·다른 PC에서 열면 자기 자신을 찾아가 실패)
const API_BASE = `http://${window.location.hostname}:8090`;
const ROS_URL = `ws://${window.location.hostname}:9090`;

// ─── rosbridge 연결 ────────────────────────────────────────────────
// shell.jsx(SOS/안전 토스트), live.jsx(원격 정지), reports.jsx(로그 분리)가
// window.ros를 쓴다. 여기서 만들지 않으면 그 기능들이 전부 조용히 죽는다.
window.rosConnected = false;
function setRosState(ok) {
  window.rosConnected = ok;
  window.dispatchEvent(new CustomEvent("ros-state", { detail: ok }));
}
if (window.ROSLIB) {
  const ros = new ROSLIB.Ros({ url: ROS_URL });
  ros.on("connection", () => setRosState(true));
  ros.on("error", () => setRosState(false));
  ros.on("close", () => setRosState(false));
  window.ros = ros;
} else {
  console.error("ROSLIB 미로드 — Dashboard.html의 roslib 스크립트를 확인하세요.");
}

const REASON_LABEL = {
  imu_lost: "IMU 연결 끊김",
  imu_emergency: "IMU 비상(기울기·충격)",
  imu_기울기: "IMU 기울기 초과",
  ultrasonic_lost: "초음파 연결 끊김",
  lidar_lost: "라이다 연결 끊김",
  odom_lost: "오도메트리 끊김",
  localization_emergency: "위치 추정 실패",
  obstacle_front: "전방 장애물",
};
const ACTION_LABEL = {
  blocked: "비상정지",
  modified: "명령수정",
  sos: "SOS",
  allowed: "정상통과",
};
const ACTION_SEVERITY = {
  sos: "critical",
  blocked: "critical",
  modified: "warning",
  allowed: "info",
};

// 서버 상태 캐시. 무기한 캐싱하면 server.py를 나중에 켜도 새로고침 전까지
// 계속 오프라인으로 보이므로 짧은 TTL을 둔다.
const PROBE_TTL_MS = 5000;
let _serverUp = false;
let _probedAt = 0;

async function probeServer() {
  if (Date.now() - _probedAt < PROBE_TTL_MS) return _serverUp;
  try {
    const ctrl = new AbortController();
    const t = setTimeout(() => ctrl.abort(), 700);
    const r = await fetch(`${API_BASE}/api/health`, { signal: ctrl.signal });
    clearTimeout(t);
    _serverUp = r.ok;
  } catch (_e) {
    _serverUp = false;
  }
  _probedAt = Date.now();
  return _serverUp;
}

// 백엔드가 꺼져 있을 때 돌려줄 빈 응답.
// (예전에는 sample_data/*.json을 파싱해 가짜 화면을 만들었지만
//  그 폴더가 존재하지 않아 조용히 빈 화면만 나왔다 → 정직한 오프라인 상태로 대체)
const EMPTY_EVENTS = {
  window_hours: 24,
  by_day: [],
  by_hour: [],
  by_severity: {},
  reasons: [],
  total_events: 0,
  recent_count: 0,
  events: [],
  sessions: [],
  session_count: 0,
  avg_confidence: null,
};

async function getJson(path, fallback) {
  if (!(await probeServer())) return fallback;
  try {
    const r = await fetch(`${API_BASE}${path}`);
    if (!r.ok) return fallback;
    return await r.json();
  } catch (_e) {
    return fallback;
  }
}

const Api = {
  REASON_LABEL,
  ACTION_LABEL,
  ACTION_SEVERITY,
  API_BASE,

  async getMode() {
    return (await probeServer()) ? "live" : "offline";
  },

  getHealth() {
    return getJson("/api/health", {
      ok: false,
      mode: "offline",
      data_dir: null,
      exists: false,
      session_count: 0,
      latest: null,
    });
  },

  getReports() {
    return getJson("/api/reports", { reports: [], count: 0 });
  },

  getReport(id) {
    return getJson(`/api/report/${encodeURIComponent(id)}`, null);
  },

  getEvents(hours = 24) {
    return getJson(`/api/events?hours=${hours}`, { ...EMPTY_EVENTS, window_hours: hours });
  },

  getLive() {
    return getJson("/api/live", { connected: false, events: [], pose: null });
  },

  // 원격 정지: rosbridge가 붙어 있으면 직접 발행, 아니면 백엔드에 위임.
  // 둘 다 실패하면 반드시 false를 돌려준다 — 정지 안 됐는데 됐다고
  // 표시하는 것이 이 화면에서 제일 위험하다.
  async remoteStop() {
    if (window.rosConnected && window.ros && window.ROSLIB) {
      try {
        new window.ROSLIB.Topic({
          ros: window.ros,
          name: "/sos_trigger",
          messageType: "std_msgs/String",
        }).publish(new window.ROSLIB.Message({ data: "manual_stop" }));
        return true;
      } catch (_e) {
        /* 아래 HTTP 폴백으로 */
      }
    }
    try {
      const r = await fetch(`${API_BASE}/api/remote_stop`, { method: "POST" });
      return r.ok;
    } catch (_e) {
      return false;
    }
  },
};

window.Api = Api;
