# wheelchair_robot_ui

휠체어 로봇의 **웹 UI 두 개**와 **관제 API 서버**를 담은 단일 패키지.
(구 `wheelchair_admin_dashboard`가 이 패키지로 병합됨)

## 구성

| 구성요소 | 경로 | 설명 |
|---|---|---|
| 랜딩 | `web/index.html` | 두 사이트로 가는 입구 |
| 탑승자 UI | `web/Wheelchair_SLAM_UI.html` + `web/js/` | 목적지 이동·수동 조작·SOS. rosbridge 직접 연동 |
| 관제 대시보드 | `web/admin/Dashboard.html` + `web/admin/src/` | 주행 통계·AI 보고서·실시간 모니터·원격 정지 |
| 관제 API | `wheelchair_robot_ui/admin_server.py` | FastAPI. `driving_data/*.json`을 읽어 집계 (기본 8090) |

## 실행

```bash
colcon build --packages-select wheelchair_robot_ui --symlink-install
source install/setup.bash
ros2 launch wheelchair_robot_ui web_ui.launch.py
```

launch 하나가 rosbridge(9090) + 정적 서버(8000) + 관제 API(8090) + log_collector를 함께 띄운다.

```
http://<host>:8000/                        랜딩
http://<host>:8000/Wheelchair_SLAM_UI.html 탑승자 UI
http://<host>:8000/admin/Dashboard.html    관제 대시보드   (로그인 smac / 0000)
```

`<host>`는 localhost가 아니어도 된다 — 프론트가 `window.location.hostname`으로
rosbridge와 API를 찾으므로 태블릿·다른 PC에서 그대로 열린다.

관제 API만 따로 띄우려면:

```bash
ros2 run wheelchair_robot_ui admin_server           # 또는 --port 8090 --data-dir <경로>
```

환경변수 `WHEELCHAIR_WS`, `WHEELCHAIR_DATA_DIR`, `PORT`로도 경로/포트를 바꿀 수 있다.

## 의존

- `rosbridge_server` (rosdep)
- `wheelchair_robot_ai` (log_collector_node, agent_analyzer)
- `fastapi`, `uvicorn` — pip 설치 필요: `pip3 install fastapi uvicorn`

## 다국어

두 UI 모두 한/영을 지원하며 사전 위치가 다르다.

- 탑승자 UI: `web/js/config.js`의 `TXT`
  `activity.jsx`·`map.jsx`가 **로드 시점에** `TXT`를 읽으므로 사전은 반드시
  가장 먼저 로드되는 `config.js`에 있어야 한다. 언어 전환은 페이지 리로드 방식.
- 관제 대시보드: `web/admin/src/shell.jsx`의 `window.dict` (ko/en). 리로드 없이 즉시 전환.

⚠️ 번역하면 안 되는 한글이 있다: `app.jsx`의 `/current_zone` 파싱
(`'비상정지'`, `'위험구역'`)은 `safety_stop_node`가 발행하는 값 그 자체다.

## 웹 파일을 편집할 때

`Dashboard.html`과 `Wheelchair_SLAM_UI.html`은 `<script type="text/babel">`로
여러 파일을 불러오고, **이 파일들은 모두 같은 전역 스코프에서 실행된다.**
두 파일에서 같은 이름을 `const`/`let`으로 선언하면 뒤 파일이 통째로 죽는다.

- 탑승자 UI: `React.useState(...)`처럼 매번 `React.`를 붙여 쓴다.
- 관제 대시보드: 훅은 `api.jsx`에서 한 번만 구조분해한다. 다른 파일에서 재선언 금지.
