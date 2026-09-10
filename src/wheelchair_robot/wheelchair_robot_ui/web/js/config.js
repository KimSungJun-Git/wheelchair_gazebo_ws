// ─── 0. ROS / 센서 / 맵 설정 ──────────────────────────────────────
// rosbridge와 관제 API는 이 페이지를 서빙한 호스트에 있다.
// (localhost로 박아두면 태블릿·다른 PC에서 열었을 때 자기 자신을 찾아가 실패)
const ROS_CONFIG = {
  url: `ws://${window.location.hostname}:9090`,  // rosbridge_websocket 주소
};
const ADMIN_API = `http://${window.location.hostname}:8090`;

const DIST_THRESHOLDS = {
  red: 0.5,
  orange: 0.8,
  white: 1.0,
};

const DEFAULT_MAP_CONFIG = {
  origin_x: -10.0,
  origin_y: -10.0,
  pixels_per_meter: 30,
  svg_width: 800,
  svg_height: 480,
};

const LIDAR_SECTORS = [
  { key: 'front',      from: -20, to:  20 },
  { key: 'frontLeft',  from:  20, to:  60 },
  { key: 'left',       from:  60, to: 100 },
  { key: 'frontRight', from: -60, to: -20 },
  { key: 'right',      from: -100, to: -60 },
];

// ─── 1. 블루 테마 토큰 적용 ────────────────────────
const TOKENS = {
  color: {
    bg: '#F8FAFC', surface: '#FFFFFF', surfaceAlt: '#BFDBFE', surfaceDark: '#191970',
    primary: '#0047AB', primaryDark: '#002F6C', primarySoft: '#BFDBFE', accent: '#3B82F6',
    ink: '#002F6C', inkMuted: '#10367D', inkFaint: '#64748B',
    success: '#10A571', successSoft: '#D8F1E5', warn: '#E6A100', warnSoft: '#FEF08A', danger: '#E5484D', dangerSoft: '#FEE2E2',
    mapBg: '#F1F5F9', mapWall: '#002F6C', mapFloor: '#E2E8F0', mapPath: '#3B82F6', mapPathGhost: '#BFDBFE', mapObstacle: '#E5484D', mapRobot: '#191970',
    line: '#BFDBFE', lineStrong: '#3B82F6',
  },
  radius: { sm: 8, md: 14, lg: 20, xl: 28, pill: 999 },
  shadow: { sm: '0 1px 2px rgba(0,47,108,0.06)', md: '0 4px 14px rgba(0,47,108,0.1)', lg: '0 10px 30px rgba(0,47,108,0.15)' },
  font: { sans: `'Pretendard', sans-serif` },
};

// ─── 2. 다국어(i18n) ──────────────────────────────────────────────
// ⚠️ 반드시 이 파일(가장 먼저 로드)에 있어야 한다.
//    activity.jsx / map.jsx가 최상위 const 초기화 시점에 TXT를 읽으므로
//    screens.jsx에 두면 TDZ 에러로 죽는다.
const IS_ENG = localStorage.getItem('wheelchair_lang') === 'en';
const toggleLanguage = () => {
  localStorage.setItem('wheelchair_lang', IS_ENG ? 'ko' : 'en');
  window.location.reload();
};
const KE = (ko, en) => (IS_ENG ? en : ko);

const TXT = {
  // ── 홈 화면 ──
  hello: KE('안녕하세요', 'Hello'),
  wheresToday: KE('오늘은 어디로 갈까요?', 'Where to today?'),
  caregiver: KE('보호자 연결됨', 'Caregiver Connected'),
  searchBtn: KE('병동·시설 검색하기', 'Search Facilities'),
  goHomeBtn: KE('대기소로 이동', 'Go to Home Base'),
  favorites: KE('즐겨찾기', 'Favorites'),
  myRoom: KE('내 방 (302호)', 'My Room (302)'),
  rehab: KE('재활치료실 (2층)', 'Rehab Room (2F)'),
  liveMap: KE('실시간 주변 지도', 'Live Map'),
  sos: 'SOS',
  endSession: KE('✨ 오늘의 주행 요약', "✨ Today's Summary"),

  // ── 검색 화면 ──
  searchTitle: KE('목적지 검색', 'Search'),
  recDest: KE('추천 목적지', 'Recommended'),
  er: KE('응급실(Emergency)', 'Emergency Room'),
  erDesc: KE('1층 서관 · 120m', '1F West Wing · 120m'),
  r101: KE('101호', 'Room 101'),
  r101Desc: KE('2층 동관 · 65m', '2F East Wing · 65m'),
  distEta: KE('거리 · ETA', 'Distance · ETA'),
  startRoute: KE('경로 시작', 'Start Route'),

  // ── 주행 화면 ──
  moving: KE('목적지로 이동 중', 'Moving to Destination'),
  manual: KE('수동', 'Manual'),
  auto: KE('자율', 'Auto'),
  stop: KE('정지', 'Stop'),
  manualControl: KE('수동조작', 'Manual'),
  stopCurrentDrive: KE('현재 주행 정지', 'Stop current drive'),
  switchToManualTitle: KE('수동 조작으로 전환', 'Switch to manual control'),
  homeTitle: KE('홈으로', 'Home'),

  // ── 알림(Alert) 화면 ──
  recAction: KE('권장 조치', 'Recommended Action'),
  resumeRoute: KE('자율 주행 재개', 'Resume Route'),
  switchToManual: KE('수동 모드 전환', 'Switch to Manual'),
  goHomeBase: KE('홈으로 이동', 'Return Home'),
  subResume: KE('경로 재개', 'Resume Route'),
  subSwitch: KE('제어 전환', 'Switch Control'),
  subDock: KE('대기소로', 'Go to Dock'),

  // ── 조이스틱 화면 ──
  joyTitle: KE('수동 주행 모드', 'Manual Drive Mode'),
  joySub: KE('화면의 큰 버튼을 눌러 직접 조작하세요.', 'Use the buttons below to control.'),
  returnAuto: KE('자율 모드로 복귀', 'Return to Auto Mode'),
  miniMap: KE('주변 미니맵', 'Local Minimap'),
  manualActive: KE('수동 제어 중', 'Manual Control Active'),
  fwd: KE('누르고 있으면 전진', 'Hold to move forward'),
  left: KE('왼쪽 틀기', 'Turn Left'),
  rev: KE('누르고 있으면 후진', 'Hold to move backward'),
  right: KE('오른쪽 틀기', 'Turn Right'),

  // ── 통신 배너 ──
  rosDown: KE(
    '⚠️ 로봇 통신 끊김 — 화면 정보가 실시간이 아닐 수 있습니다',
    '⚠️ Robot link lost — displayed data may not be live'
  ),

  // ── 장애물 근접 패널 (map.jsx) ──
  obsTitle: KE('주변 장애물', 'PROXIMITY'),
  obsDetected: KE('감지됨', 'DETECTED'),
  obsSafe: KE('안전', 'CLEAR'),
  obsClear: KE('주변 양호', 'All clear'),
  obsApproach: (dir) => KE(`${dir} 접근`, `${dir} approaching`),
  dirLabel: {
    front: KE('전방', 'Front'),
    frontLeft: KE('좌전방', 'Front-L'),
    frontRight: KE('우전방', 'Front-R'),
    left: KE('좌측', 'Left'),
    right: KE('우측', 'Right'),
  },

  // ── 활동 로그 패널 (activity.jsx) ──
  statusActive: KE('진행 중', 'Active'),
  statusCompleted: KE('완료', 'Done'),
  statusCancelled: KE('취소됨', 'Cancelled'),
  statusFailed: KE('실패', 'Failed'),
  actPanelTitle: KE('시스템 활동', 'System Activity'),
  actIdle: KE('대기 중', 'Idle'),
  actIdleShort: KE('대기', 'Idle'),
  actOpenLog: KE('로그 열기', 'Open log'),
  actRunning: (n) => KE(`${n}건 진행 중`, `${n} running`),
  actEmpty: KE('아직 기록된 동작이 없습니다.', 'No activity recorded yet.'),
  actStopAll: KE('모두 중지', 'Stop all'),
  actClear: KE('지우기', 'Clear'),
  tabAll: KE('전체', 'All'),
  tabActive: KE('진행', 'Active'),
  tabCompleted: KE('완료', 'Done'),
  tabCancelled: KE('취소', 'Cancelled'),
  tabFailed: KE('실패', 'Failed'),
  confirm: KE('확인', 'Confirm'),
  cancel: KE('취소', 'Cancel'),

  // ── 활동 라벨 / 노트 (app.jsx) ──
  actConnecting: KE('로봇 연결 시도', 'Connecting to robot'),
  actConnected: KE('연결 완료', 'Connected'),
  actNoRos: KE('UI 테스트 모드', 'UI test mode (no ROS)'),
  actNavTo: (label) => KE(`자율 주행: ${label}`, `Autonomous drive: ${label}`),
  actGoHomeBase: KE('대기소 자동 귀환', 'Auto return to home base'),
  actResumeDetour: KE('자율 주행 재개 (우회 경로)', 'Resuming drive (detour)'),
  actResumeTo: (label) => KE(`자율 주행 재개: ${label}`, `Resuming drive: ${label}`),
  noteArrived: KE('목적지 도착', 'Arrived'),
  noteNavFailed: KE('주행 실패', 'Drive failed'),
  noteNavCancelled: KE('주행 취소', 'Drive cancelled'),
  noteUserCancel: KE('사용자 취소', 'Cancelled by user'),
  noteGoHome: KE('홈으로 복귀', 'Returning home'),
  noteNewDest: KE('새 목적지 지정', 'New destination set'),
  noteToHomeBase: KE('대기소 귀환으로 변경', 'Switched to home-base return'),
  noteUserStop: KE('사용자 정지', 'Stopped by user'),
  noteEmergency: KE('긴급 정지', 'Emergency stop'),
  noteToManual: KE('수동 모드 전환', 'Switched to manual'),
  noteResume: KE('재개', 'Resumed'),
  noteManualTakeover: KE('수동 조작 전환', 'Manual takeover'),

  // ── 이벤트 로그 (app.jsx) ──
  logModeSwitch: (m) => KE(`주행 모드 전환: ${m}`, `Drive mode: ${m}`),
  logMapApplied: (w, h, res) =>
    KE(`맵 정보 자동 적용: ${w}×${h}px, ${res}m/px`,
       `Map applied: ${w}×${h}px, ${res}m/px`),
  logObstacle: KE('⚠️ 전방 장애물 감지 - 자율 주행 중단',
                  '⚠️ Front obstacle — autonomous drive paused'),
  logObstacleCleared: KE('✅ 장애물 해소', '✅ Obstacle cleared'),
  logKeepout: KE('🚫 금지구역 진입', '🚫 Entered keepout zone'),
  logNavFailed: KE('❌ 자율 주행 실패', '❌ Autonomous drive failed'),
  logSensorOk: (name) => KE(`✅ ${name} 센서 정상`, `✅ ${name} sensor OK`),
  logSensorNever: (name) => KE(`🔌 ${name} 센서 응답 없음`, `🔌 ${name} sensor not responding`),
  logSensorLost: (name, v) => KE(`🔌 ${name} 센서 끊김 (${v})`, `🔌 ${name} sensor lost (${v})`),
  logImuEmergency: KE('🚨 IMU 비상 (기울기/충격 감지)', '🚨 IMU emergency (tilt/impact)'),
  logImuOk: KE('✅ IMU 정상 복귀', '✅ IMU back to normal'),
  logLocLostGlobal: KE('🚨 위치 추적 분실 - 글로벌 재인식 시도',
                       '🚨 Localization lost — global relocalization'),
  logLocRecovered: KE('✅ 위치 추적 복구', '✅ Localization recovered'),
  logLocUncertain: KE('⚠️ 위치 추적 불확실', '⚠️ Localization uncertain'),
  logLocLost: KE('🚨 위치 추적 분실', '🚨 Localization lost'),
  logLocOk: KE('✅ 위치 추적 정상', '✅ Localization OK'),
  logSos: (msg) => `🆘 SOS: ${msg}`,
  logAvoid: (label) => `↪️ ${label}`,
  logBlocked: (reason) => KE(`🛑 명령 차단 (${reason})`, `🛑 Command blocked (${reason})`),
  logModified: (reason) => KE(`✂️ 속도 제한 (${reason})`, `✂️ Speed limited (${reason})`),
  reasonEmergency: KE('비상', 'emergency'),
  reasonObstacle: KE('장애물', 'obstacle'),
  logZoneEmergency: KE('🛑 비상정지 구역 진입', '🛑 Entered e-stop zone'),
  logZoneDanger: KE('⚠️ 위험구역 진입', '⚠️ Entered danger zone'),
  logZoneNormal: KE('✅ 일반구역 복귀', '✅ Back in normal zone'),
  logDriveStopped: KE('주행 정지', 'Drive stopped'),
  logSosCall: KE('🛑 SOS 긴급 호출', '🛑 SOS emergency call'),
  logManualEnter: KE('수동 주행 모드 진입', 'Entered manual mode'),
  logAutoEnter: KE('자율 주행 모드 진입', 'Entered autonomous mode'),
  logAutoReturn: KE('자율 모드로 복귀', 'Back to autonomous mode'),
  logEndRequest: KE('🛑 이용 종료 요청', '🛑 Session end requested'),
  logAiStarted: KE('🤖 AI 분석 시작됨', '🤖 AI analysis started'),
  logAiFailed: KE('⚠️ 분석 요청 실패', '⚠️ Analysis request failed'),
  logServerDown: KE('⚠️ 서버 연결 실패 — 관제 서버 확인',
                    '⚠️ Server unreachable — check the admin server'),
  avoidLeft: KE('왼쪽으로 우회', 'Detour left'),
  avoidRight: KE('오른쪽으로 우회', 'Detour right'),
  avoidBlocked: KE('양쪽 막힘 - 회피 불가', 'Both sides blocked — cannot avoid'),

  // ── 확인 대화상자 (app.jsx) ──
  askNavTitle: (label) => KE(`${label}(으)로 이동할까요?`, `Drive to ${label}?`),
  askNavMsg: KE('확인을 누르면 휠체어가 즉시 자율 주행을 시작합니다.',
                'The wheelchair will start driving autonomously right away.'),
  askNavOk: KE('네, 이동합니다', 'Yes, go'),
  askHomeTitle: KE('대기소로 자동 귀환할까요?', 'Return to the home base?'),
  askHomeMsg: KE('확인을 누르면 휠체어가 대기소까지 자율 주행으로 돌아갑니다.',
                 'The wheelchair will drive itself back to the home base.'),
  askHomeOk: KE('네, 귀환합니다', 'Yes, return'),
  askEndTitle: KE('오늘 이용을 종료할까요?', 'End today’s session?'),
  askEndMsg: KE('주행 기록을 AI가 분석합니다 (1~2분 소요).',
                'The AI will analyze the driving log (takes 1–2 min).'),
  askEndOk: KE('네, 종료합니다', 'Yes, end'),
  askSosTitle: KE('SOS 호출을 보낼까요?', 'Send an SOS call?'),
  askSosMsg: KE('보호자와 관제실에 즉시 알림이 전달됩니다.',
                'Your caregiver and the control room will be alerted immediately.'),
  askSosOk: KE('네, 호출합니다', 'Yes, call'),
  askNo: KE('아니오', 'No'),
  askCancelNavHomeTitle: KE('홈으로 돌아갈까요?', 'Go back home?'),
  askCancelNavTitle: KE('주행을 취소할까요?', 'Cancel the drive?'),
  askCancelNavHomeMsg: KE('진행 중인 자율 주행이 취소되고 홈 화면으로 이동합니다.',
                          'The current drive will be cancelled and you’ll return to the home screen.'),
  askCancelNavMsg: KE('진행 중인 자율 주행이 취소됩니다.', 'The current drive will be cancelled.'),
  askCancelOk: KE('네, 취소합니다', 'Yes, cancel'),
  askKeepDriving: KE('계속 주행', 'Keep driving'),

  destFallback: KE('목적지', 'Destination'),
  homeBase: KE('대기소', 'Home base'),
};

const DEST_LABELS = {
  emergency: KE('응급실(Emergency)', 'Emergency Room'),
  room_101: KE('101호', 'Room 101'),
  room_102: KE('102호', 'Room 102'),
  home: KE('대기소', 'Home base'),
};

const SENSOR_LABELS = {
  lidar: KE('라이다', 'LiDAR'),
  imu: 'IMU',
  odom: KE('모터', 'Motor'),
  ultrasonic_front: KE('초음파(전)', 'Ultrasonic (F)'),
  ultrasonic_left: KE('초음파(좌)', 'Ultrasonic (L)'),
  ultrasonic_right: KE('초음파(우)', 'Ultrasonic (R)'),
};
