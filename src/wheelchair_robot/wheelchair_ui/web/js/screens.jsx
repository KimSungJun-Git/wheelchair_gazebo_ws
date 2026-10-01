// ─── 4. Screens (Home, Search, Nav, Alert, Joystick) ──────────────
// i18n(IS_ENG / toggleLanguage / TXT)은 config.js에 있다.
// activity.jsx·map.jsx가 로드 시점에 TXT를 읽기 때문에 그쪽이 먼저여야 한다.

function HomeScreen({ onSearch, onGoHome, onSOS, onEndSession }) {
  return (
    <Frame>
      <StatusBar />
      <div style={{ padding: '24px 28px 16px', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <div><div style={{ fontSize: 18, color: TOKENS.color.inkMuted, fontWeight: 700 }}>{TXT.hello}, {IS_ENG ? 'Minji Kim' : '김민지'}님</div><div style={{ fontSize: 36, fontWeight: 800, letterSpacing: -0.5, marginTop: 6, color: TOKENS.color.primaryDark }}>{TXT.wheresToday}</div></div>
        <div style={{ display: 'flex', gap: 10, alignItems: 'center' }}>
          <button onClick={toggleLanguage} style={{ padding: '8px 16px', borderRadius: 20, border: `2px solid ${TOKENS.color.line}`, background: '#fff', cursor: 'pointer', fontWeight: 800, color: TOKENS.color.primaryDark }}>
            {IS_ENG ? '🇺🇸 ENG' : '🇰🇷 KOR'}
          </button>
          <Pill tone="success" icon="signal" size="lg">{TXT.caregiver}</Pill>
        </div>
      </div>
      <div style={{ padding: '0 28px', display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 16, flex: 1 }}>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 12 }}>
          <BigButton icon="search" tone="primary" subtitle={TXT.searchBtn} onClick={onSearch}>{TXT.searchBtn}</BigButton>
          <BigButton icon="home" tone="soft" subtitle={TXT.goHomeBtn} onClick={onGoHome}>{TXT.goHomeBtn}</BigButton>
          <Card pad={16} style={{ marginTop: 4 }}>
            <div style={{ fontSize: 17, fontWeight: 800, color: TOKENS.color.inkMuted, marginBottom: 10 }}>{TXT.favorites}</div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 8 }}>
              <button onClick={onSearch} style={{ display: 'flex', alignItems: 'center', gap: 12, padding: '16px', background: TOKENS.color.surfaceAlt, border: 'none', borderRadius: 14, cursor: 'pointer', textAlign: 'left', fontFamily: 'inherit' }}>
                <Icon name="bed" size={24} color={TOKENS.color.primary} />
                <div style={{ fontSize: 16, fontWeight: 800, color: TOKENS.color.primaryDark }}>{TXT.myRoom}</div>
              </button>
              <button onClick={onSearch} style={{ display: 'flex', alignItems: 'center', gap: 12, padding: '16px', background: TOKENS.color.surfaceAlt, border: 'none', borderRadius: 14, cursor: 'pointer', textAlign: 'left', fontFamily: 'inherit' }}>
                <Icon name="leaf" size={24} color={TOKENS.color.primary} />
                <div style={{ fontSize: 16, fontWeight: 800, color: TOKENS.color.primaryDark }}>{TXT.rehab}</div>
              </button>
            </div>
          </Card>
        </div>
        <Card pad={0} style={{ overflow: 'hidden', display: 'flex', flexDirection: 'column', cursor: 'pointer', border: `2px solid ${TOKENS.color.line}` }} onClick={onSearch}>
          <div style={{ padding: '14px 18px', borderBottom: `2px solid ${TOKENS.color.line}`, display: 'flex', justifyContent: 'space-between', background: TOKENS.color.surfaceAlt }}>
            <span style={{ fontSize: 17, fontWeight: 800, color: TOKENS.color.primaryDark }}>{TXT.liveMap}</span><Pill tone="primary" size="sm">Live</Pill>
          </div>
          <div style={{ flex: 1, background: TOKENS.color.mapBg, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
            <SlamMap width={480} height={300} showPath={false} pathProgress={0} />
          </div>
        </Card>
      </div>
      
      {/* 하단 SOS 긴급 정지 버튼 */}
      <div style={{ padding: '16px 28px 24px', display: 'flex', gap: 16 }}>
        <button onClick={onSOS} style={{ display: 'flex', alignItems: 'center', gap: 12, padding: '16px 32px', background: TOKENS.color.danger, color: '#fff', border: 'none', borderRadius: 999, fontSize: 19, fontWeight: 800, cursor: 'pointer', fontFamily: 'inherit', boxShadow: '0 8px 16px rgba(229,72,77,0.25)' }}>
          <Icon name="sos" size={26} stroke={2.5} /> {TXT.sos}
        </button>
      </div>
    </Frame>
  );
}

function SearchScreen({ onBack, onGoHome, onStartRoute }) {
  return (
    <Frame>
      <StatusBar />
      <div style={{ padding: '20px 28px', display: 'flex', alignItems: 'center', gap: 12 }}>
        <button onClick={onBack} style={{ width: 56, height: 56, borderRadius: 28, border: `2px solid ${TOKENS.color.line}`, background: TOKENS.color.surface, cursor: 'pointer', display: 'flex', alignItems: 'center', justifyContent: 'center' }}><Icon name="chevronLeft" size={28} color={TOKENS.color.primaryDark} /></button>
        <button onClick={onGoHome} style={{ width: 56, height: 56, borderRadius: 28, border: `2px solid ${TOKENS.color.line}`, background: TOKENS.color.surface, cursor: 'pointer', display: 'flex', alignItems: 'center', justifyContent: 'center' }} title={TXT.homeTitle}><Icon name="home" size={26} color={TOKENS.color.primaryDark} /></button>
        <div style={{ fontSize: 28, fontWeight: 800, color: TOKENS.color.primaryDark, marginLeft: 4 }}>{TXT.searchTitle}</div>
      </div>
      <div style={{ padding: '0 28px', flex: 1, display: 'grid', gridTemplateColumns: '1.1fr 1fr', gap: 20 }}>
        <div>
          <div style={{ fontSize: 15, fontWeight: 800, color: TOKENS.color.inkMuted, marginBottom: 12 }}>{TXT.recDest}</div>
          <Card pad={16} style={{ display: 'flex', alignItems: 'center', gap: 16, cursor: 'pointer', border: `2px solid ${TOKENS.color.primary}` }} onClick={() => onStartRoute('emergency')}>
            <div style={{ width: 52, height: 52, borderRadius: 16, background: TOKENS.color.primary, color: '#fff', display: 'flex', alignItems: 'center', justifyContent: 'center' }}><Icon name="bed" size={26} stroke={2.5} /></div>
            <div style={{ flex: 1 }}><div style={{ fontSize: 19, fontWeight: 800, color: TOKENS.color.primaryDark }}>{TXT.er}</div><div style={{ fontSize: 15, color: TOKENS.color.inkMuted, fontWeight: 600 }}>{TXT.erDesc}</div></div>
            <Icon name="chevronRight" size={28} color={TOKENS.color.primary} />
          </Card>
          <div style={{ height: 10 }} />
          <Card pad={16} style={{ display: 'flex', alignItems: 'center', gap: 16, cursor: 'pointer', border: `2px solid ${TOKENS.color.line}` }} onClick={() => onStartRoute('room_101')}>
            <div style={{ width: 52, height: 52, borderRadius: 16, background: TOKENS.color.surfaceAlt, color: TOKENS.color.primaryDark, display: 'flex', alignItems: 'center', justifyContent: 'center' }}><Icon name="bed" size={26} stroke={2.5} /></div>
            <div style={{ flex: 1 }}><div style={{ fontSize: 19, fontWeight: 800, color: TOKENS.color.primaryDark }}>{TXT.r101}</div><div style={{ fontSize: 15, color: TOKENS.color.inkMuted, fontWeight: 600 }}>{TXT.r101Desc}</div></div>
            <Icon name="chevronRight" size={28} color={TOKENS.color.inkMuted} />
          </Card>
        </div>
        <Card pad={0} style={{ display: 'flex', flexDirection: 'column', border: `2px solid ${TOKENS.color.line}` }}>
          <div style={{ flex: 1, background: TOKENS.color.mapBg }}><SlamMap width={460} height={320} showPath={true} pathProgress={0} destination={{ x: 680, y: 150, label: 'Dest' }} /></div>
          <div style={{ padding: '16px 20px', borderTop: `2px solid ${TOKENS.color.line}`, display: 'flex', alignItems: 'center', gap: 12, background: TOKENS.color.surfaceAlt }}>
            <div style={{ flex: 1 }}><div style={{ fontSize: 14, color: TOKENS.color.inkMuted, fontWeight: 700 }}>{TXT.distEta}</div><div style={{ fontSize: 22, fontWeight: 800, color: TOKENS.color.primaryDark }}>128m · 2:20</div></div>
            <button onClick={() => onStartRoute('emergency')} style={{ display: 'flex', alignItems: 'center', gap: 10, padding: '18px 28px', background: TOKENS.color.primary, color: '#fff', border: 'none', borderRadius: 999, fontSize: 18, fontWeight: 800, cursor: 'pointer', fontFamily: 'inherit', boxShadow: TOKENS.shadow.md }}>
              {TXT.startRoute} <Icon name="arrowRight" size={24} stroke={2.5} />
            </button>
          </div>
        </Card>
      </div>
    </Frame>
  );
}

function NavScreen({ mode, distances, robotWorld, mapConfig, onBack, onGoHome, onStop, onManual, onAuto }) {
  return (
    <Frame bg={TOKENS.color.mapFloor}>
      <div style={{ position: 'absolute', inset: 0, background: TOKENS.color.mapBg }}>
        <SlamMap width={1024} height={720} showPath={true} pathProgress={0.42} destination={{ x: 680, y: 150, label: TXT.moving }} robotWorld={robotWorld} mapConfig={mapConfig} />
      </div>
      <div style={{ position: 'relative', zIndex: 2, display: 'flex', padding: '20px 24px', gap: 12 }}>
        <button onClick={onBack} style={{ width: 56, height: 56, borderRadius: 28, background: 'rgba(255,255,255,0.95)', border: `2px solid ${TOKENS.color.line}`, cursor: 'pointer', display: 'flex', alignItems: 'center', justifyContent: 'center' }}><Icon name="chevronLeft" size={28} color={TOKENS.color.primaryDark} /></button>
        <button onClick={onGoHome} style={{ width: 56, height: 56, borderRadius: 28, background: 'rgba(255,255,255,0.95)', border: `2px solid ${TOKENS.color.line}`, cursor: 'pointer', display: 'flex', alignItems: 'center', justifyContent: 'center' }} title={TXT.homeTitle}><Icon name="home" size={26} color={TOKENS.color.primaryDark} /></button>
        <Card pad={0} style={{ padding: '16px 24px', background: TOKENS.color.primary, color: '#fff', display: 'flex', alignItems: 'center', gap: 12 }}><Icon name="pin" size={26} stroke={2.5} /> <span style={{ fontWeight: 800, fontSize: 20 }}>{TXT.moving}</span></Card>
      </div>
      <div style={{ position: 'absolute', left: 0, right: 0, bottom: 0, padding: 24, zIndex: 2, display: 'flex', justifyContent: 'center', gap: 16 }}>
        <div style={{ display: 'flex', gap: 12, background: 'rgba(255,255,255,0.98)', padding: 12, borderRadius: 28, boxShadow: TOKENS.shadow.lg, border: `2px solid ${TOKENS.color.line}` }}>
          <div style={{ display: 'flex', background: TOKENS.color.surfaceAlt, borderRadius: 999, padding: 4 }}>
            <button onClick={onManual} style={{ padding: '18px 28px', border: 'none', borderRadius: 999, background: mode === 'manual' ? TOKENS.color.primary : 'transparent', color: mode === 'manual' ? '#fff' : TOKENS.color.primaryDark, fontSize: 18, fontWeight: 800, cursor: 'pointer', fontFamily: 'inherit' }}>{TXT.manual}</button>
            <button onClick={onAuto} style={{ padding: '18px 28px', border: 'none', borderRadius: 999, background: mode === 'auto' ? TOKENS.color.primary : 'transparent', color: mode === 'auto' ? '#fff' : TOKENS.color.primaryDark, fontSize: 18, fontWeight: 800, cursor: 'pointer', fontFamily: 'inherit' }}>{TXT.auto}</button>
          </div>
          <button onClick={onStop} style={{ display: 'flex', alignItems: 'center', gap: 10, padding: '18px 32px', background: TOKENS.color.danger, color: '#fff', border: 'none', borderRadius: 999, fontSize: 18, fontWeight: 800, cursor: 'pointer', fontFamily: 'inherit', boxShadow: '0 4px 12px rgba(229,72,77,0.3)' }}><Icon name="stop" size={22} stroke={2.5} /> {TXT.stop}</button>
        </div>
      </div>
      <ObstacleProximity distances={distances} />
    </Frame>
  );
}

const getAlertInfo = (reason) => {
  const dict = {
    obstacle_too_close: { title: IS_ENG ? '⚠️ Obstacle Detected' : '⚠️ 전방 장애물 감지', subtitle: IS_ENG ? 'Autonomous driving paused for safety.' : '안전을 위해 자율 주행이 일시 중단되었습니다.', tone: 'warn' },
    keepout_violation: { title: IS_ENG ? '🚫 Keepout Zone' : '🚫 금지구역 진입', subtitle: IS_ENG ? 'Cannot enter this area. Please select another route.' : '진입할 수 없는 구역입니다. 다른 경로를 선택하세요.', tone: 'danger' },
    imu_emergency: { title: IS_ENG ? '🚨 Wheelchair Tilt Warning' : '🚨 휠체어 자세 위험', subtitle: IS_ENG ? 'Stopped due to tilt or impact detection.' : '기울기 또는 충격이 감지되어 정지했습니다. 휠체어 상태를 확인하세요.', tone: 'danger' },
    localization_lost: { title: IS_ENG ? '📍 Localization Lost' : '📍 위치 추적 분실', subtitle: IS_ENG ? 'Cannot find current position. System is retrying.' : '현재 위치를 찾을 수 없습니다. 시스템이 재인식을 시도 중입니다.', tone: 'danger' },
    user_stop: { title: IS_ENG ? '🛑 User Stopped' : '🛑 사용자 정지', subtitle: IS_ENG ? 'Driving stopped. Please select next action.' : '주행이 정지되었습니다. 다음 동작을 선택하세요.', tone: 'warn' },
    sos: { title: IS_ENG ? '🆘 SOS Triggered' : '🆘 SOS 호출됨', subtitle: IS_ENG ? 'Alerts sent to caregivers and control room.' : '보호자와 관제실에 알림이 전달되었습니다.', tone: 'danger' },
    default: { title: IS_ENG ? 'Driving Paused' : '주행 일시 중단', subtitle: IS_ENG ? 'Please select next action.' : '다음 동작을 선택하세요.', tone: 'warn' },
  };
  return dict[reason] || dict.default;
};

function AlertScreen({ alertReason, robotWorld, mapConfig, onResume, onManual, onBack, onGoHome, onGoHomeBase }) {
  const info = getAlertInfo(alertReason);
  const C = TOKENS.color;
  const isDanger = info.tone === 'danger';
  const headerBg = isDanger ? C.dangerSoft : C.warnSoft;
  const headerBorder = isDanger ? C.danger : C.warn;
  const headerFg = isDanger ? '#7a1f22' : '#6b4a00';
  const iconBg = isDanger ? C.danger : C.warn;
  const resumeDisabled = alertReason === 'imu_emergency' || alertReason === 'localization_lost' || alertReason === 'keepout_violation';
  
  return (
    <Frame>
      <StatusBar />
      <div style={{ background: headerBg, padding: '20px 32px', display: 'flex', alignItems: 'center', gap: 18, borderBottom: `2px solid ${headerBorder}` }}>
        <button onClick={onBack} style={{ width: 48, height: 48, borderRadius: 24, border: `2px solid ${headerBorder}`, background: '#fff', cursor: 'pointer', display: 'flex', alignItems: 'center', justifyContent: 'center' }}><Icon name="chevronLeft" size={24} color={headerFg} /></button>
        <button onClick={onGoHome} style={{ width: 48, height: 48, borderRadius: 24, border: `2px solid ${headerBorder}`, background: '#fff', cursor: 'pointer', display: 'flex', alignItems: 'center', justifyContent: 'center' }} title={TXT.homeTitle}><Icon name="home" size={22} color={headerFg} /></button>
        <div style={{ width: 56, height: 56, borderRadius: 28, background: iconBg, color: '#fff', display: 'flex', alignItems: 'center', justifyContent: 'center', flexShrink: 0 }}><Icon name="alert" size={30} stroke={2.5} /></div>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 2, minWidth: 0, flex: 1 }}>
          <div style={{ fontSize: 22, fontWeight: 800, color: headerFg, letterSpacing: -0.3, whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis' }}>{info.title}</div>
          <div style={{ fontSize: 14, fontWeight: 700, color: headerFg, opacity: 0.85, whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis' }}>{info.subtitle}</div>
        </div>
      </div>
      <div style={{ flex: 1, display: 'grid', gridTemplateColumns: '1.3fr 1fr', gap: 20, padding: 24 }}>
        <Card pad={0} style={{ display: 'flex', flexDirection: 'column', border: `2px solid ${TOKENS.color.line}` }}><SlamMap width={600} height={400} showPath={true} showObstacles={true} pathProgress={0.45} destination={{ x: 680, y: 150, label: 'Dest' }} robotWorld={robotWorld} mapConfig={mapConfig} /></Card>
        <div style={{ display: 'flex', flexDirection: 'column', gap: 12 }}>
          <Card pad={24} style={{ border: `2px solid ${TOKENS.color.line}` }}>
            <div style={{ fontSize: 16, color: TOKENS.color.inkMuted, fontWeight: 800, marginBottom: 16 }}>{TXT.recAction}</div>
            {!resumeDisabled && <>
              <BigButton tone="primary" icon="check" subtitle={TXT.subResume} onClick={onResume}>{TXT.resumeRoute}</BigButton>
              <div style={{ height: 12 }} />
            </>}
            <BigButton tone="soft" icon="home" subtitle={TXT.subSwitch} onClick={onManual}>{TXT.switchToManual}</BigButton>
            <div style={{ height: 12 }} />
            <BigButton tone="soft" icon="home" subtitle={TXT.subDock} onClick={onGoHomeBase}>{TXT.goHomeBase}</BigButton>
          </Card>
        </div>
      </div>
    </Frame>
  );
}

// ─── 4.5 수동 조작 화면 (ros2 teleop_keyboard 100% 동일 구현) ──────────────────────
const MAX_LIN_VEL = 1.0; // m/s
const MAX_ANG_VEL = 1.0; // rad/s
const LIN_VEL_STEP_SIZE = 0.01;
const ANG_VEL_STEP_SIZE = 0.01;

const MSG_TELEOP = [
  'Control Your wheelchair!',
  '---------------------------',
  'Moving around:',
  '        w',
  '   a    s    d',
  '        x',
  '',
  'w/x : increase/decrease linear velocity ',
  'a/d : increase/decrease angular velocity',
  '',
  'space key, s : force stop',
  '',
  'CTRL-C to quit'
];

function makeSimpleProfile(output, input, slop) {
  if (input > output) {
    return Math.min(input, output + slop);
  } else if (input < output) {
    return Math.max(input, output - slop);
  }
  return input;
}

function constrainVel(val, low, high) {
  if (val < low) return low;
  if (val > high) return high;
  return val;
}

function checkLinearLimitVelocity(velocity) {
  return constrainVel(velocity, -MAX_LIN_VEL, MAX_LIN_VEL);
}

function checkAngularLimitVelocity(velocity) {
  return constrainVel(velocity, -MAX_ANG_VEL, MAX_ANG_VEL);
}

function JoystickScreen({ robotWorld, mapConfig, onBack, onGoHome, setMode, cmdVelPub }) {
  const [activeKey, setActiveKey] = React.useState(null);
  const [stepMultiplier, setStepMultiplier] = React.useState(1); // 1x = 0.01, 5x = 0.05
  const [logs, setLogs] = React.useState(() => {
    return [
      { id: 1, text: 'kim@wheelchair-robot:~$ ros2 run wheelchair_teleop teleop_keyboard', type: 'cmd' },
      ...MSG_TELEOP.map((line, idx) => ({ id: 10 + idx, text: line, type: 'banner' }))
    ];
  });

  const [velDisplay, setVelDisplay] = React.useState({
    targetLin: 0.0,
    targetAng: 0.0,
    ctrlLin: 0.0,
    ctrlAng: 0.0,
  });

  const targetLinearRef = React.useRef(0.0);
  const targetAngularRef = React.useRef(0.0);
  const controlLinearRef = React.useRef(0.0);
  const controlAngularRef = React.useRef(0.0);
  const statusRef = React.useRef(0);
  const logContainerRef = React.useRef(null);

  // teleop_keyboard 키 입력 처리 로직
  const handleKeyAction = React.useCallback((keyStr) => {
    const k = keyStr.toLowerCase();
    const linStep = LIN_VEL_STEP_SIZE * stepMultiplier;
    const angStep = ANG_VEL_STEP_SIZE * stepMultiplier;

    let changed = false;
    let highlightKey = null;

    if (k === 'w' || keyStr === 'ArrowUp') {
      targetLinearRef.current = checkLinearLimitVelocity(targetLinearRef.current + linStep);
      statusRef.current += 1;
      changed = true;
      highlightKey = 'w';
    } else if (k === 'x' || keyStr === 'ArrowDown') {
      targetLinearRef.current = checkLinearLimitVelocity(targetLinearRef.current - linStep);
      statusRef.current += 1;
      changed = true;
      highlightKey = 'x';
    } else if (k === 'a' || keyStr === 'ArrowLeft') {
      targetAngularRef.current = checkAngularLimitVelocity(targetAngularRef.current + angStep);
      statusRef.current += 1;
      changed = true;
      highlightKey = 'a';
    } else if (k === 'd' || keyStr === 'ArrowRight') {
      targetAngularRef.current = checkAngularLimitVelocity(targetAngularRef.current - angStep);
      statusRef.current += 1;
      changed = true;
      highlightKey = 'd';
    } else if (k === ' ' || k === 's') {
      targetLinearRef.current = 0.0;
      controlLinearRef.current = 0.0;
      targetAngularRef.current = 0.0;
      controlAngularRef.current = 0.0;
      changed = true;
      highlightKey = 's';
    }

    if (changed) {
      targetLinearRef.current = Number(targetLinearRef.current.toFixed(4));
      targetAngularRef.current = Number(targetAngularRef.current.toFixed(4));

      // 원본 print_vels: "currently:\tlinear velocity {0}\t angular velocity {1} "
      const velText = `currently:\tlinear velocity ${targetLinearRef.current.toFixed(2)}\t angular velocity ${targetAngularRef.current.toFixed(2)} `;

      setActiveKey(highlightKey);
      setTimeout(() => setActiveKey((curr) => (curr === highlightKey ? null : curr)), 140);

      // 원본: status == 20 이면 msg 재출력
      if (statusRef.current >= 20) {
        statusRef.current = 0;
        setLogs((prev) => [
          ...prev.slice(-30),
          { id: Date.now() + Math.random(), text: velText, type: 'vel' },
          ...MSG_TELEOP.map((line, idx) => ({ id: Date.now() + Math.random() + idx, text: line, type: 'banner' }))
        ]);
      } else {
        setLogs((prev) => [
          ...prev.slice(-40),
          { id: Date.now() + Math.random(), text: velText, type: 'vel' }
        ]);
      }

      setVelDisplay({
        targetLin: targetLinearRef.current,
        targetAng: targetAngularRef.current,
        ctrlLin: controlLinearRef.current,
        ctrlAng: controlAngularRef.current,
      });
    }
  }, [stepMultiplier]);

  // 키보드 이벤트 리스너 등록
  React.useEffect(() => {
    const onKeyDown = (e) => {
      const allowed = ['w', 'W', 'x', 'X', 'a', 'A', 'd', 'D', 's', 'S', ' ', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
      if (allowed.includes(e.key)) {
        e.preventDefault();
        handleKeyAction(e.key);
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [handleKeyAction]);

  // 10Hz (100ms) 제어 프로파일 계산 및 토픽 지속 발행 루프 (teleop_keyboard 원본 while(1) 10Hz 일치)
  React.useEffect(() => {
    const slopLin = (LIN_VEL_STEP_SIZE * stepMultiplier) / 2.0;
    const slopAng = (ANG_VEL_STEP_SIZE * stepMultiplier) / 2.0;

    const intervalId = setInterval(() => {
      // make_simple_profile 로직
      controlLinearRef.current = makeSimpleProfile(
        controlLinearRef.current,
        targetLinearRef.current,
        slopLin
      );
      controlAngularRef.current = makeSimpleProfile(
        controlAngularRef.current,
        targetAngularRef.current,
        slopAng
      );

      const pubLin = Number(controlLinearRef.current.toFixed(4));
      const pubAng = Number(controlAngularRef.current.toFixed(4));

      if (cmdVelPub) {
        cmdVelPub.publish(new ROSLIB.Message({
          linear: { x: pubLin, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: pubAng }
        }));
      }

      setVelDisplay((prev) => {
        if (Math.abs(prev.ctrlLin - pubLin) > 0.001 || Math.abs(prev.ctrlAng - pubAng) > 0.001) {
          return {
            targetLin: targetLinearRef.current,
            targetAng: targetAngularRef.current,
            ctrlLin: pubLin,
            ctrlAng: pubAng,
          };
        }
        return prev;
      });
    }, 100);

    return () => {
      clearInterval(intervalId);
      // 화면 전환 또는 종료 시 긴급 제동 (속도 0 발행)
      if (cmdVelPub) {
        cmdVelPub.publish(new ROSLIB.Message({
          linear: { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 }
        }));
      }
    };
  }, [cmdVelPub, stepMultiplier]);

  // 터미널 스크롤 자동 하단 이동
  React.useEffect(() => {
    if (logContainerRef.current) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
    }
  }, [logs]);

  const isFwd = activeKey === 'w';
  const isBack = activeKey === 'x';
  const isLeft = activeKey === 'a';
  const isRight = activeKey === 'd';
  const isStop = activeKey === 's';

  const isMoving = Math.abs(velDisplay.ctrlLin) > 0.001 || Math.abs(velDisplay.ctrlAng) > 0.001;

  return (
    <Frame bg="#0b0f19">
      {/* 상단 컨트롤 헤더 */}
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', padding: '14px 24px', background: '#161e2e', borderBottom: '1px solid #1e293b' }}>
        <div style={{ display: 'flex', gap: 12, alignItems: 'center' }}>
          <button onClick={onBack} style={{ width: 42, height: 42, borderRadius: 21, background: '#1e293b', border: '1px solid #334155', display: 'flex', alignItems: 'center', justifyContent: 'center', cursor: 'pointer', color: '#f8fafc' }}>
            <Icon name="chevronLeft" size={22} color="#f8fafc" stroke={2.5} />
          </button>
          <button onClick={onGoHome} style={{ width: 42, height: 42, borderRadius: 21, background: '#1e293b', border: '1px solid #334155', display: 'flex', alignItems: 'center', justifyContent: 'center', cursor: 'pointer', color: '#f8fafc' }} title={TXT.homeTitle}>
            <Icon name="home" size={20} color="#f8fafc" stroke={2.5} />
          </button>
          <div>
            <div style={{ fontSize: 18, fontWeight: 800, color: '#f8fafc', display: 'flex', alignItems: 'center', gap: 8 }}>
              <span>{TXT.joyTitle}</span>
              <span style={{ fontSize: 11, padding: '2px 8px', borderRadius: 6, background: '#0284c7', color: '#fff', fontFamily: 'monospace', fontWeight: 700 }}>10Hz LOOP</span>
            </div>
            <div style={{ fontSize: 12, color: '#94a3b8', fontWeight: 600 }}>{TXT.joySub}</div>
          </div>
        </div>

        <div style={{ display: 'flex', gap: 12, alignItems: 'center' }}>
          {/* 스텝 배율 토글 (0.01 / 0.05) */}
          <div style={{ display: 'flex', background: '#0f172a', padding: 3, borderRadius: 8, border: '1px solid #334155' }}>
            <button
              onClick={() => setStepMultiplier(1)}
              style={{
                padding: '5px 12px', borderRadius: 6, border: 'none', cursor: 'pointer',
                background: stepMultiplier === 1 ? '#0284c7' : 'transparent',
                color: stepMultiplier === 1 ? '#fff' : '#94a3b8',
                fontSize: 12, fontWeight: 700, fontFamily: 'monospace'
              }}>
              Step: 0.01 (1x)
            </button>
            <button
              onClick={() => setStepMultiplier(5)}
              style={{
                padding: '5px 12px', borderRadius: 6, border: 'none', cursor: 'pointer',
                background: stepMultiplier === 5 ? '#0284c7' : 'transparent',
                color: stepMultiplier === 5 ? '#fff' : '#94a3b8',
                fontSize: 12, fontWeight: 700, fontFamily: 'monospace'
              }}>
              Step: 0.05 (5x)
            </button>
          </div>

          <div style={{ padding: '6px 14px', borderRadius: 999, background: isMoving ? 'rgba(16,185,129,0.15)' : 'rgba(234,179,8,0.15)', border: `1px solid ${isMoving ? '#10b981' : '#eab308'}`, color: isMoving ? '#6ee7b7' : '#fef08a', fontSize: 12, fontWeight: 800, display: 'flex', alignItems: 'center', gap: 6 }}>
            <span style={{ width: 8, height: 8, borderRadius: '50%', background: isMoving ? '#10b981' : '#eab308' }} /> {isMoving ? 'DRIVING' : (TXT.manualActive || 'KEYBOARD ACTIVE')}
          </div>

          <button onClick={setMode} style={{ display: 'flex', alignItems: 'center', gap: 8, padding: '9px 18px', background: '#2563eb', color: '#fff', border: 'none', borderRadius: 999, fontSize: 13, fontWeight: 800, cursor: 'pointer', boxShadow: '0 4px 12px rgba(37,99,235,0.4)' }}>
            <Icon name="play" size={15} stroke={2.5} /> {TXT.returnAuto}
          </button>
        </div>
      </div>

      {/* 메인 터미널 + 우측 모니터 그리드 */}
      <div style={{ flex: 1, display: 'grid', gridTemplateColumns: '1.75fr 1fr', gap: 16, padding: '16px 20px', overflow: 'hidden' }}>
        
        {/* 좌측: 리눅스 터미널 창 */}
        <div style={{ background: '#090d16', borderRadius: 14, border: '1px solid #1e293b', display: 'flex', flexDirection: 'column', overflow: 'hidden', boxShadow: '0 12px 32px rgba(0,0,0,0.6)' }}>
          {/* 터미널 상단 타이틀바 */}
          <div style={{ padding: '9px 16px', background: '#131926', borderBottom: '1px solid #1e293b', display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
            <div style={{ display: 'flex', gap: 7, alignItems: 'center' }}>
              <span style={{ width: 11, height: 11, borderRadius: '50%', background: '#ef4444', display: 'inline-block' }} />
              <span style={{ width: 11, height: 11, borderRadius: '50%', background: '#f59e0b', display: 'inline-block' }} />
              <span style={{ width: 11, height: 11, borderRadius: '50%', background: '#10b981', display: 'inline-block' }} />
            </div>
            <div style={{ fontSize: 12, color: '#94a3b8', fontFamily: 'monospace', fontWeight: 600 }}>
              bash — ros2 run wheelchair_teleop teleop_keyboard
            </div>
            <div style={{ fontSize: 11, color: '#64748b', fontFamily: 'monospace' }}>
              QoS: depth=10
            </div>
          </div>

          {/* 터미널 본문 콘솔 */}
          <div ref={logContainerRef} style={{ flex: 1, padding: '14px 18px', overflowY: 'auto', fontFamily: `'JetBrains Mono', 'Consolas', monospace`, fontSize: 12.5, lineHeight: 1.55, color: '#38bdf8', background: '#080c14' }}>
            {logs.map((log) => (
              <div key={log.id} style={{
                color: log.type === 'cmd' ? '#38bdf8'
                  : log.type === 'banner' ? '#e2e8f0'
                  : log.type === 'vel' ? '#a7f3d0'
                  : '#94a3b8',
                whiteSpace: 'pre-wrap',
                fontWeight: log.type === 'cmd' ? 700 : log.type === 'banner' ? 500 : 600,
              }}>
                {log.text}
              </div>
            ))}
            <div style={{ display: 'inline-block', width: 8, height: 14, background: '#38bdf8', marginLeft: 4, verticalAlign: 'middle', animation: 'obstaclePulse1 1s infinite' }} />
          </div>

          {/* 터미널 하단: 키보드 조작 패드 HUD (teleop_keyboard 구조 일치) */}
          <div style={{ padding: '14px 20px', background: '#0e1422', borderTop: '1px solid #1e293b', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            
            {/* W/A/S/D/X 터치/키패드 매핑 */}
            <div style={{ display: 'flex', alignItems: 'center', gap: 16 }}>
              <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 44px)', gridTemplateRows: 'repeat(3, 40px)', gap: 6 }}>
                <div />
                <button
                  onClick={() => handleKeyAction('w')}
                  title="선속도 증가 (+0.01)"
                  style={{
                    borderRadius: 8, border: `2px solid ${isFwd ? '#38bdf8' : '#334155'}`,
                    background: isFwd ? '#0284c7' : '#1e293b', color: '#fff',
                    display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center',
                    cursor: 'pointer', fontWeight: 800, fontSize: 13, fontFamily: 'monospace',
                    boxShadow: isFwd ? '0 0 14px #38bdf8' : 'none', transition: 'all 0.1s'
                  }}>
                  <span>W</span>
                  <span style={{ fontSize: 9, opacity: 0.8 }}>+LIN</span>
                </button>
                <div />

                <button
                  onClick={() => handleKeyAction('a')}
                  title="각속도 증가 (+0.01)"
                  style={{
                    borderRadius: 8, border: `2px solid ${isLeft ? '#38bdf8' : '#334155'}`,
                    background: isLeft ? '#0284c7' : '#1e293b', color: '#fff',
                    display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center',
                    cursor: 'pointer', fontWeight: 800, fontSize: 13, fontFamily: 'monospace',
                    boxShadow: isLeft ? '0 0 14px #38bdf8' : 'none', transition: 'all 0.1s'
                  }}>
                  <span>A</span>
                  <span style={{ fontSize: 9, opacity: 0.8 }}>+ANG</span>
                </button>
                <button
                  onClick={() => handleKeyAction('s')}
                  title="강제 정지 (Force Stop)"
                  style={{
                    borderRadius: 8, border: `2px solid ${isStop ? '#ef4444' : '#475569'}`,
                    background: isStop ? '#dc2626' : '#334155', color: '#fff',
                    display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center',
                    cursor: 'pointer', fontWeight: 800, fontSize: 13, fontFamily: 'monospace',
                    boxShadow: isStop ? '0 0 14px #ef4444' : 'none', transition: 'all 0.1s'
                  }}>
                  <span>S</span>
                  <span style={{ fontSize: 9, opacity: 0.8 }}>STOP</span>
                </button>
                <button
                  onClick={() => handleKeyAction('d')}
                  title="각속도 감소 (-0.01)"
                  style={{
                    borderRadius: 8, border: `2px solid ${isRight ? '#38bdf8' : '#334155'}`,
                    background: isRight ? '#0284c7' : '#1e293b', color: '#fff',
                    display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center',
                    cursor: 'pointer', fontWeight: 800, fontSize: 13, fontFamily: 'monospace',
                    boxShadow: isRight ? '0 0 14px #38bdf8' : 'none', transition: 'all 0.1s'
                  }}>
                  <span>D</span>
                  <span style={{ fontSize: 9, opacity: 0.8 }}>-ANG</span>
                </button>

                <div />
                <button
                  onClick={() => handleKeyAction('x')}
                  title="선속도 감소 (-0.01)"
                  style={{
                    borderRadius: 8, border: `2px solid ${isBack ? '#38bdf8' : '#334155'}`,
                    background: isBack ? '#0284c7' : '#1e293b', color: '#fff',
                    display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center',
                    cursor: 'pointer', fontWeight: 800, fontSize: 13, fontFamily: 'monospace',
                    boxShadow: isBack ? '0 0 14px #38bdf8' : 'none', transition: 'all 0.1s'
                  }}>
                  <span>X</span>
                  <span style={{ fontSize: 9, opacity: 0.8 }}>-LIN</span>
                </button>
                <div />
              </div>

              {/* 스페이스바 긴급 정지 버튼 */}
              <button
                onClick={() => handleKeyAction(' ')}
                style={{
                  height: 48, padding: '0 16px', borderRadius: 8,
                  border: `2px solid ${isStop ? '#ef4444' : '#ef4444'}`,
                  background: isStop ? '#dc2626' : 'rgba(239,68,68,0.15)',
                  color: '#fca5a5', cursor: 'pointer', fontWeight: 800, fontSize: 12,
                  fontFamily: 'monospace', display: 'flex', alignItems: 'center', gap: 8
                }}>
                <Icon name="xCircle" size={18} color="#fca5a5" stroke={2.5} />
                <span>[ SPACE ] FORCE STOP</span>
              </button>
            </div>

            {/* 조작 설명 힌트 */}
            <div style={{ textAlign: 'right', fontSize: 11, color: '#64748b', fontFamily: 'monospace', lineHeight: 1.6 }}>
              <div><span style={{ color: '#38bdf8', fontWeight: 700 }}>W / X</span> : 선속도 가감속 (최대 ±1.0 m/s)</div>
              <div><span style={{ color: '#38bdf8', fontWeight: 700 }}>A / D</span> : 각속도 가감속 (최대 ±1.0 rad/s)</div>
              <div><span style={{ color: '#ef4444', fontWeight: 700 }}>S / Space</span> : 속도 0.0 즉시 강제 정지</div>
            </div>
          </div>
        </div>

        {/* 우측: 실시간 미니맵 및 상태 콘솔 */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: 14 }}>
          {/* 미니맵 카드 */}
          <div style={{ background: '#161e2e', borderRadius: 14, border: '1px solid #1e293b', overflow: 'hidden', display: 'flex', flexDirection: 'column', height: 230, boxShadow: '0 8px 24px rgba(0,0,0,0.4)' }}>
            <div style={{ padding: '8px 14px', background: '#0e1422', borderBottom: '1px solid #1e293b', fontSize: 13, fontWeight: 800, color: '#f8fafc', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
              <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}><Icon name="map" size={16} /> {TXT.miniMap}</span>
              <span style={{ color: '#38bdf8', fontSize: 11, fontFamily: 'monospace' }}>SLAM LIVE</span>
            </div>
            <div style={{ flex: 1, background: '#0b0f19' }}>
              <SlamMap width="100%" height="100%" showPath={false} robotWorld={robotWorld} mapConfig={mapConfig} />
            </div>
          </div>

          {/* 텔레메트리 & 속도 프로파일 카드 */}
          <div style={{ background: '#161e2e', borderRadius: 14, border: '1px solid #1e293b', padding: '14px 16px', display: 'flex', flexDirection: 'column', gap: 10 }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
              <div style={{ fontSize: 12, fontWeight: 800, color: '#94a3b8', letterSpacing: 1 }}>TELEOP VELOCITY PROFILE</div>
              <div style={{ fontSize: 11, color: isMoving ? '#10b981' : '#64748b', fontWeight: 800, fontFamily: 'monospace' }}>
                {isMoving ? '● TRANSMITTING' : '○ IDLE'}
              </div>
            </div>

            {/* 속도 상태 그리드 (Target vs Actual Profiled) */}
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 8 }}>
              <div style={{ background: '#0b0f19', padding: '8px 10px', borderRadius: 8, border: '1px solid #1e293b' }}>
                <div style={{ fontSize: 10, color: '#64748b', fontWeight: 700 }}>TARGET LIN</div>
                <div style={{ fontSize: 16, fontWeight: 800, color: '#38bdf8', fontFamily: 'monospace' }}>
                  {velDisplay.targetLin.toFixed(2)} <span style={{ fontSize: 10, color: '#64748b' }}>m/s</span>
                </div>
              </div>
              <div style={{ background: '#0b0f19', padding: '8px 10px', borderRadius: 8, border: '1px solid #1e293b' }}>
                <div style={{ fontSize: 10, color: '#64748b', fontWeight: 700 }}>TARGET ANG</div>
                <div style={{ fontSize: 16, fontWeight: 800, color: '#38bdf8', fontFamily: 'monospace' }}>
                  {velDisplay.targetAng.toFixed(2)} <span style={{ fontSize: 10, color: '#64748b' }}>rad/s</span>
                </div>
              </div>
              <div style={{ background: '#0b0f19', padding: '8px 10px', borderRadius: 8, border: '1px solid #1e293b' }}>
                <div style={{ fontSize: 10, color: '#64748b', fontWeight: 700 }}>OUTPUT LIN (cmd_vel)</div>
                <div style={{ fontSize: 16, fontWeight: 800, color: isMoving ? '#10b981' : '#cbd5e1', fontFamily: 'monospace' }}>
                  {velDisplay.ctrlLin.toFixed(2)} <span style={{ fontSize: 10, color: '#64748b' }}>m/s</span>
                </div>
              </div>
              <div style={{ background: '#0b0f19', padding: '8px 10px', borderRadius: 8, border: '1px solid #1e293b' }}>
                <div style={{ fontSize: 10, color: '#64748b', fontWeight: 700 }}>OUTPUT ANG (cmd_vel)</div>
                <div style={{ fontSize: 16, fontWeight: 800, color: isMoving ? '#10b981' : '#cbd5e1', fontFamily: 'monospace' }}>
                  {velDisplay.ctrlAng.toFixed(2)} <span style={{ fontSize: 10, color: '#64748b' }}>rad/s</span>
                </div>
              </div>
            </div>

            {/* 오도메트리 위치 */}
            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: 6, marginTop: 2 }}>
              <div style={{ background: '#0b0f19', padding: '6px 8px', borderRadius: 6 }}>
                <div style={{ fontSize: 10, color: '#64748b' }}>X</div>
                <div style={{ fontSize: 13, fontWeight: 700, color: '#e2e8f0', fontFamily: 'monospace' }}>
                  {robotWorld?.x != null ? robotWorld.x.toFixed(2) : '0.00'}m
                </div>
              </div>
              <div style={{ background: '#0b0f19', padding: '6px 8px', borderRadius: 6 }}>
                <div style={{ fontSize: 10, color: '#64748b' }}>Y</div>
                <div style={{ fontSize: 13, fontWeight: 700, color: '#e2e8f0', fontFamily: 'monospace' }}>
                  {robotWorld?.y != null ? robotWorld.y.toFixed(2) : '0.00'}m
                </div>
              </div>
              <div style={{ background: '#0b0f19', padding: '6px 8px', borderRadius: 6 }}>
                <div style={{ fontSize: 10, color: '#64748b' }}>YAW</div>
                <div style={{ fontSize: 13, fontWeight: 700, color: '#e2e8f0', fontFamily: 'monospace' }}>
                  {robotWorld?.yaw != null ? (robotWorld.yaw * 180 / Math.PI).toFixed(1) : '0.0'}°
                </div>
              </div>
            </div>

            <div style={{ marginTop: 4, padding: '9px 12px', background: 'rgba(56,189,248,0.06)', borderRadius: 8, border: '1px solid rgba(56,189,248,0.18)', fontSize: 11.5, color: '#93c5fd', lineHeight: 1.45 }}>
              ℹ️ 키보드 키를 한 번 누르면 목표 속도가 유지되며 지속 주행합니다. 정지하려면 <strong>S</strong> 또는 <strong>Space</strong>를 누르세요.
            </div>
          </div>

        </div>
      </div>
    </Frame>
  );
}