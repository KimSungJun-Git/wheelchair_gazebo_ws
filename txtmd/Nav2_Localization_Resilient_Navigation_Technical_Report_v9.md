# Localization 신뢰도 저하 환경을 고려한 Nav2 강건 주행 아키텍처 기술보고서

AMCL 기반 정상 주행 + EKF/LiDAR 기반 Degraded Navigation + 안전 복귀 구조

**목차**

1\. 문제 정의

1.1 운용 시나리오

1.2 핵심 질문

2\. 목표 및 설계 원칙

3\. 시스템 아키텍처

3.1 전체 구성 / 3.2 센서 및 프레임 / 3.3 TF Authority

4\. Navigation Mode 상태 머신

4.1 상태 판정 / 4.2 상태 정의 / 4.3 Goal 보존 / 4.4 Fallback 진입

4.5 AMCL 재획득 / 4.6 TF handover / 4.7 velocity authority / 4.8 실패 처리

5\. 넓은 빈 공간 대응 알고리즘

6\. EKF 기반 상대 운동 추정

7\. Localization Confidence 및 빈 공간 판정

8\. Fallback Controller 설계

9\. 동적 장애물 예측 및 회피

10\. 기존 Safety Node의 확장 방향

11\. ROS 2 구현 아키텍처 및 제어권 전환

11.1 Action/BT 흐름 / 11.2 twist_mux / 11.3 Lifecycle / 11.4 Node 구성

12\. 검증 실험 계획

13\. 한계 및 리스크 분석

14\. 확장안: 특징 부족 구역의 절대 위치 보조

15\. 구현 / 개발 순서 (확정안)

16\. 기대 효과

17\. 최종 설계 요약

18\. 결론 및 다음 단계

| **항목**      | **내용**                                                                                                                                                                |
|---------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 프로젝트 대상 | ROS 2 기반 실내 자율주행 로봇 / Nav2 Navigation System                                                                                                                  |
| 핵심 문제     | LiDAR 관측 범위를 벗어난 넓은 빈 공간에서 AMCL localization 신뢰도가 저하되는 현상 및 주행 중 보행자·카트 등 동적 장애물과의 안전한 조우·회피                           |
| 기존 대응     | Safety Node가 localization 이상 감지 후 정지하고 AMCL 회복을 기다림                                                                                                     |
| 제안 확장     | 정지 일변도의 Safety Node를 Navigation Supervisor로 확장하여 일시적 localization 저하 구간을 안전하게 통과                                                              |
| 핵심 기술     | AMCL, robot_localization EKF, Wheel Encoder, IMU, LiDAR, Local Costmap, Goal Direction Fallback, 동적 장애물용 CTRV-EKF, MPPI custom critic, dynamic_obstacle_msgs(9절) |
| 설계 원칙     | 맵을 버리는 것이 아니라 global localization 의존을 일시적으로 낮추고, 회복 가능한 범위에서 상대적 주행을 수행                                                           |

본 보고서는 현재 시스템을 완전히 교체하는 것이 아니라 기존 AMCL·Nav2·Safety Node 구조를 유지하면서, localization이 구조적으로 불리한 공간에서도 안전하게 통과할 수 있도록 Navigation Mode를 확장하는 방안을 제시한다.

# 1. 문제 정의

## 1.1 운용 시나리오

실내 로봇은 사전에 작성된 2D Map을 기반으로 Nav2 Navigation을 수행하고, 목표점은 map frame의 (x, y, yaw) 좌표로 지정한다. 위치 추정은 LiDAR와 AMCL을 중심으로 수행하며, IMU와 Wheel Encoder는 odometry 생성에 활용한다.

```text
정상 주행
Map Goal (x, y, yaw) → Nav2 → Global Planner → Local Controller → cmd_vel
LiDAR → AMCL → map→odom
IMU + Encoder → EKF → odom→base_link
```

문제는 통로와 벽이 충분한 실내 구간에서는 AMCL이 안정적으로 동작하지만, 벽이나 코너 등 지도와 매칭할 특징이 멀리 떨어진 넓은 빈 공간에서는 LiDAR의 최대 측정거리(예: 10 m) 내에 유용한 구조물이 부족해질 수 있다는 점이다.

이때 AMCL의 particle 분포가 넓어지거나 pose covariance가 증가하면 현재 시스템은 localization 이상을 Safety Node가 감지하여 정지하고, localization이 회복될 때까지 기다리는 구조가 된다. 그러나 넓은 빈 공간 자체가 지속적으로 localization에 불리하다면 단순 대기만으로는 회복되지 않을 수 있다.

여기에 더해, 정상 주행이든 이 빈 공간 통과 중이든 로봇은 보행자·카트 같은 동적 장애물과 계속 조우한다. 이 문제는 위치추정 문제와 성격이 달라 별도로 다룬다: 정적 환경(지도)에 대한 로봇 자신의 위치를 추정하는 문제가 아니라, 계속 움직이는 주변 물체의 미래 위치를 예측해 충돌 없이 지나가거나 양보하는 문제다(9절).

## 1.2 핵심 질문

- “빈 공간에서 localization을 억지로 유지해야 하는가?” → 반드시 그렇지는 않다. 해당 구간의 목적은 map 기준 절대 위치를 계속 재추정하는 것보다 안전하게 다음 특징 구간까지 이동하는 것이다.

- “GPS를 사용하면 되는가?” → 실내 환경에서는 부적합하므로 현재의 map-frame Goal을 유지한다.

- “완전히 위치추정을 없애도 되는가?” → Global localization(AMCL)은 일시 중단할 수 있지만, IMU·Encoder를 이용한 상대적 운동량 추적은 Goal 방향 유지와 장애물 회피 후 복귀에 필요하다.

- “빈 공간을 미리 지도에 표시해야 하는가?” → 초기 구현에서는 필수가 아니다. AMCL confidence와 LiDAR 환경 특징을 이용해 상태 기반으로 판단하고, 반복적으로 문제가 발생하는 구간에만 선택적으로 Zone 정보를 추가할 수 있다.

- “동적 장애물도 같은 방식(localization 문제)으로 다뤄야 하는가?” → 아니다. Localization 문제는 “로봇이 정적 지도 기준 어디에 있는가”이고, 동적 장애물 문제는 “주변 물체가 다음 순간 어디로 움직이는가”이다. 서로 다른 정보(전자는 map 기준 절대 pose, 후자는 odom/base_link 기준 상대 속도)를 다루므로 인지·제어를 계층적으로 분리한다(9절).

# 2. 목표 및 설계 원칙

## 2.1 최종 목표

최종 목표는 “AMCL이 안정적일 때는 기존 Nav2 Navigation을 그대로 사용하고, localization 신뢰도가 지속적으로 저하되는 넓은 공간에서는 Goal Direction + IMU/Encoder + LiDAR Local Costmap 기반의 Degraded Navigation으로 전환하며, 다시 특징이 충분한 공간에 진입하면 AMCL을 재획득하고 정상 Navigation으로 복귀”하는 것이다.

| **원칙**                             | **설계 방향**                                                                                                       | **배제하는 접근**                                     |
|--------------------------------------|---------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------|
| Map 유지                             | Map Goal은 그대로 유지                                                                                              | GPS 전환                                              |
| Localization 역할 분리               | AMCL은 절대 위치, EKF는 상대 운동 상태                                                                              | EKF로 AMCL을 영구 대체                                |
| Safety 우선                          | 한계 초과 시 FAIL_SAFE 정지                                                                                         | 무제한 dead-reckoning                                 |
| Local sensing 지속                   | LiDAR는 Fallback에서도 장애물 감지에 사용                                                                           | 센서 없이 Goal 방향만 추종                            |
| Recovery 가능성 확보                 | 특징 공간에서 AMCL 재획득                                                                                           | Fallback을 영구 주행 모드로 사용                      |
| Localization과 동적 회피의 계층 분리 | 동적 장애물 인지·회피(9절)는 map/AMCL과 무관하게 항상 동일하게 동작(NORMAL은 2D 공간 회피, FALLBACK은 1D 시간 제어) | Navigation Mode에 따라 동적 회피 알고리즘 자체를 분기 |

# 3. 시스템 아키텍처

## 3.1 전체 구성

```text
┌───────────────┐
│ AMCL │
└───────┬───────┘
│ map→odom
▼
Map ──────── Goal ────────┌───────────────┐
│ Navigation │
│ Supervisor │
└──────┬────────┘
│ Mode
┌────────────┴─────────────┐
│ │
NORMAL NAV DEGRADED NAV
│ │
Global Planner Goal Direction
│ + LiDAR Local Costmap
Controller + EKF odom
│ │
└────────────┬─────────────┘
▼
Robot

(병행, Mode 무관) LiDAR/Odom → 동적 장애물 인지·추적(9절, map 미참조)
└─▶ NORMAL: MPPI SpatioTemporalCritic 주입 / DEGRADED: TTC 스케줄러 주입
```

## 3.2 센서 및 프레임 구조

| **구성요소**             | **주요 역할**                    | **Fallback에서의 역할**                                                            |
|--------------------------|----------------------------------|------------------------------------------------------------------------------------|
| AMCL                     | map 기준 절대 localization       | 일시적으로 신뢰하지 않음; 재획득 시 복귀                                           |
| EKF (robot_localization) | IMU + Encoder 융합으로 odom 생성 | 상대적인 이동/헤딩 추적                                                            |
| IMU                      | 각속도·가속도·자세 정보          | heading 안정화 및 이동 상태 보조                                                   |
| Wheel Encoder            | 휠 이동량                        | 전진 거리·회전량 추정                                                              |
| LiDAR                    | 환경 관측                        | Local Costmap 장애물 회피 + 환경 특징 판단 + 동적 장애물 인지 파이프라인 입력(9절) |
| Local Costmap            | 현재 주변 장애물 표현            | Fallback의 핵심 장애물 회피 계층                                                   |
| Nav2 Controller          | 속도 명령 생성                   | 정상 모드와 fallback 모드에서 제어기 전략을 분리/재사용 가능                       |

## 3.3 TF 개념

```text
map ──(AMCL)──> odom ──(EKF)──> base_link

정상: AMCL이 map→odom을 지속 보정
Fallback: 진입 직전의 신뢰 가능한 map→odom 관계를 기준으로 odom 기반 상대 이동을 사용
주의: 실제 구현에서는 TF authority 충돌을 방지하고 상태 전환 시 AMCL/EKF의 책임 범위를 명확히 관리해야 함
```

# 4. Navigation Mode 상태 머신

상태 전환에는 히스테리시스를 둔다. covariance나 particle 통계가 한 번 튀었다는 이유로 STOP과 FALLBACK을 반복하면 제어가 불안정해지므로, 연속 N회 또는 일정 dwell time을 기준으로 전환한다.

## 4.1 상태 판정의 핵심 논리

```text
Localization Confidence ↓
│
├─ 단발성 이상 → RECOVERY_WAIT
│
└─ 지속적 이상
│
├─ LiDAR 특징 충분 → 재획득 우선
│
└─ LiDAR 특징 부족/넓은 공간
↓
DEGRADED_NAV
```

## 4.2 상태 정의와 전환 조건

| **상태**       | **핵심 역할**                           | **진입 조건**                                | **종료 조건**                                                     |
|----------------|-----------------------------------------|----------------------------------------------|-------------------------------------------------------------------|
| NORMAL         | AMCL + Nav2 정상 주행                   | 초기 상태 또는 RELOCALIZATION 성공           | degraded 조건 지속                                                |
| RECOVERY_WAIT  | 정지 후 일시적 localization 회복 대기   | AMCL confidence 저하                         | Good N회 → NORMAL / persistent + feature-poor → DEGRADED_NAV      |
| DEGRADED_NAV   | AMCL 의존 없이 Goal Direction 기반 주행 | RECOVERY 실패 + 빈 공간                      | feature recovery → RELOCALIZATION / drift budget 초과 → FAIL_SAFE |
| RELOCALIZATION | 정지 상태 AMCL seeding 및 수렴 확인     | LiDAR feature 회복                           | Good N회 → NORMAL / retry limit 초과 → FAIL_SAFE                  |
| FAIL_SAFE      | 모든 자율주행 명령 차단 및 정지         | drift·relocalization·sensor safety 한계 초과 | 운영자 개입                                                       |

상태 전환에는 hysteresis와 dwell time을 적용한다. 단일 covariance spike나 짧은 feature loss로 상태가 반복 전환되지 않도록 bad N회 / good N회 조건을 사용하고, NORMAL 복귀 직후 일정 시간 동안 degraded 재판정을 보류한다.

이 문서에서 “FALLBACK”은 DEGRADED_NAV 상태를 가리키는 통칭으로 사용한다(“FALLBACK 진입”, “Fallback Controller” 등). STUCK_RECOVERY(5.2절), TTC_YIELDING/STANDOFF_NUDGE/YIELD_ABORT(9.5절)는 별도의 Supervisor 상태가 아니라 DEGRADED_NAV 내부에서 Fallback Controller가 관리하는 세부 제어 모드다.

## 4.3 Supervisor의 책임과 Goal 보존

Supervisor는 최초 NavigateToPose goal을 전달하는 주체이거나 goal 전달 경로를 직접 관리하는 위치에 둔다. Action feedback에서 goal을 역으로 복원하지 않고, goal을 최초 수신 시점에 map frame 기준으로 저장한다. 따라서 Navigation 취소와 재시작 사이에서도 목표가 보존된다.

> preserved_goal = { frame_id: "map", x: Xg, y: Yg, yaw: YawG }

- 정상 Navigation 시작 또는 goal 갱신 직후 preserved_goal 갱신

- FALLBACK 진입 직전 preserved_goal을 유지한 채 NavigateToPose cancel

- NORMAL 복귀 시 preserved_goal을 다시 NavigateToPose로 전달

## 4.4 FALLBACK 진입 handover

1.  Supervisor가 localization degradation을 지속적으로 확인하고 preserved_goal을 확정한다.

2.  AMCL의 tf_broadcast 파라미터를 false로 설정하고, 그 직후 Supervisor의 frozen map→odom 브로드캐스터를 시작한다. AMCL의 lifecycle은 ACTIVE로 유지한다(lifecycle transition을 사용하지 않는 이유는 4.6/11.3절 참조).

3.  현재 NavigateToPose action을 cancel하여 bt_navigator의 현재 Navigation 실행을 종료 방향으로 전환한다.

4.  cancel_goal_async() 요청 후 GoalStatus가 CANCELED로 전이될 때까지 최대 500ms 대기한다. 타임아웃되어도 취소 자체는 이미 요청된 상태이므로, /cmd_vel_supervisor_brake로 정지를 유지한 채 다음 단계로 진행한다(4.7절 참조).

5.  controller_server 출력과 Fallback 출력을 각각 /cmd_vel_nav, /cmd_vel_fallback으로 분리한다.

6.  twist_mux에서 Fallback priority를 더 높게 설정하여 최종 /cmd_vel authority를 arbitration한다.

7.  Fallback Controller가 Goal Direction + IMU/Encoder + LiDAR 기반으로 주행을 시작한다.

> controller_server → /cmd_vel_nav ──────┐  
> Fallback Controller → /cmd_vel_fallback ─┤  
> Supervisor(brake) → /cmd_vel_supervisor_brake ─┤→ twist_mux → /cmd_vel → Robot  
> Safety Stop → /cmd_vel_safety ───────────┘

## 4.5 AMCL 재획득 및 reseeding

Fallback에서 LiDAR 특징이 다시 충분해지면 즉시 AMCL을 켜지 않고 먼저 로봇을 정지시킨다. 이후 Supervisor가 마지막 정상 map→odom과 Fallback 동안의 누적 odom delta를 이용해 현재 map-frame pose를 추정하고 이를 /initialpose로 제공한다.

> T_map_base_est = T_map_odom_last ⊕ ΔT_odom

Fallback 거리가 짧고 drift가 작으면 상대적으로 작은 covariance를 사용하고, drift budget 소진 비율이 커질수록 covariance를 inflate한다. 계수는 실제 실험 데이터로 결정한다.

AMCL은 기본적으로 이동량(update_min_d, update_min_a)이 발생해야 particle filter를 갱신하므로, 정지 상태에서 /initialpose만 발행하는 것으로는 covariance가 실제로 수렴하지 않을 수 있다. 따라서 재획득 절차에는 정지 상태에서도 갱신을 강제하는 request_nomotion_update 서비스 호출을 포함한다.

8.  LiDAR feature recovery 조건 확인

9.  로봇 정지 및 Fallback velocity 차단

10. 추정 pose를 /initialpose로 publish (AMCL은 ACTIVE 상태를 유지하며, tf_broadcast는 계속 false로 유지된다)

11. /request_nomotion_update 서비스를 LiDAR 스캔 수신에 맞춰 N회(5~10회) 비동기 호출하여 정지 상태에서도 particle filter 갱신을 강제

12. AMCL covariance와 pose 안정성이 N회 연속 조건을 만족하는지 확인

13. 확인되면 Supervisor의 frozen TF 브로드캐스트를 먼저 중단하고, 그 직후 AMCL의 tf_broadcast 파라미터를 true로 설정하여 map→odom authority를 넘긴다(두 퍼블리셔가 동시에 존재하는 구간이 생기지 않도록 순서를 지킨다)

14. /global_costmap/clear_entirely_global_costmap 및 /local_costmap/clear_entirely_local_costmap 서비스를 호출하여 Fallback 중 drift된 pose 기준으로 누적된 장애물 마크를 제거한다

15. preserved_goal을 NavigateToPose로 재전송하여 현재 위치에서 새 global planning 시작

## 4.6 TF Authority 전환

TF handover는 AMCL의 lifecycle을 전환하는 대신, AMCL을 항상 ACTIVE로 유지한 채 tf_broadcast 파라미터만 상태별로 배타적으로 토글하는 방식으로 관리한다. Nav2 lifecycle_manager는 관리 노드와 bond(heartbeat)를 유지하는데, Supervisor가 AMCL을 외부에서 임의로 INACTIVE로 전환하면 lifecycle_manager가 이를 예기치 않은 노드 결함으로 판단해 Nav2 stack 전체를 강제 종료하거나 재시작 루프에 빠뜨릴 위험이 있다. 따라서 lifecycle transition 자체를 사용하지 않는다. NORMAL에서는 tf_broadcast=true로 AMCL이 map→odom을 단독 publish하고, FALLBACK에서는 tf_broadcast=false로 AMCL의 TF 출력을 막은 뒤 Supervisor가 마지막 정상 transform을 재발행한다. Supervisor가 재발행하는 transform은 값은 고정하되 타임스탬프를 now()로 매 주기 갱신하여 20Hz 이상으로 지속 publish한다; 고정된 과거 타임스탬프를 그대로 두면 tf2 버퍼 캐시가 만료되어 lookupTransform이 extrapolation exception을 던질 수 있다. RELOCALIZATION 준비 단계에서 tf_broadcast를 true로 되돌리기 전에는 반드시 Supervisor의 frozen TF 발행을 먼저 중단하여, 두 퍼블리셔가 동시에 map→odom을 발행하는 구간이 생기지 않도록 순서를 지킨다.

| **상태**                          | **map→odom authority** | **AMCL (lifecycle / tf_broadcast)**                      |
|-----------------------------------|------------------------|----------------------------------------------------------|
| NORMAL                            | AMCL                   | ACTIVE / tf_broadcast=true                               |
| FALLBACK                          | Supervisor frozen TF   | ACTIVE / tf_broadcast=false                              |
| RELOCALIZATION 준비(수렴 확인 중) | Supervisor frozen TF   | ACTIVE / tf_broadcast=false (내부 covariance만 모니터링) |
| RELOCALIZATION 성공 후            | AMCL                   | ACTIVE / tf_broadcast=true                               |

## 4.7 Velocity Authority와 twist_mux

정상 Navigation과 Fallback이 동시에 /cmd_vel을 직접 publish하는 구조는 사용하지 않는다. 각 command stream을 서로 다른 topic으로 분리하고 twist_mux가 하나의 최종 /cmd_vel을 생성한다. twist_mux는 활성 최우선 토픽의 마지막 값을 timeout 이전까지 계속 재발행할 수 있으므로, Fallback이 단순히 publish를 멈추는 것만으로는 RELOCALIZATION처럼 완전 정지가 필요한 구간에서 잔류 속도가 새어나갈 수 있다(Control Leakage). 이를 막기 위해 Safety Stop과 Fallback 사이에 Supervisor 전용 래칭 브레이크 채널을 둔다: 모드 전환 및 RELOCALIZATION 진행 중에는 Supervisor가 (0,0,0)을 이 채널로 지속 publish하여 물리적 정지를 명시적으로 보장한다.

| **priority** | **topic**                 | **송신 주체 / 역할**                                  | **timeout**      |
|--------------|---------------------------|-------------------------------------------------------|------------------|
| 255          | /cmd_vel_safety           | Safety Node — 충돌/비상 정지                          | 0.1s             |
| 150          | /cmd_vel_supervisor_brake | Supervisor — 모드 전환/RELOCALIZATION 중 (0,0,0) 래칭 | 0s(지속 publish) |
| 100          | /cmd_vel_fallback         | Fallback Controller — DEGRADED_NAV 주행               | 0.2s             |
| 10           | /cmd_vel_nav              | controller_server — 정상 Navigation                   | 0.2s             |

## 4.8 RELOCALIZATION 실패 처리

- N회 내 수렴 실패 → DEGRADED_NAV으로 복귀

- MAX_RELOCALIZATION_RETRY 초과 → FAIL_SAFE

- NORMAL 복귀 직후 MIN_NORMAL_DWELL_TIME 동안 재평가 보류

- 재개된 Nav2에서 goal이 막히는 경우에는 표준 replanning/recovery에 위임

# 5. 넓은 빈 공간 대응 알고리즘

## 5.1 “빈 공간에서는 위치추정이 필요 없는가?”

설계 관점에서는 “AMCL을 통한 map-frame 절대 위치추정이 필요하지 않은 구간”으로 정의할 수 있다. 대신 로봇이 목표 방향을 유지하고 장애물 회피를 수행하기 위해서는 상대 운동량이 필요하다. 따라서 EKF는 이 구간에서 global localization을 대신하는 것이 아니라, Encoder와 IMU로부터 안정적인 odom을 제공하는 보조 계층이다.

특히 Goal을 단순한 좌표 추종 문제로 다루기보다 “마지막으로 신뢰 가능한 localization 시점에서 계산한 Goal Direction을 유지하는 문제”로 정의하면 넓은 공간을 통과하는 Fallback Controller를 단순화할 수 있다. 다만 이때 추종 대상은 preserved_goal(최종 목적지) 좌표 자체가 아니라, 진입 직전 마지막으로 계산된 Global Path 상의 근거리 지점이어야 한다 — 최종 목적지가 빈 공간 너머 다른 통로에 있는 경우, 직선으로 최종 목적지를 향해 추종하면 출구 방향과 무관하게 벽면 쪽으로 계속 밀어붙이게 되기 때문이다(5.2절 Exit Sub-goal 참조).

## 5.2 Goal Direction Fallback

```text
[Fallback 진입]
1) 마지막 정상 map-frame pose P0와 직전 계산된 Global Path 확보
2) Global Path 상에서 P0로부터 drift budget 반경 이내의 최외곽 waypoint를 Exit Sub-goal(w*)로 선정하고, 이를 기준으로 초기 방향 벡터 계산(최종 목적지를 직접 겨냥하지 않음)
3) IMU/EKF로 현재 heading 및 상대 이동 상태 추적
4) LiDAR/Local Costmap으로 장애물 회피
5) 장애물 회피 후 Exit Sub-goal 방향으로 재정렬
6) 허용 거리/시간/heading drift를 초과하면 FAIL_SAFE
7) LiDAR 특징이 충분해지면 AMCL 재획득 후 NORMAL 복귀
```

Exit Sub-goal은 다음과 같이 정의한다: w\* = arg max\_{w_i ∈ P_global} { ‖w_i - P0‖ ≤ D_fallback_budget }, 이후 (x_exit_odom, y_exit_odom) = T_odom_map_frozen · (x_w\*, y_w\*)로 odom frame에 투영하여 8.2절의 방위각 추종 대상으로 사용한다. Global Path는 Nav2가 이미 계산해 /plan 토픽으로 발행하므로 Supervisor는 이를 구독해 최신 값을 캐시해두는 것만으로 충분하며, 별도의 경로 계획 로직을 새로 구현할 필요는 없다.

장애물이 오목한(concave) 형태로 배치되면 Goal 인력과 회피 반발력이 상쇄되어 Fallback Controller가 전진과 회전을 반복하는 local minima에 갇힐 수 있다. 이를 막기 위해 Fallback 진입 시 생성한 가상 직선(M-line) 대비 진행률을 감시하고, 일정 시간(예: 2초) 이상 전진 속도가 임계값(예: 0.05 m/s) 미만이면 STUCK_RECOVERY로 진입해 벽면을 일정 거리 추종한 뒤 다시 M-line으로 복귀하는 Bug2 방식의 탈출 로직을 둔다. 다만 정지 원인이 동적 장애물에 대한 의도적 양보(TTC Yielding)인 경우에는 이 감시기가 STUCK으로 오판하지 않도록 is_yielding 바이패스를 둔다(9.5절 참조).

## 5.3 중요한 제약

| **문제**                  | **영향**                                                      | **설계 대응**                                                      |
|---------------------------|---------------------------------------------------------------|--------------------------------------------------------------------|
| 빈 공간이 너무 큼         | EKF/odometry drift 누적                                       | drift budget 기반 Fallback 최대거리/시간 제한                      |
| yaw drift                 | Goal 방향 자체가 틀어질 수 있음                               | IMU yaw 품질을 별도 모니터링하고 heading error를 제한              |
| 장애물 회피 후 경로 이탈  | 목표 직선으로 단순 복귀 어려움                                | relative motion과 local trajectory를 사용하여 Goal 방향 재정렬     |
| Goal이 빈 공간 중앙       | 재획득할 특징이 없음                                          | Fallback 한계를 적용하며 필요 시 별도 랜드마크/추가 관측 전략 고려 |
| AMCL 자체 오류            | 빈 공간이 아닌데도 confidence 저하                            | LiDAR 환경 특징과 localization 지표를 함께 사용                    |
| 오목 장애물(local minima) | Goal 인력과 회피 반발력 상쇄로 제자리 진동, drift budget 소진 | M-line 진행률 감시 + STUCK_RECOVERY(벽면 추종) 탈출                |

# 6. EKF 기반 상대 운동 추정

## 6.1 EKF의 역할

EKF는 AMCL의 절대 위치 오차를 제거하지 않는다. 핵심 역할은 IMU와 Wheel Encoder의 서로 다른 특성을 융합하여 odom 프레임에서의 상대적인 pose와 velocity를 안정적으로 제공하는 것이다. 따라서 Fallback에서의 “얼마나 움직였는가”와 “현재 어떤 heading을 유지하고 있는가”를 계산하기 위한 기반 계층으로 사용한다.

```text
Wheel Encoder ───────┐
├──> robot_localization EKF ──> odom→base_link
IMU ─────────────────┘
```

## 6.2 EKF가 해결하지 못하는 것

- 장기간 누적되는 odometry drift 자체를 제거하지 못한다.

- GPS가 없다고 해서 map 기준 절대 위치를 복구하지 못한다.

- AMCL이 완전히 무너진 상태에서 수십 m 이상의 장거리 정확 navigation을 보장하지 않는다.

- 따라서 EKF 기반 Fallback은 “bounded recovery”로 설계하고, 허용 오차를 넘으면 안전 정지하는 종단 조건이 필요하다.

## 6.3 Drift Budget

Fallback 허용범위는 임의로 2 m, 5 m 등으로 정하지 않고 실측 데이터를 통해 결정한다. 최소한 선형 위치 오차와 yaw 오차를 분리하여 평가한다.

```text
선형 drift budget: E_pos(d) ≤ E_pos,max
헤딩 drift budget: |E_yaw(d)| ≤ E_yaw,max (단, 아래 RANSAC 헤딩 구속 활성 시 사실상 bounded)
최종 Fallback 조건: (거리 ≤ D_max) AND (시간 ≤ T_max) AND (yaw_error ≤ Yaw_max)
```

특히 차동구동 로봇은 작은 yaw 오차가 이동거리에 비례하여 횡방향 위치 오차로 증폭될 수 있으므로, 단순 이동거리만으로 Fallback 가능 범위를 정의하면 안 된다.

### 6.3.1 RANSAC 벽면 상대각 기반 헤딩 드리프트 억제 (Virtual Pseudo-Measurement)

$E_{pos}(d)$ 자체를 없애는 것이 아니라, 이동거리에 비례해 횡방향 오차로 증폭되는 $E_{yaw}(d)$의 증가를 특정 조건에서 0에 가깝게 묶어버리는(bound) 보정치를 도입한다. 7.2절에서 이미 검출하는 RANSAC 벽면 직선 출력을 그대로 재활용하여 새로운 인지 파이프라인 없이 구성한다.

Fallback 중에는 map→odom이 frozen 상태이므로 map frame 절대각(0°/90°)을 직접 EKF에 넣을 수 없다. 따라서 절대각 대신 **"벽면과의 상대 각도를 일정하게 유지하라"**는 상대각 구속 관측 모델을 사용한다:

```text
[RANSAC 기반 헤딩 구속 수식]
1) 복도 진입 시점(t1)에 7.2절 RANSAC이 검출한 벽 각도를 로봇 로컬 프레임 기준으로 기록:
   α_rel(t1) = RANSAC 검출 직선의 base_link 기준 상대각

2) 이 시점의 벽의 odom-frame 절대각을 역산하여 고정(Anchor):
   θ_wall_odom = ψ(t1) + α_rel(t1)     (ψ = EKF의 현재 yaw 추정치)

3) 이후 매 스캔 주기 (t > t1):
   예측값:  α_rel_pred(t) = θ_wall_odom - ψ_predicted(t)
   측정값:  α_rel_meas(t) = 그 시점 RANSAC이 재검출한 상대각
   innovation = α_rel_meas(t) - α_rel_pred(t)
   H = [∂α_rel/∂ψ] = -1   (yaw 성분에만 대응, x/y는 관측 안 함)

4) 이 innovation을 robot_localization EKF의 pseudo-measurement로 update
```

#### 활성화 게이팅 조건 (안전 가드)
오관측에 의한 왜곡을 방지하기 위해 다음 4가지 조건을 모두 만족할 때만 엄격히 활성화한다:
- **DEGRADED_NAV(FALLBACK) 상태일 때만 적용**: NORMAL 상태에서는 AMCL이 처리하므로 불필요.
- **좌우 벽 2개가 상호 평행하게 검출될 때만 적용**: 단일 벽만 볼 경우 각진 장애물이나 비스듬한 벽 오인 위험이 있으므로, 좌우 평행성($|\Delta \theta| \le 5^\circ$)이 성립해야 복도로 확정.
- **STUCK_RECOVERY(벽면 추종) 중 비활성화**: 회피 또는 탈출을 위해 의도적으로 벽에 붙어 선회하는 구간이므로 상대각 고정 전제 파기.
- **급격한 조향(TTC nudge, M-line 재정렬) 직후 쿨다운**: 의도적 회전 직후에는 벽 상대각이 바뀌는 것이 정상이므로 오탐 방지를 위해 일정 시간(예: 1.5s) 비활성화.

#### 측정 잡음(R) 튜닝 및 주의사항
측정 잡음 공분산 $R_{wall}$은 직선 복도에서 벽 상대각의 프레임 간 분산을 실측하여 결정해야 하는 튜닝 파라미터다. $R$을 너무 작게 잡으면 완만한 곡선 복도에서도 강제로 직선으로 왜곡할 위험이 있고, 너무 크면 보정 효과가 소실된다.
또한 이 제약 활성 중에는 EKF의 yaw covariance가 인위적으로 축소되므로, 4.5절 AMCL 재수렴 판정 시 실제보다 유리하게 오판되지 않도록 실측 검증 및 분리 관리가 필요하다.

# 7. Localization Confidence 및 빈 공간 판정

## 7.1 AMCL confidence

현재 Safety Node가 AMCL particle cloud의 분산 또는 관련 localization 지표를 이용하고 있다면, 이를 단일 임계값이 아닌 “연속 상태”로 확장하는 것이 적절하다. 구현 시 사용 중인 ROS 2 Nav2/AMCL 버전에 실제 제공되는 topic과 message field를 확인한 후 covariance, particle 분포, pose update 안정성 등의 지표를 선정한다.

## 7.2 환경 특징 점수

넓은 빈 공간 판정에는 LiDAR의 유효 point 수와 거리 분포, 가까운 구조물 존재 여부 등의 단순 지표를 사용할 수 있다. 초기 버전은 규칙 기반으로 충분하며, 데이터가 축적된 이후 분류 모델을 검토할 수 있다.

```text
Environment Feature Score 예시
F = w1·valid_point_ratio + w2·near_obstacle_ratio + w3·range_variance + w4·wall_presence

Confidence Low + Feature Low → Degraded Navigation 진입 후보
```

다만 F는 point 개수·거리 기반 지표만으로는 사람 무리나 이동 카트처럼 로봇 주변에 일시적으로 몰린 동적 장애물과 실제 정적 벽을 구분하지 못한다. 이 경우 F가 급상승해 RELOCALIZATION을 잘못 트리거하고, AMCL이 동적 물체를 정적 지도의 벽으로 잘못 매칭(kidnapped robot)할 위험이 있다. 정적 여부를 판정할 때 map frame으로 투영해 지도와 비교하는 방식은 그 투영 자체가 드리프트가 섞인 T_map_base_est를 기준으로 하므로, drift가 σ_hit 수준(0.1~0.2 m)만 누적돼도 실제 벽 앞에서조차 정합도가 0에 수렴해 영원히 RELOCALIZATION에 진입하지 못하는 자기참조적 문제가 생긴다. 따라서 전역 pose 추정치와 무관하게, 로봇 로컬 프레임의 원시 스캔 형상만으로 정적 구조물 여부를 판정한다.

```text
Line Inlier Ratio = N_RANSAC_line_points / N_total_valid_points

1) 로컬 스캔에서 RANSAC으로 연속된 직선 성분 검출
2) 검출된 직선 길이 ≥ 1.5 m AND Inlier Ratio ≥ 60% → 정적 구조물(벽면)로 확정

RELOCALIZATION 진입 조건: F 기준 충족 AND 정적 구조물 확정
```

이 판정을 통과한 이후에는 정합도 점수가 아니라, 검출된 벽면과의 수직 거리(d\_⊥) 투영을 통해 단계적으로 로봇 위치를 수렴시킨다. 사람/카트가 우연히 일렬로 늘어서서 긴 직선을 형성할 가능성은 낮으므로, 별도의 전역 pose 의존 없이도 동적 장애물 오인식을 충분히 걸러낼 수 있다.

이 구조의 장점은 “AMCL이 이상하다”와 “환경이 원래 localization에 불리하다”를 구분할 수 있다는 점이다. 단순 센서 오류까지 모두 Fallback으로 보내지 않도록 두 정보를 함께 사용한다. 아울러 이 RANSAC 직선 검출 출력은 6.3.1절의 헤딩 드리프트 억제 관측치(pseudo-measurement)에도 그대로 재사용된다.

# 8. Fallback Controller 설계

## 8.1 제어 목표

Fallback Controller는 전역 경로를 새로 만드는 것보다 “목표 방향 유지 + 장애물 회피 + 제한거리 내 이동”에 집중한다. 따라서 복잡한 Global Planner를 새로 만들 필요가 없고, 기존 Local Costmap과 Controller를 최대한 재사용하는 방향으로 설계한다.

| **기능**            | **입력**                                             | **출력/판단**                                           |
|---------------------|------------------------------------------------------|---------------------------------------------------------|
| Goal Direction      | 마지막 정상 pose, Map Goal                           | 목표 방위각 / 상대 방향                                 |
| Heading Control     | IMU/EKF yaw                                          | yaw error → angular velocity                            |
| Obstacle Avoidance  | LiDAR / Local Costmap + dynamic_obstacle_msgs(9.5절) | 회피 가능한 local trajectory + TTC 기반 감속/정지/nudge |
| Progress Monitoring | EKF odom                                             | 이동거리, 경과시간                                      |
| Safety Monitor      | AMCL + EKF + LiDAR                                   | NORMAL/FALLBACK/FAIL_SAFE 상태                          |

## 8.2 기본 제어 개념

진입 시점의 goal_heading만 고정 추종하면 장애물을 좌우로 우회한 뒤 같은 각도로 복귀했을 때 원래 직선이 아니라 그와 평행한 오프셋 경로를 따르게 된다. 이를 방지하기 위해 추종 대상(5.2절의 Exit Sub-goal w\*)을 odom frame의 고정 좌표로 한 번만 투영하고, 이후에는 매 제어 주기마다 현재 위치 기준으로 방위각을 다시 계산하는 방식을 사용한다.

```text
[Fallback 진입 시 1회] Exit Sub-goal w*(5.2절)를 odom frame으로 투영:
(x_g_odom, y_g_odom) = T_odom_map_frozen · (x_w*, y_w*)

[매 제어 주기] 현재 위치 기준으로 방위각을 재계산:
theta_target(t) = atan2(y_g_odom - y_robot_odom(t), x_g_odom - x_robot_odom(t))
yaw_error = wrap_to_pi(theta_target(t) - current_heading(t))
angular_cmd = K_yaw * yaw_error
linear_cmd = f(|yaw_error|, obstacle_distance, safety_constraints)

단, 장애물이 감지되면 local avoidance가 우선하며, 회피 종료 후에도 위 방식으로 방위각을 매 주기 재계산하므로 진입 시점의 heading만 고정 추종할 때 발생하는 평행 오프셋 누적이 방지된다. w*를 최종 목적지 대신 사용함으로써, 목적지가 빈 공간 너머 다른 통로에 있을 때 벽면을 향해 직선으로 밀어붙이는 위험도 함께 줄어든다.
```

# 9. 동적 장애물 예측 및 회피

## 9.1 설계 원칙: 계층 분리

동적 장애물 대응은 NORMAL과 FALLBACK에서 서로 다른 층위로 처리한다. NORMAL에서는 공간을 돌아서 피하고(Spatial Detour), FALLBACK에서는 속도를 줄여 먼저 보낸다(Temporal Yielding). Fallback Controller는 8.1절의 단순성 원칙(전역 경로를 새로 만들지 않는다)을 그대로 유지하며, 2차원 회피 궤적 생성 기능을 추가하지 않는다.

| **구분** | **담당 컨트롤러**        | **소비 정보**                                | **회피 방식**                           |
|----------|--------------------------|----------------------------------------------|-----------------------------------------|
| NORMAL   | Nav2 MPPI Controller     | map frame 2초 예측 튜브(전체 배열)           | 2D 공간 우회(S자 궤적)                  |
| FALLBACK | Fallback Controller(8절) | base_link 최근접 1개 장애물의 상대 위치/속도 | 1D 시간 제어(TTC 감속/정지) + 미세 편향 |

## 9.2 인지 파이프라인

전처리와 동적/정적 분리는 Supervisor의 Navigation Mode 상태와 무관하게 항상 동일한 방식으로 동작한다(mode-agnostic). map frame이나 AMCL 추정치를 일절 참조하지 않으므로, RELOCALIZATION 시 map→odom이 점프하거나 FALLBACK 중 TF가 frozen되어도 인지 파이프라인은 영향을 받지 않는다.

```text
[8~9Hz 원시 LaserScan (odom 기준) & 50Hz Odom]
│
▼
[Node 0: scan_deskewer_node] ── 포인트별 timestamp 계산 후 고주파 odom으로 TF 선형 보간, 111ms 회전 왜곡 제거
│
▼ (PointCloud2, odom frame)
[Node 1: dynamic_point_extractor] (Supervisor 상태와 무관, map 미참조)
├─ 0.5초 수명 Rolling Local Decaying Grid 유지 (약 4~5프레임, 로봇 주변 6m×6m)
├─ Free-space violation 검사: 캐시상 비어 있던 셀을 뚫고 들어온 점 → 동적 후보 (DBSCAN 묶음)
└─ 캐시에서 정적으로 유지된 점 → 7.2절 RANSAC 벽면 검출기로 단방향 전달
│
▼ (동적 후보 포인트)
[obstacle_tracker_node] ── ABD 분할 + 2D 타원 피팅 + CTRV-EKF + Track 수명주기(9.3절)
├─ 관측 갱신: 8~9Hz (라이다 스캔 도착 시 칼만 게인으로 위치/속도 보정)
└─ 상태 전파: 20Hz 고주파 타이머 (스캔 공백 111ms 동안 등속도 모델로 외삽)
│
▼ (dynamic_obstacle_msgs/DynamicObstacleArray, 9.6절)
NORMAL → MPPI SpatioTemporalCritic(9.4절) / FALLBACK → TTC 스케줄러(9.5절)
```

Node 1이 map을 참조하지 않는 이유는 두 가지다. 첫째, FALLBACK 중에는 map→odom이 frozen(점진적 드리프트) 상태이므로 이를 기준으로 정적/동적을 판정하면 7.2절에서 이미 걷어낸 Score_static 자기참조 문제가 재발한다. 둘째, Supervisor 상태에 따라 판정 알고리즘 자체를 분기하면 모드 전환 시점마다 또 하나의 동기화 지점이 생겨 TF/twist_mux에서와 같은 race condition 위험이 반복된다. Rolling Local Decaying Grid는 순수 프레임 간(이전 스캔 대비 현재 스캔, ego-motion 보정 포함) 비교이므로 이 문제가 발생하지 않는다.

## 9.3 Track 수명주기 및 히스테리시스

속도 상태(MOVING/SEMI_STATIC/STATIC)와 가시성 상태(TRACKING/COASTING/LOST)를 독립된 두 축으로 관리한다.

여기서 말하는 EKF는 로봇 자신의 odometry를 추정하는 6절의 EKF와는 별개의 인스턴스로, 추적 중인 장애물 track마다 하나씩 존재하며 해당 장애물의 상대 위치·속도만 추정한다.

```text
[속도 상태 전이]
IF status == MOVING:
IF ||v_estimated|| < 0.15 m/s FOR 0.5s: status = SEMI_STATIC; start_static_timer()
ELSE IF status == SEMI_STATIC:
IF ||v_estimated|| > 0.25 m/s: status = MOVING; reset_static_timer()
ELSE IF static_timer > 3.0s: status = STATIC; publish_to_costmap_static_layer()

[가시성 상태 전이 — 매 프레임, track별]
IF 이번 프레임에 매칭되는 detection 있음:
EKF update 수행; coasting_timer = 0; confidence 회복; visibility = TRACKING
ELSE:
visibility = COASTING; coasting_timer += dt
EKF predict만 수행(update 없음), Q(process noise) 확대, confidence 매 프레임 감소
IF coasting_timer > T_coast_max(예: 2.0s):
visibility = LOST; MPPI/TTC 공급 중단; 상태를 grace_buffer에 최대 T_grace(예: 4.0s) 보관

[신규 detection 연관 순서]
1) 기존 TRACKING/COASTING track과 우선 매칭
2) 실패 시 grace_buffer의 LOST track과 매칭 (탐색반경 = v_human_max×경과시간 + margin) → 매칭되면 이전 상태를 이어받아 부활, confidence는 낮게 재시작
3) 그래도 실패 시 완전 신규 track 생성
```

confidence는 track age, 최근 매칭 성공률, EKF 공분산 크기로 계산하는 0~1 값이다. COASTING 중 매 프레임 감소하고 TRACKING 복귀 시 회복된다. 이 값은 9.4절 MPPI 튜브의 초기 분산을 정하는 데 직접 사용한다 — confidence가 낮을수록(방금 생성됐거나 가려짐에서 막 복귀한 track일수록) 예측 불확실성이 크다고 보고 튜브를 더 넓게 잡는다.

## 9.4 NORMAL: MPPI 기반 2D 공간 회피

controller_server 내부 MPPI에 SpatioTemporalCritic을 custom plugin으로 추가한다. 로봇 후보 궤적의 t_k 시점과 장애물의 t_k 시점 예측 타원 사이 거리를 비대칭 Mahalanobis 비용으로 평가하여, 전방 접근에는 큰 가중치를, 통과 후 후방에는 작은 가중치를 부과함으로써 장애물 뒤로 자연스럽게 돌아 들어가는 궤적을 유도한다.

```text
dist_maha^2(t_k) = (x_robot(t_k) - μ_x(t_k))^2 / σ_long^2(t_k) + (y_robot(t_k) - μ_y(t_k))^2 / σ_lat^2(t_k)
cost(t_k) = W_front · dist_maha^2 (전방 접근) 또는 W_rear · dist_maha^2 (통과 후 후방)
(예시: W_front = 3.0, W_rear = 0.2)

σ_long(t_k), σ_lat(t_k)의 초기값은 track의 confidence에 반비례하여 팽창한다.
```

SEMI_STATIC 장애물이 전방 회랑(base_link 기준 0 \< x_rel \< 2m AND \|y_rel\| \< W_robot/2 + 0.5m) 안에 있을 때는 PathAlignCritic 가중치를 절반 이하로 줄이고 PathFollowCritic 허용 마진을 넓혀, 전역 경로 복귀 압박 없이 크게 우회할 수 있게 한다. 이때 좌/우 우회 방향은 다음 히스테리시스로 결정한다.

```text
[매 제어 주기, bias_direction ∈ {LEFT, RIGHT, UNDECIDED}]
IF bias_direction == UNDECIDED:
IF |d_right - d_left| > 0.15m (deadband):
bias_direction = RIGHT if d_right > d_left else LEFT; latch_timer = 0
# deadband 이내면 UNDECIDED 유지, 편향 주입 안 함
ELSE:
latch_timer += dt
IF 장애물이 로봇 후방으로 넘어감(상대 bearing ±90° 초과): bias_direction = UNDECIDED
ELIF latch_timer < 2.5s: 그대로 유지(격차가 역전돼도 무시)
ELIF 반대 부호 격차가 0.2m 이상(최초 결정보다 더 큰 임계값): bias_direction 전환, latch_timer = 0
```

deadband(최소 격차)와 latch(최소 유지시간), 비대칭 재전환 임계값 세 요소가 함께 있어야 좌우 여유폭이 거의 대칭인 경계 상황에서 편향 방향 자체가 매 프레임 뒤집히는 재진동을 막을 수 있다 — SEMI_STATIC 속도 히스테리시스(0.15↔0.25m/s)와 동일한 원리다.

### 9.4.1 선택적 플러그인: GRU 기반 비선형 궤적 예측기

단순 등속/등선회(CTRV) 외삽 모델은 보행자의 가속, 감속, 회전, 일시정지와 같은 비선형 행동을 반영하지 못해 예측 호라이즌($t > 1.0\text{s}$)에서 오차가 급증한다. 이를 극복하기 위해 **GRU(Gated Recurrent Unit) 기반 신경망 궤적 예측기**를 선택적 플러그인으로 연동한다.

안정성 보장을 위해 추적(Tracking)과 예측(Prediction) 노드를 물리적으로 분리한다:

```text
[obstacle_tracker_node] (기존, 9.3절)
  → CTRV-EKF로 위치·속도 필터링 + track 수명주기 전담
  → 미래 20스텝 외삽 연산은 분리하여 안정성 격리
        │
        ▼ (필터링된 현재 상태 발행: position, velocity, confidence, status)
[trajectory_predictor_node] (신규 플러그인, PC/ONNX 추론)
  ├─ confidence < 임계값 OR 이력 < 20스텝: 기존 CTRV 등속 외삽 (Fallback)
  └─ confidence 충분 AND 최근 20스텝 이력 완비:
        1) 마지막 관측점 기준 상대좌표(relative displacement)로 정규화
        2) 2-Layer GRU (hidden 48) ONNX 1-shot 순방향 추론 (20스텝 동시 생성)
        3) odom 좌표계로 역변환하여 predicted_trajectory[20] 배열 완성
        │
        ▼ (dynamic_obstacle_msgs/DynamicObstacleArray, 9.6절 메시지 규격 100% 유지)
9.4절 MPPI SpatioTemporalCritic / 9.5절 TTC 스케줄러 (소비 인터페이스 무수정)
```

이 구조의 핵심 장점은 GRU 모델 추론 실패, 메모리 부족, 또는 지연 발생 시에도 기본 `obstacle_tracker_node`는 전혀 영향을 받지 않고 즉각 기존 CTRV 물리 외삽으로 자동 폴백(Graceful Degradation)된다는 점이다.

## 9.5 FALLBACK: TTC 기반 1D 시간 제어 및 M-line 바이패스

Fallback Controller는 predicted_trajectory 배열을 사용하지 않는다. 트래커가 base_link 기준으로 변환해 함께 전달하는 최근접 장애물 1개의 상대 위치(x_rel, y_rel)와 상대 선속도(v_rel_x)만 소비한다.

```text
TTC = x_rel / (-v_rel_x) (단, x_rel > 0, v_rel_x < -0.15 m/s)

1.5s < TTC ≤ 3.0s : 목표 속도를 크립 속도(v_creep = 0.1 m/s)로 선형 감속
TTC ≤ 1.5s : 완전 정지(v_cmd = 0) 후 대기(Yielding)
사람이 지나가 TTC > 3.0s 또는 v_rel_x ≥ 0 이면 기존 K_yaw 복귀 주행 재개

횡방향 nudge(보조): 정지/저속 중 장애물이 정면(|y_rel| < 0.3m)이면, 치우친 반대 방향으로
Δθ_nudge = ±5° 오프셋을 K_yaw 제어기에 일시 주입해 벽 쪽으로 살짝 붙어 길을 터준다.
```

TTC 정지(v_cmd=0)는 5.2절 M-line 진행률 감시기의 정지 판정과 그대로 두면 충돌한다. 이를 막기 위해 M-line 감시기에 is_yielding 입력을 추가하고, Fallback Controller가 TTC로 정지하는 동안은 is_yielding=true로 설정하여 M-line 누적 정지 타이머를 0으로 고정(Pause)한다 — STUCK_RECOVERY(벽면 추종)가 사람을 향해 발동하는 것을 원천적으로 차단한다.

```text
[TTC ≤ 1.5s 감지] → 상태 전이: TTC_YIELDING → M-line 감시기: is_yielding=true (Stuck 타이머 즉시 pause)

Phase 1 (0.0~3.0s) : 완전 정지 대기. 보행자 통과 시 즉시 NORMAL/M-line 주행 재개, 타이머 리셋
Phase 2 (3.0~5.0s) : STANDOFF_NUDGE — 벽면 추종 진입 금지. 제자리에서 여유 폭 방향으로 ±10° 미세 편향 + 경적/LED 신호
Phase 3 (>5.0s) : YIELD_ABORT — 안전 후진(0.3m) 후 Supervisor에 FALLBACK_PATH_BLOCKED 이벤트 발행
```

FALLBACK_PATH_BLOCKED를 수신한 Supervisor는 새로운 전역 계획을 요청하지 않는다(FALLBACK 중에는 planner_server가 활성 계획을 만들지 않으므로). 대신 5.2절에서 이미 캐시해 둔 Global Path 상에서, 막힌 방향을 제외한 다른 Exit Sub-goal 후보(w\*)를 재선정하여 계속 FALLBACK 상태를 유지한 채 진행한다. 별도 인프라를 추가하지 않고 5.2절 메커니즘을 재사용하는 구조다.

## 9.6 인터페이스 정의

모듈 간 결합도를 낮추기 위해 dynamic_obstacle_msgs 패키지를 정의한다.

```text
# dynamic_obstacle_msgs/msg/DynamicObstacle.msg
uint32 id
uint8 status # 0: MOVING, 1: SEMI_STATIC, 2: STATIC
uint8 visibility # 0: TRACKING, 1: COASTING, 2: LOST
geometry_msgs/Point position # odom frame
geometry_msgs/Vector3 velocity # odom frame
float32 radius
float32 confidence # 0~1, track age·매칭 성공률·EKF 공분산 기반 (9.3절)
geometry_msgs/Point[] predicted_trajectory # t=0.1~2.0s, 20 steps (NORMAL/MPPI 소비, 9.4절)
float32[] predicted_major_axis
float32[] predicted_minor_axis
geometry_msgs/Vector3 relative_position # base_link frame (FALLBACK 소비, 9.5절)
geometry_msgs/Vector3 relative_velocity # base_link frame
float32 time_to_collision

# dynamic_obstacle_msgs/msg/DynamicObstacleArray.msg
std_msgs/Header header
DynamicObstacle[] obstacles
```

## 9.7 검증 계획 및 남은 한계

RPi5 CPU 예산: AMCL(상시 ACTIVE) + EKF + Nav2 + Supervisor FSM에 더해 Node 0/1, tracker, MPPI custom critic, BT 노드가 추가되므로, Phase 4 실측 검증에 RPi5 기준 전체 파이프라인 CPU 프로파일링을 KPI로 명시한다(예: MPPI critic 단독 연산 2ms 이내, 전체 인지 파이프라인 제어 주기 20Hz 유지 여부).

ABD 파라미터 사전 검증: ABD의 적응적 breakpoint 임계값은 센서의 각분해능·거리 정확도에 의존한다. 실제 채택 하드웨어인 RPLIDAR C1(각분해능 0.72°, 거리 정확도 ±30mm)의 실측 스캔으로 breakpoint 임계값과 최소 클러스터 크기를 사전 검증해야 Phase 1 KPI(정적 벽면 제거율 95% 등)가 의미 있는 수치가 된다.

남은 한계는 13절 리스크 표에 통합해 관리한다. 다만 (3) 후진 기동 시 후방 라이다 사각지대 여부는 설계가 아니라 센서 마운트 위치 기준 실측으로 먼저 확인해야 하는 항목이라는 점만 덧붙인다 — RPLIDAR C1 자체는 360° 스캔이므로 사각지대가 있다면 기구적 가림이 원인이다.

# 10. 기존 Safety Node의 확장 방향

## 10.1 기존 기능

```text
AMCL localization 이상
↓
Safety Node
↓
STOP
↓
회복 대기
↓
회복 시 재출발
```

## 10.2 제안 기능

```text
AMCL localization 이상
↓
Navigation Supervisor
│
├── 일시적 이상 → STOP / RECOVERY_WAIT
│
└── 지속적 이상
│
├── 특징 충분 → RELOCALIZATION
│
└── 특징 부족 → DEGRADED_NAV
│
├─ Goal Direction
├─ EKF + IMU + Encoder
├─ LiDAR Local Costmap
└─ Drift/Time Limit
│
┌────────────┴────────────┐
│ │
AMCL 회복 한계 초과
│ │
NORMAL FAIL_SAFE
```

이 확장은 Safety Node를 더 이상 단순 “정지 조건 감시기”로만 보지 않고, localization 품질에 따라 Navigation Mode를 선택하는 Supervisor로 정의한다는 의미가 있다.

# 11. ROS 2 구현 아키텍처 및 제어권 전환

## 11.1 Action/BT 흐름 및 Fallback 구현 전략

Nav2는 bt_navigator가 Behavior Tree를 통해 ComputePathToPose와 FollowPath action을 호출하는 구조다. 따라서 Global Planner를 사용하지 않는 상태는 planner_server 프로세스를 죽이는 것이 아니라 현재 NavigateToPose 실행을 cancel하고 Fallback 경로를 별도로 선택하는 것이 단순하다.

## 11.1.1 1차 구현: 완전 우회 방식

캡스톤 1차 구현은 별도 Fallback Controller node를 만들고 NavigateToPose cancel 이후 /cmd_vel_fallback을 publish하는 방식으로 시작한다. twist_mux가 최종 velocity authority를 중재하므로 실제 빈 공간 통과 여부를 빠르게 검증할 수 있다.

- 장점: 구현량이 적고 핵심 가설을 빠르게 검증할 수 있음

- 단점: 최종 구조는 Nav2 plugin보다 덜 통합됨

## 11.1.2 2차 구현: Controller Plugin

동작 검증 후 시간이 허용되면 Fallback Controller를 nav2_core::Controller 기반 plugin으로 리팩터링한다. controller_server 내부에서 normal/fallback controller를 선택하도록 구성하면 /cmd_vel authority가 controller_server 하나로 통합된다.

## 11.1.3 3차 구현: BT 수준 통합

최종 확장에서는 localization 상태를 나타내는 Condition을 BT에 추가하여 NORMAL이면 ComputePathToPose → FollowPath를 사용하고, DEGRADED이면 fallback path/controller 서브트리로 분기한다. 이는 가장 이상적인 구조이나 1차 구현에 필수는 아니다.

## 11.2 twist_mux 기반 Velocity Authority 및 전환 검증

> Safety Stop : priority 255 / Supervisor Brake(래칭) : priority 150 / Fallback : priority 100 / Navigation : priority 10 (상세는 4.7절 표 참조)

Cancel latency 측정은 안전성을 보장하기 위한 고정 대기시간이 아니라, controller_server의 실제 취소 동작과 timeout 설정을 검증하기 위한 실험 데이터로 사용한다.

## 11.3 Lifecycle 관리 원칙

Nav2 Lifecycle Manager가 관리하는 노드들과 AMCL의 개별 lifecycle transition은 혼용하지 않는다. Supervisor가 AMCL을 외부에서 임의로 INACTIVE로 전환하면 lifecycle_manager가 bond 단절을 노드 결함으로 오인해 Nav2 stack 전체를 강제 종료하거나 재시작 루프에 빠뜨릴 위험이 있다(4.6절 참조). 따라서 AMCL은 lifecycle_manager가 관리하는 대로 항상 ACTIVE 상태를 유지하고, Supervisor는 AMCL의 dynamic parameter 서비스(set_parameters)를 통해 tf_broadcast 파라미터만 true/false로 토글하여 map→odom 발행 권한을 제어한다. 이 방식은 lifecycle transition 자체가 없으므로 bond 충돌 위험이 원천적으로 없고, AMCL이 계속 active 상태이므로 RELOCALIZATION 단계에서 covariance 수렴을 즉시 모니터링할 수 있다. Supervisor의 set_parameters 비동기 호출에는 타임아웃 가드(예: 50ms)를 두어, AMCL의 응답이 지연되더라도 Supervisor FSM의 메인 제어 루프가 멈추지 않도록 한다.

## 11.4 ROS 2 노드 / 역할 / 인터페이스

| **노드/모듈**                                       | **역할**                                              | **주요 인터페이스(예시)**                             |
|-----------------------------------------------------|-------------------------------------------------------|-------------------------------------------------------|
| robot_localization EKF                              | IMU + Encoder → odom                                  | sensor topics → /odometry/filtered                    |
| AMCL                                                | map 기준 localization                                 | LaserScan + odometry/TF → map→odom                    |
| Localization Monitor                                | AMCL 신뢰도 모니터링                                  | AMCL pose/covariance 및 내부 통계                     |
| Environment Monitor                                 | LiDAR 환경 특징 계산                                  | /scan → feature score                                 |
| Navigation Supervisor                               | 상태 머신과 모드 전환                                 | confidence + feature + safety → mode                  |
| Fallback Controller                                 | Goal Direction 기반 저수준 주행                       | goal vector + odom + local obstacle → cmd_vel         |
| Local Costmap/Nav2 Controller                       | 실시간 장애물 회피                                    | scan/TF → local trajectory/cmd_vel                    |
| scan_deskewer_node / dynamic_point_extractor(9.2절) | 스캔 데스큐잉 및 동적/정적 포인트 분리(mode-agnostic) | /scan + odom → 동적 후보 포인트, 정적 포인트(→RANSAC) |
| obstacle_tracker_node(9.3절)                        | 장애물별 CTRV-EKF 추적 및 수명주기 관리               | 동적 후보 포인트 → dynamic_obstacle_msgs(9.6절)       |
| MPPI SpatioTemporalCritic(plugin, 9.4절)            | NORMAL 중 동적 장애물 2D 공간 회피 비용 계산          | dynamic_obstacle_msgs → MPPI critic cost              |

# 12. 검증 실험 계획

| **실험군** | **환경**                                       | **목적**                                                            | **주요 지표**                                                    |
|------------|------------------------------------------------|---------------------------------------------------------------------|------------------------------------------------------------------|
| A          | 일반 복도/벽 명확                              | 기존 Nav2 정상성 확인                                               | 도착률, 경로 길이, 주행시간                                      |
| B          | 넓은 빈 공간 5~10 m                            | AMCL confidence 저하 재현                                           | confidence, 정지시간, fallback 진입률                            |
| C          | 넓은 빈 공간 + 이동 장애물                     | Fallback local avoidance 검증                                       | 충돌, 최소 안전거리, 성공률                                      |
| D          | 빈 공간 통과 후 특징 공간                      | AMCL 재획득 검증                                                    | relocalization 시간, pose error                                  |
| E          | 장거리 빈 공간                                 | drift budget 검증                                                   | position/yaw drift, FAIL_SAFE 시점                               |
| F          | 센서 노이즈/일시 장애                          | 오탐 및 상태 플래핑 검증                                            | 불필요한 fallback/stop 횟수                                      |
| G          | NORMAL, 정면 교차 보행자(1.0m/s)               | MPPI SpatioTemporalCritic 2D 회피 검증(9.4절)                       | 최소 이격거리 0.4m 이상, 급정지 없이 우회                        |
| H          | FALLBACK, 보행자 돌발 정지 및 정면 대치        | TTC 스케줄러·M-line 바이패스·mutual standoff 3단계 탈출 검증(9.5절) | 조향 진동 없음, STUCK_RECOVERY 오발동 0건, YIELD_ABORT 정상 발행 |
| I          | 병목 통로(로봇 폭+0.3m 미만) + occlusion(기둥) | 통과 불가 판단·COASTING/LOST 재연관 검증(9.3, 9.5절)                | 무리한 기동 없이 Wait/재계획, 재등장 시 track 재연관 성공률      |

## 12.1 핵심 평가 지표

```text
Navigation Success Rate
Collision Rate
Fallback Entry Rate
Fallback Recovery Rate
Relocalization Time
Position Drift [m]
Yaw Drift [deg]
FAIL_SAFE Trigger Rate
Goal Arrival Error [m]
Minimum Obstacle Clearance [m] (9절)
Track Re-association Success Rate (9.3절)
Mutual Standoff Resolution Time (9.5절)
Average Displacement Error (ADE) [m] (9.4.1절 GRU 궤적 예측, t=0.1~2.0s 전체 20스텝 평균 오차)
Final Displacement Error (FDE) [m] (9.4.1절 GRU 궤적 예측, t=2.0s 종단 위치 오차)
Prediction Accuracy Improvement [%] (held-out 시나리오 CTRV 외삽 대비 GRU의 ADE/FDE 개선율)
```

특히 “Fallback이 성공했는가”만 보지 않고, “AMCL이 불안정한 상황에서 위험한 오동작 대신 안전하게 정지하거나 복귀했는가”를 별도 지표로 평가해야 한다. 또한 동적 장애물 궤적 예측에서는 물리 기반 CTRV 외삽과 신경망 GRU 추론의 ADE/FDE를 동일 held-out 세트에서 1:1로 비교 검증한다.

### 12.1.1 동적 장애물 궤적 예측 정량 벤치마크 실측 결과 (Held-out Test)

총 35개 전역 분산 동적 장애물 환경(Local_0904 월드, 7,824개 궤적 시퀀스)에서 수집된 데이터셋 중 20%의 Held-out 검증 세트(1,565개 시퀀스)를 대상으로 물리 기반 등속(Constant Velocity, CV) 외삽 모델과 제안된 2-Layer GRU 신경망 예측기의 예측 정밀도를 비교 검증한 정량 결과는 다음과 같다.

| 평가 지표 (Metric) | 물리 기반 등속 외삽 (Constant Velocity) | 2-Layer GRU 신경망 예측기 (제안 모델) | 성능 개선율 (Improvement) |
| :--- | :---: | :---: | :---: |
| **ADE (Average Displacement Error)** | 0.346 m | **0.294 m** | **+15.0% 정밀도 향상** |
| **FDE (Final Displacement Error, t=2.0s)** | 0.728 m | **0.577 m** | **+20.7% 정밀도 향상** |
| **추론 지연 시간 (Inference Latency, GPU)** | < 0.05 ms | **1.24 ms (35개 동시 추론)** | 실시간 제어(10Hz, 100ms) 완벽 만족 |
| **비선형 회전/감속 시 종단 오차 감소** | — | 최대 **-0.45 m (오차 45cm 억제)** | 보행자 멈칫거림/회피 조기 감지 |

> **실측 분석 요약**: 보행자가 가감속을 하거나 방향을 꺾을 때, 기존 물리 등속 외삽은 과거 속도 벡터대로 계속 앞으로 돌진한다고 가정하여 $t=2.0\text{s}$ 시점에서 0.728m에 달하는 큰 종단 오차를 유발했다. 반면 제안된 2-Layer GRU 모델은 과거 시계열 곡률과 속도 변화율을 포착하여 종단 오차를 0.577m로 20.7% 대폭 감소시켰으며, 특히 보행자가 감속하거나 멈칫거리는 행동을 신속하게 예측하여 Nav2 MPPI 경로 계획기가 불필요한 과잉 회피나 급정거를 하지 않도록 주행 안전성을 대폭 향상시켰다.


# 13. 한계 및 리스크 분석

| **리스크**                                         | **근본 원인**                                                                    | **대응**                                                                                                                     |
|----------------------------------------------------|----------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------|
| Fallback drift                                     | 절대 localization 부재                                                           | 거리/시간/yaw budget + FAIL_SAFE                                                                                             |
| Goal 방향 오차                                     | IMU yaw drift                                                                    | IMU calibration, yaw 품질 모니터링                                                                                           |
| Goal이 특징 없는 공간 중앙                         | 재획득 기준 없음                                                                 | 랜드마크 또는 추가 absolute observation이 필요한 별도 모드                                                                   |
| 잘못된 Fallback 진입                               | AMCL 단일 지표의 오탐                                                            | AMCL + LiDAR feature 결합                                                                                                    |
| 모드 플래핑                                        | 임계값 근처 반복 변동                                                            | hysteresis + N-consecutive + dwell time                                                                                      |
| TF 불일치                                          | AMCL/EKF authority 충돌                                                          | 명시적 TF ownership 및 state transition                                                                                      |
| Costmap 오정합(Ghost Obstacle)                     | 휠 슬립으로 odom-실제 이동 간 불일치가 raytracing clearing과 어긋남              | Fallback 중 obstacle_layer의 관측 지속시간(decay)을 단축해 오정합 표시를 빠르게 소거                                         |
| 대칭/직선 복도에서 위치 축퇴(Geometric Degeneracy) | 벽에 수직인 방향만 관측되고 진행축은 관측 정보가 없어 covariance가 수렴하지 않음 | 향후 확장(14절급, 핵심 구현 범위 밖): Scan Hessian 고유치로 축퇴 조기 판별 후 관측 가능축만 정합, 진행축은 EKF 예산으로 봉합. ※ 계층 구분 주의: 6.3.1절의 RANSAC 상대각 헤딩 구속은 Dead-reckoning 자체의 정확도(각도 누적 억제)를 높이는 별개 층위의 기법이며, 이 구속으로 인해 인위적으로 작아진 EKF yaw covariance가 4.5절 AMCL 재획득 시 수렴 판정에 유리하게 오작동하지 않는지 실측 검증 필요 |
| Occlusion 재등장 시 낮은 confidence                | grace_buffer 재연관 실패 시 track이 낮은 confidence로 재시작(9.3절)              | 탐색반경을 v_human_max로 상한, T_grace 튜닝                                                                                  |
| FALLBACK 상호 대치(Mutual Standoff)                | 보행자가 로봇 정지를 보고 동시에 멈춰 대치가 장기화될 수 있음                    | 3단계 M-line 바이패스(STANDOFF_NUDGE→YIELD_ABORT, 9.5절)                                                                     |
| 2D 라이다의 근본적 인지 한계                       | 바닥 턱·투명/고반사 유리·지면 경사에서 정적 지형을 동적 장애물로 오인 가능       | 설계로 해소 불가 — 센서 추가 없이는 실측 회귀 테스트로 영향 범위만 파악(9.7절)                                               |

근본적으로 Fallback은 localization 문제를 해결하는 것이 아니라 “localization이 일시적으로 불가능한 구간에서 안전한 통과를 가능하게 하는 것”이다. 따라서 빈 공간이 drift budget보다 크거나 목표가 특징 없는 공간에 장시간 존재한다면 정확한 도착을 보장하지 않고 FAIL_SAFE로 전환하는 것이 설계상 올바른 종단 조건이다.

# 14. 확장안: 특징 부족 구역의 절대 위치 보조

현재 보유한 fisheye 카메라와 AprilTag 도킹 파이프라인을 재활용할 수 있다면, 특징이 부족한 구간에 소수의 AprilTag landmark를 배치하고 이를 보조 관측으로 사용하는 확장도 가능하다. 이 방식은 새로운 센서를 추가하기보다 기존 카메라/비전 파이프라인을 확장한다는 장점이 있다.

```text
Fisheye Camera → AprilTag Pose → Absolute/Relative Observation
IMU + Encoder → EKF
↓
안정화된 pose estimate
↓
Fallback 구간의 drift 보정
```

다만 이 확장은 현재의 핵심 목표인 “센서만으로 빈 공간을 통과”보다 복잡하다. 1단계에서는 Goal Direction + LiDAR + EKF 기반 Fallback을 먼저 검증하고, 실제 drift 한계가 문제로 확인될 때 AprilTag 보조 관측을 추가하는 것이 개발 순서상 합리적이다.

유사하게, 대칭적인 직선 복도처럼 관측 방향이 편향된 환경에서는 AMCL 공분산이 진행축으로만 길게 늘어져 수렴하지 않을 수 있다(13절 참조). 이 경우 LiDAR 스캔의 Fisher Information Matrix 고유치로 축퇴를 조기 판별하고, 관측 가능한 축만 지도에 정합한 뒤 관측 불가능한 축은 EKF 드리프트 예산으로 제한하는 SEMI_DEGRADED_WALL_ALIGN 모드를 추가할 수 있다. 다만 이는 AMCL이 기본 제공하지 않는 별도의 scan-matching 기반 축퇴 검출 모듈을 새로 구현해야 하므로, AprilTag 보조 관측과 마찬가지로 핵심 구현(1~8단계) 이후의 선택적 확장으로 둔다.

## 14.1 동적 장애물 도메인 랜덤화 및 Sim-to-Real Gap 한계

9.4.1절의 GRU 궤적 예측기는 시뮬레이션 환경에서 학습 데이터를 수집하므로, 실제 현실 보행자 행동과의 간극(Sim-to-Real Gap)을 줄이기 위한 도메인 랜덤화(Domain Randomization)가 필수적이다.

1. **시뮬레이션 도메인 랜덤화 설계**:
   - **경로 무작위성**: 단순 2점 직선 왕복을 탈피하고, 진행축 법선 방향으로 $\pm 0.45\text{m}$의 무작위 횡방향 지터(Jitter)를 갖는 다중 중간 경유지(Waypoint)를 매 왕복 세그먼트마다 동적 생성하여 $\pm 15^\circ \sim 60^\circ$의 급격한 꺾임 궤적을 학습 데이터에 포함.
   - **속도 및 가속도 프로파일**: 보행자 보행 속도를 $0.35 \sim 1.15\text{ m/s}$ 범위에서 균등 분포로 무작위 배정하고, 최대 가속도($1.2\text{ m/s}^2$) 기반 슬루율 필터링을 적용해 비선형 가감속을 유도.
   - **일시정지 및 멈칫거림(Stuttering)**: 웨이포인트 도달 시 $35\%$의 확률로 $0.5 \sim 2.5\text{s}$ 동안 대기(Pause)하도록 하여, 보행자가 스마트폰을 보거나 방향을 살피며 서성이는 실내 거동을 모사.

2. **Sim-to-Real Gap의 근본적 한계와 완화 방안**:
   - **보행 기구학적 차이**: Gazebo의 2D 평면 원기둥 모델(`planar_move`)은 발걸음 진동(Gait Oscillation)이나 골반 회전이 없는 완벽한 강체 이동이다. 실제 2D LiDAR는 다리의 전후 교차 이동으로 인해 관측 클러스터의 중심이 $1\sim 2\text{Hz}$로 미세 진동(Micro-motion)하므로, EKF 트래커의 전처리 노이즈 필터링이 필수적이다.
   - **사회적 상호작용(Social Force) 부재**: 시뮬레이션 동적 장애물은 로봇의 접근에 반응하지 않고 독립적으로 움직인다. 실제 환경의 보행자는 로봇을 보고 멈추거나 피하는 상호작용(Social Interaction)을 보이므로, 추후 실차 환경 배포 시 실제 휠체어 주행 로그를 수집하여 모델을 미세조정(Fine-tuning)하거나 Social Force Critic과 상호 보완하도록 설계해야 한다.
   - **Ground Truth 분리 원칙**: 학습 파이프라인에서 정답 라벨링에 쓰인 Gazebo의 무결점 시뮬레이터 좌표(`/<name>/odom`, `libgazebo_ros_planar_move`)는 오프라인 학습/평가용으로만 격리되며, 실차 배포 코드에는 일절 포함되지 않고 순수 LiDAR 추적기(`obstacle_tracker_node`)의 관측 이력만을 입력으로 사용한다.

# 15. 구현 / 개발 순서 (확정안)

개발은 핵심 가설을 먼저 검증한 뒤 통합도를 높이는 방식으로 진행한다. 우선 “넓은 빈 공간에서 AMCL을 계속 유지하지 않아도 안전하게 목적지 방향으로 통과할 수 있는가?”를 증명하고, 이후 FSM·relocalization·plugin 통합을 단계적으로 추가한다.

| **단계** | **구현 항목**                  | **완료 조건**                                            |
|----------|--------------------------------|----------------------------------------------------------|
| 1        | 기존 Safety Node baseline 재현 | 정상 및 AMCL failure 시 기존 동작 log 확보               |
| 2        | Goal 보존 구현                 | Supervisor가 map-frame preserved_goal 유지               |
| 3        | NavigateToPose cancel 검증     | cancel 후 controller_server 동작과 정지 확인             |
| 4        | twist_mux 도입                 | navigation/fallback/safety priority 동작 확인            |
| 5        | Fallback Controller 최소 구현  | Goal Direction + LiDAR로 빈 공간 통과                    |
| 6        | IMU + Encoder EKF 연계         | 상대 이동량과 heading 입력 확보                          |
| 7        | Drift Budget 측정              | 거리·시간·yaw별 오차를 측정해 한계 산정                  |
| 8        | Supervisor FSM 구현            | 5개 상태 및 hysteresis 동작 확인                         |
| 9        | AMCL reseeding                 | 추정 pose를 /initialpose로 전달하고 재획득               |
| 10       | TF authority handover          | Supervisor ↔ AMCL 단일 publisher 전환 확인               |
| 11       | NORMAL 복귀                    | AMCL 안정 → TF 복귀 → fallback 종료 → goal 재전송        |
| 12       | Stress test                    | relocalization 실패·flapping·drift 초과·장애물 상황 검증 |
| 13       | 선택적 고도화                  | Controller plugin → BT 통합 순으로 리팩터링              |

## 15.1 최소 구현 범위

시간 제약을 고려한 최소 성공 범위는 1~8단계다. 특히 5단계에서 Gazebo의 feature-poor 구간을 실제로 통과하는 것이 핵심 proof-of-concept다. 9~12단계는 시스템을 복원력 있는 Navigation으로 완성하는 단계이며, 13단계는 일정이 허용될 때 진행한다.

## 15.2 구현 의존성

> Goal 보존  
> ↓  
> NavigateToPose cancel  
> ↓  
> twist_mux authority  
> ↓  
> Fallback Controller  
> ↓  
> EKF / Drift Budget  
> ↓  
> Supervisor FSM  
> ↓  
> AMCL reseeding  
> ↓  
> TF handover  
> ↓  
> NORMAL 복귀  
> ↓  
> Controller Plugin / BT (선택)

9절(동적 장애물 예측 및 회피)은 이 13단계와 별도 트랙으로 병행 개발한다. 9.2/9.3(전처리·추적)은 Supervisor나 Fallback Controller에 의존하지 않는 mode-agnostic 구성이므로 1단계와 동시에 시작할 수 있다. 9.4(MPPI Critic)는 기본 Nav2 controller가 있으면 되므로 1단계 이후 착수 가능하다. 9.5(TTC 스케줄러·M-line 바이패스)는 Fallback Controller(5단계)와 Supervisor FSM(8단계)의 존재를 전제하므로 그 이후에 통합한다. 12단계 Stress test에는 9절의 실험군 G/H/I(12절)를 포함한다.

# 16. 기대 효과

- 넓은 빈 공간에서 AMCL이 구조적으로 불리한 상황을 단순 정지 문제에서 “제한된 기간의 Degraded Navigation 문제”로 재정의할 수 있다.

- 기존 Map·Goal·Nav2 구조를 유지하면서 Localization failure에 대한 복원력을 높일 수 있다.

- IMU와 Encoder가 제공하는 상대 운동 정보를 사용하므로 GPS가 필요 없는 실내 구조를 유지할 수 있다.

- LiDAR Local Costmap을 Fallback에서도 유지하여 “목표 방향만 보고 직진”하는 위험한 설계를 피할 수 있다.

- Fallback 한계를 명시하고 FAIL_SAFE를 두어 정확성 실패를 안전성 실패로 확산시키지 않는 구조를 확보할 수 있다.

- 향후 Nav2 Auto Tuner가 파라미터를 평가할 때 정상 환경뿐 아니라 localization degradation 시나리오까지 포함한 강건성 평가로 확장할 수 있다.

Localization 상태와 무관하게 동작하는 동적 장애물 인지·회피(9절)를 통해, NORMAL에서는 예측 기반 2D 우회를, FALLBACK에서는 TTC 기반 양보를 일관되게 제공하여 사람이 오가는 실내 환경에서의 안전성을 함께 확보할 수 있다.

# 17. 최종 설계 요약

```text
┌───────────────────────────────┐
│ NORMAL NAV │
│ Map + AMCL + Nav2 + Costmap │
└───────────────┬───────────────┘
│
AMCL Confidence ↓
│
▼
┌────────────────────────┐
│ Localization Supervisor│
└────────────┬───────────┘
│
┌────────────┴────────────┐
│ │
일시적 이상 지속적 + 특징 부족
│ │
▼ ▼
RECOVERY_WAIT DEGRADED_NAV
│
┌───────────────┼──────────────┐
│ │ │
Goal Direction EKF LiDAR
│ IMU+Encoder Local Costmap
└───────────────┬──────────────┘
│
┌───────────┴───────────┐
│ │
특징 공간 진입 drift/시간 한계
│ │
▼ ▼
RELOCALIZATION FAIL_SAFE
│
▼
NORMAL NAV
```

최종적으로 이 시스템은 “AMCL이 항상 정확해야 Navigation을 할 수 있다”는 전제를 완화한다. 정상 공간에서는 절대 localization을 적극 사용하고, localization이 어려운 빈 공간에서는 필요한 만큼만 상대 이동과 센서 기반 회피로 통과하며, 회복 불가능한 경우에는 명확한 한계에서 정지한다. 따라서 핵심 설계 철학은 Mapless Navigation이 아니라 **Localization-Resilient Navigation**이다.

이 상태 머신과 별도로, 동적 장애물 인지·회피(9절)는 위 다이어그램의 모든 상태에 걸쳐 항상 동일하게 동작하는 병행 계층이다 — map이나 Supervisor 상태를 참조하지 않으므로 어떤 Navigation Mode에 있든 일관되게 보행자·카트를 예측하고 회피한다.

# 18. 결론 및 다음 단계

본 설계는 AMCL이 항상 정확해야 Navigation이 가능하다는 전제를 완화한다. 정상 환경에서는 기존 Map + AMCL + Nav2를 그대로 사용하고, 넓은 빈 공간에서 localization이 구조적으로 어려워지면 Supervisor가 FALLBACK으로 전환한다. Goal은 Supervisor가 preserved_goal로 보존하고, 정상 NavigateToPose는 cancel하며, twist_mux를 통해 Fallback Controller가 velocity authority를 획득한다.

Fallback에서는 AMCL 대신 IMU·Encoder 기반 상대 운동과 LiDAR 기반 장애물 회피를 사용한다. 특징이 충분한 공간에 다시 진입하면 정지 → dead-reckoning 기반 /initialpose seeding → AMCL 활성화 → N회 연속 수렴 확인 → frozen TF 종료 → AMCL authority 복귀 → preserved_goal 재전송의 순서로 정상 Navigation으로 복귀한다. 재획득 실패가 반복되거나 drift budget을 초과하면 FAIL_SAFE로 정지한다.

이와 별개로 동적 장애물 예측 및 회피(9절)를 통해 Navigation Mode와 무관하게 보행자·카트를 인지·추적하고, NORMAL에서는 MPPI 기반 2D 공간 회피를, FALLBACK에서는 TTC 기반 시간 제어와 M-line 바이패스를 적용하여 사람이 오가는 실내 환경에서도 안전하게 동작하도록 했다.

# 참고 기술 스택 및 문서 범위

본 문서는 프로젝트에서 사용 중인 ROS 2 Humble, Nav2, AMCL, robot_localization EKF, LiDAR, IMU, Wheel Encoder 구성 및 동적 장애물 인지·추적을 위한 CTRV-EKF, MPPI custom critic 구성을 기준으로 작성했다. 실제 구현 단계에서는 사용 중인 패키지 버전과 현재 Safety Node의 topic/TF 구조를 기준으로 인터페이스 명칭과 메시지 필드를 확정해야 한다.
