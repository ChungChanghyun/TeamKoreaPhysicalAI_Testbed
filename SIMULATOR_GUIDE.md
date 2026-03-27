# TAPG 기반 AGV 시뮬레이터 구조 가이드

## 1. 전체 구조

```
CBS 플래너 (PklPlanner)
    │  raw_path: [(state_id, cbs_time), ...]
    ▼
TAPGAgent ──── 위치·속도·상태를 보유하는 에이전트 객체
    │
    ▼
TAPGEnvironment        (시간 기반 실행 엔진)
TAPGEnvironmentDES     (이벤트 기반 실행 엔진)
    │
    ├── TAPG 그래프 (G: nx.DiGraph)  ← 충돌 방지 의존성
    ├── 이벤트 큐 (_eq: heapq)
    └── agent.x / agent.y / agent.theta / agent.v  ← 시각화 공유
```

CBS 플래너가 생성한 경로를 TAPG 환경이 실행합니다.
에이전트 객체는 환경과 시각화가 **같은 인스턴스를 공유**하므로,
환경이 갱신하면 렌더러는 별도 복사 없이 즉시 그릴 수 있습니다.

---

## 2. 에이전트 상태 머신

```
        TAPG 선행조건 미충족
IDLE ──────────────────────► WAITING
 ▲                               │
 │   선행조건 충족 (깨어남)         │
 └───────────────────────────────┘

IDLE ──► MOVING   (M 상태 시작)
IDLE ──► ROTATING (R 상태 시작)

MOVING   ──► IDLE (엣지 끝 도달)
ROTATING ──► IDLE (회전 완료)

IDLE (경로 끝) ──► DONE
```

| 상태 | 의미 |
|---|---|
| `IDLE` | 다음 액션 결정 대기 |
| `MOVING` | 엣지를 따라 이동 중 |
| `ROTATING` | 제자리 회전 중 |
| `WAITING` | TAPG 선행조건 대기 (다른 에이전트) |
| `DONE` | 목적지 도착 완료 |

---

## 3. 두 가지 실행 엔진

### 3-1. Time-Stepped (TAPGEnvironment)

매 프레임 고정 시간 간격 `dt`로 시뮬레이션을 전진시킵니다.

```
while not done:
    sim_time += dt          # 예: 1/60 s (60 FPS)
    env.step(sim_time)
        └── _update_positions(dt)   ← 모든 에이전트를 dt만큼 물리적으로 이동
```

`_update_positions` 내부에서 매 dt마다:
- 속도 계산 (가속/감속)
- `dist_traveled += v * dt`
- `dist_traveled >= edge_length` 이면 도착 처리

**특징**
- 구조가 단순함
- 시뮬레이션 속도가 FPS에 묶임 (빠르게 돌리려면 dt를 크게 → 정밀도 저하)
- dt가 너무 크면 도착 감지 타이밍 오차 발생 가능

---

### 3-2. Discrete Event Simulation (TAPGEnvironmentDES)

이동 완료 시각을 **미리 계산**하고 이벤트로 예약합니다.

```
while not done:
    sim_time = next_event.time    # 다음 이벤트 시각으로 점프
    env.step(sim_time)
        └── 이벤트 처리만 수행 (dt 루프 없음)
        └── _update_visuals()     ← 공식으로 O(1) 위치 계산
```

이동 시작 시:
```
SpeedProfile 생성 (t_total 계산)
    └── EDGE_DONE 이벤트 예약 @ cur_time + t_total
    └── CHECK_CHAIN 이벤트 예약 @ cur_time + t_brake  (유한 감속일 때)
```

**특징**
- 이동 중간에 dt 루프 없음 → 임의 속도로 시뮬레이션 가능
- 시각화 시: `_update_visuals`가 현재 시각의 위치를 공식으로 즉시 계산
- 배치 실험 시: 이벤트 점프로 수백 초 시뮬을 수 ms에 완료

---

## 4. 속도 프로파일 (SpeedProfile)

AGV는 세 구간으로 나뉜 **사다리꼴 속도 프로파일**을 따릅니다.

```
  v
  │        ┌──────────┐
v_peak     │ 정속 구간  │
  │       /│           │\
v_init   / │           │ \
  │     /  │           │  \
  └────────────────────────── t
       t1  │           t2  t_total
   가속 구간           감속 구간
```

| 구간 | 조건 | 거리 | 시간 |
|---|---|---|---|
| 가속 | `v_init → v_peak` | `(v_peak² - v_init²) / (2a)` | `(v_peak - v_init) / a` |
| 정속 | `v_peak` 유지 | 나머지 | `d_cruise / v_peak` |
| 감속 | `v_peak → 0` | `v_peak² / (2d)` | `v_peak / d` |

엣지 길이가 짧아 `v_peak`에 도달할 수 없으면 삼각형 프로파일로 자동 전환됩니다.

### accel/decel = ∞ (기본값)

```
v │  v_max ──────────────
  │
  └──────────────────── t
  (즉시 최고속도, 즉시 정지)
```

`t_total = edge_length / v_max`

---

## 5. 이벤트 종류

| 이벤트 | 발생 시점 | 처리 내용 |
|---|---|---|
| `TRY_ADVANCE` | 에이전트가 IDLE로 전환될 때 | S 상태 통과, 다음 M/R 액션 시작 시도 |
| `EDGE_DONE` | `cur_time + SpeedProfile.t_total` | 에이전트를 목적 노드로 스냅, 다음 TRY_ADVANCE 예약 |
| `ROTATE_DONE` | `cur_time + angle / ω` | 회전 완료, 다음 TRY_ADVANCE 예약 |
| `CHECK_CHAIN` | `cur_time + SpeedProfile.t2` (제동 시작 시점) | 연속 주행 가능 여부 확인 |

이벤트는 `heapq` 우선순위 큐에 `(time, seq)` 기준으로 정렬됩니다.
같은 시각의 이벤트는 발행 순서(`seq`)로 처리됩니다.

---

## 6. 연속 주행 (Continuous Driving)

### 문제

다음 엣지가 이미 클레임 가능한 상태일 때도 현재 엣지에서 감속 후 재출발하면
**불필요한 정지**가 발생합니다.

```
[엣지 A] ──── 감속 ──→ 정지 ──→ 가속 ──── [엣지 B]
                 ↑ 불필요
```

### 해결: CHECK_CHAIN 이벤트

제동 시작 시점(`t2`)에 다음 엣지 claimability를 확인합니다:

```
CHECK_CHAIN 발생 시:

  다음 엣지 claimable?
      │
      ├── No  → 아무것도 안 함 (원래 EDGE_DONE 대로 감속)
      │
      └── Yes → 패스스루 처리:
                  1) 현재 거리를 오프셋으로 저장
                  2) v_peak 일정 속도로 남은 d_dec 를 통과
                  3) 새 EDGE_DONE 재예약 (더 이른 시각)
                  4) 도착 시 chain_carry = v_peak (체이닝용)
```

**결과**

```
[엣지 A] ──── v_peak 유지 ──────────── [엣지 B]
                 ↑ 감속 없이 통과
```

### stale 이벤트 처리

CHECK_CHAIN이 EDGE_DONE을 재예약할 때 기존 EDGE_DONE은 무효화해야 합니다.
`token` 딕셔너리로 처리합니다:

```
_start_move:    token[agent_id] = N    → EDGE_DONE(token=N) 예약
CHECK_CHAIN:    token[agent_id] = N+1  → 새 EDGE_DONE(token=N+1) 예약
기존 EDGE_DONE: token 불일치 → 무시
```

---

## 7. 시각화 위치 계산 방식 비교

### Time-Stepped

```python
# 매 프레임 dt 누적
agent.dist_traveled += agent.v * dt
frac = agent.dist_traveled / agent.edge_length
agent.x = from_x + (to_x - from_x) * frac
```

### DES

```python
# 공식으로 O(1) 계산 (루프 없음)
elapsed = sim_time - move_start_time
d       = move_offset + profile.get_position(elapsed)
frac    = d / agent.edge_length
agent.x = from_x + (to_x - from_x) * frac
```

`get_position(elapsed)` 는 현재 구간(가속/정속/감속)에 따라 수식을 선택합니다:

| 구간 | 위치 공식 |
|---|---|
| 가속 (`elapsed ≤ t1`) | `v_init·t + ½·a·t²` |
| 정속 (`t1 < elapsed ≤ t2`) | `d_acc + v_peak·(t - t1)` |
| 감속 (`t2 < elapsed ≤ t_total`) | `d_acc + d_cru + v_peak·Δt - ½·d·Δt²` |

---

## 8. 시나리오별 권장 구성

| 사용 목적 | 엔진 | step 전략 | 비고 |
|---|---|---|---|
| 1x 실시간 시각화 | TimeStep 또는 DES | `dt = 1/FPS` | 차이 없음 |
| 고속 재생 (10x~) 시각화 | **DES** | `dt = real_dt × speed` | 큰 dt에서도 정확 |
| 배치 실험 / 통계 수집 | **DES** | 이벤트 점프 | 수백 배 빠름 |

---

## 9. 관련 파일

| 파일 | 역할 |
|---|---|
| `env_tapg.py` | Time-Stepped 실행 엔진 + TAPGAgent 정의 |
| `env_tapg_des.py` | DES 실행 엔진 (SpeedProfile, CHECK_CHAIN) |
| `planner.py` | CBS+SIPP 플래너 어댑터 |
| `pkl_loader.py` | 충돌 프로파일 pkl → MapGraph 변환 |
| `vis_tapg.py` | Pygame 시각화 (T 키로 엔진 전환) |
| `bench_tapg.py` | TimeStep vs DES 수행 시간 비교 |
