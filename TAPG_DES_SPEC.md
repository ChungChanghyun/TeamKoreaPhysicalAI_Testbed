# TAPG DES (Discrete Event Simulation) 명세서

> 이 문서는 `env_tapg.py`에 구현된 **TAPG(Temporal Action Precedence Graph) 기반 DES 엔진**의
> 설계를 정리한 것입니다.
> 코드를 읽지 않은 사람이 이 문서만으로 동일한 시스템을 새로 구현할 수 있도록 작성되었습니다.

---

## 1. 시스템 개요

TAPG DES는 **다수의 AGV(무인 반송차)가 사전 계획된 경로를 따라 이동할 때, 충돌 없이 안전하게 실행되도록 보장하는 실행 엔진**입니다.

### 핵심 아이디어

1. CBS/SIPP 플래너가 각 에이전트의 경로를 **시간 기반(time-indexed)** 상태 시퀀스로 계획합니다.
2. 이 경로들을 **DAG(Directed Acyclic Graph)** 로 변환합니다.
   - 같은 에이전트 내: **순차 엣지** (이전 상태 → 다음 상태)
   - 다른 에이전트 간: **교차 엣지** (공간 충돌이 있는 상태 쌍에 선행 관계 부여)
3. 실행 시 각 에이전트는 **자신의 cross-agent 선행 노드가 모두 완료된 후에만** 다음 액션을 실행합니다.
4. 이로써 **CBS 계획 시간과 실제 실행 시간이 달라도 충돌이 발생하지 않습니다.**

---

## 2. 상태(State) 정의

에이전트의 경로는 세 가지 상태 타입의 시퀀스입니다:

| 타입 | State ID 형식 | 설명 | 물리적 동작 |
|------|--------------|------|-----------|
| **S** (Stop) | `S,node_id,angle` | 특정 노드에 특정 각도로 정지 | 정지 상태 (대기 포함) |
| **M** (Move) | `M,from_node,to_node` | 노드 간 직선 이동 | 가속→순항→감속 주행 |
| **R** (Rotate) | `R,node_id,from_angle,to_angle` | 제자리 회전 | 각속도 기반 회전 |

### 경로 예시

```
A2의 경로: na.189 → na.25 → na.33 → na.34 (일부)
─────────────────────────────────────────────────
[0] S,na.189,0        t=394.69   ← 시작: na.189에 0도로 정지
[1] M,na.189,na.25    t=394.69   ← na.189→na.25 이동
[2] S,na.25,0         t=410.59   ← na.25 도착, 0도 정지
[3] R,na.25,0,90      t=410.59   ← 제자리 0°→90° 회전
[4] S,na.25,90        t=411.59   ← 회전 완료, 90도 정지
[5] M,na.25,na.33     t=411.59   ← na.25→na.33 이동
[6] S,na.33,90        t=412.97   ← na.33 도착
...
```

- `t` 값은 CBS 플래너가 계산한 **계획 시간**입니다.
- 실제 실행 시간은 물리 시뮬레이션에 의해 달라질 수 있습니다.

### 상태 전이 패턴

경로는 항상 다음 패턴을 따릅니다:

```
S → M → S → [R → S →] M → S → [R → S →] M → S → ...
```

- M(이동) 전후에는 반드시 S(정지)가 존재
- R(회전)은 선택적이며, 전후에 S가 존재
- 경로의 첫 상태와 마지막 상태는 항상 S

---

## 3. TAPG (Temporal Action Precedence Graph)

### 3.1 노드

TAPG의 각 노드는 하나의 상태 인스턴스를 나타냅니다:

```
TAPG 노드 키 = (state_id, agent_id, cbs_start_time)
예: ('M,na.29,na.20', 2, 410.453)
```

노드 속성:
- `agv_id`: 소속 에이전트 ID
- `start_time`: CBS 계획 시작 시간
- `duration`: 이 상태의 소요 시간 (마지막 상태는 ∞)

### 3.2 순차 엣지 (Sequential Edge)

같은 에이전트의 연속된 상태를 연결합니다:

```
(S,na.189,0, A2, 394.69) → (M,na.189,na.25, A2, 394.69) → (S,na.25,0, A2, 410.59) → ...
```

### 3.3 교차 엣지 (Cross-Agent Edge)

**서로 다른 에이전트**의 상태 간 공간 충돌이 있을 때 선행 관계를 설정합니다.

#### Affect State (영향 상태)

각 상태는 `affect_state` 집합을 가집니다. 이것은 **해당 상태가 실행 중일 때 물리적으로 충돌하는 다른 상태들의 목록**입니다.

- 예: `M,na.14,na.20`의 affect_state에 `S,na.20,180`이 포함 → 이 Move가 실행 중이면 해당 Stop 위치와 충돌

이 affect_state는 차량 크기를 고려한 충돌 프로파일(`.pkl`)에서 사전 계산됩니다.

#### 교차 엣지 빌드 알고리즘

```
for 에이전트 i의 각 상태 s1 (역순):
    for 에이전트 j의 각 상태 s2 (정순):
        if s2.time <= s1.time:
            continue  (시간 순서 조건)
        if s2 ∈ s1.affect_state  OR  s1 ∈ s2.affect_state:
            엣지 추가: s1 → s2  (s1이 완료되어야 s2 실행 가능)
            break  (j 경로에서 첫 충돌만 연결)
```

**핵심 규칙:**
- 시간 조건: `t2 > t1` (CBS 계획 시간 기준, 나중 것만)
- 충돌 조건: 양방향 affect_state 확인 (s2 ∈ affect(s1) OR s1 ∈ affect(s2))
- 최소 연결: 에이전트 j의 경로에서 **가장 이른 충돌 상태 하나**만 연결 (break)

#### 교차 엣지의 의미

```
(M,na.29,na.20, A2, 410.45) → (M,na.14,na.20, A5, 416.08)
```

= "A2의 na.29→na.20 이동이 완료되어야 A5의 na.14→na.20 이동을 시작할 수 있다"

---

## 4. DES 이벤트 정의

### 4.1 Event 구조체

```
Event {
    time:      float    — 이벤트 발생 시뮬레이션 시각
    seq:       int      — 동시각 이벤트의 FIFO 순서
    kind:      str      — 이벤트 종류
    agent_id:  int      — 대상 에이전트
    data:      dict     — 추가 데이터
}
```

이벤트는 `(time, seq)` 기준 min-heap(우선순위 큐)으로 관리됩니다.

### 4.2 이벤트 종류

이 시스템에는 **명시적 이벤트가 단 하나**입니다:

| 이벤트 | 트리거 조건 | 설명 |
|--------|-----------|------|
| **TRY_ADVANCE** | 초기화 시, 액션 완료 시, 대기 해제 시 | 에이전트가 다음 상태로 진행을 시도 |

나머지 상태 전이(이동 완료, 회전 완료)는 **이벤트가 아닌 위치/각도 기반 센서 감지**로 처리됩니다.

---

## 5. 에이전트 상태 머신

```
         ┌──────────────────────────────────────────────┐
         │                                              │
         ▼                                              │
     ┌──────┐    TRY_ADVANCE     ┌─────────┐     도달 감지
     │ IDLE │ ──── (M 실행) ────▶│ MOVING  │ ─────────┘
     │      │ ──── (R 실행) ────▶│ROTATING │ ─────────┘
     │      │                    └─────────┘
     │      │    TRY_ADVANCE
     │      │ ── (선행조건 ✗) ──▶┌─────────┐
     │      │                    │ WAITING │
     └──────┘                    │         │
         ▲                       └────┬────┘
         │     선행노드 완료            │
         └─────────────────────────────┘

     ┌──────┐
     │ DONE │  ← 경로 끝 도달
     └──────┘
```

| 상태 | 설명 |
|------|------|
| **IDLE** | 다음 액션 진행 준비 완료. TRY_ADVANCE를 처리할 수 있음 |
| **MOVING** | M 상태 실행 중. 물리 엔진이 위치를 갱신. 도착 노드에 도달하면 완료 |
| **ROTATING** | R 상태 실행 중. 물리 엔진이 각도를 갱신. 목표 각도에 도달하면 완료 |
| **WAITING** | 다음 상태의 TAPG 선행 조건 미충족. 선행 노드 완료 시 깨어남 |
| **DONE** | 경로의 모든 상태를 실행 완료 |

---

## 6. TRY_ADVANCE 이벤트 처리 상세

TRY_ADVANCE는 이 시스템의 핵심 로직입니다. 에이전트가 경로의 다음 상태로 진행을 시도합니다.

### 6.1 처리 흐름

```
TRY_ADVANCE(agent) 수신
│
├─ agent.state가 MOVING/ROTATING/DONE이면 → 무시 (이미 실행 중)
│
├─ 현재 상태가 S(Stop)인 동안 반복:
│   ├─ 위치/각도를 해당 노드로 스냅
│   ├─ 다음 상태가 M/R이면:
│   │   ├─ 이미 claimed 범위 내 → 통과
│   │   └─ claimed 범위 밖 → claimable 체크
│   │       ├─ claimable ✓ → claim 확장 후 통과
│   │       └─ claimable ✗ → 루프 중단 (WAITING 진입)
│   └─ path_idx 증가
│
├─ path_idx ≥ 경로 길이 → DONE 상태 전환
│
├─ 현재 상태가 M/R이면:
│   ├─ claimed 범위 내 → 무조건 실행 (_start_move / _start_rotate)
│   └─ claimed 범위 밖:
│       ├─ claimable ✓ → claim 확장 → 실행
│       └─ claimable ✗ → WAITING 진입 + wait_queue 등록
│
└─ 끝
```

### 6.2 S 상태의 특수 처리

S(Stop) 상태는 **DES 이벤트로 실행되지 않습니다.** TRY_ADVANCE 내에서 즉시 통과(inline skip)됩니다:
- 에이전트 위치를 해당 노드 좌표로 스냅
- 에이전트 각도를 해당 S 상태의 각도로 설정
- path_idx를 증가시키고 다음 상태 확인

이는 S 상태가 "정지해 있다"는 것이 물리적 동작이 아니라 **상태 선언**이기 때문입니다.

---

## 7. Claim (확정 구간) 메커니즘

### 7.1 개념

`claim_idx`는 **에이전트가 실행을 확정한 경로 구간의 끝 인덱스**(exclusive)입니다.

```
경로:  [S] [M] [S] [R] [S] [M] [S] [M] [S]
인덱스: 0   1   2   3   4   5   6   7   8
              ▲               ▲
          path_idx=1      claim_idx=5
          (현재 실행 중)    (여기까지 확정)
```

- `path_idx ~ claim_idx` 구간은 **이미 TAPG 조건을 통과한 확정 구간**
- 이 구간의 액션은 추가 체크 없이 무조건 실행
- claim 범위 밖의 액션은 매번 `_is_claimable` 체크 필요

### 7.2 Claim 확장 (_try_claim_next)

다음 M/R 액션 실행 전, claim 범위를 확장합니다:

```
1. 현재 claim_idx부터 다음 M/R을 탐색
2. 해당 M/R의 TAPG 노드가 claimable인지 확인
3. claimable이면: M/R + 그 다음 S까지 claim_idx 확장
4. claimable 아니면: 확장 실패 → WAITING 진입
```

### 7.3 Claimable 판정 (_is_claimable)

```python
def _is_claimable(nk, agent_id):
    """TAPG 노드 nk의 cross-agent 선행 노드가 모두 DAG에서 제거되었으면 True"""
    for pred in DAG.predecessors(nk):
        if pred.agent_id != agent_id:  # 다른 에이전트의 선행 노드
            return False               # 아직 DAG에 존재 = 미완료
    return True
```

즉, **다른 에이전트의 선행 작업이 모두 완료(DAG에서 제거)된 상태**여야 claim 가능합니다.

---

## 8. 물리 실행 엔진

M/R 상태는 TRY_ADVANCE에서 시작되고, **매 프레임 `_update_positions(dt)`에서 물리적으로 갱신**됩니다.

### 8.1 Move 실행 (_start_move → _update_positions)

```
시작 조건: TRY_ADVANCE에서 M 상태 claim 성공
│
├─ 출발 노드/도착 노드 좌표 설정
├─ 엣지 길이(edge_length), 최고 속도(max_speed) 로드
├─ 초기 속도 설정:
│   ├─ 무한 가속도 모드: v = max_speed (즉시 최고속)
│   ├─ 유한 가속도 모드: v = 0 (정지에서 출발)
│   └─ 연속 주행 모드: v = 이전 엣지 종료 속도
├─ agent.state = MOVING
│
└─ 매 프레임 _update_positions(dt):
    ├─ 사다리꼴 속도 프로파일 적용:
    │   ├─ 가속 구간: v < max_speed → v += accel × dt
    │   ├─ 순항 구간: v = max_speed 유지
    │   └─ 감속 구간: 잔여거리 ≤ 제동거리 → v -= decel × dt
    ├─ 이동 거리 누적: dist_traveled += v × dt
    ├─ 보간 위치 계산: (x, y) = lerp(from, to, dist/edge_length)
    └─ 도달 판정: dist_traveled ≥ edge_length → 도착 처리
```

### 8.2 Rotate 실행 (_start_rotate → _update_positions)

```
시작 조건: TRY_ADVANCE에서 R 상태 claim 성공
│
├─ 시작 각도(from_deg), 목표 각도(to_deg) 파싱
├─ 최단 경로 방향 계산 (CCW/CW 중 짧은 쪽)
├─ agent.state = ROTATING
│
└─ 매 프레임 _update_positions(dt):
    ├─ 회전량 누적: angle_traversed += ANGULAR_SPEED × dt
    │   (ANGULAR_SPEED = π/4 rad/s, 90° 회전에 2초)
    ├─ 보간 각도: theta = lerp(from_theta, to_theta, traversed/total)
    └─ 완료 판정: angle_traversed ≥ angle_total → 완료 처리
```

### 8.3 도달/완료 처리 (_handle_arrival)

M 또는 R 완료 시:

```
1. 현재 TAPG 노드 + 이전의 모든 S 노드를 DAG에서 제거
2. 제거된 노드의 wait_queue에 등록된 에이전트들을 깨움
   → 깨어난 에이전트: WAITING → IDLE, TRY_ADVANCE 스케줄
3. (유한 가속도 모드) 연속 주행 시도 → 성공하면 멈추지 않고 다음 M 진입
4. (연속 주행 불가) v = 0, TRY_ADVANCE 스케줄
```

---

## 9. 대기 및 깨우기 메커니즘

### 9.1 Wait Queue

에이전트가 WAITING 상태에 진입할 때, 자신을 막고 있는 **cross-agent 선행 노드의 wait_queue**에 등록합니다.

```
wait_queues = {
    (M,na.29,na.20, A2, 410.45): [A5],   # A5가 이 노드의 완료를 기다림
    (S,na.20,180, A2, 412.63):   [A5],    # A5가 이 노드의 완료도 기다림
}
```

### 9.2 이벤트 기반 깨우기

선행 노드가 `_complete_node`로 DAG에서 제거될 때:

```
1. wait_queue에서 대기 에이전트 목록 추출
2. 각 대기 에이전트: WAITING → IDLE
3. TRY_ADVANCE 이벤트 스케줄 (time = 현재 + ε)
```

### 9.3 주기적 깨우기 (방어 메커니즘)

이벤트 누락을 방어하기 위해, 0.5초 간격으로 모든 WAITING 에이전트를 재확인합니다:

```
매 0.5초 (시뮬레이션 시간):
    for 각 WAITING 에이전트:
        현재 상태의 TAPG 노드가 claimable이면:
            WAITING → IDLE
            TRY_ADVANCE 스케줄
```

---

## 10. 연속 주행 (Move Chaining)

유한 가속도 모드에서, 에이전트가 노드에 도착했을 때 **멈추지 않고 다음 엣지로 바로 진입**할 수 있습니다.

### 조건

1. 현재 속도 > 0 (이동 중이었음)
2. 완료된 상태가 M (Move)
3. 다음 상태 시퀀스가 S → M 패턴
4. 다음 M이 이미 claimed이거나 claim 가능

### 동작

```
M 완료 (v = carry_v)
│
├─ S 상태를 inline으로 통과 (위치 스냅만, 정지하지 않음)
├─ 다음 M이 claimable?
│   ├─ YES → _start_move(v_init=carry_v)  ← 속도 유지
│   └─ NO  → v = 0, TRY_ADVANCE          ← 정지 후 대기
│
└─ 감속 판정 중 다음 M이 claimable이면 감속 생략 (통과 주행)
```

---

## 11. 동적 경로 확장 (Incremental Replan)

에이전트가 목적지에 도달(DONE)하면, 새로운 목적지를 배정받고 경로가 확장됩니다.

### 11.1 제약 조건 빌드 (vis_combined.py)

새 경로 계획 시, 활동 중인 에이전트의 기존 경로를 제약으로 반영합니다:

```
각 활동 에이전트 a:
│
├─ Claimed 구간 (path_idx ~ claim_idx):
│   각 상태: constraint(loc=state_id, time=[sim_time, 다음상태시작])
│   마지막 claimed 상태: time=[sim_time, unclaimed구간시작]  ← 대기 갭 방지
│   + affect_state도 동일하게 block
│
└─ Unclaimed 구간 (claim_idx ~):
    각 상태: constraint(loc=state_id, time=[CBS시작, 다음상태CBS시작])
    + affect_state도 동일하게 block
```

### 11.2 TAPG 확장 (extend_agents_batch)

```
Phase 1: 새 경로의 DAG 노드 + 순차 엣지 추가
Phase 2: 새 경로 ↔ 기존 모든 경로 간 교차 엣지 빌드
Phase 3: 새 에이전트에 TRY_ADVANCE 스케줄
```

### 11.3 시간 보정 (recompute_earliest_schedule)

리플랜 전에 TAPG의 모든 노드 시간을 재계산합니다:

```
1. 위상 정렬 순서로 earliest start 전파
2. 시작 노드: t = current_time
3. 후속 노드: t = max(모든 predecessor의 finish_time)
4. DAG 그래프를 새 시간으로 재구성
5. 에이전트 raw_path의 시간도 동기화
```

---

## 12. 메인 루프 (step)

```python
def step(sim_time):
    dt = sim_time - self.sim_time
    self.sim_time = sim_time

    # 1) 이벤트 큐에서 현재 시각 이하의 이벤트 모두 처리
    while event_queue.top().time <= sim_time:
        process(event_queue.pop())

    # 2) 물리 위치/각도 갱신 + 도달 감지
    _update_positions(dt, sim_time)

    # 3) 주기적 WAITING 에이전트 재확인 (0.5초 간격)
    if sim_time - last_check >= 0.5:
        _periodic_wakeup(sim_time)
```

호출 측(시각화 루프)에서 매 프레임마다 `step(현재_시뮬레이션_시각)`을 호출합니다.

---

## 13. 입력 데이터 구조

### Collision Profile (.pkl)

```python
{
    'Stop_state':    {state_id: State(cost, affect_state=[...])},
    'Move_state':    {state_id: State(cost, affect_state=[...])},
    'Rotate_state':  {state_id: State(cost, affect_state=[...])},
    'stop_regions':  {state_id: Shapely Polygon},  # 차량 점유 영역
    'move_regions':  {state_id: Shapely Polygon},  # 이동 시 sweep 영역
    'od_pairs':      [(src, dst), ...],             # 포트 쌍 목록
}
```

- `State.cost`: 해당 상태의 소요 시간 (M: 이동시간, R: 회전시간, S: 0)
- `State.affect_state`: 이 상태와 물리적으로 충돌하는 다른 상태들의 ID 리스트

### CBS 경로 (raw_path)

```python
raw_path = [(state_id, cbs_start_time), ...]
# 예: [('S,na.1,0', 0.0), ('M,na.1,na.2', 0.0), ('S,na.2,0', 2.5), ...]
```

---

## 14. 전체 실행 시퀀스 예시

```
t=0.00  초기화
        ├─ CBS 플래너가 모든 에이전트 경로 계산
        ├─ TAPG 빌드 (노드 + 순차 엣지 + 교차 엣지)
        └─ 모든 에이전트에 TRY_ADVANCE 스케줄

t=0.00  TRY_ADVANCE(A0)
        ├─ S,na.1,0 즉시 통과 (위치 스냅)
        ├─ M,na.1,na.2 claimable 확인 → 선행 노드 없음 → claim 확장
        └─ _start_move: state=MOVING, v=1000mm/s

t=0.00  TRY_ADVANCE(A1)
        ├─ S,na.5,90 즉시 통과
        ├─ M,na.5,na.6 claimable 확인 → A0의 선행 노드 존재
        └─ state=WAITING, wait_queue 등록

        ... (매 프레임 _update_positions로 A0 이동) ...

t=2.50  A0 도달 감지 (dist_traveled ≥ edge_length)
        ├─ _handle_arrival: M,na.1,na.2 DAG에서 제거
        ├─ A1의 wait_queue에서 A1 발견 → A1 깨움
        ├─ A0: TRY_ADVANCE 스케줄 (다음 상태 진행)
        └─ A1: IDLE → TRY_ADVANCE (이제 M,na.5,na.6 claim 가능)

        ... (반복) ...

t=150.0 A0 경로 끝 도달 → state=DONE
        ├─ 새 목적지 배정 → CBS 리플랜
        ├─ 기존 에이전트 경로를 제약으로 → 새 경로 계산
        └─ extend_agents_batch: TAPG 확장 → TRY_ADVANCE 스케줄
```

---

## 부록: 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `accel` | ∞ | 가속도 (mm/s²). ∞이면 즉시 최고속 |
| `decel` | ∞ | 감속도 (mm/s²). ∞이면 즉시 정지 |
| `ANGULAR_SPEED` | π/4 rad/s | 회전 각속도 (90°에 2초) |
| `PERIODIC_INTERVAL` | 0.5s | WAITING 재확인 주기 |
| `DEFAULT_SPEED` | 1000 mm/s | 엣지 기본 최고 속도 |
