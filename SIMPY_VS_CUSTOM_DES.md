# SimPy vs Custom DES Engine: 상세 비교 분석

## 목차
1. ["범용 시뮬레이션"이란?](#1-범용-시뮬레이션이란)
2. [SimPy 아키텍처 상세](#2-simpy-아키텍처-상세)
3. [현재 Custom DES 아키텍처 상세](#3-현재-custom-des-아키텍처-상세)
4. [항목별 비교](#4-항목별-비교)
5. [물류 처리 확장 관점의 분석](#5-물류-처리-확장-관점의-분석)
6. [결론 및 권장사항](#6-결론-및-권장사항)

---

## 1. "범용 시뮬레이션"이란?

"범용(General-purpose)"이라는 표현은 **특정 도메인에 종속되지 않는 DES 프레임워크**를 의미한다.
SimPy는 공장, 물류, 네트워크, 병원, 공항 등 어떤 시스템이든 모델링할 수 있는 범용 도구이다.

반면, 현재 구현은 **반도체 Fab 물류(OHT/AGV/3DS/Elevator)에 특화된 도메인 전용 엔진**이다.

### 범용 vs 도메인 전용의 의미

| 구분 | 범용 (SimPy) | 도메인 전용 (현재 구현) |
|------|-------------|----------------------|
| **추상화 수준** | Process, Resource, Store 등 일반적인 개념 제공 | OHTSegment, ZCU, TAPG DAG 등 도메인 개념 직접 구현 |
| **적용 범위** | 어떤 DES 문제든 모델링 가능 | 반도체 Fab 물류에 최적화 |
| **코드 재사용** | 다른 프로젝트에 그대로 적용 가능 | 이 프로젝트 전용 |
| **새 기능 추가** | 프레임워크 위에 쌓음 | 엔진 자체를 수정/확장 |

**물류 처리를 다루는 것 자체는 두 접근법 모두 가능하다.** "범용"이라는 것은 "더 많은 것을 할 수 있다"는 뜻이 아니라, "미리 만들어진 빌딩블록으로 빠르게 조립할 수 있다"는 뜻이다.

---

## 2. SimPy 아키텍처 상세

### 2.1 이벤트 루프

```
내부 구조: heapq 기반 min-heap
정렬 기준: (simulation_time, priority, event_id)
처리 방식: 단일 스레드, 결정론적(deterministic)
```

SimPy의 이벤트 루프는 본질적으로 `heapq`를 감싼 것이다. 이벤트를 `(시간, 우선순위, ID)` 튜플로 힙에 넣고, 가장 이른 것부터 꺼내 처리한다.

### 2.2 프로세스 모델 (Generator Coroutine)

SimPy의 핵심 패러다임은 **Python generator를 이용한 프로세스 기술**이다:

```python
import simpy

def oht_vehicle(env, vehicle_id, track, resource):
    while True:
        job = yield job_store.get()                  # 작업 대기
        yield env.timeout(travel_time(job.src))      # 출발지까지 이동

        with resource.request() as req:              # 포트 리소스 요청
            yield req                                # 리소스 획득 대기
            yield env.timeout(loading_time)          # 로딩

        yield env.timeout(travel_time(job.dst))      # 목적지까지 이동

        with resource.request() as req:
            yield req
            yield env.timeout(unloading_time)        # 언로딩
```

**장점:** 프로세스의 전체 생애주기를 하나의 함수에 **순차적으로** 기술할 수 있다. 상태 머신을 명시적으로 관리할 필요가 없다.

**단점:** `yield` 지점이 곧 이벤트 경계이므로, 연속적인 물리 상태(위치, 속도, 가속도)를 yield 사이에서 추적하려면 별도 로직이 필요하다.

### 2.3 리소스 타입

SimPy가 기본 제공하는 5가지 리소스 추상화:

#### Resource (용량 제한 + 대기열)
```python
port = simpy.Resource(env, capacity=1)

with port.request() as req:
    yield req            # 다른 프로세스가 점유 중이면 대기
    yield env.timeout(3) # 작업 수행
# 자동 해제
```
- **용도:** 포트, 장비, 서버 등 capacity가 있는 공유 자원
- **대기열:** FIFO 기본

#### PriorityResource (우선순위 기반 대기열)
```python
port = simpy.PriorityResource(env, capacity=1)

with port.request(priority=1) as req:  # 낮은 숫자 = 높은 우선순위
    yield req
    ...
```
- **용도:** 긴급 작업 우선 처리 등

#### PreemptiveResource (선점 가능)
```python
machine = simpy.PreemptiveResource(env, capacity=1)

with machine.request(priority=0) as req:
    yield req
    try:
        yield env.timeout(10)  # 작업 중
    except simpy.Interrupt as interrupt:
        # 더 높은 우선순위 프로세스에 의해 선점됨
        print(f"Preempted by {interrupt.cause}")
```
- **용도:** 긴급 유지보수, 고장 등으로 현재 작업이 중단되어야 할 때

#### Container (연속/이산 수량)
```python
fuel_tank = simpy.Container(env, capacity=100, init=50)

yield fuel_tank.get(10)   # 10 단위 소모 (부족하면 대기)
yield fuel_tank.put(30)   # 30 단위 충전 (초과하면 대기)
```
- **용도:** 연료, 전력, 재고 수량 등 동질적(homogeneous) 자원

#### Store / FilterStore (이산 객체 저장)
```python
job_store = simpy.FilterStore(env)
job_store.put(Job(type='A', priority=1))
job_store.put(Job(type='B', priority=2))

# 조건부 꺼내기
job = yield job_store.get(filter=lambda j: j.type == 'A')
```
- **용도:** 작업 큐, 버퍼, WIP(Work-In-Progress) 관리

### 2.4 이벤트 조합

```python
# 둘 중 하나라도 완료되면 진행
result = yield env.any_of([timeout_event, resource_event])

# 모두 완료되어야 진행
result = yield env.all_of([part_a_ready, part_b_ready])

# 프로세스 인터럽트
process.interrupt(cause="breakdown")
```

### 2.5 실시간 시뮬레이션

```python
env = simpy.rt.RealtimeEnvironment(factor=1.0, strict=True)
# factor=1.0: 실시간 1:1 매핑
# strict=True: 실시간보다 느리면 RuntimeError 발생
```

---

## 3. 현재 Custom DES 아키텍처 상세

### 3.1 이벤트 루프

```python
# 직접 heapq 사용
heapq.heappush(self._heap, Event(t, kind, agent_id, token, data))

# step() 호출 시 시간까지의 이벤트 일괄 처리
def step(self, t_now):
    while self._heap and self._heap[0].t <= t_now:
        ev = heapq.heappop(self._heap)
        if self._valid(ev):
            self._process(ev)
    self._update_visuals(t_now)
```

### 3.2 이벤트 기반 상태 전이 (SimPy의 generator 대신)

프로세스를 generator로 기술하지 않고, **이벤트 타입별 핸들러**로 상태 전이를 관리:

```
OHT 이벤트:
  TRY_ADVANCE  → 다음 세그먼트 진입 시도
  SEGMENT_DONE → 현재 세그먼트 완주
  CATCH_UP     → follower가 leader에 접근 (감속 필요)
  GAP_CLEAR    → leader가 가속/이탈하여 gap 확보

TAPG 이벤트:
  TRY_ADVANCE  → TAPG DAG 경로 진행 시도

3DS 이벤트:
  S3D_TRY_ADVANCE → 셔틀 edge 진입 시도
  S3D_ARRIVE      → 셔틀 도착

MCS 이벤트:
  JOB_ARRIVE   → 새 작업 생성 (Poisson 도착)
  DWELL_DONE   → 로딩/언로딩 완료
  TRY_ASSIGN   → 유휴 차량에 작업 할당 시도

Elevator 이벤트:
  LIFT_MOVE_DONE → 엘리베이터 이동 완료
  LIFT_XFER_DONE → 적재/하역 완료
```

### 3.3 토큰 기반 이벤트 무효화

```python
def _post(self, t, kind, agent, data=None):
    tok = agent.adv_token if kind == TRY_ADVANCE else agent.token
    heapq.heappush(self._heap, Event(t, kind, agent.id, tok, data))

def _valid(self, ev):
    agent = self.agents[ev.agent_id]
    expected = agent.adv_token if ev.kind == TRY_ADVANCE else agent.token
    return ev.token == expected  # 토큰 불일치 → 무효(stale) 이벤트
```

**SimPy에는 이런 메커니즘이 없다.** SimPy에서 이벤트를 취소하려면 별도의 플래그 검사나 `Interrupt`를 사용해야 한다.

### 3.4 분석적 물리 모델

충돌 회피를 **2차 방정식으로 정확히 계산**:

```
위치 모델: p(t) = p0 + v*dt + 0.5*a*dt^2  (가속 구간)
                 = p_cap + v_cap*(dt - dt_cap)  (정속 구간)

안전 거리: h_safe(v) = h_min + v^2 / (2 * decel)

catch-up 시간: follower가 leader의 안전 거리를 침범하는 정확한 시각을
              다항식 근(root)으로 계산 → CATCH_UP 이벤트 스케줄링
```

### 3.5 리소스 관리 (직접 구현)

| 리소스 | 구현 방식 |
|--------|----------|
| **세그먼트 큐** | `segment.queue: List[OHTAgent]` — FIFO, front-detection |
| **ZCU (Zone Control)** | `_zcu_holders[zone_id]` + `_zcu_waitlists[zone_id]` — mutex |
| **엘리베이터** | `_queue: deque[LiftRequest]` — priority 기반 큐 |
| **작업 큐** | `job_queue: deque[Job]` — FIFO, unbounded |
| **TAPG 의존성** | `affect_state` DAG — cross-agent edge로 충돌 방지 |

---

## 4. 항목별 비교

### 4.1 성능

| 항목 | Custom DES (현재) | SimPy |
|------|-------------------|-------|
| **이벤트 스케줄링** | `heapq` 직접 — O(log n) | `heapq` + Event 객체 래핑 — O(log n) + 상수 오버헤드 |
| **이벤트 처리** | 함수 호출 1회 | generator resume + yield + 콜백 체인 |
| **이벤트 무효화** | 토큰 비교 O(1) — stale 이벤트 즉시 폐기 | 지원 없음 — 별도 플래그/Interrupt 필요 |
| **물리 계산** | 분석적 해 (closed-form) | 지원 없음 — 직접 구현 필요 |
| **메모리** | Event namedtuple (~64 bytes) | Event + callbacks + Process + generator frame (~200+ bytes) |
| **벤치마크 추정** | 기준선 (1x) | **약 1.5~3x 느림** (coroutine 오버헤드 30%+ 및 추가 래핑) |

> **참고:** C 기반 DES 엔진(Cimba 등)은 SimPy 대비 **~45x 빠름**.
> R 기반 simmer도 SimPy 대비 **1.6~1.9x 빠름**.
> Python 내에서는 heapq 직접 사용이 가장 빠른 선택지이다.

### 4.2 코드 가독성 및 유지보수

| 항목 | Custom DES | SimPy |
|------|-----------|-------|
| **프로세스 기술** | 이벤트 핸들러 + 상태 머신 (분산) | generator 함수 (순차적, 한 곳에 집중) |
| **상태 관리** | 명시적 `agent.state`, `agent.token` | generator가 암묵적으로 관리 |
| **새 프로세스 추가** | 이벤트 타입 정의 + 핸들러 구현 + 상태 전이 추가 | generator 함수 1개 작성 |
| **디버깅** | 이벤트 로그 추적 (어떤 이벤트가 언제 발생했는지) | generator 스택 트레이스 (어디서 yield했는지) |
| **학습 곡선** | 도메인 지식 필요 (OHT, ZCU, TAPG 등) | SimPy API 학습 후 빠르게 적용 가능 |

**SimPy의 가독성 이점 예시:**

```python
# SimPy: 차량 전체 생애주기가 한 함수에 보임
def vehicle_process(env, vehicle, mcs, track):
    while True:
        job = yield mcs.job_store.get()           # 1. 작업 대기
        path = plan_path(vehicle.pos, job.src)
        for segment in path:
            with segment.request() as req:
                yield req                          # 2. 세그먼트 진입 대기
                yield env.timeout(segment.travel)  # 3. 이동
        yield env.timeout(job.load_time)           # 4. 로딩
        path = plan_path(job.src, job.dst)
        for segment in path:
            with segment.request() as req:
                yield req
                yield env.timeout(segment.travel)
        yield env.timeout(job.unload_time)         # 5. 언로딩
        mcs.complete(job)
```

```python
# Custom DES: 동일 로직이 여러 핸들러에 분산됨
def _on_try_advance(self, ev):    # 세그먼트 진입 처리
    ...
def _on_segment_done(self, ev):   # 세그먼트 완주 처리
    ...
def _on_catch_up(self, ev):       # 충돌 회피
    ...
# + MCS의 _on_job_arrive, _on_dwell_done, _on_try_assign ...
```

### 4.3 리소스 경쟁 모델링

| 시나리오 | Custom DES | SimPy |
|----------|-----------|-------|
| **단순 mutex** (포트 1개) | 직접 lock/waitlist 구현 (~30줄) | `Resource(env, capacity=1)` (1줄) |
| **우선순위 큐** (긴급 작업) | 직접 정렬 삽입 구현 | `PriorityResource(env, capacity=1)` (1줄) |
| **선점** (장비 고장) | 직접 인터럽트 + 상태 복원 구현 | `PreemptiveResource` + `try/except Interrupt` |
| **재고 관리** (버퍼) | 직접 카운터 + put/get 이벤트 | `Container(env, capacity=100)` |
| **작업 큐 필터링** | 직접 deque 순회 + 조건 검사 | `FilterStore.get(filter=lambda j: ...)` |
| **복합 대기** (A와 B 모두 필요) | 직접 플래그 조합 구현 | `yield env.all_of([req_a, req_b])` |

**SimPy가 유리한 경우:** 리소스 종류가 많아지고 경쟁 패턴이 복잡해질수록 SimPy의 기본 제공 추상화가 개발 속도를 높인다.

**Custom DES가 유리한 경우:** 리소스 경쟁이 물리 모델과 밀접하게 결합되어 있을 때 (예: OHT 세그먼트 큐 + 운동학 기반 catch-up). SimPy의 `Resource.request()`는 이산 대기만 지원하고, 연속적인 거리/속도 기반 조건을 표현할 수 없다.

### 4.4 물리/운동학 통합

| 항목 | Custom DES | SimPy |
|------|-----------|-------|
| **연속 위치 추적** | `p(t) = p0 + v*dt + 0.5*a*dt^2` — 언제든 정확한 위치 계산 | 지원 없음 — yield 간 위치 변화를 별도로 계산해야 함 |
| **분석적 충돌 시간** | 2차 방정식 근으로 정확한 시각 계산 | 불가능 — 타임스텝 기반 근사 또는 별도 solver 필요 |
| **속도/가속도 변경** | 즉시 kinematic 상태 업데이트 + 관련 이벤트 재스케줄링 | Process 내에서 상태 변수를 수동 관리해야 함 |
| **시각화 연동** | `step(t_now)` 후 보간된 위치 사용 — 프레임 기반 렌더링과 자연스럽게 호환 | `env.run(until=t)` 후 상태 추출 — 유사하지만 추가 래핑 필요 |

### 4.5 확장성 (물류 처리 확장 시)

현재 시스템에 **추가될 수 있는 물류 기능**과 각 접근법의 대응:

| 확장 기능 | Custom DES | SimPy |
|----------|-----------|-------|
| **Stocker/Bay 버퍼** | deque + 이벤트 직접 구현 | `Store` 또는 `FilterStore` |
| **다중 엘리베이터 스케줄링** | 이미 `ElevatorController`로 구현 | `PriorityResource(capacity=N)` |
| **장비 고장/유지보수** | 고장 이벤트 + 상태 복원 로직 직접 구현 | `PreemptiveResource` + `Interrupt` |
| **Lot 배치(batching)** | 카운터 + 조건부 이벤트 직접 구현 | `Container` + `AllOf` |
| **다중 Layer 간 이동** | 이미 Elevator + 3DS로 구현 | `Resource` + `Process` 조합 |
| **동적 재배치(rerouting)** | 토큰 무효화 + 재경로 | `Interrupt` + 새 Process 생성 |
| **WIP 제한(Kanban)** | 카운터 + 진입 차단 로직 | `Container(capacity=wip_limit)` |
| **통계/KPI** | `KPITracker` 직접 구현 (이미 있음) | 직접 구현 필요 (SimPy는 통계 기능 미제공) |

---

## 5. 물류 처리 확장 관점의 분석

### 5.1 현재 시스템이 이미 커버하는 범위

```
작업 생성 (Poisson) → 작업 큐 → 차량 할당 →
  출발지 이동 (OHT/AGV/3DS) → 로딩 →
  목적지 이동 → 언로딩 →
  완료 + KPI 기록 → 차량 재할당
```

이것은 이미 **end-to-end 물류 루프**이다. "범용 시뮬레이션 도구가 아니면 물류를 못 다룬다"는 것은 사실이 아니다.

### 5.2 SimPy로 전환 시 잃는 것

1. **분석적 충돌 회피**: OHT의 CATCH_UP/GAP_CLEAR 메커니즘은 SimPy의 프로세스 모델로 표현하기 어렵다. `yield env.timeout(catch_up_time)`으로 대체하면, leader의 속도가 변경될 때 follower의 이벤트를 재스케줄링하는 것이 복잡해진다.

2. **토큰 기반 이벤트 무효화**: SimPy에서는 `process.interrupt()`를 사용하거나 매 yield에서 플래그를 검사해야 한다. 현재의 O(1) 토큰 비교보다 무겁다.

3. **TAPG DAG 실행 모델**: CBS 플래너가 생성한 cross-agent 의존성 그래프를 SimPy 프로세스 간 동기화로 표현하려면 `Event`를 직접 생성하고 연결해야 한다 — SimPy의 기본 리소스 모델이 이 패턴에 맞지 않는다.

4. **성능**: 이벤트 수가 많은 대규모 시뮬레이션에서 1.5~3x 속도 저하.

### 5.3 SimPy로 전환 시 얻는 것

1. **리소스 추상화 재사용**: 새로운 공유 자원(stocker, conveyor, 포트 등)을 추가할 때 Resource/Store/Container를 즉시 사용 가능. 직접 구현 대비 코드량 50~70% 감소.

2. **프로세스 가독성**: 차량의 전체 생애주기를 하나의 generator 함수에서 읽을 수 있어, 새로운 개발자가 시스템을 이해하기 쉬움.

3. **Interrupt 지원**: 장비 고장, 긴급 작업 삽입, 동적 재배치 등의 "예외 상황" 모델링이 `try/except Interrupt` 패턴으로 깔끔해짐.

4. **조건부 대기**: `AnyOf`, `AllOf`, `FilterStore.get(filter=...)` 등으로 복합 조건 대기를 선언적으로 표현 가능.

5. **커뮤니티/생태계**: SimPy는 Python DES의 사실상 표준. 레퍼런스, 튜토리얼, 기존 모델이 풍부하여 외부 협업자가 참여하기 쉬움.

### 5.4 하이브리드 접근법 (권장)

물류 확장을 고려할 때 가장 현실적인 접근법은 **"전면 전환"이 아니라 "레이어별 선택"**이다:

```
┌────────────────────────────────────────────────────────┐
│ 상위 레이어: 물류 제어 (MCS, 작업 할당, 재고 관리)     │
│ → SimPy의 Store/Resource/Container가 유용할 수 있음    │
│ → 단, 현재 MCS도 이미 잘 동작함                        │
├────────────────────────────────────────────────────────┤
│ 하위 레이어: 물리 수송 (OHT, AGV, 3DS, Elevator)       │
│ → Custom DES 유지 (물리 모델, 충돌 회피가 핵심)        │
│ → SimPy로 대체하면 핵심 강점을 잃음                    │
└────────────────────────────────────────────────────────┘
```

이 하이브리드는 기술적으로 가능하다:
- 상위: SimPy `Environment.run(until=t)` 또는 직접 구현한 이벤트 루프
- 하위: 현재 `env.step(t)` 그대로 유지
- 연결: 콜백 (`on_dispatch`, `notify_arrived`) 인터페이스는 프레임워크와 무관

**다만**, 현재 MCS 자체가 이미 간결하게 구현되어 있고 (406줄), SimPy를 도입해도 줄어드는 코드량이 크지 않다. 새로운 리소스 타입이 10개 이상 추가되는 시점에서 재고려하는 것이 합리적이다.

---

## 6. 결론 및 권장사항

### 핵심 요약

| 판단 기준 | 결론 |
|----------|------|
| **실행 속도** | Custom DES가 1.5~3x 빠름 |
| **물리 모델 통합** | Custom DES가 근본적으로 유리 (분석적 해, 토큰 무효화) |
| **리소스 모델링 편의성** | SimPy가 유리 (기본 제공 Resource/Store/Container) |
| **코드 가독성** | SimPy가 유리 (generator 기반 프로세스) |
| **물류 처리 가능 여부** | **둘 다 가능** — 현재 구현도 이미 end-to-end 물류 루프 보유 |
| **전환 비용** | 높음 — 핵심 물리 엔진 재작성 필요 |

### 권장사항

**현 시점에서 SimPy 전환은 불필요하다.**

1. 현재 엔진의 핵심 강점(분석적 충돌 회피, 토큰 무효화, 운동학 모델)은 SimPy로 대체할 수 없거나 대체 시 성능/정확도가 떨어진다.
2. 물류 처리 확장은 현재 아키텍처 위에서 충분히 가능하다 (MCS가 이미 작업 생성→할당→수송→완료 루프를 관리).
3. 향후 리소스 경쟁이 대폭 복잡해지는 시점(예: stocker bay 관리, 다중 conveyor, WIP 제한 등)에서 **상위 물류 레이어에 한정하여** SimPy 도입을 재고려할 수 있다.
4. **하위 물리 수송 레이어는 어떤 경우에도 Custom DES를 유지하는 것이 바람직하다.**
