# System Architecture - TeamKoreaPhysicalAI Testbed Simulator

> OHT + AGV + 3DS Shuttle 통합 시뮬레이터의 전체 아키텍처, 경로 계획 방식,
> 이벤트 구조, 서브시스템 간 연동을 설명합니다.

---

## 1. 시스템 전체 구조

```
                        vis_combined.py (통합 시뮬레이터)
                     ┌──────────┼──────────┐
                     │          │          │
              ┌──────▼──────┐ ┌▼────────┐ ┌▼──────────┐
              │  OHT 서브시스템 │ │AGV 서브시스템│ │3DS 서브시스템  │
              │ env_oht_des │ │env_tapg  │ │ env_3ds   │
              │             │ │          │ │ elevator  │
              └──────────────┘ └─────┬────┘ └───────────┘
                                     │
                          ┌──────────▼──────────┐
                          │ pkl_prioritized_planner │
                          │  (Prioritized SIPP)     │
                          └──────────┬──────────┘
                                     │
                          ┌──────────▼──────────┐
                          │    pkl_loader        │
                          │ (Collision Profile)  │
                          └─────────────────────┘
```

### 서브시스템별 역할

| 서브시스템 | 대상 | 충돌 회피 방식 | 경로 계획 |
|-----------|------|-------------|----------|
| **OHT** | 천장 레일 반송차 | 전방 감지 + 세그먼트 큐 + ZCU | 고정 경로 (최단 경로) |
| **AGV** | 바닥 무인 반송차 | TAPG (의존 그래프) | Prioritized SIPP |
| **3DS** | 층별 셔틀 | 층당 1대 (충돌 없음) | Prioritized SIPP |
| **Elevator** | 층간 화물 이송 | 공유 자원 큐 | 요청 순서 (FIFO) |

---

## 2. AGV Path Planning - Prioritized SIPP

### 2.1 개요

AGV 경로 계획은 **Prioritized SIPP (Safe Interval Path Planning)** 방식을 사용합니다.

```
1. 의존 그래프로 에이전트 계획 순서 결정
2. 높은 우선순위 에이전트부터 SIPP로 경로 계획
3. 계획된 경로를 다음 에이전트의 constraint로 추가
4. 반복
```

### 2.2 State Graph

일반적인 노드-엣지 그래프가 아닌 **상태 그래프(State Graph)** 위에서 경로를 탐색합니다.

```
Stop State: S,node_id,heading_angle
  - 특정 노드에 특정 각도로 정지한 상태
  - cost = 0 (정지 상태이므로 시간 소요 없음)
  - next_state = [M,node,neighbor1, M,node,neighbor2, R,node,angle,other_angle, ...]

Move State: M,from_node,to_node
  - 두 노드 간 직선 이동
  - cost = edge_length / max_speed (이동 시간)
  - next_state = [S,to_node,edge_angle]

Rotate State: R,node,from_angle,to_angle
  - 제자리 회전
  - cost = angle_diff / angular_speed (회전 시간)
  - next_state = [S,node,to_angle]
```

### 2.3 Collision Profile (affect_state)

각 상태에는 **affect_state** 집합이 있습니다. 이는 해당 상태가 활성일 때 **물리적으로 충돌하는 다른 상태들**의 목록입니다.

```
예: M,na.48,na.193의 affect_state = {S,na.193,0, S,na.193,180, S,na.48,0, ...}
    → 이 Move가 실행 중이면 위 Stop 상태들과 물리적 충돌
```

affect_state는 차량의 2D 점유 영역(Shapely Polygon)의 교차 여부로 사전 계산되며, `.pkl` 파일로 저장됩니다.

### 2.4 SIPP 탐색

SIPP는 **Safe Interval** 개념을 사용합니다. 각 상태에 대해 다른 에이전트의 constraint가 없는 시간 구간(safe interval)을 계산하고, 그 구간 내에서만 진입을 허용합니다.

```python
# Successor 생성 (간략화)
for neighbor in current_state.next_state:
    blocked_intervals = constraint_table[neighbor]
    safe_intervals = compute_safe_intervals(blocked_intervals)

    for interval in safe_intervals:
        arrive_time = max(current_time + cost, interval.start)

        # M->S, R->S 전이: 즉시 도착만 허용 (물리적으로 이미 노드에 있음)
        if is_arrival_transition and arrive_time > current_time + cost:
            continue  # 대기 불가 - skip

        add_successor(neighbor, arrive_time, interval)
```

**핵심 제약 (M->S, R->S 전이):**
Move/Rotate 완료 후 Stop 상태로 전이할 때, 에이전트는 이미 해당 노드에 물리적으로 존재합니다. 따라서 safe interval이 나중에 시작하더라도 "대기 후 진입"이 불가능합니다. safe interval이 도착 시점을 포함하지 않으면 해당 successor는 사용할 수 없습니다.

### 2.5 Constraint 빌드 (Incremental Replan)

에이전트가 목적지에 도달하면 새 목적지를 배정받고 재계획됩니다. 이때 활동 중인 다른 에이전트의 경로를 constraint로 변환합니다.

```
각 활동 에이전트의 경로:

  ┌── Claimed 구간 (path_idx ~ claim_idx) ──┐┌── Unclaimed 구간 ──┐
  │ 이미 실행 확정. sim_time부터 block.      ││ CBS 시간 기반 block  │
  └─────────────────────────────────────────┘└───────────────────┘

  + 현재 위치 (path_idx): 항상 sim_time부터 block
  + 각 상태의 affect_state도 동일하게 block
```

### 2.6 TAPG (Temporal Action Precedence Graph)

계획된 경로들을 DAG로 변환하여 실행 순서를 관리합니다.

```
노드: (state_id, agent_id, cbs_time)
엣지:
  - 순차 엣지: 같은 에이전트의 연속 상태
  - 교차 엣지: 다른 에이전트 간 affect_state 충돌 시

교차 엣지 조건:
  for agent_i의 state s1, agent_j의 state s2:
    if s2.time > s1.time:
      if s2 in affect(s1) or s1 in affect(s2):
        edge: s1 -> s2  (s1 완료 후 s2 실행 가능)
```

Claim 조건: 다른 에이전트의 선행 TAPG 노드가 모두 DAG에서 제거(완료)되었을 때만 실행 가능.

---

## 3. OHT 서브시스템

### 3.1 구조

OHT(Overhead Hoist Transport)는 천장 레일 위를 주행하는 반송차입니다. 단방향 세그먼트로 구성된 네트워크에서 전방 감지 기반으로 충돌을 회피합니다.

```
OHTMap (네트워크)
  └── Segment: 단방향 링크 (from_node -> to_node)
       - length, max_speed, capacity
       - queue: 세그먼트 내 에이전트 목록 (front -> back 순서)

OHTAgent (차량)
  └── path: [segment_id, ...] (최단 경로)

OHTEnvironmentDES (실행 엔진)
  └── 이벤트 큐 기반 실행
```

### 3.2 충돌 회피: 전방 감지 + 세그먼트 큐

```
세그먼트 진입 시:
  - 세그먼트 큐에 등록 (뒤쪽에 추가)
  - 앞에 리더가 있으면: 리더와의 거리 모니터링 -> CATCH_UP 예약
  - 앞에 리더가 없으면: 최고 속도로 주행

CATCH_UP 이벤트:
  - 리더와의 거리가 최소 안전 거리(h_min)에 도달하는 시점
  - 발생 시: 리더 속도로 감속 (FOLLOWING 상태)

GAP_CLEAR 이벤트:
  - 리더가 세그먼트를 떠나거나 가속한 경우
  - 발생 시: 최고 속도로 복귀 가능
```

### 3.3 ZCU (Zone Control Unit)

합류/분기 노드에서의 배타적 접근 제어:

```
ZCU = 특정 노드 그룹의 진입 잠금

에이전트 진입 시:
  ZCU 잠금 획득 시도
    -> 성공: 통과
    -> 실패: BLOCKED 상태, waitlist 등록

ZCU 소유자가 출구 세그먼트 완료 시:
  잠금 해제 -> waitlist의 다음 에이전트 깨움
```

### 3.4 OHT DES 이벤트

| 이벤트 | 발생 조건 | 처리 |
|--------|---------|------|
| TRY_ADVANCE | 경로 시작, 세그먼트 완료 후 | 다음 세그먼트 진입 시도 |
| SEGMENT_DONE | 세그먼트 끝 도달 | 큐에서 제거, 다음 TRY_ADVANCE |
| CATCH_UP | 리더와의 거리 = h_min | 리더 속도로 감속 |
| GAP_CLEAR | 리더 가속/퇴장 | 최고 속도 복귀 |

---

## 4. 3DS 서브시스템

### 4.1 구조

3DS(3-Dimensional Shuttle)는 층별 셔틀로, 각 층에 1대씩 운영됩니다.

```
FloorGraph (층별 그래프)
  └── KaistTB_3DS_F1.json ~ F3.json에서 로드

Shuttle (셔틀)
  └── AGV와 유사한 구조 (위치, 속도, 상태)
  └── TAPG 없음 (층당 1대이므로 충돌 회피 불필요)

Env3DS (실행 엔진)
  └── 층별 독립 실행
  └── 가감속 물리 (사다리꼴 속도 프로파일)
```

### 4.2 3DS DES 이벤트

| 이벤트 | 설명 |
|--------|------|
| S3D_TRY_ADVANCE | 다음 엣지 진입 시도 |
| S3D_ARRIVE | 엣지 끝 도달 (물리 기반 감지) |

### 4.3 3DS 경로 계획

3DS도 Prioritized SIPP를 사용하되, 층당 1대이므로 cross-agent constraint가 없습니다. 목적지 도달 후 새 목적지를 배정받아 재계획합니다.

---

## 5. 엘리베이터 서브시스템

### 5.1 개요

엘리베이터는 층간 화물 이송을 담당하는 **공유 자원**입니다. 셔틀이 gate 노드에 화물을 내려놓으면 엘리베이터가 수직 이송합니다.

### 5.2 상태 머신

```
IDLE ──(요청 수락)──> MOVING ──(출발층 도착)──> LOADING
                                                  │
IDLE <──(하역 완료)── UNLOADING <──(도착층 도착)── MOVING
```

### 5.3 처리 흐름

```
1. request(from_floor, to_floor, callback)
   -> IDLE이면 즉시 처리, BUSY면 큐에 적재

2. 출발층으로 이동 (현재층 != 출발층이면)
   -> LIFT_MOVE_DONE 이벤트

3. 화물 적재 (LOADING)
   -> LIFT_XFER_DONE 이벤트

4. 도착층으로 이동
   -> LIFT_MOVE_DONE 이벤트

5. 화물 하역 (UNLOADING)
   -> LIFT_XFER_DONE 이벤트
   -> callback 호출 + IDLE 전환 + 다음 요청 처리
```

### 5.4 엘리베이터 DES 이벤트

| 이벤트 | 설명 |
|--------|------|
| LIFT_MOVE_DONE | 목표 층 도착 |
| LIFT_XFER_DONE | 적재/하역 완료 |

---

## 6. 충돌 프로파일 (.pkl)

### 6.1 생성

`gen_collision_profile.py` (회전 포함) 또는 `gen_collision_profile_no_rot.py` (회전 없음)로 생성합니다.

```
입력: Maps/*.json (노드 좌표, 엣지 정보, 차량 크기)
출력: *.pkl (상태 그래프 + 충돌 프로파일)
```

### 6.2 내용

```python
{
    'Stop_state':   {state_id: State(cost, next_state, affect_state)},
    'Move_state':   {state_id: State(cost, next_state, affect_state)},
    'Rotate_state': {state_id: State(cost, next_state, affect_state)},
    'stop_regions': {state_id: Shapely Polygon},  # 차량 정지 점유 영역
    'move_regions': {state_id: Shapely Polygon},  # 이동 시 sweep 영역
    'od_pairs':     [(src_node, dst_node), ...],   # 출발-도착 쌍
}
```

### 6.3 affect_state 계산 원리

두 상태 A, B의 점유 영역(Shapely Polygon)이 교차하면:
- B를 A의 affect_state에 추가
- A를 B의 affect_state에 추가

이로써 어떤 상태가 활성일 때 물리적으로 충돌하는 모든 다른 상태를 알 수 있습니다.

---

## 7. 서브시스템 간 연동

### 7.1 vis_combined.py의 메인 루프

```python
while running:
    dt = clock.tick(FPS) * sim_speed
    sim_time += dt

    # 1) OHT 환경 step
    oht_env.step(sim_time)

    # 2) AGV 환경 step
    agv_env.step(sim_time)

    # 3) 3DS 환경 step (층별)
    for floor in s3d_floors:
        floor.env.step(sim_time)

    # 4) 엘리베이터 이벤트 처리
    lift_ctrl.step(sim_time)

    # 5) AGV/3DS 완료 시 replan
    if any_agv_done:
        _replan_done_agvs(sim_time)
    for floor in s3d_floors:
        if any_shuttle_done:
            _replan_3ds_floor(floor_id, sim_time)

    # 6) 렌더링
    draw()
```

### 7.2 엘리베이터-3DS 연동

```
셔틀 A (F1) 화물 이송 요청 -> F3

1. 셔틀 A: gate_F1으로 이동 -> 도착 시 엘리베이터 request
2. 엘리베이터: F1으로 이동 -> 적재 -> F3으로 이동 -> 하역
3. 셔틀 B (F3): gate_F3에서 화물 픽업 -> 목적지로 이동
```

### 7.3 AGV replan과 TAPG 확장

```
AGV 도착(DONE) 감지
    │
    ├── recompute_earliest_schedule()  ← TAPG 시간 보정
    ├── constraint 빌드 (활동 에이전트 경로)
    ├── 새 목적지 배정 + SIPP 계획
    └── extend_agents_batch()  ← TAPG DAG 확장 + cross-edge 빌드
```

---

## 8. 디버그 도구

### 키보드 단축키

| 키 | 기능 |
|----|------|
| D | AGV 상태 + TAPG 정보 + replan 이력을 `logs/` 에 저장 |
| SPACE | 시뮬레이션 시작/일시정지 |
| R | 전체 리셋 |

### 자동 감지

- **Collision 감지**: 두 AGV 간 거리가 차량 길이 미만이면 자동 로그 + pause
- **Cycle 감지**: TAPG DAG에 순환이 발생하면 자동 로그

### 로그 파일

| 파일 | 내용 |
|------|------|
| `logs/plan_log.txt` | 매 replan마다 전체 AGV 경로 + BLOCKED_BY 정보 |
| `logs/replan_history.txt` | D키 입력 시 각 AGV의 마지막 replan 이력 (경로, constraint) |
| `collision_log.txt` | collision 발생 시 상세 정보 (TAPG cross-pred 포함) |
