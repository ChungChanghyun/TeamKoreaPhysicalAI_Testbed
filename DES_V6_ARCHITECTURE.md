# graph_des_v6 — Pure DES Architecture

## 1. Overview

graph_des_v6는 OHT(Overhead Hoist Transport) 네트워크를 위한 **순수 이산 사건 시뮬레이션(Discrete Event Simulation)** 엔진이다.

v5(hybrid DES + time-stepping)와 달리, 외부 프레임 루프의 `step()` 펌핑 없이 **이벤트가 스스로 다음 이벤트를 예약하는 자기 구동(self-sustaining) 구조**로 동작한다.

```
┌─────────────────────────────────────────────────────┐
│                    Event Heap                        │
│  (min-heap, 시간순 정렬)                              │
│                                                     │
│  ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐      │
│  │SEG_END│ │PHASE │ │REPLAN│ │STOP  │ │ZCU   │      │
│  │t=3.54│ │t=7.20│ │t=8.1 │ │t=18.7│ │GRANT │      │
│  └──┬───┘ └──┬───┘ └──┬───┘ └──┬───┘ └──┬───┘      │
│     └────────┴────────┴────────┴────────┘           │
└─────────────────────────────────────────────────────┘
         │  pop (earliest)
         ▼
   ┌──────────┐
   │ _dispatch │──→ handler 실행 ──→ 다음 이벤트 push
   └──────────┘
```

---

## 2. Continuous Trajectory Guarantee

이벤트 사이 구간에서 OHT의 운동은 **등가속도(constant acceleration)**이다.

```
position(t) = seg_offset + vel × (t - t_ref) + ½ × acc × (t - t_ref)²
velocity(t) = vel + acc × (t - t_ref)
```

- `t_ref`: 마지막 이벤트 처리 시각 (kinematic state가 갱신된 시점)
- `vel`, `acc`: t_ref 시점의 속도, 가속도
- `v_max` 도달 시 cap, `v=0` 도달 시 clamp

이벤트는 **상태가 바뀌는 정확한 시점**에만 발생하고, 그 사이는 해석적(analytical) 보간이므로 **이산화 오차(discretization error)가 없다.**

임의 시각 t에 대한 위치 조회(`query_positions(t)`)는 위 공식으로 계산하며, 시뮬레이션 상태를 변경하지 않는다(non-mutating).

---

## 3. Event Types

| Event | 발생 조건 | Handler | 다음 예약 |
|---|---|---|---|
| `START` | `start_all()` 호출 | `_replan()` | 아래 replan 결과에 따름 |
| `SEG_END` | segment 끝 도달 | `_on_seg_end()` → occupancy 갱신 → `_replan()` | replan 결과 |
| `PHASE_DONE` | target speed 도달 (가속 완료) | `_on_phase_done()` → cruise 전환 → `_replan()` | replan 결과 |
| `STOPPED` | 감속 → v=0 도달 | `_on_stopped()` | ZCU wait-queue 등록 또는 REPLAN |
| `ZCU_GRANT` | ZCU zone 해제 통지 | `_on_zcu_grant()` → `_replan()` | replan 결과 |
| `REPLAN` | 제약 접근 (braking dist 도달) | `_replan()` | replan 결과 |

---

## 4. Core Decision: `_replan()`

`_replan(t, v)`는 모든 이벤트 핸들러의 최종 판단 함수이다.
한 번 호출 시 **정확히 1개의 이벤트**를 heap에 예약한다.

### 4.1 제약 계산

```
free_dist = min(leader_gap - h_min, zcu_limit, dest_dist)
target_v  = min(v_max, seg_speed, lookahead_v)
```

| 제약 | 소스 | 계산 |
|---|---|---|
| Leader gap | `gap()` | `gap_d + leader_extra - h_min` |
| ZCU limit | `_dist_to_next_zcu_node()` | 다음 blocked ZCU까지 거리 (hop 포함) |
| Destination | `_dist_to_dest()` | dest_node까지 path 거리 |
| Curve speed | `_lookahead_speed()` | `sqrt(v_slow² + 2 × d_max × dist)` |

### 4.2 Decision Tree

```
_replan(t, v)
│
├─ free_dist ≤ 0  ──────────────────────── Case 1: STOP (blocked)
│   └─ v=0, acc=0
│   └─ ZCU blocked? → wait-queue 등록 (ZCU_GRANT 대기)
│   └─ Leader blocked? → follower notify로 wakeup 대기
│
├─ free_dist > 100000 ──────────────────── Case 2: FREE RUNNING
│   AND dist_to_slow > 100000
│   └─ _go(target_v) → ACCEL or CRUISE
│   └─ _schedule_next_event() → SEG_END or PHASE_DONE
│
└─ else ────────────────────────────────── Case 3: CONSTRAINED
    │
    ├─ free_dist ≤ brake_dist ──────────── Case 3a: BRAKE NOW
    │   └─ acc = -v²/(2×free_dist)
    │   └─ → STOPPED 예약
    │
    └─ free_dist > brake_dist ──────────── Case 3b: GO (constrained)
        └─ _go(v_target) → ACCEL or CRUISE
        └─ _schedule_next_event() → SEG_END or PHASE_DONE or REPLAN
```

### 4.3 Event Scheduling (`_schedule_next_event`)

`_schedule_next_event()`는 해석적 kinematics로 **정확한** 다음 이벤트 시각을 계산한다.

```
candidates:
  1. SEG_END    = _time_to_travel(vel, acc, dist_to_seg_end, v_max)
  2. PHASE_DONE = (target_v - vel) / acc          [가속 중일 때만]
  3. REPLAN     = (replan_dist - brake_dist) / vel [순항 중일 때만]

→ 가장 빠른 것 1개를 heap에 push
```

핵심 설계: **가속 중에는 REPLAN을 예약하지 않는다.** 가속 중에는 braking distance가 매 순간 변하므로 정확한 접근 시점을 계산할 수 없다. 대신 PHASE_DONE이 먼저 발생하고, cruise 전환 후 일정한 속도에서 정확한 접근 시점을 계산한다.

---

## 5. X Marker

매 replan 시 **다음 이벤트가 발생할 위치**에 X marker를 찍는다.

```
_schedule_next_event() 끝:
  travel_dist = v._dist_traveled(best_t - t)
  _pin_marker_at_dist(v, travel_dist)

Case 3a (brake):
  _pin_marker_at_dist(v, free_dist)     ← 정지 위치

Case 1 (blocked):
  _pin_marker_at_dist(v, 0)             ← 현재 위치
```

marker는 `(x_marker_pidx, x_marker_offset)` 으로 저장되며, path가 trim될 때 pidx도 함께 조정된다.

---

## 6. ZCU (Zone Control Unit) — Event-Driven

### 6.1 v5 (polling)
```
매 replan마다: _is_zcu_blocked() → blocked이면 감속, open이면 hop
```

### 6.2 v6 (event-driven wait-queue)
```
_replan()
  └─ _dist_to_next_zcu_node(): ZCU blocked 발견
      └─ 감속 → STOPPED
          └─ _on_stopped(): _register_zcu_wait(node_id)
              └─ _zcu_waiters[node_id].append(v)
                  └─ (대기 — 이벤트 없음, heap에서 제거됨)

선행차가 ZCU zone 이탈:
  └─ _on_seg_end() → _update_occupancy()
      └─ zone 비었음 → _release_zcu_waiters(node_id)
          └─ 대기 차량에 ZCU_GRANT 이벤트 push
              └─ _on_zcu_grant() → _replan() → 재출발
```

### 6.3 ZCU Hop Logic

`_dist_to_next_zcu_node()` 내부에서 path를 전방 탐색하며 **한 번의 호출로 여러 ZCU를 건너뛴다(hop)**:

```
while dist < 100000:
    if next_node ∈ zcu_nodes:
        if dist > brake_dist:
            → marker pin, return (inf, dist)     # 아직 멀다
        if blocked:
            → return (dist, dist)                # 여기서 멈춰야
        else:
            → hop (continue to next ZCU)         # open, 건너뜀
```

---

## 7. Leader-Follower — O(k) Subscription

### 7.1 구조
```
Vehicle:
  .leader    → 내 앞 차량 (1개)
  .followers → 나를 leader로 하는 차량들 (리스트)
```

`assign_leaders()`에서 segment occupancy 기반으로 leader 배정 + followers 리스트 구축.

### 7.2 Notification

v5: `_notify_followers()` → **전체 N대 순회** O(N)
v6: `_notify_followers(t, v)` → **v.followers만 순회** O(k)

```python
def _notify_followers(self, t, v):
    for f in v.followers:
        if f.state == STOP and f.waiting_at_zcu is None:
            _post(t, REPLAN, f)       # 정지 상태 follower 깨우기
        else:
            _invalidate(f)
            _replan(t, f)             # 주행 중 follower 재계획
```

재진입 방지: `_in_notify` 플래그로 `_replan → _notify → _replan → _notify → ...` 무한 재귀 차단.

---

## 8. Simulation vs Rendering 분리

```
┌──────────────────────┐     ┌──────────────────────┐
│   DES Engine          │     │   Renderer (pygame)   │
│                      │     │                      │
│  run_until(t_end)    │     │  query_positions(t)  │
│    └─ pop events     │     │    └─ v.update_render│
│    └─ dispatch       │     │       (non-mutating) │
│    └─ push events    │     │    └─ v.x, v.y 읽기  │
│                      │     │                      │
│  State: vel, acc,    │     │  State: 읽기만        │
│  seg_offset 변경     │     │                      │
└──────────────────────┘     └──────────────────────┘
```

- `run_until(t)`: 이벤트만 처리, 렌더링 없음
- `query_positions(t)`: 해석적 보간으로 위치 계산, 시뮬레이션 상태 불변
- `step(t)`: 하위 호환 wrapper (`run_until` + `query_positions`)

---

## 9. Event Chain Examples

### 9.1 제약 없음 (1대, ZCU 없음)
```
START → ACCEL(a=500)
  → SEG_END (seg 0→1, t=3.54)
  → SEG_END (seg 1→2, t=5.01)
  → SEG_END (seg 2→3, t=6.14)
  → PHASE_DONE (v=3600, t=7.20) → CRUISE
  → SEG_END (seg 4→5, t=7.96)
  → SEG_END (seg 5→6, t=8.83)
  → ... (무한 반복, STOPPED 없음)
```

### 9.2 Leader 제약 (circular, 20대)
```
START → ACCEL(a=500)
  → SEG_END (seg 0→1)
  → SEG_END (seg 1→2)
  → PHASE_DONE (v=2364) → leader 접근, DECEL 전환
  → STOPPED (v=0) → leader에 의해 blocked
  → (follower notify로 wakeup 대기)
  → REPLAN → ACCEL(a=500)
  → ... (반복)
```

### 9.3 ZCU 제약 (network, merge zone)
```
CRUISE(v=3600)
  → REPLAN (braking dist ≈ ZCU dist)
      └─ _dist_to_next_zcu_node():
         ZCU_1 open → hop
         ZCU_2 blocked → free_dist = 5000
  → DECEL(a=-decel)
  → STOPPED(v=0) → _register_zcu_wait(ZCU_2)
  → (heap에서 제거, 대기)
  → ZCU_GRANT (선행차 이탈 시 통지)
  → _replan() → ACCEL → ...
```

---

## 10. Kinematics Helper

```python
_time_to_travel(v0, acc, dist, v_max) → float
```

주어진 초기속도, 가속도, 이동거리, 최대속도에서 **정확한 소요 시간**을 해석적으로 계산.

| Case | 계산 |
|---|---|
| acc > 0 (가속) | v_max 도달 전: 2차 방정식. 도달 후: 잔여거리/v_max |
| acc < 0 (감속) | v=0 전에 도달: 2차 방정식. 도달 못함: inf |
| acc = 0 (등속) | dist / v0 |

이 함수가 `_schedule_next_event()`에서 SEG_END 시각을 정확히 계산하는 데 사용된다.

---

## 11. File Structure

```
graph_des_v5.py          ← Map classes (GraphMap, MapSegment, ZCUZone, etc.)
graph_des_v6.py          ← Pure DES engine (Vehicle, Event, GraphDESv6)
test_graph_v6.py         ← oht.large.map.json visualizer
test_circular_v6_graph.py ← circular.map.json visualizer
```

graph_des_v6는 graph_des_v5에서 Map 구조체(`GraphMap`, `MapSegment`, `ZCUZone` 등)를 import하고, 엔진 부분(`Vehicle`, `Event`, `GraphDESv6`)만 새로 구현한다.
