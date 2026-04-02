# graph_des_v6 — Pure DES Architecture

## 1. 설계 원칙

- **Pure DES**: 이벤트가 자기 자신의 다음 이벤트를 예약하는 자기 구동 구조
- **Committed Events**: 한번 heap에 들어간 이벤트는 **절대 취소되지 않음** (`_invalidate` 없음)
- **연속 궤적 보장**: 이벤트 사이 구간은 등가속도 운동, 해석적 보간으로 이산화 오차 없음
- **시뮬레이션/렌더링 분리**: `run_until(t)` + `query_positions(t)`

```
position(t) = seg_offset + vel × (t - t_ref) + ½ × acc × (t - t_ref)²
velocity(t) = vel + acc × (t - t_ref)
```

---

## 2. 이벤트 타입

| Event | 발생 조건 | 역할 |
|---|---|---|
| `START` | `start_all()` | 초기 plan 생성 |
| `SEG_END` | segment 끝 도달 | occupancy 갱신, ZCU exit release, replan |
| `PHASE_DONE` | target speed 도달 | cruise 전환, replan |
| `BOUNDARY` | plan boundary의 braking point 도달 | **ZCU lock 시도 (유일한 장소)** |
| `STOPPED` | v=0 도달 | 정지, REPLAN 예약 |
| `REPLAN` | 주기적 재시도 | leader/dest 제약 재평가 |
| `ZCU_GRANT` | ZCU lock 해제 통지 | 대기 차량 재개 |

---

## 3. 핵심 구조: Plan과 Lock의 분리

### _replan() — Plan만 생성, Lock은 건드리지 않음

```
_replan(t, v):
  1. 위치 갱신 (advance_position)
  2. 제약 계산:
     - 다음 ZCU boundary까지 거리 (lock 시도 안 함, 거리만 계산)
     - leader free_dist (committed event 기반)
     - dest 거리
  3. plan_boundary = min(ZCU, leader, dest)
  4. marker를 plan_boundary에 pin
  5. 이벤트 예약: SEG_END, PHASE_DONE, BOUNDARY 중 가장 빠른 것
```

### _on_boundary() — Lock 시도하는 유일한 장소

```
_on_boundary(t, v):
  1. marker에 기록된 boundary node 확인
  2. _relevant_zones(): 해당 node의 관련 zone 목록
     - Diverge: 무조건 포함 (물리적 node 공유)
     - Merge: path가 bnd_node → merge_node 방향인 경우만
  3. 모든 관련 zone에 lock request (atomic)
     - 전부 granted → passed_zcu에 추가 → _replan (plan 확장)
     - 하나라도 denied → 감속/정지 → _zone_wait (ZCU_GRANT 대기)
```

---

## 4. 이벤트 체인

### 4.1 정상 주행 (ZCU 통과)

```
START → _replan
  → plan: boundary B1까지
  → ACCEL(a=500)

SEG_END → _replan
  → 여전히 B1 방향, 가속 계속
  → SEG_END 또는 PHASE_DONE 예약

PHASE_DONE → cruise 전환 → _replan
  → BOUNDARY 예약 (B1까지 braking point)

BOUNDARY fire (B1 braking point)
  → lock request → GRANTED
  → passed_zcu.add(B1)
  → _replan: 다음 boundary B2까지 plan
  → ACCEL/CRUISE → SEG_END → ... → BOUNDARY(B2) → ...
```

### 4.2 ZCU 대기

```
BOUNDARY fire (B1 braking point)
  → lock request → DENIED
  → 감속 시작 (DECEL)
  → SEG_END (중간 segment) → _replan → 계속 감속
  → STOPPED (B1 앞에서 v=0)
  → REPLAN 0.2초 후 예약

  (다른 OHT가 ZCU 통과 완료 → SEG_END에서 exit release)
  → ZCU_GRANT → _replan → 새 plan → BOUNDARY → lock request → ...
```

### 4.3 Leader 추종

```
_replan:
  → leader_free = gap + leader_extra - h_min
  → plan_boundary = min(ZCU, leader_free)
  → leader가 가까우면: plan_boundary = leader_free
  → 감속하여 leader 뒤에서 정지
  → STOPPED → REPLAN 0.5초 → leader 이동 확인 → 재출발
```

---

## 5. ZCU Lock 시스템

### 5.1 구조

```
ZCU Zone = Merge 또는 Diverge 지점
Lock ID = "{zone.node_id}_{zone.kind}"  (예: "1077_merge", "3316_diverge")

boundary node: OHT가 ZCU에 진입하기 전 정지/판단하는 node
  - Merge:  boundary = merge node의 predecessor (A, B)
  - Diverge: boundary = diverge node 자체

exit node: OHT가 ZCU를 빠져나가는 node (lock release 지점)
  - Merge:  exit = merge node 자체
  - Diverge: exit = diverge의 successor nodes
```

### 5.2 Lock 획득 (오직 _on_boundary에서)

```
_on_boundary:
  1. _relevant_zones(v, bnd_node) → 관련 zone 목록
  2. 각 zone에 _zone_request(v, lock_id)
  3. 전부 granted → 통과, plan 확장
  4. 하나라도 denied → 감속 정지, _zone_wait → ZCU_GRANT 대기
```

### 5.3 Lock 해제 (SEG_END의 _process_crossed_nodes에서)

```
_on_seg_end / _on_stopped / _on_phase_done:
  → advance_position → crossed nodes 반환
  → _process_crossed_nodes:
     각 crossed node에 대해:
       _exit_to_zones[node]에 등록된 zone 확인
       해당 zone의 lock holder가 이 OHT이면 → _zone_release
       → _zone_lock = None
       → waiter가 있으면 첫 번째 waiter에게 ZCU_GRANT 발송
```

### 5.4 Merge+Diverge 겹침

하나의 node가 merge이면서 diverge인 경우:
```
  A ──\    /── C
       ●
  B ──/    \── D

lock_id가 별도: "●_merge", "●_diverge"
boundary node에서 _relevant_zones가 필터:
  - diverge: 무조건 lock
  - merge: path 방향 확인 후 lock
```

### 5.5 _relevant_zones 필터 로직

```python
for zone, lock_id in boundary_to_zones[bnd_node]:
    if zone.kind == 'diverge':
        → 항상 포함 (물리적 node 공유, 방향 무관)
    elif zone.kind == 'merge':
        → path[i] == bnd_node and path[i+1] == zone.node_id 인 경우만 포함
```

---

## 6. _schedule_next_event — 정확한 이벤트 시점 계산

```
candidates:
  1. SEG_END:    _time_to_travel(vel, acc, dist_to_seg_end, v_max)
  2. PHASE_DONE: (target_v - vel) / acc  [가속 중]
  3. BOUNDARY:   가속 중 → _time_to_boundary_during_accel (삼각형 프로파일)
                 순항 중 → (plan_boundary - brake_dist) / vel
  4. REPLAN:     curve 감속 접근 (dist_to_slow)

→ 가장 빠른 것 1개를 heap에 push
```

### 삼각형 프로파일 (가속 중 BOUNDARY 계산)

가속하면서 boundary에 정지하기 위한 감속 시작 시점:
```
v_peak = v0 + a_acc × t₁
accel_dist = v0×t₁ + ½×a_acc×t₁²
brake_dist = v_peak² / (2×d_max)
accel_dist + brake_dist = boundary_dist

→ 2차 방정식으로 t₁ 해석적 계산
```

---

## 7. Leader 관리

### leader 갱신
- `_update_leader(v)`: 매 `_replan` 호출 시 전방 15000mm 이내 vehicle 탐색
- 전체 vehicle을 v의 forward path segments와 대조

### leader 기반 free_dist
```python
gap_d = path 기반 거리
leader_extra = leader의 committed event까지 확정 이동거리 + braking distance
leader_free = min(gap_d + leader_extra - h_min, gap_d - h_min)
```
- leader의 event는 committed (cancel 불가) → leader_extra는 정확한 최소 보장 거리
- leader 상태 변경 통지(notify) 없음 → follower는 자기 이벤트에서 재확인

---

## 8. Segment Occupancy

- `_seg_occupants`: rendering/gap 계산용 (ZCU lock과 무관)
- `_update_occupancy`: segment 변경 시 갱신
- `_process_crossed_nodes`: advance_position이 여러 segment를 한번에 넘을 때 
  모든 중간 node에서 ZCU exit release 수행

---

## 9. X Marker

- marker = plan boundary 위치 (OHT가 최종적으로 정지할 수 있는 지점)
- ZCU boundary가 제약이면: boundary node에 pin
- leader/dest가 제약이면: 해당 거리에 pin
- marker는 OHT→marker 연결선으로 시각화

---

## 10. 파일 구조

```
graph_des_v5.py           ← Map 구조체 (GraphMap, MapSegment, ZCUZone 등)
graph_des_v6.py           ← Pure DES 엔진 (Vehicle, Event, GraphDESv6)
test_graph_v6.py          ← oht.large.map.json 시각화 (pygame)
test_circular_v6_graph.py ← circular.map.json 시각화
test_boundary_fix.py      ← 단일 OHT boundary 검증
DES_V6_ARCHITECTURE.md    ← 이 문서
```
