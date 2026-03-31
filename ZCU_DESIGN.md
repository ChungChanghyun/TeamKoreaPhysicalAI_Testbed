# ZCU (Zone Control Unit) 설계 문서

## 1. 현재 구현 상태 (Rollback 버전)

### 구조
- **ZCU zone**: merge 노드(in-degree >= 2)마다 자동 생성
- **entry segments**: merge 노드로 들어오는 모든 세그먼트
- **exit segments**: entry segments와 동일 (entry = exit)
- 총 206개 merge ZCU (oht.large.map.json 기준)

### 동작 (Simple Lock)
```
TRY_ADVANCE (entry segment 진입 시도)
  ├─ _zcu_holders[zone] 비어있음 → lock 획득, 진입
  └─ 다른 차량이 보유 → BLOCKED, waitlist 등록

SEGMENT_DONE (entry segment 완료 = merge 노드 도착)
  └─ _release_zcu() → lock 해제, waitlist 첫 번째 차량 wake
```

### 장점
- 단순하고 deadlock 없음
- 성능 우수 (O(1) lock check)

### 문제점
1. **직선 entry에서 연달아 주행 불가**: 같은 방향이어도 한 대씩만 진입
2. **곡선 entry에서 후행 직선이 추월**: 곡선 차량이 lock 보유 중 느리게 주행하는 동안 직선 차량은 ZCU가 아닌 구간에서 자유 주행
3. **분기점(diverge) 미보호**: 다른 경로로 간 전방 차량이 front-detection에서 보이지 않음

---

## 2. 시도했던 개선 및 결과

### 2-1. ZCU lock을 merge 도착 시 획득 (occupancy-based)

**변경**: entry segment 진입 시가 아닌, merge 노드 도착 시(`_on_segment_done`) lock 획득

**의도**: 직선 entry에서 자유 queue 허용

**결과**:
- ✅ 직선 연달아 주행 가능
- ❌ 곡선 차량이 아직 주행 중인데 직선 차량이 merge를 먼저 통과 (순서 역전)
- ❌ lock 해제 타이밍 복잡화 → deadlock 발생

### 2-2. 곡선/직선 분리 (Curve occupancy + ZCU)

**변경**:
- 곡선 entry: segment queue 기반 점유 체크 (별도)
- 직선 entry: merge 도착 시 ZCU lock

**의도**: 곡선은 한 대씩, 직선은 queue 자유

**결과**:
- ✅ 곡선 한 대씩 통과
- ❌ 곡선 차량의 lock과 ZCU lock 타이밍 불일치
- ❌ "merge에 먼저 도착한 직선 차량이 아직 안 도착한 곡선 차량 때문에 BLOCKED" 역전 현상

### 2-3. Occupancy-based 점유 체크 (lock 제거)

**변경**: lock/release 대신 다른 entry segment의 `seg.queue` 직접 확인

**의도**: 실시간 상태 기반 판단, lock 타이밍 이슈 제거

**결과**:
- ✅ 같은 방향 자유 queue
- ✅ 다른 방향 차단
- ❌ 직선 차량이 곡선 차량보다 빠르게 entry 진입 → 동시 진입 발생
- ❌ node_blocker 오판 → deadlock chain

### 2-4. Diverge ZCU 추가

**변경**: 분기점(out-degree >= 2)에도 ZCU 적용

**의도**: 분기 후 다른 경로 차량 보호

**결과**:
- ✅ 분기점에서 한 방향씩 출발
- ❌ merge ZCU + diverge ZCU 조합에서 순환 대기 → deadlock
  - 차량 A: merge ZCU에서 B 대기
  - 차량 B: diverge ZCU에서 A 대기

---

## 3. 핵심 문제 분석

### Deadlock의 근본 원인
```
         ┌── ZCU_MERGE_M ──┐
    D₁ → ├── entry_a ──────┤→ M → ...
         └── entry_b ──────┘
         ↑
    ZCU_DIV_D₁ (diverge)
```

merge ZCU와 diverge ZCU가 같은 세그먼트를 서로 다른 역할로 참조:
- `entry_a`는 merge의 entry이면서 diverge의 exit
- 두 ZCU가 동시에 같은 세그먼트를 잠그면 순환 대기 발생

### 동시 진입의 근본 원인
lock/occupancy 체크 시점이 이벤트 처리 순서에 의존:
1. t=5.0: 차량A가 곡선 entry 진입 (lock 획득)
2. t=5.0: 차량B가 직선 entry 진입 시도 → lock 때문에 BLOCKED ✅

하지만:
1. t=5.0: 차량B가 직선 entry 진입 시도 (TRY_ADVANCE 먼저 처리) → lock 없음 → 진입 ✅
2. t=5.0: 차량A가 곡선 entry 진입 → lock 획득

같은 시간 t에 두 이벤트가 있으면 **처리 순서에 따라 결과가 달라짐**

---

## 4. 해결해야 할 요구사항 정리

### R1: 직선 entry — 연달아 주행 가능
같은 entry segment에서 여러 대가 queue로 주행 (front-detection 간격 유지)

### R2: 곡선(U턴) entry — 한 대씩 진입
U턴 구간에 한 대만 존재, 중간 node에 도달하면 직선 차량 출발 가능

### R3: S커브(shortcut) — 한 대씩 진입
shortcut 구간에 한 대만 존재, merge에 도달해야 후행 차량 출발

### R4: 분기점(diverge) — 전방 차량 경로 보호
분기 후 다른 경로로 간 차량이 안전 지점에 도달할 때까지 후행 차량 대기

### R5: Deadlock 방지
merge ZCU와 diverge ZCU가 순환 대기를 만들지 않아야 함

### R6: 순서 보장
merge에 먼저 도착한 차량이 먼저 통과 (도착 순서 = 통과 순서)

---

## 5. 제안하는 설계 방향

### 방안 A: Unified Zone (merge + diverge를 하나의 zone으로)

```
       D (diverge)
      / \
  [seg_a] [seg_b]  ←── 하나의 "bypass zone"
      \ /
       M (merge)
```

- bypass zone 전체에 대해 한 대씩 통과
- diverge 진입 시 lock 획득, merge 통과 시 lock 해제
- 같은 방향(seg_a→seg_a 연속)은 자유 통과

**장점**: 단일 lock으로 R1~R6 모두 충족 가능
**단점**: zone 범위가 넓어 throughput 감소

### 방안 B: 계층적 ZCU (Priority-based)

- Merge ZCU: 합류점에서 도착 순서 기반 통과
- Diverge protection: merge ZCU의 연장으로 처리 (별도 lock 없음)
- 곡선/S커브: segment-level occupancy (ZCU와 독립)

**장점**: deadlock 원천 차단 (단일 lock 계층)
**단점**: 구현 복잡도 높음

### 방안 C: Token-passing (현재 lock 대신 token 순환)

- 각 ZCU zone에 token 하나
- token을 가진 방향만 진입 가능
- token은 일정 시간 또는 대기 차량 수 기준으로 방향 전환

**장점**: 공정성 보장, deadlock 없음
**단점**: token 전환 타이밍 설계 필요

---

## 6. 현재 유지되는 수정사항 (롤백 후에도 포함)

| 수정 | 파일 | 상태 |
|------|------|------|
| 성능 최적화 (`_segs_to`, `_node_agents` 인덱스) | env_oht_des.py | ✅ 유지 |
| 세그먼트별 속도 제한 (`seg.max_speed`) | env_oht_des.py | ✅ 유지 |
| 다음 세그먼트 속도 미리 감속 | env_oht_des.py | ✅ 유지 |
| BLOCKED 원인 추적 (`block_reason`, `blocked_by`) | env_oht_des.py | ✅ 유지 |
| U턴 중간 node 추가 | visualize_oht_sim.py | ✅ 유지 |
| ZCU hover 시각화 | visualize_oht_sim.py | ✅ 유지 |
| Blocking chain 시각화 | visualize_oht_sim.py | ✅ 유지 |
| Cascade depth 제한 | env_oht_des.py | ✅ 유지 |
| Diverge ZCU | visualize_oht_sim.py | ❌ 롤백 |
| Occupancy-based ZCU | env_oht_des.py | ❌ 롤백 |
| Curve group 점유 체크 | env_oht_des.py | ❌ 롤백 |
