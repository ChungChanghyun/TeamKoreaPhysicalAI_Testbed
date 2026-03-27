# OHT DES Simulation Engine — Design Specification

## Overview

OHT(Overhead Hoist Transport) 전방감지 기반 DES(Discrete Event Simulation) 엔진.
Automod Path Mover의 이벤트 구동 방식을 참고하여, 세그먼트 단위 주행과 cross-segment 전방감지를 구현.

파일: `mapf_edu/env_oht_des.py`

---

## 1. Map 구조

### OHTNode
- `id`: 노드 ID (문자열)
- `x, y`: 좌표 (mm)

### OHTSegment
- `from_id → to_id`: 방향성 있는 세그먼트
- `length`: 경로 길이 (mm, 곡선이면 유클리드 거리보다 김)
- `queue`: 이 세그먼트 위의 에이전트 리스트 (front → back 순서)

### OHTMap
- `nodes`, `segments`, `adj` (인접 리스트)
- `h_min`: 안전거리 = vehicle_length + margin (현재 1308mm)
- `max_brake`: 최대 속도에서의 제동거리 = v_max² / (2 × decel)
- `accel`, `decel`: 가속/감속도 (mm/s²)
- ZCU zones: merge 구간 상호배제

---

## 2. Agent 상태

| 상태 | 설명 |
|------|------|
| `IDLE` | 노드에 위치, 다음 세그먼트 진입 대기 |
| `MOVING` | 세그먼트 위 주행 중 (최대 속도 지향) |
| `FOLLOWING` | 앞차 때문에 감속/정지 중 |
| `BLOCKED` | 노드에서 세그먼트 진입 불가 (gap < h_min 또는 ZCU) |
| `DONE` | 경로 완주 |

### Agent 운동학
- `v`: 현재 속도 (mm/s)
- `a`: 현재 가속도 (양수=가속, 음수=감속, 0=등속)
- `v_cap`: 목표 속도
- `seg_entry_t`, `seg_entry_pos`: 세그먼트 진입 시점/위치
- `pos(t)`: 시간 t에서의 세그먼트 상 위치 (등가속 운동 공식)
- `vel(t)`: 시간 t에서의 순간 속도

### Leader-Follower 관계 (직접 등록 방식)
- `agent.leader`: 내가 따르고 있는 앞차 (또는 None)
- `agent.followers`: 나를 따르고 있는 뒷차 리스트

등록/해제:
- `_follow(follower, leader)`: follower를 leader의 followers에 등록
- `_unfollow(follower)`: follower를 현재 leader에서 해제
- `_notify_followers(t, leader)`: leader의 모든 followers에게 GAP_CLEAR 이벤트 발송

---

## 3. 이벤트 (4종)

| 이벤트 | 발생 시점 | 의미 |
|--------|----------|------|
| `TRY_ADVANCE` | 세그먼트 진입 시도 | 노드의 에이전트가 다음 세그먼트 진입을 시도 |
| `SEGMENT_DONE` | 세그먼트 끝 도달 | 운동학적으로 세그먼트 length에 도달 |
| `CATCH_UP` | 감속 시작 시점 | 현재 속도로 가면 leader와 h_min 이내로 접근할 시점 |
| `GAP_CLEAR` | 앞차 상태 변화 | leader가 출발/가속하여 gap이 확보됨 → 재가속 가능 |

### Token 기반 이벤트 무효화
- `agent.token`: CATCH_UP, GAP_CLEAR, SEGMENT_DONE용. 세그먼트 진입 시 증가.
- `agent.adv_token`: TRY_ADVANCE 전용. 재시도 예약 시 증가.
- 이벤트 처리 시 token 불일치 → stale로 무시.

---

## 4. 이벤트 처리 흐름

### 4.1 TRY_ADVANCE

에이전트가 노드에서 다음 세그먼트 진입을 시도.

```
1. 경로 끝? → DONE
2. 진입 세그먼트의 queue tail / to_id node_blocker 확인
   gap < h_min → BLOCKED, wait 후 재시도
3. [cross_segment] 짧은 세그먼트(<h_min) 연쇄 탐색 + 한 step 더
   누적 거리 < h_min이고 blocker 있으면 → BLOCKED
4. ZCU 체크 → lock 못 잡으면 BLOCKED
5. 통과 → 세그먼트 진입:
   - queue에 추가, MOVING
   - token++
   - SEGMENT_DONE 예약 (운동학 계산)
   - _notify_followers: 노드를 떠났으므로 등록된 followers에게 GAP_CLEAR
   - _signal_node_unblocked: BLOCKED 에이전트 깨움
   - queue 앞에 차 있으면 → _schedule_catch_up (leader 등록)
   - queue 맨앞이면 → _schedule_front_catchup (cross-seg leader 탐색)
```

### 4.2 SEGMENT_DONE

에이전트가 세그먼트 끝에 도달.

```
1. queue에서 제거
2. _notify_followers: 등록된 모든 followers에게 GAP_CLEAR
3. 속도 보존, seg = None, IDLE
4. ZCU exit → lock 해제, 대기자 깨움
5. _wake_blocked_for: 이 세그먼트에 BLOCKED된 에이전트 깨움
6. TRY_ADVANCE 즉시 게시 → 다음 세그먼트 진입 시도
```

### 4.3 CATCH_UP

앞차와 h_min까지 접근할 시점에 도달.

```
[queue 맨앞 (front)]
1. _look_ahead_leader로 cross-segment leader 탐색
2. leader 발견:
   a. gap > h_min이고 정지 중(FOLLOWING) → leader 해제, 재가속(MOVING), followers 알림
   b. dt <= 1e-6 → _slow_and_cascade (즉시 감속)
   c. dt > 0 → CATCH_UP 재예약
3. leader 없음(result=None):
   FOLLOWING이면 → leader 해제, 재가속(MOVING), followers 알림

[queue 중간]
→ 바로 앞차(queue[idx-1]) 속도에 맞춰 _slow_and_cascade
```

### 4.4 GAP_CLEAR

leader의 상태 변화로 gap이 확보됨.

```
1. state != FOLLOWING → 무시 (이미 재개됨)
2. 이전 leader 해제 (_unfollow) — scheduling 전에!
3. 재가속 (_speed_up), MOVING
4. SEGMENT_DONE 재예약
5. 앞차 확인:
   - queue 중간 → _schedule_catch_up (새 leader 등록)
   - queue 맨앞 → _schedule_front_catchup (cross-seg leader 탐색)
6. _notify_followers: 자기 followers에게 GAP_CLEAR
```

**중요**: `_unfollow`는 반드시 `_schedule_front_catchup`/`_schedule_catch_up` **전에** 호출.
scheduling 안에서 새 leader가 등록될 수 있으므로, 순서가 바뀌면 새 leader 등록이 즉시 해제되어 교착 발생.

---

## 5. 핵심 보조 함수

### _look_ahead_leader(t, agent, seg) → (gap, speed, leader_agent) | None

세그먼트 맨앞 에이전트의 전방 탐색.

```
remaining = seg.length - agent.pos(t)
look_node = seg.to_id

while True:
    node_blocker at look_node? → return (gap, 0, blocker)
    다음 세그먼트에 queue 있으면? → return (gap + front.pos, front.vel, front)
    세그먼트 길이 누적
    길이 >= h_min? → 한 번 더 node_blocker 확인 후 종료
return None
```

- `cross_segment=False`: to_id의 node_blocker만 확인
- `cross_segment=True`: h_min 미만 세그먼트를 연쇄 탐색 + 한 step 더

### _schedule_front_catchup(t, agent, seg)

front-of-queue 에이전트의 CATCH_UP 예약.

```
result = _look_ahead_leader(...)
None → leader 해제. FOLLOWING이면 재가속 + followers 알림
있음 → _follow(agent, leader)
  정지 중 → return (GAP_CLEAR로 깨워질 때까지 대기)
  _accel_brake_dt로 감속 시점 계산
  dt <= 1e-6 → 즉시 _slow_and_cascade
  dt > 0 → CATCH_UP 예약
```

### _schedule_catch_up(t, follower, leader)

같은 세그먼트 내 follower의 CATCH_UP 예약.

```
_follow(follower, leader)
vf <= vl → return (접근 안 함)
_accel_brake_dt로 감속 시점 계산 → CATCH_UP 예약 또는 즉시 감속
```

### _slow_and_cascade(t, agent, speed)

감속 처리 + 연쇄 전파.

```
agent._slow_to(speed) → FOLLOWING
SEGMENT_DONE 재예약 (감속 반영)
같은 seg follower에게 _schedule_catch_up (연쇄 감속)
```

### _notify_followers(t, leader)

leader의 상태가 변했을 때 등록된 모든 followers에게 GAP_CLEAR 발송.

호출 시점:
- SEGMENT_DONE (세그먼트 이탈)
- _on_gap_clear (재가속)
- _on_catch_up resume (전방 clear)
- reassign (DONE → 재배치)
- TRY_ADVANCE 진입 성공 (노드 이탈)

### _signal_node_unblocked(t, node_id)

노드가 비었을 때 해당 노드로 향하는 세그먼트의:
- FOLLOWING 에이전트에게 GAP_CLEAR
- BLOCKED 에이전트에게 TRY_ADVANCE 재시도
- cross-segment: 짧은 세그먼트 + 한 step 역방향 전파

---

## 6. 진입 시 Cross-Segment 체크

`_on_try_advance`에서 세그먼트 진입 전에 수행.

```
누적 거리 = seg.length (진입하려는 세그먼트)
경로를 따라 다음 세그먼트들 순회:
  - 중간 노드에 node_blocker? 누적거리 < h_min → BLOCKED
  - 세그먼트에 queue? 누적거리 + front.pos < h_min → BLOCKED
  - 세그먼트 길이 누적
  - 길이 >= h_min → 한 번 더 node_blocker 확인 후 종료
```

탐색 중단 기준: 세그먼트 길이 >= h_min (이 안에서 안전거리 확보 가능).
단, 항상 한 step(node) 더 확인하여 긴 세그먼트 끝의 blocker도 감지.

---

## 7. 초기 배치

`OHTMap.nearby_nodes(node_id, h_min)` — 양방향(forward+reverse) BFS로 h_min 이내 노드 집합 반환.

배치 알고리즘:
```
excluded = set()
for each agent:
    random node not in excluded, with bfs_path > 1
    excluded |= nearby_nodes(start, h_min)
```

---

## 8. 알려진 Gap 위반 원인

20개 seed, 8대 OHT 테스트 결과 (120초 시뮬레이션):

| 유형 | 빈도 | 원인 | 심각도 |
|------|------|------|--------|
| BOTH_NODE (0mm) | 3/20 | reassign 시 같은 노드에 두 OHT 배치 | 높음 — 로직 버그 |
| SAME_SEG (4~81mm) | 6/20 | CATCH_UP 감속 타이밍 부정확 (운동학 이산화 오차) | 중간 |
| CROSS_SEG (800mm) | 11/20 | 곡선 세그먼트의 경로 길이 vs 유클리드 거리 차이 | 낮음 — 구조적 한계 |

### BOTH_NODE (reassign 충돌)
DONE 에이전트가 `reassign`될 때 다른 에이전트와 같은 노드에 위치.
현재 `reassign`에서 주변 에이전트 확인 없음.

### SAME_SEG (CATCH_UP 타이밍)
`_accel_brake_dt`가 leader 등속 가정으로 감속 시점을 계산.
leader가 동시에 감속하면 실제 gap이 예측보다 빨리 줄어듦.

### CROSS_SEG (곡선 거리)
경로 거리(세그먼트 길이 합)로 h_min을 만족해도
유클리드 거리가 더 짧을 수 있음. 구조적으로 완전 해결 어려움.

---

## 9. ZCU (Zone Control Unit)

merge/diverge 노드의 상호배제.

- `entry_segs`: ZCU 진입 세그먼트 → lock 획득
- `exit_segs`: ZCU 퇴출 세그먼트 → lock 해제
- lock 못 잡으면 BLOCKED, waitlist에 등록
- lock 해제 시 waitlist 첫 번째 에이전트에게 TRY_ADVANCE

---

## 10. 설계 원칙

1. **이벤트 드리븐**: 주기적 폴링 없음. 모든 상태 변화는 관련 에이전트에게 명시적 이벤트로 전파.
2. **Leader-Follower 직접 등록**: 세그먼트/노드 역추적이 아니라, 감속 원인인 leader에 직접 등록. leader 상태 변화 시 followers에게 GAP_CLEAR.
3. **Token 기반 무효화**: 세그먼트 진입 시 token 증가. stale 이벤트 자동 무시.
4. **Cross-segment 전방감지**: h_min 미만 세그먼트를 연쇄 탐색 + 한 step 더. 진입 전(BLOCKED)과 주행 중(CATCH_UP) 모두 적용.
5. **Automod 참고**: claim/release 패턴, compute events → wait for next event 순환 (Automod User Guide p.507, p.518, p.825).
