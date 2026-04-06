# 다음 세션 작업 지시

## 현재 상태
`graph_des_v6.py`는 OHT 네트워크용 Pure DES 엔진이다. `DES_V6_ARCHITECTURE.md`에 전체 구조가 문서화되어 있다.

## 핵심 구조
- **Committed Events**: heap에 넣은 이벤트는 절대 취소 불가 (`_invalidate` 없음)
- **Plan**: `_replan()`이 다음 ZCU boundary까지의 velocity profile을 생성하고, 해당 구간의 모든 이벤트(SEG_END, PHASE_DONE, BOUNDARY)를 **한번에** heap에 push
- **ZCU Lock**: `_on_boundary()`에서만 lock 시도. `_replan()`은 lock을 건드리지 않음
- **SEG_END**: occupancy/exit release만 수행, replan 안 함
- **PHASE_DONE**: 상태 변경만, replan 안 함
- **BOUNDARY**: plan의 마지막 이벤트. ZCU lock 시도 후 새 plan 생성

## 해결해야 할 문제: Leader-aware velocity profile

### 현재 문제
`_schedule_plan_events()`에서 leader의 committed trajectory를 반영하여 안전 속도를 제한하려 했으나:
1. BOUNDARY event가 13만번 폭발 (leader constraint와 ZCU boundary 계산이 간섭)
2. 음수 gap 발생 (leader trajectory extrapolation이 next_event_t 이후에 부정확)
3. OHT들이 서로 뚫고 지나감

### 해결 방향 (합의됨)

**Leader를 우선, ZCU를 그 다음에 처리:**

```
1단계: leader의 committed trajectory 기반 safe distance 계산
  - leader.vel, leader.acc, leader.t_ref, leader.next_event_t 사용
  - v_safe = sqrt(v_leader² + 2·d_max·(gap - h_min))
  - sim_t > leader.next_event_t이면: leader_extra_dist를 상한으로 사용

2단계: plan 범위 = min(leader_safe_dist, ZCU_boundary_dist)

3단계: _schedule_plan_events에서:
  - 매 simulation step마다 leader 위치를 계산하여 v_safe 제한
  - leader 제약으로 감속/정지가 필요하면 그에 맞는 PHASE_DONE/STOPPED 생성
  - ZCU boundary가 범위 내에 있으면 마지막에 BOUNDARY event 추가
  - leader에 의한 감속과 ZCU boundary 감속을 분리하여 간섭 방지
```

### 구현 시 주의사항
- `_go()` 호출로 초기 acc 설정 후 `_schedule_plan_events`에 위임
- leader_gap 계산은 `self.gap(v, t)`로 path 기반 거리 사용
- leader의 `_dist_traveled(dt)`는 `next_event_t - t_ref` 까지만 유효
- BOUNDARY 폭발 방지: leader constraint로 속도가 줄어도 BOUNDARY 위치는 ZCU까지의 고정 거리
- `_on_seg_end`에서 replan 안 함 — 이미 commit된 이벤트가 heap에 있음

## 관련 파일
- `graph_des_v6.py`: 엔진 (수정 대상)
- `graph_des_v5.py`: Map 구조체 (import만)
- `test_graph_v6.py`: pygame 시각화
- `DES_V6_ARCHITECTURE.md`: 아키텍처 문서
- `oht.large.map.json`: 테스트 맵

## 테스트 방법
```python
# 200대 OHT, 60초
import random; random.seed(42)
from graph_des_v5 import GraphMap, random_safe_path
from graph_des_v6 import GraphDESv6, Vehicle

gmap = GraphMap('oht.large.map.json')
des = GraphDESv6(gmap)
# ... 200대 spawn ...
des.start_all()
for t in range(1, 61): des.run_until(float(t))
```

검증 기준:
- moving=200 (전원 주행)
- merge_violations=0
- gap violations=0 (min_gap > vehicle.length)
- BOUNDARY events: 수천 이하 (폭발 안 함)
