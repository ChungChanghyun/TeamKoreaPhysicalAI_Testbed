# ZCU Logic for v3 Engine

## 개요

합류점(merge)과 분기점(diverge)에서 곡선/직선 세그먼트 간의 충돌을 방지하는 로직.
ZCU 우선권 로직은 별도로 추가 예정 — 여기서는 상호 배제만 다룸.

## 세그먼트 분류

### 곡선 세그먼트 (curve)
- `path_points > 2` (Bezier 보간점 존재)
- U턴, S커브, shortcut 등
- **한 대씩만 진입 가능**

### 직선 세그먼트 (straight)
- `path_points <= 2` (시작점, 끝점만)
- **여러 대 연달아 주행 가능** (자유 queue)

## 합류점 (Merge) 보호

```
직선 entry ──→ M (merge node)
곡선 entry ──→ M
```

### 규칙
| 현재 lock 보유 | 직선 진입 | 곡선 진입 |
|---------------|----------|----------|
| 없음          | 자유     | 자유 (lock 획득) |
| 직선          | 자유 queue | 대기 |
| 곡선          | 대기     | 대기 |

### 구현
- `bypass_curve_segs`: 합류점의 곡선 entry 세그먼트 집합
- `bypass_straight_segs`: 합류점의 직선 entry 세그먼트 집합 (곡선 counterpart가 있는 경우만)
- 곡선 진입: lock 획득, 다른 차량 대기
- 직선 진입: lock holder가 곡선이면 대기, 직선이면 자유 통과
- Lock 해제: 세그먼트 완료 시. 직선이고 queue 남아있으면 다음 차량에 이전.

### U턴 후반부 (mid→merge)
- 곡선 세그먼트이지만 **직선으로 취급** (`from_id`가 `_curve_mid_`이면)
- 이미 U턴에 committed된 상태이므로 감속할 필요 없음

## 분기점 (Diverge) 보호

```
D (diverge) ──→ 직선 exit
             └──→ 곡선 exit
```

### 규칙
| 상대 exit에 차량 | 직선 출발 | 곡선 출발 |
|-----------------|----------|----------|
| 없음            | 자유     | 자유     |
| 직선            | 자유     | 대기     |
| 곡선            | 대기     | 대기     |

### 구현
- `diverge_curve_exits`: 분기점의 곡선 exit 세그먼트 집합
- `diverge_straight_exits`: 분기점의 직선 exit 세그먼트 집합
- 상대 exit의 `seg.queue`를 직접 확인 (lock 없음, 실시간 점유 체크)
- 곡선끼리: 한 대씩
- 직선과 곡선: 상호 배제
- 직선끼리: 자유

## U턴 Curve Group

```
diverge ──[전반부]──→ MID ──[후반부]──→ merge
```

### 규칙
- 전반부 진입 시: 후반부에 차량 있으면 대기 (+ 자기 세그먼트 queue도 확인)
- 후반부 진입 시: 체크 없음 (이미 committed)
- 전반부와 후반부는 `_curve_group` ID로 연결

### 구현
- `curve_groups`: gid → [(fn,tn), (fn,tn)] 매핑
- 진입 시 `getattr(seg, '_curve_group', None)`으로 그룹 확인
- `from_id.startswith('_curve_mid_')`이면 후반부 → 체크 skip

## v3 엔진에서의 통합 방법

v3 엔진은 `_plan()`에서 모든 판단을 하므로, ZCU도 `_plan()` 안에서 처리:

```python
def _plan(t, v):
    v.set_state(t)

    # 1. 현재 세그먼트가 ZCU 관련인지 확인
    seg_key = (v.current_from, v.current_to)

    # 2. Merge ZCU: 곡선 entry면 lock 확인
    #    → lock 보유자 있으면 free_dist = 0 (정지)

    # 3. Diverge: 상대 exit에 차량 있으면 free_dist = 0

    # 4. Curve group: U턴 전반부면 후반부 확인

    # 5. 일반 _plan 로직 (v_safe, RESUME 등)
```

### 핵심: ZCU는 `free_dist`를 0으로 만드는 것

- ZCU에 의해 blocked → `free_dist = 0` → `v_safe = 0` → 정지
- ZCU 해제 → `_notify_followers` → `_plan` 재호출 → `free_dist > 0` → 출발
- 별도의 BLOCKED 상태나 lock/waitlist 불필요
- 기존 _plan의 정지/재출발 메커니즘이 그대로 작동
