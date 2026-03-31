# OHT DES v3 — 주행 이벤트 구조

## 핵심 원리

- 각 OHT는 track 상의 **절대 위치(mm)**로 관리
- 위치/속도는 **등가속도 운동 방정식**으로 계산 (이벤트 사이 보간)
- 모든 주행 판단은 **`_plan()`** 하나에서 수행
- 이벤트는 **판단 시점을 예약**하는 것이지, 동작 자체를 예약하는 것이 아님

## 운동 방정식

```
pos(t) = pos₀ + vel₀ · dt + ½ · acc · dt²
vel(t) = vel₀ + acc · dt

제한: vel ∈ [0, v_max]
      가속 시 v_max 도달하면 등속 전환
      감속 시 0 도달하면 정지
```

## 이벤트 종류

| 이벤트 | 발생 시점 | 하는 일 |
|--------|----------|--------|
| `START` | t=0 | `_plan()` 최초 호출 |
| `RESUME` | 판단 필요 시점 | `_plan()` 재호출 → 가감속 재결정 |
| `STOPPED` | 감속 완료 시점 | vel=0 설정, loading 확인, `RESUME` 대기 |

**모든 이벤트는 결국 `_plan()`을 호출합니다.**

## `_plan()` 판단 흐름

```
_plan(t, vehicle):
    1. set_state(t)  — 현재 pos/vel 갱신 (운동방정식 기반)
    2. gap 계산      — 앞차까지의 track 거리
    3. free_dist 계산 — gap - h_min (안전거리 뺀 주행 가능 거리)
    4. v_safe 계산    — sqrt(2 · d_max · free_dist) (free_dist 내 정지 가능 최대 속도)
    5. v_target 결정  — min(v_max, v_safe) + 앞차 속도 고려
    6. 판단:
       ├─ free_dist > brake_dist → 여유 있음 → 가속/cruise + 다음 RESUME 스케줄
       └─ free_dist ≤ brake_dist → 감속 필요 → 감속 시작 + STOPPED/RESUME 스케줄
```

## RESUME 스케줄 타이밍

### 여유 있을 때 (cruise/accel)
```
braking point = free_dist - future_brake_dist
  → "여기까지 가면 감속을 시작해야 한다"
  → 해당 위치 도달 시간을 계산하여 RESUME 스케줄

가속 중: t_accel (v_target 도달 시간) + t_coast (braking point까지 등속 시간)
등속 중: t_coast = coast_dist / vel
```

### 감속 중일 때
```
감속 50% 지점에서 RESUME 스케줄
  → _plan() 재호출 → 앞차 상태 재확인
  → 앞차 출발했으면 → 감속 취소, 가속 전환
  → 앞차 여전히 정지 → 감속 계속, 또 50% 지점에 RESUME
```

### 정지 상태일 때
```
0.2초 후 RESUME 스케줄
  → _plan() 재호출 → gap 확인 → 여유 생기면 출발
```

## `_notify_followers()` — 앞차 상태 변화 전파

```
앞차가 정지/출발할 때 호출
  → 모든 active follower(ACCEL/CRUISE/DECEL)의 기존 이벤트 invalidate
  → 즉시 _plan() 재호출
  → follower가 새 상황에 맞게 가감속 재결정
```

## 안전 보장 메커니즘

### v_safe에 의한 속도 제한
```
v_safe = sqrt(2 · d_max · free_dist)

의미: "지금 당장 앞차가 정지해도, 이 속도 이하면 free_dist 안에서 멈출 수 있다"
  → 어떤 상황에서도 v_safe 이하로만 주행하면 충돌 불가
  → 앞차 멀면 v_safe 높음 → 빠르게 주행
  → 앞차 가까우면 v_safe 낮음 → 자연스럽게 감속
```

### braking_distance
```
brake_dist = vel² / (2 · d_max)

의미: 현재 속도에서 정지까지 필요한 거리
  → free_dist > brake_dist 이면 아직 감속 불필요
  → free_dist ≤ brake_dist 이면 지금 감속 시작해야 함
```

### 감속 시 정확한 정지
```
required_decel = vel² / (2 · free_dist)

의미: free_dist 끝에서 정확히 v=0이 되는 감속도
  → min(required_decel, d_max) 적용
  → 정지 위치 = pos + free_dist (= 앞차 - h_min)
```

## 이벤트 흐름 예시

### 정상 주행 (앞차 멀리 있음)
```
t=0    START → _plan() → 가속 시작 (acc=a_max)
t=0.7  RESUME → _plan() → v_max 도달 → cruise (acc=0)
t=3.2  RESUME → _plan() → braking point 도달 → 아직 여유 → cruise 유지
t=5.8  RESUME → _plan() → braking point 도달 → 아직 여유 → cruise 유지
...반복 (앞차와 같은 속도면 gap 유지, RESUME 간격 일정)
```

### 앞차 정지 시
```
t=10.0 RESUME → _plan() → free=5000, brake=1296 → cruise, RESUME@t=11.0
t=10.5 앞차 STOPPED → _notify_followers → 내 이벤트 invalidate → _plan() 즉시 호출
       → free=3200, brake=1296 → 아직 여유 → cruise, RESUME@t=10.9
t=10.9 RESUME → _plan() → free=1296, brake=1296 → 감속 시작
       → stop_pos 설정 (X 마커), RESUME@(50% 지점)
t=11.2 RESUME → _plan() → 앞차 여전히 정지 → 감속 계속, RESUME@(50% 지점)
t=11.4 RESUME → _plan() → vel≈0 → STOP
t=11.6 RESUME → _plan() → 앞차 아직 정지 → STOP 유지
```

### 앞차 재출발 시
```
t=15.0 앞차 RESUME → 가속 시작 → _notify_followers → 내 _plan() 호출
       → free 증가 → 가속 시작, X 마커 사라짐
```

## 파라미터

| 파라미터 | 값 | 의미 |
|---------|-----|------|
| `v_max` | 3600 mm/s | 최대 주행 속도 |
| `a_max` | 500 mm/s² | 최대 가속도 |
| `d_max` | 500 mm/s² | 최대 감속도 |
| `h_min` | 1150 mm | 최소 안전 거리 (차체 750 + 마진 400) |
| `length` | 750 mm | 차체 길이 |

## 이벤트 효율성

- 각 차량은 항상 **최대 1개의 pending event**를 가짐
- `_invalidate()`로 이전 이벤트 무효화 → 새 이벤트로 교체
- `_notify_followers()`는 상태 변화 시에만 호출
- 정상 주행 시: RESUME 간격 = coast 시간 (수 초)
- heap 크기 ≈ 차량 수 (일정)
