"""
mcs_unified.py — 단일 힙 기반 Material Control System (MCS)

모든 서브시스템(OHT, AGV, 3DS, Elevator)의 이벤트와 MCS 이벤트가
하나의 힙에서 시간순으로 처리되어 인과관계가 자동 보장된다.

설계 원칙
─────────
  1. 단일 힙: 모든 이벤트(주행, 작업, 엘리베이터)가 하나의 heapq에 존재
  2. 시스템 태그: 이벤트에 system 필드로 라우팅 (OHT/AGV/3DS/LIFT/MCS)
  3. 서브시스템 독립: 서로 다른 시스템의 Vehicle은 물리적 상호작용 없음
  4. MCS는 시스템 경계만 연결: DONE → LOADING → DWELL_DONE → DISPATCH → 주행 재개

이벤트 흐름 (OHT 기준 예시)
────────────────────────────
  [MCS] LOAD_CREATED  → port 대기큐 적재, 할당 시도
  [MCS] TRY_ASSIGN    → idle 차량에 Load 배정, DISPATCH to src
  [OHT] TRY_ADVANCE   → 세그먼트 진입 시도
  [OHT] SEGMENT_DONE  → 세그먼트 완료, 다음 진입
  [OHT] ...           → 경로 끝 도달: state=DONE
  ★ [MCS] VEHICLE_ARRIVED → LOADING dwell 스케줄
  [MCS] DWELL_DONE    → 적재 완료, DISPATCH to dst
  [OHT] TRY_ADVANCE   → ...
  [OHT] ...           → 목적지 도착: state=DONE
  ★ [MCS] VEHICLE_ARRIVED → UNLOADING dwell 스케줄
  [MCS] DWELL_DONE    → 하역 완료, Load 완료, 차량 idle, 재할당 시도
"""
from __future__ import annotations

import heapq
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Callable, Tuple, Any
from enum import Enum


# ═══════════════════════════════════════════════════════════════════════════════
# Event — 모든 서브시스템이 공유하는 단일 이벤트 구조
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass(order=True)
class Event:
    """단일 힙에서 정렬되는 이벤트.

    정렬 기준: (t, seq) — 같은 시각이면 삽입 순서(seq)로 FIFO 보장.
    """
    t:      float
    seq:    int
    system: str  = field(compare=False)   # 'OHT', 'AGV', '3DS_F1', 'LIFT', 'MCS'
    kind:   str  = field(compare=False)   # 이벤트 타입
    vid:    int  = field(compare=False, default=-1)   # vehicle/agent id
    token:  int  = field(compare=False, default=0)    # stale 판별
    data:   Any  = field(compare=False, default=None) # 부가 데이터


# ═══════════════════════════════════════════════════════════════════════════════
# MCS 이벤트 타입
# ═══════════════════════════════════════════════════════════════════════════════

LOAD_CREATED     = 'MCS_LOAD_CREATED'
TRY_ASSIGN       = 'MCS_TRY_ASSIGN'
VEHICLE_ARRIVED  = 'MCS_VEHICLE_ARRIVED'
DWELL_DONE       = 'MCS_DWELL_DONE'


# ═══════════════════════════════════════════════════════════════════════════════
# States
# ═══════════════════════════════════════════════════════════════════════════════

class LoadState(Enum):
    WAITING     = 'WAITING'
    ASSIGNED    = 'ASSIGNED'
    ON_VEHICLE  = 'ON_VEHICLE'
    DELIVERED   = 'DELIVERED'
    COMPLETED   = 'COMPLETED'


class VehicleJobState(Enum):
    IDLE        = 'IDLE'
    TO_PICKUP   = 'TO_PICKUP'
    LOADING     = 'LOADING'
    TO_DELIVERY = 'TO_DELIVERY'
    UNLOADING   = 'UNLOADING'


# ═══════════════════════════════════════════════════════════════════════════════
# Load / Port / VehicleBinding
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class Load:
    load_id:    int
    src_port:   str
    dst_port:   str
    system:     str
    priority:   int = 0
    state:      LoadState = LoadState.WAITING
    vehicle_id: Optional[int] = None

    t_created:     float = 0.0
    t_assigned:    float = 0.0
    t_pickup_arr:  float = 0.0
    t_pickup_end:  float = 0.0
    t_deliver_arr: float = 0.0
    t_completed:   float = 0.0

    def wait_time(self) -> float:
        if self.t_assigned > 0:
            return self.t_assigned - self.t_created
        return -1.0

    def cycle_time(self) -> float:
        if self.t_completed > 0:
            return self.t_completed - self.t_created
        return -1.0

    def travel_time(self) -> float:
        if self.t_completed > 0:
            dwell = (self.t_pickup_end - self.t_pickup_arr) + \
                    (self.t_completed - self.t_deliver_arr)
            return self.cycle_time() - self.wait_time() - dwell
        return -1.0


@dataclass
class Port:
    node_id:    str
    system:     str               # 소속 시스템 ('OHT', 'AGV', '3DS_F1', ...)
    prod_rate:  float = 0.0       # loads/min, 0이면 소비 전용
    dest_ports: List[str] = field(default_factory=list)
    waiting_loads: List[Load] = field(default_factory=list)


@dataclass
class VehicleBinding:
    vehicle_id: int
    system:     str               # 소속 시스템
    load:       Optional[Load] = None
    phase:      VehicleJobState = VehicleJobState.IDLE
    token:      int = 0


# ═══════════════════════════════════════════════════════════════════════════════
# KPI Tracker (기존과 동일)
# ═══════════════════════════════════════════════════════════════════════════════

class KPITracker:

    def __init__(self):
        self.completed_loads: List[Load] = []
        self._vehicle_busy: Dict[int, float] = {}
        self._vehicle_busy_start: Dict[int, float] = {}

    def record_complete(self, load: Load):
        self.completed_loads.append(load)

    def mark_busy(self, vehicle_id: int, t: float):
        if vehicle_id not in self._vehicle_busy_start:
            self._vehicle_busy_start[vehicle_id] = t

    def mark_idle(self, vehicle_id: int, t: float):
        start = self._vehicle_busy_start.pop(vehicle_id, None)
        if start is not None:
            self._vehicle_busy.setdefault(vehicle_id, 0.0)
            self._vehicle_busy[vehicle_id] += t - start

    def throughput(self, t_now: float) -> float:
        if t_now <= 0:
            return 0.0
        return len(self.completed_loads) / (t_now / 60.0)

    def avg_cycle_time(self, last_n: int = 0) -> float:
        loads = self.completed_loads[-last_n:] if last_n else self.completed_loads
        if not loads:
            return 0.0
        return sum(l.cycle_time() for l in loads) / len(loads)

    def avg_wait_time(self, last_n: int = 0) -> float:
        loads = self.completed_loads[-last_n:] if last_n else self.completed_loads
        if not loads:
            return 0.0
        return sum(l.wait_time() for l in loads) / len(loads)

    def utilization(self, vehicle_ids: List[int], t_now: float) -> float:
        if not vehicle_ids or t_now <= 0:
            return 0.0
        total = 0.0
        for vid in vehicle_ids:
            busy = self._vehicle_busy.get(vid, 0.0)
            if vid in self._vehicle_busy_start:
                busy += t_now - self._vehicle_busy_start[vid]
            total += busy / t_now
        return total / len(vehicle_ids)

    @property
    def total_completed(self) -> int:
        return len(self.completed_loads)


# ═══════════════════════════════════════════════════════════════════════════════
# MCS Engine — 단일 힙 참조 방식
# ═══════════════════════════════════════════════════════════════════════════════

class MCSEngine:
    """단일 힙 기반 MCS.

    외부에서 생성한 힙(list)을 참조로 받아서 사용한다.
    모든 서브시스템이 같은 힙 객체를 공유하므로
    이벤트 인과관계가 시간순으로 자동 보장된다.

    Parameters
    ----------
    heap : list
        공유 이벤트 힙 (heapq). 모든 서브시스템이 동일 객체 참조.
    seq_counter : list[int]
        공유 시퀀스 카운터 [0]. 이벤트 삽입 순서 보장용.
        리스트로 감싸서 mutable reference 역할.
    systems : dict[str, SystemConfig]
        시스템별 설정. key = 시스템 ID ('OHT', 'AGV', ...).
    dwell_time : float
        적재/하역 소요 시간 (초)
    seed : int or None
        재현성
    """

    _global_load_id = 0

    def __init__(self, heap: list, seq_counter: list,
                 dwell_time: float = 3.0,
                 seed: int | None = None):
        self.heap = heap                # 공유 힙
        self._seq = seq_counter         # 공유 시퀀스 [n]
        self.dwell_time = dwell_time
        self._rng = random.Random(seed)

        self.ports: Dict[str, Port] = {}
        self.bindings: Dict[int, VehicleBinding] = {}
        self.kpi = KPITracker()

        # 시스템별 콜백 등록
        # dispatch_cb[system](vid, goal_node, t) — 차량 이동 명령
        # vehicle_node_cb[system](vid) → str — 차량 현재 노드
        # distance_cb[system](src, dst) → float — 경로 거리
        self._dispatch_cb:     Dict[str, Callable] = {}
        self._vehicle_free_cb: Dict[str, Callable] = {}
        self._vehicle_node_cb: Dict[str, Callable] = {}
        self._distance_cb:     Dict[str, Callable] = {}

    # ── 시스템 등록 ──────────────────────────────────────────────────────────

    def register_system(self, system: str,
                        port_nodes: List[str],
                        on_dispatch: Callable[[int, str, float], None],
                        is_vehicle_free: Callable[[int], bool] | None = None,
                        get_vehicle_node: Callable[[int], Optional[str]] | None = None,
                        get_distance: Callable[[str, str], float] | None = None,
                        port_prod_rate: float = 0.5):
        """서브시스템 등록 및 포트 생성.

        각 시스템마다 별도 호출. OHT, AGV, 3DS_F1, ... 각각 등록.
        is_vehicle_free: transport layer에서 차량이 실제 유휴(DONE)인지 확인.
        """
        self._dispatch_cb[system] = on_dispatch
        if is_vehicle_free:
            self._vehicle_free_cb[system] = is_vehicle_free
        if get_vehicle_node:
            self._vehicle_node_cb[system] = get_vehicle_node
        if get_distance:
            self._distance_cb[system] = get_distance

        for nid in port_nodes:
            port_key = f'{system}:{nid}'
            self.ports[port_key] = Port(
                node_id=nid,
                system=system,
                prod_rate=port_prod_rate,
                dest_ports=[],
            )

    def start_production(self, t: float = 0.0):
        """모든 포트의 초기 LOAD_CREATED 이벤트 스케줄."""
        for port_key, port in self.ports.items():
            self._schedule_port_production(port_key, port, t)

    # ── 차량 등록 ────────────────────────────────────────────────────────────

    def register_vehicle(self, vehicle_id: int, system: str):
        if vehicle_id not in self.bindings:
            self.bindings[vehicle_id] = VehicleBinding(
                vehicle_id=vehicle_id, system=system)

    def unregister_vehicle(self, vehicle_id: int):
        self.bindings.pop(vehicle_id, None)

    # ── 이벤트 posting (공유 힙에 삽입) ──────────────────────────────────────

    def _post(self, t: float, kind: str, vid: int = -1,
              token: int = 0, data: Any = None):
        seq = self._seq[0]
        self._seq[0] += 1
        heapq.heappush(self.heap,
                       Event(t, seq, 'MCS', kind, vid, token, data))

    # ── 이벤트 핸들러 (외부 step 루프에서 호출) ──────────────────────────────

    def handle_event(self, ev: Event):
        """MCS 이벤트 처리. system=='MCS'인 이벤트만 여기로 라우팅."""
        if ev.kind == LOAD_CREATED:
            self._on_load_created(ev.t, ev.data)
        elif ev.kind == TRY_ASSIGN:
            self._do_assign(ev.t)
        elif ev.kind == VEHICLE_ARRIVED:
            self._on_vehicle_arrived(ev.t, ev.vid, ev.token)
        elif ev.kind == DWELL_DONE:
            self._on_dwell_done(ev.t, ev.vid, ev.token)

    def handle_all(self, t_now: float):
        """t_now까지의 모든 MCS 이벤트를 힙에서 꺼내 처리."""
        while self.heap and self.heap[0].t <= t_now:
            ev = heapq.heappop(self.heap)
            self.handle_event(ev)

    # ── Load 생성 ────────────────────────────────────────────────────────────

    def _schedule_port_production(self, port_key: str, port: Port, t: float):
        if port.prod_rate <= 0:
            return
        lam = port.prod_rate / 60.0
        interval = self._rng.expovariate(lam)
        self._post(t + interval, LOAD_CREATED,
                   data={'port_key': port_key})

    def _on_load_created(self, t: float, data: dict):
        port_key = data['port_key']
        port = self.ports.get(port_key)
        if port is None:
            return

        # 같은 시스템 내 다른 포트 중 목적지 결정
        if port.dest_ports:
            candidates = port.dest_ports
        else:
            candidates = [pk for pk, p in self.ports.items()
                          if p.system == port.system and pk != port_key]
        if not candidates:
            self._schedule_port_production(port_key, port, t)
            return

        dst_key = self._rng.choice(candidates)
        dst_port = self.ports[dst_key]

        MCSEngine._global_load_id += 1
        load = Load(
            load_id=MCSEngine._global_load_id,
            src_port=port.node_id,
            dst_port=dst_port.node_id,
            system=port.system,
            t_created=t,
        )

        port.waiting_loads.append(load)
        self._do_assign(t)
        self._schedule_port_production(port_key, port, t)

    # ── 할당 ─────────────────────────────────────────────────────────────────

    def _do_assign(self, t: float):
        """시스템별로 대기 Load를 idle 차량에 할당.

        시스템이 다른 차량-Load 간에는 할당하지 않는다.
        """
        # 시스템별 idle 차량 그룹핑 (transport layer에서도 free인지 확인)
        idle_by_sys: Dict[str, List[int]] = {}
        for vid, b in self.bindings.items():
            if b.phase == VehicleJobState.IDLE:
                free_cb = self._vehicle_free_cb.get(b.system)
                if free_cb and not free_cb(vid):
                    continue  # DES에서 아직 주행 중 → 할당 불가
                idle_by_sys.setdefault(b.system, []).append(vid)

        for system, idle_vids in idle_by_sys.items():
            if not idle_vids:
                continue

            # 해당 시스템의 대기 Load 수집 (Oldest first)
            waiting: List[Load] = []
            for pk, port in self.ports.items():
                if port.system != system:
                    continue
                for load in port.waiting_loads:
                    if load.state == LoadState.WAITING:
                        waiting.append(load)
            if not waiting:
                continue
            waiting.sort(key=lambda l: l.t_created)

            # idle 차량 현재 노드 캐시
            vehicle_nodes: Dict[int, str] = {}
            get_node = self._vehicle_node_cb.get(system)
            if get_node:
                for vid in idle_vids:
                    node = get_node(vid)
                    if node:
                        vehicle_nodes[vid] = node

            used: Set[int] = set()
            dispatch = self._dispatch_cb.get(system)
            if dispatch is None:
                continue

            for load in waiting:
                available = [v for v in idle_vids if v not in used]
                if not available:
                    break

                chosen = self._select_nearest(
                    available, vehicle_nodes, load.src_port, system)

                load.state = LoadState.ASSIGNED
                load.vehicle_id = chosen
                load.t_assigned = t

                b = self.bindings[chosen]
                b.load = load
                b.phase = VehicleJobState.TO_PICKUP
                b.token += 1

                self.kpi.mark_busy(chosen, t)
                used.add(chosen)

                dispatch(chosen, load.src_port, t)

    def _select_nearest(self, candidates: List[int],
                        vehicle_nodes: Dict[int, str],
                        target_node: str,
                        system: str) -> int:
        get_dist = self._distance_cb.get(system)
        if not get_dist or not vehicle_nodes:
            return candidates[0]

        best_vid = candidates[0]
        best_dist = float('inf')
        for vid in candidates:
            v_node = vehicle_nodes.get(vid)
            if v_node is None:
                continue
            if v_node == target_node:
                return vid
            dist = get_dist(v_node, target_node)
            if dist < best_dist:
                best_dist = dist
                best_vid = vid
        return best_vid

    # ── 차량 도착 (서브시스템이 직접 힙에 삽입) ────────────────────────────────

    def get_binding_token(self, vehicle_id: int) -> int:
        """서브시스템이 VEHICLE_ARRIVED 이벤트를 힙에 넣을 때 사용할 token."""
        b = self.bindings.get(vehicle_id)
        return b.token if b else 0

    def _on_vehicle_arrived(self, t: float, vid: int, token: int):
        b = self.bindings.get(vid)
        if b is None or b.load is None or b.token != token:
            return

        load = b.load

        if b.phase == VehicleJobState.TO_PICKUP:
            # src 도착 → LOADING dwell
            load.state = LoadState.ON_VEHICLE
            load.t_pickup_arr = t
            b.phase = VehicleJobState.LOADING
            b.token += 1

            # 포트 대기큐에서 제거
            port_key = f'{b.system}:{load.src_port}'
            port = self.ports.get(port_key)
            if port and load in port.waiting_loads:
                port.waiting_loads.remove(load)

            self._post(t + self.dwell_time, DWELL_DONE,
                       vid=vid, token=b.token)

        elif b.phase == VehicleJobState.TO_DELIVERY:
            # dst 도착 → UNLOADING dwell
            load.state = LoadState.DELIVERED
            load.t_deliver_arr = t
            b.phase = VehicleJobState.UNLOADING
            b.token += 1

            self._post(t + self.dwell_time, DWELL_DONE,
                       vid=vid, token=b.token)

    # ── Dwell 완료 ───────────────────────────────────────────────────────────

    def _on_dwell_done(self, t: float, vid: int, token: int):
        b = self.bindings.get(vid)
        if b is None or b.load is None or b.token != token:
            return

        load = b.load
        dispatch = self._dispatch_cb.get(b.system)

        if b.phase == VehicleJobState.LOADING:
            # 적재 완료 → dst로 출발
            load.t_pickup_end = t
            b.phase = VehicleJobState.TO_DELIVERY
            b.token += 1
            if dispatch:
                dispatch(vid, load.dst_port, t)

        elif b.phase == VehicleJobState.UNLOADING:
            # 하역 완료 → Load 소비, 차량 idle
            load.state = LoadState.COMPLETED
            load.t_completed = t
            self.kpi.record_complete(load)
            self.kpi.mark_idle(vid, t)
            b.load = None
            b.phase = VehicleJobState.IDLE
            b.token += 1

            # 즉시 재할당 시도
            self._post(t, TRY_ASSIGN)

    # ── 포트 설정 ────────────────────────────────────────────────────────────

    def set_port_rate(self, system: str, node_id: str, rate: float):
        port_key = f'{system}:{node_id}'
        port = self.ports.get(port_key)
        if port:
            port.prod_rate = rate

    def set_port_destinations(self, system: str, node_id: str,
                              dest_nodes: List[str]):
        port_key = f'{system}:{node_id}'
        port = self.ports.get(port_key)
        if port:
            port.dest_ports = [f'{system}:{n}' for n in dest_nodes]

    # ── 조회 ─────────────────────────────────────────────────────────────────

    def get_phase(self, vehicle_id: int) -> VehicleJobState:
        b = self.bindings.get(vehicle_id)
        return b.phase if b else VehicleJobState.IDLE

    def get_load(self, vehicle_id: int) -> Optional[Load]:
        b = self.bindings.get(vehicle_id)
        return b.load if b else None

    @property
    def total_waiting(self) -> int:
        return sum(len([l for l in p.waiting_loads
                        if l.state == LoadState.WAITING])
                   for p in self.ports.values())

    @property
    def active_count(self) -> int:
        return sum(1 for b in self.bindings.values()
                   if b.phase != VehicleJobState.IDLE)

    def stats_summary(self, t: float, system: str | None = None) -> dict:
        """전체 또는 특정 시스템의 KPI 요약."""
        if system:
            vids = [vid for vid, b in self.bindings.items()
                    if b.system == system]
        else:
            vids = list(self.bindings.keys())
        return {
            'system': system or 'ALL',
            'waiting': self.total_waiting,
            'active': self.active_count,
            'completed': self.kpi.total_completed,
            'throughput': self.kpi.throughput(t),
            'avg_cycle': self.kpi.avg_cycle_time(last_n=20),
            'avg_wait': self.kpi.avg_wait_time(last_n=20),
            'utilization': self.kpi.utilization(vids, t),
        }

    def port_summary(self, system: str | None = None) -> List[dict]:
        result = []
        for pk, port in self.ports.items():
            if system and port.system != system:
                continue
            n_wait = len([l for l in port.waiting_loads
                          if l.state == LoadState.WAITING])
            if n_wait > 0:
                result.append({'port': pk, 'waiting': n_wait})
        result.sort(key=lambda x: -x['waiting'])
        return result


# ═══════════════════════════════════════════════════════════════════════════════
# 서브시스템용 헬퍼 — DONE 시 공유 힙에 MCS 이벤트 직접 삽입
# ═══════════════════════════════════════════════════════════════════════════════

def post_vehicle_arrived(heap: list, seq_counter: list,
                         vid: int, token: int, t: float):
    """서브시스템(OHT/AGV/3DS)이 차량 DONE 감지 시 호출.

    사용 예 (OHT _on_try_advance 내부):
        if nxt_idx >= len(path):
            agent.state = DONE
            post_vehicle_arrived(self.heap, self._seq, agent.id, binding_token, t)
    """
    seq = seq_counter[0]
    seq_counter[0] += 1
    heapq.heappush(heap,
                   Event(t, seq, 'MCS', VEHICLE_ARRIVED, vid, token))


# ═══════════════════════════════════════════════════════════════════════════════
# 통합 step 함수 — 메인 루프에서 호출
# ═══════════════════════════════════════════════════════════════════════════════

def unified_step(t_now: float, heap: list,
                 handlers: Dict[str, Callable[[Event], None]]):
    """단일 힙에서 t_now까지의 모든 이벤트를 처리.

    Parameters
    ----------
    t_now : float
        현재 시뮬레이션 시각
    heap : list
        공유 이벤트 힙
    handlers : dict
        system 태그 → 핸들러 함수 매핑.
        예: {'OHT': oht_env.handle_event,
             'AGV': agv_env.handle_event,
             'MCS': mcs.handle_event,
             'LIFT': lift_ctrl.handle_event, ...}
    """
    while heap and heap[0].t <= t_now:
        ev = heapq.heappop(heap)
        handler = handlers.get(ev.system)
        if handler:
            handler(ev)
