"""
mcs.py — Material Control System (상위 관제 시스템)  Phase 1

순수 DES 기반, Load-driven 작업 생성 / 할당 / 추적 / KPI 기록.

핵심 개념 (Automod 참조)
────────────────────────
  Load         포트에 물리적으로 존재하는 화물. src_port에서 생성되어 dst_port로 이동.
  Port         Load가 생성(produce)되거나 소비(consume)되는 위치.
               각 포트는 독립적인 생성률을 가짐.
  Job          Load의 수송 작업. Load가 포트에 생성되면 자동으로 Job 발행.
  Vehicle      Job을 수행하는 운반체. idle이면 대기 중인 Load에 할당됨.

이벤트
──────
  LOAD_CREATED  포트에서 Load 생성 → 대기큐 적재 → 할당 시도
  DWELL_DONE    적재/하역 완료 → 다음 단계 전이
  TRY_ASSIGN    유휴 차량 발생 시 대기 Load에 할당 시도

작업 흐름
────────
  Port produce → Load 생성 (포트 대기큐에 적재)
    → idle 차량 있으면 즉시 할당 (dispatch to src)
    → 차량 src 도착 → LOADING dwell
    → dwell 완료 → dispatch to dst
    → 차량 dst 도착 → UNLOADING dwell
    → dwell 완료 → Load 소비, KPI 기록, 차량 idle
    → 다른 대기 Load 있으면 즉시 재할당
"""
from __future__ import annotations

import heapq
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Callable, Tuple
from collections import deque
from enum import Enum


# ── Event types ──────────────────────────────────────────────────────────────

LOAD_CREATED = 'LOAD_CREATED'
DWELL_DONE   = 'DWELL_DONE'
TRY_ASSIGN   = 'TRY_ASSIGN'


# ── States ───────────────────────────────────────────────────────────────────

class LoadState(Enum):
    WAITING     = 'WAITING'       # 포트에서 pickup 대기
    ASSIGNED    = 'ASSIGNED'      # 차량 배정됨, 차량이 오는 중
    ON_VEHICLE  = 'ON_VEHICLE'    # 차량에 적재됨
    DELIVERED   = 'DELIVERED'     # 목적지 도착, 하역 중
    COMPLETED   = 'COMPLETED'     # 완료


class VehicleJobState(Enum):
    IDLE        = 'IDLE'
    TO_PICKUP   = 'TO_PICKUP'
    LOADING     = 'LOADING'
    TO_DELIVERY = 'TO_DELIVERY'
    UNLOADING   = 'UNLOADING'


# ── Load ─────────────────────────────────────────────────────────────────────

@dataclass
class Load:
    """포트에서 생성된 화물."""
    load_id:    int
    src_port:   str
    dst_port:   str
    system:     str
    priority:   int = 0
    state:      LoadState = LoadState.WAITING
    vehicle_id: Optional[int] = None

    # timestamps
    t_created:     float = 0.0
    t_assigned:    float = 0.0
    t_pickup_arr:  float = 0.0
    t_pickup_end:  float = 0.0
    t_deliver_arr: float = 0.0
    t_completed:   float = 0.0

    def wait_time(self) -> float:
        """대기 시간 (생성 → 차량 할당)."""
        if self.t_assigned > 0:
            return self.t_assigned - self.t_created
        return -1.0

    def cycle_time(self) -> float:
        """전체 사이클 (생성 → 완료)."""
        if self.t_completed > 0:
            return self.t_completed - self.t_created
        return -1.0

    def travel_time(self) -> float:
        """순수 이동 시간."""
        if self.t_completed > 0:
            dwell = (self.t_pickup_end - self.t_pickup_arr) + \
                    (self.t_completed - self.t_deliver_arr)
            return self.cycle_time() - self.wait_time() - dwell
        return -1.0


# ── Port ─────────────────────────────────────────────────────────────────────

@dataclass
class Port:
    """Load 생성/소비 위치.

    Parameters
    ----------
    node_id : str       맵 노드 ID
    prod_rate : float   생성률 (loads/min). 0이면 생성 안 함 (소비 전용).
    dest_ports : list    이 포트에서 생성된 Load의 가능한 목적지 노드 목록.
                         비어있으면 시스템 내 다른 모든 포트가 목적지 후보.
    """
    node_id:    str
    prod_rate:  float = 0.0
    dest_ports: List[str] = field(default_factory=list)

    # runtime
    waiting_loads: List[Load] = field(default_factory=list)


# ── Vehicle binding ──────────────────────────────────────────────────────────

@dataclass
class VehicleBinding:
    vehicle_id: int
    load: Optional[Load] = None
    phase: VehicleJobState = VehicleJobState.IDLE
    token: int = 0


# ── KPI Tracker ──────────────────────────────────────────────────────────────

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


# ── MCS Engine (DES) ────────────────────────────────────────────────────────

class MCSEngine:
    """
    단일 운송 시스템의 DES 기반 관제 엔진.  (Phase 1: Load-driven)

    Parameters
    ----------
    system : str
        시스템 식별자
    port_nodes : list[str]
        포트 노드 ID 리스트
    on_dispatch : callable(vehicle_id, goal_node, t) -> None
        차량 이동 명령 콜백
    is_vehicle_free : callable(vehicle_id) -> bool
        transport layer에서 차량이 실제 유휴인지 확인
    port_prod_rate : float
        포트당 기본 Load 생성률 (loads/min). 개별 설정 가능.
    dwell_time : float
        적재/하역 소요 시간 (초)
    seed : int or None
        재현성
    """

    _global_load_id = 0

    def __init__(self, system: str, port_nodes: List[str],
                 on_dispatch: Callable[[int, str, float], None],
                 is_vehicle_free: Callable[[int], bool] | None = None,
                 get_vehicle_node: Callable[[int], Optional[str]] | None = None,
                 get_distance: Callable[[str, str], float] | None = None,
                 port_prod_rate: float = 0.5,
                 dwell_time: float = 3.0,
                 seed: int | None = None):
        self.system = system
        self.on_dispatch = on_dispatch
        self.is_vehicle_free = is_vehicle_free
        self.get_vehicle_node = get_vehicle_node  # vid → 현재 노드
        self.get_distance = get_distance            # (from_node, to_node) → 경로 거리
        self.dwell_time = dwell_time
        self._rng = random.Random(seed)

        self._heap: List[tuple] = []
        self.bindings: Dict[int, VehicleBinding] = {}
        self.kpi = KPITracker()

        # 포트 생성 — 모든 포트가 생산+소비 겸용 (Phase 1 기본)
        self.ports: Dict[str, Port] = {}
        for nid in port_nodes:
            self.ports[nid] = Port(
                node_id=nid,
                prod_rate=port_prod_rate,
                dest_ports=[],   # 빈 리스트 = 다른 모든 포트가 후보
            )

        # 각 포트의 최초 LOAD_CREATED 이벤트 스케줄
        for port in self.ports.values():
            self._schedule_port_production(port, 0.0)

    # ── Port config ──────────────────────────────────────────────────────

    def set_port_rate(self, node_id: str, rate: float):
        """특정 포트의 생성률 변경."""
        port = self.ports.get(node_id)
        if port:
            port.prod_rate = rate

    def set_port_destinations(self, node_id: str, dest_nodes: List[str]):
        """특정 포트의 목적지 리스트 설정."""
        port = self.ports.get(node_id)
        if port:
            port.dest_ports = list(dest_nodes)

    # ── Vehicle management ───────────────────────────────────────────────

    def register_vehicle(self, vehicle_id: int):
        if vehicle_id not in self.bindings:
            self.bindings[vehicle_id] = VehicleBinding(vehicle_id=vehicle_id)

    def unregister_vehicle(self, vehicle_id: int):
        self.bindings.pop(vehicle_id, None)

    def request_assign(self, t: float):
        """외부: 유휴 차량 발생 시 호출."""
        self._post(t, TRY_ASSIGN)

    # ── DES core ─────────────────────────────────────────────────────────

    def step(self, t: float):
        """시각 t까지의 모든 이벤트를 처리."""
        while self._heap and self._heap[0][0] <= t:
            ev = heapq.heappop(self._heap)
            self._handle_event(ev[0], ev[1], ev[2], ev[3], ev[4])

    def _post(self, t: float, ev_type: str, vid: int = -1,
              token: int = 0, data: dict | None = None):
        heapq.heappush(self._heap, (t, ev_type, vid, token, data or {}))

    def _handle_event(self, t: float, ev_type: str, vid: int,
                      token: int, data: dict):
        if ev_type == LOAD_CREATED:
            self._on_load_created(t, data)
        elif ev_type == DWELL_DONE:
            b = self.bindings.get(vid)
            if b is None or b.token != token:
                return  # stale
            self._on_dwell_done(t, b)
        elif ev_type == TRY_ASSIGN:
            self._do_assign(t)

    # ── Load production ──────────────────────────────────────────────────

    def _schedule_port_production(self, port: Port, t: float):
        """포트의 다음 Load 생성 이벤트를 스케줄."""
        if port.prod_rate <= 0:
            return
        lam = port.prod_rate / 60.0   # per second
        interval = self._rng.expovariate(lam)
        self._post(t + interval, LOAD_CREATED,
                   data={'port': port.node_id})

    def _on_load_created(self, t: float, data: dict):
        """포트에서 Load 생성."""
        port_id = data['port']
        port = self.ports.get(port_id)
        if port is None:
            return

        # 목적지 결정
        candidates = port.dest_ports if port.dest_ports else \
                     [p for p in self.ports if p != port_id]
        if not candidates:
            self._schedule_port_production(port, t)
            return

        dst = self._rng.choice(candidates)

        MCSEngine._global_load_id += 1
        load = Load(
            load_id=MCSEngine._global_load_id,
            src_port=port_id,
            dst_port=dst,
            system=self.system,
            t_created=t,
        )

        # 포트의 대기큐에 적재 — Load가 물리적으로 이 포트에 존재
        port.waiting_loads.append(load)

        # 즉시 할당 시도 (Load Activation: load가 차량을 wake)
        self._do_assign(t)

        # 다음 생성 스케줄
        self._schedule_port_production(port, t)

    # ── Assignment ───────────────────────────────────────────────────────

    def _do_assign(self, t: float):
        """대기 중인 Load를 유휴 차량에 할당.

        전략: 대기 시간이 가장 긴 Load 우선 (Oldest),
              해당 Load의 포트까지 layout 최단경로가 가장 짧은 idle 차량 할당.
              get_distance 미설정 시 첫 idle 차량 할당 (fallback).
        """
        check = self.is_vehicle_free
        idle = [vid for vid, b in self.bindings.items()
                if b.phase == VehicleJobState.IDLE
                and (check is None or check(vid))]
        if not idle:
            return

        # idle 차량의 현재 노드 캐시
        vehicle_nodes: Dict[int, str] = {}
        if self.get_vehicle_node:
            for vid in idle:
                node = self.get_vehicle_node(vid)
                if node:
                    vehicle_nodes[vid] = node

        # 모든 포트에서 대기 중인 WAITING Load를 생성 시각순으로 수집
        waiting_loads: List[Load] = []
        for port in self.ports.values():
            for load in port.waiting_loads:
                if load.state == LoadState.WAITING:
                    waiting_loads.append(load)
        if not waiting_loads:
            return

        # Oldest first
        waiting_loads.sort(key=lambda l: l.t_created)

        used_vehicles: Set[int] = set()

        for load in waiting_loads:
            available = [v for v in idle if v not in used_vehicles]
            if not available:
                break

            # 차량 선택: layout 최단경로 기준 nearest
            chosen = self._select_nearest_vehicle(
                available, vehicle_nodes, load.src_port)

            # 할당
            load.state = LoadState.ASSIGNED
            load.vehicle_id = chosen
            load.t_assigned = t

            b = self.bindings[chosen]
            b.load = load
            b.phase = VehicleJobState.TO_PICKUP
            b.token += 1

            self.kpi.mark_busy(chosen, t)
            used_vehicles.add(chosen)

            self.on_dispatch(chosen, load.src_port, t)

    def _select_nearest_vehicle(self, candidates: List[int],
                                vehicle_nodes: Dict[int, str],
                                target_node: str) -> int:
        """target_node까지 layout 거리가 가장 짧은 차량 선택."""
        if not self.get_distance or not vehicle_nodes:
            return candidates[0]   # fallback: 첫 차량

        best_vid = candidates[0]
        best_dist = float('inf')

        for vid in candidates:
            v_node = vehicle_nodes.get(vid)
            if v_node is None:
                continue
            if v_node == target_node:
                return vid  # 이미 해당 포트에 있음 — 즉시 선택
            dist = self.get_distance(v_node, target_node)
            if dist < best_dist:
                best_dist = dist
                best_vid = vid

        return best_vid

    # ── Vehicle arrived (외부 호출) ──────────────────────────────────────

    def notify_arrived(self, vehicle_id: int, t: float):
        """차량이 목적지에 도착했을 때 외부에서 호출."""
        b = self.bindings.get(vehicle_id)
        if b is None or b.load is None:
            return

        load = b.load

        if b.phase == VehicleJobState.TO_PICKUP:
            # src 도착 → Load 적재 시작 (LOADING dwell)
            load.state = LoadState.ON_VEHICLE
            load.t_pickup_arr = t
            b.phase = VehicleJobState.LOADING
            b.token += 1

            # 포트 대기큐에서 제거
            port = self.ports.get(load.src_port)
            if port and load in port.waiting_loads:
                port.waiting_loads.remove(load)

            self._post(t + self.dwell_time, DWELL_DONE,
                       vid=vehicle_id, token=b.token)

        elif b.phase == VehicleJobState.TO_DELIVERY:
            # dst 도착 → 하역 시작 (UNLOADING dwell)
            load.state = LoadState.DELIVERED
            load.t_deliver_arr = t
            b.phase = VehicleJobState.UNLOADING
            b.token += 1

            self._post(t + self.dwell_time, DWELL_DONE,
                       vid=vehicle_id, token=b.token)

    # ── Dwell done ───────────────────────────────────────────────────────

    def _on_dwell_done(self, t: float, b: VehicleBinding):
        load = b.load
        if load is None:
            return

        if b.phase == VehicleJobState.LOADING:
            # 적재 완료 → dst로 출발
            load.t_pickup_end = t
            b.phase = VehicleJobState.TO_DELIVERY
            b.token += 1
            self.on_dispatch(b.vehicle_id, load.dst_port, t)

        elif b.phase == VehicleJobState.UNLOADING:
            # 하역 완료 → Load 소비, 차량 idle
            load.state = LoadState.COMPLETED
            load.t_completed = t
            self.kpi.record_complete(load)
            self.kpi.mark_idle(b.vehicle_id, t)
            b.load = None
            b.phase = VehicleJobState.IDLE
            b.token += 1

            # 차량 idle → 즉시 재할당 시도
            self._post(t, TRY_ASSIGN)

    # ── Query ────────────────────────────────────────────────────────────

    def get_phase(self, vehicle_id: int) -> VehicleJobState:
        b = self.bindings.get(vehicle_id)
        return b.phase if b else VehicleJobState.IDLE

    def get_load(self, vehicle_id: int) -> Optional[Load]:
        b = self.bindings.get(vehicle_id)
        return b.load if b else None

    @property
    def total_waiting(self) -> int:
        """모든 포트의 대기 Load 수."""
        return sum(len([l for l in p.waiting_loads if l.state == LoadState.WAITING])
                   for p in self.ports.values())

    @property
    def active_count(self) -> int:
        return sum(1 for b in self.bindings.values()
                   if b.phase != VehicleJobState.IDLE)

    def stats_summary(self, t: float) -> dict:
        vids = list(self.bindings.keys())
        return {
            'system': self.system,
            'waiting': self.total_waiting,
            'active': self.active_count,
            'completed': self.kpi.total_completed,
            'throughput': self.kpi.throughput(t),
            'avg_cycle': self.kpi.avg_cycle_time(last_n=20),
            'avg_wait': self.kpi.avg_wait_time(last_n=20),
            'utilization': self.kpi.utilization(vids, t),
        }

    def port_summary(self) -> List[dict]:
        """포트별 대기 Load 수."""
        result = []
        for pid, port in self.ports.items():
            n_wait = len([l for l in port.waiting_loads
                          if l.state == LoadState.WAITING])
            if n_wait > 0:
                result.append({'port': pid, 'waiting': n_wait})
        result.sort(key=lambda x: -x['waiting'])
        return result
