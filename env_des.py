"""
env_des.py — Discrete Event Simulation environment for AGV path execution.

실제 위치를 연속으로 추적하지 않고 '의미 있는 사건(노드 도착/출발)'만 처리합니다.

이벤트 종류
───────────
  TRY_DEPART  — 에이전트가 다음 노드로 출발을 시도
  ARRIVE      — 에이전트가 목적 노드에 도착

충돌 방지
─────────
  출발 전 move_state + 도착 stop_state의 affect_state 전체를 예약 시도합니다.
  하나라도 다른 에이전트가 점유 중이면 WAITING 상태로 대기합니다.
  점유 해제 시 대기 중인 에이전트를 깨워 다시 출발을 시도합니다.

시각화 연동
───────────
  DESAgent.x, y, theta 는 항상 최신 추정 위치를 유지합니다.
  MOVING 상태에서는 from → to 선형 보간, 나머지는 현재 노드 위치에 고정됩니다.
"""
from __future__ import annotations
import math
import heapq
from dataclasses import dataclass, field
from typing import List, Dict, Optional

from pkl_loader import PklMapGraph

# ── 에이전트 상태 상수 ─────────────────────────────────────────────────────────
IDLE    = 'idle'
MOVING  = 'moving'
WAITING = 'waiting'
DONE    = 'done'


# ── 이벤트 ────────────────────────────────────────────────────────────────────

@dataclass(order=True)
class Event:
    time:     float
    seq:      int
    kind:     str  = field(compare=False)
    agent_id: int  = field(compare=False)
    data:     dict = field(compare=False, default_factory=dict)


# ── DES 에이전트 ───────────────────────────────────────────────────────────────

class DESAgent:
    """
    이벤트 기반 에이전트.
    kinematic 모델 없이 edge 이동 시간 = length / max_speed 로 고정합니다.
    """
    def __init__(self, agent_id: int, color: tuple, node_path: List[str]):
        self.id        = agent_id
        self.color     = color
        self.node_path = node_path
        self.path_idx  = 0
        self.state     = IDLE

        # 시각적 위치 (시뮬레이터가 매 프레임 읽음)
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.v     = 0.0   # 표시용 속도 (실제 물리량 아님)

        # MOVING 보간용
        self.from_x:      float = 0.0
        self.from_y:      float = 0.0
        self.to_x:        float = 0.0
        self.to_y:        float = 0.0
        self.depart_time: float = 0.0
        self.arrive_time: float = 1.0

        # 현재 보유한 예약 state_id 목록
        self.reserved: List[str] = []

    @property
    def cur_node(self) -> str:
        if self.path_idx < len(self.node_path):
            return self.node_path[self.path_idx]
        return self.node_path[-1]


# ── DES 환경 ──────────────────────────────────────────────────────────────────

class DESEnvironment:
    """
    이벤트 큐 + 예약 테이블 기반 DES 실행 환경.

    step(sim_time) 을 매 프레임 호출하면 현재 시간까지의 이벤트를 처리하고
    에이전트의 시각적 위치를 업데이트합니다.
    """

    def __init__(self, graph: PklMapGraph):
        self.graph        = graph
        self._eq: list    = []          # heapq: Event
        self._seq         = 0
        self.agents:       Dict[int, DESAgent] = {}
        self.reservations: Dict[str, int]      = {}  # state_id → agent_id
        self.wait_queues:  Dict[str, List[int]] = {}  # state_id → [agent_id]
        self.sim_time      = 0.0

    # ── 에이전트 등록 ──────────────────────────────────────────────────────────

    def add_agent(self, agent: DESAgent, t_start: float = 0.0):
        """에이전트를 등록하고 첫 번째 출발 이벤트를 예약합니다."""
        self.agents[agent.id] = agent

        if not agent.node_path:
            agent.state = DONE
            return

        # 시작 노드 위치 설정
        node = self.graph.nodes.get(agent.node_path[0])
        if node is None:
            agent.state = DONE
            return

        agent.x = node.x
        agent.y = node.y

        # 초기 heading
        if len(agent.node_path) >= 2:
            edge = self.graph.get_edge(agent.node_path[0], agent.node_path[1])
            agent.theta = edge.angle if edge else 0.0

        # 시작 stop_state 예약
        stop_id = self._stop_id_for(agent.node_path[0], agent.theta)
        if stop_id:
            self._reserve(stop_id, agent.id)
            agent.reserved = [stop_id]

        self._schedule(t_start, 'TRY_DEPART', agent.id)

    def remove_agent(self, agent_id: int):
        """에이전트를 제거하고 모든 예약을 해제합니다."""
        agent = self.agents.pop(agent_id, None)
        if agent:
            for s in agent.reserved:
                self._do_release(s, agent_id)

    # ── 메인 루프 ──────────────────────────────────────────────────────────────

    def step(self, sim_time: float):
        """sim_time까지의 이벤트를 처리하고 시각적 위치를 보간합니다."""
        self.sim_time = sim_time
        while self._eq and self._eq[0].time <= sim_time:
            ev = heapq.heappop(self._eq)
            self._process(ev)
        self._update_visuals(sim_time)

    # ── 이벤트 처리 ────────────────────────────────────────────────────────────

    def _process(self, ev: Event):
        handler = {'TRY_DEPART': self._on_try_depart,
                   'ARRIVE':     self._on_arrive}.get(ev.kind)
        if handler:
            handler(ev)

    def _on_try_depart(self, ev: Event):
        agent = self.agents.get(ev.agent_id)
        if agent is None or agent.state in (MOVING, DONE):
            return

        idx = agent.path_idx
        if idx >= len(agent.node_path) - 1:
            agent.state = DONE
            agent.v     = 0.0
            return

        from_n  = agent.node_path[idx]
        to_n    = agent.node_path[idx + 1]
        edge    = self.graph.get_edge(from_n, to_n)
        if edge is None:
            agent.state = DONE
            return

        move_id    = f"M,{from_n},{to_n}"
        to_stop_id = self._stop_id_for(to_n, edge.angle)

        # 예약이 필요한 state 집합
        needed = self._collect_needed(move_id, to_stop_id)

        # 다른 에이전트가 점유 중인 state 확인
        blocking = {s for s in needed
                    if self.reservations.get(s, agent.id) != agent.id}

        if not blocking:
            # ── 출발 ──────────────────────────────────────────────────────
            # 현재 stop_state 해제
            for s in list(agent.reserved):
                self._do_release(s, agent.id)
            agent.reserved = []

            # 새 state 예약
            for s in needed:
                self._reserve(s, agent.id)
            agent.reserved = list(needed)

            travel_time   = edge.length / edge.max_speed
            arrive_t      = ev.time + travel_time
            from_node     = self.graph.nodes[from_n]
            to_node       = self.graph.nodes[to_n]

            agent.from_x      = from_node.x
            agent.from_y      = from_node.y
            agent.to_x        = to_node.x
            agent.to_y        = to_node.y
            agent.theta       = edge.angle
            agent.depart_time = ev.time
            agent.arrive_time = arrive_t
            agent.state       = MOVING
            agent.v           = edge.max_speed

            self._schedule(arrive_t, 'ARRIVE', agent.id,
                           node=to_n, stop_id=to_stop_id)
        else:
            # ── 대기 ──────────────────────────────────────────────────────
            agent.state = WAITING
            agent.v     = 0.0
            for s in blocking:
                q = self.wait_queues.setdefault(s, [])
                if agent.id not in q:
                    q.append(agent.id)

    def _on_arrive(self, ev: Event):
        agent = self.agents.get(ev.agent_id)
        if agent is None:
            return

        node_id  = ev.data['node']
        stop_id  = ev.data['stop_id']
        node     = self.graph.nodes.get(node_id)
        if node:
            agent.x = node.x
            agent.y = node.y

        agent.path_idx += 1
        agent.state     = IDLE
        agent.v         = 0.0

        # move_state + affect_states 해제, stop_state는 유지
        keep = {stop_id} if stop_id else set()
        for s in list(agent.reserved):
            if s not in keep:
                self._do_release(s, agent.id)
        agent.reserved = list(keep)

        # 다음 출발 시도 (즉시)
        self._schedule(ev.time, 'TRY_DEPART', agent.id)

    # ── 예약 관리 ──────────────────────────────────────────────────────────────

    def _reserve(self, state_id: str, agent_id: int):
        self.reservations[state_id] = agent_id

    def _do_release(self, state_id: str, agent_id: int):
        if self.reservations.get(state_id) == agent_id:
            del self.reservations[state_id]
            # 대기 중인 에이전트 깨우기
            waiting = self.wait_queues.pop(state_id, [])
            for wid in waiting:
                wagent = self.agents.get(wid)
                if wagent and wagent.state == WAITING:
                    wagent.state = IDLE
                    self._schedule(self.sim_time + 1e-9,
                                   'TRY_DEPART', wid)

    # ── 시각 보간 ──────────────────────────────────────────────────────────────

    def _update_visuals(self, sim_time: float):
        for agent in self.agents.values():
            if agent.state == MOVING:
                T    = agent.arrive_time - agent.depart_time
                frac = min(1.0, (sim_time - agent.depart_time) / T) if T > 0 else 1.0
                agent.x = agent.from_x + (agent.to_x - agent.from_x) * frac
                agent.y = agent.from_y + (agent.to_y - agent.from_y) * frac

    # ── 헬퍼 ──────────────────────────────────────────────────────────────────

    def _schedule(self, time: float, kind: str, agent_id: int, **data):
        ev = Event(time, self._seq, kind, agent_id, data)
        self._seq += 1
        heapq.heappush(self._eq, ev)

    def _stop_id_for(self, node_id: str, heading_rad: float) -> Optional[str]:
        """노드 ID와 heading(radian) → stop_state_id 조회."""
        angle_deg = round(math.degrees(heading_rad)) % 360
        sid = f"S,{node_id},{angle_deg}"
        if sid in self.graph.stop_states_raw:
            return sid
        # 가장 가까운 heading으로 fallback
        best = None
        best_diff = float('inf')
        for k in self.graph.stop_states_raw:
            parts = k.split(',')
            if parts[0] == 'S' and parts[1] == str(node_id):
                d = min((int(parts[2]) - angle_deg) % 360,
                        (angle_deg - int(parts[2])) % 360)
                if d < best_diff:
                    best_diff = d
                    best = k
        return best

    def _collect_needed(self, move_id: str,
                        to_stop_id: Optional[str]) -> set:
        """출발 전 예약해야 할 state_id 집합을 반환합니다."""
        needed = set()
        needed.add(move_id)
        if to_stop_id:
            needed.add(to_stop_id)

        # affect_state 추가
        for sid in list(needed):
            obj = (self.graph.move_states_raw.get(sid)
                   or self.graph.stop_states_raw.get(sid))
            if obj and hasattr(obj, 'affect_state'):
                needed.update(obj.affect_state)
        return needed

    # ── 상태 조회 ──────────────────────────────────────────────────────────────

    def all_done(self) -> bool:
        return all(a.state == DONE for a in self.agents.values())

    def reservation_count(self) -> int:
        return len(self.reservations)
