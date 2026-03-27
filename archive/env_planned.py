"""
env_planned.py — Lightweight path executor for pre-planned conflict-free paths.

SIPP planner가 보장한 시간 기반 경로를 따라 에이전트를 이동시킵니다.
런타임 예약/충돌 체크 없이 순수 경로 실행 + 보간만 수행합니다.

에이전트 상태
────────────
  IDLE    : 다음 이동 준비 중 (노드에 정차)
  MOVING  : 엣지 이동 중
  DONE    : 경로 완주
"""
from __future__ import annotations
import math
from typing import Dict, List, Optional, Tuple

# ── 상태 상수 ────────────────────────────────────────────────────────────────
IDLE    = 'idle'
MOVING  = 'moving'
DONE    = 'done'


# ── Agent ────────────────────────────────────────────────────────────────────

class PlannedAgent:
    """Planned path를 따라 이동하는 에이전트."""

    def __init__(self, agent_id: int, color: tuple, node_path: List[str]):
        self.id        = agent_id
        self.color     = color
        self.node_path = node_path
        self.path_idx  = 0
        self.state     = IDLE

        # 시각적 위치
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.v     = 0.0

        # 이동 보간용
        self.from_x: float = 0.0
        self.from_y: float = 0.0
        self.to_x:   float = 0.0
        self.to_y:   float = 0.0
        self.depart_time: float = 0.0
        self.arrive_time: float = 1.0

    @property
    def cur_node(self) -> str:
        if self.path_idx < len(self.node_path):
            return self.node_path[self.path_idx]
        return self.node_path[-1]


# ── Environment ──────────────────────────────────────────────────────────────

class PlannedEnvironment:
    """
    충돌 회피 없이 계획된 경로를 실행하는 환경.

    step(sim_time) 호출 시:
    1. IDLE 에이전트 → 다음 엣지 출발 (즉시)
    2. MOVING 에이전트 → 도착 시간이면 다음 노드로 전진
    3. 위치 보간 (MOVING 에이전트)
    """

    def __init__(self, graph):
        self.graph  = graph
        self.agents: Dict[int, PlannedAgent] = {}

    def add_agent(self, agent: PlannedAgent, t_start: float = 0.0):
        self.agents[agent.id] = agent
        if not agent.node_path or len(agent.node_path) < 2:
            agent.state = DONE
            return

        node = self.graph.nodes.get(agent.node_path[0])
        if node:
            agent.x = node.x
            agent.y = node.y

        # 초기 heading
        if len(agent.node_path) >= 2:
            edge = self.graph.get_edge(agent.node_path[0], agent.node_path[1])
            if edge:
                agent.theta = edge.angle

        # 첫 출발 예약
        self._try_depart(agent, t_start)

    def remove_agent(self, agent_id: int):
        self.agents.pop(agent_id, None)

    def step(self, sim_time: float):
        """sim_time까지 에이전트 상태를 갱신합니다."""
        for agent in list(self.agents.values()):
            if agent.state == DONE:
                continue

            # MOVING → 도착 체크
            if agent.state == MOVING and sim_time >= agent.arrive_time:
                agent.path_idx += 1
                node = self.graph.nodes.get(agent.cur_node)
                if node:
                    agent.x = node.x
                    agent.y = node.y
                agent.state = IDLE
                agent.v = 0.0

                # 즉시 다음 구간 출발 시도
                self._try_depart(agent, agent.arrive_time)

            # MOVING → 위치 보간
            if agent.state == MOVING:
                T = agent.arrive_time - agent.depart_time
                frac = min(1.0, (sim_time - agent.depart_time) / T) if T > 0 else 1.0
                agent.x = agent.from_x + (agent.to_x - agent.from_x) * frac
                agent.y = agent.from_y + (agent.to_y - agent.from_y) * frac

    def _try_depart(self, agent: PlannedAgent, t: float):
        """에이전트의 다음 구간 출발을 시도합니다."""
        idx = agent.path_idx
        if idx >= len(agent.node_path) - 1:
            agent.state = DONE
            agent.v = 0.0
            return

        from_n = agent.node_path[idx]
        to_n   = agent.node_path[idx + 1]
        edge   = self.graph.get_edge(from_n, to_n)
        if edge is None:
            agent.state = DONE
            return

        from_node = self.graph.nodes[from_n]
        to_node   = self.graph.nodes[to_n]
        travel    = edge.length / edge.max_speed if edge.max_speed > 0 else 1.0

        agent.from_x      = from_node.x
        agent.from_y      = from_node.y
        agent.to_x        = to_node.x
        agent.to_y        = to_node.y
        agent.theta        = edge.angle
        agent.depart_time = t
        agent.arrive_time = t + travel
        agent.state       = MOVING
        agent.v           = edge.max_speed

    def all_done(self) -> bool:
        return all(a.state == DONE for a in self.agents.values())
