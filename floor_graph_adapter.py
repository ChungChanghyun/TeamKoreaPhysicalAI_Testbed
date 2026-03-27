"""
floor_graph_adapter.py — FloorGraph → PklMapGraph 호환 어댑터.

3DS FloorGraph(단순 노드/엣지)를 PklMapGraph 인터페이스로 변환하여
PklPrioritizedPlanner(SIPP) + TAPGEnvironment 를 그대로 사용할 수 있게 합니다.

3DS는 회전이 없으므로 heading=0 고정, R 상태 없음.

State graph 구조
────────────────
  Stop: S,node_id,0    cost=0    next_state → [M,node_id,nb for nb in neighbors]
  Move: M,from,to      cost=L/v  next_state → [S,to,0]

Affect state (충돌 프로파일, 노드 기반)
────────────────────────────────────────
  S,n,0 은 노드 n을 점유 → n을 사용하는 모든 M 상태와 충돌
  M,a,b 는 노드 a,b를 점유 → a,b에 관련된 모든 S,M 상태와 충돌
"""
from __future__ import annotations
import math
from typing import Dict, List, Tuple, Optional
from env_3ds import FloorGraph, FloorNode


# ── Lightweight State (PklMapGraph State 호환) ───────────────────────────────

class SimpleState:
    """PklPrioritizedPlanner가 요구하는 최소 State 인터페이스."""
    __slots__ = ('state_id', 'cost', 'next_state', 'affect_state')

    def __init__(self, state_id: str, cost: float,
                 next_state: List[str], affect_state: List[str]):
        self.state_id     = state_id
        self.cost         = cost
        self.next_state   = next_state
        self.affect_state = affect_state


# ── Minimal Node/Edge (PklMapGraph 호환) ─────────────────────────────────────

class AdapterNode:
    __slots__ = ('id', 'x', 'y', 'kind')

    def __init__(self, nid: str, x: float, y: float):
        self.id   = nid
        self.x    = x
        self.y    = y
        self.kind = 'Normal'


class AdapterEdge:
    __slots__ = ('id', 'from_id', 'to_id', 'length', 'angle', 'max_speed')

    def __init__(self, from_id: str, to_id: str,
                 from_node: AdapterNode, to_node: AdapterNode,
                 max_speed: float = 1000.0):
        self.id        = f"M,{from_id},{to_id}"
        self.from_id   = from_id
        self.to_id     = to_id
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        self.length    = math.hypot(dx, dy)
        self.angle     = math.atan2(dy, dx)
        self.max_speed = max_speed


# ── FloorGraphAdapter ────────────────────────────────────────────────────────

class FloorGraphAdapter:
    """
    FloorGraph를 PklMapGraph 인터페이스로 래핑.

    PklPrioritizedPlanner, TAPGEnvironment와 호환됩니다.

    Usage
    ─────
      fg = FloorGraph('3DS_F1')
      # ... add nodes/edges ...
      adapter = FloorGraphAdapter(fg, port_nodes=['00000365', '00000370'])
      planner = PklPrioritizedPlanner(adapter)
      env     = TAPGEnvironment(adapter, accel=500, decel=500)
    """

    def __init__(self, floor_graph: FloorGraph,
                 port_nodes: List[str] = None,
                 default_speed: float = 1000.0):
        self.floor_id = floor_graph.floor_id
        self._fg      = floor_graph
        self._speed   = default_speed

        # ── PklMapGraph 호환 속성 빌드 ──
        self.nodes: Dict[str, AdapterNode] = {}
        self.edges: Dict[Tuple[str, str], AdapterEdge] = {}
        self.adj:   Dict[str, List[str]] = {}

        # 노드
        for nid, fn in floor_graph.nodes.items():
            self.nodes[nid] = AdapterNode(nid, fn.x, fn.y)
            self.adj[nid] = floor_graph.adj.get(nid, [])

        # 엣지
        for (fid, tid), fe in floor_graph.edges.items():
            self.edges[(fid, tid)] = AdapterEdge(
                fid, tid, self.nodes[fid], self.nodes[tid],
                fe.max_speed if hasattr(fe, 'max_speed') else default_speed)

        # 차량 크기 (3DS: 850×850mm)
        self.vehicle_length = 850.0
        self.vehicle_width  = 850.0

        # 포트
        if port_nodes:
            self.ports = {str(i): nid for i, nid in enumerate(port_nodes)}
        else:
            # 기본: 모든 끝단 노드 (인접 1개 이하)를 포트로
            self.ports = {}
            idx = 0
            for nid in self.nodes:
                if len(self.adj.get(nid, [])) <= 1:
                    self.ports[str(idx)] = nid
                    idx += 1
            # 포트가 너무 적으면 모든 노드를 포트로
            if len(self.ports) < 2:
                self.ports = {str(i): nid for i, nid in enumerate(self.nodes)}

        # ── State graph 빌드 ──
        self.stop_states_raw:   Dict[str, SimpleState] = {}
        self.move_states_raw:   Dict[str, SimpleState] = {}
        self.rotate_states_raw: Dict[str, SimpleState] = {}  # 3DS는 회전 없음

        # 노드→관련 상태 ID 매핑 (affect_state 계산용)
        self._node_states: Dict[str, List[str]] = {nid: [] for nid in self.nodes}

        self._build_states()

        # stop_regions / move_regions — 시각화용 (없어도 플래닝 동작)
        self.stop_regions = {}
        self.move_regions = {}

    def _build_states(self):
        """Stop/Move 상태 그래프 + affect_state 빌드."""
        # 1) Stop 상태 생성
        for nid in self.nodes:
            sid = f'S,{nid},0'
            next_states = []
            for nb in self.adj.get(nid, []):
                mid = f'M,{nid},{nb}'
                next_states.append(mid)
            self.stop_states_raw[sid] = SimpleState(
                sid, cost=0.0, next_state=next_states, affect_state=[])
            self._node_states[nid].append(sid)

        # 2) Move 상태 생성
        for (fid, tid), edge in self.edges.items():
            mid = f'M,{fid},{tid}'
            cost = edge.length / edge.max_speed if edge.max_speed > 0 else 0.0
            next_states = [f'S,{tid},0']
            self.move_states_raw[mid] = SimpleState(
                mid, cost=cost, next_state=next_states, affect_state=[])
            self._node_states[fid].append(mid)
            self._node_states[tid].append(mid)

        # 3) Affect state: 같은 노드를 점유하는 모든 상태끼리 충돌
        for nid, state_ids in self._node_states.items():
            for sid in state_ids:
                state = (self.stop_states_raw.get(sid) or
                         self.move_states_raw.get(sid))
                if state is None:
                    continue
                # 이 노드를 사용하는 다른 모든 상태가 affect
                affects = [other for other in state_ids if other != sid]
                state.affect_state = affects

    # ── PklMapGraph 호환 메서드 ──────────────────────────────────────────────

    def get_edge(self, from_id: str, to_id: str):
        return self.edges.get((from_id, to_id))

    def neighbors(self, node_id: str) -> List[str]:
        return self.adj.get(node_id, [])

    @property
    def bbox(self) -> Tuple[float, float, float, float]:
        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        if not xs:
            return (0, 0, 0, 0)
        return min(xs), min(ys), max(xs), max(ys)
