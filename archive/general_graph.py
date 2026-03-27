"""
general_graph.py — PKL 없이 임의 방향 그래프를 정의하는 유틸리티.

PklMapGraph 인터페이스와 호환되므로 TAPGEnvironment/TAPGEnvironmentDES,
vis_tapg.py 등을 그대로 재사용할 수 있습니다.

충돌 규칙 (affect_state)
─────────────────────────
  'head_on'      : 반대 방향 엣지만 충돌로 처리
  'node_sharing' : 같은 출발·도착 노드를 공유하는 모든 엣지도 충돌로 처리 (기본)
"""
from __future__ import annotations
import math
from typing import Dict, List, Tuple, Optional


# ── 기본 자료구조 ──────────────────────────────────────────────────────────────

class GNode:
    def __init__(self, node_id: str, x: float, y: float, kind: str = 'Normal'):
        self.id   = node_id
        self.x    = x
        self.y    = y
        self.kind = kind

    def __repr__(self):
        return f'GNode({self.id}, {self.x/1000:.1f}m, {self.y/1000:.1f}m)'


class GEdge:
    def __init__(self, from_id: str, to_id: str,
                 from_node: GNode, to_node: GNode,
                 max_speed: float = 1000.0):
        self.id        = f'M,{from_id},{to_id}'
        self.from_id   = from_id
        self.to_id     = to_id
        dx             = to_node.x - from_node.x
        dy             = to_node.y - from_node.y
        self.length    = math.hypot(dx, dy)
        self.angle     = math.atan2(dy, dx)   # map-space (y-up)
        self.max_speed = max_speed

    def __repr__(self):
        return f'GEdge({self.from_id}→{self.to_id}, L={self.length:.0f}mm)'


class GState:
    """
    Stop / Move 상태.
    TAPG가 affect_state 를 조회해 교차 에이전트 의존성을 결정합니다.
    """
    def __init__(self, state_id: str, affect_state: list = None):
        self.id           = state_id
        self.affect_state = affect_state or []


# ── 일반 그래프 ────────────────────────────────────────────────────────────────

class GeneralGraph:
    """
    임의 방향 그래프.  PklMapGraph 인터페이스와 호환.

    Parameters
    ----------
    nodes_xy       : {node_id: (x_mm, y_mm)}
    edges          : [(from_id, to_id), ...]  단방향 엣지 목록
    max_speed      : 기본 엣지 최고속도 (mm/s)
    vehicle_length : AGV 길이 (mm)
    vehicle_width  : AGV 폭   (mm)
    conflict_rule  : 'head_on' | 'node_sharing'
    port_nodes     : port 로 표시할 node_id 목록 (시각화용)
    """

    def __init__(self,
                 nodes_xy:       Dict[str, Tuple[float, float]],
                 edges:          List[Tuple[str, str]],
                 max_speed:      float = 1000.0,
                 vehicle_length: float = 2000.0,
                 vehicle_width:  float = 1000.0,
                 conflict_rule:  str   = 'head_on',
                 port_nodes:     List[str] = None):

        # ── 노드 ──────────────────────────────────────────────────────────────
        self.nodes: Dict[str, GNode] = {
            nid: GNode(nid, x, y)
            for nid, (x, y) in nodes_xy.items()
        }

        # ── 엣지 ──────────────────────────────────────────────────────────────
        self.edges: Dict[Tuple[str, str], GEdge] = {}
        self.adj:   Dict[str, List[str]]          = {nid: [] for nid in self.nodes}

        for fn, tn in edges:
            if fn not in self.nodes or tn not in self.nodes:
                continue
            if (fn, tn) in self.edges:
                continue
            self.edges[(fn, tn)] = GEdge(fn, tn,
                                         self.nodes[fn], self.nodes[tn],
                                         max_speed)
            self.adj.setdefault(fn, []).append(tn)

        # ── 시각화용 속성 (PklMapGraph 호환) ──────────────────────────────────
        self.vehicle_length = vehicle_length
        self.vehicle_width  = vehicle_width
        self.stop_regions   = {}
        self.move_regions   = {}
        self.ports: Dict[str, str] = {}
        if port_nodes:
            for i, nid in enumerate(port_nodes):
                self.ports[str(i)] = nid

        # ── Stop / Move 상태 생성 ──────────────────────────────────────────────
        self._build_states(conflict_rule)

    # ── 상태 생성 ─────────────────────────────────────────────────────────────

    def _build_states(self, rule: str):
        edge_set = set(self.edges.keys())

        # 노드별 관련 엣지 인덱스 (stop affect_state 계산용)
        node_moves: Dict[str, List[str]] = {nid: [] for nid in self.nodes}
        for (fn, tn) in self.edges:
            mid = f'M,{fn},{tn}'
            node_moves[fn].append(mid)   # 출발하는 move
            node_moves[tn].append(mid)   # 도착하는 move

        # Stop 상태: 노드당 1개
        # affect_state: 해당 노드를 출발/도착하는 모든 Move + 해당 노드의 Stop
        self.stop_states_raw: Dict[str, GState] = {}
        for nid in self.nodes:
            sid = f'S,{nid},0'
            # 이 노드에서 출발하거나 이 노드에 도착하는 모든 move를 차단
            conflicts = list(node_moves[nid])
            self.stop_states_raw[sid] = GState(sid, conflicts)

        # Move 상태: 엣지당 1개 + affect_state 계산
        self.move_states_raw: Dict[str, GState] = {}
        for (fn, tn) in self.edges:
            sid = f'M,{fn},{tn}'
            conflicts: List[str] = []

            # head-on 충돌: 반대 방향 엣지
            if (tn, fn) in edge_set:
                conflicts.append(f'M,{tn},{fn}')

            # 출발/도착 노드의 stop state 충돌
            conflicts.append(f'S,{fn},0')
            conflicts.append(f'S,{tn},0')

            if rule == 'node_sharing':
                # 같은 출발 노드를 공유하는 엣지 (두 AGV가 동시에 같은 노드 이탈)
                for (fn2, tn2) in edge_set:
                    if fn2 == fn and tn2 != tn:
                        conflicts.append(f'M,{fn2},{tn2}')
                # 같은 도착 노드를 공유하는 엣지 (두 AGV가 동시에 같은 노드 진입)
                for (fn2, tn2) in edge_set:
                    if tn2 == tn and fn2 != fn:
                        conflicts.append(f'M,{fn2},{tn2}')

            self.move_states_raw[sid] = GState(sid, list(set(conflicts)))

    # ── PklMapGraph 호환 메서드 ────────────────────────────────────────────────

    def get_edge(self, from_id: str, to_id: str) -> Optional[GEdge]:
        return self.edges.get((from_id, to_id))

    def neighbors(self, node_id: str) -> List[str]:
        return self.adj.get(node_id, [])

    @property
    def bbox(self) -> Tuple[float, float, float, float]:
        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        pad = self.vehicle_length
        return min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad

    def __repr__(self):
        return (f'GeneralGraph({len(self.nodes)} nodes, '
                f'{len(self.edges)} edges)')
