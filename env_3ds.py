"""
env_3ds.py — 3D Shuttle DES 환경.

층당 셔틀 1대 → 층 내 충돌 회피 불필요.
AGV(env_tapg) 구조를 따르되 TAPG 없이 단순 경로 추종 + 가감속 물리.

셔틀 상태
─────────
  IDLE      대기 중 (노드 위)
  MOVING    엣지 이동 중
  DONE      경로 완료

DES 이벤트
──────────
  S3D_TRY_ADVANCE   다음 엣지 진입 시도
  S3D_ARRIVE         엣지 끝 도달 (물리 기반 감지)

맵 구조
───────
  KaistTB_map.json의 노드/세그먼트를 area별로 분리하여 층별 그래프 구성.
  각 층은 독립 그래프이며, 층간 이동은 elevator.py가 담당.

Usage
─────
  env = Env3DS(floors={'1': graph_f1, '2': graph_f2, '3': graph_f3},
               accel=500.0, decel=500.0)
  env.add_shuttle('1', shuttle_id=0, start_node='00000365')
  env.add_shuttle('2', shuttle_id=1, start_node='00000031')

  env.assign(shuttle_id=0, node_path=['00000365','00000031',...], t=0.0)
  env.step(sim_time)

job_3ds 연동
────────────
  job_ctrl.on_shuttle_path_complete 호출은 env가 경로 완료 시 콜백으로 처리.
"""
from __future__ import annotations
import math
import heapq
import collections
from dataclasses import dataclass, field
from env_tapg import _compute_travel, _pos_along_profile
from typing import Dict, List, Optional, Tuple, Callable


# ── States ───────────────────────────────────────────────────────────────────
IDLE   = 'IDLE'
MOVING = 'MOVING'
DONE   = 'DONE'

# ── Event types ──────────────────────────────────────────────────────────────
S3D_TRY_ADVANCE = 'S3D_TRY_ADVANCE'
S3D_ARRIVE      = 'S3D_ARRIVE'


# ── Event (env_oht_des 호환) ─────────────────────────────────────────────────

class Event:
    __slots__ = ('t', 'kind', 'agent_id', 'token', 'data')

    def __init__(self, t: float, kind: str, agent_id: int,
                 token: int, data=None):
        self.t        = t
        self.kind     = kind
        self.agent_id = agent_id
        self.token    = token
        self.data     = data

    def __lt__(self, other):
        return self.t < other.t


# ── Floor Graph ──────────────────────────────────────────────────────────────

class FloorNode:
    __slots__ = ('id', 'x', 'y')

    def __init__(self, nid: str, x: float, y: float):
        self.id = nid
        self.x  = x
        self.y  = y


class FloorEdge:
    __slots__ = ('from_id', 'to_id', 'length', 'angle', 'max_speed')

    def __init__(self, from_id: str, to_id: str,
                 from_node: FloorNode, to_node: FloorNode,
                 max_speed: float = 1000.0):
        self.from_id   = from_id
        self.to_id     = to_id
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        self.length    = math.hypot(dx, dy)
        self.angle     = math.atan2(dy, dx)
        self.max_speed = max_speed


class FloorGraph:
    """
    한 층의 노드/엣지 그래프.

    Parameters
    ----------
    floor_id  : 층 식별자
    nodes     : {node_id: FloorNode}
    edges     : {(from_id, to_id): FloorEdge}
    """

    def __init__(self, floor_id: str):
        self.floor_id = floor_id
        self.nodes: Dict[str, FloorNode] = {}
        self.edges: Dict[Tuple[str, str], FloorEdge] = {}
        self.adj:   Dict[str, List[str]] = {}

    def add_node(self, nid: str, x: float, y: float):
        self.nodes[nid] = FloorNode(nid, x, y)
        self.adj.setdefault(nid, [])

    def add_edge(self, from_id: str, to_id: str,
                 max_speed: float = 1000.0):
        if from_id not in self.nodes or to_id not in self.nodes:
            return
        e = FloorEdge(from_id, to_id,
                      self.nodes[from_id], self.nodes[to_id],
                      max_speed)
        self.edges[(from_id, to_id)] = e
        if to_id not in self.adj.get(from_id, []):
            self.adj.setdefault(from_id, []).append(to_id)

    def neighbors(self, nid: str) -> List[str]:
        return self.adj.get(nid, [])

    def bfs(self, start: str, goal: str = None) -> List[str]:
        """BFS 경로 탐색. goal=None이면 가장 먼 노드까지."""
        if start not in self.nodes:
            return [start]
        visited = {start}
        queue = collections.deque([(start, [start])])
        best = [start]
        while queue:
            cur, path = queue.popleft()
            if goal is not None and cur == goal:
                return path
            if len(path) > len(best):
                best = path
            for nb in self.adj.get(cur, []):
                if nb not in visited:
                    visited.add(nb)
                    queue.append((nb, path + [nb]))
        return best if goal is None else [start]

    @property
    def bbox(self) -> Tuple[float, float, float, float]:
        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        if not xs:
            return (0, 0, 0, 0)
        return min(xs), min(ys), max(xs), max(ys)


def build_floor_graph(floor_id: str, nodes_data: list,
                      segments_data: list,
                      max_speed: float = 1000.0) -> FloorGraph:
    """
    KaistTB_map.json에서 특정 area의 노드/세그먼트로 FloorGraph 생성.

    Parameters
    ----------
    nodes_data    : [{'id', 'x', 'y', 'area'}, ...]
    segments_data : [{'startNodeId', 'endNodeId', ...}, ...]
    """
    fg = FloorGraph(floor_id)
    floor_nodes = set()
    for nd in nodes_data:
        if nd.get('area') == floor_id:
            fg.add_node(nd['id'], nd['x'], nd['y'])
            floor_nodes.add(nd['id'])

    for seg in segments_data:
        fid = seg.get('startNodeId', seg.get('from', ''))
        tid = seg.get('endNodeId', seg.get('to', ''))
        if fid in floor_nodes and tid in floor_nodes:
            fg.add_edge(fid, tid, max_speed)
            # 양방향
            if seg.get('bidirectional', True):
                fg.add_edge(tid, fid, max_speed)

    return fg


# ── Shuttle ──────────────────────────────────────────────────────────────────

class Shuttle:
    """3D Shuttle 에이전트."""

    def __init__(self, shuttle_id: int, floor_id: str,
                 color: tuple = (100, 200, 255),
                 max_speed: float = 1000.0):
        self.id        = shuttle_id
        self.floor_id  = floor_id
        self.color     = color
        self.max_speed = max_speed

        # 경로
        self.node_path: List[str] = []
        self.path_idx:  int       = 0

        # 상태
        self.state = IDLE
        self.token = 0

        # 위치 / 속도
        self.x:     float = 0.0
        self.y:     float = 0.0
        self.theta: float = 0.0
        self.v:     float = 0.0

        # DES 운동학 프로파일
        self.from_x:        float = 0.0
        self.from_y:        float = 0.0
        self.to_x:          float = 0.0
        self.to_y:          float = 0.0
        self.edge_length:   float = 0.0
        self.edge_speed:    float = 0.0
        self.entry_t:       float = 0.0   # 엣지 진입 시각
        self.v0:            float = 0.0   # 진입 시 속도
        self.do_decel:      bool  = True
        self._t_accel:      float = 0.0
        self._d_accel:      float = 0.0
        self._t_cruise:     float = 0.0
        self._d_cruise:     float = 0.0
        self._t_decel:      float = 0.0
        self._t_total:      float = 0.0
        self._v_exit:       float = 0.0

    @property
    def cur_node(self) -> str:
        if self.path_idx < len(self.node_path):
            return self.node_path[self.path_idx]
        if self.node_path:
            return self.node_path[-1]
        return ''


# ── Env3DS ───────────────────────────────────────────────────────────────────

class Env3DS:
    """
    3D Shuttle DES 환경.

    Parameters
    ----------
    floors : {floor_id: FloorGraph}
    accel  : mm/s² (가속도, INF → 즉시 최고속도)
    decel  : mm/s² (감속도, INF → 즉시 정지)
    heap   : 외부 공유 힙. None이면 자체 힙 생성.
    on_path_complete : callable(shuttle_id, node_id, t)
                       경로 완료 시 콜백 (job_3ds 연동용)
    """

    def __init__(self, floors: Dict[str, FloorGraph], *,
                 accel: float = math.inf,
                 decel: float = math.inf,
                 heap: list = None,
                 on_path_complete: Callable = None):
        self.floors = floors
        self.accel  = accel
        self.decel  = decel
        self._heap  = heap if heap is not None else []
        self.on_path_complete = on_path_complete

        self._shuttles: Dict[int, Shuttle] = {}
        self._floor_shuttle: Dict[str, int] = {}   # floor_id → shuttle_id

    # ── 셔틀 등록 ────────────────────────────────────────────────────────────

    def add_shuttle(self, floor_id: str, shuttle_id: int,
                    start_node: str, color: tuple = (100, 200, 255),
                    max_speed: float = 1000.0):
        fg = self.floors.get(floor_id)
        if fg is None:
            raise ValueError(f"Floor {floor_id} not registered")

        s = Shuttle(shuttle_id, floor_id, color, max_speed)
        node = fg.nodes.get(start_node)
        if node:
            s.x, s.y = node.x, node.y
        s.node_path = [start_node]
        s.path_idx  = 0

        self._shuttles[shuttle_id] = s
        self._floor_shuttle[floor_id] = shuttle_id

    def get_shuttle(self, shuttle_id: int) -> Optional[Shuttle]:
        return self._shuttles.get(shuttle_id)

    def floor_shuttle(self, floor_id: str) -> Optional[Shuttle]:
        sid = self._floor_shuttle.get(floor_id)
        return self._shuttles.get(sid) if sid is not None else None

    # ── 경로 할당 ────────────────────────────────────────────────────────────

    def assign(self, shuttle_id: int, node_path: List[str], t: float):
        """셔틀에 새 노드 경로 할당."""
        s = self._shuttles.get(shuttle_id)
        if s is None:
            return
        s.node_path = list(node_path)
        s.path_idx  = 0
        s.state     = IDLE
        s.v         = 0.0
        s.token    += 1

        # 시작 위치 설정
        fg = self.floors.get(s.floor_id)
        if fg and node_path:
            node = fg.nodes.get(node_path[0])
            if node:
                s.x, s.y = node.x, node.y

        self._post_advance(t, s)

    # ── DES 이벤트 처리 ──────────────────────────────────────────────────────

    def handle_event(self, ev) -> bool:
        """3DS 이벤트이면 처리하고 True 반환."""
        if ev.kind == S3D_TRY_ADVANCE:
            s = self._shuttles.get(ev.agent_id)
            if s is None or ev.token != s.token:
                return True   # stale
            self._on_try_advance(ev.t, s)
            return True
        if ev.kind == S3D_ARRIVE:
            s = self._shuttles.get(ev.agent_id)
            if s is None or ev.token != s.token:
                return True   # stale
            self._on_arrive(ev.t, s)
            return True
        return False

    def step(self, t_now: float, dt: float = 0.0):
        """
        독립 실행 모드: 이벤트 처리 + 시각화 보간.

        Parameters
        ----------
        t_now : 현재 시뮬레이션 시각
        dt    : (무시됨, 하위호환용)
        """
        # 이벤트 처리 (TRY_ADVANCE, S3D_ARRIVE)
        while self._heap and self._heap[0].t <= t_now:
            ev = heapq.heappop(self._heap)
            self.handle_event(ev)
        # 시각화용 위치 보간
        self._interpolate_visuals(t_now)

    # ── Internal — 이벤트 핸들러 ─────────────────────────────────────────────

    def _on_try_advance(self, t: float, s: Shuttle):
        """다음 엣지로 진입."""
        nxt_idx = s.path_idx + 1
        if nxt_idx >= len(s.node_path):
            # 경로 완료
            s.state = DONE
            s.v     = 0.0
            if self.on_path_complete:
                self.on_path_complete(s.id, s.cur_node, t)
            return

        fg = self.floors.get(s.floor_id)
        if fg is None:
            s.state = DONE
            return

        from_id = s.node_path[s.path_idx]
        to_id   = s.node_path[nxt_idx]
        edge    = fg.edges.get((from_id, to_id))
        if edge is None:
            # 엣지 없음 — 스킵하고 다음 시도
            s.path_idx = nxt_idx
            self._post_advance(t, s)
            return

        # 엣지 진입
        from_node = fg.nodes[from_id]
        to_node   = fg.nodes[to_id]

        s.path_idx      = nxt_idx
        s.state         = MOVING
        s.from_x        = from_node.x
        s.from_y        = from_node.y
        s.to_x          = to_node.x
        s.to_y          = to_node.y
        s.edge_length   = edge.length
        s.edge_speed    = min(s.max_speed, edge.max_speed)
        s.theta         = edge.angle
        s.entry_t       = t

        is_last_edge = (s.path_idx + 1 >= len(s.node_path))
        do_decel = is_last_edge
        s.do_decel = do_decel

        # 초기 속도 결정
        if not math.isfinite(self.accel):
            s.v0 = s.edge_speed
        else:
            s.v0 = s.v  # 이전 속도 유지 (0 또는 carry_v)

        # 사다리꼴 프로파일 계산
        (s._t_accel, s._d_accel,
         s._t_cruise, s._d_cruise,
         s._t_decel, s._t_total,
         s._v_exit) = _compute_travel(
            s.v0, s.edge_speed, self.accel, self.decel,
            edge.length, do_decel)

        s.v = s.v0

        # ARRIVE 이벤트 예약
        s.token += 1
        self._post(t + s._t_total, S3D_ARRIVE, s)

    def _on_arrive(self, t: float, s: Shuttle):
        """엣지 끝 도달 처리."""
        s.x = s.to_x
        s.y = s.to_y
        carry_v = s._v_exit
        s.state = IDLE

        nxt_idx = s.path_idx + 1
        if nxt_idx < len(s.node_path) and carry_v > 0:
            # 연속 주행: 속도 유지
            s.v = carry_v
            self._post_advance(t, s)
        else:
            s.v = 0.0
            self._post_advance(t, s)

    # ── 시각화용 위치 보간 ────────────────────────────────────────────────────

    def _interpolate_visuals(self, t_now: float):
        """셔틀 위치를 운동학 프로파일에서 분석적으로 계산."""
        for s in self._shuttles.values():
            if s.state != MOVING:
                continue
            dt = t_now - s.entry_t
            if dt < 0:
                dt = 0.0
            pos, v = _pos_along_profile(
                dt, s.v0, self.accel, s.edge_speed,
                self.decel,
                s._t_accel, s._d_accel,
                s._t_cruise, s._d_cruise,
                s._t_decel, s.do_decel)
            s.v = v
            if s.edge_length > 0:
                frac = min(pos / s.edge_length, 1.0)
                s.x = s.from_x + (s.to_x - s.from_x) * frac
                s.y = s.from_y + (s.to_y - s.from_y) * frac

    # ── Internal — 이벤트 게시 ───────────────────────────────────────────────

    def _post(self, t: float, kind: str, s: Shuttle):
        heapq.heappush(self._heap,
                       Event(t, kind, s.id, s.token))

    def _post_advance(self, t: float, s: Shuttle):
        s.token += 1
        self._post(t, S3D_TRY_ADVANCE, s)

    # ── Queries ──────────────────────────────────────────────────────────────

    @property
    def shuttles(self) -> List[Shuttle]:
        return list(self._shuttles.values())

    def all_done(self) -> bool:
        return all(s.state == DONE for s in self._shuttles.values())
