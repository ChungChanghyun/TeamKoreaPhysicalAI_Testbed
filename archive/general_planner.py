"""
general_planner.py — GeneralGraph 용 경량 플래너.

CBS 없이 우선순위 기반(prioritized) 공간-시간 A* 를 사용합니다.
에이전트를 순서대로 계획하며, 이미 계획된 에이전트의 점유 시간을
예약 테이블(reservation table)에 등록해 충돌을 회피합니다.

TAPG 와의 역할 분담
────────────────────
  이 플래너  : 물리적 충돌이 없는 경로·타이밍 생성 (공간-시간 분리)
  TAPG       : 실행 중 타이밍 오차를 감지하고 순서를 강제 (실행 시 안전망)
"""
from __future__ import annotations
import heapq, math
from typing import Dict, List, Tuple, Optional
INF = math.inf


# ── BFS (토폴로지 최단 경로) ──────────────────────────────────────────────────

def bfs_path(graph, start: str, goal: str) -> Optional[List[str]]:
    """홉 수 기준 최단 경로. 없으면 None."""
    if start == goal:
        return [start]
    from collections import deque
    prev = {start: None}
    q    = deque([start])
    while q:
        cur = q.popleft()
        for nb in graph.adj.get(cur, []):
            if nb not in prev:
                prev[nb] = cur
                if nb == goal:
                    path = []
                    node = nb
                    while node is not None:
                        path.append(node)
                        node = prev[node]
                    return list(reversed(path))
                q.append(nb)
    return None


# ── 공간-시간 A* ──────────────────────────────────────────────────────────────

class ReservationTable:
    """
    에이전트가 점유하는 (node, time_interval) 쌍을 저장합니다.
    시간을 dt 단위로 이산화해 충돌을 판정합니다.
    """

    def __init__(self, dt: float = 0.1):
        self.dt  = dt
        # {node_id: sorted list of (t_enter, t_leave)} — 예약된 시간 구간
        self._table: Dict[str, List[Tuple[float, float]]] = {}

    def reserve(self, node: str, t_enter: float, t_leave: float):
        self._table.setdefault(node, []).append((t_enter, t_leave))

    def is_free(self, node: str, t_enter: float, t_leave: float,
                margin: float = 0.05) -> bool:
        """해당 노드의 [t_enter, t_leave] 구간이 예약 없으면 True."""
        for (ta, tb) in self._table.get(node, []):
            # 겹침 여부: [ta-m, tb+m] ∩ [t_enter, t_leave] ≠ ∅
            if not (t_leave + margin <= ta or t_enter >= tb + margin):
                return False
        return True

    def reserve_path(self, raw_path: list):
        """raw_path [(state_id, t), ...] 의 S 상태를 전부 예약합니다."""
        path = [(sid, t) for sid, t in raw_path if sid.startswith('S,')]
        for k, (sid, t_enter) in enumerate(path):
            nid = sid.split(',')[1]
            t_leave = path[k + 1][1] if k + 1 < len(path) else t_enter + 1.0
            self.reserve(nid, t_enter, t_leave)


# ── raw_path 생성 ─────────────────────────────────────────────────────────────

def path_to_raw(graph, node_path: List[str],
                start_time: float = 0.0) -> list:
    """
    node 경로 → raw_path [(state_id, cbs_time), ...].
    S 상태와 M 상태를 엣지 이동 시간 기준으로 시간을 배정합니다.
    """
    raw = []
    t   = start_time
    raw.append((f'S,{node_path[0]},0', t))
    for i in range(len(node_path) - 1):
        fn, tn = node_path[i], node_path[i + 1]
        raw.append((f'M,{fn},{tn}', t))
        edge = graph.get_edge(fn, tn)
        t   += edge.length / edge.max_speed
        raw.append((f'S,{tn},0', t))
    return raw


def spacetime_astar(graph, start: str, goal: str,
                    table: ReservationTable,
                    start_time: float = 0.0,
                    max_wait: float   = 30.0) -> Optional[list]:
    """
    공간-시간 A*.  예약 테이블을 참고해 충돌 없는 경로를 탐색합니다.

    상태: (node, time)
    휴리스틱: 유클리드 거리 / max_speed
    대기: 같은 노드에서 dt 만큼 대기 가능 (총 max_wait 초 한도)
    """
    dt       = table.dt
    max_spd  = max((e.max_speed for e in graph.edges.values()), default=1000.0)
    goal_pos = (graph.nodes[goal].x, graph.nodes[goal].y)

    def heuristic(nid):
        n = graph.nodes[nid]
        return math.hypot(n.x - goal_pos[0], n.y - goal_pos[1]) / max_spd

    # (f, seq, node, time, path)
    seq   = 0
    start_state = (start_time, start, [])
    heap  = [(heuristic(start) + start_time, seq, start, start_time, [])]
    best  = {}   # (node, t_bucket) → best_g

    while heap:
        f, _, node, t, path = heapq.heappop(heap)
        key = (node, round(t / dt))
        if key in best and best[key] <= t:
            continue
        best[key] = t

        new_path = path + [(f'S,{node},0', t)]

        if node == goal:
            return new_path

        # 이동: 인접 노드로
        for nb in graph.adj.get(node, []):
            edge   = graph.get_edge(node, nb)
            t_move = edge.length / edge.max_speed
            t_arr  = t + t_move
            if table.is_free(nb, t, t_arr):
                move_path = new_path + [(f'M,{node},{nb}', t)]
                # S 상태는 append 를 다음 반복에서
                g = t_arr
                h = heuristic(nb)
                seq += 1
                heapq.heappush(heap, (g + h, seq, nb, t_arr,
                                      new_path + [(f'M,{node},{nb}', t)]))

        # 대기: 현재 노드에서 dt 대기
        t_wait = t + dt
        if t_wait - start_time <= max_wait and table.is_free(node, t, t_wait):
            seq += 1
            heapq.heappush(heap, (t_wait + heuristic(node), seq,
                                  node, t_wait, path))

    return None   # 경로 없음


# ── 멀티 에이전트 플래너 ──────────────────────────────────────────────────────

def plan_agents(graph,
                start_goal_pairs: List[Tuple[str, str]],
                use_spacetime:    bool  = True,
                dt:               float = 0.1,
                max_wait:         float = 30.0
                ) -> List[list]:
    """
    우선순위 기반 멀티 에이전트 플래너.

    Parameters
    ----------
    graph           : GeneralGraph
    start_goal_pairs: [(start_node, goal_node), ...]
    use_spacetime   : True  → 공간-시간 A* (충돌 회피)
                      False → BFS (토폴로지만, TAPG 에 충돌 해소 위임)

    Returns
    -------
    raw_paths : [[(state_id, time), ...], ...]
    """
    table     = ReservationTable(dt=dt)
    raw_paths = []

    for i, (start, goal) in enumerate(start_goal_pairs):
        if use_spacetime:
            raw = spacetime_astar(graph, start, goal, table,
                                  start_time=0.0, max_wait=max_wait)
            if raw is None:
                raise RuntimeError(
                    f'Agent {i+1}: no path found from {start} to {goal}')
        else:
            node_path = bfs_path(graph, start, goal)
            if node_path is None:
                raise RuntimeError(
                    f'Agent {i+1}: no path found from {start} to {goal}')
            raw = path_to_raw(graph, node_path)

        table.reserve_path(raw)
        raw_paths.append(raw)
        print(f'  Agent {i+1}: {start} → {goal}  '
              f'({len([s for s,_ in raw if s.startswith("S,")])} stops, '
              f'arrival={raw[-1][1]:.1f}s)')

    return raw_paths


# ── 경로 출력 유틸 ────────────────────────────────────────────────────────────

def print_raw_path(raw_path: list, label: str = ''):
    if label:
        print(f'\n{label}')
    for sid, t in raw_path:
        print(f'  {t:6.2f}s  {sid}')
