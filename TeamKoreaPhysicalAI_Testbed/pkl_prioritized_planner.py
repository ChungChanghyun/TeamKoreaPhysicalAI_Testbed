"""
pkl_prioritized_planner.py — Dependency-graph based prioritized SIPP planner
for PklMapGraph (state-level planning with collision profiles).

Works on the state graph directly:
  Stop states: S,node_id,heading  (next_state → Move states)
  Move states: M,from_id,to_id   (next_state → Stop states)

Output: [(state_id, time), ...] per agent — directly consumable by TAPGEnvironment.
"""
from __future__ import annotations
import math, heapq, time, random
from typing import Dict, List, Tuple, Optional, Set
from collections import defaultdict

try:
    import networkx as nx
except ImportError:
    nx = None

from pkl_loader import PklMapGraph


# ── SIPP node ────────────────────────────────────────────────────────────────

class SIPPNode:
    __slots__ = ('state', 'g', 'h', 'f', 'time', 'interval')

    def __init__(self, state: str, g: float, time: float,
                 interval: Tuple[float, float]):
        self.state    = state
        self.g        = g
        self.time     = time
        self.interval = interval
        self.h        = 0.0
        self.f        = g

    def __lt__(self, other):
        return self.f < other.f


# ── Interval helpers ─────────────────────────────────────────────────────────

def _merge_intervals(intervals):
    if not intervals:
        return []
    intervals = sorted(intervals)
    merged = [intervals[0]]
    for s, e in intervals[1:]:
        if s <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], e))
        else:
            merged.append((s, e))
    return merged


def _safe_intervals(t_lo, t_hi, blocked):
    merged = _merge_intervals(blocked)
    result = []
    cur = t_lo
    for bs, be in merged:
        if bs > cur:
            result.append((cur, bs))
        cur = max(cur, be)
    if cur < t_hi:
        result.append((cur, t_hi))
    return result


# ── Heuristic (reverse Dijkstra on state graph) ─────────────────────────────

def _build_state_heuristic(graph: PklMapGraph, goal_node: str) -> Dict[str, float]:
    """Reverse Dijkstra from any S,goal_node,* to all reachable states."""
    # Build reverse adjacency: for each state, who can transition TO it?
    radj: Dict[str, List[Tuple[str, float]]] = defaultdict(list)

    for sid, s in graph.stop_states_raw.items():
        for ns in s.next_state:
            # Stop → Move: cost of the stop state (0 for stops)
            radj[ns].append((sid, s.cost))
    for sid, s in graph.move_states_raw.items():
        for ns in s.next_state:
            # Move → Stop: cost of the move state
            radj[ns].append((sid, s.cost))

    # Initialize from all stop states at goal_node
    dist: Dict[str, float] = {}
    heap = []
    for sid in graph.stop_states_raw:
        parts = sid.split(',')
        if parts[1] == goal_node:
            dist[sid] = 0.0
            heapq.heappush(heap, (0.0, sid))

    while heap:
        d, cur = heapq.heappop(heap)
        if d > dist.get(cur, float('inf')):
            continue
        for prev_sid, cost in radj.get(cur, []):
            nd = d + cost
            if nd < dist.get(prev_sid, float('inf')):
                dist[prev_sid] = nd
                heapq.heappush(heap, (nd, prev_sid))

    return dist


# ── Planner ──────────────────────────────────────────────────────────────────

class PklPrioritizedPlanner:
    """Prioritized SIPP planner operating on PklMapGraph state graph."""

    def __init__(self, graph: PklMapGraph):
        self.graph = graph
        self._port_nodes: List[str] = list(set(graph.ports.values()))
        self._h_cache: Dict[str, Dict[str, float]] = {}

    def _heuristic(self, goal_node: str) -> Dict[str, float]:
        if goal_node not in self._h_cache:
            self._h_cache[goal_node] = _build_state_heuristic(self.graph, goal_node)
        return self._h_cache[goal_node]

    # ── State lookup ─────────────────────────────────────────────────────────

    def _get_state(self, state_id: str):
        if state_id.startswith('S,'):
            return self.graph.stop_states_raw.get(state_id)
        elif state_id.startswith('M,') or state_id.startswith('R,'):
            return self.graph.move_states_raw.get(state_id)
        return None

    def _node_from_state(self, state_id: str) -> str:
        parts = state_id.split(',')
        if parts[0] == 'S':
            return parts[1]
        elif parts[0] == 'M':
            return parts[2]
        return parts[1]

    def _is_stop_at_node(self, state_id: str, node_id: str) -> bool:
        """Check if state_id is a Stop state at the given node."""
        parts = state_id.split(',')
        return parts[0] == 'S' and parts[1] == node_id

    def _find_stop_state(self, node_id: str) -> Optional[str]:
        """Find any stop state at node_id (prefer heading 0)."""
        best = None
        for sid in self.graph.stop_states_raw:
            parts = sid.split(',')
            if parts[1] == node_id:
                if parts[2] == '0':
                    return sid
                if best is None:
                    best = sid
        return best

    # ── Constraint building ──────────────────────────────────────────────────

    def _build_constraints(self, planned_path: List[Tuple[str, float]],
                           agent_id: int) -> List[dict]:
        constraints = []
        for idx, (state_id, t_start) in enumerate(planned_path):
            state = self._get_state(state_id)
            if state is None:
                continue

            t_end = planned_path[idx + 1][1] if idx < len(planned_path) - 1 else float('inf')
            if t_start == t_end and idx < len(planned_path) - 1:
                continue

            # Block the state itself
            constraints.append({
                'agent': agent_id, 'loc': state_id,
                'timestep': (t_start, t_end),
            })

            # Block all affect_states
            for aff_id in state.affect_state:
                aff = self._get_state(aff_id)
                aff_cost = aff.cost if aff else 0.0
                c_start = max(0.0, t_start - aff_cost)
                constraints.append({
                    'agent': agent_id, 'loc': aff_id,
                    'timestep': (c_start, t_end),
                })

        return constraints

    def _make_constraint_table(self, all_constraints, exclude_agent):
        table: Dict[str, List[Tuple[float, float]]] = {}
        for c in all_constraints:
            if c['agent'] == exclude_agent:
                continue
            loc = c['loc']
            if loc not in table:
                table[loc] = []
            table[loc].append(c['timestep'])
        return table

    # ── SIPP search on state graph ───────────────────────────────────────────

    def _get_successors(self, state_id: str, cur_time: float,
                        interval: Tuple[float, float],
                        goal_node: str,
                        c_table: Dict[str, List]) -> List[SIPPNode]:
        """Automod SIPP 방식: 부모 interval 하나에 대해서만 successor 생성."""
        state = self._get_state(state_id)
        if state is None:
            return []

        successors = []
        cost = state.cost
        start_t = cur_time + cost   # S: cost=0, M/R: traversal time
        end_t = interval[1]

        # 현재 상태의 노드 추출 (같은 노드 내 전이 판별용)
        cur_node = self._node_of_state(state_id)

        for neighbor in state.next_state:
            # goal node면 end_t를 inf로 (목적지에서 무한 대기 가능)
            local_end = float('inf') if self._is_stop_at_node(neighbor, goal_node) else end_t

            # M→S, R→S 전이: 이미 해당 노드에 도착/존재하므로 arrive 지연 불가
            # safe interval이 start_t를 포함하지 않으면 사용 불가
            # S→M, S→R 전이: S에서 대기 후 출발 가능하므로 제약 없음
            must_arrive_now = (state_id.startswith(('M,', 'R,'))
                               and neighbor.startswith('S,'))

            # neighbor의 safe intervals: 부모 interval 범위 내에서 계산
            blocked = c_table.get(neighbor, [])
            safe = _safe_intervals(interval[0], local_end, blocked)

            for si in safe:
                if si[0] > local_end or si[1] <= start_t:
                    continue
                arrive = max(start_t, si[0])

                # M→S, R→S: 도착 시점을 늦출 수 없음
                if must_arrive_now and arrive > start_t + 1e-6:
                    continue

                successors.append(SIPPNode(neighbor, arrive, arrive, si))

        return successors

    @staticmethod
    def _node_of_state(state_id: str) -> Optional[str]:
        """state_id에서 노드 ID 추출. S,node,angle → node / R,node,a,b → node / M,from,to → None"""
        parts = state_id.split(',')
        if parts[0] in ('S', 'R') and len(parts) >= 2:
            return parts[1]
        return None  # M 상태는 두 노드 간 이동이므로 same-node 판별 대상 아님

    def _is_goal_constrained(self, state_id, t, c_table):
        for ts, te in c_table.get(state_id, []):
            if te > t:
                return True
        return False

    def _sipp_search(self, start_node: str, goal_node: str,
                     c_table: Dict, start_time: float = 0.0,
                     timeout: float = 10.0) -> Optional[List[Tuple[str, float]]]:
        """SIPP A* on state graph from start_node to goal_node."""
        t0 = time.time()
        h_table = self._heuristic(goal_node)

        # Find best start stop state at start_node
        start_sid = self._find_stop_state(start_node)
        if start_sid is None:
            return None

        blocked = c_table.get(start_sid, [])
        safe = _safe_intervals(start_time, float('inf'), blocked)
        if not safe:
            return None
        init_interval = safe[0]
        actual_start = max(start_time, init_interval[0])

        s0 = SIPPNode(start_sid, actual_start, actual_start, init_interval)
        s0.h = h_table.get(start_sid, float('inf'))
        s0.f = s0.g + s0.h

        open_list = [s0]
        came_from = {(start_sid, init_interval): None}
        cost_so_far = {(start_sid, init_interval): s0.g}

        while open_list:
            if time.time() - t0 > timeout:
                return None

            current = heapq.heappop(open_list)
            key = (current.state, current.interval)

            # Goal check: any Stop state at goal_node, unconstrained
            if self._is_stop_at_node(current.state, goal_node):
                if not self._is_goal_constrained(current.state, current.time, c_table):
                    return self._reconstruct(came_from, cost_so_far, key)

            if current.g > cost_so_far.get(key, float('inf')):
                continue

            succs = self._get_successors(current.state, current.time,
                                         current.interval, goal_node, c_table)

            for s in succs:
                sk = (s.state, s.interval)
                g_new = s.time
                h_val = h_table.get(s.state, float('inf'))
                f_val = g_new + h_val

                if sk not in cost_so_far or g_new < cost_so_far[sk]:
                    cost_so_far[sk] = g_new
                    came_from[sk] = key
                    s.g = g_new
                    s.h = h_val
                    s.f = f_val
                    heapq.heappush(open_list, s)

        return None

    def _reconstruct(self, came_from, cost_so_far, goal_key):
        path = []
        cur = goal_key
        while cur is not None:
            state_id = cur[0]
            t = cost_so_far[cur]
            path.append((state_id, t))
            cur = came_from.get(cur)
        path.reverse()
        return path

    # ── Dependency graph ─────────────────────────────────────────────────────

    def _build_dependency_graph(self, agent_nodes, agent_goals):
        G = nx.DiGraph()
        for aid in agent_nodes:
            G.add_node(aid)
        for i, goal_i in agent_goals.items():
            for j, cur_j in agent_nodes.items():
                if i != j and goal_i == cur_j:
                    G.add_edge(i, j)
        return G

    # ── Path utilities ───────────────────────────────────────────────────────

    @staticmethod
    def path_to_nodes(state_path: List[Tuple[str, float]]) -> List[str]:
        nodes = []
        for state_id, _ in state_path:
            parts = state_id.split(',')
            if parts[0] == 'S':
                nid = parts[1]
            elif parts[0] == 'M':
                nid = parts[2]
            else:
                continue
            if not nodes or nodes[-1] != nid:
                nodes.append(nid)
        return nodes

    # ── Main planning interface ──────────────────────────────────────────────

    def plan(self, agent_positions: Dict[int, str],
             agent_goals: Dict[int, str],
             existing_constraints: Optional[List[dict]] = None,
             start_times: Optional[Dict[int, float]] = None,
             timeout_per_agent: float = 10.0) -> Optional['PklPlanResult']:
        """
        Plan conflict-free paths for all agents.

        Parameters
        ----------
        agent_positions : {agent_id: node_id}
        agent_goals     : {agent_id: node_id}  (port destinations)

        Returns
        -------
        PklPlanResult with .paths (state-level) and .node_paths
        """
        if nx is None:
            raise ImportError("networkx is required")

        all_constraints = list(existing_constraints or [])
        agent_nodes = dict(agent_positions)
        goals = dict(agent_goals)
        starts = dict(start_times or {})

        # Validate goals
        for aid, goal in list(goals.items()):
            if goal not in self.graph.nodes:
                print(f"[WARN] Goal {goal} not in graph for agent {aid}")
                goals.pop(aid)

        at_goal = {aid for aid, g in goals.items() if agent_nodes.get(aid) == g}
        planned: Dict[int, List[Tuple[str, float]]] = {}
        remaining = set(goals.keys())

        for aid in at_goal:
            t = starts.get(aid, 0.0)
            sid = self._find_stop_state(goals[aid])
            if sid:
                planned[aid] = [(sid, t)]
                cs = self._build_constraints(planned[aid], aid)
                all_constraints.extend(cs)
            remaining.discard(aid)

        max_iter = len(remaining) * 3 + 10

        for iteration in range(max_iter):
            if not remaining:
                break

            G_dep = self._build_dependency_graph(agent_nodes, goals)

            # Phase 1: Z set (out-degree 0, not yet planned)
            Z = [a for a in G_dep.nodes
                 if G_dep.out_degree(a) == 0 and a in remaining]

            best_aid, best_goal = None, None
            best_dist = float('inf')

            for aid in Z:
                goal_n = goals[aid]
                cur_n  = agent_nodes[aid]
                h = self._heuristic(goal_n)
                start_sid = self._find_stop_state(cur_n)
                dist = h.get(start_sid, float('inf')) if start_sid else float('inf')
                if dist < best_dist:
                    best_dist = dist
                    best_aid  = aid
                    best_goal = goal_n

            # Phase 2: blockers
            if best_aid is None:
                planned_set = set(planned.keys())
                blockers = [a for a in G_dep.nodes
                            if G_dep.in_degree(a) > 0 and a in planned_set]
                empty_ports = self._find_empty_ports(agent_nodes, goals)
                if blockers and empty_ports:
                    for aid in blockers:
                        cur_n = agent_nodes[aid]
                        start_sid = self._find_stop_state(cur_n)
                        for port in empty_ports:
                            h = self._heuristic(port)
                            dist = h.get(start_sid, float('inf')) if start_sid else float('inf')
                            if dist < best_dist:
                                best_dist = dist
                                best_aid  = aid
                                best_goal = port

            # Phase 3: cycle breaking
            if best_aid is None:
                cycles = list(nx.simple_cycles(G_dep))
                if not cycles:
                    break
                empty_ports = self._find_empty_ports(agent_nodes, goals)
                if not empty_ports:
                    break
                cyc_agents = [a for cyc in cycles for a in cyc]
                planned_set = set(planned.keys())
                prefer = [a for a in cyc_agents if a in planned_set] or cyc_agents
                for aid in prefer:
                    cur_n = agent_nodes[aid]
                    start_sid = self._find_stop_state(cur_n)
                    for port in empty_ports:
                        h = self._heuristic(port)
                        dist = h.get(start_sid, float('inf')) if start_sid else float('inf')
                        if dist < best_dist:
                            best_dist = dist
                            best_aid  = aid
                            best_goal = port

            if best_aid is None:
                break

            # ── SIPP search ──────────────────────────────────────────────────
            c_table = self._make_constraint_table(all_constraints, best_aid)
            t_start = starts.get(best_aid, 0.0)
            if best_aid in planned:
                t_start = planned[best_aid][-1][1]

            path = self._sipp_search(
                agent_nodes[best_aid], best_goal, c_table,
                start_time=t_start, timeout=timeout_per_agent
            )

            if path is None:
                print(f"[FAIL] SIPP failed: agent {best_aid}, "
                      f"{agent_nodes[best_aid]} → {best_goal}")
                # Fallback: try alternate port
                alt = self._find_alternate_goal(
                    best_aid, agent_nodes[best_aid], best_goal,
                    c_table, t_start, timeout_per_agent,
                    agent_nodes, goals)
                if alt is None:
                    break
                path, best_goal = alt

            # Accumulate path
            if best_aid in planned:
                planned[best_aid] = planned[best_aid] + path[1:]
            else:
                planned[best_aid] = path

            # Update constraints
            new_cs = self._build_constraints(planned[best_aid], best_aid)
            all_constraints = [c for c in all_constraints if c['agent'] != best_aid]
            all_constraints.extend(new_cs)

            agent_nodes[best_aid] = best_goal
            if best_goal == goals.get(best_aid):
                remaining.discard(best_aid)

        result = PklPlanResult()
        for aid, path in planned.items():
            result.paths[aid] = path
            result.node_paths[aid] = self.path_to_nodes(path)
        return result

    def _find_alternate_goal(self, aid, cur_node, failed_goal, c_table,
                             t_start, timeout, agent_nodes, agent_goals):
        """Try alternate port destinations when SIPP fails."""
        occupied = set(agent_nodes.values())
        targeted = set(agent_goals.values())
        used = occupied | targeted | {failed_goal}
        for port in self._port_nodes:
            if port in used:
                continue
            path = self._sipp_search(cur_node, port, c_table,
                                     start_time=t_start, timeout=timeout/2)
            if path is not None:
                return path, port
        return None

    def _find_empty_ports(self, agent_nodes, agent_goals):
        occupied = set(agent_nodes.values())
        targeted = set(agent_goals.values())
        used = occupied | targeted
        return [p for p in self._port_nodes if p not in used]

    def assign_random_goals(self, agent_positions: Dict[int, str]) -> Dict[int, str]:
        goals = {}
        used = set()
        ports = list(self._port_nodes)
        random.shuffle(ports)
        for aid, cur in agent_positions.items():
            for p in ports:
                if p != cur and p not in used:
                    goals[aid] = p
                    used.add(p)
                    break
            else:
                for p in ports:
                    if p != cur:
                        goals[aid] = p
                        break
        return goals


class PklPlanResult:
    def __init__(self):
        self.paths:      Dict[int, List[Tuple[str, float]]] = {}
        self.node_paths: Dict[int, List[str]] = {}

    @property
    def makespan(self) -> float:
        if not self.paths:
            return 0.0
        return max(p[-1][1] for p in self.paths.values() if p)
