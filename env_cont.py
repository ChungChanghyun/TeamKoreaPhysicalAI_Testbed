"""
env_cont.py — Continuous-time kinematic environment.

Agent state: (x, y, theta, v) — all floats, mm / radians / seconds.

Path planning
─────────────
build_timed_path(node_path, graph, init_theta, t0)
  1. For each consecutive edge pair, check if headings match.
     • Same heading  → pass-through junction (v > 0)
     • Heading change → stop junction (v = 0, rotate first)
  2. compute_junction_velocities() runs a forward-backward feasibility pass
     so that every junction speed is reachable given edge lengths and A_MAX.
  3. Each edge is planned with a generalized trapezoidal profile (v0 → vf).

Maneuver types
──────────────
WaitManeuver   — stationary at a node
RotateManeuver — in-place rotation at a node (v = 0)
MoveManeuver   — traverse an edge with a (v0→vf) trapezoidal profile
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Tuple, Union

from map_loader import MapGraph

# ── Motion constants ───────────────────────────────────────────────────────────
A_MAX     = 500.0       # mm/s²   linear accel / decel
OMEGA_MAX = math.pi     # rad/s   rotation speed  (≈ 180°/s)

AGENT_COLORS = [
    ( 70, 130, 180), (220,  80,  80), ( 80, 200, 100),
    (210, 170,  50), (170,  90, 210), ( 60, 200, 200),
    (230, 130,  50), (200,  80, 150),
]

IDLE   = 'IDLE'
MOVING = 'MOVING'


# ── Angle helpers ──────────────────────────────────────────────────────────────

def angle_diff(a: float, b: float) -> float:
    """Signed shortest difference a − b in (−π, π]."""
    d = (a - b) % (2 * math.pi)
    if d > math.pi:
        d -= 2 * math.pi
    return d


def angle_lerp(a: float, b: float, t: float) -> float:
    """Shortest-path interpolation from a toward b by fraction t ∈ [0,1]."""
    return a + angle_diff(b, a) * max(0.0, min(1.0, t))


# ── Generalized trapezoidal velocity profile ───────────────────────────────────

def trapezoid_params(L: float, v0: float, vf: float,
                     v_max: float, a: float) -> Tuple:
    """
    Plan movement over distance L, starting at v0 and ending at vf,
    with maximum speed v_max and acceleration magnitude a.

    The profile has up to three phases:
      Phase 1 – accelerate from v0 to v_peak  (duration t1)
      Phase 2 – cruise at v_peak              (duration t_c)
      Phase 3 – decelerate from v_peak to vf  (duration t3)

    Returns (v_peak, t1, t_c, t3, t_total).
    """
    v0    = max(0.0, min(v0, v_max))
    vf    = max(0.0, min(vf, v_max))

    # Distance needed to accel from v0 to v_max, then decel to vf
    d1_full = (v_max ** 2 - v0 ** 2) / (2.0 * a)
    d3_full = (v_max ** 2 - vf ** 2) / (2.0 * a)

    if d1_full + d3_full <= L:          # ── Trapezoid: can reach v_max ────
        t1    = (v_max - v0) / a
        t3    = (v_max - vf) / a
        t_c   = (L - d1_full - d3_full) / v_max
        return v_max, t1, t_c, t3, t1 + t_c + t3

    # ── Triangle: cannot reach v_max, find v_peak ─────────────────────────
    # Solve:  (v_peak²−v0²)/(2a)  +  (v_peak²−vf²)/(2a)  = L
    #  →  v_peak² = a·L + (v0²+vf²)/2
    v_peak_sq = a * L + (v0 ** 2 + vf ** 2) / 2.0
    v_peak    = math.sqrt(max(0.0, v_peak_sq))
    v_peak    = min(v_peak, v_max)

    # Guard: peak must be ≥ both endpoints
    # (can be violated only if L is smaller than the minimum braking distance)
    if v_peak < v0 or v_peak < vf:
        # Physically tight — just run the best straight-line approximation
        v_peak = max(v0, vf)

    t1 = (v_peak - v0) / a if v_peak > v0 else 0.0
    t3 = (v_peak - vf) / a if v_peak > vf else 0.0
    return v_peak, t1, 0.0, t3, t1 + t3


def trapezoid_state(dt: float, L: float,
                    v0: float, vf: float,
                    v_peak: float, t1: float,
                    t_c: float, t3: float,
                    t_total: float, a: float) -> Tuple[float, float]:
    """
    Returns (distance_from_start, velocity) at time dt into the maneuver.
    Works for any combination of v0, vf (including non-zero).
    """
    if dt <= 0.0:
        return 0.0, v0
    if dt >= t_total:
        return L, vf

    # Phase 1 — accelerate to v_peak
    if dt <= t1:
        d = v0 * dt + 0.5 * a * dt ** 2
        v = v0 + a * dt
        return d, v

    d1 = v0 * t1 + 0.5 * a * t1 ** 2  if t1 > 0 else 0.0

    # Phase 2 — cruise
    t2_end = t1 + t_c
    if dt <= t2_end:
        d = d1 + v_peak * (dt - t1)
        return d, v_peak

    d2 = d1 + v_peak * t_c

    # Phase 3 — decelerate to vf
    td = dt - t2_end
    d  = d2 + v_peak * td - 0.5 * a * td ** 2
    v  = max(vf, v_peak - a * td)
    return d, v


# ── Junction velocity planner ──────────────────────────────────────────────────

def compute_junction_velocities(node_path: List[str],
                                graph: MapGraph,
                                init_theta: float) -> List[float]:
    """
    Returns a list of velocities at each node in node_path.

    Rules
    ─────
    • First node  → v = 0 (agent starts from rest)
    • Last node   → v = 0 (agent must stop)
    • Intermediate node:
        – heading changes  → v = 0  (must stop to rotate)
        – heading same     → v = min(edge1.max_speed, edge2.max_speed)
    After setting desired speeds, a forward and backward feasibility pass
    ensures every junction speed is achievable given edge lengths and A_MAX.
    """
    n = len(node_path)
    v = [0.0] * n   # junction velocities

    # ── Desired pass-through speed ──────────────────────────────────────────
    for i in range(1, n - 1):
        e_in  = graph.get_edge(node_path[i - 1], node_path[i])
        e_out = graph.get_edge(node_path[i],     node_path[i + 1])
        if e_in is None or e_out is None:
            continue
        if abs(angle_diff(e_out.angle, e_in.angle)) < 1e-4:   # same heading
            v[i] = min(e_in.max_speed, e_out.max_speed)
        # else v[i] remains 0

    # ── Forward pass: limit by what is reachable from v[i-1] ───────────────
    for i in range(1, n):
        edge = graph.get_edge(node_path[i - 1], node_path[i])
        if edge is None:
            continue
        # Maximum speed achievable at end of edge given entry v[i-1]
        v_max_fwd = math.sqrt(v[i - 1] ** 2 + 2.0 * A_MAX * edge.length)
        v[i] = min(v[i], v_max_fwd, edge.max_speed)

    # ── Backward pass: limit by what allows decelerating to v[i+1] ─────────
    for i in range(n - 2, -1, -1):
        edge = graph.get_edge(node_path[i], node_path[i + 1])
        if edge is None:
            continue
        v_max_bwd = math.sqrt(v[i + 1] ** 2 + 2.0 * A_MAX * edge.length)
        v[i] = min(v[i], v_max_bwd)

    # First node is always 0
    v[0] = 0.0
    return v


# ── Maneuvers ──────────────────────────────────────────────────────────────────

@dataclass
class WaitManeuver:
    """Agent is stationary at a node."""
    node_id : str
    x       : float
    y       : float
    theta   : float
    t_start : float
    t_end   : float

    def state_at(self, t: float) -> Tuple[float, float, float, float]:
        return self.x, self.y, self.theta, 0.0


@dataclass
class RotateManeuver:
    """Agent turns in place at a node (v = 0 throughout)."""
    node_id     : str
    x           : float
    y           : float
    theta_start : float
    theta_end   : float
    t_start     : float
    t_end       : float

    def state_at(self, t: float) -> Tuple[float, float, float, float]:
        if self.t_end <= self.t_start:
            return self.x, self.y, self.theta_end, 0.0
        frac  = (t - self.t_start) / (self.t_end - self.t_start)
        theta = angle_lerp(self.theta_start, self.theta_end, frac)
        return self.x, self.y, theta, 0.0


@dataclass
class MoveManeuver:
    """
    Agent traverses an edge.  Supports non-zero entry (v0) and exit (vf)
    velocities for pass-through nodes.
    """
    from_id  : str
    to_id    : str
    x0       : float
    y0       : float
    x1       : float
    y1       : float
    theta    : float          # heading (constant during move)
    length   : float          # mm
    t_start  : float
    t_end    : float
    v0       : float = 0.0    # entry velocity
    vf       : float = 0.0    # exit velocity
    v_peak   : float = 0.0    # peak speed in profile
    t1       : float = 0.0    # accel duration
    t_c      : float = 0.0    # cruise duration
    t3       : float = 0.0    # decel duration
    a        : float = A_MAX

    def state_at(self, t: float) -> Tuple[float, float, float, float]:
        dt = t - self.t_start
        T  = self.t_end - self.t_start
        if T <= 0.0 or self.length <= 0.0:
            return self.x1, self.y1, self.theta, self.vf
        dist, vel = trapezoid_state(
            dt, self.length,
            self.v0, self.vf,
            self.v_peak, self.t1, self.t_c, self.t3, T,
            self.a,
        )
        frac = max(0.0, min(1.0, dist / self.length))
        x = self.x0 + (self.x1 - self.x0) * frac
        y = self.y0 + (self.y1 - self.y0) * frac
        return x, y, self.theta, vel


Maneuver = Union[WaitManeuver, RotateManeuver, MoveManeuver]


# ── Path builder ───────────────────────────────────────────────────────────────

def build_timed_path(node_path: List[str], graph: MapGraph,
                     init_theta: float, t0: float = 0.0) -> List:
    """
    Convert an ordered list of node IDs into a timestamped list of Maneuvers.

    Junction velocities are computed by compute_junction_velocities(), which
    allows agents to pass through intermediate nodes without stopping when the
    edge heading does not change.
    """
    if len(node_path) < 1:
        return []

    junction_v = compute_junction_velocities(node_path, graph, init_theta)

    maneuvers: List = []
    t     = t0
    theta = init_theta
    first = graph.nodes[node_path[0]]
    maneuvers.append(WaitManeuver(node_path[0], first.x, first.y, theta, t, t))

    for i in range(1, len(node_path)):
        prev_id = node_path[i - 1]
        curr_id = node_path[i]
        edge    = graph.get_edge(prev_id, curr_id)
        if edge is None:
            continue

        prev_n = graph.nodes[prev_id]
        curr_n = graph.nodes[curr_id]
        v0     = junction_v[i - 1]
        vf     = junction_v[i]

        # ── Rotate if heading changes (only when v0 == 0) ─────────────────
        edge_theta = edge.angle
        d_angle    = abs(angle_diff(edge_theta, theta))
        if d_angle > 1e-4:
            t_rot = d_angle / OMEGA_MAX
            maneuvers.append(RotateManeuver(
                prev_id, prev_n.x, prev_n.y,
                theta, edge_theta,
                t, t + t_rot,
            ))
            t    += t_rot
            theta = edge_theta

        # ── Move along edge ───────────────────────────────────────────────
        v_peak, t1, t_c, t3, t_move = trapezoid_params(
            edge.length, v0, vf, edge.max_speed, A_MAX
        )
        maneuvers.append(MoveManeuver(
            prev_id, curr_id,
            prev_n.x, prev_n.y,
            curr_n.x, curr_n.y,
            edge_theta, edge.length,
            t, t + t_move,
            v0=v0, vf=vf,
            v_peak=v_peak, t1=t1, t_c=t_c, t3=t3, a=A_MAX,
        ))
        t    += t_move
        theta = edge_theta

    return maneuvers


# ── Kinematic Agent ────────────────────────────────────────────────────────────

class KinematicAgent:
    """
    Continuous-time kinematic agent following a planned path on a MapGraph.
    Coordinates in mm, angles in radians, time in seconds.
    """

    def __init__(self, agent_id: int, start_node: str, graph: MapGraph,
                 start_theta: float = 0.0):
        self.id    = agent_id
        self.color = AGENT_COLORS[agent_id % len(AGENT_COLORS)]
        self.graph = graph

        node       = graph.nodes[start_node]
        self.x     = node.x
        self.y     = node.y
        self.theta = start_theta
        self.v     = 0.0

        self.cur_node  = start_node
        self.status    = IDLE
        self.node_path = [start_node]

        self._maneuvers  = []
        self._man_idx    = 0
        self._path_end_t = 0.0

    # ── Path assignment ───────────────────────────────────────────────────────

    def set_node_path(self, node_path: List[str], t_start: float = 0.0):
        """Assign a path as an ordered list of node IDs."""
        self.node_path   = node_path
        self._maneuvers  = build_timed_path(
            node_path, self.graph, self.theta, t_start
        )
        self._man_idx    = 0
        self._path_end_t = (self._maneuvers[-1].t_end
                            if self._maneuvers else t_start)
        self.status      = MOVING if len(node_path) > 1 else IDLE

    def set_plan(self, plan: list):
        """
        Accept a plan from the algorithm as a list of (state, t_depart) pairs.

        Each state is a StopState / MoveState / RotateState from state_graph.py.
        t_depart is when the agent STARTS executing that state (from the planner).

        The agent physically executes each state with the kinematic model,
        departing at t_depart.  Actual arrival time is determined by physics
        (may differ slightly from the planner's fixed-cost estimate).

        Example plan format
        ───────────────────
        plan = [
            (stop_state_A,  t=0.0),    # wait at A
            (rotate_A_0_90, t=0.0),    # rotate to face edge direction
            (move_A_B,      t=0.5),    # depart A at t=0.5
            (stop_state_B,  t=6.0),    # arrive and wait at B
        ]
        """
        # Import here to avoid circular import at module level
        from state_graph import StopState, MoveState, RotateState

        maneuvers: List = []
        node_path: List[str] = []

        for i, (state, t_dep) in enumerate(plan):
            # Duration: time until next state departs (or use physical cost)
            if i + 1 < len(plan):
                t_end_plan = plan[i + 1][1]
            else:
                t_end_plan = t_dep + getattr(state, 'cost', 0.0)

            if isinstance(state, StopState):
                node_path.append(state.node_id)
                maneuvers.append(WaitManeuver(
                    state.node_id, state.x, state.y, state.theta,
                    t_dep, t_end_plan,
                ))

            elif isinstance(state, RotateState):
                node_path.append(state.node_id)
                # Physical rotation time (may differ from plan slot)
                t_rot = t_dep + state.cost
                maneuvers.append(RotateManeuver(
                    state.node_id, state.x, state.y,
                    state.theta_start, state.theta_end,
                    t_dep, t_rot,
                ))

            elif isinstance(state, MoveState):
                fn = self.graph.nodes[state.from_id]
                tn = self.graph.nodes[state.to_id]
                node_path.append(state.from_id)
                # Kinematic execution: full stop-start (v0=0, vf=0)
                v_peak, t1, t_c, t3, t_move = trapezoid_params(
                    state.length, 0.0, 0.0, state.max_speed, A_MAX
                )
                maneuvers.append(MoveManeuver(
                    state.from_id, state.to_id,
                    fn.x, fn.y, tn.x, tn.y,
                    state.theta, state.length,
                    t_dep, t_dep + t_move,       # physical end time
                    v0=0.0, vf=0.0,
                    v_peak=v_peak, t1=t1, t_c=t_c, t3=t3, a=A_MAX,
                ))

        if node_path:
            node_path.append(
                plan[-1][0].node_id
                if hasattr(plan[-1][0], 'node_id')
                else plan[-1][0].to_id
            )
        self.node_path   = node_path
        self._maneuvers  = maneuvers
        self._man_idx    = 0
        self._path_end_t = maneuvers[-1].t_end if maneuvers else 0.0
        self.status      = MOVING if maneuvers else IDLE

    def path_end_time(self) -> float:
        return self._path_end_t

    # ── Update ────────────────────────────────────────────────────────────────

    def update(self, sim_time: float):
        """Set agent state to that corresponding to sim_time."""
        if not self._maneuvers:
            return

        while (self._man_idx < len(self._maneuvers) - 1
               and sim_time > self._maneuvers[self._man_idx].t_end):
            self._man_idx += 1

        m         = self._maneuvers[self._man_idx]
        t_clamped = max(m.t_start, min(sim_time, m.t_end))
        self.x, self.y, self.theta, self.v = m.state_at(t_clamped)

        if isinstance(m, MoveManeuver):
            self.cur_node = m.to_id if sim_time >= m.t_end else m.from_id
        else:
            self.cur_node = m.node_id

        if sim_time >= self._path_end_t:
            self.status = IDLE
            self.v      = 0.0

    # ── Geometry ──────────────────────────────────────────────────────────────

    def footprint_corners(self, width_mm: float,
                          length_mm: float) -> List[Tuple[float, float]]:
        """Four corners of the agent rectangle in map-space mm."""
        ct = math.cos(self.theta)
        st = math.sin(self.theta)
        hl, hw = length_mm / 2.0, width_mm / 2.0
        return [
            (self.x + fl * ct - fw * st,
             self.y + fl * st + fw * ct)
            for fl, fw in [(-hl, -hw), (-hl, hw), (hl, hw), (hl, -hw)]
        ]

    # ── Info ──────────────────────────────────────────────────────────────────

    def maneuver_type(self) -> str:
        if not self._maneuvers:
            return 'none'
        return type(self._maneuvers[self._man_idx]).__name__.replace('Maneuver', '').lower()

    def __repr__(self):
        return (f'KinematicAgent({self.id}, '
                f'({self.x/1000:.2f},{self.y/1000:.2f})m, '
                f'v={self.v/1000:.2f}m/s, {self.status})')
