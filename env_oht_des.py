"""
env_oht_des.py — OHT front-detection DES simulation engine.

Collision avoidance: Automod-style analytical catch-up scheduling.
Each directed segment maintains a front→back queue.
When a follower is faster than the leader, a CATCH_UP event is scheduled
at t = now + (gap - h_min) / (v_follower - v_leader).
Token-based invalidation silently discards stale events.

ZCU (Zone Control Unit): merge/diverge nodes where only one OHT may pass
at a time.  An agent acquires the ZCU lock when it enters the entry segment;
it releases the lock when it completes the exit segment.  Agents blocked on a
ZCU wait in a per-zone waitlist (no timer retry).

Agent states
────────────
  IDLE      : waiting to be assigned to a segment
  MOVING    : traversing a segment at full speed
  FOLLOWING : speed-limited behind a slower/stopped leader
  BLOCKED   : queued at segment entry (no room or ZCU occupied)

Events
──────
  TRY_ADVANCE   : agent tries to enter the next segment in its path
  SEGMENT_DONE  : agent reaches end node of current segment
  CATCH_UP      : follower would catch the leader — must slow to match speed
  GAP_CLEAR     : leader sped up or left, follower may resume full speed
"""
from __future__ import annotations
import math, heapq, collections
from typing import Dict, List, Optional, Tuple

# ── Agent states ──────────────────────────────────────────────────────────────
IDLE      = 'IDLE'
MOVING    = 'MOVING'
FOLLOWING = 'FOLLOWING'
BLOCKED   = 'BLOCKED'
DONE      = 'DONE'

# ── Event types ───────────────────────────────────────────────────────────────
TRY_ADVANCE  = 'TRY_ADVANCE'
SEGMENT_DONE = 'SEGMENT_DONE'
CATCH_UP     = 'CATCH_UP'
GAP_CLEAR    = 'GAP_CLEAR'


# ── Map data container ────────────────────────────────────────────────────────

class OHTNode:
    def __init__(self, nid: str, x: float, y: float):
        self.id = nid
        self.x  = x
        self.y  = y


class ZCUZone:
    """
    Immutable definition of a ZCU (Zone Control Unit) zone.
    One vehicle at a time is allowed through the zone.
    entry_segs : set of (from_id, to_id) — acquiring the lock
    exit_segs  : set of (from_id, to_id) — releasing the lock
    """
    def __init__(self, zone_id: str,
                 entry_segs: frozenset, exit_segs: frozenset):
        self.id         = zone_id
        self.entry_segs = entry_segs
        self.exit_segs  = exit_segs


def _build_path_points(parts: list, start_xy: Tuple[float, float],
                       n_arc: int = 16) -> List[Tuple[float, float]]:
    """
    Convert segment parts (Straight / Arc) into a dense polyline.
    Arc format: quadratic Bézier — [control_point, end_point].
    Returns list of (x, y) including start and end.
    """
    pts: List[Tuple[float, float]] = [start_xy]
    cur = start_xy
    for part in parts:
        raw = [(p['x'], p['y']) for p in part['points']]
        if part['kind'] == 'Straight':
            pts.append(raw[0])
            cur = raw[0]
        elif part['kind'] == 'Arc':
            p0, p1, p2 = cur, raw[0], raw[1]
            for i in range(1, n_arc + 1):
                t = i / n_arc
                x = (1-t)**2 * p0[0] + 2*t*(1-t) * p1[0] + t**2 * p2[0]
                y = (1-t)**2 * p0[1] + 2*t*(1-t) * p1[1] + t**2 * p2[1]
                pts.append((x, y))
            cur = p2
    return pts


def _polyline_length(pts: List[Tuple[float, float]]) -> float:
    return sum(math.hypot(pts[i+1][0] - pts[i][0],
                          pts[i+1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def _interp_path(pts: List[Tuple[float, float]],
                 pos_mm: float) -> Tuple[float, float, float]:
    """
    Given a polyline and a distance along it, return (x, y, theta).
    theta is the heading of the current sub-segment (map-space, y-up).
    """
    remaining = pos_mm
    for i in range(len(pts) - 1):
        p1, p2 = pts[i], pts[i + 1]
        seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if remaining <= seg_len or i == len(pts) - 2:
            t = (remaining / seg_len) if seg_len > 0 else 1.0
            t = min(t, 1.0)
            x = p1[0] + (p2[0] - p1[0]) * t
            y = p1[1] + (p2[1] - p1[1]) * t
            theta = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            return x, y, theta
        remaining -= seg_len
    p = pts[-1]
    theta = math.atan2(pts[-1][1] - pts[-2][1],
                       pts[-1][0] - pts[-2][0]) if len(pts) >= 2 else 0.0
    return p[0], p[1], theta


def _safe_gap(v: float, decel: float, h_min: float) -> float:
    """Dynamic following distance: h_min + braking distance at current speed."""
    return h_min + v * v / (2.0 * decel)


def _solve_brake_phase(vf: float, af: float, vl: float, al: float,
                       D: float, gap: float, h_min: float,
                       t_end: float) -> float:
    """Solve trigger condition within a single constant-acceleration phase.

    Both follower and leader have constant acceleration (af, al).
    Returns dt in [0, t_end] or inf if no trigger in this phase.

    gap(t) = gap + (vl - vf)*t + 0.5*(al - af)*t^2
    rel(t) = vf + af*t - (vl + al*t) = (vf-vl) + (af-al)*t
    trigger: gap(t) = h_min + rel(t)^2 / (2*D)   when rel(t) > 0
    """
    u  = vf - vl         # relative speed
    da = af - al          # relative acceleration

    # Already triggered?
    brake_d = u * u / (2.0 * D) if u > 0 else 0.0
    if gap <= h_min + brake_d:
        return 0.0

    # Quadratic: A*t^2 + B*t + C = 0
    # A = da*(da + D)
    # B = 2*u*(da + D)
    # C = u^2 - 2*D*(gap - h_min)
    K = da + D
    A_c = da * K
    B_c = 2.0 * u * K
    C_c = u * u - 2.0 * D * (gap - h_min)

    if abs(A_c) < 1e-12:
        # Linear: B_c*t + C_c = 0
        if abs(B_c) < 1e-12:
            return float('inf')
        dt = -C_c / B_c
        if 0 < dt <= t_end:
            # verify rel > 0 at this time
            if u + da * dt > 0:
                return dt
        return float('inf')

    disc = B_c * B_c - 4.0 * A_c * C_c
    if disc < 0:
        return float('inf')

    sq = math.sqrt(max(0.0, disc))
    for sign in [1, -1]:
        dt = (-B_c + sign * sq) / (2.0 * A_c)
        if 1e-9 < dt <= t_end:
            # verify follower is faster than leader at trigger time
            if u + da * dt > 0:
                return dt

    return float('inf')


def _accel_brake_dt(vf: float, vl: float, af: float, vf_max: float,
                    decel: float, gap: float, h_min: float,
                    al: float = 0.0, vl_max: float = 0.0) -> float:
    """
    Time until follower should start braking to maintain h_min from leader.

    Follower: speed vf, acceleration af (>=0), speed cap vf_max
    Leader  : speed vl, acceleration al (can be negative), speed cap vl_max
    gap     : current gap
    Returns dt >= 0, or inf if no braking needed.

    Decomposes into phases based on when follower/leader reach speed cap
    or when a decelerating leader stops, then solves trigger condition
    analytically in each phase.
    """
    D = decel

    # Already triggered?
    u = vf - vl
    brake_d = u * u / (2.0 * D) if u > 0 else 0.0
    if gap <= h_min + brake_d:
        return 0.0

    # Phase boundaries: when follower reaches speed cap
    if af > 1e-9 and vf < vf_max:
        tf_cap = (vf_max - vf) / af
    else:
        tf_cap = float('inf')
        af = 0.0

    # Phase boundary: when leader reaches speed cap (accelerating)
    #                 or when leader stops (decelerating)
    tl_boundary = float('inf')
    if al > 1e-9 and vl_max > 0 and vl < vl_max:
        # leader accelerating toward cap
        tl_boundary = (vl_max - vl) / al
    elif al < -1e-9 and vl > 0:
        # leader decelerating — will stop at t = -vl/al
        tl_boundary = -vl / al

    # Collect phase boundaries
    boundaries = sorted(set(t for t in [tf_cap, tl_boundary] if t < float('inf')))
    boundaries.append(float('inf'))

    t_offset = 0.0
    cur_vf, cur_vl = vf, vl
    cur_af, cur_al = af, al
    cur_gap = gap

    for t_end_abs in boundaries:
        phase_dur = t_end_abs - t_offset
        if phase_dur <= 0:
            continue

        dt = _solve_brake_phase(cur_vf, cur_af, cur_vl, cur_al,
                                D, cur_gap, h_min, phase_dur)
        if dt < float('inf'):
            return t_offset + dt

        # Advance state to end of this phase
        vf_end = cur_vf + cur_af * phase_dur
        vf_end = min(vf_end, vf_max)
        vl_end = cur_vl + cur_al * phase_dur
        vl_end = max(vl_end, 0.0)  # clamp: speed doesn't go negative
        if vl_max > 0:
            vl_end = min(vl_end, vl_max)

        f_disp = cur_vf * phase_dur + 0.5 * cur_af * phase_dur * phase_dur
        l_disp = cur_vl * phase_dur + 0.5 * cur_al * phase_dur * phase_dur
        cur_gap = cur_gap - f_disp + l_disp

        cur_vf = vf_end
        cur_vl = vl_end
        # follower: stop accelerating once at cap
        cur_af = 0.0 if cur_vf >= vf_max else af
        # leader: stop accelerating/decelerating once at cap or stopped
        if cur_al > 0 and vl_max > 0 and cur_vl >= vl_max:
            cur_al = 0.0
        elif cur_al < 0 and cur_vl <= 0:
            cur_al = 0.0  # leader stopped
        t_offset = t_end_abs

        # Both at constant speed and not closing
        if abs(cur_af) < 1e-9 and abs(cur_al) < 1e-9:
            u2 = cur_vf - cur_vl
            if u2 <= 0:
                return float('inf')
            brake_d = u2 * u2 / (2.0 * D)
            trig = h_min + brake_d
            if cur_gap <= trig:
                return t_offset
            return t_offset + (cur_gap - trig) / u2

    return float('inf')


def _time_to_pos(p0: float, v0: float, a: float, v_cap: float,
                 target: float) -> float:
    """
    Time to travel from position p0 to target, starting at speed v0
    with constant acceleration a until speed reaches v_cap, then constant.

    Returns float('inf') if unreachable (e.g. decelerating to 0 before target).
    """
    dist = target - p0
    if dist <= 1e-6:
        return 0.0
    if abs(a) < 1e-9:
        return dist / v0 if v0 > 1e-9 else float('inf')
    # Time to reach v_cap from v0 under acceleration a
    dt_cap = (v_cap - v0) / a
    p_at_cap = v0 * dt_cap + 0.5 * a * dt_cap ** 2
    if p_at_cap >= dist:
        # Target reached during accel/decel phase — solve quadratic:
        # 0.5*a*dt² + v0*dt - dist = 0
        # disc = v0² + 2*a*dist
        disc = v0 ** 2 + 2.0 * a * dist
        if disc < 0:
            return float('inf')
        sqrt_disc = math.sqrt(max(0.0, disc))
        dt = (-v0 + sqrt_disc) / a
        if dt < -1e-9:
            dt = (-v0 - sqrt_disc) / a
        return max(0.0, dt)
    else:
        if v_cap <= 1e-9:
            return float('inf')   # decelerating to zero — never reaches target
        return dt_cap + (dist - p_at_cap) / v_cap


class OHTSegment:
    def __init__(self, sid, from_id: str, to_id: str,
                 length: float, max_speed: float = 1000.0,
                 path_points: List[Tuple[float, float]] = None):
        self.id          = sid
        self.from_id     = from_id
        self.to_id       = to_id
        self.length      = length
        self.max_speed   = max_speed
        self.path_points = path_points or []   # dense polyline for vis + xy interp
        # ordered queue: front element = [0], back = [-1]
        self.queue: List[OHTAgent] = []


class OHTMap:
    """
    Lightweight graph built from KaistTB_map.json OHT_A sub-network.
    """
    def __init__(self, json_path: str, area: str = 'OHT_A'):
        import json
        with open(json_path, 'r', encoding='utf-8') as f:
            d = json.load(f)

        area_ids = {n['id'] for n in d['nodes'] if n.get('area') == area}
        self.nodes: Dict[str, OHTNode] = {
            n['id']: OHTNode(n['id'], n['x'], n['y'])
            for n in d['nodes'] if n['id'] in area_ids
        }

        self.segments: Dict[Tuple[str, str], OHTSegment] = {}
        self.adj: Dict[str, List[str]] = {nid: [] for nid in self.nodes}

        for s in d['segments']:
            fn, tn = s['startNodeId'], s['endNodeId']
            if fn in area_ids and tn in area_ids:
                n1, n2 = self.nodes[fn], self.nodes[tn]
                start_xy = (n1.x, n1.y)
                parts = s.get('parts')
                if parts:
                    path_pts = _build_path_points(parts, start_xy)
                    length   = _polyline_length(path_pts)
                else:
                    path_pts = [(n1.x, n1.y), (n2.x, n2.y)]
                    length   = math.hypot(n2.x - n1.x, n2.y - n1.y)
                seg = OHTSegment(s['id'], fn, tn, length,
                                 s.get('speed', 1000), path_pts)
                self.segments[(fn, tn)] = seg
                self.adj[fn].append(tn)

        # vehicle model
        oht_vm = next((v for v in d.get('vehicleModels', []) if v['id'] == 'OHT'), {})
        dim = oht_vm.get('dimension', {})
        sm  = oht_vm.get('safetyMargin', {})
        self.vehicle_length = dim.get('length', 1108)
        self.vehicle_width  = dim.get('width',  460)
        margin = 200   # mm — front safety margin only
        self.h_min = self.vehicle_length + margin
        self.accel = oht_vm.get('acceleration', 500)   # mm/s²
        self.decel = oht_vm.get('deceleration', 500)   # mm/s²
        # max braking distance at full speed
        max_v = max((s.get('speed', 1000) for s in d['segments']), default=1000)
        self.max_brake = max_v ** 2 / (2.0 * self.decel)

        # port nodes
        self.port_nodes = {
            p['nodeId'] for p in d.get('ports', [])
            if p['nodeId'] in area_ids
        }

        # ── ZCU zones (merge / diverge one-at-a-time control) ─────────────
        # Only zones whose nodes exist in the loaded sub-network are active.
        _raw_zones = [
            # 합류점(MERGE)만 ZCU 적용 — 분기점은 전방감지 로직으로 처리
            ZCUZone('ZCU_10027',
                    frozenset({('10023', '10027'), ('10024', '10027')}),
                    frozenset({('10027', '10028')})),
            ZCUZone('ZCU_10002',
                    frozenset({('10034', '10002'), ('10001', '10002')}),
                    frozenset({('10002', '10003'), ('10002', '10001')})),
        ]
        # keep only zones whose segments are present in this sub-network
        present = set(self.segments.keys())
        self.zcu_zones: List[ZCUZone] = [
            z for z in _raw_zones
            if z.entry_segs & present and z.exit_segs & present
        ]
        self._entry_zcu: Dict[Tuple[str, str], ZCUZone] = {}
        self._exit_zcu:  Dict[Tuple[str, str], ZCUZone] = {}
        for z in self.zcu_zones:
            for k in z.entry_segs:
                if k in present:
                    self._entry_zcu[k] = z
            for k in z.exit_segs:
                if k in present:
                    self._exit_zcu[k] = z

        # set of ZCU node IDs — the merge/diverge nodes themselves (for visualiser)
        self.zcu_node_ids: set = set()
        for z in self.zcu_zones:
            # zone node = target of entry segments = source of exit segments
            self.zcu_node_ids |= {tn for _, tn in z.entry_segs}
            self.zcu_node_ids |= {fn for fn, _ in z.exit_segs}

        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        pad = self.vehicle_length
        self.bbox = (min(xs)-pad, min(ys)-pad, max(xs)+pad, max(ys)+pad)

    def nearby_nodes(self, node_id: str, max_dist: float) -> set:
        """Return set of node IDs reachable from *node_id* within *max_dist*
        graph distance, considering BOTH forward and reverse segment edges.
        Used to build exclusion zones for safe initial placement."""
        # build reverse adjacency once (cached)
        if not hasattr(self, '_radj'):
            self._radj: Dict[str, List[str]] = {nid: [] for nid in self.nodes}
            for (fn, tn) in self.segments:
                self._radj[tn].append(fn)
        visited = {node_id}
        queue = collections.deque([(node_id, 0.0)])
        while queue:
            cur, dist = queue.popleft()
            # forward neighbours
            for nxt in self.adj.get(cur, []):
                seg = self.segments.get((cur, nxt))
                if seg and dist + seg.length <= max_dist and nxt not in visited:
                    visited.add(nxt)
                    queue.append((nxt, dist + seg.length))
            # reverse neighbours
            for nxt in self._radj.get(cur, []):
                seg = self.segments.get((nxt, cur))
                if seg and dist + seg.length <= max_dist and nxt not in visited:
                    visited.add(nxt)
                    queue.append((nxt, dist + seg.length))
        return visited

    def bfs_path(self, start: str, goal: str = None) -> List[str]:
        """BFS from start; returns loop path if goal is None."""
        if goal is None:
            # pick furthest reachable node as goal
            visited = {start}
            queue   = collections.deque([(start, [start])])
            best    = [start]
            while queue:
                cur, path = queue.popleft()
                if len(path) > len(best):
                    best = path
                for nb in self.adj.get(cur, []):
                    if nb not in visited:
                        visited.add(nb)
                        queue.append((nb, path + [nb]))
            return best

        visited = {start}
        queue   = collections.deque([(start, [start])])
        while queue:
            cur, path = queue.popleft()
            if cur == goal:
                return path
            for nb in self.adj.get(cur, []):
                if nb not in visited:
                    visited.add(nb)
                    queue.append((nb, path + [nb]))
        return [start]


# ── Agent ─────────────────────────────────────────────────────────────────────

class OHTAgent:
    _id_counter = 0

    def __init__(self, agent_id: int, color: tuple,
                 node_path: List[str], max_speed: float = 1000.0):
        self.id        = agent_id
        self.color     = color
        self.node_path = node_path   # sequence of node IDs
        self.path_idx  = 0           # index of CURRENT node (already at)
        self.max_speed = max_speed

        # kinematic state
        self.state     = IDLE
        self.v         = 0.0         # current speed mm/s
        self.seg: Optional[OHTSegment] = None   # current segment
        self.seg_entry_t = 0.0       # time when entered current segment
        self.seg_entry_pos = 0.0     # position along segment at entry

        # token for stale kinematic-event invalidation (CATCH_UP / GAP_CLEAR)
        self.token     = 0
        # token for stale TRY_ADVANCE invalidation
        self.adv_token = 0

        # leader-follower relationship (direct registration)
        self.leader: Optional['OHTAgent'] = None      # who I'm following
        self.followers: List['OHTAgent']  = []         # who's following me

        # acceleration state
        self.a     = 0.0    # current acceleration mm/s² (negative = decel)
        self.v_cap = 0.0    # target speed for current accel/decel phase

        # screen position cache (set by OHTEnvironmentDES.step)
        self.x         = 0.0
        self.y         = 0.0
        self.theta     = 0.0         # heading (map-space rad, y-up)

        # bypass zone tracking
        self._on_bypass_curve: bool = False   # True if currently on a curve entry

        # blocking reason (for debugging / UI)
        self.block_reason: str = ''           # 'node_blocker', 'gap', 'zcu', 'cross_seg'
        self.blocked_by: Optional['OHTAgent'] = None  # agent causing block
        self.block_zcu_id: str = ''           # ZCU zone id if blocked by ZCU

    # ── Kinematics ────────────────────────────────────────────────────────────

    @property
    def cur_node(self) -> str:
        return self.node_path[self.path_idx]

    def pos(self, t: float) -> float:
        """Position along current segment at time t (mm from start node)."""
        if self.seg is None:
            return 0.0
        dt = t - self.seg_entry_t
        if abs(self.a) < 1e-9:
            return self.seg_entry_pos + self.v * dt
        dt_cap = (self.v_cap - self.v) / self.a
        if dt_cap < 0:
            dt_cap = 0.0
        if dt <= dt_cap:
            return self.seg_entry_pos + self.v * dt + 0.5 * self.a * dt ** 2
        p_at_cap = self.seg_entry_pos + self.v * dt_cap + 0.5 * self.a * dt_cap ** 2
        return p_at_cap + self.v_cap * max(0.0, dt - dt_cap)

    def vel(self, t: float) -> float:
        """Instantaneous speed at time t."""
        if self.seg is None:
            return 0.0
        if abs(self.a) < 1e-9:
            return self.v
        dt = t - self.seg_entry_t
        dt_cap = (self.v_cap - self.v) / self.a
        if dt_cap < 0 or dt >= dt_cap:
            return self.v_cap
        return max(0.0, self.v + self.a * dt)

    def xy(self, t: float, nodes: Dict[str, OHTNode]) -> Tuple[float, float, float]:
        """(x, y, theta) in map coordinates at time t."""
        if self.seg is None:
            n = nodes.get(self.cur_node)
            if n:
                return n.x, n.y, self.theta
            return 0.0, 0.0, self.theta
        p = min(self.pos(t), self.seg.length)
        if self.seg.path_points:
            return _interp_path(self.seg.path_points, p)
        # fallback: straight line
        n1, n2 = nodes[self.seg.from_id], nodes[self.seg.to_id]
        dx, dy = n2.x - n1.x, n2.y - n1.y
        L = self.seg.length if self.seg.length > 0 else 1.0
        frac = p / L
        return (n1.x + dx * frac,
                n1.y + dy * frac,
                math.atan2(dy, dx))

    def _speed_up(self, t: float, accel: float):
        """Start accelerating toward segment max_speed (increments token)."""
        if self.seg is None:
            return
        p   = min(self.pos(t), self.seg.length)
        v_c = self.vel(t)
        self.seg_entry_pos = p
        self.seg_entry_t   = t
        self.token        += 1
        self.v     = v_c
        seg_max = min(self.max_speed, self.seg.max_speed)
        self.v_cap = seg_max
        self.a     = accel if v_c < seg_max else 0.0

    def _slow_to(self, t: float, speed: float, decel: float):
        """Start decelerating toward speed (increments token)."""
        if self.seg is None:
            return
        p   = self.pos(t)
        v_c = self.vel(t)
        self.seg_entry_pos = p
        self.seg_entry_t   = t
        self.token        += 1
        self.v_cap = speed
        if v_c <= speed:
            self.v = speed
            self.a = 0.0
        else:
            self.v = v_c
            self.a = -decel


# ── Event ─────────────────────────────────────────────────────────────────────

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


# ── Environment ───────────────────────────────────────────────────────────────

class OHTEnvironmentDES:
    """
    Discrete-Event Simulation engine for OHT with front-detection.

    Usage
    -----
    env = OHTEnvironmentDES(oht_map)
    for agent in agents:
        env.add_agent(agent, t_start=0.0)
    env.step(t)   # call from Pygame main loop with current sim time
    """

    def __init__(self, oht_map: OHTMap, cross_segment: bool = True):
        self.map     = oht_map
        self._agents: Dict[int, OHTAgent] = {}
        self._heap:   List[Event]         = []
        self.cross_segment = cross_segment   # N-hop cross-segment detection
        self.event_count   = 0               # total events processed

        # ── Reverse segment index: to_id → list of segments ending there ──
        self._segs_to: Dict[str, List[OHTSegment]] = {}
        for seg in oht_map.segments.values():
            self._segs_to.setdefault(seg.to_id, []).append(seg)

        # ── Node-blocker index: node_id → set of agents sitting there ─────
        self._node_agents: Dict[str, set] = {}  # node_id → {agent}

        # ZCU runtime state (reset with env, not persistent in OHTMap)
        self._zcu_holders:   Dict[str, Optional[OHTAgent]] = {
            z.id: None for z in oht_map.zcu_zones}
        self._zcu_holder_seg: Dict[str, Optional[Tuple[str,str]]] = {
            z.id: None for z in oht_map.zcu_zones}  # which entry seg the holder used
        self._zcu_waitlists: Dict[str, List[OHTAgent]] = {
            z.id: [] for z in oht_map.zcu_zones}
        self._agent_zcu: Dict[int, 'ZCUZone'] = {}  # agent_id → zone held
        # merge_node_id → ZCUZone (for fast lookup when departing merge)
        self._merge_zcu: Dict[str, 'ZCUZone'] = {}
        for z in oht_map.zcu_zones:
            for (_, tn) in z.entry_segs:
                self._merge_zcu[tn] = z
                break

    # ── Agent management ──────────────────────────────────────────────────────

    def add_agent(self, agent: OHTAgent, t_start: float = 0.0):
        self._agents[agent.id] = agent
        agent.state     = IDLE
        agent.v         = 0.0
        agent.a         = 0.0
        agent.v_cap     = agent.max_speed
        agent.seg       = None
        agent.adv_token = 0
        agent.token     = 0
        agent.leader    = None
        agent.followers = []
        # Register in node-blocker index
        self._node_agents.setdefault(agent.cur_node, set()).add(agent)
        self._post_advance(t_start, agent)

    # ── Leader-follower registration ─────────────────────────────────────

    def _follow(self, follower: OHTAgent, leader: OHTAgent):
        """Register follower as following leader."""
        if follower.leader is leader:
            return
        self._unfollow(follower)
        follower.leader = leader
        if follower not in leader.followers:
            leader.followers.append(follower)

    def _unfollow(self, follower: OHTAgent):
        """Unregister follower from its current leader."""
        if follower.leader is not None:
            ldr = follower.leader
            if follower in ldr.followers:
                ldr.followers.remove(follower)
            follower.leader = None

    def _notify_followers(self, t: float, leader: OHTAgent):
        """Leader's acceleration changed — recalculate each follower's state."""
        for f in list(leader.followers):
            if f.seg is None:
                continue
            self._recalc_follower(t, f)

    def _recalc_follower(self, t: float, agent: OHTAgent):
        """Recalculate follower's catch-up state based on current leader info.
        Called when leader's acceleration changes."""
        seg = agent.seg
        if seg is None:
            return
        q_idx = seg.queue.index(agent) if agent in seg.queue else -1
        if q_idx < 0:
            return

        if q_idx == 0:
            # front of queue — check cross-segment leader
            self._schedule_front_catchup(t, agent, seg)
        else:
            # behind another agent in same queue
            leader = seg.queue[q_idx - 1]
            vf = agent.vel(t)
            vl = leader.vel(t)
            gap = leader.pos(t) - agent.pos(t)

            if gap > self.map.h_min and agent.state == FOLLOWING and vf <= 0:
                # gap is safe — resume
                self._unfollow(agent)
                agent._speed_up(t, self.map.accel)
                agent.state = MOVING
                remaining = seg.length - agent.seg_entry_pos
                if remaining > 0:
                    dt_done = _time_to_pos(0.0, agent.v, agent.a, agent.v_cap, remaining)
                    self._post(t + dt_done, SEGMENT_DONE, agent)
                self._schedule_catch_up(t, agent, leader)
                self._notify_followers(t, agent)
                return

            if vf <= vl and agent.a <= leader.a:
                return  # not closing

            dt = _accel_brake_dt(vf, vl, agent.a, agent.v_cap,
                                 self.map.decel, gap, self.map.h_min,
                                 al=leader.a, vl_max=leader.v_cap)
            if dt <= 1e-6:
                target = leader.v_cap if leader.a < 0 else vl
                self._slow_and_cascade(t, agent, target)
            elif dt < float('inf'):
                self._post(t + dt, CATCH_UP, agent)

    def remove_agent(self, agent_id: int):
        agent = self._agents.pop(agent_id, None)
        if agent is None:
            return
        # clean up node-blocker index
        na = self._node_agents.get(agent.cur_node)
        if na:
            na.discard(agent)
        # clean up leader-follower
        self._unfollow(agent)
        for f in list(agent.followers):
            self._unfollow(f)
        # clean up segment queue
        if agent.seg:
            seg = agent.seg
            if agent in seg.queue:
                seg.queue.remove(agent)
            agent.seg = None
        # clean up ZCU state
        for z in self.map.zcu_zones:
            wl = self._zcu_waitlists[z.id]
            if agent in wl:
                wl.remove(agent)
            if self._zcu_holders[z.id] is agent:
                # release zone and wake next waiter (at t=0 — processed next step)
                self._zcu_holders[z.id] = None
                self._zcu_holder_seg[z.id] = None
                if wl:
                    nxt = wl.pop(0)
                    if nxt.id in self._agents:
                        self._post_advance(0.0, nxt)

    # ── Event posting ─────────────────────────────────────────────────────────

    def _post(self, t: float, kind: str, agent: OHTAgent, data=None):
        tok = agent.adv_token if kind == TRY_ADVANCE else agent.token
        heapq.heappush(self._heap,
                       Event(t, kind, agent.id, tok, data))

    def _post_advance(self, t: float, agent: OHTAgent):
        """Post a TRY_ADVANCE, invalidating any previous pending one."""
        agent.adv_token += 1
        self._post(t, TRY_ADVANCE, agent)

    # ── Main step ─────────────────────────────────────────────────────────────

    def step(self, t_now: float):
        """Process all events with t ≤ t_now."""
        while self._heap and self._heap[0].t <= t_now:
            ev = heapq.heappop(self._heap)
            agent = self._agents.get(ev.agent_id)
            if agent is None:
                continue
            if ev.kind == TRY_ADVANCE:
                if ev.token != agent.adv_token:
                    continue  # stale retry
                self.event_count += 1
                self._on_try_advance(ev.t, agent)
            elif ev.kind == SEGMENT_DONE:
                if ev.token != agent.token:
                    continue  # stale
                self.event_count += 1
                self._on_segment_done(ev.t, agent)
            elif ev.kind == CATCH_UP:
                if ev.token != agent.token:
                    continue
                self.event_count += 1
                self._on_catch_up(ev.t, agent)
            elif ev.kind == GAP_CLEAR:
                if ev.token != agent.token:
                    continue
                self.event_count += 1
                self._on_gap_clear(ev.t, agent)

        # update screen positions
        for agent in self._agents.values():
            x, y, th = agent.xy(t_now, self.map.nodes)
            agent.x, agent.y, agent.theta = x, y, th

    # ── TRY_ADVANCE ───────────────────────────────────────────────────────────

    def _on_try_advance(self, t: float, agent: OHTAgent):
        path = agent.node_path
        nxt_idx = agent.path_idx + 1
        if nxt_idx >= len(path):
            agent.state = DONE
            return

        from_id = path[agent.path_idx]
        to_id   = path[nxt_idx]
        seg     = self.map.segments.get((from_id, to_id))
        if seg is None:
            agent.state = DONE
            return

        # ── Effective tail: whichever is closer — queue tail or node blocker
        # A BLOCKED/IDLE agent physically sitting at to_id acts as a stopped
        # vehicle at position seg.length (end of this segment).
        nb = self._node_blocker(to_id, agent)

        eff_tail_pos = None
        eff_tail_v   = None
        if seg.queue:
            tail = seg.queue[-1]
            eff_tail_pos = min(tail.pos(t), seg.length)
            eff_tail_v   = tail.v
        if nb is not None:
            # node blocker is at seg.length with speed 0
            if eff_tail_pos is None or seg.length <= eff_tail_pos:
                eff_tail_pos = seg.length
                eff_tail_v   = 0.0

        if eff_tail_pos is not None:
            gap = eff_tail_pos
            safe_d = _safe_gap(agent.v, self.map.decel, self.map.h_min)
            if gap < safe_d - 1.0:
                agent.state = BLOCKED
                agent.v     = 0.0       # physically stopped at node
                agent.block_reason = 'node_blocker' if nb is not None else 'gap'
                agent.blocked_by   = nb if nb is not None else (seg.queue[-1] if seg.queue else None)
                agent.block_zcu_id = ''
                if eff_tail_v > 0:
                    wait = (self.map.h_min - gap) / eff_tail_v
                else:
                    wait = 0.5
                self._post_advance(t + wait, agent)
                self._notify_node_occupied(t, from_id)
                return

        # ── Cross-segment gap check (path-aware) ─────────────────────────────
        # Look through short segments + always one more node beyond.
        if self.cross_segment:
            cs_total = seg.length
            cs_node  = to_id
            cs_pi    = nxt_idx
            h_min    = self.map.h_min
            looked_past_long = False

            def _block_cross(wait_t, reason, blocker):
                agent.state = BLOCKED
                agent.v     = 0.0
                agent.block_reason = reason
                agent.blocked_by   = blocker
                agent.block_zcu_id = ''
                self._post_advance(t + wait_t, agent)
                self._notify_node_occupied(t, from_id)

            while True:
                if cs_node != to_id:
                    nb2 = self._node_blocker(cs_node, agent)
                    if nb2 is not None:
                        if cs_total < h_min - 1.0:
                            _block_cross(0.5, 'cross_node', nb2)
                            return
                        break
                if cs_pi + 1 >= len(path):
                    break
                cs_next = path[cs_pi + 1]
                cs_seg = self.map.segments.get((cs_node, cs_next))
                if cs_seg is None:
                    break
                if cs_seg.queue:
                    cs_front = cs_seg.queue[0]
                    cross_gap = cs_total + cs_front.pos(t)
                    if cross_gap < h_min - 1.0:
                        shortfall = h_min - cross_gap
                        cs_v = cs_front.vel(t)
                        wait = _time_to_pos(0.0, max(cs_v, 0.0),
                                            self.map.accel, cs_front.max_speed, shortfall)
                        wait = min(max(wait + 0.1, 0.3), 5.0)
                        _block_cross(wait, 'cross_seg', cs_front)
                        return
                    break
                cs_total += cs_seg.length
                cs_node   = cs_next
                cs_pi    += 1
                # Long segment: check one more node then stop
                if cs_seg.length >= h_min:
                    if looked_past_long:
                        break
                    looked_past_long = True

        # ── U-turn protection ─────────────────────────────────────────────
        # U-turn first half (diverge→mid): one vehicle at a time.
        #   Check own queue + sibling (second half) queue.
        # U-turn second half (mid→merge): NO check — already committed.
        curve_gid = getattr(seg, '_curve_group', None)
        if curve_gid is not None:
            is_first_half = not from_id.startswith('_curve_mid_')
            if is_first_half:
                # Block if THIS segment already has a vehicle (one at a time)
                if seg.queue:
                    agent.state = BLOCKED
                    agent.v     = 0.0
                    agent.block_reason = 'curve_group'
                    agent.blocked_by   = seg.queue[-1]
                    agent.block_zcu_id = ''
                    self._post_advance(t + 0.5, agent)
                    self._notify_node_occupied(t, from_id)
                    return
                # Also block if sibling (second half) has a vehicle
                check_segs = getattr(self.map, 'curve_groups', {}).get(curve_gid, [])
                for ck in check_segs:
                    if ck == (from_id, to_id):
                        continue
                    ck_seg = self.map.segments.get(ck)
                    if ck_seg and ck_seg.queue:
                        agent.state = BLOCKED
                        agent.v     = 0.0
                        agent.block_reason = 'curve_group'
                        agent.blocked_by   = ck_seg.queue[-1]
                        agent.block_zcu_id = ''
                        self._post_advance(t + 0.5, agent)
                        self._notify_node_occupied(t, from_id)
                        return

        # ── Diverge protection (occupancy-based, no lock) ──────────────────
        # At diverge nodes: if a curve exit has a vehicle, block other exits.
        # If a straight exit has vehicles, block curve exits.
        # Same-direction straight exits: free queue.
        div_curve_exits = getattr(self.map, 'diverge_curve_exits', set())
        div_straight_exits = getattr(self.map, 'diverge_straight_exits', set())
        is_div_curve = (from_id, to_id) in div_curve_exits
        is_div_straight = (from_id, to_id) in div_straight_exits
        if is_div_curve or is_div_straight:
            # Find all sibling exits from same diverge node
            all_exits = [(from_id, s) for s in self.map.adj.get(from_id, [])]
            blocker = None
            for ek in all_exits:
                if ek == (from_id, to_id):
                    continue  # skip self
                ek_seg = self.map.segments.get(ek)
                if not ek_seg or not ek_seg.queue:
                    continue
                ek_is_curve = ek in div_curve_exits
                if is_div_straight and ek_is_curve:
                    # I'm straight, other is curve with vehicle → block me
                    blocker = ek_seg.queue[-1]
                    break
                elif is_div_curve and not ek_is_curve:
                    # I'm curve, other is straight with vehicle → block me
                    blocker = ek_seg.queue[-1]
                    break
                elif is_div_curve and ek_is_curve:
                    # Both curve → block (one at a time)
                    blocker = ek_seg.queue[-1]
                    break
                # Both straight → no block (free queue)
            if blocker is not None:
                agent.state = BLOCKED
                agent.v     = 0.0
                agent.block_reason = 'diverge'
                agent.blocked_by   = blocker
                agent.block_zcu_id = ''
                self._post_advance(t + 0.5, agent)
                self._notify_node_occupied(t, from_id)
                return

        # ── Bypass zone check (merge ZCU) ─────────────────────────────────
        # Curve entries: lock required (one at a time)
        # Straight entries: free queue, but blocked if a curve is occupied
        zcu = self.map._entry_zcu.get((from_id, to_id))
        if zcu is not None:
            is_curve = (from_id, to_id) in getattr(self.map, 'bypass_curve_segs', set())
            is_straight = (from_id, to_id) in getattr(self.map, 'bypass_straight_segs', set())
            # U-turn second half: treat as straight (already committed to U-turn)
            # Only block if a CURVE vehicle holds the lock (another conflicting direction)
            if is_curve and from_id.startswith('_curve_mid_'):
                is_curve = False
                is_straight = True
            holder = self._zcu_holders[zcu.id]

            if is_curve:
                # Curve entry: must acquire lock. Block if anyone holds it.
                if holder is not None and holder is not agent:
                    agent.state = BLOCKED
                    agent.v     = 0.0
                    agent.block_reason = 'zcu_curve'
                    agent.blocked_by   = holder
                    agent.block_zcu_id = zcu.id
                    wl = self._zcu_waitlists[zcu.id]
                    if agent not in wl:
                        wl.append(agent)
                    self._notify_node_occupied(t, from_id)
                    return
            elif is_straight:
                # Straight entry: only blocked if a CURVE holds the lock.
                # If another straight holds it, free to enter (queue behind).
                if holder is not None:
                    holder_on_curve = getattr(holder, '_on_bypass_curve', False)
                    if holder_on_curve:
                        agent.state = BLOCKED
                        agent.v     = 0.0
                        agent.block_reason = 'zcu_straight_wait'
                        agent.blocked_by   = holder
                        agent.block_zcu_id = zcu.id
                        wl = self._zcu_waitlists[zcu.id]
                        if agent not in wl:
                            wl.append(agent)
                        self._notify_node_occupied(t, from_id)
                        return
            else:
                # Fallback: standard lock check
                if holder is not None and holder is not agent:
                    agent.state = BLOCKED
                    agent.v     = 0.0
                    agent.block_reason = 'zcu'
                    agent.blocked_by   = holder
                    agent.block_zcu_id = zcu.id
                    wl = self._zcu_waitlists[zcu.id]
                    if agent not in wl:
                        wl.append(agent)
                    self._notify_node_occupied(t, from_id)
                    return

        # enter segment — carry speed from previous segment (0 if fresh start)
        # Remove from node-blocker index (agent is leaving from_id)
        na = self._node_agents.get(from_id)
        if na:
            na.discard(agent)
        v_entry = agent.v
        seg_max = min(agent.max_speed, seg.max_speed)
        agent.path_idx      = nxt_idx
        agent.seg           = seg
        agent.seg_entry_t   = t
        agent.seg_entry_pos = 0.0
        agent.v             = v_entry
        agent.v_cap         = seg_max
        # If over segment speed limit, decelerate; otherwise accelerate
        if v_entry > seg_max:
            agent.a = -self.map.decel
        elif v_entry < seg_max:
            agent.a = self.map.accel
        else:
            agent.a = 0.0
        agent.token        += 1
        agent.state         = MOVING
        agent.block_reason  = ''
        agent.blocked_by    = None
        agent.block_zcu_id  = ''
        seg.queue.append(agent)

        # Acquire ZCU lock (curve entries always; straight entries only if no curve holds it)
        if zcu is not None:
            is_curve = (from_id, to_id) in getattr(self.map, 'bypass_curve_segs', set())
            # U-turn second half: treat as straight for lock purposes too
            if is_curve and from_id.startswith('_curve_mid_'):
                is_curve = False
            agent._on_bypass_curve = is_curve
            if is_curve:
                # Curve: always acquire lock
                self._zcu_holders[zcu.id] = agent
            elif self._zcu_holders[zcu.id] is None:
                # Straight: acquire only if no one holds it (first straight)
                self._zcu_holders[zcu.id] = agent
            # else: straight following another straight — don't overwrite holder
            wl = self._zcu_waitlists[zcu.id]
            if agent in wl:
                wl.remove(agent)

        # schedule arrival at end node using kinematic helper
        dt_done = _time_to_pos(0.0, agent.v, agent.a, agent.v_cap, seg.length)
        self._post(t + dt_done, SEGMENT_DONE, agent)

        # Agent left from_id — notify registered followers + wake BLOCKED
        self._notify_followers(t, agent)
        self._signal_node_unblocked(t, from_id)

        # (ZCU release handled in _on_segment_done via _release_zcu)

        # check if following queue leader
        q_idx = seg.queue.index(agent)
        if q_idx > 0:
            self._schedule_catch_up(t, agent, seg.queue[q_idx - 1])
        else:
            # front of queue — check node blocker and cross-segment leader
            self._schedule_front_catchup(t, agent, seg)

    # ── SEGMENT_DONE ──────────────────────────────────────────────────────────

    def _on_segment_done(self, t: float, agent: OHTAgent):
        seg = agent.seg
        if seg is None:
            return

        # remove from queue
        if agent in seg.queue:
            seg.queue.remove(agent)

        # notify all registered followers that this leader departed
        self._notify_followers(t, agent)

        # save speed at exit, capped by next segment's max speed
        exit_v = agent.vel(t)
        nxt_idx = agent.path_idx + 1
        if nxt_idx < len(agent.node_path):
            nxt_seg = self.map.segments.get(
                (agent.node_path[agent.path_idx], agent.node_path[nxt_idx]))
            if nxt_seg:
                exit_v = min(exit_v, nxt_seg.max_speed)
        agent.v     = exit_v
        agent.a     = 0.0
        agent.v_cap = agent.max_speed
        agent.seg   = None
        agent.state = IDLE
        # Register in node-blocker index (agent now sitting at seg.to_id)
        self._node_agents.setdefault(seg.to_id, set()).add(agent)

        # ZCU exit — release zone if this was an exit segment
        zcu = self.map._exit_zcu.get((seg.from_id, seg.to_id))
        if zcu is not None and self._zcu_holders[zcu.id] is agent:
            # For straight entries: only release if no more vehicles on this segment
            is_straight = (seg.from_id, seg.to_id) in getattr(self.map, 'bypass_straight_segs', set())
            if is_straight and seg.queue:
                # Transfer lock to next vehicle in same-direction queue
                self._zcu_holders[zcu.id] = seg.queue[0]
            else:
                self._release_zcu(t, zcu)

        # wake any agent blocked waiting to enter this segment
        self._wake_blocked_for(t, seg)

        # Re-evaluate cross-segment followers: leader situation changed
        if self.cross_segment:
            self._check_incoming_followers(t, seg.from_id)

        # advance to next node
        self._post_advance(t, agent)

    def _wake_blocked_at_node(self, t: float, node_id: str):
        """Wake all BLOCKED agents sitting at a specific node."""
        agents_at = self._node_agents.get(node_id)
        if agents_at:
            for a in list(agents_at):
                if a.state == BLOCKED and a.seg is None:
                    self._post_advance(t, a)

    def _release_zcu(self, t: float, zcu: ZCUZone):
        """Release a ZCU zone and wake the next waiting agent."""
        self._zcu_holders[zcu.id] = None
        wl = self._zcu_waitlists[zcu.id]
        while wl:
            nxt = wl.pop(0)
            if nxt.id in self._agents:
                self._post_advance(t, nxt)
                break   # only wake one — it will re-check and acquire

    def _wake_blocked_for(self, t: float, seg: OHTSegment):
        """Post an immediate TRY_ADVANCE to any BLOCKED agent that wants to enter seg."""
        for a in self._agents.values():
            if a.state != BLOCKED or a.seg is not None:
                continue
            idx = a.path_idx
            if (idx + 1 < len(a.node_path)
                    and a.node_path[idx]   == seg.from_id
                    and a.node_path[idx+1] == seg.to_id):
                self._post_advance(t, a)

    def _node_blocker(self, node_id: str,
                      exclude: OHTAgent = None) -> Optional[OHTAgent]:
        """Return any agent physically at node_id (not on a segment)."""
        agents_at = self._node_agents.get(node_id)
        if not agents_at:
            return None
        for a in agents_at:
            if a is not exclude:
                return a
        return None

    def _look_ahead_leader(self, t: float, agent: OHTAgent,
                           seg: OHTSegment) -> Optional[Tuple[float, float, OHTAgent]]:
        """
        Multi-hop look-ahead from front-of-queue agent on seg.
        Follows agent's node_path through short / empty segments until
        a node-blocker or queued vehicle is found.

        When self.cross_segment is False, only checks node_blocker at
        seg.to_id (same-segment-only mode for A/B comparison).

        Returns (gap, leader_speed, leader_agent) or None.
        """
        remaining = seg.length - agent.pos(t)
        total_gap = remaining
        look_node = seg.to_id
        pi = agent.path_idx          # points to seg.to_id

        if not self.cross_segment:
            # ── Legacy mode: node-blocker at to_id only ──────────────
            nb = self._node_blocker(look_node, agent)
            if nb is not None:
                return (total_gap, 0.0, nb)
            return None

        # ── N-hop cross-segment mode ─────────────────────────────────
        # Look through segments using dynamic safe gap (h_min + brake dist).
        # Always check one more node past a long segment.
        safe_d = _safe_gap(agent.vel(t), self.map.decel, self.map.h_min)
        looked_past_long = False
        while True:
            nb = self._node_blocker(look_node, agent)
            if nb is not None:
                return (total_gap, 0.0, nb)

            if pi + 1 >= len(agent.node_path):
                break

            next_to  = agent.node_path[pi + 1]
            next_seg = self.map.segments.get((look_node, next_to))
            if next_seg is None:
                break

            if next_seg.queue:
                front = next_seg.queue[0]
                if front is not agent:
                    return (total_gap + front.pos(t), front.vel(t), front)
                break

            # Add this segment's length and check one more node
            total_gap += next_seg.length
            look_node = next_to
            pi += 1

            # Long segment (>= safe_d): check one more node then stop
            if next_seg.length >= safe_d:
                if looked_past_long:
                    break
                looked_past_long = True

        return None

    def _schedule_front_catchup(self, t: float, agent: OHTAgent,
                                seg: OHTSegment):
        """Schedule a CATCH_UP for the front-of-queue agent on seg,
        using multi-hop look-ahead through short / empty segments.

        trigger_gap = h_min + (vf - vl)² / (2 * decel)
        """
        result = self._look_ahead_leader(t, agent, seg)
        if result is None:
            self._unfollow(agent)
            if agent.state == FOLLOWING:
                # no leader — resume
                agent._speed_up(t, self.map.accel)
                agent.state = MOVING
                remaining = seg.length - agent.seg_entry_pos
                if remaining > 0:
                    dt_done = _time_to_pos(0.0, agent.v, agent.a, agent.v_cap, remaining)
                    self._post(t + dt_done, SEGMENT_DONE, agent)
                self._notify_followers(t, agent)
            return

        gap_eff, vl, ldr = result
        self._follow(agent, ldr)
        vf = agent.vel(t)

        if vf <= 0 and agent.a <= 0:
            return  # stopped — will be woken by explicit GAP_CLEAR

        if vf <= vl and agent.a <= 0:
            return                      # not closing

        dt = _accel_brake_dt(vf, vl, agent.a, agent.v_cap,
                             self.map.decel, gap_eff, self.map.h_min,
                             al=ldr.a, vl_max=ldr.v_cap)
        if dt <= 1e-6:
            target = ldr.v_cap if ldr.a < 0 else vl
            self._slow_and_cascade(t, agent, target)
        elif dt < float('inf'):
            self._post(t + dt, CATCH_UP, agent)

    def _check_incoming_followers(self, t: float, node_id: str):
        """When an agent just entered a segment starting at node_id,
        notify front agents on incoming segments so they can update
        their catch-up schedule.  Propagates backward: through short
        segments and one step beyond a long segment."""
        visited = {node_id}
        queue   = [(node_id, False)]
        while queue:
            nid, prev_long = queue.pop(0)
            for seg in self._segs_to.get(nid, ()):
                if not seg.queue:
                    continue
                front = seg.queue[0]
                if front.state in (MOVING, FOLLOWING):
                    self._schedule_front_catchup(t, front, seg)
                if seg.from_id in visited or prev_long:
                    continue
                visited.add(seg.from_id)
                queue.append((seg.from_id, seg.length >= self.map.h_min))

    def _signal_node_unblocked(self, t: float, node_id: str):
        """Called when an agent leaves node_id.
        Notifies any agents on incoming segments that were slowed by its presence.
        In cross-segment mode, propagates back: through short segments (<h_min)
        and always one step beyond a long segment (matching _look_ahead_leader)."""
        visited = {node_id}
        queue = [(node_id, False)]   # (node, prev_was_long)
        while queue:
            nid, prev_long = queue.pop(0)
            for seg in self._segs_to.get(nid, ()):
                for qa in seg.queue:
                    if qa.state == FOLLOWING:
                        self._post(t, GAP_CLEAR, qa)
                self._wake_blocked_for(t, seg)
                if not self.cross_segment or seg.from_id in visited:
                    continue
                if prev_long:
                    continue  # already extended one step past long seg
                visited.add(seg.from_id)
                queue.append((seg.from_id, seg.length >= self.map.h_min))

    def _notify_node_occupied(self, t: float, node_id: str):
        """Called when an agent just became a node blocker at node_id.
        Schedules slow-downs for the front agent on each incoming segment."""
        for seg in self._segs_to.get(node_id, ()):
            if not seg.queue:
                continue
            front = seg.queue[0]
            if front.state in (MOVING, FOLLOWING):
                self._schedule_front_catchup(t, front, seg)

    # ── CATCH_UP ──────────────────────────────────────────────────────────────

    _cascade_depth = 0

    def _slow_and_cascade(self, t: float, agent: OHTAgent, speed: float):
        """Decelerate agent toward speed, then cascade catch-up to its follower.

        Also schedules SEGMENT_DONE at the reduced speed so the agent is not
        stuck indefinitely if no GAP_CLEAR arrives (e.g. cross-segment following).
        """
        agent._slow_to(t, speed, self.map.decel)
        agent.state = FOLLOWING
        if agent.seg:
            # Reschedule SEGMENT_DONE at the new (reduced) speed
            remaining = agent.seg.length - agent.seg_entry_pos
            dt_done = _time_to_pos(0.0, agent.v, agent.a, agent.v_cap, remaining)
            if dt_done < float('inf'):
                self._post(t + dt_done, SEGMENT_DONE, agent)
            # speed <= 0: agent stopped — will be woken by explicit GAP_CLEAR
            # Cascade to follower in same segment
            q  = agent.seg.queue
            qi = q.index(agent) if agent in q else -1
            if 0 <= qi < len(q) - 1:
                self._schedule_catch_up(t, q[qi + 1], agent)
            # Notify cross-segment followers that this agent slowed
            # Guard against infinite recursion
            if self.cross_segment and self._cascade_depth < 10:
                self._cascade_depth += 1
                self._check_incoming_followers(t, agent.seg.from_id)
                self._cascade_depth -= 1

    def _on_catch_up(self, t: float, agent: OHTAgent):
        """Follower reached braking trigger point — decelerate to match leader."""
        seg = agent.seg
        if seg is None:
            return
        q_idx = seg.queue.index(agent) if agent in seg.queue else -1
        if q_idx < 0:
            return

        if q_idx == 0:
            result = self._look_ahead_leader(t, agent, seg)
            if result is not None:
                gap_eff, vl, ldr = result
                self._follow(agent, ldr)
                vf = agent.vel(t)

                if gap_eff > self.map.h_min and vf <= 0 and agent.state == FOLLOWING:
                    # Stopped but gap is safe — resume
                    self._unfollow(agent)
                    agent._speed_up(t, self.map.accel)
                    agent.state = MOVING
                    remaining = seg.length - agent.seg_entry_pos
                    if remaining > 0:
                        dt_done = _time_to_pos(0.0, agent.v, agent.a, agent.v_cap, remaining)
                        self._post(t + dt_done, SEGMENT_DONE, agent)
                    self._schedule_front_catchup(t, agent, seg)
                    self._notify_followers(t, agent)
                    return

                dt = _accel_brake_dt(vf, vl, agent.a, agent.v_cap,
                                     self.map.decel, gap_eff, self.map.h_min,
                                     al=ldr.a, vl_max=ldr.v_cap)
                if dt <= 1e-6:
                    target = ldr.v_cap if ldr.a < 0 else vl
                    self._slow_and_cascade(t, agent, target)
                elif dt < float('inf'):
                    self._post(t + dt, CATCH_UP, agent)
                # inf — gap safe, no closing; agent will be woken by GAP_CLEAR
            elif agent.state == FOLLOWING:
                # No leader ahead — resume
                self._unfollow(agent)
                agent._speed_up(t, self.map.accel)
                agent.state = MOVING
                remaining = seg.length - agent.seg_entry_pos
                if remaining > 0:
                    dt_done = _time_to_pos(0.0, agent.v, agent.a, agent.v_cap, remaining)
                    self._post(t + dt_done, SEGMENT_DONE, agent)
                self._notify_followers(t, agent)
            return

        leader = seg.queue[q_idx - 1]
        target = leader.v_cap if leader.a < 0 else leader.vel(t)
        self._slow_and_cascade(t, agent, target)

    # ── GAP_CLEAR ─────────────────────────────────────────────────────────────

    def _on_gap_clear(self, t: float, agent: OHTAgent):
        """Gap opened up — agent resumes acceleration toward max speed."""
        if agent.state != FOLLOWING:
            return  # already resumed or state changed
        seg = agent.seg
        if seg is None:
            return

        # unfollow old leader BEFORE scheduling (which may register a new one)
        self._unfollow(agent)

        agent._speed_up(t, self.map.accel)
        agent.state = MOVING

        p_now     = agent.seg_entry_pos   # updated by _speed_up
        remaining = seg.length - p_now
        if remaining <= 0:
            self._on_segment_done(t, agent)
            return

        dt_done = _time_to_pos(0.0, agent.v, agent.a, agent.v_cap, remaining)
        self._post(t + dt_done, SEGMENT_DONE, agent)

        # check if still behind someone
        q_idx = seg.queue.index(agent) if agent in seg.queue else -1
        if q_idx > 0:
            self._schedule_catch_up(t, agent, seg.queue[q_idx - 1])
        elif q_idx == 0:
            self._schedule_front_catchup(t, agent, seg)

        # this agent resumed — notify own followers
        self._notify_followers(t, agent)

    # ── catch-up scheduling helper ────────────────────────────────────────────

    def _schedule_catch_up(self, t: float, follower: OHTAgent, leader: OHTAgent):
        """
        Schedule CATCH_UP at the point where follower must start braking to avoid
        closing within h_min of leader.

        trigger_gap = h_min + (vf - vl)² / (2 * decel)
        This ensures that when braking begins, the follower will decelerate to
        leader speed exactly as gap reaches h_min.
        """
        self._follow(follower, leader)
        vf = follower.vel(t)
        vl = leader.vel(t)
        if vf <= vl and follower.a <= leader.a and leader.a >= 0:
            return  # not closing — no action needed
        # leader 감속 중이면 gap이 줄어들 수 있으므로 항상 계산

        gap = leader.pos(t) - follower.pos(t)
        dt  = _accel_brake_dt(vf, vl, follower.a, follower.v_cap,
                              self.map.decel, gap, self.map.h_min,
                              al=leader.a, vl_max=leader.v_cap)
        if dt <= 1e-6:
            # Match leader's target speed, not instantaneous speed
            target = leader.v_cap if leader.a < 0 else vl
            self._slow_and_cascade(t, follower, target)
        elif dt < float('inf'):
            self._post(t + dt, CATCH_UP, follower)

    # ── public helpers ────────────────────────────────────────────────────────

    def reassign(self, agent: OHTAgent, new_path: List[str], t: float):
        """Assign a new path and restart agent from current node."""
        old_node = agent.cur_node
        # Update node-blocker index
        na = self._node_agents.get(old_node)
        if na:
            na.discard(agent)
        if agent.seg and agent in agent.seg.queue:
            agent.seg.queue.remove(agent)
        agent.seg        = None
        agent.node_path  = new_path
        agent.path_idx   = 0
        agent.state      = IDLE
        # Re-register at new start node
        self._node_agents.setdefault(new_path[0], set()).add(agent)
        agent.token     += 1
        # agent was a node_blocker — notify registered followers directly
        self._notify_followers(t, agent)
        self._post_advance(t, agent)

    @property
    def agents(self):
        return list(self._agents.values())

    def all_done(self) -> bool:
        return all(a.state == DONE for a in self._agents.values())
