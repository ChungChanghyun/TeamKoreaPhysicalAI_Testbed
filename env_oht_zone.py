"""
env_oht_zone.py — Zone-based (block signaling) DES for OHT.

Track is subdivided into exclusive zones of length >= h_min.
Collision avoidance = zone reservation queue (claim/release).

Chain claim (연속 주행)
───────────────────────
  - Vehicle looks ahead and claims multiple consecutive free zones at once.
  - If a zone is held by another vehicle that has reservations further ahead,
    the follower can join the queue ("follow-through") and transit continuously.
  - Single speed profile across entire chain: v_start → V_max → 0.
  - ZONE_PASS events at each zone boundary for progressive release.
  - Chain is extended mid-transit when new zones become available.

Zone reservation queue
──────────────────────
  Per zone: deque of agents.  queue[0] = current holder (physical occupant).
  queue[1:] = followers (in transit toward this zone, will become holder
  when predecessor leaves).

Events
──────
  TRY_CLAIM    : build chain, start continuous transit
  ZONE_PASS    : crossed zone boundary — release previous zone, try extend
  TRANSIT_DONE : reached chain end — release previous zone, try new chain
"""
from __future__ import annotations
import math, heapq, collections, json
from typing import Dict, List, Optional, Tuple

# ── Agent states ──────────────────────────────────────────────────────────────
IDLE    = 'IDLE'
MOVING  = 'MOVING'
BLOCKED = 'BLOCKED'
DONE    = 'DONE'

# ── Event types ───────────────────────────────────────────────────────────────
TRY_CLAIM    = 'TRY_CLAIM'
ZONE_PASS    = 'ZONE_PASS'
TRANSIT_DONE = 'TRANSIT_DONE'


# ── Helpers ───────────────────────────────────────────────────────────────────

def _build_path_points(parts: list, start_xy: Tuple[float, float],
                       n_arc: int = 16) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = [start_xy]
    cur = start_xy
    for part in parts:
        raw = [(p['x'], p['y']) for p in part['points']]
        if part['kind'] == 'Straight':
            pts.append(raw[0]); cur = raw[0]
        elif part['kind'] == 'Arc':
            p0, p1, p2 = cur, raw[0], raw[1]
            for i in range(1, n_arc + 1):
                t = i / n_arc
                x = (1-t)**2*p0[0] + 2*t*(1-t)*p1[0] + t**2*p2[0]
                y = (1-t)**2*p0[1] + 2*t*(1-t)*p1[1] + t**2*p2[1]
                pts.append((x, y))
            cur = p2
    return pts


def _polyline_length(pts: List[Tuple[float, float]]) -> float:
    return sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
               for i in range(len(pts)-1))


def _interp_polyline(pts: List[Tuple[float, float]],
                     frac: float,
                     total_len: float = None) -> Tuple[float, float, float]:
    """(x, y, theta) at fraction frac ∈ [0, 1] along polyline."""
    if len(pts) < 2:
        return (pts[0][0], pts[0][1], 0.0) if pts else (0, 0, 0)
    if total_len is None:
        total_len = _polyline_length(pts)
    target = max(0.0, min(frac, 1.0)) * total_len
    accum = 0.0
    for i in range(len(pts) - 1):
        seg_len = math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
        if accum + seg_len >= target or i == len(pts) - 2:
            t = (target - accum) / seg_len if seg_len > 0 else 1.0
            t = min(max(t, 0), 1.0)
            x = pts[i][0] + (pts[i+1][0]-pts[i][0]) * t
            y = pts[i][1] + (pts[i+1][1]-pts[i][1]) * t
            th = math.atan2(pts[i+1][1]-pts[i][1], pts[i+1][0]-pts[i][0])
            return (x, y, th)
        accum += seg_len
    return (pts[-1][0], pts[-1][1], 0.0)


# ── Speed profile ────────────────────────────────────────────────────────────

class SpeedProfile:
    """Speed profile: v_start → V_max → 0.

    Generalised trapezoid that supports non-zero initial speed,
    used for chain extension mid-transit.
    """

    def __init__(self, length: float, v_max: float,
                 accel: float, decel: float, v_start: float = 0.0):
        self.length = length
        if length <= 0:
            self.total_time = 0.0
            self._phases: List[Tuple[float, float, float]] = []
            return

        a = min(accel, 1e12)
        d = min(decel, 1e12)
        v0 = min(max(v_start, 0.0), v_max)
        V = v_max

        d_accel = (V * V - v0 * v0) / (2 * a) if v0 < V else 0.0
        d_decel = V * V / (2 * d)

        if d_accel + d_decel <= length:
            # Normal: accel → cruise → decel
            t_a = (V - v0) / a if v0 < V else 0.0
            t_c = (length - d_accel - d_decel) / V if V > 0 else 0.0
            t_d = V / d
            self._phases = []
            if t_a > 1e-9:
                self._phases.append((t_a, a, v0))
            if t_c > 1e-9:
                self._phases.append((t_c, 0.0, V))
            if t_d > 1e-9:
                self._phases.append((t_d, -d, V))
        else:
            # Can't reach V — find peak speed
            # (vp²-v0²)/(2a) + vp²/(2d) = length
            numer = (length + v0 * v0 / (2 * a)) * 2 * a * d / (a + d)
            if numer <= v0 * v0:
                # Must decelerate from v0
                d_stop = v0 * v0 / (2 * d)
                if d_stop <= length:
                    t_c = (length - d_stop) / v0 if v0 > 1e-9 else 0.0
                    t_d = v0 / d
                    self._phases = []
                    if t_c > 1e-9:
                        self._phases.append((t_c, 0.0, v0))
                    self._phases.append((t_d, -d, v0))
                else:
                    disc = v0 * v0 - 2 * d * length
                    t_d = (v0 - math.sqrt(max(0.0, disc))) / d if d > 0 else 0.0
                    self._phases = [(max(t_d, 0.0), -d, v0)]
            else:
                vp = math.sqrt(max(0.0, numer))
                t_a = (vp - v0) / a
                t_d = vp / d
                self._phases = [(t_a, a, v0), (t_d, -d, vp)]

        self.total_time = sum(p[0] for p in self._phases)

    def pos_at(self, t: float) -> float:
        if t <= 0:
            return 0.0
        if t >= self.total_time:
            return self.length
        elapsed = 0.0
        dist = 0.0
        for dur, acc, v0 in self._phases:
            dt = min(t - elapsed, dur)
            if dt <= 0:
                break
            dist += v0 * dt + 0.5 * acc * dt * dt
            elapsed += dur
            if elapsed >= t:
                break
        return min(dist, self.length)

    def vel_at(self, t: float) -> float:
        if t <= 0:
            return self._phases[0][2] if self._phases else 0.0
        if t >= self.total_time:
            return 0.0
        elapsed = 0.0
        for dur, acc, v0 in self._phases:
            if t <= elapsed + dur:
                return max(0.0, v0 + acc * (t - elapsed))
            elapsed += dur
        return 0.0

    def frac_at(self, t: float) -> float:
        return self.pos_at(t) / self.length if self.length > 0 else 1.0

    def time_to_pos(self, target: float) -> float:
        """Time to reach target distance (mm)."""
        if target <= 0:
            return 0.0
        if target >= self.length:
            return self.total_time
        elapsed = 0.0
        dist = 0.0
        for dur, acc, v0 in self._phases:
            end_dist = dist + v0 * dur + 0.5 * acc * dur * dur
            if end_dist >= target:
                need = target - dist
                if abs(acc) < 1e-9:
                    return elapsed + (need / v0 if v0 > 1e-9 else 0.0)
                disc = v0 * v0 + 2 * acc * need
                if disc < 0:
                    return elapsed + dur
                dt = (-v0 + math.sqrt(max(0.0, disc))) / acc
                return elapsed + min(max(0.0, dt), dur)
            dist = end_dist
            elapsed += dur
        return self.total_time


# backward compat alias
TrapezoidProfile = SpeedProfile


# ── Map structures ────────────────────────────────────────────────────────────

class ZoneNode:
    def __init__(self, nid: str, x: float, y: float, is_original: bool = True):
        self.id = nid
        self.x  = x
        self.y  = y
        self.is_original = is_original


def _extract_sub_polyline(pts: List[Tuple[float, float]],
                          total_len: float,
                          frac_s: float, frac_e: float
                          ) -> List[Tuple[float, float]]:
    """Extract the sub-section of a polyline between two fractions."""
    d_start = frac_s * total_len
    d_end   = frac_e * total_len
    sub: List[Tuple[float, float]] = []
    accum = 0.0
    started = False
    for i in range(len(pts) - 1):
        seg_len = math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
        nxt_accum = accum + seg_len
        if not started and nxt_accum >= d_start:
            t = (d_start - accum) / seg_len if seg_len > 0 else 0.0
            sub.append((pts[i][0] + (pts[i+1][0]-pts[i][0])*t,
                        pts[i][1] + (pts[i+1][1]-pts[i][1])*t))
            started = True
        if started:
            if nxt_accum >= d_end:
                t = (d_end - accum) / seg_len if seg_len > 0 else 1.0
                sub.append((pts[i][0] + (pts[i+1][0]-pts[i][0])*t,
                            pts[i][1] + (pts[i+1][1]-pts[i][1])*t))
                break
            sub.append(pts[i+1])
        accum = nxt_accum
    if len(sub) < 2:
        sub = [pts[0], pts[-1]]
    return sub


class ZoneSegment:
    """A zone (block) between two zone-nodes."""

    def __init__(self, from_id: str, to_id: str,
                 length: float, max_speed: float,
                 orig_pts: List[Tuple[float, float]],
                 frac_start: float, frac_end: float):
        self.from_id   = from_id
        self.to_id     = to_id
        self.length    = length
        self.max_speed = max_speed
        # Extract and cache only this zone's portion of the polyline
        total = _polyline_length(orig_pts)
        self._pts = _extract_sub_polyline(orig_pts, total, frac_start, frac_end)
        self._pts_len = _polyline_length(self._pts)

    def interp(self, frac: float) -> Tuple[float, float, float]:
        return _interp_polyline(self._pts, frac, self._pts_len)


# ── Zone map ──────────────────────────────────────────────────────────────────

class ZoneMap:
    """
    Loads KaistTB OHT_A sub-network and subdivides segments into zones
    of length ≈ zone_target (default = h_min).
    """

    def __init__(self, json_path: str, area: str = 'OHT_A',
                 zone_target: float = None):
        with open(json_path, 'r', encoding='utf-8') as f:
            d = json.load(f)

        oht_vm = next((v for v in d.get('vehicleModels', [])
                        if v['id'] == 'OHT'), {})
        dim = oht_vm.get('dimension', {})
        self.vehicle_length = dim.get('length', 1108)
        self.vehicle_width  = dim.get('width',  460)
        self.h_min          = self.vehicle_length + 200
        self.accel          = oht_vm.get('acceleration', 500)
        self.decel          = oht_vm.get('deceleration', 500)

        if zone_target is None:
            zone_target = self.h_min
        self.zone_target = zone_target

        area_ids   = {n['id'] for n in d['nodes'] if n.get('area') == area}
        orig_nodes = {n['id']: (n['x'], n['y'])
                      for n in d['nodes'] if n['id'] in area_ids}

        self.nodes:    Dict[str, ZoneNode]              = {}
        self.segments: Dict[Tuple[str, str], ZoneSegment] = {}
        self.adj:      Dict[str, List[str]]             = {}
        self._edge_expansion: Dict[Tuple[str, str], List[str]] = {}

        for nid, (x, y) in orig_nodes.items():
            self.nodes[nid] = ZoneNode(nid, x, y, is_original=True)
            self.adj[nid] = []

        # ── Detect junction nodes (merge/diverge) ─────────────────────────
        in_deg:  Dict[str, int] = collections.Counter()
        out_deg: Dict[str, int] = collections.Counter()
        orig_adj:  Dict[str, List[str]] = {nid: [] for nid in orig_nodes}
        orig_segs: Dict[Tuple[str, str], dict] = {}

        for s in d['segments']:
            fn, tn = s['startNodeId'], s['endNodeId']
            if fn in area_ids and tn in area_ids:
                in_deg[tn]  += 1
                out_deg[fn] += 1
                orig_adj[fn].append(tn)
                orig_segs[(fn, tn)] = s

        # Junction = merge (in>1) OR diverge (out>1) OR dead-end (in=0 or out=0)
        junction_nodes = set()
        for nid in orig_nodes:
            if (in_deg.get(nid, 0) != 1 or out_deg.get(nid, 0) != 1):
                junction_nodes.add(nid)
        self.zcu_nodes = ({nid for nid, deg in in_deg.items()  if deg > 1} |
                          {nid for nid, deg in out_deg.items() if deg > 1})

        # ── Build linear chains between junctions ─────────────────────────
        # A "chain" is a sequence of nodes where each interior node has
        # exactly in_degree=1, out_degree=1.  Junctions are chain endpoints.
        # Chains are then re-zoned so every zone >= zone_target.
        visited_edges: set = set()
        chains: List[List[str]] = []   # each chain = [n0, n1, n2, ..., nK]

        for start in sorted(junction_nodes):
            for nxt in orig_adj.get(start, []):
                if (start, nxt) in visited_edges:
                    continue
                chain = [start, nxt]
                visited_edges.add((start, nxt))
                cur = nxt
                while cur not in junction_nodes:
                    succs = orig_adj.get(cur, [])
                    if len(succs) != 1:
                        break
                    nxt2 = succs[0]
                    if (cur, nxt2) in visited_edges:
                        break
                    visited_edges.add((cur, nxt2))
                    chain.append(nxt2)
                    cur = nxt2
                chains.append(chain)

        # ── For each chain: concatenate polylines, re-zone at zone_target ─
        for chain in chains:
            # Build concatenated polyline and per-edge info
            full_pts: List[Tuple[float, float]] = []
            edge_speeds: List[float] = []
            edge_cum: List[float] = []   # cumulative length at each edge end
            cum = 0.0

            for k in range(len(chain) - 1):
                fn, tn = chain[k], chain[k + 1]
                s = orig_segs.get((fn, tn))
                if s is None:
                    break
                n1, n2 = orig_nodes[fn], orig_nodes[tn]
                parts = s.get('parts')
                if parts:
                    seg_pts = _build_path_points(parts, n1)
                else:
                    seg_pts = [n1, n2]
                # Append (skip first point to avoid duplicates)
                if not full_pts:
                    full_pts.extend(seg_pts)
                else:
                    full_pts.extend(seg_pts[1:])
                seg_len = _polyline_length(seg_pts)
                cum += seg_len
                edge_cum.append(cum)
                edge_speeds.append(s.get('speed', 1000))

            total_len = cum
            if total_len < 1.0 or not edge_cum:
                # Degenerate chain — create 1 zone per original edge
                for k in range(len(chain) - 1):
                    fn, tn = chain[k], chain[k + 1]
                    s_data = orig_segs.get((fn, tn))
                    if s_data is None:
                        continue
                    n1, n2 = orig_nodes[fn], orig_nodes[tn]
                    seg_len = math.hypot(n2[0]-n1[0], n2[1]-n1[1])
                    zseg = ZoneSegment(fn, tn, seg_len, s_data.get('speed', 1000),
                                       [n1, n2], 0.0, 1.0)
                    self.segments[(fn, tn)] = zseg
                    self.adj.setdefault(fn, []).append(tn)
                    self._edge_expansion[(fn, tn)] = [fn, tn]
                continue

            # Number of zones for the entire chain
            n_zones = max(1, round(total_len / zone_target))
            zone_len = total_len / n_zones

            # Place zone boundaries along the concatenated polyline
            zone_nids = [chain[0]]
            for k in range(1, n_zones):
                frac = k / n_zones
                x, y, _ = _interp_polyline(full_pts, frac)
                zid = f'z_{chain[0]}_{chain[-1]}_{k}'
                self.nodes[zid] = ZoneNode(zid, x, y, is_original=False)
                self.adj[zid] = []
                zone_nids.append(zid)
            zone_nids.append(chain[-1])

            # Average speed for the chain
            avg_speed = (sum(edge_speeds) / len(edge_speeds)
                         if edge_speeds else 1000.0)

            # Create zone segments
            for k in range(n_zones):
                zfrom, zto = zone_nids[k], zone_nids[k + 1]
                frac_s = k / n_zones
                frac_e = (k + 1) / n_zones
                zseg = ZoneSegment(zfrom, zto, zone_len, avg_speed,
                                   full_pts, frac_s, frac_e)
                self.segments[(zfrom, zto)] = zseg
                self.adj.setdefault(zfrom, []).append(zto)

            # Store chain expansion: junction→junction
            self._edge_expansion[(chain[0], chain[-1])] = zone_nids

        self.port_nodes = {p['nodeId'] for p in d.get('ports', [])
                           if p['nodeId'] in area_ids}

        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        pad = self.vehicle_length
        self.bbox = (min(xs)-pad, min(ys)-pad, max(xs)+pad, max(ys)+pad)

        self.n_original_nodes = len(orig_nodes)
        self.n_original_segs  = sum(1 for s in d['segments']
                                    if s['startNodeId'] in area_ids
                                    and s['endNodeId'] in area_ids)

    def expand_path(self, orig_path: List[str]) -> List[str]:
        """Convert original junction-to-junction path to zone path.
        NOTE: Only works when orig_path consists of junction nodes.
        For general paths, use bfs_path() on the zone graph directly.
        """
        if not orig_path:
            return []
        result = [orig_path[0]]
        for i in range(len(orig_path) - 1):
            exp = self._edge_expansion.get((orig_path[i], orig_path[i+1]))
            if exp:
                result.extend(exp[1:])
            else:
                # Might be a direct zone edge (single-segment chain)
                if (orig_path[i], orig_path[i+1]) in self.segments:
                    result.append(orig_path[i+1])
                # else: nodes might not be in zone graph — skip
        return result

    def bfs_path(self, start: str, goal: str = None) -> List[str]:
        if start not in self.adj:
            return [start]
        if goal is None:
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

    def nearby_nodes(self, node_id: str, max_dist: float) -> set:
        if not hasattr(self, '_radj'):
            self._radj: Dict[str, List[str]] = {nid: [] for nid in self.nodes}
            for (fn, tn) in self.segments:
                self._radj.setdefault(tn, []).append(fn)
        visited = {node_id}
        queue   = collections.deque([(node_id, 0.0)])
        while queue:
            cur, dist = queue.popleft()
            for nxt in self.adj.get(cur, []):
                seg = self.segments.get((cur, nxt))
                if seg and dist + seg.length <= max_dist and nxt not in visited:
                    visited.add(nxt)
                    queue.append((nxt, dist + seg.length))
            for nxt in self._radj.get(cur, []):
                seg = self.segments.get((nxt, cur))
                if seg and dist + seg.length <= max_dist and nxt not in visited:
                    visited.add(nxt)
                    queue.append((nxt, dist + seg.length))
        return visited

    def original_node_ids(self) -> List[str]:
        return [nid for nid, n in self.nodes.items() if n.is_original]

    def startable_node_ids(self) -> List[str]:
        """Zone-nodes that have outgoing edges (valid vehicle start positions)."""
        return [nid for nid in self.nodes if self.adj.get(nid)]


# ── Agent ─────────────────────────────────────────────────────────────────────

class ZoneAgent:
    """Vehicle in zone-based system.
    Interface-compatible with OHTAgent for vis_oht rendering."""

    def __init__(self, agent_id: int, color: tuple,
                 zone_path: List[str], max_speed: float = 1000.0):
        self.id        = agent_id
        self.color     = color
        self.node_path = zone_path
        self.zone_path = zone_path
        self.path_idx  = 0
        self.max_speed = max_speed
        self.state     = IDLE

        self.v     = 0.0
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # ── Chain transit state ───────────────────────────────────────────────
        self._profile:    Optional[SpeedProfile] = None
        self._transit_t0: float = 0.0
        self._pos_offset: float = 0.0       # absolute chain pos at profile start

        # chain[k] = zone-node IDs to transit through (destinations, not origin)
        self._chain:      List[str]         = []
        self._chain_segs: List[ZoneSegment] = []
        self._chain_cum:  List[float]       = []   # cumulative length at each dest
        self._chain_idx:  int               = 0    # zone boundaries crossed so far

        self.token     = 0
        self.adv_token = 0

    @property
    def cur_node(self) -> str:
        return self.zone_path[self.path_idx]

    # convenience for position interpolation
    @property
    def _current_seg(self) -> Optional[ZoneSegment]:
        """Active segment for vis_oht position rendering."""
        if not self._chain_segs or self._chain_idx >= len(self._chain_segs):
            return None
        return self._chain_segs[self._chain_idx]


# ── Event ─────────────────────────────────────────────────────────────────────

class _Event:
    __slots__ = ('t', 'kind', 'agent_id', 'token', 'data')

    def __init__(self, t, kind, agent_id, token, data=None):
        self.t = t; self.kind = kind; self.agent_id = agent_id
        self.token = token; self.data = data

    def __lt__(self, other):
        return self.t < other.t


# ── DES Environment ──────────────────────────────────────────────────────────

class ZoneEnvironmentDES:
    """
    Zone-based DES with chain claim and follow-through.

    Parameters
    ──────────
    zone_map       : ZoneMap instance
    early_release  : (unused, kept for API compat — progressive release is always on)
    """

    def __init__(self, zone_map: ZoneMap, early_release: bool = True):
        self.map = zone_map
        self._agents: Dict[int, ZoneAgent] = {}
        self._heap:   List[_Event]         = []

        # Per-zone reservation queue: deque[0] = holder, [1:] = followers
        self._queues: Dict[str, collections.deque] = {
            nid: collections.deque() for nid in zone_map.nodes}

        # Waitlists for vehicles that couldn't join any queue (truly blocked)
        self._waitlists: Dict[str, List[ZoneAgent]] = {
            nid: [] for nid in zone_map.nodes}

        # Derived holder dict for vis_oht compatibility
        self._holders: Dict[str, Optional[ZoneAgent]] = {
            nid: None for nid in zone_map.nodes}

        self.event_count = 0

    # ── Holder sync ───────────────────────────────────────────────────────────

    def _sync_holder(self, nid: str):
        """Update _holders from _queues for vis_oht rendering."""
        q = self._queues.get(nid)
        self._holders[nid] = q[0] if q else None

    # ── Agent management ──────────────────────────────────────────────────────

    def add_agent(self, agent: ZoneAgent, t_start: float = 0.0):
        self._agents[agent.id] = agent
        agent.state     = IDLE
        agent.path_idx  = 0
        agent.token     = 0
        agent.adv_token = 0
        agent._profile  = None
        agent._chain    = []
        agent._chain_segs = []
        agent._chain_cum  = []
        agent._chain_idx  = 0
        agent._pos_offset = 0.0

        start = agent.zone_path[0]
        self._ensure_queue(start)
        q = self._queues[start]
        if agent not in q:
            q.append(agent)
        self._sync_holder(start)

        node = self.map.nodes.get(start)
        if node:
            agent.x, agent.y = node.x, node.y
        # Face the direction of the first segment
        if len(agent.zone_path) > 1:
            seg = self.map.segments.get((agent.zone_path[0], agent.zone_path[1]))
            if seg:
                _, _, theta = seg.interp(0.0)
                agent.theta = theta

        self._post_advance(t_start, agent)

    def remove_agent(self, agent_id: int):
        agent = self._agents.pop(agent_id, None)
        if agent is None:
            return
        # Remove from all queues
        for nid, q in self._queues.items():
            if agent in q:
                q.remove(agent)
                self._sync_holder(nid)
                if not q:
                    self._wake_waiters(0.0, nid)
        for wl in self._waitlists.values():
            if agent in wl:
                wl.remove(agent)

    def _ensure_queue(self, nid: str):
        if nid not in self._queues:
            self._queues[nid] = collections.deque()
            self._waitlists[nid] = []
            self._holders[nid] = None

    # ── Event posting ─────────────────────────────────────────────────────────

    def _post(self, t: float, kind: str, agent: ZoneAgent, data=None):
        tok = agent.adv_token if kind == TRY_CLAIM else agent.token
        heapq.heappush(self._heap, _Event(t, kind, agent.id, tok, data))

    def _post_advance(self, t: float, agent: ZoneAgent):
        agent.adv_token += 1
        self._post(t, TRY_CLAIM, agent)

    # ── Main step ─────────────────────────────────────────────────────────────

    def step(self, t_now: float):
        while self._heap and self._heap[0].t <= t_now:
            ev = heapq.heappop(self._heap)
            agent = self._agents.get(ev.agent_id)
            if agent is None:
                continue

            if ev.kind == TRY_CLAIM:
                if ev.token != agent.adv_token:
                    continue
                self.event_count += 1
                self._on_try_claim(ev.t, agent)
            elif ev.kind == ZONE_PASS:
                if ev.token != agent.token:
                    continue
                self.event_count += 1
                self._on_zone_pass(ev.t, agent, ev.data)
            elif ev.kind == TRANSIT_DONE:
                if ev.token != agent.token:
                    continue
                self.event_count += 1
                self._on_transit_done(ev.t, agent)

        for agent in self._agents.values():
            self._update_pos(t_now, agent)

    # ── Chain building ────────────────────────────────────────────────────────

    def _build_chain(self, agent: ZoneAgent
                     ) -> Tuple[List[str], List[ZoneSegment]]:
        """Build a chain of consecutive FREE zones the agent can claim.

        Chain length is capped at the braking distance from max speed —
        the first zone-node beyond brake_dist is the last one included.
        This prevents vehicles from hogging long stretches of track.
        Remaining zones are claimed incrementally via ZONE_PASS extension.
        """
        chain_nids: List[str]         = []
        chain_segs: List[ZoneSegment] = []

        v_max = agent.max_speed
        brake_dist = v_max * v_max / (2.0 * self.map.decel)
        cum_len = 0.0
        past_brake = False

        for i in range(agent.path_idx + 1, len(agent.zone_path)):
            nid  = agent.zone_path[i]
            prev = agent.zone_path[i - 1]
            seg  = self.map.segments.get((prev, nid))
            if seg is None:
                break

            self._ensure_queue(nid)
            q = self._queues[nid]

            if not q:
                q.append(agent)
                self._sync_holder(nid)
                chain_nids.append(nid)
                chain_segs.append(seg)
                cum_len += seg.length
                # Stop after the first zone that pushes us past brake_dist
                if cum_len >= brake_dist:
                    past_brake = True
                    break
                continue

            if q[0] is agent:
                chain_nids.append(nid)
                chain_segs.append(seg)
                cum_len += seg.length
                if cum_len >= brake_dist:
                    break
                continue

            # Blocked — register for notification, stop
            if agent not in q:
                q.append(agent)
                self._sync_holder(nid)
            break

        return chain_nids, chain_segs

    # ── Chain transit ─────────────────────────────────────────────────────────

    def _start_chain(self, t: float, agent: ZoneAgent,
                     chain_nids: List[str], chain_segs: List[ZoneSegment],
                     v_start: float = 0.0):
        """Start (or restart) continuous transit through a chain of zones."""
        agent._chain      = chain_nids
        agent._chain_segs = chain_segs
        agent._chain_idx  = 0
        agent._pos_offset = 0.0
        agent._transit_t0 = t
        agent.state       = MOVING
        agent.token      += 1

        # Cumulative lengths
        cum = 0.0
        agent._chain_cum = []
        for seg in chain_segs:
            cum += seg.length
            agent._chain_cum.append(cum)

        total = cum
        v_max = min(agent.max_speed,
                    min(s.max_speed for s in chain_segs)) if chain_segs else 0.0

        agent._profile = SpeedProfile(total, v_max,
                                      self.map.accel, self.map.decel,
                                      v_start=v_start)

        # Schedule TRANSIT_DONE
        self._post(t + agent._profile.total_time, TRANSIT_DONE, agent)

        # Schedule ZONE_PASS at each intermediate boundary
        for k in range(len(agent._chain_cum) - 1):
            t_pass = agent._profile.time_to_pos(agent._chain_cum[k])
            self._post(t + t_pass, ZONE_PASS, agent, data=k)

    # ── TRY_CLAIM ─────────────────────────────────────────────────────────────

    def _on_try_claim(self, t: float, agent: ZoneAgent):
        idx = agent.path_idx
        if idx + 1 >= len(agent.zone_path):
            agent.state = DONE
            agent.v     = 0.0
            return

        chain_nids, chain_segs = self._build_chain(agent)

        if not chain_nids:
            # Truly blocked — register in waitlist for next zone
            next_nid = agent.zone_path[idx + 1]
            agent.state = BLOCKED
            agent.v     = 0.0
            self._ensure_queue(next_nid)
            wl = self._waitlists[next_nid]
            if agent not in wl:
                wl.append(agent)
            return

        self._start_chain(t, agent, chain_nids, chain_segs, v_start=0.0)

    # ── ZONE_PASS ─────────────────────────────────────────────────────────────

    def _on_zone_pass(self, t: float, agent: ZoneAgent, boundary_k: int):
        """Agent crossed zone boundary *boundary_k* within its chain.
        boundary_k corresponds to arriving at chain[boundary_k].
        All zones in the chain are owned (holder = agent), so no emergency
        stop is needed — the chain only contains directly-claimed zones.
        """
        if boundary_k >= len(agent._chain):
            return

        # Update chain progress
        agent._chain_idx = boundary_k + 1
        agent.path_idx  += 1

        # Release the zone we just LEFT
        if boundary_k == 0:
            # Left the starting node
            start_nid = agent.zone_path[agent.path_idx - 1]
            self._release_zone(t, start_nid, agent)
        else:
            prev_nid = agent._chain[boundary_k - 1]
            self._release_zone(t, prev_nid, agent)

        # Try to extend chain
        self._try_extend(t, agent)

    # ── TRANSIT_DONE ──────────────────────────────────────────────────────────

    def _on_transit_done(self, t: float, agent: ZoneAgent):
        if not agent._chain:
            return

        # Release the zone before the last
        if len(agent._chain) >= 2:
            prev_nid = agent._chain[-2]
            self._release_zone(t, prev_nid, agent)
        elif agent.path_idx > 0:
            prev_nid = agent.zone_path[agent.path_idx - 1]
            self._release_zone(t, prev_nid, agent)

        # Advance path_idx to chain end
        agent.path_idx = agent.path_idx + len(agent._chain) - agent._chain_idx
        agent._profile    = None
        agent._chain      = []
        agent._chain_segs = []
        agent._chain_cum  = []
        agent._chain_idx  = 0
        agent.v           = 0.0
        agent.state       = IDLE

        # Try to start a new chain immediately
        self._post_advance(t, agent)

    # ── Chain extension ───────────────────────────────────────────────────────

    def _try_extend(self, t: float, agent: ZoneAgent):
        """Try to extend agent's current chain with more claimable zones.
        Extension is also capped at braking distance from current speed."""
        if not agent._chain or agent.state != MOVING:
            return

        chain_end_path_idx = agent.path_idx + (len(agent._chain) - agent._chain_idx)
        if chain_end_path_idx + 1 >= len(agent.zone_path):
            return

        # Remaining claimed distance from current position
        elapsed = t - agent._transit_t0
        pos_now = agent._pos_offset + (agent._profile.pos_at(elapsed)
                                        if agent._profile else 0.0)
        v_now   = agent._profile.vel_at(elapsed) if agent._profile else 0.0
        claimed_ahead = (agent._chain_cum[-1] if agent._chain_cum else 0.0) - pos_now
        brake_dist = v_now * v_now / (2.0 * self.map.decel) if v_now > 0 else 0.0
        # Use max-speed brake distance as the cap
        brake_cap = agent.max_speed ** 2 / (2.0 * self.map.decel)

        if claimed_ahead >= brake_cap:
            return   # already have enough claimed ahead

        new_nids: List[str]         = []
        new_segs: List[ZoneSegment] = []
        ext_len = 0.0

        for i in range(chain_end_path_idx + 1, len(agent.zone_path)):
            nid  = agent.zone_path[i]
            prev = agent.zone_path[i - 1]
            seg  = self.map.segments.get((prev, nid))
            if seg is None:
                break

            self._ensure_queue(nid)
            q = self._queues[nid]

            if not q:
                q.append(agent)
                self._sync_holder(nid)
                new_nids.append(nid)
                new_segs.append(seg)
                ext_len += seg.length
                if claimed_ahead + ext_len >= brake_cap:
                    break
            elif q[0] is agent:
                new_nids.append(nid)
                new_segs.append(seg)
                ext_len += seg.length
                if claimed_ahead + ext_len >= brake_cap:
                    break
            else:
                if agent not in q:
                    q.append(agent)
                    self._sync_holder(nid)
                break

        if not new_nids:
            return

        # Extend chain
        agent._chain.extend(new_nids)
        agent._chain_segs.extend(new_segs)
        old_total = agent._chain_cum[-1] if agent._chain_cum else 0.0
        for seg in new_segs:
            old_total += seg.length
            agent._chain_cum.append(old_total)

        # Rebuild profile from current speed
        elapsed  = t - agent._transit_t0
        v_now    = agent._profile.vel_at(elapsed) if agent._profile else 0.0
        pos_now  = (agent._pos_offset +
                    (agent._profile.pos_at(elapsed) if agent._profile else 0.0))
        remaining = agent._chain_cum[-1] - pos_now

        v_max = min(agent.max_speed,
                    min(s.max_speed for s in agent._chain_segs))

        agent._pos_offset = pos_now
        agent._transit_t0 = t
        agent._profile = SpeedProfile(remaining, v_max,
                                      self.map.accel, self.map.decel,
                                      v_start=v_now)
        agent.token += 1   # invalidate old TRANSIT_DONE and ZONE_PASS

        # Schedule new TRANSIT_DONE
        self._post(t + agent._profile.total_time, TRANSIT_DONE, agent)

        # Schedule new ZONE_PASS for remaining boundaries
        for k in range(agent._chain_idx, len(agent._chain_cum) - 1):
            cum = agent._chain_cum[k]
            if cum > pos_now:
                t_pass = agent._profile.time_to_pos(cum - pos_now)
                self._post(t + t_pass, ZONE_PASS, agent, data=k)

    # ── Zone release ──────────────────────────────────────────────────────────

    def _release_zone(self, t: float, nid: str, agent: ZoneAgent):
        """Remove agent from zone queue.  Promote next holder or wake waiters."""
        q = self._queues.get(nid)
        if q is None:
            return
        if agent in q:
            q.remove(agent)
        self._sync_holder(nid)

        if q:
            new_front = q[0]
            if new_front.state == BLOCKED:
                # Was emergency-stopped or waiting — restart
                wl = self._waitlists.get(nid, [])
                if new_front in wl:
                    wl.remove(new_front)
                self._post_advance(t, new_front)
            elif new_front.state == IDLE:
                # Waiting at previous node — restart
                self._post_advance(t, new_front)
            elif new_front.state == MOVING and new_front._chain:
                # In transit approaching this zone — extend chain to include it
                self._try_extend(t, new_front)
        else:
            self._wake_waiters(t, nid)

    def _wake_waiters(self, t: float, nid: str):
        wl = self._waitlists.get(nid, [])
        while wl:
            waiter = wl.pop(0)
            if waiter.id in self._agents:
                self._post_advance(t, waiter)
                break

    # ── Position update ───────────────────────────────────────────────────────

    def _update_pos(self, t: float, agent: ZoneAgent):
        if agent._profile and agent._chain_segs:
            elapsed = t - agent._transit_t0
            abs_pos = agent._pos_offset + agent._profile.pos_at(elapsed)
            agent.v = agent._profile.vel_at(elapsed)

            # Find which segment we're in
            for k in range(len(agent._chain_cum)):
                if abs_pos <= agent._chain_cum[k] + 1e-3:
                    seg = agent._chain_segs[k]
                    seg_start = agent._chain_cum[k] - seg.length
                    local_frac = ((abs_pos - seg_start) / seg.length
                                  if seg.length > 0 else 1.0)
                    local_frac = max(0.0, min(1.0, local_frac))
                    x, y, theta = seg.interp(local_frac)
                    agent.x, agent.y, agent.theta = x, y, theta
                    return
            # Past end — at last chain node
            if agent._chain:
                node = self.map.nodes.get(agent._chain[-1])
                if node:
                    agent.x, agent.y = node.x, node.y
        else:
            node = self.map.nodes.get(agent.cur_node)
            if node:
                agent.x, agent.y = node.x, node.y
            agent.v = 0.0

    # ── Public helpers ────────────────────────────────────────────────────────

    def reassign(self, agent: ZoneAgent, zone_path: List[str], t: float):
        # Remove from all queues
        for nid, q in self._queues.items():
            if agent in q:
                q.remove(agent)
                self._sync_holder(nid)
                if not q:
                    self._wake_waiters(t, nid)
        for wl in self._waitlists.values():
            if agent in wl:
                wl.remove(agent)

        agent.zone_path = zone_path
        agent.node_path = zone_path
        agent.path_idx  = 0
        agent.state     = IDLE
        agent._profile  = None
        agent._chain    = []
        agent._chain_segs = []
        agent._chain_cum  = []
        agent._chain_idx  = 0
        agent._pos_offset = 0.0
        agent.token    += 1

        start = zone_path[0]
        self._ensure_queue(start)
        q = self._queues[start]
        if agent not in q:
            q.append(agent)
        self._sync_holder(start)

        node = self.map.nodes.get(start)
        if node:
            agent.x, agent.y = node.x, node.y
        if len(zone_path) > 1:
            seg = self.map.segments.get((zone_path[0], zone_path[1]))
            if seg:
                _, _, theta = seg.interp(0.0)
                agent.theta = theta

        self._post_advance(t, agent)

    @property
    def agents(self) -> List[ZoneAgent]:
        return list(self._agents.values())

    def all_done(self) -> bool:
        return all(a.state == DONE for a in self._agents.values())

    def node_utilization(self) -> float:
        held = sum(1 for q in self._queues.values() if q)
        return held / len(self._queues) if self._queues else 0.0
