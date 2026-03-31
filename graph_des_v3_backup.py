"""
graph_des_v3.py — Graph-based DES engine for OHT networks.

Extends circular_des_v3.py approach to work with arbitrary OHT maps.

Key principles (from circular_des_v3):
- _plan() single decision function using v_safe = sqrt(2*d_max*free_dist)
- RESUME events at braking points
- STOPPED events when vel=0
- _notify_followers() for state changes
- pos_at(t)/vel_at(t) with v_max clamping (both accel and decel)
- stop_pos always computed (never None while moving)

Graph extensions:
- Vehicle has path: List[str] (node IDs) and path_idx: int
- Position = (segment index in path, offset along segment)
- Gap computation follows the vehicle's path through multiple segments
- Seamless segment transitions (no stopping at nodes unless needed)
- Loading/unloading: probabilistic at node arrivals
"""

from __future__ import annotations
import math, heapq, random, json, collections
from typing import Dict, List, Optional, Tuple


# ── Map structures ────────────────────────────────────────────────────────────

class ZCUZone:
    """Zone Control Unit at a merge or diverge point.
    Simple occupancy-based: checks _seg_occupants to decide if blocked.
    Integrated into _plan via free_dist limitation."""

    def __init__(self, node_id: str, kind: str,
                 curve_segs: set, straight_segs: set):
        self.node_id = node_id
        self.kind = kind
        self.curve_segs = curve_segs
        self.straight_segs = straight_segs

    def is_curve(self, seg_key: Tuple[str, str]) -> bool:
        return seg_key in self.curve_segs

    def all_segs(self) -> set:
        return self.curve_segs | self.straight_segs


class MapNode:
    __slots__ = ('id', 'x', 'y')
    def __init__(self, nid: str, x: float, y: float):
        self.id = nid
        self.x = x
        self.y = y


class MapSegment:
    __slots__ = ('id', 'from_id', 'to_id', 'length', 'max_speed', 'path_points', '_curve_group')
    def __init__(self, sid, from_id: str, to_id: str, length: float,
                 max_speed: float = 2000.0,
                 path_points: List[Tuple[float, float]] = None):
        self.id = sid
        self.from_id = from_id
        self.to_id = to_id
        self.length = length
        self.max_speed = max_speed
        self.path_points = path_points or []
        self._curve_group = None


def _build_path_points(parts: list, start_xy: Tuple[float, float],
                       n_arc: int = 16) -> List[Tuple[float, float]]:
    """Convert segment parts (Straight / Arc) into a dense polyline."""
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
    """Given a polyline and a distance along it, return (x, y, theta)."""
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


# ── Graph Map ─────────────────────────────────────────────────────────────────

class GraphMap:
    """Loads an xmsmap-v4.0 JSON and provides graph traversal."""

    def __init__(self, json_path: str):
        with open(json_path, 'r', encoding='utf-8') as f:
            d = json.load(f)

        self.nodes: Dict[str, MapNode] = {
            n['id']: MapNode(n['id'], n['x'], n['y'])
            for n in d['nodes']
        }

        self.segments: Dict[Tuple[str, str], MapSegment] = {}
        self.adj: Dict[str, List[str]] = {nid: [] for nid in self.nodes}

        for s in d['segments']:
            fn, tn = s['startNodeId'], s['endNodeId']
            if fn not in self.nodes or tn not in self.nodes:
                continue
            n1, n2 = self.nodes[fn], self.nodes[tn]
            start_xy = (n1.x, n1.y)
            parts = s.get('parts')
            if parts:
                path_pts = _build_path_points(parts, start_xy)
                length = _polyline_length(path_pts)
            else:
                path_pts = [start_xy, (n2.x, n2.y)]
                length = math.hypot(n2.x - n1.x, n2.y - n1.y)
            if length < 0.1:
                continue
            seg = MapSegment(s['id'], fn, tn, length,
                             s.get('speed', 2000), path_pts)
            self.segments[(fn, tn)] = seg
            self.adj[fn].append(tn)

        # Reverse adjacency
        self.adj_rev: Dict[str, List[str]] = collections.defaultdict(list)
        for (fn, tn) in self.segments:
            self.adj_rev[tn].append(fn)

        # Split U-turn curves at midpoint (needs adj_rev)
        self._split_uturns()

        # Rebuild adj_rev after split
        self.adj_rev = collections.defaultdict(list)
        for (fn, tn) in self.segments:
            self.adj_rev[tn].append(fn)

        # Compute main loop (largest SCC) for safe path generation
        self.main_loop = self._compute_main_scc()

        # Vehicle params from JSON
        oht_vm = next((v for v in d.get('vehicleModels', []) if v['id'] == 'OHT'), {})
        dim = oht_vm.get('dimension', {})
        self.vehicle_length = dim.get('length', 750)
        self.vehicle_width = dim.get('width', 500)

        # Bounding box
        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        pad = 2000
        self.bbox = (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)

        # Port nodes
        self.port_nodes = {
            p['nodeId'] for p in d.get('ports', [])
            if p.get('nodeId') in self.nodes
        }

        # ── ZCU: classify merge/diverge segments ─────────────────────────
        self._build_zcu()

    def _is_uturn(self, fn: str, tn: str) -> bool:
        """Check if a curve segment is a U-turn (direction reversal > 120 degrees)."""
        # Incoming straight direction to fn
        in_dirs = []
        for pred in self.adj_rev.get(fn, []):
            seg = self.segments.get((pred, fn))
            if seg and len(seg.path_points) <= 2:
                pn = self.nodes[pred]
                n = self.nodes[fn]
                dx, dy = n.x - pn.x, n.y - pn.y
                l = math.hypot(dx, dy)
                if l > 1:
                    in_dirs.append((dx / l, dy / l))
        # Outgoing straight direction from tn
        out_dirs = []
        for succ in self.adj.get(tn, []):
            seg = self.segments.get((tn, succ))
            if seg and len(seg.path_points) <= 2:
                n = self.nodes[tn]
                sn = self.nodes[succ]
                dx, dy = sn.x - n.x, sn.y - n.y
                l = math.hypot(dx, dy)
                if l > 1:
                    out_dirs.append((dx / l, dy / l))
        if not in_dirs or not out_dirs:
            return False
        dot = in_dirs[0][0] * out_dirs[0][0] + in_dirs[0][1] * out_dirs[0][1]
        angle = math.degrees(math.acos(max(-1, min(1, dot))))
        return angle > 120

    def _split_uturns(self):
        """Split U-turn curve segments at midpoint, creating a mid node.
        S-curves are NOT split (no physical space to stop mid-shortcut)."""
        to_split = []
        for (fn, tn), seg in list(self.segments.items()):
            if seg.path_points and len(seg.path_points) > 2:
                if self._is_uturn(fn, tn):
                    to_split.append((fn, tn, seg))

        self.curve_groups = collections.defaultdict(list)
        mid_id = 0
        for fn, tn, seg in to_split:
            pts = seg.path_points
            total_len = _polyline_length(pts)
            if total_len < 100:
                continue

            # Find midpoint
            half = total_len / 2.0
            accum = 0.0
            mid_x, mid_y = pts[-1]
            split_idx = len(pts) - 1
            for i in range(len(pts) - 1):
                dx = pts[i + 1][0] - pts[i][0]
                dy = pts[i + 1][1] - pts[i][1]
                sl = math.hypot(dx, dy)
                if accum + sl >= half:
                    t = (half - accum) / sl if sl > 0 else 0.5
                    mid_x = pts[i][0] + dx * t
                    mid_y = pts[i][1] + dy * t
                    split_idx = i + 1
                    break
                accum += sl

            # Create mid node
            mid_nid = f"_curve_mid_{mid_id}"
            mid_id += 1
            self.nodes[mid_nid] = MapNode(mid_nid, mid_x, mid_y)
            self.adj[mid_nid] = []
            self.adj_rev[mid_nid] = []

            # Split path_points
            pts_first = list(pts[:split_idx]) + [(mid_x, mid_y)]
            pts_second = [(mid_x, mid_y)] + list(pts[split_idx:])
            len_first = _polyline_length(pts_first)
            len_second = _polyline_length(pts_second)

            # Remove original
            del self.segments[(fn, tn)]
            self.adj[fn].remove(tn)
            self.adj_rev[tn].remove(fn)

            # Create two new segments
            seg1 = MapSegment(f"{seg.id}_a", fn, mid_nid, len_first,
                              seg.max_speed, pts_first)
            seg2 = MapSegment(f"{seg.id}_b", mid_nid, tn, len_second,
                              seg.max_speed, pts_second)
            self.segments[(fn, mid_nid)] = seg1
            self.segments[(mid_nid, tn)] = seg2
            self.adj[fn].append(mid_nid)
            self.adj[mid_nid].append(tn)
            self.adj_rev[mid_nid].append(fn)
            self.adj_rev[tn].append(mid_nid)

            # Curve group
            gid = f"_cg_{mid_id - 1}"
            seg1._curve_group = gid
            seg2._curve_group = gid
            self.curve_groups[gid].append((fn, mid_nid))
            self.curve_groups[gid].append((mid_nid, tn))

        if to_split:
            print(f"U-turn splits: {len(to_split)} -> "
                  f"{len(self.nodes)} nodes, {len(self.segments)} segments")

    def _is_curve_seg(self, fn: str, tn: str) -> bool:
        seg = self.segments.get((fn, tn))
        return seg is not None and seg.path_points and len(seg.path_points) > 2

    def _build_zcu(self):
        """Build ZCU zones at merge/diverge points.

        Each zone has:
        - node_id: the merge/diverge node
        - curve_segs: set of (fn,tn) curve segments
        - straight_segs: set of (fn,tn) straight segments
        - kind: 'merge' or 'diverge'
        """
        self.zcu_zones: List[ZCUZone] = []
        self.seg_to_zone: Dict[Tuple[str,str], ZCUZone] = {}  # segment → zone
        self.merge_nodes = set()
        self.diverge_nodes = set()
        self.merge_curve_entries = set()
        self.merge_straight_entries = set()
        self.diverge_curve_exits = set()
        self.diverge_straight_exits = set()

        # Merge zones
        for nid in self.nodes:
            preds = self.adj_rev.get(nid, [])
            if len(preds) < 2:
                continue
            entries = [(p, nid) for p in preds if (p, nid) in self.segments]
            if len(entries) < 2:
                continue
            curves = [k for k in entries if self._is_curve_seg(*k)]
            straights = [k for k in entries if not self._is_curve_seg(*k)]
            if not curves:
                continue

            zone = ZCUZone(nid, 'merge', set(curves), set(straights))
            self.zcu_zones.append(zone)
            self.merge_nodes.add(nid)
            for k in curves:
                self.seg_to_zone[k] = zone
                self.merge_curve_entries.add(k)
            for k in straights:
                self.seg_to_zone[k] = zone
                self.merge_straight_entries.add(k)

        # Diverge zones — any node with out-degree >= 2
        for nid in self.nodes:
            succs = self.adj.get(nid, [])
            if len(succs) < 2:
                continue
            exits = [(nid, s) for s in succs if (nid, s) in self.segments]
            if len(exits) < 2:
                continue
            curves = [k for k in exits if self._is_curve_seg(*k)]
            straights = [k for k in exits if not self._is_curve_seg(*k)]

            zone = ZCUZone(nid, 'diverge', set(curves), set(straights))
            self.zcu_zones.append(zone)
            self.diverge_nodes.add(nid)
            for k in curves:
                self.seg_to_zone[k] = zone
                self.diverge_curve_exits.add(k)
            for k in straights:
                self.seg_to_zone[k] = zone
                self.diverge_straight_exits.add(k)

        print(f"ZCU: {len(self.merge_nodes)} merge, "
              f"{len(self.diverge_nodes)} diverge, "
              f"{len(self.zcu_zones)} zones total")

    def _compute_main_scc(self) -> set:
        """Kosaraju's algorithm to find the largest strongly connected component."""
        order = []
        vis = set()
        for start in self.nodes:
            if start in vis:
                continue
            stack = [(start, False)]
            while stack:
                node, done = stack.pop()
                if done:
                    order.append(node)
                    continue
                if node in vis:
                    continue
                vis.add(node)
                stack.append((node, True))
                for nb in self.adj.get(node, []):
                    if nb not in vis:
                        stack.append((nb, False))

        radj = collections.defaultdict(list)
        for (fn, tn) in self.segments:
            radj[tn].append(fn)

        vis2 = set()
        largest = []
        for node in reversed(order):
            if node in vis2:
                continue
            comp = []
            stack = [node]
            while stack:
                n = stack.pop()
                if n in vis2:
                    continue
                vis2.add(n)
                comp.append(n)
                for nb in radj.get(n, []):
                    if nb not in vis2:
                        stack.append(nb)
            if len(comp) > len(largest):
                largest = comp
        return set(largest)

    def segment_between(self, from_id: str, to_id: str) -> Optional[MapSegment]:
        return self.segments.get((from_id, to_id))


def random_safe_path(gmap: GraphMap, start_node: str, length: int = 200) -> List[str]:
    """Generate a random walk staying within the main loop (SCC)."""
    main = gmap.main_loop
    path = [start_node]
    cur = start_node
    for _ in range(length):
        neighbors = gmap.adj.get(cur, [])
        safe = [n for n in neighbors if n in main]
        if not safe:
            break
        visited = set(path[-20:])
        unvisited = [n for n in safe if n not in visited]
        nxt = random.choice(unvisited) if unvisited else random.choice(safe)
        path.append(nxt)
        cur = nxt
    return path


# ── Vehicle states ────────────────────────────────────────────────────────────

IDLE    = 'IDLE'
ACCEL   = 'ACCEL'
CRUISE  = 'CRUISE'
DECEL   = 'DECEL'
STOP    = 'STOP'
LOADING = 'LOADING'


# ── Vehicle ───────────────────────────────────────────────────────────────────

class Vehicle:
    def __init__(self, vid: int, gmap: GraphMap, path: List[str], color=(200, 200, 200)):
        self.id = vid
        self.gmap = gmap
        self.color = color

        # Path through graph
        self.path: List[str] = path      # node IDs
        self.path_idx: int = 0           # index into path (current segment = path[path_idx] -> path[path_idx+1])

        # Position within current segment
        self.seg_offset: float = 0.0     # mm along current segment

        # Kinematic state — piecewise constant acceleration
        self.vel: float = 0.0            # current speed (mm/s)
        self.acc: float = 0.0            # current acceleration (mm/s^2)
        self.t_ref: float = 0.0          # reference time

        # Physics params
        self.v_max: float = 3600.0       # mm/s
        self.a_max: float = 500.0        # mm/s^2
        self.d_max: float = 500.0        # mm/s^2
        self.length: float = 750.0       # mm
        self.h_min: float = self.length + 400  # mm

        # State
        self.state: str = IDLE
        self.leader: Optional[Vehicle] = None
        self.token: int = 0

        # Stop position (segment-based)
        self.stop_dist: Optional[float] = None  # distance ahead to stop point
        self.x_marker_abs: float = 0.0         # absolute path distance of X marker (monotonic)

        # Render cache
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self.gap_to_leader: float = float('inf')

        # Precompute segment lengths along path for fast lookup
        self._seg_lengths: List[float] = []
        self._seg_speeds: List[float] = []
        self._rebuild_seg_cache()

    def _rebuild_seg_cache(self):
        """Cache segment lengths and speeds for the current path."""
        self._seg_lengths = []
        self._seg_speeds = []
        for i in range(len(self.path) - 1):
            seg = self.gmap.segment_between(self.path[i], self.path[i + 1])
            if seg:
                self._seg_lengths.append(seg.length)
                self._seg_speeds.append(seg.max_speed)
            else:
                self._seg_lengths.append(0.0)
                self._seg_speeds.append(self.v_max)

    @property
    def seg_from(self) -> str:
        if self.path_idx < len(self.path):
            return self.path[self.path_idx]
        return self.path[-1]

    @property
    def seg_to(self) -> str:
        if self.path_idx + 1 < len(self.path):
            return self.path[self.path_idx + 1]
        return self.path[-1]

    def current_segment(self) -> Optional[MapSegment]:
        return self.gmap.segment_between(self.seg_from, self.seg_to)

    def current_seg_length(self) -> float:
        if self.path_idx < len(self._seg_lengths):
            return self._seg_lengths[self.path_idx]
        seg = self.current_segment()
        return seg.length if seg else 0.0

    def current_seg_speed(self) -> float:
        if self.path_idx < len(self._seg_speeds):
            return self._seg_speeds[self.path_idx]
        return self.v_max

    # ── Kinematics (constant acceleration with v_max/0 clamping) ──────────

    def _dist_traveled(self, dt: float) -> float:
        """Distance traveled in dt seconds from current kinematic state."""
        if dt <= 0:
            return 0.0
        if self.acc > 0:
            t_cap = max(0, (self.v_max - self.vel) / self.acc) if self.vel < self.v_max else 0
            if dt <= t_cap:
                return self.vel * dt + 0.5 * self.acc * dt ** 2
            else:
                d_cap = self.vel * t_cap + 0.5 * self.acc * t_cap ** 2
                return d_cap + self.v_max * (dt - t_cap)
        elif self.acc < 0:
            t_stop = -self.vel / self.acc if self.acc != 0 else float('inf')
            if dt <= t_stop:
                return self.vel * dt + 0.5 * self.acc * dt ** 2
            else:
                return self.vel * t_stop + 0.5 * self.acc * t_stop ** 2
        else:
            return self.vel * dt

    def vel_at(self, t: float) -> float:
        dt = t - self.t_ref
        if dt < 0:
            dt = 0
        if self.acc > 0:
            return min(self.v_max, self.vel + self.acc * dt)
        elif self.acc < 0:
            return max(0.0, self.vel + self.acc * dt)
        return self.vel

    def dist_at(self, t: float) -> float:
        """Distance traveled since t_ref."""
        return self._dist_traveled(t - self.t_ref)

    def advance_position(self, t: float):
        """Move seg_offset forward by distance traveled since t_ref,
        advancing path_idx as needed. Updates vel, t_ref."""
        dist = self.dist_at(t)
        self.vel = self.vel_at(t)
        self.t_ref = t
        self.seg_offset += dist
        # Advance through segments
        while self.path_idx < len(self.path) - 1:
            seg_len = self.current_seg_length()
            if seg_len <= 0:
                self.path_idx += 1
                self.seg_offset = 0.0
                continue
            if self.seg_offset >= seg_len - 0.01:
                self.seg_offset -= seg_len
                self.path_idx += 1
                if self.seg_offset < 0:
                    self.seg_offset = 0.0
            else:
                break

    def set_state(self, t: float):
        """Snapshot kinematic state at time t, advancing position."""
        self.advance_position(t)
        self.acc = 0.0  # will be set by _plan

    def abs_path_dist(self) -> float:
        """Cumulative distance traveled along the path (for monotonic X marker)."""
        d = 0.0
        for i in range(min(self.path_idx, len(self._seg_lengths))):
            d += self._seg_lengths[i]
        d += self.seg_offset
        return d

    def braking_distance(self, from_vel: float = -1) -> float:
        v = from_vel if from_vel >= 0 else self.vel
        if v <= 0:
            return 0.0
        return v * v / (2 * self.d_max)

    def dist_ahead_on_path(self, n_segs: int = 50) -> float:
        """Total distance remaining on the path from current position."""
        if self.path_idx >= len(self.path) - 1:
            return 0.0
        total = self.current_seg_length() - self.seg_offset
        for i in range(self.path_idx + 1, min(self.path_idx + n_segs, len(self.path) - 1)):
            if i < len(self._seg_lengths):
                total += self._seg_lengths[i]
            else:
                seg = self.gmap.segment_between(self.path[i], self.path[i + 1])
                if seg:
                    total += seg.length
        return total

    def update_render(self, t: float):
        """Set x, y, theta from current position using segment path_points."""
        # Calculate where we are without mutating state
        dist = self._dist_traveled(t - self.t_ref)
        offset = self.seg_offset + dist
        pidx = self.path_idx

        # Walk through segments to find the right one
        while pidx < len(self.path) - 1:
            if pidx < len(self._seg_lengths):
                seg_len = self._seg_lengths[pidx]
            else:
                seg = self.gmap.segment_between(self.path[pidx], self.path[pidx + 1])
                seg_len = seg.length if seg else 0.0
            if seg_len <= 0:
                pidx += 1
                offset = 0.0
                continue
            if offset >= seg_len - 0.01 and pidx < len(self.path) - 2:
                offset -= seg_len
                pidx += 1
                if offset < 0:
                    offset = 0.0
            else:
                break

        # Interpolate within segment
        if pidx < len(self.path) - 1:
            seg = self.gmap.segment_between(self.path[pidx], self.path[pidx + 1])
            if seg and seg.path_points:
                self.x, self.y, self.theta = _interp_path(seg.path_points, max(0, offset))
                return

        # Fallback to node position
        nid = self.path[min(pidx, len(self.path) - 1)]
        node = self.gmap.nodes.get(nid)
        if node:
            self.x, self.y = node.x, node.y

    def needs_path_extension(self) -> bool:
        """True if within 10 segments of path end."""
        return self.path_idx >= len(self.path) - 10

    def extend_path(self, new_nodes: List[str]):
        """Append new nodes to path and rebuild cache."""
        # Trim already-passed nodes (keep current and a small buffer)
        if self.path_idx > 5:
            trim = self.path_idx - 2
            self.path = self.path[trim:]
            self.path_idx -= trim
            self._seg_lengths = self._seg_lengths[trim:]
            self._seg_speeds = self._seg_speeds[trim:]

        # Append (skip duplicate start)
        if new_nodes and new_nodes[0] == self.path[-1]:
            new_nodes = new_nodes[1:]
        self.path.extend(new_nodes)

        # Extend cache
        old_len = len(self._seg_lengths)
        for i in range(old_len, len(self.path) - 1):
            seg = self.gmap.segment_between(self.path[i], self.path[i + 1])
            if seg:
                self._seg_lengths.append(seg.length)
                self._seg_speeds.append(seg.max_speed)
            else:
                self._seg_lengths.append(0.0)
                self._seg_speeds.append(self.v_max)


# ── Events ────────────────────────────────────────────────────────────────────

class Event:
    __slots__ = ('t', 'kind', 'vid', 'token')
    def __init__(self, t: float, kind: str, vid: int, token: int):
        self.t = t
        self.kind = kind
        self.vid = vid
        self.token = token

    def __lt__(self, other):
        return self.t < other.t


# ── DES Engine ────────────────────────────────────────────────────────────────

class GraphDESv3:
    def __init__(self, gmap: GraphMap, stop_prob: float = 0.02,
                 stop_dur: Tuple[float, float] = (2.0, 5.0)):
        self.gmap = gmap
        self.vehicles: Dict[int, Vehicle] = {}
        self.heap: List[Event] = []
        self.event_count: int = 0
        self.stop_prob = stop_prob
        self.stop_dur = stop_dur
        self.stops_executed: int = 0

        # Segment occupancy: (from_id, to_id) -> list of vehicles on that segment
        self._seg_occupants: Dict[Tuple[str, str], List[Vehicle]] = collections.defaultdict(list)

    def add_vehicle(self, v: Vehicle):
        self.vehicles[v.id] = v
        key = (v.seg_from, v.seg_to)
        self._seg_occupants[key].append(v)

    def _post(self, t: float, kind: str, v: Vehicle):
        heapq.heappush(self.heap, Event(t, kind, v.id, v.token))

    def _invalidate(self, v: Vehicle):
        v.token += 1

    # ── Gap computation (multi-hop) ───────────────────────────────────────

    def gap(self, follower: Vehicle, t: float) -> Tuple[float, float]:
        """Compute (gap_mm, leader_vel) from follower to its leader.
        Follows the follower's path forward to find leader's position."""
        leader = follower.leader
        if leader is None:
            return float('inf'), 0.0

        # Same segment?
        f_pidx = follower.path_idx
        f_off = follower.seg_offset + follower._dist_traveled(t - follower.t_ref)
        l_pidx = leader.path_idx
        l_off = leader.seg_offset + leader._dist_traveled(t - leader.t_ref)

        f_from, f_to = follower.seg_from, follower.seg_to
        l_from, l_to = leader.seg_from, leader.seg_to

        if f_from == l_from and f_to == l_to:
            # Same segment
            gap_d = l_off - f_off
            if gap_d < 0:
                gap_d = 0
            return gap_d, leader.vel_at(t)

        # Multi-hop: walk follower's path forward to find the leader's segment
        dist = 0.0
        # Distance to end of follower's current segment
        if f_pidx < len(follower._seg_lengths):
            dist += follower._seg_lengths[f_pidx] - f_off
        else:
            seg = follower.current_segment()
            dist += (seg.length if seg else 0) - f_off

        # Walk forward through follower's path
        max_look = min(f_pidx + 80, len(follower.path) - 1)
        for i in range(f_pidx + 1, max_look):
            fn = follower.path[i]
            tn = follower.path[i + 1] if i + 1 < len(follower.path) else None
            if tn is None:
                break
            # Check if leader is on this segment
            if fn == l_from and tn == l_to:
                dist += l_off
                return dist, leader.vel_at(t)
            # Add segment length
            if i < len(follower._seg_lengths):
                dist += follower._seg_lengths[i]
            else:
                seg = self.gmap.segment_between(fn, tn)
                dist += seg.length if seg else 0

            if dist > 200000:  # 200m lookahead limit
                break

        # Leader not found on path — use Euclidean distance as fallback
        fx, fy = follower.x, follower.y
        lx, ly = leader.x, leader.y
        euc = math.hypot(lx - fx, ly - fy)
        return max(euc, 1.0), leader.vel_at(t)

    # ── Core: _plan() ────────────────────────────────────────────────────

    def _plan(self, t: float, v: Vehicle):
        """Single decision function. Compute free_dist, set accel, schedule RESUME."""
        old_key = (v.seg_from, v.seg_to)
        v.set_state(t)
        new_key = (v.seg_from, v.seg_to)

        # Update segment occupancy if changed
        if old_key != new_key:
            if v in self._seg_occupants[old_key]:
                self._seg_occupants[old_key].remove(v)
            if v not in self._seg_occupants[new_key]:
                self._seg_occupants[new_key].append(v)
            # Leaving a ZCU segment → notify followers (they may now pass)
            if old_key[0] is not None:
                old_zone = self.gmap.seg_to_zone.get(old_key)
                if old_zone:
                    # Check if this zone's segment is now empty
                    still_occupied = any(
                        self._seg_occupants.get(sk)
                        for sk in old_zone.all_segs()
                        if sk != new_key
                    )
                    if not still_occupied:
                        self._notify_followers(t)

        # Check if path needs extension
        if v.needs_path_extension():
            last_node = v.path[-1]
            ext = random_safe_path(self.gmap, last_node, length=100)
            v.extend_path(ext)

        # Check if at end of path
        if v.path_idx >= len(v.path) - 1:
            v.vel = 0.0
            v.acc = 0.0
            v.state = STOP
            self._post(t + 1.0, 'RESUME', v)
            return

        seg_speed = v.current_seg_speed()
        target_v = min(v.v_max, seg_speed)
        leader = v.leader

        # ── Free distance: bounded by leader gap and ZCU boundaries ──────
        # Walk path ahead: find leader gap AND ZCU boundaries.
        # X marker stops at the first blocked ZCU.
        # If ZCU is open, X extends through it to the next ZCU.
        zcu_limit = self._free_dist_with_zcu(v, t)

        if leader is None:
            free_dist = zcu_limit
            leader_vel = 0.0
            if free_dist > 100000:
                self._go(t, v, target_v)
                self._post(t + 1.0, 'RESUME', v)
                return
        else:
            gap_d, leader_vel = self.gap(v, t)
            free_from_gap = gap_d - v.h_min
            free_dist = min(free_from_gap, zcu_limit)

            if free_dist > 100000:
                self._go(t, v, target_v)
                self._post(t + 1.0, 'RESUME', v)
                return

        if free_dist <= 0:
            v.acc = 0.0
            v.vel = 0.0
            v.state = STOP
            v.stop_dist = 0.0
            self._post(t + 0.3, 'RESUME', v)
            return

        # ── v_safe ───────────────────────────────────────────────────────
        v_safe = math.sqrt(max(0, 2 * v.d_max * free_dist))
        v_target = min(target_v, v_safe)

        leader_vel_now = leader_vel if leader else 0.0
        if leader_vel_now > 0.1 and leader_vel_now <= v_safe:
            v_target = min(target_v, max(v_target, leader_vel_now))

        # ── Set motion ───────────────────────────────────────────────────
        brake_dist = v.braking_distance()
        v.stop_dist = free_dist

        if free_dist > brake_dist:
            self._go(t, v, v_target)
            future_brake = v_target * v_target / (2 * v.d_max)
            coast_dist = max(0, free_dist - future_brake)
            effective_v = v_target if v_target > 0 else 1
            if v.acc > 0:
                t_accel = max(0, (v_target - v.vel) / v.a_max)
                d_accel = v.vel * t_accel + 0.5 * v.a_max * t_accel ** 2
                t_next = t_accel + max(0, coast_dist - d_accel) / effective_v
            else:
                t_next = coast_dist / effective_v
            self._post(t + max(0.05, t_next), 'RESUME', v)
        else:
            if v.vel > 0.1 and free_dist > 0.1:
                decel = min(v.vel * v.vel / (2 * free_dist), v.d_max)
                v.acc = -decel
                v.state = DECEL
                t_stop = v.vel / decel
                v.stop_dist = free_dist
                self._post(t + t_stop * 0.5, 'RESUME', v)
            else:
                v.acc = 0.0
                v.vel = 0.0
                v.state = STOP
                v.stop_dist = 0.0
                self._post(t + 0.2, 'RESUME', v)

    def _is_zcu_blocked(self, zone: ZCUZone, seg_key: Tuple[str, str],
                        v: Vehicle) -> bool:
        """Is this ZCU segment blocked for vehicle v?"""
        is_curve = zone.is_curve(seg_key)

        # Check curve_group: U-turn first half blocks if second half occupied
        seg_obj = self.gmap.segments.get(seg_key)
        if seg_obj:
            cg = getattr(seg_obj, '_curve_group', None)
            if cg:
                is_first_half = not seg_key[0].startswith('_curve_mid_')
                if is_first_half:
                    for sk in self.gmap.curve_groups.get(cg, []):
                        if sk == seg_key:
                            continue
                        occ = [o for o in self._seg_occupants.get(sk, []) if o is not v]
                        if occ:
                            return True

        # U-turn second half: treat as straight for ZCU purposes
        if seg_key[0].startswith('_curve_mid_'):
            is_curve = False

        for other_key in zone.all_segs():
            if other_key == seg_key:
                continue
            occupants = self._seg_occupants.get(other_key, [])
            occ = [o for o in occupants if o is not v]
            if not occ:
                continue
            if is_curve:
                return True
            elif zone.is_curve(other_key):
                # Check if other is U-turn second half (treat as straight)
                if other_key[0].startswith('_curve_mid_'):
                    continue
                return True
        return False

    def _free_dist_with_zcu(self, v: Vehicle, t: float) -> float:
        """Walk the vehicle's path. X marker stops at ZCU boundaries.

        - Reaches a ZCU → check if blocked or open
        - Blocked → stop here
        - Open → extend through this ZCU, but stop at NEXT ZCU boundary
        Returns distance in mm.
        """
        dist = v.current_seg_length() - v.seg_offset
        pi = v.path_idx
        passed_open_zcu = False  # already passed through one open ZCU

        while dist < 100000 and pi + 1 < len(v.path) - 1:
            fn = v.path[pi + 1]
            tn = v.path[pi + 2] if pi + 2 < len(v.path) else None
            if tn is None:
                break

            seg_key = (fn, tn)
            zone = self.gmap.seg_to_zone.get(seg_key)

            if zone:
                if self._is_zcu_blocked(zone, seg_key, v):
                    # Blocked → X stops at ZCU entry node
                    return dist

                if passed_open_zcu:
                    # Already passed one open ZCU → stop at this ZCU boundary
                    # (X marker covers one ZCU at a time)
                    return dist

                # First open ZCU → pass through, mark that we've passed one
                passed_open_zcu = True

            seg = self.gmap.segments.get(seg_key)
            if seg:
                dist += seg.length
            else:
                break
            pi += 1

        return dist if dist < 100000 else float('inf')

    def _go(self, t: float, v: Vehicle, target_v: float):
        if v.vel < target_v - 1:
            v.acc = v.a_max
            v.state = ACCEL
        elif v.vel > target_v + 1:
            v.acc = -v.d_max
            v.state = DECEL
        else:
            v.vel = target_v
            v.acc = 0.0
            v.state = CRUISE

    # ── Event handlers ────────────────────────────────────────────────────

    def step(self, t_now: float):
        while self.heap and self.heap[0].t <= t_now:
            ev = heapq.heappop(self.heap)
            v = self.vehicles.get(ev.vid)
            if v is None or ev.token != v.token:
                continue
            self.event_count += 1

            if ev.kind == 'START':
                self._plan(ev.t, v)

            elif ev.kind == 'RESUME':
                self._plan(ev.t, v)

            elif ev.kind == 'STOPPED':
                v.set_state(ev.t)
                v.vel = 0.0
                v.acc = 0.0
                v.state = STOP

                # Loading at node?
                if random.random() < self.stop_prob:
                    delay = random.uniform(*self.stop_dur)
                    v.state = LOADING
                    self.stops_executed += 1
                    self._post(ev.t + delay, 'RESUME', v)
                    self._notify_followers(ev.t)
                else:
                    self._post(ev.t + 0.2, 'RESUME', v)

        # Update rendering
        for v in self.vehicles.values():
            v.update_render(t_now)
            g, _ = self.gap(v, t_now)
            v.gap_to_leader = g

    def _notify_followers(self, t: float):
        for v in self.vehicles.values():
            if v.leader is None or v.state == LOADING:
                continue
            if v.state in (STOP, IDLE):
                continue
            self._invalidate(v)
            self._plan(t, v)

    # ── Leader assignment ─────────────────────────────────────────────────

    def assign_leaders(self):
        """For each vehicle, find the closest vehicle ahead on its path
        and assign it as leader."""
        # Build a mapping: (seg_from, seg_to) -> list of (offset, vehicle)
        seg_vehs: Dict[Tuple[str, str], List[Tuple[float, Vehicle]]] = collections.defaultdict(list)
        for v in self.vehicles.values():
            key = (v.seg_from, v.seg_to)
            seg_vehs[key].append((v.seg_offset, v))

        for v in self.vehicles.values():
            best_leader = None
            best_dist = float('inf')

            # Check current segment: any vehicle ahead of us?
            key = (v.seg_from, v.seg_to)
            for off, other in seg_vehs[key]:
                if other.id == v.id:
                    continue
                if off > v.seg_offset:
                    d = off - v.seg_offset
                    if d < best_dist:
                        best_dist = d
                        best_leader = other

            if best_leader is None:
                # Look ahead through path
                dist_accum = v.current_seg_length() - v.seg_offset
                for i in range(v.path_idx + 1, min(v.path_idx + 30, len(v.path) - 1)):
                    fn = v.path[i]
                    tn = v.path[i + 1] if i + 1 < len(v.path) else None
                    if tn is None:
                        break
                    fwd_key = (fn, tn)
                    if fwd_key in seg_vehs:
                        # Find vehicle closest to start of this segment
                        candidates = seg_vehs[fwd_key]
                        if candidates:
                            candidates_sorted = sorted(candidates, key=lambda x: x[0])
                            off, other = candidates_sorted[0]
                            if other.id != v.id:
                                d = dist_accum + off
                                if d < best_dist:
                                    best_dist = d
                                    best_leader = other
                                break  # found leader
                    seg = self.gmap.segment_between(fn, tn)
                    dist_accum += seg.length if seg else 0
                    if dist_accum > 100000:
                        break

            v.leader = best_leader

    def start_all(self):
        """Initialize all vehicles and assign leaders."""
        self.assign_leaders()
        for v in self.vehicles.values():
            self._post(0.0, 'START', v)

    def reassign_leaders_periodic(self, t: float):
        """Periodically re-assign leaders (call every few sim seconds)."""
        self.assign_leaders()
        # Notify all moving vehicles to re-plan with new leaders
        for v in self.vehicles.values():
            if v.state in (ACCEL, CRUISE, DECEL):
                self._invalidate(v)
                self._plan(t, v)
