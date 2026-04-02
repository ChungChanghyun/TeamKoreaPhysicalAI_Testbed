"""
graph_des_v6.py — Pure DES engine for OHT networks.

Architecture:
- Pure DES: events are self-sustaining, committed, and NEVER cancelled.
- Two types of follower relationships:
  1. Path follower: same-route leader's committed events guarantee safe distance.
  2. ZCU follower:  ZCU lock holder's SEG_END at exit node releases the lock.
- ZCU lock system:
  - Merge:  boundary nodes = predecessor nodes (A, B). Exit = merge node SEG_END.
  - Diverge: boundary node = diverge node. Exit = successor node SEG_END.
  - Merge+Diverge: both zones handled independently with unique lock IDs.
- No event invalidation. No _notify_followers. All events are committed trajectories.
- Rendering completely separated via query_positions(t).
"""

from __future__ import annotations
import math, heapq, collections
from typing import Dict, List, Optional, Tuple, Set

from graph_des_v5 import (
    ZCUZone, MapNode, MapSegment, GraphMap,
    random_safe_path, _interp_path,
)

# ── Vehicle states ────────────────────────────────────────────────────────────

IDLE    = 'IDLE'
ACCEL   = 'ACCEL'
CRUISE  = 'CRUISE'
DECEL   = 'DECEL'
STOP    = 'STOP'
LOADING = 'LOADING'

# ── Event types ───────────────────────────────────────────────────────────────

EV_START      = 'START'
EV_REPLAN     = 'REPLAN'       # re-evaluate at plan boundary (leader/dest)
EV_SEG_END    = 'SEG_END'      # reached end of current segment
EV_PHASE_DONE = 'PHASE_DONE'   # reached target speed (accel complete)
EV_STOPPED    = 'STOPPED'      # decelerated to v=0
EV_ZCU_GRANT  = 'ZCU_GRANT'   # ZCU lock released, waiting vehicle may proceed
EV_BOUNDARY   = 'BOUNDARY'    # reached braking point before plan boundary


# ── Kinematics helper ────────────────────────────────────────────────────────

def _time_to_boundary_during_accel(v0: float, a_acc: float, d_max: float,
                                   dist: float, v_max: float) -> float:
    """Time at which braking must begin to stop at `dist`, while accelerating.

    Vehicle is accelerating at a_acc from v0. We need to find the time t₁
    at which to switch to deceleration (-d_max) so that the vehicle stops
    exactly at distance `dist`.

    Triangular profile: accel(a_acc) for t₁, then decel(-d_max) to v=0.
      v_peak = v0 + a_acc * t₁  (capped at v_max)
      accel_dist = v0*t₁ + 0.5*a_acc*t₁²
      brake_dist = v_peak² / (2*d_max)
      accel_dist + brake_dist = dist

    Substituting v_peak = v0 + a_acc*t₁:
      v0*t₁ + 0.5*a_acc*t₁² + (v0 + a_acc*t₁)² / (2*d_max) = dist

    This is a quadratic in t₁:
      (0.5*a_acc + a_acc²/(2*d_max)) * t₁²
      + (v0 + a_acc*v0/d_max) * t₁
      + (v0²/(2*d_max) - dist) = 0

    Returns time t₁ (from now) or inf if impossible.
    """
    if dist <= 0:
        return 0.0
    # Check if already need to brake (current brake_dist >= dist)
    if v0 * v0 / (2 * d_max) >= dist:
        return 0.0

    # Quadratic coefficients: A*t² + B*t + C = 0
    A = 0.5 * a_acc + a_acc * a_acc / (2 * d_max)
    B = v0 + a_acc * v0 / d_max
    C = v0 * v0 / (2 * d_max) - dist

    disc = B * B - 4 * A * C
    if disc < 0:
        return float('inf')

    t1 = (-B + math.sqrt(disc)) / (2 * A)
    if t1 < 0:
        return 0.0

    # Check v_max cap: if v_peak would exceed v_max, use trapezoidal profile
    v_peak = v0 + a_acc * t1
    if v_peak > v_max:
        # Time to reach v_max
        t_accel = (v_max - v0) / a_acc
        d_accel = v0 * t_accel + 0.5 * a_acc * t_accel ** 2
        d_brake = v_max * v_max / (2 * d_max)
        d_cruise_needed = dist - d_accel - d_brake
        if d_cruise_needed < 0:
            # Can't reach v_max and still stop — original t1 was correct
            # but capped scenario means we brake earlier
            return t1
        # Cruise phase at v_max, then brake
        t_cruise = d_cruise_needed / v_max
        return t_accel + t_cruise

    return t1


def _time_to_travel(v0: float, acc: float, dist: float, v_max: float) -> float:
    if dist <= 0:
        return 0.0
    if acc > 0:
        t_cap = (v_max - v0) / acc if v0 < v_max else 0.0
        d_cap = v0 * t_cap + 0.5 * acc * t_cap ** 2
        if dist <= d_cap:
            disc = v0 * v0 + 2 * acc * dist
            return (-v0 + math.sqrt(max(0, disc))) / acc if disc >= 0 else float('inf')
        else:
            return t_cap + (dist - d_cap) / v_max
    elif acc < 0:
        d_stop = v0 * v0 / (2 * abs(acc))
        if dist > d_stop:
            return float('inf')
        disc = v0 * v0 + 2 * acc * dist
        if disc < 0:
            return float('inf')
        return (-v0 + math.sqrt(disc)) / acc
    else:
        return dist / v0 if v0 > 0 else float('inf')


# ── Vehicle ───────────────────────────────────────────────────────────────────

class Vehicle:
    def __init__(self, vid: int, gmap: GraphMap, path: List[str],
                 color=(200, 200, 200)):
        self.id = vid
        self.gmap = gmap
        self.color = color

        self.path: List[str] = path
        self.path_idx: int = 0
        self.seg_offset: float = 0.0

        self.vel: float = 0.0
        self.acc: float = 0.0
        self.t_ref: float = 0.0

        self.v_max: float = 3600.0
        self.a_max: float = 500.0
        self.d_max: float = 500.0
        self.length: float = 750.0
        self.h_min: float = self.length + 400

        self.state: str = IDLE
        self.next_event_t: float = 0.0

        # Path leader (same-route vehicle ahead)
        self.leader: Optional[Vehicle] = None

        # X marker
        self.stop_dist: Optional[float] = None
        self.x_marker_pidx: int = 0
        self.x_marker_offset: float = 0.0
        self.x_marker_node: Optional[str] = None

        # Destination
        self.dest_node: Optional[str] = None
        self.dest_reached: bool = False

        # ZCU state
        self.waiting_at_zcu: Optional[str] = None   # zone lock_id if waiting
        self.passed_zcu: Set[str] = set()            # boundary nodes granted

        # Render cache
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self.gap_to_leader: float = float('inf')

        self._seg_lengths: List[float] = []
        self._seg_speeds: List[float] = []
        self._rebuild_seg_cache()

    def _rebuild_seg_cache(self):
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
        return self.path[self.path_idx] if self.path_idx < len(self.path) else self.path[-1]

    @property
    def seg_to(self) -> str:
        return self.path[self.path_idx + 1] if self.path_idx + 1 < len(self.path) else self.path[-1]

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

    def dist_to_seg_end(self) -> float:
        return max(0.0, self.current_seg_length() - self.seg_offset)

    def _dist_traveled(self, dt: float) -> float:
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
        dt = max(0, t - self.t_ref)
        if self.acc > 0:
            return min(self.v_max, self.vel + self.acc * dt)
        elif self.acc < 0:
            return max(0.0, self.vel + self.acc * dt)
        return self.vel

    def braking_distance(self, from_vel: float = -1) -> float:
        v = from_vel if from_vel >= 0 else self.vel
        return v * v / (2 * self.d_max) if v > 0 else 0.0

    def advance_position(self, t: float) -> List[str]:
        """Advance to time t. Returns list of nodes crossed (arrived at)."""
        dist = self._dist_traveled(t - self.t_ref)
        self.vel = self.vel_at(t)
        self.t_ref = t
        self.seg_offset += dist
        crossed = []
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
                crossed.append(self.seg_from)
            else:
                break
        return crossed

    def set_state(self, t: float) -> List[str]:
        crossed = self.advance_position(t)
        self.acc = 0.0
        return crossed

    def update_render(self, t: float):
        dist = self._dist_traveled(t - self.t_ref)
        offset = self.seg_offset + dist
        pidx = self.path_idx
        while pidx < len(self.path) - 1:
            seg_len = self._seg_lengths[pidx] if pidx < len(self._seg_lengths) else 0.0
            if seg_len <= 0:
                pidx += 1; offset = 0.0; continue
            if offset >= seg_len - 0.01 and pidx < len(self.path) - 2:
                offset -= seg_len; pidx += 1
                if offset < 0: offset = 0.0
            else:
                break
        if pidx < len(self.path) - 1:
            seg = self.gmap.segment_between(self.path[pidx], self.path[pidx + 1])
            if seg and seg.path_points:
                self.x, self.y, self.theta = _interp_path(seg.path_points, max(0, offset))
                return
        nid = self.path[min(pidx, len(self.path) - 1)]
        node = self.gmap.nodes.get(nid)
        if node:
            self.x, self.y = node.x, node.y

    def needs_path_extension(self) -> bool:
        return self.path_idx >= len(self.path) - 10

    def extend_path(self, new_nodes: List[str]):
        if self.path_idx > 5:
            trim = self.path_idx - 2
            self.path = self.path[trim:]
            self.path_idx -= trim
            self._seg_lengths = self._seg_lengths[trim:]
            self._seg_speeds = self._seg_speeds[trim:]
            self.x_marker_pidx = max(-1, self.x_marker_pidx - trim)
        if new_nodes and new_nodes[0] == self.path[-1]:
            new_nodes = new_nodes[1:]
        self.path.extend(new_nodes)
        old_len = len(self._seg_lengths)
        for i in range(old_len, len(self.path) - 1):
            seg = self.gmap.segment_between(self.path[i], self.path[i + 1])
            if seg:
                self._seg_lengths.append(seg.length)
                self._seg_speeds.append(seg.max_speed)
            else:
                self._seg_lengths.append(0.0)
                self._seg_speeds.append(self.v_max)


# ── Event ─────────────────────────────────────────────────────────────────────

class Event:
    __slots__ = ('t', 'kind', 'vid')

    def __init__(self, t: float, kind: str, vid: int):
        self.t = t
        self.kind = kind
        self.vid = vid

    def __lt__(self, other):
        return self.t < other.t


# ── Pure DES Engine ───────────────────────────────────────────────────────────

class GraphDESv6:
    """Pure DES engine — all events are committed, never cancelled."""

    def __init__(self, gmap: GraphMap):
        self.gmap = gmap
        self.vehicles: Dict[int, Vehicle] = {}
        self.heap: List[Event] = []
        self.sim_time: float = 0.0
        self.event_count: int = 0

        # Segment occupancy (for gap computation & rendering)
        self._seg_occupants: Dict[Tuple[str, str], List[Vehicle]] = \
            collections.defaultdict(list)

        # ── ZCU lock system ──────────────────────────────────────────────
        # lock_id = (zone.node_id, zone.kind) to handle merge+diverge at same node
        # boundary_node → list of (zone, lock_id)  (a node can be boundary for multiple zones)
        self._boundary_to_zones: Dict[str, List[Tuple[ZCUZone, str]]] = \
            collections.defaultdict(list)
        self._zone_lock: Dict[str, Optional[Vehicle]] = {}       # lock_id → holder
        self._zone_waiters: Dict[str, List[Vehicle]] = \
            collections.defaultdict(list)                         # lock_id → waiters
        self._boundary_nodes: Set[str] = set()
        # exit_node → list of (zone, lock_id)  (to release lock on SEG_END)
        self._exit_to_zones: Dict[str, List[Tuple[ZCUZone, str]]] = \
            collections.defaultdict(list)

        self._build_zcu_locks()

    def _build_zcu_locks(self):
        for zone in self.gmap.zcu_zones:
            lock_id = f"{zone.node_id}_{zone.kind}"
            self._zone_lock[lock_id] = None

            if zone.kind == 'merge':
                # Boundary = predecessor nodes (entry points before merge)
                for seg_key in zone.all_segs():
                    pred_node = seg_key[0]
                    self._boundary_to_zones[pred_node].append((zone, lock_id))
                    self._boundary_nodes.add(pred_node)
                # Exit = merge node itself
                self._exit_to_zones[zone.node_id].append((zone, lock_id))

            elif zone.kind == 'diverge':
                # Boundary = diverge node itself
                self._boundary_to_zones[zone.node_id].append((zone, lock_id))
                self._boundary_nodes.add(zone.node_id)
                # Exit = any successor node
                for seg_key in zone.all_segs():
                    succ_node = seg_key[1]
                    self._exit_to_zones[succ_node].append((zone, lock_id))

        # For merge locks held via a boundary node where the OHT goes
        # a different direction: the OHT never reaches the merge node,
        # so we also register ALL successors of each boundary node as
        # exit points for ALL zones at that boundary.
        # This way, arriving at ANY next node after the boundary releases
        # all locks acquired at that boundary.
        for bnd_node, zone_list in self._boundary_to_zones.items():
            successors = self.gmap.adj.get(bnd_node, [])
            for succ in successors:
                for zone, lock_id in zone_list:
                    # Avoid duplicates
                    existing = self._exit_to_zones.get(succ, [])
                    if (zone, lock_id) not in existing:
                        self._exit_to_zones[succ].append((zone, lock_id))

        print(f"ZCU locks: {len(self._zone_lock)} zones, "
              f"{len(self._boundary_nodes)} boundary nodes, "
              f"{len(self._exit_to_zones)} exit nodes")

    # ── Vehicle management ────────────────────────────────────────────────

    def add_vehicle(self, v: Vehicle):
        self.vehicles[v.id] = v
        self._seg_occupants[(v.seg_from, v.seg_to)].append(v)

    def _post(self, t: float, kind: str, v: Vehicle):
        v.next_event_t = t
        heapq.heappush(self.heap, Event(t, kind, v.id))

    # ── Public API ────────────────────────────────────────────────────────

    def run_until(self, t_end: float):
        while self.heap and self.heap[0].t <= t_end:
            ev = heapq.heappop(self.heap)
            v = self.vehicles.get(ev.vid)
            if v is None:
                continue
            self.sim_time = ev.t
            self.event_count += 1
            self._dispatch(ev, v)

    def query_positions(self, t: float):
        for v in self.vehicles.values():
            v.update_render(t)
            if v.leader is not None:
                g, _ = self.gap(v, t)
                v.gap_to_leader = g
            else:
                v.gap_to_leader = float('inf')

    def step(self, t_now: float):
        self.run_until(t_now)
        self.query_positions(t_now)

    def start_all(self):
        self.assign_leaders()
        for v in self.vehicles.values():
            self._post(0.0, EV_START, v)

    # ── Event dispatch ────────────────────────────────────────────────────

    def _dispatch(self, ev: Event, v: Vehicle):
        handler = {
            EV_START:      self._replan,
            EV_REPLAN:     self._replan,
            EV_SEG_END:    self._on_seg_end,
            EV_PHASE_DONE: self._on_phase_done,
            EV_STOPPED:    self._on_stopped,
            EV_ZCU_GRANT:  self._on_zcu_grant,
            EV_BOUNDARY:   self._on_boundary,
        }.get(ev.kind)
        if handler:
            handler(ev.t, v)

    # ── Crossed-node processing ─────────────────────────────────────────

    def _process_crossed_nodes(self, t: float, v: Vehicle, crossed: List[str]):
        """Handle ZCU exit releases and passed_zcu cleanup for all crossed nodes."""
        for node in crossed:
            # Clear passed_zcu
            v.passed_zcu.discard(node)
            # ZCU exit release
            for zone, lock_id in self._exit_to_zones.get(node, []):
                if self._zone_lock.get(lock_id) is v:
                    self._zone_release(t, lock_id)

    # ── Event handlers ────────────────────────────────────────────────────

    def _on_seg_end(self, t: float, v: Vehicle):
        old_key = (v.seg_from, v.seg_to)
        crossed = v.advance_position(t)
        new_key = (v.seg_from, v.seg_to)
        self._update_occupancy(v, old_key, new_key, t)
        self._process_crossed_nodes(t, v, crossed)

        if v.needs_path_extension():
            ext = random_safe_path(self.gmap, v.path[-1], length=100)
            v.extend_path(ext)

        self._replan(t, v)

    def _on_phase_done(self, t: float, v: Vehicle):
        crossed = v.advance_position(t)
        v.acc = 0.0
        v.state = CRUISE
        self._process_crossed_nodes(t, v, crossed)
        self._replan(t, v)

    def _on_stopped(self, t: float, v: Vehicle):
        crossed = v.set_state(t)
        v.vel = 0.0
        v.acc = 0.0
        v.state = STOP
        self._process_crossed_nodes(t, v, crossed)

        # Check if stopped at a ZCU boundary
        bnd_node = v.x_marker_node
        zones = self._relevant_zones(v, bnd_node) if bnd_node else []
        if zones:
            # Try to acquire all needed locks at this boundary
            all_granted = True
            for zone, lock_id in zones:
                if not self._zone_request(v, lock_id):
                    all_granted = False
                    self._zone_wait(v, lock_id)
                    break
            if all_granted:
                v.passed_zcu.add(bnd_node)
                self._post(t + 0.01, EV_REPLAN, v)
        else:
            # Path leader or dest stop → replan at own schedule
            self._post(t + 0.2, EV_REPLAN, v)

    def _on_zcu_grant(self, t: float, v: Vehicle):
        v.waiting_at_zcu = None
        self._replan(t, v)

    def _relevant_zones(self, v: Vehicle, bnd_node: str) -> List[Tuple[ZCUZone, str]]:
        """Determine which zones at a boundary node this OHT must lock.

        - Diverge: always lock (physical node is shared, direction-independent)
        - Merge: only lock if OHT's path goes bnd_node → merge_node
                 (no need to lock a merge the OHT isn't heading toward)
        """
        result = []
        for zone, lock_id in self._boundary_to_zones.get(bnd_node, []):
            if zone.kind == 'diverge':
                result.append((zone, lock_id))
            elif zone.kind == 'merge':
                for i in range(v.path_idx, min(v.path_idx + 40, len(v.path) - 1)):
                    if v.path[i] == bnd_node and v.path[i + 1] == zone.node_id:
                        result.append((zone, lock_id))
                        break
        return result

    def _on_boundary(self, t: float, v: Vehicle):
        bnd_node = v.x_marker_node
        zones = self._relevant_zones(v, bnd_node) if bnd_node else []

        if zones:
            # Try to acquire all relevant locks at this boundary
            all_granted = True
            denied_lock_id = None
            for zone, lock_id in zones:
                if not self._zone_request(v, lock_id):
                    all_granted = False
                    denied_lock_id = lock_id
                    break
            if all_granted:
                # Lock granted → continue current motion, extend plan
                # Do NOT call set_state() — preserve current vel/acc
                v.passed_zcu.add(bnd_node)
                self._replan(t, v, skip_set_state=True)
                return

            # Lock denied → must set_state to prepare for braking
            old_key = (v.seg_from, v.seg_to)
            crossed = v.set_state(t)
            new_key = (v.seg_from, v.seg_to)
            if old_key != new_key:
                self._update_occupancy(v, old_key, new_key, t)
            self._process_crossed_nodes(t, v, crossed)

            bnd_dist, _, _ = self._find_first_boundary(v)
            if bnd_dist == float('inf'):
                bnd_dist = 0.0
            if v.vel > 0.1 and bnd_dist > 0.1:
                decel = min(v.vel * v.vel / (2 * bnd_dist), v.d_max)
                v.acc = -decel
                v.state = DECEL
                self._pin_marker_at_dist(v, bnd_dist)
                self._post(t + v.vel / decel, EV_STOPPED, v)
            else:
                v.vel = 0.0
                v.acc = 0.0
                v.state = STOP
                self._pin_marker_at_dist(v, 0)
                self._zone_wait(v, denied_lock_id)
            return

        # Non-ZCU boundary or no relevant zones → replan normally
        self._replan(t, v)

    # ── Core: _replan() ───────────────────────────────────────────────────

    def _replan(self, t: float, v: Vehicle, skip_set_state: bool = False):
        old_key = (v.seg_from, v.seg_to)
        if skip_set_state:
            crossed = v.advance_position(t)
        else:
            crossed = v.set_state(t)
        new_key = (v.seg_from, v.seg_to)
        if old_key != new_key:
            self._update_occupancy(v, old_key, new_key, t)
        self._process_crossed_nodes(t, v, crossed)

        if v.needs_path_extension():
            ext = random_safe_path(self.gmap, v.path[-1], length=100)
            v.extend_path(ext)

        if v.path_idx >= len(v.path) - 1:
            v.vel = 0.0; v.acc = 0.0; v.state = STOP
            return

        # Destination check
        if v.dest_node and not v.dest_reached:
            if v.seg_from == v.dest_node or \
               (v.path_idx > 0 and v.path[v.path_idx] == v.dest_node):
                v.dest_reached = True
        if v.dest_node and v.dest_reached:
            v.vel = 0.0; v.acc = 0.0; v.state = STOP
            return

        # ── Constraints ──────────────────────────────────────────────────

        seg_speed = v.current_seg_speed()
        target_v = min(v.v_max, seg_speed)
        lookahead_v, dist_to_slow = self._lookahead_speed(v)
        target_v = min(target_v, lookahead_v)

        # ZCU boundary
        bnd_dist, bnd_pi, bnd_node = self._find_first_boundary(v)

        # Path leader — refresh at each replan
        self._update_leader(v)
        leader = v.leader
        leader_free = float('inf')
        if leader is not None:
            gap_d, _ = self.gap(v, t)
            leader_extra = self._leader_extra_dist(leader, t)
            leader_free = min(gap_d + leader_extra - v.h_min,
                              gap_d - v.h_min)

        # Dest
        dest_dist = self._dist_to_dest(v)

        # Plan boundary
        plan_boundary = min(bnd_dist, leader_free, dest_dist)

        # Pin marker
        if plan_boundary < 100000:
            if bnd_dist <= leader_free and bnd_dist <= dest_dist and bnd_node:
                self._pin_marker(v, bnd_pi, bnd_node)
            else:
                self._pin_marker_at_dist(v, plan_boundary)
            v.stop_dist = plan_boundary
        else:
            v.stop_dist = None
            self._pin_marker_at_dist(v, v.dist_to_seg_end() + 5000)

        # ── Scheduling ───────────────────────────────────────────────────

        if plan_boundary <= 0:
            v.vel = 0.0; v.acc = 0.0; v.state = STOP; v.stop_dist = 0.0
            self._pin_marker_at_dist(v, 0)
            # Determine what caused the block
            zcu_is_constraint = (bnd_node and bnd_dist < 100000 and
                                 bnd_dist <= leader_free and bnd_dist <= dest_dist)
            if zcu_is_constraint:
                zones = self._relevant_zones(v, bnd_node)
                if zones:
                    granted = True
                    for zone, lock_id in zones:
                        if not self._zone_request(v, lock_id):
                            granted = False
                            self._zone_wait(v, lock_id)
                            break
                    if granted:
                        v.passed_zcu.add(bnd_node)
                        self._post(t + 0.01, EV_REPLAN, v)
                    # else: waiting for ZCU_GRANT (no event needed)
                    return
            # Blocked by leader or dest → periodic retry
            self._post(t + 0.5, EV_REPLAN, v)
            return

        if plan_boundary > 100000 and dist_to_slow > 100000:
            self._go(t, v, target_v)
            self._schedule_next_event(t, v, target_v, plan_boundary, dist_to_slow)
            return

        # Constrained by plan_boundary and/or slow segments
        brake_dist = v.braking_distance()

        # If within braking distance of the plan boundary, commit to stopping
        if plan_boundary < 100000 and plan_boundary <= brake_dist + 1.0:
            if v.vel > 0.1 and plan_boundary > 0.1:
                decel = min(v.vel * v.vel / (2 * plan_boundary), v.d_max)
                v.acc = -decel
                v.state = DECEL
                t_stop = v.vel / decel
                t_seg = _time_to_travel(v.vel, v.acc, v.dist_to_seg_end(), v.v_max)
                if t_seg < t_stop and t_seg < float('inf'):
                    self._post(t + t_seg, EV_SEG_END, v)
                else:
                    self._post(t + t_stop, EV_STOPPED, v)
            else:
                v.vel = 0.0; v.acc = 0.0; v.state = STOP; v.stop_dist = 0.0
                self._pin_marker_at_dist(v, 0)
                zcu_is_constraint = (bnd_node and bnd_dist < 100000 and
                                     bnd_dist <= leader_free and bnd_dist <= dest_dist)
                if zcu_is_constraint:
                    zones = self._relevant_zones(v, bnd_node)
                    if zones:
                        granted = True
                        for zone, lock_id in zones:
                            if not self._zone_request(v, lock_id):
                                granted = False
                                self._zone_wait(v, lock_id)
                                break
                        if granted:
                            v.passed_zcu.add(bnd_node)
                            self._post(t + 0.01, EV_REPLAN, v)
                        return
                # Blocked by leader or dest → periodic retry
                self._post(t + 0.5, EV_REPLAN, v)
        else:
            # Not yet within braking distance — go toward target_v and let
            # the BOUNDARY event fire at the right braking point.
            # Do NOT use v_safe as an intermediate target — it causes
            # oscillating decel/cruise cycles.
            self._go(t, v, target_v)
            self._schedule_next_event(t, v, target_v, plan_boundary,
                                      dist_to_slow)

    # ── Event scheduling ──────────────────────────────────────────────────

    def _schedule_next_event(self, t: float, v: Vehicle, target_v: float,
                             plan_boundary: float, dist_to_slow: float):
        best_t = float('inf')
        best_kind = EV_REPLAN

        accel_phase = v.acc > 0 and v.vel < target_v - 0.1
        decel_phase = v.acc < 0
        cruise_phase = not accel_phase and not decel_phase and v.vel > 0.1

        # 1. Segment end — always check
        t_seg = _time_to_travel(v.vel, v.acc, v.dist_to_seg_end(), v.v_max)
        if t + t_seg < best_t:
            best_t = t + t_seg
            best_kind = EV_SEG_END

        # 2. Phase done (accel → cruise transition)
        if accel_phase:
            t_phase = (target_v - v.vel) / v.acc
            if t + t_phase < best_t:
                best_t = t + t_phase
                best_kind = EV_PHASE_DONE

        # 3. Decel done (decel → cruise transition at lower target speed)
        if decel_phase and target_v > 0.5:
            t_decel = (v.vel - target_v) / abs(v.acc)
            if t_decel > 0.001 and t + t_decel < best_t:
                best_t = t + t_decel
                best_kind = EV_PHASE_DONE

        # 4. Plan boundary approach
        if plan_boundary < 100000:
            if accel_phase:
                # Triangular/trapezoidal profile: exact time to start braking
                t_boundary = _time_to_boundary_during_accel(
                    v.vel, v.acc, v.d_max, plan_boundary, v.v_max)
            elif cruise_phase:
                # Cruise: linear approach to braking point
                brake_d = v.braking_distance()
                approach_dist = max(0, plan_boundary - brake_d)
                t_boundary = approach_dist / v.vel if approach_dist > 0 else 0.0
            elif decel_phase:
                # Already decelerating — don't schedule BOUNDARY, the decel
                # will either stop (STOPPED) or reach target (PHASE_DONE)
                t_boundary = float('inf')
            else:
                t_boundary = 0.0

            if t_boundary < float('inf'):
                if t_boundary < 0.001:
                    t_boundary = 0.01
                if t + t_boundary < best_t:
                    best_t = t + t_boundary
                    best_kind = EV_BOUNDARY

        # 5. Slow segment approach (curve braking) — cruise only
        #    Only schedule a REPLAN if there's meaningful approach distance.
        #    If already within brake+margin of the slow segment, SEG_END
        #    or BOUNDARY events will handle the transition.
        if cruise_phase and dist_to_slow < 100000:
            brake_d = v.braking_distance()
            approach_dist = dist_to_slow - brake_d - 500
            if approach_dist > 100:  # meaningful distance to cover
                t_slow = approach_dist / v.vel
                if t + t_slow < best_t:
                    best_t = t + t_slow
                    best_kind = EV_REPLAN

        if best_t <= t + 0.005:
            best_t = t + 0.01
            # Preserve BOUNDARY event kind — clamping to 0.01 is fine
            # but turning it into REPLAN causes infinite 0.01s loops
            if best_kind not in (EV_BOUNDARY, EV_SEG_END, EV_PHASE_DONE):
                best_kind = EV_REPLAN
        if best_t == float('inf'):
            best_t = t + 2.0
            best_kind = EV_REPLAN

        self._post(best_t, best_kind, v)

    # ── Leader ────────────────────────────────────────────────────────────

    def _update_leader(self, v: Vehicle):
        """Find the nearest vehicle ahead on v's path.

        Builds a set of (seg_from, seg_to) along v's forward path,
        then checks all vehicles to find any on those segments.
        Uses event-time positions (seg_from, seg_to, seg_offset).
        """
        # Build forward segment set with accumulated distance
        fwd_segs: Dict[Tuple[str, str], float] = {}  # seg_key → dist to seg start
        dist_accum = 0.0

        # Current segment: distance offset = negative (we're partway through)
        key = (v.seg_from, v.seg_to)
        fwd_segs[key] = -v.seg_offset

        dist_accum = v.current_seg_length() - v.seg_offset
        for i in range(v.path_idx + 1,
                       min(v.path_idx + 30, len(v.path) - 1)):
            fn = v.path[i]
            tn = v.path[i + 1] if i + 1 < len(v.path) else None
            if tn is None:
                break
            fwd_segs[(fn, tn)] = dist_accum
            seg = self.gmap.segment_between(fn, tn)
            dist_accum += seg.length if seg else 0
            if dist_accum > 15000:
                break

        # Search all vehicles
        best_leader = None
        best_dist = float('inf')
        for other in self.vehicles.values():
            if other.id == v.id:
                continue
            other_key = (other.seg_from, other.seg_to)
            if other_key in fwd_segs:
                d = fwd_segs[other_key] + other.seg_offset
                if other_key == key:
                    # Same segment: only if ahead
                    if other.seg_offset <= v.seg_offset:
                        continue
                if 0 < d < best_dist:
                    best_dist = d
                    best_leader = other

        v.leader = best_leader

    def assign_leaders(self):
        """Initial leader assignment for all vehicles at startup."""
        seg_vehs: Dict[Tuple[str, str], List[Tuple[float, Vehicle]]] = \
            collections.defaultdict(list)
        for v in self.vehicles.values():
            seg_vehs[(v.seg_from, v.seg_to)].append((v.seg_offset, v))

        for v in self.vehicles.values():
            best_leader = None
            best_dist = float('inf')

            key = (v.seg_from, v.seg_to)
            for off, other in seg_vehs[key]:
                if other.id != v.id and off > v.seg_offset:
                    d = off - v.seg_offset
                    if d < best_dist:
                        best_dist = d
                        best_leader = other

            if best_leader is None:
                dist_accum = v.current_seg_length() - v.seg_offset
                for i in range(v.path_idx + 1,
                               min(v.path_idx + 30, len(v.path) - 1)):
                    fn = v.path[i]
                    tn = v.path[i + 1] if i + 1 < len(v.path) else None
                    if tn is None:
                        break
                    fwd_key = (fn, tn)
                    if fwd_key in seg_vehs and seg_vehs[fwd_key]:
                        cands = sorted(seg_vehs[fwd_key], key=lambda x: x[0])
                        off, other = cands[0]
                        if other.id != v.id:
                            d = dist_accum + off
                            if d < best_dist:
                                best_dist = d
                                best_leader = other
                            break
                    seg = self.gmap.segment_between(fn, tn)
                    dist_accum += seg.length if seg else 0
                    if dist_accum > 100000:
                        break

            v.leader = best_leader

    def gap(self, follower: Vehicle, t: float) -> Tuple[float, float]:
        leader = follower.leader
        if leader is None:
            return float('inf'), 0.0

        f_off = follower.seg_offset + follower._dist_traveled(t - follower.t_ref)
        l_off = leader.seg_offset + leader._dist_traveled(t - leader.t_ref)

        if follower.seg_from == leader.seg_from and \
           follower.seg_to == leader.seg_to:
            return max(0, l_off - f_off), leader.vel_at(t)

        dist = 0.0
        if follower.path_idx < len(follower._seg_lengths):
            dist += follower._seg_lengths[follower.path_idx] - f_off
        else:
            seg = follower.current_segment()
            dist += (seg.length if seg else 0) - f_off

        for i in range(follower.path_idx + 1,
                       min(follower.path_idx + 80, len(follower.path) - 1)):
            fn = follower.path[i]
            tn = follower.path[i + 1] if i + 1 < len(follower.path) else None
            if tn is None:
                break
            if fn == leader.seg_from and tn == leader.seg_to:
                return dist + l_off, leader.vel_at(t)
            if i < len(follower._seg_lengths):
                dist += follower._seg_lengths[i]
            else:
                seg = self.gmap.segment_between(fn, tn)
                dist += seg.length if seg else 0
            if dist > 200000:
                break

        euc = math.hypot(leader.x - follower.x, leader.y - follower.y)
        return max(euc, 1.0), leader.vel_at(t)

    def _leader_extra_dist(self, leader: Vehicle, t: float) -> float:
        """Committed distance: leader WILL travel at least this far.
        Based on leader's committed event (next_event_t), which is never cancelled."""
        t_event = leader.next_event_t
        if t_event <= t:
            return leader.braking_distance(leader.vel_at(t))
        dist_now = leader._dist_traveled(t - leader.t_ref)
        dist_event = leader._dist_traveled(t_event - leader.t_ref)
        confirmed_travel = dist_event - dist_now
        confirmed_vel = leader.vel_at(t_event)
        return confirmed_travel + leader.braking_distance(confirmed_vel)

    # ── ZCU lock ──────────────────────────────────────────────────────────

    def _zone_request(self, v: Vehicle, lock_id: str) -> bool:
        holder = self._zone_lock.get(lock_id)
        if holder is None or holder is v:
            self._zone_lock[lock_id] = v
            return True
        return False

    def _zone_release(self, t: float, lock_id: str):
        self._zone_lock[lock_id] = None
        waiters = self._zone_waiters.get(lock_id, [])
        if waiters:
            next_v = waiters.pop(0)
            next_v.waiting_at_zcu = None
            self._post(t + 0.01, EV_ZCU_GRANT, next_v)

    def _zone_wait(self, v: Vehicle, lock_id: str):
        v.waiting_at_zcu = lock_id
        if v not in self._zone_waiters[lock_id]:
            self._zone_waiters[lock_id].append(v)

    # ── Segment occupancy ─────────────────────────────────────────────────

    def _update_occupancy(self, v: Vehicle, old_key: Tuple[str, str],
                          new_key: Tuple[str, str], t: float):
        if old_key == new_key:
            return
        if v in self._seg_occupants[old_key]:
            self._seg_occupants[old_key].remove(v)
        if v not in self._seg_occupants[new_key]:
            self._seg_occupants[new_key].append(v)

    # ── Boundary / ZCU helpers ────────────────────────────────────────────

    def _find_first_boundary(self, v: Vehicle) -> Tuple[float, int, Optional[str]]:
        dist = v.current_seg_length() - v.seg_offset
        pi = v.path_idx
        while dist < 100000 and pi + 1 < len(v.path) - 1:
            next_node = v.path[pi + 1]
            if next_node in self._boundary_nodes and \
               next_node not in v.passed_zcu:
                return dist, pi, next_node
            tn = v.path[pi + 2] if pi + 2 < len(v.path) else None
            if tn is None:
                break
            seg = self.gmap.segments.get((next_node, tn))
            if seg:
                dist += seg.length
            else:
                break
            pi += 1
        return float('inf'), -1, None

    def _pin_marker(self, v: Vehicle, pi: int, node_id: str):
        if pi < len(v._seg_lengths):
            seg_len = v._seg_lengths[pi]
        else:
            seg = v.gmap.segment_between(v.path[pi], v.path[pi + 1])
            seg_len = seg.length if seg else 0.0
        v.x_marker_pidx = pi
        v.x_marker_offset = seg_len
        v.x_marker_node = node_id

    def _pin_marker_at_dist(self, v: Vehicle, dist: float):
        remaining = dist
        pi = v.path_idx
        off = v.seg_offset + remaining
        while pi < len(v.path) - 1:
            seg_len = v._seg_lengths[pi] if pi < len(v._seg_lengths) else 0.0
            if off <= seg_len or pi >= len(v.path) - 2:
                v.x_marker_pidx = pi
                v.x_marker_offset = min(off, seg_len)
                v.x_marker_node = None
                return
            off -= seg_len
            pi += 1
        v.x_marker_pidx = max(0, len(v.path) - 2)
        v.x_marker_offset = v._seg_lengths[v.x_marker_pidx] \
            if v.x_marker_pidx < len(v._seg_lengths) else 0.0
        v.x_marker_node = None

    # ── Dest / lookahead / go ─────────────────────────────────────────────

    def _dist_to_dest(self, v: Vehicle) -> float:
        if v.dest_node is None or v.dest_reached:
            return float('inf')
        dist = v.current_seg_length() - v.seg_offset
        pi = v.path_idx
        while pi + 1 < len(v.path):
            if v.path[pi + 1] == v.dest_node:
                if dist < 1.0:
                    v.dest_reached = True
                return max(0.0, dist)
            pi += 1
            if pi + 1 < len(v.path):
                dist += v._seg_lengths[pi] if pi < len(v._seg_lengths) else 0
            if dist > 200000:
                break
        return float('inf')

    def _lookahead_speed(self, v: Vehicle) -> Tuple[float, float]:
        cur_speed = v.current_seg_speed()
        dist = v.current_seg_length() - v.seg_offset
        pi = v.path_idx
        best_v = cur_speed
        best_dist = float('inf')
        max_look = v.v_max * v.v_max / (2 * v.d_max) + 2000
        while dist < max_look and pi + 1 < len(v.path) - 1:
            pi += 1
            seg_spd = v._seg_speeds[pi] if pi < len(v._seg_speeds) else v.v_max
            if seg_spd < best_v:
                margin = max(500, v.vel * 0.5)
                safe_dist = max(0.0, dist - margin)
                v_safe = math.sqrt(seg_spd * seg_spd + 2 * v.d_max * safe_dist)
                if v_safe < best_v:
                    best_v = v_safe
                    if best_dist == float('inf'):
                        best_dist = dist
            dist += v._seg_lengths[pi] if pi < len(v._seg_lengths) else 0
        return best_v, best_dist

    def _go(self, t: float, v: Vehicle, target_v: float):
        # Use a tolerance band to prevent oscillation when lookahead
        # speed changes slightly between replans
        tol = 50.0
        if v.vel < target_v - tol:
            v.acc = v.a_max
            v.state = ACCEL
        elif v.vel > target_v + tol:
            v.acc = -v.d_max
            v.state = DECEL
        else:
            # Within tolerance — cruise at current speed (don't snap)
            v.acc = 0.0
            v.state = CRUISE
