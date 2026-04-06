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
from follower_plan import (
    MotionEvent as ME,
    compute_follower_events as _compute_follower,
    state_at as _fp_state_at,
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

        # Plan generation (incremented on replan, used to skip stale events)
        self.plan_gen: int = 0

        # Committed trajectory: full velocity profile for follower_plan
        # list of (t, dist_from_plan_start, vel, acc)
        self.committed_traj: List = []
        self.committed_traj_t0: float = 0.0

        # Committed segment entries: [(t_enter, seg_key, offset_in_plan), ...]
        # Which segments this vehicle will occupy as part of its committed plan.
        self.committed_segs: List = []

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
    __slots__ = ('t', 'kind', 'vid', 'gen')

    def __init__(self, t: float, kind: str, vid: int, gen: int = 0):
        self.t = t
        self.kind = kind
        self.vid = vid
        self.gen = gen

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

        # NOTE: merge exit = merge node only. diverge exit = successor nodes only.
        # No blanket "all successors of boundary" — _relevant_zones already
        # filters out merge locks for directions the OHT doesn't take,
        # so there's no need to release at wrong exits.

        print(f"ZCU locks: {len(self._zone_lock)} zones, "
              f"{len(self._boundary_nodes)} boundary nodes, "
              f"{len(self._exit_to_zones)} exit nodes")

    # ── Vehicle management ────────────────────────────────────────────────

    def add_vehicle(self, v: Vehicle):
        self.vehicles[v.id] = v
        self._seg_occupants[(v.seg_from, v.seg_to)].append(v)

    def _post(self, t: float, kind: str, v: Vehicle):
        v.next_event_t = t
        heapq.heappush(self.heap, Event(t, kind, v.id, v.plan_gen))

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
        # Sort segment occupant queues by offset (highest first = furthest ahead)
        for queue in self._seg_occupants.values():
            queue.sort(key=lambda v: -v.seg_offset)
        self.assign_leaders()
        for v in self.vehicles.values():
            self._post(0.0, EV_START, v)

    # ── Event dispatch ────────────────────────────────────────────────────

    def _dispatch(self, ev: Event, v: Vehicle):
        # Skip stale events from superseded plans (except SEG_END for occupancy)
        if ev.gen < v.plan_gen and ev.kind not in (EV_SEG_END, EV_START,
                                                     EV_ZCU_GRANT):
            return
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
        """Segment boundary crossed — occupancy/exit only, NO replan."""
        old_key = (v.seg_from, v.seg_to)
        crossed = v.advance_position(t)
        new_key = (v.seg_from, v.seg_to)
        self._update_occupancy(v, old_key, new_key, t)
        self._process_crossed_nodes(t, v, crossed)

        if v.needs_path_extension():
            ext = random_safe_path(self.gmap, v.path[-1], length=100)
            v.extend_path(ext)

    def _on_phase_done(self, t: float, v: Vehicle):
        """Kinematic phase complete — update state only, NO replan."""
        crossed = v.advance_position(t)
        v.acc = 0.0
        v.state = CRUISE
        self._process_crossed_nodes(t, v, crossed)
        # No replan — BOUNDARY or next phase events are already in the heap.

    def _on_stopped(self, t: float, v: Vehicle):
        crossed = v.set_state(t)
        v.vel = 0.0
        v.acc = 0.0
        v.state = STOP
        self._process_crossed_nodes(t, v, crossed)

        # Check if at a ZCU boundary — register as waiter for event-driven wakeup
        bnd_dist, _, bnd_node = self._find_first_boundary(v)
        if bnd_node and bnd_dist < 1.0:
            # Stopped right at boundary → try lock or wait
            zones = self._relevant_zones(v, bnd_node)
            for zone, lock_id in zones:
                if not self._zone_request(v, lock_id):
                    self._zone_wait(v, lock_id)
                    return
            # All granted
            v.passed_zcu.add(bnd_node)
            self._post(t + 0.01, EV_REPLAN, v)
            return

        # Not at ZCU boundary → replan
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

    def _brake_to_stop(self, t: float, v: Vehicle, stop_dist: float):
        """Decelerate to stop at stop_dist ahead. Posts SEG_END + STOPPED."""
        if v.vel > 0.1 and stop_dist > 0.1:
            decel = max(1.0, v.vel * v.vel / (2 * stop_dist))
            decel = min(decel, v.d_max)
            v.acc = -decel
            v.state = DECEL
            self._pin_marker_at_dist(v, stop_dist)
            t_stop = v.vel / decel
            # Record braking in committed trajectory
            base_dist = v.committed_traj[-1][1] if v.committed_traj else 0.0
            v.committed_traj.append((t, base_dist, v.vel, -decel))
            d_stop = base_dist + v.vel * t_stop - 0.5 * decel * t_stop ** 2
            v.committed_traj.append((t + t_stop, d_stop, 0.0, 0.0))
            t_seg = _time_to_travel(v.vel, v.acc,
                                    v.dist_to_seg_end(), v.v_max)
            if t_seg < t_stop and t_seg < float('inf'):
                self._post(t + t_seg, EV_SEG_END, v)
            self._post(t + t_stop, EV_STOPPED, v)
        else:
            v.vel = 0.0; v.acc = 0.0; v.state = STOP
            self._pin_marker_at_dist(v, 0)
            self._commit_state(v, t)
            self._post(t + 0.2, EV_REPLAN, v)

    def _on_boundary(self, t: float, v: Vehicle):
        """Reached braking point before plan boundary.

        Three cases based on what caused the boundary:
        1. ZCU node (x_marker_node set) → lock attempt
        2. Leader constraint (x_marker_node is None) → brake to stop, then REPLAN
        """
        self._trim_committed(v, t)
        bnd_node = v.x_marker_node

        # ── Case 1: ZCU boundary ────────────────────────────────────
        zones = self._relevant_zones(v, bnd_node) if bnd_node else []

        if zones:
            all_granted = True
            denied_lock_id = None
            for zone, lock_id in zones:
                if not self._zone_request(v, lock_id):
                    all_granted = False
                    denied_lock_id = lock_id
                    break

            if all_granted:
                v.passed_zcu.add(bnd_node)
                self._replan(t, v, skip_set_state=True)
                return

            # Lock denied → brake to boundary, then wait for ZCU_GRANT
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
                self._brake_to_stop(t, v, bnd_dist)
            else:
                v.vel = 0.0; v.acc = 0.0; v.state = STOP
                self._pin_marker_at_dist(v, 0)
                self._commit_state(v, t)
                self._zone_wait(v, denied_lock_id)
            return

        # ── Case 2: Leader constraint (no ZCU node) ─────────────────
        # Brake to stop at h_min behind leader's current position.
        # After STOPPED, REPLAN will re-evaluate and extend if leader moved.
        old_key = (v.seg_from, v.seg_to)
        crossed = v.set_state(t)
        new_key = (v.seg_from, v.seg_to)
        if old_key != new_key:
            self._update_occupancy(v, old_key, new_key, t)
        self._process_crossed_nodes(t, v, crossed)

        self._update_leader(v, t)
        if v.leader is not None:
            gap_d, _ = self.gap(v, t)
            remaining = self._leader_committed_remaining(v.leader, t)
            stop_dist = max(0, gap_d + remaining - v.h_min)
        else:
            stop_dist = 0.0

        self._brake_to_stop(t, v, stop_dist)

    # ── Core: _replan() ───────────────────────────────────────────────────

    def _replan(self, t: float, v: Vehicle, skip_set_state: bool = False):
        v.plan_gen += 1
        self._trim_committed(v, t)
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
            self._commit_state(v, t)
            return

        # Destination check
        if v.dest_node and not v.dest_reached:
            if v.seg_from == v.dest_node or \
               (v.path_idx > 0 and v.path[v.path_idx] == v.dest_node):
                v.dest_reached = True
        if v.dest_node and v.dest_reached:
            v.vel = 0.0; v.acc = 0.0; v.state = STOP
            self._commit_state(v, t)
            return

        # ── Constraints ──────────────────────────────────────────────────

        seg_speed = v.current_seg_speed()
        target_v = min(v.v_max, seg_speed)
        lookahead_v, dist_to_slow = self._lookahead_speed(v)
        target_v = min(target_v, lookahead_v)

        # ZCU boundary — find first un-granted boundary (no lock attempt here)
        bnd_dist, bnd_pi, bnd_node = self._find_first_boundary(v)
        # Skip boundary nodes with no relevant zones
        if bnd_dist < 100000 and bnd_node:
            zones = self._relevant_zones(v, bnd_node)
            if not zones:
                v.passed_zcu.add(bnd_node)
                bnd_dist, bnd_pi, bnd_node = self._find_first_boundary(v)

        # Path leader — refresh at each replan
        self._update_leader(v, t)
        leader = v.leader
        leader_free = float('inf')
        if leader is not None:
            gap_d, _ = self.gap(v, t)
            remaining = self._leader_committed_remaining(leader, t)
            leader_free = gap_d + remaining - v.h_min

        # Dest
        dest_dist = self._dist_to_dest(v)

        # Leader too close → immediate stop (before plan_boundary calculation)
        if leader is not None and leader_free <= 0:
            v.vel = 0.0; v.acc = 0.0; v.state = STOP; v.stop_dist = 0.0
            self._pin_marker_at_dist(v, 0)
            self._commit_state(v, t)
            self._post(t + 0.5, EV_REPLAN, v)
            return

        # Plan boundary — ZCU + dest + leader stop position
        plan_boundary = min(bnd_dist, dest_dist, leader_free)

        # Pin marker
        if plan_boundary < 100000:
            if bnd_dist <= dest_dist and bnd_dist <= leader_free and bnd_node:
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
            self._commit_state(v, t)
            # No lock attempts in _replan — locks are handled by _on_boundary
            self._post(t + 0.5, EV_REPLAN, v)
            return

        # Set initial motion and let _schedule_plan_events handle the full profile
        # including leader-following, curve speed, and ZCU boundary braking.
        self._go(t, v, target_v)
        self._schedule_plan_events(t, v, target_v, plan_boundary)

    # ── Event scheduling ──────────────────────────────────────────────────

    def _build_leader_events(self, v: Vehicle, leader: Vehicle,
                              t: float, gap_d: float) -> list:
        """Build leader's committed trajectory as MotionEvent list for follower_plan.

        Uses leader.committed_traj (full velocity profile including ZCU braking)
        so the follower can see the leader's entire plan.
        """
        traj = leader.committed_traj
        if not traj:
            # No committed trajectory available — fallback to current state
            vl = leader.vel_at(t)
            al = leader.acc
            if al < 0 and (leader.vel + leader.acc * (t - leader.t_ref)) < 1e-9:
                al = 0.0; vl = 0.0
            return [ME(t, gap_d, vl, al),
                    ME(t + 200, gap_d + vl * 200, vl, 0.0)]

        # Interpolate leader's distance at time t from committed_traj
        leader_dist_at_t = 0.0
        for i in range(len(traj)):
            ti, di, vi, ai = traj[i]
            t_next = traj[i + 1][0] if i + 1 < len(traj) else ti + 200
            if t <= t_next + 1e-9:
                dt = max(0, t - ti)
                # Clamp decel to stop
                if ai < 0 and vi > 0:
                    t_stop = vi / abs(ai)
                    dt = min(dt, t_stop)
                leader_dist_at_t = di + vi * dt + 0.5 * ai * dt * dt
                break
        else:
            # t is after all traj entries
            last_t, last_d, last_v, last_a = traj[-1]
            dt = max(0, t - last_t)
            leader_dist_at_t = last_d + last_v * dt

        # Build MotionEvents offset by gap
        events = []
        for (ti, di, vi, ai) in traj:
            if ti < t - 0.01:
                continue
            x_leader = gap_d + (di - leader_dist_at_t)
            events.append(ME(ti, x_leader, vi, ai))

        if not events:
            vl = leader.vel_at(t)
            events = [ME(t, gap_d, vl, 0.0)]

        # Extend last phase to horizon
        last = events[-1]
        if last.v > 0.1:
            events.append(ME(last.t + 200, last.x + last.v * 200, last.v, 0.0))
        else:
            events.append(ME(last.t + 200, last.x, 0, 0))

        return events

    def _schedule_plan_events(self, t: float, v: Vehicle, target_v: float,
                              plan_boundary: float):
        """Generate ALL events from current position to plan boundary.

        Uses analytical follower trajectory computation (follower_plan.py):
        1. Build leader's committed MotionEvents
        2. Compute follower's safe acceleration profile analytically
        3. Walk through the profile, posting SEG_END at segment boundaries,
           PHASE_DONE at acceleration changes, BOUNDARY at ZCU braking point
        """
        leader = v.leader

        # ── Compute leader-safe follower plan ────────────────────────
        fp = None  # follower plan: list of MotionEvent
        if leader is not None:
            gap_d, _ = self.gap(v, t)
            if gap_d < 100000:
                lead_events = self._build_leader_events(v, leader, t, gap_d)
                follower_init = ME(t, 0.0, v.vel, v.acc)
                fp = _compute_follower(
                    lead_events, follower_init,
                    v.h_min, -v.d_max, v.a_max, target_v)

        # Helper: look up planned acceleration at time sim_t
        def planned_acc_at(sim_t):
            if fp is None:
                return None
            a = fp[0].a
            for fe in fp:
                if fe.t <= sim_t + 1e-9:
                    a = fe.a
                else:
                    break
            return a

        # Helper: next follower plan transition after sim_t
        def next_plan_transition(sim_t):
            if fp is None:
                return None, None
            for fe in fp:
                if fe.t > sim_t + 1e-9:
                    return fe.t, fe.a
            return None, None

        # ── Simulation loop ──────────────────────────────────────────
        sim_vel = v.vel
        sim_acc = v.acc
        sim_t = t
        sim_dist = 0.0
        sim_pidx = v.path_idx
        sim_off = v.seg_offset
        sim_target_v = target_v
        posted_any = False
        last_kind = None
        last_t = t

        # Record committed trajectory and segment entries for followers
        base_dist = v.committed_traj[-1][1] if v.committed_traj else 0.0
        traj = [(t, base_dist, v.vel, v.acc)]
        c_segs = []  # committed_segs: [(t_enter, t_exit, seg_key, plan_dist)]
        prev_acc = v.acc
        current_leader = leader

        for _ in range(100):
            if sim_pidx >= len(v.path) - 1:
                break

            seg_len = v._seg_lengths[sim_pidx] if sim_pidx < len(v._seg_lengths) else 0.0
            remaining_in_seg = max(0.01, seg_len - sim_off)
            dist_to_bnd = plan_boundary - sim_dist

            if dist_to_bnd <= 0:
                break

            # ── Leader constraint from analytical plan ───────────────
            pa = planned_acc_at(sim_t)
            effective_target = sim_target_v

            if pa is not None:
                if pa < -1e-9 and sim_acc > pa:
                    # Plan says decelerate harder
                    sim_acc = pa
                    effective_target = 0.0
                elif pa < sim_acc and sim_vel > 0.1:
                    sim_acc = pa

            candidates = []

            # SEG_END
            dt_seg = _time_to_travel(sim_vel, sim_acc, remaining_in_seg, v.v_max)
            if dt_seg < float('inf') and remaining_in_seg <= dist_to_bnd:
                candidates.append((dt_seg, EV_SEG_END))

            # Next follower plan transition → REPLAN (so _replan re-evaluates leader)
            t_next, a_next = next_plan_transition(sim_t)
            if t_next is not None:
                dt_plan = t_next - sim_t
                if 0.001 < dt_plan:
                    candidates.append((dt_plan, EV_REPLAN))

            # PHASE_DONE for target speed (non-leader)
            if sim_acc > 0 and sim_vel < effective_target - 0.5:
                dt_phase = (effective_target - sim_vel) / sim_acc
                if dt_phase > 0.001:
                    candidates.append((dt_phase, EV_PHASE_DONE))

            # BOUNDARY (braking point for plan boundary — ZCU or leader)
            if dist_to_bnd > 0 and plan_boundary < 100000:
                if sim_acc > 0 and sim_vel < effective_target - 0.5:
                    dt_bnd = _time_to_boundary_during_accel(
                        sim_vel, sim_acc, v.d_max, dist_to_bnd, v.v_max)
                elif sim_vel > 0.1 and sim_acc >= 0:
                    brake_d = sim_vel * sim_vel / (2 * v.d_max)
                    approach = max(0, dist_to_bnd - brake_d)
                    dt_bnd = approach / sim_vel if sim_vel > 0 else float('inf')
                else:
                    dt_bnd = float('inf')
                if dt_bnd < float('inf') and dt_bnd >= 0:
                    candidates.append((max(0.001, dt_bnd), EV_BOUNDARY))

            # STOPPED
            if sim_acc < 0 and sim_vel > 0.1:
                dt_stop = sim_vel / abs(sim_acc)
                candidates.append((dt_stop, EV_STOPPED))
            elif sim_vel <= 0.1 and sim_acc <= 0:
                self._post(sim_t + 0.01, EV_STOPPED, v)
                posted_any = True
                last_kind = EV_STOPPED
                last_t = sim_t + 0.01
                break

            if not candidates:
                break

            candidates.sort()
            dt_best, kind_best = candidates[0]

            self._post(sim_t + dt_best, kind_best, v)
            posted_any = True
            last_kind = kind_best
            last_t = sim_t + dt_best

            if kind_best in (EV_BOUNDARY, EV_STOPPED, EV_REPLAN):
                # Record terminal phase in trajectory
                if kind_best == EV_REPLAN:
                    # Analytical plan says brake here — record predicted braking
                    t_r = sim_t + dt_best
                    d_r = base_dist + sim_dist + sim_vel * dt_best + 0.5 * sim_acc * dt_best ** 2
                    v_r = max(0, sim_vel + sim_acc * dt_best)
                    pa = planned_acc_at(t_r)
                    if pa is not None and pa < -1e-9 and v_r > 0.1:
                        traj.append((t_r, d_r, v_r, pa))
                        t_stop = t_r + v_r / abs(pa)
                        d_stop = d_r + v_r ** 2 / (2 * abs(pa))
                        traj.append((t_stop, d_stop, 0, 0))
                    else:
                        traj.append((t_r, d_r, v_r, sim_acc))
                elif kind_best == EV_BOUNDARY:
                    # Vehicle will brake to stop at the boundary
                    t_bnd = sim_t + dt_best
                    v_bnd = max(0, sim_vel + sim_acc * dt_best)
                    d_bnd = base_dist + sim_dist + sim_vel * dt_best + 0.5 * sim_acc * dt_best ** 2
                    traj.append((t_bnd, d_bnd, v_bnd, -v.d_max))
                    # Braking to stop
                    if v_bnd > 0.1:
                        t_stop_dt = v_bnd / v.d_max
                        d_stop = d_bnd + v_bnd * t_stop_dt - 0.5 * v.d_max * t_stop_dt ** 2
                        traj.append((t_bnd + t_stop_dt, d_stop, 0, 0))
                elif kind_best == EV_STOPPED:
                    t_s = sim_t + dt_best
                    d_s = base_dist + sim_dist + sim_vel * dt_best + 0.5 * sim_acc * dt_best ** 2
                    traj.append((t_s, d_s, 0, 0))
                break

            # Advance simulation
            if kind_best == EV_SEG_END:
                if sim_acc > 0:
                    t_cap = (v.v_max - sim_vel) / sim_acc if sim_vel < v.v_max else 0
                    sim_vel = sim_vel + sim_acc * min(dt_best, t_cap)
                    sim_vel = min(sim_vel, v.v_max)
                elif sim_acc < 0:
                    sim_vel = max(0, sim_vel + sim_acc * dt_best)

                sim_t += dt_best
                sim_dist += remaining_in_seg
                sim_pidx += 1
                sim_off = 0.0

                if sim_pidx >= len(v.path) - 1:
                    break

                # Record segment entry for committed_segs (t_enter, t_exit, seg_key, plan_dist)
                new_seg_key = (v.path[sim_pidx], v.path[sim_pidx + 1]) \
                    if sim_pidx + 1 < len(v.path) else None
                if new_seg_key:
                    # Close previous segment's t_exit
                    if c_segs:
                        prev = c_segs[-1]
                        c_segs[-1] = (prev[0], sim_t, prev[2], prev[3])
                    c_segs.append((sim_t, float('inf'), new_seg_key, base_dist + sim_dist))

                # ── Check for leader change at this segment ─────────
                # Build forward segments from sim position
                sim_fwd = {}
                sim_fwd[new_seg_key] = -sim_off  # current sim segment
                d_acc = (v._seg_lengths[sim_pidx] if sim_pidx < len(v._seg_lengths) else 0) - sim_off
                for j in range(sim_pidx + 1, min(sim_pidx + 30, len(v.path) - 1)):
                    fn = v.path[j]
                    tn = v.path[j + 1] if j + 1 < len(v.path) else None
                    if tn is None:
                        break
                    sim_fwd[(fn, tn)] = d_acc
                    sg = self.gmap.segment_between(fn, tn)
                    d_acc += sg.length if sg else 0
                    if d_acc > 20000:
                        break

                new_leader = self._find_leader_on_path(v, sim_fwd, v.id, sim_t)
                is_zcu_node = (v.path[sim_pidx] in self._boundary_nodes) if sim_pidx < len(v.path) else False

                if new_leader is not current_leader:
                    if not is_zcu_node and current_leader is not None:
                        pass  # DEBUG: leader changed at non-ZCU — could log here
                    current_leader = new_leader
                    # Rebuild analytical plan with new leader
                    if current_leader is not None:
                        gap_d_new, _ = self.gap_from_pos(
                            v, sim_pidx, sim_off, current_leader, sim_t)
                        if gap_d_new < 100000:
                            lead_events = self._build_leader_events(
                                v, current_leader, sim_t, gap_d_new)
                            follower_init = ME(sim_t, 0.0, sim_vel, sim_acc)
                            fp = _compute_follower(
                                lead_events, follower_init,
                                v.h_min, -v.d_max, v.a_max, sim_target_v)
                    else:
                        fp = None

                new_seg_speed = v._seg_speeds[sim_pidx] if sim_pidx < len(v._seg_speeds) else v.v_max
                if new_seg_speed < sim_target_v:
                    sim_target_v = new_seg_speed
                    if sim_vel > sim_target_v + 0.5:
                        sim_acc = -v.d_max

                # Record acceleration change
                if abs(sim_acc - prev_acc) > 1e-9:
                    traj.append((sim_t, base_dist + sim_dist, sim_vel, sim_acc))
                    prev_acc = sim_acc

            elif kind_best == EV_PHASE_DONE:
                dist_traveled = sim_vel * dt_best + 0.5 * sim_acc * dt_best ** 2
                sim_vel = max(0, sim_vel + sim_acc * dt_best)
                pa_new = planned_acc_at(sim_t + dt_best)
                sim_acc = pa_new if pa_new is not None else 0.0
                sim_t += dt_best
                sim_dist += dist_traveled
                sim_off += dist_traveled

                # Record phase transition
                traj.append((sim_t, base_dist + sim_dist, sim_vel, sim_acc))
                prev_acc = sim_acc

        # Close last c_segs entry's t_exit with plan end time
        if c_segs:
            last_entry = c_segs[-1]
            t_end = last_t if last_t > last_entry[0] else last_entry[0] + 10.0
            c_segs[-1] = (last_entry[0], t_end, last_entry[2], last_entry[3])

        # Save committed trajectory and segment entries
        # Extend committed trajectory (skip first if it duplicates last entry)
        if v.committed_traj and traj and abs(v.committed_traj[-1][0] - traj[0][0]) < 1e-9:
            v.committed_traj.extend(traj[1:])
        else:
            v.committed_traj.extend(traj)
        v.committed_segs.extend(c_segs)

        if not posted_any:
            self._post(t + 0.5, EV_REPLAN, v)
        elif last_kind not in (EV_BOUNDARY, EV_STOPPED):
            self._post(last_t + 0.01, EV_REPLAN, v)

    # ── Leader ────────────────────────────────────────────────────────────

    def _find_leader_on_path(self, v: Vehicle, fwd_segs: dict,
                              exclude_id: int = -1,
                              t: float = 0.0) -> Optional[Vehicle]:
        """Find nearest vehicle on forward segments using segment occupancy queues.

        O(forward_segments × occupants_per_segment) instead of O(N_vehicles).
        Single-track guarantee: queue order = position order within segment.
        """
        best_leader = None
        best_dist = float('inf')
        self_key = None
        self_off = 0.0
        for sk, sd in fwd_segs.items():
            if sd < -0.01:
                self_key = sk
                self_off = -sd
                break

        for seg_key, seg_dist in fwd_segs.items():
            for other in self._seg_occupants.get(seg_key, []):
                if other.id == exclude_id:
                    continue
                d = seg_dist + other.seg_offset
                if seg_key == self_key and other.seg_offset <= self_off:
                    continue  # behind me on same segment
                if 0 < d < best_dist:
                    best_dist = d
                    best_leader = other

        return best_leader

    def _update_leader(self, v: Vehicle, t: float = 0.0):
        """Find nearest vehicle ahead using segment occupancy queues. O(path_segments)."""
        seg_key = (v.seg_from, v.seg_to)
        queue = self._seg_occupants.get(seg_key, [])

        # Find leader on same segment (queue: index 0 = furthest ahead)
        for i, other in enumerate(queue):
            if other is v:
                if i > 0:
                    v.leader = queue[i - 1]
                    return
                break

        # No leader on current segment → walk forward segments
        look_dist = max(20000, v.vel * v.vel / (2 * v.d_max) + v.h_min + 5000)
        dist = v.current_seg_length() - v.seg_offset

        for i in range(v.path_idx + 1,
                       min(v.path_idx + 50, len(v.path) - 1)):
            fn = v.path[i]
            tn = v.path[i + 1] if i + 1 < len(v.path) else None
            if tn is None:
                break
            fwd_key = (fn, tn)
            fwd_queue = self._seg_occupants.get(fwd_key)
            if fwd_queue:
                # Nearest vehicle on forward segment = last in queue (closest to start)
                v.leader = fwd_queue[-1]
                return
            seg = self.gmap.segment_between(fn, tn)
            dist += seg.length if seg else 0
            if dist > look_dist:
                break

        v.leader = None

    def assign_leaders(self):
        """Initial leader assignment using sorted segment queues."""
        for v in self.vehicles.values():
            self._update_leader(v, 0.0)

    def gap_from_pos(self, v: Vehicle, pidx: int, offset: float,
                     leader: Vehicle, t: float) -> Tuple[float, float]:
        """Compute gap from a simulated position (pidx, offset) to leader at time t."""
        # Leader position (extrapolated)
        l_off = leader.seg_offset + leader._dist_traveled(t - leader.t_ref)
        l_pidx = leader.path_idx
        while l_pidx < len(leader.path) - 1:
            sl = leader._seg_lengths[l_pidx] if l_pidx < len(leader._seg_lengths) else 0
            if sl <= 0 or l_off < sl - 0.01:
                break
            l_off -= sl
            l_pidx += 1
        l_seg_from = leader.path[l_pidx] if l_pidx < len(leader.path) else leader.path[-1]
        l_seg_to = leader.path[l_pidx + 1] if l_pidx + 1 < len(leader.path) else leader.path[-1]

        # Walk forward from (pidx, offset) to find leader's segment
        dist = 0.0
        if pidx < len(v._seg_lengths):
            dist += v._seg_lengths[pidx] - offset
        for i in range(pidx + 1, min(pidx + 80, len(v.path) - 1)):
            fn = v.path[i]
            tn = v.path[i + 1] if i + 1 < len(v.path) else None
            if tn is None:
                break
            if fn == l_seg_from and tn == l_seg_to:
                return max(0, dist + l_off), leader.vel_at(t)
            if i < len(v._seg_lengths):
                dist += v._seg_lengths[i]
            else:
                seg = self.gmap.segment_between(fn, tn)
                dist += seg.length if seg else 0
            if dist > 100000:
                break

        # Also check committed_segs of leader
        for (*_t, seg_key, plan_dist) in leader.committed_segs:
            for i in range(pidx, min(pidx + 80, len(v.path) - 1)):
                fn = v.path[i]
                tn = v.path[i + 1] if i + 1 < len(v.path) else None
                if tn is None:
                    break
                if (fn, tn) == seg_key:
                    d = 0.0
                    if i == pidx:
                        d = -offset
                    else:
                        d = v._seg_lengths[pidx] - offset if pidx < len(v._seg_lengths) else 0
                        for j in range(pidx + 1, i):
                            if j < len(v._seg_lengths):
                                d += v._seg_lengths[j]
                    return max(0, d), leader.vel_at(t)

        return float('inf'), 0.0

    def gap(self, follower: Vehicle, t: float) -> Tuple[float, float]:
        leader = follower.leader
        if leader is None:
            return float('inf'), 0.0

        f_off = follower.seg_offset + follower._dist_traveled(t - follower.t_ref)
        l_off = leader.seg_offset + leader._dist_traveled(t - leader.t_ref)

        # Advance follower through segments to handle extrapolation overshoot
        f_pidx = follower.path_idx
        while f_pidx < len(follower.path) - 1:
            sl = follower._seg_lengths[f_pidx] if f_pidx < len(follower._seg_lengths) else 0
            if sl <= 0 or f_off < sl - 0.01:
                break
            f_off -= sl
            f_pidx += 1

        # Advance leader through segments similarly
        l_pidx = leader.path_idx
        l_seg_from = leader.seg_from
        l_seg_to = leader.seg_to
        while l_pidx < len(leader.path) - 1:
            sl = leader._seg_lengths[l_pidx] if l_pidx < len(leader._seg_lengths) else 0
            if sl <= 0 or l_off < sl - 0.01:
                break
            l_off -= sl
            l_pidx += 1
            l_seg_from = leader.path[l_pidx] if l_pidx < len(leader.path) else leader.path[-1]
            l_seg_to = leader.path[l_pidx + 1] if l_pidx + 1 < len(leader.path) else leader.path[-1]

        # Same segment check (after virtual advance)
        f_seg_from = follower.path[f_pidx] if f_pidx < len(follower.path) else follower.path[-1]
        f_seg_to = follower.path[f_pidx + 1] if f_pidx + 1 < len(follower.path) else follower.path[-1]
        if f_seg_from == l_seg_from and f_seg_to == l_seg_to:
            return max(0, l_off - f_off), leader.vel_at(t)

        # Walk forward from follower's virtual position
        dist = 0.0
        if f_pidx < len(follower._seg_lengths):
            dist += follower._seg_lengths[f_pidx] - f_off
        else:
            seg = self.gmap.segment_between(f_seg_from, f_seg_to)
            dist += (seg.length if seg else 0) - f_off

        for i in range(f_pidx + 1,
                       min(f_pidx + 80, len(follower.path) - 1)):
            fn = follower.path[i]
            tn = follower.path[i + 1] if i + 1 < len(follower.path) else None
            if tn is None:
                break
            if fn == l_seg_from and tn == l_seg_to:
                return max(0, dist + l_off), leader.vel_at(t)
            if i < len(follower._seg_lengths):
                dist += follower._seg_lengths[i]
            else:
                seg = self.gmap.segment_between(fn, tn)
                dist += seg.length if seg else 0
            if dist > 200000:
                break

        euc = math.hypot(leader.x - follower.x, leader.y - follower.y)
        return max(euc, 1.0), leader.vel_at(t)

    def _leader_committed_remaining(self, leader: Vehicle, t: float) -> float:
        """How much further the leader will travel based on its committed_traj.

        Returns the remaining committed distance from the leader's current
        position to the end of its committed plan. If committed_traj is empty,
        falls back to next_event_t based estimate.
        """
        traj = leader.committed_traj
        if not traj:
            # Fallback: use next_event_t
            t_event = leader.next_event_t
            if t_event <= t:
                return 0.0
            dist_now = leader._dist_traveled(t - leader.t_ref)
            dist_event = leader._dist_traveled(t_event - leader.t_ref)
            return max(0, dist_event - dist_now)

        # Find leader's current distance within committed_traj
        leader_dist_now = 0.0
        for i in range(len(traj)):
            ti, di, vi, ai = traj[i]
            t_next = traj[i + 1][0] if i + 1 < len(traj) else ti + 200
            if t <= t_next + 1e-9:
                dt = max(0, t - ti)
                if ai < 0 and vi > 0:
                    dt = min(dt, vi / abs(ai))
                leader_dist_now = di + vi * dt + 0.5 * ai * dt * dt
                break
        else:
            leader_dist_now = traj[-1][1]

        # Total committed distance = last entry's distance
        # (already includes braking phase if plan ends with BOUNDARY→STOPPED)
        leader_dist_end = traj[-1][1]

        return max(0, leader_dist_end - leader_dist_now)

    # ── ZCU lock ──────────────────────────────────────────────────────────

    def _zone_request(self, v: Vehicle, lock_id: str) -> bool:
        holder = self._zone_lock.get(lock_id)
        if holder is None or holder is v:
            self._zone_lock[lock_id] = v
            return True
        return False

    def _zone_holder_exit_time(self, lock_id: str) -> float:
        """Get the committed exit time of the current lock holder.

        The holder's next_event_t is committed (never cancelled).
        Walk the holder's committed events to estimate when it reaches
        the exit node. Conservative: use next_event_t + braking time
        as the earliest possible exit.
        """
        holder = self._zone_lock.get(lock_id)
        if holder is None:
            return 0.0
        # The holder's committed trajectory: it will at minimum travel
        # until next_event_t. At that point it has vel_at(next_event_t),
        # and may continue further. The exit happens when holder reaches
        # the exit node via SEG_END.
        # Best estimate: holder's next_event_t (conservative lower bound)
        return holder.next_event_t

    def _zone_release(self, t: float, lock_id: str):
        self._zone_lock[lock_id] = None
        waiters = self._zone_waiters.get(lock_id, [])
        if waiters:
            next_v = waiters.pop(0)
            next_v.waiting_at_zcu = None
            self._post(t, EV_ZCU_GRANT, next_v)

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

    # ── Committed trajectory management ──────────────────────────────────

    def _trim_committed(self, v: Vehicle, t: float):
        """Trim committed_traj: keep past entries, discard future predictions."""
        if not v.committed_traj:
            v.committed_traj_t0 = t
            return
        # Keep entries at or before current time
        v.committed_traj = [e for e in v.committed_traj if e[0] <= t + 1e-9]
        v.committed_segs = [e for e in v.committed_segs if e[0] <= t + 1e-9]
        # Trim old history to bound memory (keep last 30s)
        cutoff = t - 30.0
        if v.committed_traj and v.committed_traj[0][0] < cutoff:
            v.committed_traj = [e for e in v.committed_traj if e[0] >= cutoff]
            v.committed_segs = [e for e in v.committed_segs if e[0] >= cutoff]
        if not v.committed_traj:
            v.committed_traj_t0 = t

    def _commit_state(self, v: Vehicle, t: float):
        """Record current state as a committed trajectory entry."""
        base_dist = v.committed_traj[-1][1] if v.committed_traj else 0.0
        if v.committed_traj and abs(v.committed_traj[-1][0] - t) < 1e-9:
            v.committed_traj[-1] = (t, base_dist, v.vel, v.acc)
        else:
            v.committed_traj.append((t, base_dist, v.vel, v.acc))

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
