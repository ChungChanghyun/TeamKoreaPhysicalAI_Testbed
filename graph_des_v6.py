"""
graph_des_v6.py — Pure DES engine for OHT networks.

Key differences from v5 (hybrid DES + time-stepping):
- Pure DES: events are self-sustaining. No external step() pump required.
- Explicit event types: SEG_END, PHASE_DONE, STOPPED, ZCU_GRANT, REPLAN
- ZCU lock/queue: event-driven notification instead of polling
- Leader→follower subscription: O(k) notification instead of O(N) scan
- Rendering completely separated from simulation via query_positions(t)

Continuous trajectory guarantee:
  Between events, motion is constant-acceleration.
  Position at any time t: seg_offset + vel*(t-t_ref) + 0.5*acc*(t-t_ref)²
  This is computed analytically — no discretization error.
"""

from __future__ import annotations
import math, heapq, collections
from typing import Dict, List, Optional, Tuple

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
EV_REPLAN     = 'REPLAN'       # generic re-evaluate decision
EV_SEG_END    = 'SEG_END'      # reached end of current segment
EV_PHASE_DONE = 'PHASE_DONE'   # kinematic phase complete (reached target speed)
EV_STOPPED    = 'STOPPED'      # decelerated to v=0
EV_ZCU_GRANT  = 'ZCU_GRANT'   # ZCU released, waiting vehicle may proceed
EV_BOUNDARY   = 'BOUNDARY'    # reached plan boundary (braking point to ZCU or leader)


# ── Kinematics helper ────────────────────────────────────────────────────────

def _time_to_travel(v0: float, acc: float, dist: float, v_max: float) -> float:
    """Exact time to travel `dist` given initial velocity, acceleration, speed cap.

    Returns float('inf') if the vehicle stops before reaching dist.
    """
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
        # Might stop before reaching dist
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

        # Kinematic state (piecewise-constant acceleration)
        self.vel: float = 0.0
        self.acc: float = 0.0
        self.t_ref: float = 0.0          # reference time for kinematics

        self.v_max: float = 3600.0
        self.a_max: float = 500.0
        self.d_max: float = 500.0
        self.length: float = 750.0
        self.h_min: float = self.length + 400

        self.state: str = IDLE
        self.token: int = 0
        self.next_event_t: float = 0.0

        # Leader / follower subscription
        self.leader: Optional[Vehicle] = None
        self.followers: List[Vehicle] = []  # vehicles that have me as leader

        # X marker (ZCU)
        self.stop_dist: Optional[float] = None
        self.x_marker_pidx: int = 0
        self.x_marker_offset: float = 0.0
        self.x_marker_node: Optional[str] = None

        # Destination
        self.dest_node: Optional[str] = None
        self.dest_reached: bool = False

        # ZCU wait state
        self.waiting_at_zcu: Optional[str] = None
        # ZCU nodes that have been granted passage (cleared on actual crossing)
        self.passed_zcu: set = set()

        # Render cache (written by query_positions, read by renderer)
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self.gap_to_leader: float = float('inf')

        # Segment cache
        self._seg_lengths: List[float] = []
        self._seg_speeds: List[float] = []
        self._rebuild_seg_cache()

    # ── Segment cache ─────────────────────────────────────────────────────

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

    # ── Path helpers ──────────────────────────────────────────────────────

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

    def dist_to_seg_end(self) -> float:
        return max(0.0, self.current_seg_length() - self.seg_offset)

    # ── Kinematics (analytical, between events) ──────────────────────────

    def _dist_traveled(self, dt: float) -> float:
        """Distance traveled in dt seconds from t_ref, respecting v_max and v=0 caps."""
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

    def abs_path_dist(self) -> float:
        d = 0.0
        for i in range(min(self.path_idx, len(self._seg_lengths))):
            d += self._seg_lengths[i]
        d += self.seg_offset
        return d

    # ── Position advance (mutating — called at event time) ───────────────

    def advance_position(self, t: float):
        dist = self._dist_traveled(t - self.t_ref)
        self.vel = self.vel_at(t)
        self.t_ref = t
        self.seg_offset += dist
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
        """Advance position and reset acceleration (used at replan points)."""
        self.advance_position(t)
        self.acc = 0.0

    # ── Render position (non-mutating — for external query) ──────────────

    def update_render(self, t: float):
        dist = self._dist_traveled(t - self.t_ref)
        offset = self.seg_offset + dist
        pidx = self.path_idx

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

        if pidx < len(self.path) - 1:
            seg = self.gmap.segment_between(self.path[pidx], self.path[pidx + 1])
            if seg and seg.path_points:
                self.x, self.y, self.theta = _interp_path(seg.path_points, max(0, offset))
                return

        nid = self.path[min(pidx, len(self.path) - 1)]
        node = self.gmap.nodes.get(nid)
        if node:
            self.x, self.y = node.x, node.y

    # ── Path extension ────────────────────────────────────────────────────

    def needs_path_extension(self) -> bool:
        return self.path_idx >= len(self.path) - 10

    def extend_path(self, new_nodes: List[str]):
        if self.path_idx > 5:
            trim = self.path_idx - 2
            self.path = self.path[trim:]
            self.path_idx -= trim
            self._seg_lengths = self._seg_lengths[trim:]
            self._seg_speeds = self._seg_speeds[trim:]
            # Adjust X marker index
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
    __slots__ = ('t', 'kind', 'vid', 'token')

    def __init__(self, t: float, kind: str, vid: int, token: int):
        self.t = t
        self.kind = kind
        self.vid = vid
        self.token = token

    def __lt__(self, other):
        return self.t < other.t


# ── Pure DES Engine ───────────────────────────────────────────────────────────

class GraphDESv6:
    """Pure DES engine — events drive themselves, no external pump needed.

    Usage:
        engine = GraphDESv6(gmap)
        engine.add_vehicle(v)
        engine.start_all()

        # Simulation: advance to any time
        engine.run_until(t_end)

        # Rendering: query positions at any time (non-mutating)
        engine.query_positions(t_render)
        # then read v.x, v.y, v.theta, v.gap_to_leader
    """

    def __init__(self, gmap: GraphMap):
        self.gmap = gmap
        self.vehicles: Dict[int, Vehicle] = {}
        self.heap: List[Event] = []
        self.sim_time: float = 0.0
        self.event_count: int = 0

        # Segment occupancy (for rendering / gap only, NOT for ZCU logic)
        self._seg_occupants: Dict[Tuple[str, str], List[Vehicle]] = \
            collections.defaultdict(list)

        # Re-entrancy guard for follower notification
        self._in_notify: bool = False

        # ── ZCU lock system ──────────────────────────────────────────────
        # Boundary nodes: where OHTs must stop/check before entering a zone.
        #   Merge:  boundary = predecessor nodes (A, B) before the merge node
        #   Diverge: boundary = the diverge node itself
        #
        # boundary_node_id → zone object
        self._boundary_to_zone: Dict[str, ZCUZone] = {}
        # zone.node_id → vehicle holding the lock (or None)
        self._zone_lock: Dict[str, Optional[Vehicle]] = {}
        # zone.node_id → [vehicles waiting for lock]
        self._zone_waiters: Dict[str, List[Vehicle]] = collections.defaultdict(list)
        # Set of all boundary node IDs (for fast lookup in _find_first_boundary)
        self._boundary_nodes: set = set()

        self._build_zcu_locks()

    # ── Vehicle management ────────────────────────────────────────────────

    def add_vehicle(self, v: Vehicle):
        self.vehicles[v.id] = v
        key = (v.seg_from, v.seg_to)
        self._seg_occupants[key].append(v)

    # ── Event posting / invalidation ─────────────────────────────────────

    def _post(self, t: float, kind: str, v: Vehicle):
        v.next_event_t = t
        heapq.heappush(self.heap, Event(t, kind, v.id, v.token))

    def _invalidate(self, v: Vehicle):
        v.token += 1

    # ── Public API ────────────────────────────────────────────────────────

    def run_until(self, t_end: float):
        """Process all events up to t_end.  Pure DES — no rendering side-effects."""
        while self.heap and self.heap[0].t <= t_end:
            ev = heapq.heappop(self.heap)
            v = self.vehicles.get(ev.vid)
            if v is None or ev.token != v.token:
                continue
            self.sim_time = ev.t
            self.event_count += 1
            self._dispatch(ev, v)

    def query_positions(self, t: float):
        """Compute render-state for every vehicle at time t (non-mutating)."""
        for v in self.vehicles.values():
            v.update_render(t)
            if v.leader is not None:
                g, _ = self.gap(v, t)
                v.gap_to_leader = g
            else:
                v.gap_to_leader = float('inf')

    def step(self, t_now: float):
        """Backwards-compatible wrapper: run_until + query_positions."""
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

    # ── Event handlers ────────────────────────────────────────────────────

    def _on_seg_end(self, t: float, v: Vehicle):
        """Vehicle reached end of current segment → update occupancy, replan."""
        old_key = (v.seg_from, v.seg_to)
        v.advance_position(t)
        new_key = (v.seg_from, v.seg_to)
        self._update_occupancy(v, old_key, new_key, t)

        # Clear passed_zcu for nodes we've actually crossed
        if v.passed_zcu:
            # seg_from is the node we just arrived at
            v.passed_zcu.discard(v.seg_from)

        if v.needs_path_extension():
            ext = random_safe_path(self.gmap, v.path[-1], length=100)
            v.extend_path(ext)

        self._replan(t, v)

    def _on_phase_done(self, t: float, v: Vehicle):
        """Kinematic phase complete (reached target speed) → now cruising, replan."""
        v.advance_position(t)
        v.acc = 0.0
        v.state = CRUISE
        self._replan(t, v)

    def _on_stopped(self, t: float, v: Vehicle):
        """Vehicle decelerated to v=0."""
        v.set_state(t)
        v.vel = 0.0
        v.acc = 0.0
        v.state = STOP

        # Check if stopped at a ZCU boundary → try lock or wait
        bnd_node = v.x_marker_node
        zone = self._boundary_to_zone.get(bnd_node) if bnd_node else None
        if zone:
            if self._zone_request(v, zone):
                # Lock granted while stopping → can proceed
                v.passed_zcu.add(bnd_node)
                self._post(t + 0.01, EV_REPLAN, v)
            else:
                # Lock denied → wait in queue
                self._zone_wait(v, zone)
        else:
            # Not ZCU-related stop → replan after short delay
            self._post(t + 0.2, EV_REPLAN, v)

        self._notify_followers(t, v)

    def _on_zcu_grant(self, t: float, v: Vehicle):
        """ZCU released — re-evaluate whether we can proceed."""
        v.waiting_at_zcu = None
        self._replan(t, v)

    def _on_boundary(self, t: float, v: Vehicle):
        """Reached plan boundary (braking point before ZCU or leader limit).

        At this point, decide whether to extend the plan or brake:
        - ZCU boundary: check occupancy → pass through or stop
        - Leader boundary: replan with updated gap
        """
        old_key = (v.seg_from, v.seg_to)
        v.set_state(t)
        new_key = (v.seg_from, v.seg_to)
        if old_key != new_key:
            self._update_occupancy(v, old_key, new_key, t)

        # If boundary was a ZCU boundary node, try to acquire lock
        bnd_node = v.x_marker_node
        zone = self._boundary_to_zone.get(bnd_node) if bnd_node else None

        if zone:
            if self._zone_request(v, zone):
                # Lock granted → extend plan past this boundary
                v.passed_zcu.add(bnd_node)
                self._replan(t, v)
                return

            # Lock denied → brake to stop at boundary
            bnd_dist, bnd_pi, _ = self._find_first_boundary(v)
            if bnd_dist == float('inf'):
                bnd_dist = 0.0  # already at boundary
            if v.vel > 0.1 and bnd_dist > 0.1:
                decel = min(v.vel * v.vel / (2 * bnd_dist), v.d_max)
                v.acc = -decel
                v.state = DECEL
                t_stop = v.vel / decel
                self._pin_marker_at_dist(v, bnd_dist)
                self._post(t + t_stop, EV_STOPPED, v)
            else:
                v.vel = 0.0
                v.acc = 0.0
                v.state = STOP
                self._pin_marker_at_dist(v, 0)
                self._zone_wait(v, zone)
            self._notify_followers(t, v)
            return

        # Non-ZCU boundary (leader, dest, etc.) → full replan
        self._replan(t, v)

    # ── Core: _replan() ───────────────────────────────────────────────────

    def _replan(self, t: float, v: Vehicle):
        """Central decision function.  Determines motion + schedules EXACT next event.

        Plan boundary = min(first ZCU ahead, leader free_dist, dest_dist).
        Events are only scheduled within the plan boundary.
        At the boundary's braking point, EV_BOUNDARY fires to decide
        whether to extend (ZCU open / leader moved) or stop.
        """
        old_key = (v.seg_from, v.seg_to)
        old_vel = v.vel
        v.set_state(t)
        new_key = (v.seg_from, v.seg_to)

        if old_key != new_key:
            self._update_occupancy(v, old_key, new_key, t)

        # Extend path if needed
        if v.needs_path_extension():
            ext = random_safe_path(self.gmap, v.path[-1], length=100)
            v.extend_path(ext)

        # End of path
        if v.path_idx >= len(v.path) - 1:
            v.vel = 0.0
            v.acc = 0.0
            v.state = STOP
            return

        # Destination check
        if v.dest_node and not v.dest_reached:
            if v.seg_from == v.dest_node or \
               (v.path_idx > 0 and v.path[v.path_idx] == v.dest_node):
                v.dest_reached = True
        if v.dest_node and v.dest_reached:
            v.vel = 0.0
            v.acc = 0.0
            v.state = STOP
            self._notify_followers(t, v)
            return

        # ── Compute constraints ──────────────────────────────────────────

        seg_speed = v.current_seg_speed()
        target_v = min(v.v_max, seg_speed)

        # Look-ahead braking for curves
        lookahead_v, dist_to_slow = self._lookahead_speed(v)
        target_v = min(target_v, lookahead_v)

        # First boundary node ahead (ZCU entry/diverge point)
        bnd_dist, bnd_pi, bnd_node = self._find_first_boundary(v)

        # Leader limit
        leader = v.leader
        leader_vel_now = 0.0
        leader_free = float('inf')
        if leader is not None:
            gap_d, leader_vel_now = self.gap(v, t)
            leader_extra = self._leader_extra_dist(leader, t)
            leader_free = min(gap_d + leader_extra - v.h_min,
                              gap_d - v.h_min)

        # Dest limit
        dest_dist = self._dist_to_dest(v)

        # Plan boundary = min of all constraints
        plan_boundary = min(bnd_dist, leader_free, dest_dist)

        # Pin marker at plan boundary
        if plan_boundary < 100000:
            if bnd_dist <= leader_free and bnd_dist <= dest_dist and bnd_node:
                # Boundary is a ZCU boundary node
                self._pin_marker(v, bnd_pi, bnd_node)
            else:
                # Boundary is leader or dest
                self._pin_marker_at_dist(v, plan_boundary)
            v.stop_dist = plan_boundary
        else:
            v.stop_dist = None
            self._pin_marker_at_dist(v, v.dist_to_seg_end() + 5000)

        # ── Decision + exact event scheduling ────────────────────────────

        # Case 1: blocked — stop immediately
        if plan_boundary <= 0:
            v.vel = 0.0
            v.acc = 0.0
            v.state = STOP
            v.stop_dist = 0.0
            self._pin_marker_at_dist(v, 0)
            # Check if blocked by ZCU boundary
            zone = self._boundary_to_zone.get(bnd_node) if bnd_node else None
            if zone:
                if not self._zone_request(v, zone):
                    self._zone_wait(v, zone)
                else:
                    v.passed_zcu.add(bnd_node)
                    self._post(t + 0.01, EV_REPLAN, v)
            elif leader is not None:
                pass  # woken by _notify_followers
            else:
                self._post(t + 0.5, EV_REPLAN, v)
            self._notify_followers(t, v)
            return

        # Case 2: no constraint within range
        if plan_boundary > 100000 and dist_to_slow > 100000:
            self._go(t, v, target_v)
            self._schedule_next_event(t, v, target_v, plan_boundary, dist_to_slow)
            self._notify_if_changed(t, v, old_vel)
            return

        # Case 3: constrained by plan boundary
        v_safe = math.sqrt(max(0, 2 * v.d_max * plan_boundary)) \
                 if plan_boundary < 100000 else v.v_max
        v_target = min(target_v, v_safe)

        if leader_vel_now > 0.1 and leader_vel_now <= v_safe:
            v_target = min(target_v, max(v_target, leader_vel_now))

        brake_dist = v.braking_distance()

        # Case 3a: within braking distance of boundary
        if plan_boundary < 100000 and plan_boundary <= brake_dist:
            if v.vel > 0.1 and plan_boundary > 0.1:
                decel = min(v.vel * v.vel / (2 * plan_boundary), v.d_max)
                v.acc = -decel
                v.state = DECEL
                t_stop = v.vel / decel
                self._post(t + t_stop, EV_STOPPED, v)
            else:
                v.vel = 0.0
                v.acc = 0.0
                v.state = STOP
                v.stop_dist = 0.0
                self._pin_marker_at_dist(v, 0)
                zone = self._boundary_to_zone.get(bnd_node) if bnd_node else None
                if zone:
                    if not self._zone_request(v, zone):
                        self._zone_wait(v, zone)
                    else:
                        v.passed_zcu.add(bnd_node)
                        self._post(t + 0.01, EV_REPLAN, v)
                elif leader is not None:
                    pass
                else:
                    self._post(t + 0.3, EV_REPLAN, v)
        # Case 3b: approaching boundary, schedule BOUNDARY event at braking point
        else:
            self._go(t, v, v_target)
            self._schedule_next_event(t, v, v_target, plan_boundary, dist_to_slow)

        self._notify_if_changed(t, v, old_vel)

    # ── Exact event scheduling ────────────────────────────────────────────

    def _schedule_next_event(self, t: float, v: Vehicle, target_v: float,
                             plan_boundary: float, dist_to_slow: float):
        """Compute the EXACT next event time from kinematics.

        Candidates (in priority order):
        1. SEG_END    — vehicle reaches end of current segment
        2. PHASE_DONE — vehicle reaches target speed (during accel)
        3. BOUNDARY   — vehicle reaches braking point before plan boundary
        4. REPLAN     — vehicle approaches a slow segment (curve)

        Key insight: during ACCEL, braking_distance changes every moment.
        So we defer BOUNDARY until PHASE_DONE, which gives a stable cruise
        speed for exact approach time computation.
        """
        best_t = float('inf')
        best_kind = EV_REPLAN

        # 1. Segment end
        t_seg = _time_to_travel(v.vel, v.acc, v.dist_to_seg_end(), v.v_max)
        if t + t_seg < best_t:
            best_t = t + t_seg
            best_kind = EV_SEG_END

        # 2. Phase done (accel → cruise)
        accel_phase = v.acc > 0 and v.vel < target_v - 0.1
        if accel_phase:
            t_phase = (target_v - v.vel) / v.acc
            if t + t_phase < best_t:
                best_t = t + t_phase
                best_kind = EV_PHASE_DONE

        # 3. Plan boundary approach — only when NOT accelerating
        if not accel_phase:
            if plan_boundary < 100000:
                brake_d = v.braking_distance()
                approach_dist = max(0, plan_boundary - brake_d)
                if approach_dist > 0 and v.vel > 0:
                    t_approach = approach_dist / v.vel
                else:
                    t_approach = 0.01
                if t + t_approach < best_t:
                    best_t = t + t_approach
                    best_kind = EV_BOUNDARY

            # 4. Slow segment approach (curve braking)
            if dist_to_slow < 100000:
                brake_d = v.braking_distance()
                approach_dist = max(0, dist_to_slow - brake_d - 500)
                if approach_dist > 0 and v.vel > 0:
                    t_slow = approach_dist / v.vel
                else:
                    t_slow = 0.01
                if t + t_slow < best_t:
                    best_t = t + t_slow
                    best_kind = EV_REPLAN

        # Clamp minimum dt to prevent infinite loops
        if best_t <= t + 0.005:
            best_t = t + 0.01
            best_kind = EV_REPLAN

        # Fallback: if nothing scheduled (shouldn't happen), replan in 2s
        if best_t == float('inf'):
            best_t = t + 2.0
            best_kind = EV_REPLAN

        self._post(best_t, best_kind, v)

    # ── Leader / follower ─────────────────────────────────────────────────

    def assign_leaders(self):
        """Assign leaders and build follower subscription lists."""
        for v in self.vehicles.values():
            v.followers = []

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

        # Build follower lists for O(k) notification
        for v in self.vehicles.values():
            if v.leader is not None:
                v.leader.followers.append(v)

    def _notify_followers(self, t: float, v: Vehicle):
        """Notify only v's direct followers.  O(k) instead of O(N)."""
        if self._in_notify:
            return
        self._in_notify = True
        for f in v.followers:
            if f.state in (STOP, IDLE, LOADING):
                # Stopped follower waiting for leader → wake it up
                if f.state == STOP and f.waiting_at_zcu is None:
                    self._invalidate(f)
                    self._post(t + 0.01, EV_REPLAN, f)
                continue
            self._invalidate(f)
            self._replan(t, f)
        self._in_notify = False

    def _notify_if_changed(self, t: float, v: Vehicle, old_vel: float):
        """Only notify followers if our motion changed significantly."""
        if abs(v.vel - old_vel) > 50 or \
           (v.acc < -10 and old_vel > 10) or \
           (v.state == STOP and old_vel > 10):
            self._notify_followers(t, v)

    # ── Gap computation ───────────────────────────────────────────────────

    def gap(self, follower: Vehicle, t: float) -> Tuple[float, float]:
        leader = follower.leader
        if leader is None:
            return float('inf'), 0.0

        f_off = follower.seg_offset + follower._dist_traveled(t - follower.t_ref)
        l_off = leader.seg_offset + leader._dist_traveled(t - leader.t_ref)

        if follower.seg_from == leader.seg_from and \
           follower.seg_to == leader.seg_to:
            return max(0, l_off - f_off), leader.vel_at(t)

        # Walk path forward
        dist = 0.0
        if follower.path_idx < len(follower._seg_lengths):
            dist += follower._seg_lengths[follower.path_idx] - f_off
        else:
            seg = follower.current_segment()
            dist += (seg.length if seg else 0) - f_off

        max_look = min(follower.path_idx + 80, len(follower.path) - 1)
        for i in range(follower.path_idx + 1, max_look):
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

    # ── ZCU lock system ─────────────────────────────────────────────────

    def _build_zcu_locks(self):
        """Build boundary-node → zone mapping.

        Merge:  boundary nodes = predecessors of merge node (entry points A, B)
        Diverge: boundary node = diverge node itself
        """
        for zone in self.gmap.zcu_zones:
            self._zone_lock[zone.node_id] = None

            if zone.kind == 'merge':
                # Boundary = predecessor nodes (start of entry segments)
                for seg_key in zone.all_segs():
                    pred_node = seg_key[0]  # (pred, merge_node)
                    self._boundary_to_zone[pred_node] = zone
                    self._boundary_nodes.add(pred_node)
            elif zone.kind == 'diverge':
                # Boundary = diverge node itself
                self._boundary_to_zone[zone.node_id] = zone
                self._boundary_nodes.add(zone.node_id)

        print(f"ZCU locks: {len(self._zone_lock)} zones, "
              f"{len(self._boundary_nodes)} boundary nodes")

    def _zone_request(self, v: Vehicle, zone: ZCUZone) -> bool:
        """Request lock on a zone. Returns True if granted."""
        holder = self._zone_lock.get(zone.node_id)
        if holder is None or holder is v:
            self._zone_lock[zone.node_id] = v
            return True
        return False

    def _zone_release(self, t: float, v: Vehicle, zone: ZCUZone):
        """Release lock on a zone and wake first waiter."""
        if self._zone_lock.get(zone.node_id) is v:
            self._zone_lock[zone.node_id] = None
            # Wake waiters
            waiters = self._zone_waiters.get(zone.node_id, [])
            if waiters:
                next_v = waiters.pop(0)
                next_v.waiting_at_zcu = None
                self._invalidate(next_v)
                self._post(t + 0.01, EV_ZCU_GRANT, next_v)

    def _zone_wait(self, v: Vehicle, zone: ZCUZone):
        """Register vehicle as waiting for a zone lock."""
        v.waiting_at_zcu = zone.node_id
        if v not in self._zone_waiters[zone.node_id]:
            self._zone_waiters[zone.node_id].append(v)

    # ── Segment occupancy (rendering/gap only) ────────────────────────────

    def _update_occupancy(self, v: Vehicle, old_key: Tuple[str, str],
                          new_key: Tuple[str, str], t: float):
        if old_key == new_key:
            return
        if v in self._seg_occupants[old_key]:
            self._seg_occupants[old_key].remove(v)
        if v not in self._seg_occupants[new_key]:
            self._seg_occupants[new_key].append(v)

        # Release zone lock when we leave a zone's segments
        old_zone = self.gmap.seg_to_zone.get(old_key)
        if old_zone and self._zone_lock.get(old_zone.node_id) is v:
            # Check if we're still on any segment of this zone
            still_in = False
            for sk in old_zone.all_segs():
                if v in self._seg_occupants.get(sk, []):
                    still_in = True
                    break
            if not still_in:
                self._zone_release(t, v, old_zone)

    # ── Boundary distance ─────────────────────────────────────────────────

    def _find_first_boundary(self, v: Vehicle) -> Tuple[float, int, Optional[str]]:
        """Find distance to the first boundary node ahead (not yet granted).

        Returns (dist, pi, boundary_node_id).
        v.path[pi+1] == boundary_node_id.
        If none found, returns (inf, -1, None).
        """
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
        """Pin X marker at `dist` mm ahead of the vehicle's current position.

        Walks the path forward from (path_idx, seg_offset) to find the
        exact (pidx, offset) for the marker.
        """
        remaining = dist
        pi = v.path_idx
        off = v.seg_offset + remaining

        while pi < len(v.path) - 1:
            if pi < len(v._seg_lengths):
                seg_len = v._seg_lengths[pi]
            else:
                seg = v.gmap.segment_between(v.path[pi], v.path[pi + 1])
                seg_len = seg.length if seg else 0.0
            if off <= seg_len or pi >= len(v.path) - 2:
                # Marker is within this segment
                v.x_marker_pidx = pi
                v.x_marker_offset = min(off, seg_len)
                v.x_marker_node = None
                return
            off -= seg_len
            pi += 1

        # Past end of path — pin at last segment end
        v.x_marker_pidx = max(0, len(v.path) - 2)
        v.x_marker_offset = v._seg_lengths[v.x_marker_pidx] \
            if v.x_marker_pidx < len(v._seg_lengths) else 0.0
        v.x_marker_node = None

    # ── Destination distance ──────────────────────────────────────────────

    def _dist_to_dest(self, v: Vehicle) -> float:
        if v.dest_node is None or v.dest_reached:
            return float('inf')
        dist = v.current_seg_length() - v.seg_offset
        pi = v.path_idx
        while pi + 1 < len(v.path):
            next_node = v.path[pi + 1]
            if next_node == v.dest_node:
                if dist < 1.0:
                    v.dest_reached = True
                return max(0.0, dist)
            pi += 1
            if pi + 1 < len(v.path):
                if pi < len(v._seg_lengths):
                    dist += v._seg_lengths[pi]
                else:
                    seg = v.gmap.segment_between(v.path[pi], v.path[pi + 1])
                    dist += seg.length if seg else 0
            if dist > 200000:
                break
        return float('inf')

    # ── Look-ahead speed for curves ───────────────────────────────────────

    def _lookahead_speed(self, v: Vehicle) -> Tuple[float, float]:
        cur_speed = v.current_seg_speed()
        dist = v.current_seg_length() - v.seg_offset
        pi = v.path_idx
        best_v = cur_speed
        best_dist = float('inf')
        max_look = v.v_max * v.v_max / (2 * v.d_max) + 2000

        while dist < max_look and pi + 1 < len(v.path) - 1:
            pi += 1
            if pi < len(v._seg_speeds):
                seg_spd = v._seg_speeds[pi]
            else:
                seg = v.gmap.segment_between(v.path[pi], v.path[pi + 1])
                seg_spd = seg.max_speed if seg else v.v_max
            if seg_spd < best_v:
                margin = max(500, v.vel * 0.5)
                safe_dist = max(0.0, dist - margin)
                v_safe = math.sqrt(seg_spd * seg_spd + 2 * v.d_max * safe_dist)
                if v_safe < best_v:
                    best_v = v_safe
                    if best_dist == float('inf'):
                        best_dist = dist
            if pi < len(v._seg_lengths):
                dist += v._seg_lengths[pi]
            else:
                seg = v.gmap.segment_between(v.path[pi], v.path[pi + 1])
                dist += seg.length if seg else 0

        return best_v, best_dist

    # ── Leader extra distance ─────────────────────────────────────────────

    def _leader_extra_dist(self, leader: Vehicle, t: float) -> float:
        t_confirmed = getattr(leader, 'next_event_t', t)
        if t_confirmed <= t:
            return leader.braking_distance(leader.vel_at(t))
        dist_now = leader._dist_traveled(t - leader.t_ref)
        dist_event = leader._dist_traveled(t_confirmed - leader.t_ref)
        confirmed_travel = dist_event - dist_now
        confirmed_vel = leader.vel_at(t_confirmed)
        return confirmed_travel + leader.braking_distance(confirmed_vel)

    # ── Motion command ────────────────────────────────────────────────────

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
