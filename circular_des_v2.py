"""
circular_des_v2.py — Clean DES engine for circular loop with S-curve physics.

Event-driven: no per-frame updates. Events are scheduled at exact transition times.
Each vehicle computes its full trajectory considering leader distance.

Events:
  ARRIVE    : vehicle reaches end of segment
  DEPART    : vehicle departs node (after optional loading/unloading)
  REPLAN    : leader changed state, follower replans trajectory
"""

from __future__ import annotations
import math, heapq, random
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass

from oht_physics import (
    OHTVehicle, MotionProfile, plan_segment_profile, plan_scurve_accel,
    decel_distance, safe_distance, Phase,
    IDLE, ACCEL, CRUISE, DECEL, STOP,
)


# ── Events ────────────────────────────────────────────────────────────────────

ARRIVE = 'ARRIVE'
DEPART = 'DEPART'
REPLAN = 'REPLAN'


@dataclass
class Event:
    t: float
    kind: str
    vehicle_id: int
    token: int  # for invalidation

    def __lt__(self, other):
        return self.t < other.t


# ── Circular Track ────────────────────────────────────────────────────────────

class CircularTrack:
    """Simple circular track with N nodes."""

    def __init__(self, n_nodes: int = 40, radius: float = 20000,
                 seg_speed: float = 3600):
        self.n_nodes = n_nodes
        self.radius = radius
        self.seg_speed = seg_speed

        self.node_ids = [f"C{i}" for i in range(n_nodes)]
        self.node_xy = {}
        self.seg_length = {}

        for i in range(n_nodes):
            angle = 2 * math.pi * i / n_nodes
            self.node_xy[f"C{i}"] = (radius * math.cos(angle),
                                      radius * math.sin(angle))

        for i in range(n_nodes):
            fn, tn = f"C{i}", f"C{(i + 1) % n_nodes}"
            p1, p2 = self.node_xy[fn], self.node_xy[tn]
            self.seg_length[(fn, tn)] = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

        self.track_length = sum(self.seg_length.values())

    def next_node(self, node_id: str) -> str:
        idx = int(node_id[1:])
        return f"C{(idx + 1) % self.n_nodes}"

    def seg_between(self, from_id: str, to_id: str) -> float:
        return self.seg_length.get((from_id, to_id), 0.0)

    def interp_pos(self, from_id: str, to_id: str, frac: float) -> Tuple[float, float]:
        p1 = self.node_xy[from_id]
        p2 = self.node_xy[to_id]
        return (p1[0] + (p2[0] - p1[0]) * frac,
                p1[1] + (p2[1] - p1[1]) * frac)

    def heading(self, from_id: str, to_id: str) -> float:
        p1 = self.node_xy[from_id]
        p2 = self.node_xy[to_id]
        return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


# ── Vehicle State ─────────────────────────────────────────────────────────────

class Vehicle:
    def __init__(self, vid: int, color: tuple, track: CircularTrack,
                 start_node: str, **physics_params):
        self.id = vid
        self.color = color
        self.track = track

        # Physics
        self.a_max = physics_params.get('a_max', 500.0)
        self.d_max = physics_params.get('d_max', 500.0)
        self.j_max = physics_params.get('j_max', 1000.0)
        self.v_max = physics_params.get('v_max', 3600.0)
        self.h_min = physics_params.get('h_min', 950.0)
        self.length = physics_params.get('length', 750.0)

        # Location
        self.at_node: Optional[str] = start_node  # node if stationary, None if on segment
        self.seg_from: Optional[str] = None
        self.seg_to: Optional[str] = None
        self.seg_length: float = 0.0

        # Motion
        self.profile: Optional[MotionProfile] = None
        self.profile_start_t: float = 0.0
        self.vel: float = 0.0
        self.pos: float = 0.0  # position along current segment

        # Token for event invalidation
        self.token: int = 0

        # State
        self.state: str = IDLE
        self.loading_until: float = -1.0  # -1 = not loading

        # Leader tracking
        self.leader: Optional[Vehicle] = None

        # Rendering
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0

        # Stats
        self.gap_to_leader: float = float('inf')

        # Init position
        xy = track.node_xy[start_node]
        self.x, self.y = xy

    def update_xy(self, t: float):
        """Update world position from motion profile."""
        if self.at_node is not None:
            self.x, self.y = self.track.node_xy[self.at_node]
            self.vel = 0.0
            return

        if self.profile is not None:
            dt = t - self.profile_start_t
            self.pos, self.vel, _ = self.profile.state_at(dt)
            self.vel = max(0.0, self.vel)
            self.pos = max(0.0, min(self.pos, self.seg_length))

        if self.seg_from and self.seg_to:
            # pos_offset: accumulated position from mid-segment retries
            offset = getattr(self, '_pos_offset', 0.0)
            actual_pos = offset + self.pos
            orig_seg_len = self.track.seg_between(self.seg_from, self.seg_to)
            frac = actual_pos / orig_seg_len if orig_seg_len > 0 else 0
            frac = max(0.0, min(1.0, frac))
            self.x, self.y = self.track.interp_pos(self.seg_from, self.seg_to, frac)
            self.theta = self.track.heading(self.seg_from, self.seg_to)

    def decel_dist(self, from_speed: float = -1) -> float:
        if from_speed < 0:
            from_speed = self.vel
        return decel_distance(from_speed, self.d_max, self.j_max)

    def safe_gap(self, leader_vel: float = 0.0) -> float:
        return safe_distance(self.vel, leader_vel, self.d_max, self.j_max, self.h_min)


# ── DES Engine ────────────────────────────────────────────────────────────────

class CircularDES:
    """Event-driven simulation for circular track."""

    def __init__(self, track: CircularTrack, stop_prob: float = 0.01,
                 stop_duration: Tuple[float, float] = (2.0, 5.0)):
        self.track = track
        self.vehicles: Dict[int, Vehicle] = {}
        self.heap: List[Event] = []
        self.event_count = 0
        self.stop_prob = stop_prob
        self.stop_duration = stop_duration
        self.stops_executed = 0
        self._t = 0.0

    def add_vehicle(self, v: Vehicle):
        self.vehicles[v.id] = v
        self._post(0.0, DEPART, v)

    def _post(self, t: float, kind: str, v: Vehicle):
        heapq.heappush(self.heap, Event(t, kind, v.id, v.token))

    def _invalidate(self, v: Vehicle):
        v.token += 1

    # ── Compute gap to leader ─────────────────────────────────────────────

    def _track_pos(self, v: Vehicle, t: float) -> Tuple[float, float]:
        """Return (absolute_position_on_track, velocity) for a vehicle.
        Position is measured from C0 going forward."""
        n = self.track.n_nodes
        if v.at_node is not None:
            idx = int(v.at_node[1:])
            pos = 0.0
            for i in range(idx):
                pos += self.track.seg_between(f"C{i}", f"C{(i+1)%n}")
            return pos, 0.0
        else:
            v.update_xy(t)
            idx = int(v.seg_from[1:])
            pos = 0.0
            for i in range(idx):
                pos += self.track.seg_between(f"C{i}", f"C{(i+1)%n}")
            offset = getattr(v, '_pos_offset', 0.0)
            pos += offset + v.pos
            return pos, v.vel

    def _compute_gap(self, follower: Vehicle, t: float) -> Tuple[float, float]:
        """Compute (gap_mm, leader_vel). Gap = track distance from follower to leader."""
        leader = follower.leader
        if leader is None:
            return float('inf'), 0.0
        f_pos, _ = self._track_pos(follower, t)
        l_pos, l_vel = self._track_pos(leader, t)
        gap = (l_pos - f_pos) % self.track.track_length
        # gap=0 means same position — treat as full track (they haven't lapped)
        if gap < 1.0:
            gap = self.track.track_length
        return gap, l_vel

    # ── Core events ───────────────────────────────────────────────────────

    def step(self, t_now: float):
        """Process all events up to t_now."""
        while self.heap and self.heap[0].t <= t_now:
            ev = heapq.heappop(self.heap)
            v = self.vehicles.get(ev.vehicle_id)
            if v is None:
                continue
            # REPLAN is advisory — always process. Others require token match.
            if ev.kind != REPLAN and ev.token != v.token:
                continue
            self.event_count += 1
            self._t = ev.t

            if ev.kind == DEPART:
                self._on_depart(ev.t, v)
            elif ev.kind == ARRIVE:
                self._on_arrive(ev.t, v)
            elif ev.kind == REPLAN:
                self._on_replan(ev.t, v)
            # RETRY no longer used

        # Update all positions for rendering
        for v in self.vehicles.values():
            v.update_xy(t_now)
            gap, lv = self._compute_gap(v, t_now)
            v.gap_to_leader = gap

    def _on_depart(self, t: float, v: Vehicle):
        """Vehicle departs from node, enters next segment.
        Always travels the FULL segment. Exit speed based on gap to leader."""
        if v.at_node is None:
            return

        from_node = v.at_node
        to_node = self.track.next_node(from_node)
        seg_len = self.track.seg_between(from_node, to_node)

        gap, leader_vel = self._compute_gap(v, t)

        if v.leader is not None and gap < v.h_min + 10:
            # Too close — wait
            v.state = STOP
            self._post(t + 0.3, DEPART, v)
            return

        v_cruise = min(v.v_max, self.track.seg_speed)

        # Compute exit speed: how fast can we be at the end of this segment
        # and still stop before hitting leader?
        gap_after = gap - seg_len  # gap to leader after traversing this segment
        if v.leader is None or gap_after > decel_distance(v_cruise, v.d_max, v.j_max) + v.h_min:
            v_exit = v_cruise  # pass through at full speed
        elif gap_after > v.h_min:
            # Find max exit speed that allows stopping within remaining gap
            budget = gap_after - v.h_min
            lo, hi = 0.0, v_cruise
            for _ in range(20):
                mid = (lo + hi) / 2
                if decel_distance(mid, v.d_max, v.j_max) <= budget:
                    lo = mid
                else:
                    hi = mid
            v_exit = lo
        else:
            v_exit = 0.0

        v.profile = plan_segment_profile(v.vel, v_cruise, v_exit,
                                         seg_len, v.a_max, v.j_max)
        v.profile_start_t = t
        v.at_node = None
        v.seg_from = from_node
        v.seg_to = to_node
        v.seg_length = seg_len
        v.pos = 0.0
        v.vel = v.vel  # carry entry speed
        v.state = ACCEL

        arrival_t = t + v.profile.total_time
        self._post(arrival_t, ARRIVE, v)
        self._notify_followers(t, v)

    def _on_arrive(self, t: float, v: Vehicle):
        """Vehicle arrives at end of segment (always full segment)."""
        # Read exit velocity BEFORE clearing profile
        exit_vel = 0.0
        if v.profile is not None:
            _, exit_vel, _ = v.profile.state_at(v.profile.total_time)
            exit_vel = max(0.0, exit_vel)

        v.at_node = v.seg_to
        v.seg_from = None
        v.seg_to = None
        v.pos = 0.0
        v._pos_offset = 0.0
        v.profile = None
        v.vel = exit_vel
        v.state = IDLE

        # Loading/unloading?
        if random.random() < self.stop_prob:
            delay = random.uniform(*self.stop_duration)
            v.loading_until = t + delay
            v.vel = 0.0
            v.state = STOP
            self.stops_executed += 1
            self._post(t + delay, DEPART, v)
        else:
            self._post(t, DEPART, v)

        self._notify_followers(t, v)

    def _on_replan(self, t: float, v: Vehicle):
        """Leader state changed — replan if needed."""
        if v.at_node is not None:
            # At node — try to depart if stopped
            if v.state in (STOP, IDLE):
                has_depart = any(e.vehicle_id == v.id and e.token == v.token
                                and e.kind == DEPART for e in self.heap)
                if not has_depart:
                    self._post(t, DEPART, v)
            return

        if v.profile is None:
            return

        # On segment — check if current trajectory is still safe
        v.update_xy(t)
        gap, leader_vel = self._compute_gap(v, t)
        remaining = v.seg_length - v.pos
        if remaining < 1.0:
            return

        gap_at_end = gap - remaining
        current_exit_v = max(0.0, v.profile.state_at(v.profile.total_time)[1])

        # Can we still stop safely after this segment with current exit speed?
        if gap_at_end > decel_distance(current_exit_v, v.d_max, v.j_max) + v.h_min:
            return  # still safe

        # Need to replan: compute new exit speed for remaining distance
        if gap_at_end > v.h_min:
            budget = gap_at_end - v.h_min
            lo, hi = 0.0, current_exit_v
            for _ in range(20):
                mid = (lo + hi) / 2
                if decel_distance(mid, v.d_max, v.j_max) <= budget:
                    lo = mid
                else:
                    hi = mid
            new_exit = lo
        else:
            new_exit = 0.0

        # Replan: create new profile for remaining distance
        # Keep original seg_from/seg_to and seg_length for XY interpolation
        v_cruise = min(v.v_max, self.track.seg_speed)
        new_profile = plan_segment_profile(v.vel, v_cruise, new_exit,
                                           remaining, v.a_max, v.j_max)

        # The profile starts at pos=0, but we're at v.pos along the original segment.
        # Store the offset so update_xy maps profile pos to correct XY.
        v._pos_offset = v.pos
        v.profile = new_profile
        v.profile_start_t = t
        v.pos = 0.0
        v.seg_length = remaining
        v.state = DECEL if new_exit < v.vel else ACCEL

        self._invalidate(v)
        self._post(t + new_profile.total_time, ARRIVE, v)

    def _on_retry(self, t: float, v: Vehicle):
        """Vehicle stopped mid-segment, retry with updated gap info."""
        if v.at_node is not None:
            # Already at a node (maybe arrived meanwhile), just depart
            self._post(t, DEPART, v)
            return

        if v.seg_from is None:
            return

        remaining = v.seg_length - v.pos
        if remaining < 1.0:
            # Close enough to end — snap to node
            v.at_node = v.seg_to
            v.seg_from = None
            v.seg_to = None
            v.pos = 0.0
            v.vel = 0.0
            v.state = IDLE
            self._post(t, DEPART, v)
            self._notify_followers(t, v)
            return

        # Check gap from current position
        gap, leader_vel = self._compute_gap(v, t)
        travel_budget = max(0.0, gap - v.h_min) if gap < float('inf') else remaining

        if travel_budget < 1.0:
            # Still blocked — retry later
            self._post(t + 0.3, 'RETRY', v)
            return

        # Plan profile for remaining distance
        move_dist = min(remaining, travel_budget)
        v_cruise = min(v.v_max, self.track.seg_speed)
        if leader_vel > 0:
            v_cruise = min(v_cruise, leader_vel + 200)

        v.profile = plan_segment_profile(0.0, v_cruise, 0.0,
                                         move_dist, v.a_max, v.j_max)
        v.profile_start_t = t
        # Keep seg_from/seg_to/seg_length unchanged, adjust pos tracking
        # pos will be updated by profile relative to profile start
        old_pos = v.pos
        v.state = ACCEL

        # We need pos to accumulate. Override update_xy to add old_pos.
        # Simplest: adjust seg_length and treat as new sub-segment
        v.seg_length = remaining
        v.pos = 0.0
        # But we need XY to reflect the offset. Adjust seg_from to current position.
        # For circular track, interpolation uses seg_from→seg_to, so we need
        # to create a virtual start point at current position.
        # Store offset for XY calculation
        v._pos_offset = old_pos

        arrival_t = t + v.profile.total_time
        self._invalidate(v)
        self._post(arrival_t, ARRIVE, v)
        self._notify_followers(t, v)

    def _notify_followers(self, t: float, leader: Vehicle):
        """Notify all vehicles following this leader to replan.
        Does NOT invalidate existing events — REPLAN is advisory."""
        for v in self.vehicles.values():
            if v.leader is leader and v is not leader:
                self._post(t, REPLAN, v)

    # ── Setup helpers ─────────────────────────────────────────────────────

    def assign_leaders(self):
        """Assign each vehicle its leader (vehicle directly ahead on the loop)."""
        # Sort vehicles by their starting position on the track
        ordered = []
        for v in self.vehicles.values():
            if v.at_node is not None:
                idx = int(v.at_node[1:])
                ordered.append((idx, v))
        ordered.sort(key=lambda x: x[0])

        for i in range(len(ordered)):
            follower = ordered[i][1]
            leader = ordered[(i + 1) % len(ordered)][1]
            follower.leader = leader
