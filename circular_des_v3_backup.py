"""
circular_des_v3.py — Track-position-based DES for circular loop.

Key difference from v1/v2:
- Vehicles have a continuous position on the track (not segment-bound)
- Can stop at ANY point along a segment (not just at nodes)
- Stop position = leader_position - safe_distance
- When leader moves, follower recalculates stop position

Events:
  STOP_AT   : vehicle decelerates to v=0 at a specific track position
  PASS_NODE : vehicle passes a node boundary (for path tracking)
  RESUME    : leader moved, recalculate and resume
"""

from __future__ import annotations
import math, heapq, random
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass


# ── Track ─────────────────────────────────────────────────────────────────────

class Track:
    def __init__(self, n_nodes=40, radius=20000, seg_speed=3600):
        self.n = n_nodes
        self.radius = radius
        self.max_speed = seg_speed

        # Precompute node positions and cumulative distances
        self.node_xy = []
        self.node_cum_dist = [0.0]  # cumulative distance at each node
        for i in range(n_nodes):
            angle = 2 * math.pi * i / n_nodes
            self.node_xy.append((radius * math.cos(angle), radius * math.sin(angle)))

        for i in range(n_nodes):
            p1 = self.node_xy[i]
            p2 = self.node_xy[(i + 1) % n_nodes]
            self.node_cum_dist.append(
                self.node_cum_dist[-1] + math.hypot(p2[0]-p1[0], p2[1]-p1[1]))

        self.length = self.node_cum_dist[-1]  # total track length
        self.seg_len = self.length / n_nodes   # uniform segment length

    def xy_at(self, track_pos: float) -> Tuple[float, float, float]:
        """(x, y, heading) at absolute track position."""
        track_pos = track_pos % self.length
        # Find which segment
        for i in range(self.n):
            d0 = self.node_cum_dist[i]
            d1 = self.node_cum_dist[i + 1]
            if track_pos <= d1 + 0.01:
                frac = (track_pos - d0) / (d1 - d0) if d1 > d0 else 0
                frac = max(0, min(1, frac))
                p1 = self.node_xy[i]
                p2 = self.node_xy[(i + 1) % self.n]
                x = p1[0] + (p2[0] - p1[0]) * frac
                y = p1[1] + (p2[1] - p1[1]) * frac
                th = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
                return x, y, th
        # Fallback
        return self.node_xy[0][0], self.node_xy[0][1], 0.0

    def dist_between(self, pos_a: float, pos_b: float) -> float:
        """Forward distance from pos_a to pos_b on circular track."""
        return (pos_b - pos_a) % self.length

    def node_pos(self, node_idx: int) -> float:
        """Track position of node i."""
        return self.node_cum_dist[node_idx % self.n]


# ── Vehicle ───────────────────────────────────────────────────────────────────

IDLE    = 'IDLE'
ACCEL   = 'ACCEL'
CRUISE  = 'CRUISE'
DECEL   = 'DECEL'
STOP    = 'STOP'
LOADING = 'LOADING'


class Vehicle:
    def __init__(self, vid: int, track: Track, start_pos: float, color=(200,200,200)):
        self.id = vid
        self.track = track
        self.color = color

        # Physics params
        self.v_max = track.max_speed  # mm/s
        self.a_max = 500.0            # mm/s²
        self.d_max = 500.0            # mm/s²
        self.length = 750.0           # mm
        self.h_min = self.length + 400  # mm (safe gap margin)

        # Kinematic state — piecewise constant acceleration
        self.pos = start_pos      # absolute track position (mm)
        self.vel = 0.0            # current speed (mm/s)
        self.acc = 0.0            # current acceleration (mm/s²)
        self.t_ref = 0.0          # reference time for kinematic equations

        # Target
        self.stop_pos: Optional[float] = None  # where to stop (track pos)
        self.state = IDLE

        # Leader
        self.leader: Optional[Vehicle] = None

        # Token for event invalidation
        self.token = 0

        # Rendering cache
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.gap_to_leader = float('inf')

    # ── Kinematics (constant acceleration) ────────────────────────────────

    def pos_at(self, t: float) -> float:
        dt = t - self.t_ref
        if dt < 0:
            dt = 0
        if self.acc > 0:
            # Accelerating — clamp at v_max
            t_cap = (self.v_max - self.vel) / self.acc if self.vel < self.v_max else 0
            if t_cap < 0:
                t_cap = 0
            if dt <= t_cap:
                return self.pos + self.vel * dt + 0.5 * self.acc * dt ** 2
            else:
                p_cap = self.pos + self.vel * t_cap + 0.5 * self.acc * t_cap ** 2
                return p_cap + self.v_max * (dt - t_cap)
        elif self.acc < 0:
            # Decelerating — clamp at 0
            t_stop = -self.vel / self.acc if self.acc != 0 else float('inf')
            if dt <= t_stop:
                return self.pos + self.vel * dt + 0.5 * self.acc * dt ** 2
            else:
                return self.pos + self.vel * t_stop + 0.5 * self.acc * t_stop ** 2
        else:
            return self.pos + self.vel * dt

    def vel_at(self, t: float) -> float:
        dt = t - self.t_ref
        if self.acc > 0:
            return min(self.v_max, self.vel + self.acc * dt)
        elif self.acc < 0:
            return max(0.0, self.vel + self.acc * dt)
        return self.vel

    def set_state(self, t: float):
        """Snapshot current kinematic state at time t."""
        self.pos = self.pos_at(t)
        self.vel = self.vel_at(t)
        self.t_ref = t

    def time_to_reach(self, target_pos: float, t_now: float) -> float:
        """Time from t_now to reach target_pos with current kinematics."""
        dist = target_pos - self.pos_at(t_now)
        if dist <= 0:
            return 0.0
        v = self.vel_at(t_now)
        a = self.acc
        if abs(a) < 1e-9:
            return dist / v if v > 0.1 else float('inf')
        # s = v*t + 0.5*a*t² => 0.5*a*t² + v*t - dist = 0
        disc = v * v + 2 * a * dist
        if disc < 0:
            return float('inf')
        if abs(a) < 1e-9:
            return dist / v if v > 1e-9 else float('inf')
        t1 = (-v + math.sqrt(max(0, disc))) / a
        return max(0.0, t1)

    def braking_distance(self, from_vel: float = -1) -> float:
        """Distance to stop from given velocity using d_max."""
        v = from_vel if from_vel >= 0 else self.vel
        if v <= 0:
            return 0.0
        return v * v / (2 * self.d_max)

    def safe_gap(self, leader_vel: float = 0.0) -> float:
        """Minimum safe following distance."""
        d_f = self.braking_distance(self.vel)
        d_l = self.braking_distance(leader_vel) if leader_vel > 0 else 0
        return self.h_min + max(0, d_f - d_l)

    def update_render(self, t: float):
        self.x, self.y, self.theta = self.track.xy_at(self.pos_at(t))


# ── Events ────────────────────────────────────────────────────────────────────

@dataclass
class Event:
    t: float
    kind: str
    vid: int
    token: int

    def __lt__(self, other):
        return self.t < other.t


# ── DES Engine ────────────────────────────────────────────────────────────────

class CircularDESv3:
    def __init__(self, track: Track, stop_prob=0.02, stop_dur=(2.0, 5.0)):
        self.track = track
        self.vehicles: Dict[int, Vehicle] = {}
        self.heap: List[Event] = []
        self.event_count = 0
        self.stop_prob = stop_prob
        self.stop_dur = stop_dur
        self.stops_executed = 0

    def add_vehicle(self, v: Vehicle):
        self.vehicles[v.id] = v

    def _post(self, t: float, kind: str, v: Vehicle):
        heapq.heappush(self.heap, Event(t, kind, v.id, v.token))

    def _invalidate(self, v: Vehicle):
        v.token += 1

    # ── Gap computation ───────────────────────────────────────────────────

    def gap(self, follower: Vehicle, t: float) -> Tuple[float, float]:
        """(gap_mm, leader_vel) from follower to leader."""
        leader = follower.leader
        if leader is None:
            return float('inf'), 0.0
        f_pos = follower.pos_at(t)
        l_pos = leader.pos_at(t)
        g = (l_pos - f_pos) % self.track.length
        if g < 1.0:
            g = self.track.length  # same position = full lap ahead
        return g, leader.vel_at(t)

    # ── Core: compute where to stop and when to start braking ─────────────

    def _leader_min_pos(self, leader: Vehicle, t: float) -> float:
        """Leader's minimum guaranteed future position.
        Even if leader brakes right now, it will reach at least this far."""
        pos_now = leader.pos_at(t)
        vel_now = leader.vel_at(t)
        # Leader needs at least braking_distance to stop
        return pos_now + leader.braking_distance(vel_now)

    def _plan(self, t: float, v: Vehicle):
        """Single decision function. Computes free_dist using leader's
        guaranteed minimum position, then sets accel and schedules next RESUME."""
        v.set_state(t)

        target_v = min(v.v_max, self.track.max_speed)
        leader = v.leader

        # ── No leader: free run ──────────────────────────────────────────
        if leader is None:
            self._go(t, v, target_v)
            self._post(t + 1.0, 'RESUME', v)
            return

        gap_d, leader_vel = self.gap(v, t)
        if gap_d > self.track.length * 0.4:
            self._go(t, v, target_v)
            self._post(t + 1.0, 'RESUME', v)
            return

        # ── Free distance: minimum of current gap and leader's guaranteed pos ─
        leader_min = self._leader_min_pos(leader, t)
        free_from_guaranteed = ((leader_min - v.pos) % self.track.length) - v.h_min
        free_from_gap = gap_d - v.h_min
        # Use the more conservative (smaller) value
        free_dist = min(free_from_guaranteed, free_from_gap)

        if free_dist <= 0 or free_dist > self.track.length * 0.5:
            # Too close or wrap artifact — stop
            v.acc = 0.0
            v.vel = 0.0
            v.state = STOP
            v.stop_pos = v.pos
            self._post(t + 0.2, 'RESUME', v)
            return

        # ── v_safe: max speed to stop within free_dist ───────────────────
        v_safe = math.sqrt(max(0, 2 * v.d_max * free_dist))
        v_target = min(target_v, v_safe)

        # Match leader speed if possible (gap won't shrink)
        if leader_vel > 0.1 and leader_vel <= v_safe:
            v_target = min(target_v, max(v_target, leader_vel))

        # ── Set motion ───────────────────────────────────────────────────
        brake_dist = v.braking_distance()

        # Always compute stop_pos: where I'd stop if I braked right now
        v.stop_pos = (v.pos + free_dist) % self.track.length

        if free_dist > brake_dist:
            # Room to go
            self._go(t, v, v_target)

            # Next RESUME: when I reach braking point
            future_brake = v_target * v_target / (2 * v.d_max)
            coast_dist = max(0, free_dist - future_brake)
            effective_v = v_target if v_target > 0 else 1
            t_next = coast_dist / effective_v
            if v.acc > 0:
                t_accel = max(0, (v_target - v.vel) / v.a_max)
                d_accel = v.vel * t_accel + 0.5 * v.a_max * t_accel ** 2
                t_next = t_accel + max(0, coast_dist - d_accel) / effective_v
            self._post(t + max(0.05, t_next), 'RESUME', v)
        else:
            # Must brake to stop within free_dist
            if v.vel > 0.1 and free_dist > 0.1:
                decel = min(v.vel * v.vel / (2 * free_dist), v.d_max)
                v.acc = -decel
                v.state = DECEL
                t_stop = v.vel / decel
                v.stop_pos = (v.pos + free_dist) % self.track.length
                # Re-check halfway (leader may resume)
                self._post(t + t_stop * 0.5, 'RESUME', v)
            else:
                v.acc = 0.0
                v.vel = 0.0
                v.state = STOP
                v.stop_pos = v.pos
                self._post(t + 0.2, 'RESUME', v)

    def _go(self, t: float, v: Vehicle, target_v: float):
        """Set vehicle to accelerate toward target_v or cruise at it."""
        # stop_pos always reflects where we'd stop if leader brakes now
        # (computed in _plan, not cleared here)
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
                # Reached cruise speed, or periodic re-check, or leader changed
                self._plan(ev.t, v)

            elif ev.kind == 'STOPPED':
                # Vehicle reached stop position
                # Use pos_at(t) — it already clamps correctly for decel
                v.set_state(ev.t)
                v.vel = 0.0
                v.acc = 0.0
                v.state = STOP

                # Loading/unloading?
                # Check if at a node position
                at_node = False
                for i in range(self.track.n):
                    nd = self.track.node_pos(i)
                    if abs((v.pos - nd) % self.track.length) < 50:
                        at_node = True
                        break

                if at_node and random.random() < self.stop_prob:
                    delay = random.uniform(*self.stop_dur)
                    v.state = LOADING
                    self.stops_executed += 1
                    self._post(ev.t + delay, 'RESUME', v)
                    # Notify followers
                    self._notify_followers(ev.t)
                else:
                    # Just waiting for leader — will be woken by RESUME
                    # Schedule periodic check in case leader moved
                    self._post(ev.t + 0.2, 'RESUME', v)

            elif ev.kind == 'LOADING_DONE':
                self._plan(ev.t, v)

        # Update rendering
        for v in self.vehicles.values():
            v.update_render(t_now)
            g, _ = self.gap(v, t_now)
            v.gap_to_leader = g

    def _notify_followers(self, t: float):
        """Notify all active followers to replan.
        Each follower re-evaluates its gap and adjusts accordingly."""
        for v in self.vehicles.values():
            if v.leader is None or v.state == LOADING:
                continue
            if v.state in (STOP, IDLE):
                continue  # already stopped, will check on own RESUME
            self._invalidate(v)
            self._plan(t, v)

    # ── Setup ─────────────────────────────────────────────────────────────

    def start_all(self):
        """Initialize all vehicles."""
        # Assign leaders (next vehicle on track)
        vlist = sorted(self.vehicles.values(), key=lambda v: v.pos)
        for i in range(len(vlist)):
            vlist[i].leader = vlist[(i + 1) % len(vlist)]

        for v in self.vehicles.values():
            self._post(0.0, 'START', v)
