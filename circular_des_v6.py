"""
circular_des_v6.py — Trajectory-confirmed DES for circular track.

Core concept:
- X marker = confirmed trajectory endpoint.
- Trajectory = sequence of (t, acc) events. Immutable once confirmed.
- Only event: DECISION_POINT (braking distance from marker).
  At decision point: extend marker or start decelerating.
- Follower sees leader's full trajectory to compute gap-safe trajectory.
"""

from __future__ import annotations
import math, heapq
from typing import Dict, List, Optional, Tuple


# ── Track ─────────────────────────────────────────────────────────────────────

class Track:
    def __init__(self, n_nodes=40, radius=20000, seg_speed=3600):
        self.n = n_nodes
        self.radius = radius
        self.max_speed = seg_speed
        self.node_xy = []
        self.node_cum_dist = [0.0]
        for i in range(n_nodes):
            angle = 2 * math.pi * i / n_nodes
            self.node_xy.append((radius * math.cos(angle), radius * math.sin(angle)))
        for i in range(n_nodes):
            p1 = self.node_xy[i]
            p2 = self.node_xy[(i + 1) % n_nodes]
            self.node_cum_dist.append(
                self.node_cum_dist[-1] + math.hypot(p2[0]-p1[0], p2[1]-p1[1]))
        self.length = self.node_cum_dist[-1]

    def xy_at(self, pos: float) -> Tuple[float, float, float]:
        pos = pos % self.length
        for i in range(self.n):
            d0 = self.node_cum_dist[i]
            d1 = self.node_cum_dist[i + 1]
            if pos <= d1 or i == self.n - 1:
                t = (pos - d0) / (d1 - d0) if d1 > d0 else 0
                p0 = self.node_xy[i]
                p1 = self.node_xy[(i + 1) % self.n]
                x = p0[0] + (p1[0] - p0[0]) * t
                y = p0[1] + (p1[1] - p0[1]) * t
                th = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
                return x, y, th
        return 0, 0, 0

    def dist_forward(self, a: float, b: float) -> float:
        return (b - a) % self.length


# ── Trajectory ────────────────────────────────────────────────────────────────

class Trajectory:
    """Piecewise constant-acceleration trajectory.
    Defined by initial conditions + list of (time, new_acc) transitions.
    Immutable once created."""

    def __init__(self, t0: float, pos0: float, vel0: float, v_max: float):
        self.t0 = t0
        self.pos0 = pos0
        self.vel0 = vel0
        self.v_max = v_max
        self.segments: List[Tuple[float, float]] = []  # [(t, acc), ...]
        # Each segment: from t_i with acc_i until t_{i+1}

    def add(self, t: float, acc: float):
        self.segments.append((t, acc))

    def _state_at(self, t: float) -> Tuple[float, float]:
        """(pos, vel) at time t."""
        pos, vel = self.pos0, self.vel0
        prev_t = self.t0
        for seg_t, seg_acc in self.segments:
            if t <= seg_t:
                break
            dt = seg_t - prev_t
            if dt > 0:
                pos, vel = self._advance(pos, vel, seg_acc if prev_t == self.t0 else self.segments[self.segments.index((seg_t, seg_acc)) - 1][1] if self.segments.index((seg_t, seg_acc)) > 0 else 0, dt)
            prev_t = seg_t
        # This is getting complex. Let me simplify.
        # Actually, let me just store segments differently.
        pass

    def pos_at(self, t: float) -> float:
        """Position at time t."""
        return self._eval(t)[0]

    def vel_at(self, t: float) -> float:
        """Velocity at time t."""
        return self._eval(t)[1]

    def _eval(self, t: float) -> Tuple[float, float]:
        """Evaluate (pos, vel) at time t."""
        pos = self.pos0
        vel = self.vel0
        cur_t = self.t0
        cur_acc = self.segments[0][1] if self.segments else 0.0

        for i, (seg_t, seg_acc) in enumerate(self.segments):
            # Current segment runs from cur_t with cur_acc until seg_t
            if i == 0:
                # First segment: from t0 to seg_t with... wait, segments define
                # transitions. Let me redefine:
                # segments[i] = (t_i, acc_i): at time t_i, acceleration becomes acc_i
                # So from t0 to segments[0].t: acc = segments[0].acc?
                # No, from t0 the acc is segments[0].acc.
                # segments[0] = (t0, initial_acc)
                # segments[1] = (t1, new_acc) — at t1, acc changes
                pass

            dt = min(t, seg_t) - cur_t if i > 0 else 0
            # This is getting messy. Let me use a simpler structure.
            pass

        return pos, vel


# OK, the Trajectory class above is overengineered.
# Let me use a simpler approach: just store the phase transitions directly.

# ── Vehicle states ────────────────────────────────────────────────────────────

IDLE    = 'IDLE'
ACCEL   = 'ACCEL'
CRUISE  = 'CRUISE'
DECEL   = 'DECEL'
STOP    = 'STOP'
DWELL   = 'DWELL'


# ── Phase: one segment of constant acceleration ──────────────────────────────

class Phase:
    """One phase of constant acceleration."""
    __slots__ = ('t_start', 'pos_start', 'vel_start', 'acc', 't_end')
    def __init__(self, t_start, pos_start, vel_start, acc, t_end):
        self.t_start = t_start
        self.pos_start = pos_start
        self.vel_start = vel_start
        self.acc = acc
        self.t_end = t_end  # when this phase ends

    def pos_at(self, t):
        dt = min(t, self.t_end) - self.t_start
        if dt <= 0:
            return self.pos_start
        return self.pos_start + self.vel_start * dt + 0.5 * self.acc * dt**2

    def vel_at(self, t):
        dt = min(t, self.t_end) - self.t_start
        if dt <= 0:
            return self.vel_start
        return self.vel_start + self.acc * dt

    def pos_end(self):
        return self.pos_at(self.t_end)

    def vel_end(self):
        return self.vel_at(self.t_end)


def build_trajectory(t0, pos0, vel0, a_max, d_max, v_max, dist,
                     leader_phases=None, gap0=0.0, min_gap=0.0):
    """Build trajectory that covers `dist` and ends at vel=0.

    If leader_phases: for each leader phase, compute B's acceleration
    analytically to maintain gap >= min_gap.
    """
    if dist <= 0:
        return []

    if not leader_phases:
        return _build_free_trajectory(t0, pos0, vel0, a_max, d_max, v_max, dist)

    return _build_following_trajectory(t0, pos0, vel0, a_max, d_max, v_max, dist,
                                       leader_phases, gap0, min_gap)


def _build_following_trajectory(t0, pos0, vel0, a_max, d_max, v_max, dist,
                                 leader_phases, gap0, min_gap):
    """Build trajectory following leader, phase by phase."""
    phases = []
    t = t0
    pos = pos0
    vel = vel0
    remaining = dist

    # Track gap: gap = gap0 + leader_travel - follower_travel
    leader_travel = 0.0
    follower_travel = 0.0

    for lp in leader_phases:
        if remaining <= 1.0:
            break

        l_acc = lp.acc
        l_vel = lp.vel_at(t)
        phase_end_t = lp.t_end
        dt = phase_end_t - t
        if dt <= 0.001:
            continue

        # Current gap
        gap = gap0 + leader_travel - follower_travel
        free_gap = max(0, gap - min_gap)

        # Determine follower acc for this leader phase
        dv = vel - l_vel

        if dv > 0.1 and free_gap > 0:
            # Case 3: B faster than A → use formula
            acc_b = l_acc - (dv * dv) / (2 * free_gap)
            acc_b = max(acc_b, -d_max)
        elif dv < -0.1:
            # Case 1: B slower than A → accelerate
            # Limit: don't exceed v_safe
            v_safe = math.sqrt(max(0, l_vel**2 + 2 * d_max * free_gap))
            v_safe = min(v_max, v_safe)
            if vel < v_safe - 1:
                acc_b = a_max
                # But don't overshoot v_safe during this phase
                t_to_vsafe = (v_safe - vel) / a_max if a_max > 0 else dt
                if t_to_vsafe < dt:
                    # Split: accel to v_safe, then match
                    d1 = vel * t_to_vsafe + 0.5 * a_max * t_to_vsafe**2
                    if d1 < remaining:
                        phases.append(Phase(t, pos, vel, a_max, t + t_to_vsafe))
                        pos += d1
                        vel = v_safe
                        t += t_to_vsafe
                        follower_travel += d1
                        remaining -= d1
                        # Leader travel during this sub-phase
                        leader_travel += l_vel * t_to_vsafe + 0.5 * l_acc * t_to_vsafe**2
                        l_vel = lp.vel_at(t)
                        dt = phase_end_t - t
                        if dt <= 0.001:
                            continue
                        # Now at v_safe, match leader
                        dv = vel - l_vel
                        gap = gap0 + leader_travel - follower_travel
                        free_gap = max(0, gap - min_gap)
                        acc_b = l_acc  # mirror
                    else:
                        acc_b = a_max
                else:
                    acc_b = a_max
            else:
                # At or near v_safe → mirror leader
                acc_b = l_acc
        else:
            # Case 2: same speed → mirror
            acc_b = l_acc

        # Clamp
        acc_b = max(-d_max, min(a_max, acc_b))

        # Check v_max cap
        if vel >= v_max and acc_b > 0:
            acc_b = 0

        # Check: must be able to stop at marker
        brake_d = vel * vel / (2 * d_max) if vel > 0 else 0
        if remaining <= brake_d * 1.05 and acc_b > 0:
            acc_b = 0

        # Compute follower distance in this phase
        d_follower = vel * dt + 0.5 * acc_b * dt**2
        new_vel = vel + acc_b * dt

        # Clamp negative velocity
        if new_vel < 0:
            # Stops mid-phase
            t_stop = vel / abs(acc_b) if acc_b < 0 else dt
            dt = t_stop
            d_follower = vel * dt + 0.5 * acc_b * dt**2
            new_vel = 0

        # Don't exceed remaining distance
        if d_follower > remaining and remaining > 0:
            # Adjust: stop at marker boundary
            d_follower = remaining
            # Solve: vel*dt + 0.5*acc_b*dt² = remaining
            if abs(acc_b) > 0.01:
                disc = vel*vel + 2*acc_b*remaining
                if disc >= 0:
                    dt = (-vel + math.sqrt(disc)) / acc_b
                else:
                    dt = remaining / max(vel, 0.1)
            else:
                dt = remaining / max(vel, 0.1)
            new_vel = vel + acc_b * dt

        if dt > 0.001:
            phases.append(Phase(t, pos, vel, acc_b, t + dt))
            # Leader travel in this duration
            leader_travel += l_vel * dt + 0.5 * l_acc * dt**2

        pos += d_follower
        vel = max(0, new_vel)
        t += dt
        follower_travel += d_follower
        remaining -= d_follower

    # After all leader phases: leader stopped.
    # Follower needs to cover remaining distance and stop.
    if remaining > 1.0 and vel > 0.1:
        decel_phases = _build_free_trajectory(t, pos, vel, a_max, d_max, v_max, remaining)
        phases.extend(decel_phases)
    elif remaining > 1.0:
        # Stopped but distance left — accelerate then decel
        decel_phases = _build_free_trajectory(t, pos, vel, a_max, d_max, v_max, remaining)
        phases.extend(decel_phases)

    return phases


def _build_free_trajectory(t0, pos0, vel0, a_max, d_max, v_max, dist):
    """Simple accel→cruise→decel for given distance, no leader."""
    phases = []
    target_v = v_max

    if vel0 < target_v:
        t_accel = (target_v - vel0) / a_max
        d_accel = vel0 * t_accel + 0.5 * a_max * t_accel**2
        if d_accel > dist:
            coeff = 1/(2*a_max) + 1/(2*d_max)
            v_peak_sq = (dist + vel0**2/(2*a_max)) / coeff
            if v_peak_sq < vel0**2:
                target_v = vel0
                d_accel = 0
                t_accel = 0
            else:
                target_v = math.sqrt(v_peak_sq)
                t_accel = (target_v - vel0) / a_max
                d_accel = vel0 * t_accel + 0.5 * a_max * t_accel**2
    else:
        t_accel = 0
        d_accel = 0

    t_decel = target_v / d_max if target_v > 0 else 0
    d_decel = target_v * t_decel - 0.5 * d_max * t_decel**2 if t_decel > 0 else 0

    d_cruise = max(0, dist - d_accel - d_decel)
    t_cruise = d_cruise / target_v if target_v > 0 else 0

    t, pos = t0, pos0
    if t_accel > 0.001:
        phases.append(Phase(t, pos, vel0, a_max, t + t_accel))
        pos += d_accel; t += t_accel
    if t_cruise > 0.001:
        phases.append(Phase(t, pos, target_v, 0, t + t_cruise))
        pos += d_cruise; t += t_cruise
    if t_decel > 0.001:
        phases.append(Phase(t, pos, target_v, -d_max, t + t_decel))

    return phases


# ── Vehicle ───────────────────────────────────────────────────────────────────

class Vehicle:
    def __init__(self, vid: int, track: Track, pos: float, color=(200,200,200)):
        self.id = vid
        self.track = track
        self.color = color

        self.pos: float = pos
        self.vel: float = 0.0
        self.v_max: float = 3600.0
        self.a_max: float = 500.0
        self.d_max: float = 500.0
        self.length: float = 750.0
        self.h_min: float = 1150.0

        self.state: str = IDLE
        self.token: int = 0

        # Confirmed trajectory: list of Phases
        self.phases: List[Phase] = []
        self.x_marker_dist: float = 0.0  # distance from current pos to marker

        # Leader
        self.leader: Optional[Vehicle] = None

        # Destination
        self.dest_pos: Optional[float] = None
        self.dest_reached: bool = False

        # Render
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        self.gap_to_leader: float = float('inf')

    def pos_at(self, t: float) -> float:
        """Position at time t from confirmed trajectory."""
        for phase in self.phases:
            if t <= phase.t_end:
                return phase.pos_at(t)
        # Past all phases: stationary at last position
        if self.phases:
            return self.phases[-1].pos_end()
        return self.pos

    def vel_at(self, t: float) -> float:
        for phase in self.phases:
            if t <= phase.t_end:
                return phase.vel_at(t)
        return 0.0

    def marker_remaining(self, t: float) -> float:
        """Distance from current position at time t to the marker."""
        cur_pos = self.pos_at(t)
        # marker absolute pos = pos (at trajectory start) + x_marker_dist
        marker_abs = self.pos + self.x_marker_dist
        return marker_abs - cur_pos

    def acc_at(self, t: float) -> float:
        for phase in self.phases:
            if t <= phase.t_end:
                return phase.acc
        return 0.0

    def trajectory_end_time(self) -> float:
        if self.phases:
            return self.phases[-1].t_end
        return 0.0

    def decision_point_time(self) -> float:
        """Time when the decel phase starts (= decision point).
        This is where we decide to extend or brake."""
        if len(self.phases) >= 2:
            # Last phase is decel — decision is at its start
            return self.phases[-1].t_start
        if self.phases:
            return self.phases[-1].t_end
        return 0.0

    def update_render(self, t: float):
        p = self.pos_at(t)
        self.x, self.y, self.theta = self.track.xy_at(p)


# ── Events ────────────────────────────────────────────────────────────────────

class Event:
    __slots__ = ('t', 'kind', 'vid', 'token')
    def __init__(self, t, kind, vid, token):
        self.t = t; self.kind = kind; self.vid = vid; self.token = token
    def __lt__(self, other):
        return self.t < other.t


# ── DES Engine ────────────────────────────────────────────────────────────────

class CircularDESv6:
    def __init__(self, track: Track, zcu_interval: int = 10):
        self.track = track
        self.vehicles: Dict[int, Vehicle] = {}
        self.heap: List[Event] = []
        self.event_count: int = 0

        self.zcu_positions: List[float] = []
        for i in range(0, track.n, zcu_interval):
            self.zcu_positions.append(track.node_cum_dist[i])
        self.zcu_positions.sort()

    def add_vehicle(self, v: Vehicle):
        self.vehicles[v.id] = v

    def _post(self, t: float, kind: str, v: Vehicle):
        heapq.heappush(self.heap, Event(t, kind, v.id, v.token))

    def _invalidate(self, v: Vehicle):
        v.token += 1

    # ── Trajectory computation ────────────────────────────────────────────

    def _plan_trajectory(self, v: Vehicle, t: float):
        """Compute full trajectory to x_marker and schedule decision event."""
        if v.state == DWELL:
            return

        cur_pos = v.pos_at(t)
        cur_vel = v.vel_at(t)
        v.pos = cur_pos
        v.vel = cur_vel

        # Compute marker as distance from current position
        dist = self._compute_marker_dist(v, t)
        v.x_marker_dist = dist

        if dist <= 1.0:
            v.phases = []
            v.vel = 0.0
            v.state = STOP
            return

        # Build trajectory with leader constraint
        leader = v.leader
        if leader is not None and leader.phases:
            _, gap = self._find_ahead(v, t)
            phases = build_trajectory(t, cur_pos, cur_vel,
                                      v.a_max, v.d_max, v.v_max, dist,
                                      leader_phases=leader.phases,
                                      gap0=gap,
                                      min_gap=v.h_min + leader.length)
        else:
            phases = build_trajectory(t, cur_pos, cur_vel,
                                      v.a_max, v.d_max, v.v_max, dist)
        v.phases = phases

        if not phases:
            v.vel = 0.0
            v.state = STOP
            self._wake_follower(v, t)
            return

        v.state = ACCEL if phases[0].acc > 0 else (CRUISE if phases[0].acc == 0 else DECEL)

        # Schedule DECISION_POINT: start of decel phase (or end if no decel)
        # Decision point = where we decide to extend marker or commit to braking
        if len(phases) >= 2 and phases[-1].acc < 0:
            # Decel phase exists — decision at its start
            decision_t = phases[-1].t_start
        else:
            # No decel — decision at end of last phase
            decision_t = phases[-1].t_end

        self._post(decision_t, 'DECISION_POINT', v)

    def _safe_target_v(self, follower: Vehicle, leader: Vehicle,
                       t: float, dist: float, max_v: float) -> float:
        """Compute max safe velocity for follower given leader's trajectory.

        Ensures gap >= h_min at all times by checking:
        1. At each phase transition of the leader, what's the gap?
        2. Compute the max follower velocity that maintains h_min.
        """
        f_pos = follower.pos
        safe_v = max_v

        # Sample leader positions at key times and compute safe speed
        for phase in leader.phases:
            for sample_t in [phase.t_start, phase.t_end]:
                if sample_t <= t:
                    continue
                l_pos = leader.pos_at(sample_t)
                gap_at_t = l_pos - f_pos  # approximate (follower also moves)
                # Rough: if follower goes at v, it reaches f_pos + v*(sample_t - t)
                # gap = l_pos - (f_pos + v*dt) >= h_min + leader.length
                dt = sample_t - t
                if dt > 0:
                    v_limit = (l_pos - f_pos - follower.h_min - leader.length) / dt
                    if v_limit > 0:
                        safe_v = min(safe_v, v_limit)

        # Also use the formula: acc_B = acc_A - Δv²/(2*free_gap)
        # to limit acceleration
        l_vel = leader.vel_at(t)
        l_pos = leader.pos_at(t)
        gap = l_pos - f_pos
        free_gap = gap - follower.h_min - leader.length
        if free_gap > 0 and follower.vel > l_vel:
            dv = follower.vel - l_vel
            # Max velocity to not violate: v_safe = sqrt(l_vel² + 2*d_max*free_gap)
            v_safe = math.sqrt(max(0, l_vel**2 + 2 * follower.d_max * free_gap))
            safe_v = min(safe_v, v_safe)

        return max(0, safe_v)

    def _compute_marker_dist(self, v: Vehicle, t: float) -> float:
        """Compute marker as DISTANCE from current position.
        = min(dist_to_next_zcu, gap + leader.marker_dist - leader.length - h_min, dest_dist)"""
        cur_pos = v.pos_at(t)

        # Distance to next ZCU
        next_zcu = self._next_zcu(cur_pos)
        dist = next_zcu - cur_pos

        # Distance limited by leader's confirmed trajectory
        ahead, gap = self._find_ahead(v, t)
        if ahead is not None:
            # leader's marker extends gap + leader.x_marker_dist from me
            # safe limit = gap + leader.x_marker_dist - leader.length - h_min
            ahead_limit = gap + ahead.marker_remaining(t) - ahead.length - v.h_min
            if ahead_limit < dist:
                dist = ahead_limit
                v.leader = ahead
            else:
                v.leader = None
        else:
            v.leader = None

        # Destination
        if v.dest_pos is not None and not v.dest_reached:
            dest_dist = self.track.dist_forward(
                cur_pos % self.track.length,
                v.dest_pos % self.track.length)
            if dest_dist < dist:
                dist = dest_dist

        return dist

    def _next_zcu(self, pos: float) -> float:
        wrapped = pos % self.track.length
        for zp in self.zcu_positions:
            if zp > wrapped + 0.1:
                return pos + (zp - wrapped)
        return pos + (self.track.length - wrapped + self.zcu_positions[0])

    def _find_ahead(self, v: Vehicle, t: float) -> Tuple[Optional[Vehicle], float]:
        best = None
        best_gap = float('inf')
        v_pos = v.pos_at(t)
        for other in self.vehicles.values():
            if other.id == v.id or other.state == DWELL:
                continue
            o_pos = other.pos_at(t)
            gap = self.track.dist_forward(
                v_pos % self.track.length,
                o_pos % self.track.length)
            if 0 < gap < best_gap:
                best = other
                best_gap = gap
        return best, best_gap

    def _wake_follower(self, v: Vehicle, t: float):
        """Wake follower when v's trajectory changed.
        - STOP follower: replan (may start moving)
        - Moving follower: replan to extend marker based on v's new x_marker"""
        for other in self.vehicles.values():
            if other.leader is v:
                self._invalidate(other)
                self._plan_trajectory(other, t)
                break

    # ── Event handlers ────────────────────────────────────────────────────

    def step(self, t_now: float):
        while self.heap and self.heap[0].t <= t_now:
            ev = heapq.heappop(self.heap)
            v = self.vehicles.get(ev.vid)
            if v is None or ev.token != v.token:
                continue
            self.event_count += 1

            if ev.kind == 'DECISION_POINT':
                # Try to extend marker
                old_dist = v.x_marker_dist
                # From current marker position, look for next marker
                cur_pos = v.pos_at(ev.t)
                marker_abs = cur_pos + old_dist  # absolute pos of current marker
                new_dist = self._compute_marker_dist_from(v, ev.t, marker_abs)

                if new_dist > old_dist + 1.0:
                    # Extension: replan from current state
                    v.x_marker_dist = new_dist
                    self._plan_trajectory(v, ev.t)
                    self._wake_follower(v, ev.t)
                else:
                    # Can't extend — commit to decel (already in trajectory)
                    # Schedule STOPPED at trajectory end
                    end_t = v.trajectory_end_time()
                    self._post(end_t, 'STOPPED', v)

            elif ev.kind == 'STOPPED':
                v.pos = v.pos_at(ev.t)
                v.vel = 0.0
                v.phases = []

                if v.dest_pos is not None:
                    dest_dist = self.track.dist_forward(
                        v.pos % self.track.length,
                        v.dest_pos % self.track.length)
                    if dest_dist < v.length:
                        v.state = DWELL
                        v.dest_reached = True
                        self._wake_follower(v, ev.t)
                        continue

                v.state = STOP
                v.x_marker_dist = 0.0
                self._wake_follower(v, ev.t)

            elif ev.kind == 'DEPART':
                v.dest_pos = None
                v.dest_reached = False
                v.state = IDLE
                self._plan_trajectory(v, ev.t)
                self._wake_follower(v, ev.t)

        for v in self.vehicles.values():
            v.update_render(t_now)
            _, gap = self._find_ahead(v, t_now)
            v.gap_to_leader = gap

    def _compute_marker_dist_from(self, v: Vehicle, t: float, from_pos: float) -> float:
        """Compute marker distance starting from from_pos (for extension)."""
        cur_pos = v.pos_at(t)
        next_zcu = self._next_zcu(from_pos)
        dist = next_zcu - cur_pos  # distance from current pos to next ZCU after from_pos

        ahead, gap = self._find_ahead(v, t)
        if ahead is not None:
            ahead_limit = gap + ahead.marker_remaining(t) - ahead.length - v.h_min
            if ahead_limit < dist:
                dist = ahead_limit
                v.leader = ahead
            else:
                v.leader = None
        else:
            v.leader = None

        if v.dest_pos is not None and not v.dest_reached:
            dest_dist = self.track.dist_forward(
                cur_pos % self.track.length,
                v.dest_pos % self.track.length)
            if dest_dist < dist:
                dist = dest_dist

        return dist

    # ── Start / API ──────────────────────────────────────────────────────

    def start_all(self):
        # Assign leaders
        for v in self.vehicles.values():
            ahead, _ = self._find_ahead(v, 0.0)
            v.leader = ahead

        # Confirm front-to-back using _start_vehicle (uses assigned leader)
        ordered = sorted(self.vehicles.values(),
                         key=lambda v: v.pos % self.track.length, reverse=True)

        # First: cycle break — leader's trajectory not confirmed yet.
        # Use leader's current pos as conservative limit (pretend leader is stationary).
        first = ordered[0]
        self._start_vehicle_cycle_break(first, 0.0)

        # Rest: leader.x_marker is confirmed
        for v in ordered[1:]:
            self._start_vehicle(v, 0.0, use_leader=True)

    def _start_vehicle_cycle_break(self, v: Vehicle, t: float):
        """For cycle-break vehicle: use leader's current pos (not x_marker_dist)."""
        next_zcu = self._next_zcu(v.pos)
        dist = next_zcu - v.pos

        if v.leader is not None:
            gap = self.track.dist_forward(
                v.pos % self.track.length,
                v.leader.pos % self.track.length)
            ahead_limit = gap - v.leader.length - v.h_min
            if ahead_limit < dist:
                dist = ahead_limit

        if v.dest_pos is not None:
            dest_dist = self.track.dist_forward(
                v.pos % self.track.length, v.dest_pos % self.track.length)
            if dest_dist < dist:
                dist = dest_dist

        v.x_marker_dist = dist

        if dist <= 1.0:
            v.phases = []
            v.vel = 0.0
            v.state = STOP
            return

        phases = build_trajectory(t, v.pos, 0.0,
                                  v.a_max, v.d_max, v.v_max, dist)
        v.phases = phases
        v.state = ACCEL if phases and phases[0].acc > 0 else IDLE

        if phases:
            if len(phases) >= 2 and phases[-1].acc < 0:
                decision_t = phases[-1].t_start
            else:
                decision_t = phases[-1].t_end
            self._post(decision_t, 'DECISION_POINT', v)

    def _start_vehicle(self, v: Vehicle, t: float, use_leader: bool):
        """Initial trajectory using assigned leader."""
        next_zcu = self._next_zcu(v.pos)
        dist = next_zcu - v.pos

        if use_leader and v.leader is not None:
            gap = self.track.dist_forward(
                v.pos % self.track.length,
                v.leader.pos % self.track.length)
            ahead_limit = gap + v.leader.x_marker_dist - v.leader.length - v.h_min
            if ahead_limit < dist:
                dist = ahead_limit

        if v.dest_pos is not None:
            dest_dist = self.track.dist_forward(
                v.pos % self.track.length, v.dest_pos % self.track.length)
            if dest_dist < dist:
                dist = dest_dist

        v.x_marker_dist = dist

        if dist <= 1.0:
            v.phases = []
            v.vel = 0.0
            v.state = STOP
            return

        if use_leader and v.leader is not None and v.leader.phases:
            gap = self.track.dist_forward(
                v.pos % self.track.length,
                v.leader.pos % self.track.length)
            phases = build_trajectory(t, v.pos, 0.0,
                                      v.a_max, v.d_max, v.v_max, dist,
                                      leader_phases=v.leader.phases,
                                      gap0=gap,
                                      min_gap=v.h_min + v.leader.length)
        else:
            phases = build_trajectory(t, v.pos, 0.0,
                                      v.a_max, v.d_max, v.v_max, dist)
        v.phases = phases
        v.state = ACCEL if phases and phases[0].acc > 0 else IDLE

        # Schedule decision point
        if phases:
            if len(phases) >= 2 and phases[-1].acc < 0:
                decision_t = phases[-1].t_start
            else:
                decision_t = phases[-1].t_end
            self._post(decision_t, 'DECISION_POINT', v)

    def depart(self, vid: int, t: float):
        v = self.vehicles.get(vid)
        if v and v.state == DWELL:
            self._invalidate(v)
            self._post(t, 'DEPART', v)
