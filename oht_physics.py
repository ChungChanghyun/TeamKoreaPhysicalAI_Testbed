"""
oht_physics.py — Precise OHT kinematic model with S-Curve motion profiles.

DES-oriented: computes exact times for state transitions, not per-frame updates.
Each OHT plans its full trajectory upon entering a segment, considering:
  - Acceleration / deceleration limits
  - Jerk limits (S-curve profile)
  - Segment speed limits
  - Next segment speed (pre-deceleration)
  - Leader vehicle distance and speed (safe following)

States: IDLE, ACCEL, CRUISE, DECEL, STOP
"""

from __future__ import annotations
import math
from typing import Optional, Tuple, List
from dataclasses import dataclass, field


# ── S-Curve Motion Profile ────────────────────────────────────────────────────

@dataclass
class Phase:
    """One phase of motion: constant jerk for duration dt."""
    dt: float       # duration (s)
    j: float        # jerk (mm/s³) during this phase
    a0: float       # acceleration at start (mm/s²)
    v0: float       # velocity at start (mm/s)
    s0: float       # position at start (mm)

    @property
    def a_end(self) -> float:
        return self.a0 + self.j * self.dt

    @property
    def v_end(self) -> float:
        return self.v0 + self.a0 * self.dt + 0.5 * self.j * self.dt ** 2

    @property
    def s_end(self) -> float:
        return (self.s0 + self.v0 * self.dt
                + 0.5 * self.a0 * self.dt ** 2
                + (1.0 / 6.0) * self.j * self.dt ** 3)

    def pos_at(self, t: float) -> float:
        """Position at time t within this phase (0 <= t <= dt)."""
        t = min(t, self.dt)
        return (self.s0 + self.v0 * t
                + 0.5 * self.a0 * t ** 2
                + (1.0 / 6.0) * self.j * t ** 3)

    def vel_at(self, t: float) -> float:
        """Velocity at time t within this phase."""
        t = min(t, self.dt)
        return max(0.0, self.v0 + self.a0 * t + 0.5 * self.j * t ** 2)

    def accel_at(self, t: float) -> float:
        """Acceleration at time t within this phase."""
        t = min(t, self.dt)
        return self.a0 + self.j * t


@dataclass
class MotionProfile:
    """Complete S-curve motion profile as a sequence of phases."""
    phases: List[Phase] = field(default_factory=list)

    @property
    def total_time(self) -> float:
        return sum(p.dt for p in self.phases)

    @property
    def total_dist(self) -> float:
        return self.phases[-1].s_end if self.phases else 0.0

    def state_at(self, t: float) -> Tuple[float, float, float]:
        """Returns (position, velocity, acceleration) at time t."""
        elapsed = 0.0
        for phase in self.phases:
            if t <= elapsed + phase.dt:
                dt_in = t - elapsed
                return phase.pos_at(dt_in), phase.vel_at(dt_in), phase.accel_at(dt_in)
            elapsed += phase.dt
        # Past end
        last = self.phases[-1] if self.phases else None
        if last:
            return last.s_end, max(0.0, last.v_end), 0.0
        return 0.0, 0.0, 0.0


def plan_scurve_accel(v0: float, v_target: float, a_max: float,
                      j_max: float) -> List[Phase]:
    """Plan S-curve acceleration from v0 to v_target.
    Returns list of phases (up to 3: jerk-up, const-accel, jerk-down).
    """
    if abs(v_target - v0) < 0.1:
        return []

    sign = 1.0 if v_target > v0 else -1.0
    dv = abs(v_target - v0)
    a = a_max
    j = j_max

    # Time to ramp acceleration up/down
    t_jerk = a / j  # time to reach a_max from 0

    # Velocity gained during jerk-up + jerk-down (two parabolic phases)
    dv_jerk = a * t_jerk  # = a²/j

    if dv_jerk >= dv:
        # Can't reach full acceleration — triangular profile
        t_jerk = math.sqrt(dv / j)
        phases = []
        # Phase 1: jerk up
        phases.append(Phase(dt=t_jerk, j=sign * j, a0=0.0, v0=v0, s0=0.0))
        p1 = phases[0]
        # Phase 2: jerk down (symmetric)
        phases.append(Phase(dt=t_jerk, j=-sign * j,
                            a0=p1.a_end, v0=p1.v_end, s0=p1.s_end))
        return phases

    # Trapezoidal acceleration profile
    t_const = (dv - dv_jerk) / a  # time at constant acceleration

    phases = []
    # Phase 1: jerk up (0 → a_max)
    phases.append(Phase(dt=t_jerk, j=sign * j, a0=0.0, v0=v0, s0=0.0))
    p1 = phases[0]
    # Phase 2: constant acceleration
    phases.append(Phase(dt=t_const, j=0.0,
                        a0=p1.a_end, v0=p1.v_end, s0=p1.s_end))
    p2 = phases[1]
    # Phase 3: jerk down (a_max → 0)
    phases.append(Phase(dt=t_jerk, j=-sign * j,
                        a0=p2.a_end, v0=p2.v_end, s0=p2.s_end))
    return phases


def plan_stop_profile(v0: float, a_max: float, j_max: float) -> List[Phase]:
    """Plan S-curve deceleration from v0 to 0."""
    return plan_scurve_accel(v0, 0.0, a_max, j_max)


def decel_distance(v: float, a_max: float, j_max: float) -> float:
    """Compute distance needed to decelerate from v to 0 using S-curve."""
    if v <= 0:
        return 0.0
    phases = plan_stop_profile(v, a_max, j_max)
    if not phases:
        return 0.0
    return phases[-1].s_end - phases[0].s0


def plan_segment_profile(v_entry: float, v_cruise: float, v_exit: float,
                         seg_length: float, a_max: float, j_max: float
                         ) -> MotionProfile:
    """Plan full S-curve profile for traversing a segment.

    Phases: [accel to v_cruise] [cruise] [decel to v_exit]
    If segment is too short for full accel+decel, reduces peak speed.
    """
    profile = MotionProfile()
    v_cruise = min(v_cruise, 100000)  # sanity cap

    # Try full profile: accel → cruise → decel
    accel_phases = plan_scurve_accel(v_entry, v_cruise, a_max, j_max)
    decel_phases = plan_scurve_accel(v_cruise, v_exit, a_max, j_max)

    accel_dist = accel_phases[-1].s_end if accel_phases else 0.0
    decel_dist = abs(decel_phases[-1].s_end - decel_phases[0].s0) if decel_phases else 0.0

    if accel_dist + decel_dist > seg_length + 1.0:
        # Not enough space — binary search for achievable peak speed
        lo, hi = min(v_entry, v_exit), v_cruise
        for _ in range(30):
            mid = (lo + hi) / 2
            ap = plan_scurve_accel(v_entry, mid, a_max, j_max)
            dp = plan_scurve_accel(mid, v_exit, a_max, j_max)
            ad = ap[-1].s_end if ap else 0.0
            dd = abs(dp[-1].s_end - dp[0].s0) if dp else 0.0
            if ad + dd <= seg_length + 0.5:
                lo = mid
            else:
                hi = mid
        v_cruise = lo
        accel_phases = plan_scurve_accel(v_entry, v_cruise, a_max, j_max)
        decel_phases = plan_scurve_accel(v_cruise, v_exit, a_max, j_max)
        accel_dist = accel_phases[-1].s_end if accel_phases else 0.0
        decel_dist = abs(decel_phases[-1].s_end - decel_phases[0].s0) if decel_phases else 0.0

    cruise_dist = max(0.0, seg_length - accel_dist - decel_dist)

    # Build complete profile with absolute positions
    s_offset = 0.0

    for p in accel_phases:
        profile.phases.append(Phase(dt=p.dt, j=p.j, a0=p.a0, v0=p.v0,
                                    s0=p.s0 + s_offset))
    if accel_phases:
        s_offset = accel_phases[-1].s_end

    if cruise_dist > 0.1 and v_cruise > 0.1:
        profile.phases.append(Phase(dt=cruise_dist / v_cruise, j=0.0,
                                    a0=0.0, v0=v_cruise, s0=s_offset))
        s_offset += cruise_dist

    decel_s0 = decel_phases[0].s0 if decel_phases else 0.0
    for p in decel_phases:
        profile.phases.append(Phase(dt=p.dt, j=p.j, a0=p.a0, v0=p.v0,
                                    s0=p.s0 + s_offset - decel_s0))

    if not profile.phases:
        # Fallback: stationary
        profile.phases.append(Phase(dt=0.01, j=0.0, a0=0.0, v0=0.0, s0=0.0))

    return profile


# ── Safe Following Distance ───────────────────────────────────────────────────

def safe_distance(v_follower: float, v_leader: float,
                  a_max: float, j_max: float, h_min: float) -> float:
    """Minimum safe gap: if leader emergency-stops, follower can stop in time.

    D_safe = h_min + d_stop(v_follower) - d_stop(v_leader)
    where d_stop = distance to stop from given speed.
    """
    d_f = decel_distance(v_follower, a_max, j_max)
    d_l = decel_distance(v_leader, a_max, j_max)
    return h_min + max(0.0, d_f - d_l)


def time_to_close_gap(gap: float, v_f: float, v_l: float,
                      d_safe: float) -> float:
    """Time until gap shrinks to d_safe, assuming constant speeds.
    Returns inf if not closing."""
    if v_f <= v_l:
        return float('inf')
    closing_rate = v_f - v_l
    excess = gap - d_safe
    if excess <= 0:
        return 0.0
    return excess / closing_rate


# ── OHT Vehicle ──────────────────────────────────────────────────────────────

IDLE  = 'IDLE'
ACCEL = 'ACCEL'
CRUISE = 'CRUISE'
DECEL = 'DECEL'
STOP  = 'STOP'
EMERGENCY = 'EMERGENCY'


class OHTVehicle:
    """Single OHT with S-curve motion profile and safe following logic."""

    def __init__(self, vid: int, a_max: float = 500.0, d_max: float = 500.0,
                 j_max: float = 1000.0, v_max: float = 3600.0,
                 h_min: float = 950.0, length: float = 750.0):
        self.id = vid
        self.a_max = a_max      # max acceleration (mm/s²)
        self.d_max = d_max      # max deceleration (mm/s²)
        self.j_max = j_max      # max jerk (mm/s³)
        self.v_max = v_max      # max speed (mm/s)
        self.h_min = h_min      # minimum gap (mm)
        self.length = length    # vehicle length (mm)

        # Current state
        self.state = IDLE
        self.pos = 0.0          # position along current segment (mm)
        self.vel = 0.0          # current velocity (mm/s)
        self.accel = 0.0        # current acceleration (mm/s²)

        # Motion profile for current segment
        self.profile: Optional[MotionProfile] = None
        self.profile_start_t: float = 0.0

        # Segment info
        self.seg_length: float = 0.0
        self.seg_max_speed: float = 0.0

        # Path
        self.path: List[str] = []
        self.path_idx: int = 0

        # Leader
        self.leader: Optional[OHTVehicle] = None

        # World position (for rendering)
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0

        # Color
        self.color: Tuple[int, int, int] = (200, 200, 200)

    def plan_free_run(self, t: float, seg_length: float, seg_speed: float,
                      v_exit: float = 0.0):
        """Plan motion profile for free-running (no leader constraint).
        Accelerate to min(v_max, seg_speed), cruise, decel to v_exit."""
        v_cruise = min(self.v_max, seg_speed)
        v_exit = min(v_exit, v_cruise)
        self.profile = plan_segment_profile(
            self.vel, v_cruise, v_exit,
            seg_length, self.a_max, self.j_max)
        self.profile_start_t = t
        self.seg_length = seg_length
        self.seg_max_speed = seg_speed
        self.state = ACCEL if self.vel < v_cruise else CRUISE

    def safe_gap(self, leader_vel: float = 0.0) -> float:
        """Minimum safe gap given current follower speed and leader speed."""
        return safe_distance(self.vel, leader_vel, self.d_max, self.j_max, self.h_min)

    def update_state(self, t: float):
        """Update pos/vel/accel from motion profile at time t."""
        if self.profile is None:
            return
        dt = t - self.profile_start_t
        self.pos, self.vel, self.accel = self.profile.state_at(dt)
        self.vel = max(0.0, self.vel)

        # Update state label
        if self.vel < 0.1 and abs(self.accel) < 0.1:
            self.state = STOP
        elif self.accel > 0.1:
            self.state = ACCEL
        elif self.accel < -0.1:
            self.state = DECEL
        else:
            self.state = CRUISE

    def time_to_segment_end(self, t: float) -> float:
        """Time remaining until reaching end of segment."""
        if self.profile is None:
            return float('inf')
        return max(0.0, self.profile.total_time - (t - self.profile_start_t))

    def arrival_time(self) -> float:
        """Absolute time of arrival at segment end."""
        if self.profile is None:
            return float('inf')
        return self.profile_start_t + self.profile.total_time
