"""
Analytical follower trajectory computation for OHT DES.

Given a leader's committed event sequence (piecewise constant-acceleration),
compute the follower's event sequence that maintains gap >= d_safe
using only closed-form quadratic solutions. No simulation loops,
no numerical methods, no binary search.
"""
from __future__ import annotations
import math
from dataclasses import dataclass

# ── Data structure ───────────────────────────────────────────────────────────

@dataclass
class MotionEvent:
    t: float  # event time
    x: float  # position at t
    v: float  # velocity at t
    a: float  # acceleration from t onward

    def __repr__(self):
        kind = "ACCEL" if self.a > 1e-9 else ("BRAKE" if self.a < -1e-9 else
               ("STOP" if abs(self.v) < 1e-9 else "CRUISE"))
        return f"({self.t:.4f}s  x={self.x:.4f}  v={self.v:.4f}  a={self.a:.4f} {kind})"


# ── 1. Utilities ─────────────────────────────────────────────────────────────

def state_at(ev: MotionEvent, t: float) -> tuple[float, float]:
    """Position and velocity at time t, given constant acceleration from ev."""
    dt = t - ev.t
    if dt < 0:
        dt = 0.0
    v = ev.v + ev.a * dt
    # Clamp: if decelerating, velocity can't go below 0
    if ev.a < 0 and v < 0:
        # Stopped earlier
        t_stop = -ev.v / ev.a  # time from ev.t to stop
        x = ev.x + ev.v * t_stop + 0.5 * ev.a * t_stop ** 2
        return x, 0.0
    x = ev.x + ev.v * dt + 0.5 * ev.a * dt ** 2
    return x, v


def event_at(ev: MotionEvent, t: float) -> MotionEvent:
    """Propagate ev to time t, keeping the same acceleration."""
    x, v = state_at(ev, t)
    return MotionEvent(t, x, v, ev.a)


def gap_at_t(lead: MotionEvent, follow: MotionEvent, t: float) -> float:
    """Gap at time t (lead_x - follow_x). Does NOT subtract d_safe."""
    xl, _ = state_at(lead, t)
    xf, _ = state_at(follow, t)
    return xl - xf


def solve_quadratic(a: float, b: float, c: float) -> list[float]:
    """Real positive roots of ax² + bx + c = 0, ascending order."""
    if abs(a) < 1e-15:
        # Linear
        if abs(b) < 1e-15:
            return []
        r = -c / b
        return [r] if r > 1e-12 else []
    disc = b * b - 4 * a * c
    if disc < 0:
        return []
    sq = math.sqrt(disc)
    r1 = (-b - sq) / (2 * a)
    r2 = (-b + sq) / (2 * a)
    roots = []
    for r in sorted([r1, r2]):
        if r > 1e-12:
            roots.append(r)
    return roots


# ── 2. Core analysis ─────────────────────────────────────────────────────────

def _gap_coefficients(lead: MotionEvent, follow: MotionEvent, t_start: float):
    """
    Express gap(t_start + dt) = G0 + G1*dt + 0.5*G2*dt²

    where gap = x_lead - x_follow (raw distance, d_safe not subtracted).

    Returns (G0, G1, G2).
    """
    xl, vl = state_at(lead, t_start)
    xf, vf = state_at(follow, t_start)
    G0 = xl - xf
    G1 = vl - vf
    # lead accel might have caused a stop before t_start;
    # state_at handles clamping, but we need the actual a for the interval.
    # If the vehicle stopped, its effective a is 0.
    al = lead.a if (lead.v + lead.a * (t_start - lead.t)) > -1e-9 else 0.0
    af = follow.a if (follow.v + follow.a * (t_start - follow.t)) > -1e-9 else 0.0
    G2 = al - af
    return G0, G1, G2


def gap_minimum_in_interval(
    lead: MotionEvent,
    follow: MotionEvent,
    t_start: float,
    t_end: float,
) -> tuple[float, float]:
    """
    Minimum gap and the time it occurs in [t_start, t_end].
    gap(dt) = G0 + G1*dt + 0.5*G2*dt²   (dt measured from t_start)

    Minimum of a quadratic: dt* = -G1/G2  (if G2 > 0).
    Compare gap at dt*, t_start, t_end.
    """
    G0, G1, G2 = _gap_coefficients(lead, follow, t_start)
    dur = t_end - t_start

    def gap_val(dt):
        return G0 + G1 * dt + 0.5 * G2 * dt * dt

    best_gap = gap_val(0)
    best_t = t_start

    end_gap = gap_val(dur)
    if end_gap < best_gap:
        best_gap = end_gap
        best_t = t_end

    # Interior extremum
    if abs(G2) > 1e-15:
        dt_ext = -G1 / G2
        if 0 < dt_ext < dur:
            ext_gap = gap_val(dt_ext)
            if ext_gap < best_gap:
                best_gap = ext_gap
                best_t = t_start + dt_ext

    return best_gap, best_t


def time_gap_reaches(
    lead: MotionEvent,
    follow: MotionEvent,
    t_start: float,
    t_end: float,
    d_target: float,
) -> float | None:
    """
    First time in [t_start, t_end] where gap drops to d_target.
    gap(dt) = G0 + G1*dt + 0.5*G2*dt² = d_target
    → 0.5*G2*dt² + G1*dt + (G0 - d_target) = 0
    """
    G0, G1, G2 = _gap_coefficients(lead, follow, t_start)
    dur = t_end - t_start
    roots = solve_quadratic(0.5 * G2, G1, G0 - d_target)
    for r in roots:
        if r <= dur + 1e-12:
            return t_start + r
    return None


def braking_start_time(
    lead: MotionEvent,
    follow: MotionEvent,
    t_start: float,
    t_end: float,
    d_safe: float,
    a_brake: float,   # negative
) -> float | None:
    """
    Find t* in [t_start, t_end] such that if follower switches to a_brake at t*,
    the subsequent gap minimum equals exactly d_safe.

    After switching at t*:
      - follower accel becomes a_brake
      - velocities converge at dt_match = Δv(t*) / D   where D = a_brake_eff - a_lead
        (D < 0 since follower brakes harder)
        Actually: D = a_lead - a_brake (positive, since a_brake < a_lead typically)
      - at that match point, gap is minimised:
        gap_min = gap(t*) - Δv(t*)² / (2·D)

    Setting gap_min = d_safe and expressing gap(t*), Δv(t*) as functions of
    dt = t* - t_start, we get a quadratic in dt.

    Derivation
    ----------
    Let dt = t* - t_start.
    At t_start: gap₀, vl₀, vf₀, al, af (all known constants).

    gap(t*) = gap₀ + (vl₀ - vf₀)·dt + ½·(al - af)·dt²
    Δv(t*)  = vf(t*) - vl(t*) = (vf₀ - vl₀) + (af - al)·dt

    Let Δv₀ = vf₀ - vl₀,  Δa = af - al,  so Δv(t*) = Δv₀ + Δa·dt

    D = al - a_brake   (relative decel of follower vs lead after switch)
      (If follower brakes at a_brake and lead has a_l, the closing rate
       decelerates at rate |a_brake| - a_l if a_l ≤ 0, etc.)
      Actually: after the switch, follower acc = a_brake, lead acc = al.
      Relative velocity Δv_rel = v_follow - v_lead decreases at rate (a_brake - al).
      Time to match: dt_match = Δv(t*) / (al - a_brake)   [al - a_brake > 0 usually]
      Gap consumed:  Δv(t*)² / (2·(al - a_brake))

    D = al - a_brake

    gap_min = gap(t*) - Δv(t*)² / (2·D) = d_safe

    Substituting:
      gap₀ + (-Δv₀)·dt + ½·(-Δa)·dt²  -  (Δv₀ + Δa·dt)² / (2D)  =  d_safe

    Note: vl₀ - vf₀ = -Δv₀,  al - af = -Δa

    Expand (Δv₀ + Δa·dt)² = Δv₀² + 2·Δv₀·Δa·dt + Δa²·dt²

    Collect powers of dt:

    dt² coefficient:  ½·(-Δa) - Δa²/(2D)  =  -Δa/2 · (1 + Δa/D)  =  -Δa·(D + Δa)/(2D)
    dt¹ coefficient:  -Δv₀ - Δv₀·Δa/D  =  -Δv₀·(D + Δa)/D
    dt⁰ coefficient:  gap₀ - d_safe - Δv₀²/(2D)

    Quadratic:  A·dt² + B·dt + C = 0
      A = -Δa·(D + Δa) / (2D)
      B = -Δv₀·(D + Δa) / D
      C = gap₀ - d_safe - Δv₀² / (2D)

    Solve for dt, pick smallest positive root in [0, t_end - t_start].
    """
    xl, vl = state_at(lead, t_start)
    xf, vf = state_at(follow, t_start)
    gap0 = xl - xf

    al = lead.a if (lead.v + lead.a * (t_start - lead.t)) > -1e-9 else 0.0
    af = follow.a if (follow.v + follow.a * (t_start - follow.t)) > -1e-9 else 0.0

    dv0 = vf - vl       # Δv₀ = v_follow - v_lead
    da = af - al         # Δa = a_follow - a_lead
    D = al - a_brake     # relative decel after switch

    if D < 1e-12:
        # a_brake ≈ a_lead or worse. Braking won't converge velocities.
        # Fallback: find when gap = d_safe directly (linear/quadratic gap equation).
        # gap(dt) = gap0 + (-dv0)*dt + 0.5*(-da)*dt²  [current motions]
        # Solve gap(dt) = d_safe, brake immediately at or before that time.
        G0 = gap0 - d_safe
        G1 = -dv0
        G2 = -da
        roots = solve_quadratic(0.5 * G2, G1, G0)
        dur = t_end - t_start
        for r in roots:
            if -1e-9 <= r <= dur + 1e-9:
                # Brake a bit before the gap reaches d_safe
                r = max(0.0, r * 0.8)  # safety margin
                return t_start + r
        return None

    # Coefficients
    K = D + da           # = al - a_brake + af - al = af - a_brake
    A = -da * K / (2 * D)
    B = -dv0 * K / D
    C = gap0 - d_safe - dv0 * dv0 / (2 * D)

    dur = t_end - t_start

    # Constraint: braking must complete within the interval.
    # dt_match(dt) = (Δv₀ + Δa·dt) / D
    # dt + dt_match ≤ dur  →  dt ≤ (dur·D - Δv₀) / (D + Δa)
    dt_max = dur
    if abs(D + da) > 1e-12:
        dt_constrained = (dur * D - dv0) / (D + da)
        if dt_constrained > 0:
            dt_max = min(dt_max, dt_constrained)

    roots = solve_quadratic(A, B, C)

    for r in roots:
        if -1e-9 <= r <= dt_max + 1e-9:
            r = max(0.0, min(r, dt_max))
            vf_star = vf + af * r
            vl_star = vl + al * r
            if vf_star > vl_star + 1e-9:
                return t_start + r

    # If unconstrained root exists but exceeds dt_max, use dt_max
    # (brake earlier so braking completes at interval boundary)
    for r in roots:
        if r > dt_max and r <= dur + 1e-9:
            r = max(0.0, dt_max)
            vf_star = vf + af * r
            vl_star = vl + al * r
            if vf_star > vl_star + 1e-9:
                return t_start + r

    return None


# ── 3. Main function ─────────────────────────────────────────────────────────

def compute_follower_events(
    lead_events: list[MotionEvent],
    follower_init: MotionEvent,
    d_safe: float,
    a_brake: float,     # negative, e.g. -2.0
    a_accel: float,     # positive, e.g. +2.0
    v_max: float,
) -> list[MotionEvent]:
    """
    Compute follower event sequence that maintains gap >= d_safe.

    Iterates over lead event intervals. In each interval both vehicles
    have constant acceleration, so the gap is a quadratic in time.
    All critical times are found via the quadratic formula.
    """
    result: list[MotionEvent] = [follower_init]
    f = follower_init  # current follower state (evolving)

    for i in range(len(lead_events) - 1):
        le = lead_events[i]
        t_seg_start = le.t
        t_seg_end = lead_events[i + 1].t
        if t_seg_end <= f.t:
            continue  # follower already past this interval

        # Clamp interval start to follower's current time
        t_start = max(t_seg_start, f.t)
        t_end = t_seg_end

        if t_start >= t_end - 1e-12:
            continue

        # Propagate follower to t_start (same acceleration)
        f = event_at(f, t_start)

        # ── Check if follower is braking and can resume ─────────────
        al_seg = le.a if (le.v + le.a * (t_start - le.t)) > -1e-9 else 0.0

        if f.a < -1e-9:
            _, vl = state_at(le, t_start)

            if f.v <= 1e-9:
                # Follower stopped.
                f = MotionEvent(t_start, f.x, 0.0, 0.0)
                if result[-1].t < t_start:
                    result.append(f)
            elif f.v <= vl + 1e-9:
                # Follower at or below lead speed → match leader's acceleration
                f = MotionEvent(t_start, f.x, min(f.v, vl), al_seg)
                result.append(f)

        # ── Check if follower stopped and can accelerate ────────────
        if abs(f.v) < 1e-9 and abs(f.a) < 1e-9:
            gap_now = gap_at_t(le, f, t_start)
            if gap_now > d_safe + 1e-6:
                # Can start accelerating
                f = MotionEvent(t_start, f.x, 0.0, a_accel)
                result.append(f)

        # ── Check if follower can accelerate (currently cruising) ───
        # Only accelerate if leader is NOT decelerating and gap allows it.
        _, vl_now = state_at(le, t_start)
        al_eff = le.a if (le.v + le.a * (t_start - le.t)) > -1e-9 else 0.0

        if (abs(f.a) < 1e-9 and f.v >= 0 and f.v < v_max - 1e-9
                and al_eff >= -1e-9 and f.v <= vl_now + 1e-9):
            # Leader not decelerating and follower not faster → try accel
            f_trial_accel = MotionEvent(f.t, f.x, f.v, a_accel)
            t_brake = braking_start_time(le, f_trial_accel, t_start, t_end,
                                         d_safe, a_brake)
            if t_brake is None:
                # Safe to accelerate entire interval. Cap at v_max.
                dt_to_vmax = (v_max - f.v) / a_accel
                t_vmax = t_start + dt_to_vmax
                if t_vmax < t_end:
                    f = MotionEvent(t_start, f.x, f.v, a_accel)
                    if result[-1].t < t_start or result[-1].a != a_accel:
                        result.append(f)
                    f = event_at(f, t_vmax)
                    f = MotionEvent(t_vmax, f.x, v_max, 0.0)
                    result.append(f)
                    f = event_at(f, t_end)
                    continue
                else:
                    f = MotionEvent(t_start, f.x, f.v, a_accel)
                    if result[-1].t < t_start or result[-1].a != a_accel:
                        result.append(f)
                    f = event_at(f, t_end)
                    continue
            elif t_brake > t_start + 0.01:
                # Accelerate until t_brake, then brake
                f = MotionEvent(t_start, f.x, f.v, a_accel)
                if result[-1].t < t_start or result[-1].a != a_accel:
                    result.append(f)
                f = event_at(f, t_brake)
                f = MotionEvent(t_brake, f.x, f.v, a_brake)
                result.append(f)
                # Check if follower stops during this interval
                if f.v > 0:
                    t_stop = f.t + f.v / abs(a_brake)
                    if t_stop < t_end:
                        f = event_at(f, t_stop)
                        f = MotionEvent(t_stop, f.x, 0.0, 0.0)
                        result.append(f)
                f = event_at(f, t_end)
                continue

        # ── Main safety check: current motion ───────────────────────
        gap_min, t_gap_min = gap_minimum_in_interval(le, f, t_start, t_end)

        if gap_min >= d_safe - 1e-9:
            # Safe for entire interval → advance follower state
            f = event_at(f, t_end)
            continue

        # ── Need to brake. Find t* ──────────────────────────────────
        t_star = braking_start_time(le, f, t_start, t_end, d_safe, a_brake)

        if t_star is not None and t_star > t_start + 1e-12:
            # Coast/accel until t*, then brake
            f = event_at(f, t_star)
            f = MotionEvent(t_star, f.x, f.v, a_brake)
            result.append(f)
        else:
            # Must brake immediately (or already should have)
            if f.a != a_brake:
                f = MotionEvent(t_start, f.x, f.v, a_brake)
                result.append(f)

        # ── Check if follower stops during this interval ────────────
        if f.a < 0 and f.v > 0:
            t_stop_dt = f.v / abs(f.a)
            t_stop = f.t + t_stop_dt
            if t_stop < t_end:
                f = MotionEvent(t_stop, f.x + f.v * t_stop_dt + 0.5 * f.a * t_stop_dt ** 2,
                                0.0, 0.0)
                result.append(f)

        # Advance to interval end
        f = event_at(f, t_end)

    # ── Handle last lead event (coast/stop indefinitely) ────────────
    le_last = lead_events[-1]
    t_last = le_last.t
    if f.t < t_last:
        f = event_at(f, t_last)

    # If follower is still faster than lead, brake to match
    _, vl_last = state_at(le_last, f.t)
    if f.v > vl_last + 1e-9 and f.a >= 0:
        # Compute braking start for the "infinite" last segment
        t_horizon = f.t + 200  # large enough
        t_star = braking_start_time(le_last, f, f.t, t_horizon, d_safe, a_brake)
        if t_star is not None:
            f = event_at(f, t_star)
            f = MotionEvent(t_star, f.x, f.v, a_brake)
            result.append(f)
        else:
            f = MotionEvent(f.t, f.x, f.v, a_brake)
            result.append(f)

        # Compute when velocities match
        if f.a < 0:
            al = le_last.a
            D = al - a_brake
            if D > 1e-12:
                dv = f.v - vl_last
                dt_match = dv / D
                if dt_match > 0:
                    t_match = f.t + dt_match
                    x_match, v_match = state_at(f, t_match)
                    f = MotionEvent(t_match, x_match, v_match, al)
                    result.append(f)

    return result


# ── Verification ─────────────────────────────────────────────────────────────

def verify(lead_events, follower_events, d_safe, dt=0.01, verbose=True):
    """Numerically verify gap >= d_safe at fine time resolution."""
    t_end = max(lead_events[-1].t, follower_events[-1].t) + 5.0

    def pos_at(events, t):
        # Find the active event
        ev = events[0]
        for e in events:
            if e.t <= t + 1e-12:
                ev = e
            else:
                break
        x, _ = state_at(ev, t)
        return x

    def vel_at(events, t):
        ev = events[0]
        for e in events:
            if e.t <= t + 1e-12:
                ev = e
            else:
                break
        _, v = state_at(ev, t)
        return v

    min_gap = float('inf')
    min_gap_t = 0.0
    violations = 0
    n = int(t_end / dt)

    for i in range(n + 1):
        t = i * dt
        xl = pos_at(lead_events, t)
        xf = pos_at(follower_events, t)
        gap = xl - xf
        if gap < min_gap:
            min_gap = gap
            min_gap_t = t
        if gap < d_safe - 0.01:
            violations += 1

    if verbose:
        print(f"  verify: min_gap={min_gap:.4f} at t={min_gap_t:.2f}, "
              f"d_safe={d_safe}, violations={violations}  "
              f"[{'OK' if violations == 0 else 'FAIL'}]")

    return violations == 0


# ── Test cases ───────────────────────────────────────────────────────────────

def print_events(label, events):
    print(f"  {label}:")
    for e in events:
        print(f"    {e}")


def run_test(name, lead_events, follower_init, d_safe, a_brake, a_accel, v_max):
    print(f"\n{'='*60}")
    print(f"  {name}")
    print(f"{'='*60}")
    print_events("Lead", lead_events)

    result = compute_follower_events(
        lead_events, follower_init, d_safe, a_brake, a_accel, v_max)

    print_events("Follower", result)
    ok = verify(lead_events, result, d_safe)
    return ok


if __name__ == "__main__":
    results = []

    # Case 1: Lead cruises → hard brake → stop
    results.append(run_test(
        "Case 1: Lead cruise → brake → stop",
        lead_events=[
            MotionEvent(t=0,  x=20, v=5.0, a=0),
            MotionEvent(t=10, x=70, v=5.0, a=-2.5),
            MotionEvent(t=12, x=75, v=0.0, a=0),
        ],
        follower_init=MotionEvent(t=0, x=0, v=5.0, a=0),
        d_safe=5.0, a_brake=-2.5, a_accel=2.0, v_max=8.0,
    ))

    # Case 2: Lead cruise → slow down → cruise at lower speed
    results.append(run_test(
        "Case 2: Lead cruise → decel → slow cruise",
        lead_events=[
            MotionEvent(t=0,  x=15, v=6.0, a=0),
            MotionEvent(t=5,  x=45, v=6.0, a=-2.0),
            MotionEvent(t=7,  x=55, v=2.0, a=0),
            MotionEvent(t=20, x=81, v=2.0, a=0),
        ],
        follower_init=MotionEvent(t=0, x=0, v=6.0, a=0),
        d_safe=5.0, a_brake=-2.5, a_accel=2.0, v_max=8.0,
    ))

    # Case 3: Follower initially faster than lead
    results.append(run_test(
        "Case 3: Follower faster than lead",
        lead_events=[
            MotionEvent(t=0, x=12, v=3.0, a=0),
            MotionEvent(t=20, x=72, v=3.0, a=0),
        ],
        follower_init=MotionEvent(t=0, x=0, v=6.0, a=0),
        d_safe=5.0, a_brake=-2.5, a_accel=2.0, v_max=8.0,
    ))

    # Case 4: Follower already braking, lead decelerates more
    results.append(run_test(
        "Case 4: Follower braking, lead decelerates further",
        lead_events=[
            MotionEvent(t=0,  x=20, v=5.0, a=0),
            MotionEvent(t=5,  x=45, v=5.0, a=-1.0),
            MotionEvent(t=8,  x=58, v=2.0, a=-2.0),
            MotionEvent(t=9,  x=59, v=0.0, a=0),
        ],
        follower_init=MotionEvent(t=0, x=0, v=7.0, a=0),
        d_safe=5.0, a_brake=-3.0, a_accel=2.0, v_max=8.0,
    ))

    # Case 5: Lead stopped, follower approaching fast
    results.append(run_test(
        "Case 5: Lead stopped, follower approaching",
        lead_events=[
            MotionEvent(t=0, x=50, v=0, a=0),
            MotionEvent(t=100, x=50, v=0, a=0),
        ],
        follower_init=MotionEvent(t=0, x=0, v=10.0, a=0),
        d_safe=5.0, a_brake=-3.0, a_accel=2.0, v_max=10.0,
    ))

    # Case 6: Lead accelerating away, follower from rest
    results.append(run_test(
        "Case 6: Lead accelerating away",
        lead_events=[
            MotionEvent(t=0,  x=10, v=0, a=2.0),
            MotionEvent(t=5,  x=35, v=10.0, a=0),
            MotionEvent(t=15, x=135, v=10.0, a=0),
        ],
        follower_init=MotionEvent(t=0, x=0, v=0, a=2.0),
        d_safe=3.0, a_brake=-3.0, a_accel=2.0, v_max=12.0,
    ))

    # Case 7: Tight gap, same speed, lead brakes
    results.append(run_test(
        "Case 7: Tight gap, same speed, lead brakes hard",
        lead_events=[
            MotionEvent(t=0,  x=8, v=5.0, a=0),
            MotionEvent(t=2,  x=18, v=5.0, a=-5.0),
            MotionEvent(t=3,  x=20.5, v=0.0, a=0),
        ],
        follower_init=MotionEvent(t=0, x=0, v=5.0, a=0),
        d_safe=3.0, a_brake=-5.0, a_accel=2.0, v_max=8.0,
    ))

    # Case 8: Follower from rest, lead cruising (should accelerate to follow)
    results.append(run_test(
        "Case 8: Follower from rest, lead cruising far ahead",
        lead_events=[
            MotionEvent(t=0,  x=50, v=5.0, a=0),
            MotionEvent(t=30, x=200, v=5.0, a=0),
        ],
        follower_init=MotionEvent(t=0, x=0, v=0, a=0),
        d_safe=5.0, a_brake=-2.5, a_accel=2.0, v_max=8.0,
    ))

    print(f"\n{'='*60}")
    print(f"  {sum(results)}/{len(results)} PASSED")
    print(f"{'='*60}")
