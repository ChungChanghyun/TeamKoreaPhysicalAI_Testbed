"""
1D Leader-Follower: clean analytical trajectory computation.

Approach: "plan from the end"
1. Find leader's final stop position (or steady velocity)
2. Compute follower's target: stop at leader_stop - h_min
3. Plan ACCEL → CRUISE → DECEL profile in one shot
4. Verify gap >= h_min at all intermediate points
"""
import math


# ── Leader trajectory ────────────────────────────────────────────────────────

class LeaderPlan:
    def __init__(self, phases):
        """phases: [(duration, v_start, acc), ...]"""
        self.phases = []
        t, x = 0.0, 0.0
        for dur, v0, acc in phases:
            if acc < 0 and v0 > 0:
                dur = min(dur, v0 / abs(acc))
            self.phases.append((t, t + dur, x, v0, acc))
            x += v0 * dur + 0.5 * acc * dur ** 2
            t += dur
        self._x_end = x
        self._v_end = max(0, self.phases[-1][3] + self.phases[-1][4] *
                          (self.phases[-1][1] - self.phases[-1][0])) if self.phases else 0

    def at(self, t):
        for t0, t1, x0, v0, acc in self.phases:
            if t <= t1 + 1e-12:
                dt = max(0, t - t0)
                if acc < 0:
                    t_stop = v0 / abs(acc)
                    if dt > t_stop:
                        return x0 + v0 * t_stop + 0.5 * acc * t_stop ** 2, 0.0
                return x0 + v0 * dt + 0.5 * acc * dt ** 2, max(0, v0 + acc * dt)
        # After plan: coast at final velocity
        return self._x_end + self._v_end * (t - self.phases[-1][1]), self._v_end

    @property
    def t_end(self):
        return self.phases[-1][1] if self.phases else 0

    def stop_position(self):
        """If leader eventually stops, return (x_stop, t_stop). Else None."""
        if self._v_end > 0.1:
            return None  # Leader doesn't stop
        return self._x_end, self.t_end


# ── Profile computation ──────────────────────────────────────────────────────

def plan_accel_cruise_decel(x_start, v_start, x_target, v_target,
                            v_max, a_max, d_max):
    """
    Compute (accel→cruise→decel) or (accel→decel) profile.

    Returns: [(duration, v0, acc), ...] — at most 3 phases.
    """
    dist = x_target - x_start
    if dist <= 0:
        return []

    # Decel distance: from v_peak to v_target
    # Accel distance: from v_start to v_peak
    # Cruise distance: remainder

    # First check: can we reach v_max?
    d_accel_to_vmax = (v_max ** 2 - v_start ** 2) / (2 * a_max)
    d_decel_from_vmax = (v_max ** 2 - v_target ** 2) / (2 * d_max)

    if d_accel_to_vmax + d_decel_from_vmax <= dist:
        # Trapezoidal profile: accel → cruise → decel
        d_cruise = dist - d_accel_to_vmax - d_decel_from_vmax
        result = []
        if d_accel_to_vmax > 0.1:
            result.append(((v_max - v_start) / a_max, v_start, a_max))
        if d_cruise > 0.1:
            result.append((d_cruise / v_max, v_max, 0))
        if d_decel_from_vmax > 0.1:
            result.append(((v_max - v_target) / d_max, v_max, -d_max))
        return result

    # Triangular profile: accel → decel (no cruise)
    # v_peak² = (2*a_max*d_max*dist + d_max*v_start² + a_max*v_target²) / (a_max + d_max)
    numer = 2 * a_max * d_max * dist + d_max * v_start ** 2 + a_max * v_target ** 2
    denom = a_max + d_max
    v_peak_sq = numer / denom

    if v_peak_sq < v_start ** 2:
        # Can't even accelerate — just decel to target
        d_decel = (v_start ** 2 - v_target ** 2) / (2 * d_max)
        if d_decel > dist:
            # Need stronger decel (may not maintain d_max)
            # a = (v_start² - v_target²) / (2 * dist)
            a = (v_start ** 2 - v_target ** 2) / (2 * dist) if dist > 0 else d_max
            dt = (v_start - v_target) / a if a > 0 else 0
            return [(dt, v_start, -a)] if dt > 0 else []
        dt = (v_start - v_target) / d_max
        return [(dt, v_start, -d_max)] if dt > 0.001 else []

    v_peak = math.sqrt(v_peak_sq)
    v_peak = min(v_peak, v_max)

    result = []
    if v_peak > v_start + 0.1:
        result.append(((v_peak - v_start) / a_max, v_start, a_max))
    if v_peak > v_target + 0.1:
        result.append(((v_peak - v_target) / d_max, v_peak, -d_max))
    return result


# ── Follower plan ────────────────────────────────────────────────────────────

def compute_follower_events(gap_0, v_f0, leader, v_max, a_max, d_max, h_min):
    """
    Compute follower trajectory given leader's committed plan.

    Strategy:
    1. Leader stops at x_ls → follower target = gap_0 + x_ls - h_min
    2. Leader cruises at v_l → follower must match v_l at gap = h_min
    3. Plan accel → cruise → decel profile to reach target
    4. Verify gap >= h_min at all intermediate points
    """
    leader_stop = leader.stop_position()

    if leader_stop is not None:
        x_ls, t_ls = leader_stop
        x_target = gap_0 + x_ls - h_min

        # Plan accel→cruise→decel to x_target with v=0
        profile = plan_accel_cruise_decel(0, v_f0, x_target, 0.0,
                                          v_max, a_max, d_max)

        # Verify intermediate gaps. If violated, reduce v_peak.
        min_gap, _ = verify_profile(gap_0, leader, profile, h_min)
        if min_gap < h_min - 1.0:
            # Binary search for v_peak that satisfies gap constraint
            lo_v, hi_v = max(0, v_f0), v_max
            for _ in range(40):
                mid_v = (lo_v + hi_v) / 2
                trial = plan_accel_cruise_decel(0, v_f0, x_target, 0.0,
                                                mid_v, a_max, d_max)
                mg, _ = verify_profile(gap_0, leader, trial, h_min)
                if mg >= h_min - 1.0:
                    lo_v = mid_v
                    profile = trial
                else:
                    hi_v = mid_v
        return profile

    # ── Leader doesn't stop → match speed ────────────────────────────
    _, v_l = leader.at(leader.t_end)
    # Target: reach v_l while maintaining gap >= h_min
    # Profile: accel or decel to v_l, then cruise at v_l

    if abs(v_f0 - v_l) < 1.0:
        # Already matched → cruise
        return [(10.0, v_f0, 0)]

    if v_f0 > v_l:
        # Decelerate to v_l.
        # During decel: gap_change = v_l*dt - (v_f0*dt - 0.5*d_max*dt²)
        #             = (v_l - v_f0)*dt + 0.5*d_max*dt²
        dt_decel = (v_f0 - v_l) / d_max
        return [(dt_decel, v_f0, -d_max), (10.0, v_l, 0)]

    # v_f0 < v_l → accelerate to v_l (gap increases, always safe)
    dt_accel = (v_l - v_f0) / a_max
    return [(dt_accel, v_f0, a_max), (10.0, v_l, 0)]


# ── Verification ─────────────────────────────────────────────────────────────

def follower_at(t, events):
    x, v = 0.0, events[0][1] if events else 0.0
    for dur, v0, acc in events:
        if t <= dur + 1e-12:
            if acc < 0 and v0 > 0:
                t_stop = v0 / abs(acc)
                if t > t_stop:
                    return x + v0 * t_stop + 0.5 * acc * t_stop ** 2, 0.0
            return x + v0 * t + 0.5 * acc * t ** 2, max(0, v0 + acc * t)
        x += v0 * dur + 0.5 * acc * dur ** 2
        v = max(0, v0 + acc * dur)
        t -= dur
    return x + v * t, v


def verify_profile(gap_0, leader, events, h_min, dt=0.005):
    """Quick gap check for profile optimization."""
    t_end = min(sum(d for d, _, _ in events) + 1.0, 100)
    min_gap = float('inf')
    for i in range(int(t_end / dt)):
        t = i * dt
        x_l, _ = leader.at(t)
        x_f, _ = follower_at(t, events)
        gap = gap_0 + x_l - x_f
        if gap < min_gap:
            min_gap = gap
    return min_gap, 0


def verify(gap_0, leader, events, h_min, dt=0.001):
    t_end = sum(d for d, _, _ in events) + 1.0
    t_end = min(t_end, 100)
    min_gap, min_t = float('inf'), 0
    for i in range(int(t_end / dt)):
        t = i * dt
        x_l, _ = leader.at(t)
        x_f, _ = follower_at(t, events)
        gap = gap_0 + x_l - x_f
        if gap < min_gap:
            min_gap, min_t = gap, t
    return min_gap, min_t


# ── Test ─────────────────────────────────────────────────────────────────────

def run_test(name, gap_0, v_f0, leader_phases, v_max=3600, a_max=500, d_max=500, h_min=1150):
    print(f"\n{'='*60}")
    print(f"  {name}")
    print(f"  gap={gap_0}  v_f={v_f0}  h_min={h_min}")
    print(f"{'='*60}")

    leader = LeaderPlan(leader_phases)
    print("  Leader:")
    for t0, t1, x0, v0, acc in leader.phases:
        x1, v1 = leader.at(t1)
        kind = "ACCEL" if acc > 0 else ("DECEL" if acc < 0 else "CRUISE")
        print(f"    [{t0:.2f}->{t1:.2f}] {kind:6s} v:{v0:.0f}->{v1:.0f}  x:{x0:.0f}->{x1:.0f}")

    ls = leader.stop_position()
    if ls:
        print(f"    Leader stops at x={ls[0]:.0f}, t={ls[1]:.2f}")
        print(f"    Follower target: x={gap_0 + ls[0] - h_min:.0f}, v=0")

    events = compute_follower_events(gap_0, v_f0, leader, v_max, a_max, d_max, h_min)
    print("  Follower:")
    t_acc = 0
    for dur, v0, acc in events:
        v1 = max(0, v0 + acc * dur)
        dx = v0 * dur + 0.5 * acc * dur ** 2
        kind = "ACCEL" if acc > 0 else ("DECEL" if acc < 0 else "CRUISE")
        print(f"    [{t_acc:.2f}->{t_acc+dur:.2f}] {kind:6s} v:{v0:.0f}->{v1:.0f}  dx:{dx:.0f}")
        t_acc += dur

    min_gap, min_t = verify(gap_0, leader, events, h_min)
    ok = min_gap >= h_min - 1.0
    print(f"  min_gap={min_gap:.1f} at t={min_t:.3f}  [{'OK' if ok else 'VIOLATION'}]")
    return ok


if __name__ == "__main__":
    results = []

    # 1. Leader cruising, follower from rest (leader doesn't stop → match speed)
    results.append(run_test("Leader cruise 3000, follower from rest",
        gap_0=5000, v_f0=0,
        leader_phases=[(20, 3000, 0)]))

    # 2. Leader cruise → stop, follower from rest
    results.append(run_test("Leader cruise then stop, follower from rest",
        gap_0=5000, v_f0=0,
        leader_phases=[(3, 3000, 0), (6, 3000, -500)]))

    # 3. Follower faster than leader (safe gap)
    results.append(run_test("Follower fast (3000), leader slow (1000), safe gap",
        gap_0=10000, v_f0=3000,
        leader_phases=[(30, 1000, 0)]))

    # 4. Leader stopped, follower approaching
    results.append(run_test("Leader stopped, follower at 3600",
        gap_0=15000, v_f0=3600,
        leader_phases=[(20, 0, 0)]))

    # 5. Leader accel→cruise→decel→stop
    results.append(run_test("Leader accel->cruise->decel->stop",
        gap_0=4000, v_f0=0,
        leader_phases=[(4, 1000, 500), (4, 3000, 0), (6, 3000, -500)]))

    # 6. Tight gap, both moving, leader stops
    results.append(run_test("Tight gap, both at 2000, leader stops",
        gap_0=1500, v_f0=2000,
        leader_phases=[(2, 2000, 0), (4, 2000, -500)]))

    # 7. Leader accelerating away
    results.append(run_test("Leader accelerating away",
        gap_0=2000, v_f0=0,
        leader_phases=[(7.2, 0, 500), (10, 3600, 0)]))

    print(f"\n{'='*60}")
    print(f"  {sum(results)}/{len(results)} PASSED")
    print(f"{'='*60}")
