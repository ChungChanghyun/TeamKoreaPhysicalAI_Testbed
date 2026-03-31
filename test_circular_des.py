"""
Circular loop DES test environment.

Simple circular track with N OHTs running at high density.
Tests: acceleration, deceleration, max speed, safe gap maintenance.
Loading/unloading stops are implemented via delayed _post_advance —
using the engine's normal flow (node arrival → delayed departure).

Usage: python test_circular_des.py
"""

import sys, os, math, random, time, collections

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'TeamKoreaPhysicalAI_Testbed'))
from env_oht_des import (OHTMap, OHTAgent, OHTEnvironmentDES, OHTNode, OHTSegment,
                          _polyline_length, _time_to_pos,
                          IDLE, MOVING, FOLLOWING, BLOCKED, DONE,
                          SEGMENT_DONE, TRY_ADVANCE)


# ── Build circular map ────────────────────────────────────────────────────────

def make_circular_map(n_nodes=40, radius=20000, seg_speed=3600):
    """Create a circular loop with n_nodes evenly spaced nodes."""
    nodes = {}
    segments = {}
    adj = {}

    for i in range(n_nodes):
        nid = f"C{i}"
        angle = 2 * math.pi * i / n_nodes
        nodes[nid] = OHTNode(nid, radius * math.cos(angle), radius * math.sin(angle))
        adj[nid] = []

    for i in range(n_nodes):
        fn, tn = f"C{i}", f"C{(i + 1) % n_nodes}"
        n1, n2 = nodes[fn], nodes[tn]
        length = math.hypot(n2.x - n1.x, n2.y - n1.y)
        seg = OHTSegment(i + 1, fn, tn, length, seg_speed,
                         [(n1.x, n1.y), (n2.x, n2.y)])
        segments[(fn, tn)] = seg
        adj[fn].append(tn)

    m = object.__new__(OHTMap)
    m.nodes = nodes
    m.segments = segments
    m.adj = adj
    m.vehicle_length = 750
    m.vehicle_width = 500
    m.h_min = m.vehicle_length + 400  # 1150mm (400mm margin for catch-up precision)
    m.accel = 500
    m.decel = 500
    m.max_brake = seg_speed ** 2 / (2.0 * m.decel)
    m.port_nodes = set()
    m.zcu_zones = []
    m._entry_zcu = {}
    m._exit_zcu = {}
    m.zcu_node_ids = set()
    m.bypass_curve_segs = set()
    m.bypass_straight_segs = set()

    track_len = sum(s.length for s in segments.values())
    return m, track_len


# ── Subclass to inject loading/unloading delays ──────────────────────────────

class CircularTestEnv(OHTEnvironmentDES):
    """Extends DES env with probabilistic loading/unloading stops at nodes."""

    def __init__(self, oht_map, stop_prob=0.01, stop_duration=(2.0, 5.0),
                 **kwargs):
        super().__init__(oht_map, **kwargs)
        self.stop_prob = stop_prob          # probability per node arrival
        self.stop_duration = stop_duration  # (min, max) seconds
        self.stops_executed = 0

    def _on_segment_done(self, t, agent):
        """Override: after normal segment completion, maybe delay next advance."""
        seg = agent.seg
        if seg is None:
            return

        # ── Normal segment done processing (copied from parent) ──────────
        if agent in seg.queue:
            seg.queue.remove(agent)
        self._notify_followers(t, agent)

        exit_v = agent.vel(t)
        nxt_idx = agent.path_idx + 1
        if nxt_idx < len(agent.node_path):
            nxt_seg = self.map.segments.get(
                (agent.node_path[agent.path_idx], agent.node_path[nxt_idx]))
            if nxt_seg:
                exit_v = min(exit_v, nxt_seg.max_speed)

        agent.v = exit_v
        agent.a = 0.0
        agent.v_cap = agent.max_speed
        agent.seg = None
        agent.state = IDLE
        self._node_agents.setdefault(seg.to_id, set()).add(agent)

        zcu = self.map._exit_zcu.get((seg.from_id, seg.to_id))
        if zcu is not None and self._zcu_holders[zcu.id] is agent:
            is_straight = (seg.from_id, seg.to_id) in getattr(
                self.map, 'bypass_straight_segs', set())
            if is_straight and seg.queue:
                self._zcu_holders[zcu.id] = seg.queue[0]
            else:
                self._release_zcu(t, zcu)

        self._wake_blocked_for(t, seg)
        if self.cross_segment:
            self._check_incoming_followers(t, seg.from_id)

        # ── Loading/unloading delay ──────────────────────────────────────
        if random.random() < self.stop_prob:
            delay = random.uniform(*self.stop_duration)
            self.stops_executed += 1
            agent.v = 0.0
            agent.state = BLOCKED  # visually show as stopped
            # Re-notify followers with v=0 (first notify was with exit_v)
            self._notify_followers(t, agent)
            self._notify_node_occupied(t, seg.to_id)
            if self.cross_segment:
                self._signal_node_unblocked(t, seg.to_id)  # propagate back
                self._check_incoming_followers(t, seg.to_id)
            self._post_advance(t + delay, agent)
        else:
            self._post_advance(t, agent)


# ── Test runner ───────────────────────────────────────────────────────────────

def run_test(n_vehicles=20, n_nodes=40, sim_duration=120.0,
             stop_prob=0.01, stop_duration=(2.0, 5.0),
             seed=42, verbose=True):
    random.seed(seed)
    oht_map, track_len = make_circular_map(n_nodes=n_nodes)
    seg_length = track_len / n_nodes

    if verbose:
        print(f"Track: {n_nodes} nodes, {track_len:.0f}mm circ, "
              f"seg={seg_length:.0f}mm")
        print(f"Vehicles: {n_vehicles}, h_min={oht_map.h_min}mm, "
              f"accel/decel={oht_map.accel}/{oht_map.decel}")
        spacing = track_len / n_vehicles
        print(f"Spacing: {spacing:.0f}mm "
              f"({'OK' if spacing > oht_map.h_min else 'DENSE'})")
        print(f"Stops: p={stop_prob}/node, dur={stop_duration}")
        print()

    env = CircularTestEnv(oht_map, stop_prob=stop_prob,
                          stop_duration=stop_duration, cross_segment=True)

    # Place vehicles evenly
    node_list = [f"C{i}" for i in range(n_nodes)]
    agents = []
    for i in range(n_vehicles):
        start_idx = int(i * n_nodes / n_vehicles) % n_nodes
        path = []
        for lap in range(500):
            for j in range(n_nodes):
                path.append(node_list[(start_idx + j + lap * n_nodes) % n_nodes])
        agent = OHTAgent(i, (255, 100, 100), path, max_speed=3600)
        agents.append(agent)
        env.add_agent(agent, t_start=0.0)

    # ── Run ───────────────────────────────────────────────────────────────
    dt = 0.05
    n_steps = int(sim_duration / dt)

    gap_violations = 0
    gap_min = float('inf')
    speed_violations = 0
    total_gap_samples = 0
    gap_sum = 0.0

    t_start = time.perf_counter()

    for step in range(n_steps):
        t = (step + 1) * dt
        env.step(t)

        # Gap check
        seg_agents = collections.defaultdict(list)
        for a in agents:
            if a.seg:
                seg_agents[(a.seg.from_id, a.seg.to_id)].append(a)

        for key, ags in seg_agents.items():
            if len(ags) < 2:
                continue
            positions = sorted([(a, a.pos(t)) for a in ags], key=lambda x: -x[1])
            for i in range(len(positions) - 1):
                af, pf = positions[i]
                ab, pb = positions[i + 1]
                gap = pf - pb
                total_gap_samples += 1
                gap_sum += gap
                if gap < oht_map.vehicle_length:
                    gap_violations += 1
                    if gap_violations <= 3 and verbose:
                        print(f"  GAP t={t:.2f}: #{af.id}(v={af.vel(t):.0f},{af.state})"
                              f" - #{ab.id}(v={ab.vel(t):.0f},{ab.state})"
                              f" gap={gap:.0f}mm")
                gap_min = min(gap_min, gap)

        for a in agents:
            if a.seg and a.vel(t) > a.seg.max_speed + 50:
                speed_violations += 1

    wall = time.perf_counter() - t_start
    ratio = sim_duration / wall
    sc = collections.Counter(a.state for a in agents)
    avg_gap = gap_sum / total_gap_samples if total_gap_samples > 0 else 0

    if verbose:
        print()
        print(f"=== {sim_duration:.0f}s in {wall:.2f}s = {ratio:.0f}x realtime, "
              f"{env.event_count} events ===")
        print(f"States: {dict(sc)}")
        print(f"Stops: {env.stops_executed}")
        print(f"Gap violations: {gap_violations}, min gap: {gap_min:.0f}mm, "
              f"avg gap: {avg_gap:.0f}mm")
        print(f"Speed violations: {speed_violations}")
        result = "PASS" if gap_violations == 0 and speed_violations == 0 else "FAIL"
        print(f"Result: {result}")
        print()

    return {
        'gap_violations': gap_violations,
        'gap_min': gap_min,
        'speed_violations': speed_violations,
        'events': env.event_count,
        'wall': wall,
        'ratio': ratio,
        'stops': env.stops_executed,
    }


if __name__ == '__main__':
    print("=" * 60)
    print("Test 1: Normal density, occasional stops")
    print("=" * 60)
    run_test(n_vehicles=20, n_nodes=40, sim_duration=120.0,
             stop_prob=0.01, seed=42)

    print("=" * 60)
    print("Test 2: High density, occasional stops")
    print("=" * 60)
    run_test(n_vehicles=35, n_nodes=40, sim_duration=120.0,
             stop_prob=0.01, seed=42)

    print("=" * 60)
    print("Test 3: High density, frequent stops")
    print("=" * 60)
    run_test(n_vehicles=30, n_nodes=40, sim_duration=120.0,
             stop_prob=0.05, stop_duration=(1.0, 3.0), seed=42)

    print("=" * 60)
    print("Test 4: Max density stress test")
    print("=" * 60)
    run_test(n_vehicles=38, n_nodes=40, sim_duration=60.0,
             stop_prob=0.02, seed=42)
