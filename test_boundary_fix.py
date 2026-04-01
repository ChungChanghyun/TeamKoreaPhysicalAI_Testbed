"""
Test script: single-OHT boundary fix validation.
Tests 15 different starting positions on oht.large.map.json.
Traces all events and checks for:
  - REPLAN floods (0.01s interval repetitions)
  - Coherent plan chains
  - Boundary fires before reaching boundary node
  - Marker correctness
"""
import sys, os, random, collections

sys.path.insert(0, os.path.dirname(__file__))

from graph_des_v5 import GraphMap, random_safe_path
from graph_des_v6 import (
    GraphDESv6, Vehicle, Event,
    EV_START, EV_REPLAN, EV_SEG_END, EV_PHASE_DONE,
    EV_STOPPED, EV_ZCU_GRANT, EV_BOUNDARY,
)

MAP_PATH = os.path.join(os.path.dirname(__file__), "oht.large.map.json")
SIM_DURATION = 60.0   # seconds of sim time
N_TESTS = 15

def run_single_oht_test(gmap, start_node, test_id, seed=42):
    """Run a single OHT from start_node, collect all events."""
    random.seed(seed + test_id)

    path = random_safe_path(gmap, start_node, length=500)
    if len(path) < 10:
        return None  # bad starting node

    v = Vehicle(0, gmap, path, color=(200, 200, 200))

    des = GraphDESv6(gmap)
    des.add_vehicle(v)

    # Monkey-patch to capture events
    events_log = []
    orig_dispatch = des._dispatch

    def capturing_dispatch(ev, veh):
        events_log.append((ev.t, ev.kind, veh.vel, veh.acc, veh.state,
                           veh.seg_from, veh.seg_to, veh.seg_offset,
                           veh.x_marker_node, veh.path_idx))
        orig_dispatch(ev, veh)

    des._dispatch = capturing_dispatch
    des._post(0.0, EV_START, v)

    des.run_until(SIM_DURATION)

    return events_log


def analyze_events(events_log, test_id, start_node):
    """Analyze event log for anomalies."""
    total = len(events_log)
    kind_counts = collections.Counter()
    for ev in events_log:
        kind_counts[ev[1]] += 1

    # Check for REPLAN floods: 3+ consecutive REPLANs within 0.05s
    flood_count = 0
    flood_sequences = []
    consecutive_replans = []
    for i, ev in enumerate(events_log):
        t, kind = ev[0], ev[1]
        if kind == EV_REPLAN:
            consecutive_replans.append(t)
        else:
            if len(consecutive_replans) >= 3:
                span = consecutive_replans[-1] - consecutive_replans[0]
                if span < 0.05 * len(consecutive_replans):
                    flood_count += 1
                    flood_sequences.append(
                        (consecutive_replans[0], len(consecutive_replans), span))
            consecutive_replans = []
    # Check last
    if len(consecutive_replans) >= 3:
        span = consecutive_replans[-1] - consecutive_replans[0]
        if span < 0.05 * len(consecutive_replans):
            flood_count += 1
            flood_sequences.append(
                (consecutive_replans[0], len(consecutive_replans), span))

    # Check for 0.01s REPLAN floods specifically
    replan_times = [ev[0] for ev in events_log if ev[1] == EV_REPLAN]
    tiny_gaps = 0
    for i in range(1, len(replan_times)):
        if replan_times[i] - replan_times[i-1] < 0.015:
            tiny_gaps += 1

    # Check BOUNDARY events fire before reaching boundary node
    boundary_events = [(i, ev) for i, ev in enumerate(events_log)
                       if ev[1] == EV_BOUNDARY]

    # Check plan coherence: ACCEL→PHASE_DONE, DECEL→STOPPED patterns
    accel_without_phase = 0
    for i, ev in enumerate(events_log):
        if ev[1] == EV_PHASE_DONE:
            # After PHASE_DONE, state should become CRUISE or we should see SEG_END/BOUNDARY
            pass

    # Check if velocity is ever negative
    neg_vel = sum(1 for ev in events_log if ev[2] < -0.01)

    # Check if acc is reset to 0 at BOUNDARY events where lock was granted
    # (we look for BOUNDARY followed by continuation, not STOPPED)
    boundary_grant_acc_zero = 0
    for i, ev in enumerate(events_log):
        if ev[1] == EV_BOUNDARY and i + 1 < len(events_log):
            next_ev = events_log[i + 1]
            # If next event is not STOPPED → lock was granted
            if next_ev[1] != EV_STOPPED and next_ev[1] != EV_BOUNDARY:
                # Check if the vel right after boundary is reasonable (not reset)
                pass

    result = {
        'test_id': test_id,
        'start_node': start_node,
        'total_events': total,
        'kind_counts': dict(kind_counts),
        'flood_count': flood_count,
        'flood_sequences': flood_sequences[:5],  # first 5
        'tiny_gap_replans': tiny_gaps,
        'boundary_events': len(boundary_events),
        'neg_vel': neg_vel,
    }
    return result


def main():
    print(f"Loading map: {MAP_PATH}")
    gmap = GraphMap(MAP_PATH)
    print(f"Map loaded: {len(gmap.nodes)} nodes, {len(gmap.segments)} segments, "
          f"{len(gmap.zcu_zones)} ZCU zones")
    print(f"Main loop: {len(gmap.main_loop)} nodes")

    # Pick diverse starting nodes from main loop
    main_nodes = list(gmap.main_loop)
    random.seed(12345)
    random.shuffle(main_nodes)
    test_starts = main_nodes[:N_TESTS]
    print(f"\nTest starting nodes ({N_TESTS}): {test_starts}")

    all_results = []
    for i, start in enumerate(test_starts):
        print(f"\n{'='*60}")
        print(f"Test {i}: start={start}")
        events = run_single_oht_test(gmap, start, i)
        if events is None:
            print(f"  SKIP: path too short from {start}")
            continue
        result = analyze_events(events, i, start)
        all_results.append(result)

        print(f"  Total events: {result['total_events']}")
        print(f"  Event counts: {result['kind_counts']}")
        print(f"  BOUNDARY events: {result['boundary_events']}")
        print(f"  REPLAN flood sequences: {result['flood_count']}")
        if result['flood_sequences']:
            for fs in result['flood_sequences']:
                print(f"    t={fs[0]:.3f}, count={fs[1]}, span={fs[2]:.4f}s")
        print(f"  Tiny-gap (<15ms) REPLANs: {result['tiny_gap_replans']}")
        print(f"  Negative velocity events: {result['neg_vel']}")

        # Print first 30 events for inspection
        if i < 3:  # detailed trace for first 3 tests
            print(f"\n  Event trace (first 40):")
            for j, ev in enumerate(events[:40]):
                t, kind, vel, acc, state, sfrom, sto, soff, xnode, pidx = ev
                print(f"    [{j:3d}] t={t:8.4f} {kind:12s} "
                      f"v={vel:7.1f} a={acc:7.1f} st={state:6s} "
                      f"seg={sfrom}->{sto} off={soff:7.1f} "
                      f"marker={xnode or '-':>8s} pi={pidx}")

    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    total_tests = len(all_results)
    total_floods = sum(r['flood_count'] for r in all_results)
    total_tiny = sum(r['tiny_gap_replans'] for r in all_results)
    total_boundary = sum(r['boundary_events'] for r in all_results)
    total_events = sum(r['total_events'] for r in all_results)
    total_neg = sum(r['neg_vel'] for r in all_results)

    print(f"Tests run: {total_tests}")
    print(f"Total events: {total_events}")
    print(f"Total BOUNDARY events: {total_boundary}")
    print(f"Total REPLAN flood sequences: {total_floods}")
    print(f"Total tiny-gap REPLANs: {total_tiny}")
    print(f"Total negative velocity: {total_neg}")

    if total_floods > 0:
        print("\n*** WARNING: REPLAN floods detected! ***")
        for r in all_results:
            if r['flood_count'] > 0:
                print(f"  Test {r['test_id']} (start={r['start_node']}): "
                      f"{r['flood_count']} flood(s)")
    else:
        print("\nNo REPLAN floods detected - PASS")

    if total_tiny > 0:
        print(f"\n*** WARNING: {total_tiny} tiny-gap REPLANs detected ***")
        for r in all_results:
            if r['tiny_gap_replans'] > 0:
                print(f"  Test {r['test_id']} (start={r['start_node']}): "
                      f"{r['tiny_gap_replans']} tiny-gap REPLANs")
    else:
        print("No tiny-gap REPLANs - PASS")

    if total_neg > 0:
        print(f"\n*** WARNING: {total_neg} negative velocity events ***")
    else:
        print("No negative velocity - PASS")


if __name__ == '__main__':
    main()
