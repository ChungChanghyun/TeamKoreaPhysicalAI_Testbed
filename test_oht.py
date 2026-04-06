"""
test_oht.py — OHT DES 단독 테스트.

기능:
  1. gap violation 감지 (h_min 이하 접근)
  2. 동일 노드 동시 점유 감지
  3. 다중 seed 스트레스 테스트
  4. 특정 노드 쌍(예: 10022/10023) 모니터링

Usage:
  python test_oht.py                    # 기본 테스트 (20 seed)
  python test_oht.py --seeds 100        # 100 seed 테스트
  python test_oht.py --agents 8         # OHT 8대
  python test_oht.py --duration 600     # 10분 시뮬레이션
  python test_oht.py --watch 10022 10023  # 특정 노드 모니터링
"""
from __future__ import annotations
import sys, os, math, random, argparse, collections

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from env_oht_des import (OHTMap, OHTAgent, OHTEnvironmentDES,
                          IDLE, MOVING, FOLLOWING, BLOCKED, DONE)

JSON_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'Maps', 'KaistTB.map_latest.json')


# ── Gap 계산 헬퍼 ─────────────────────────────────────────────────────────────

def _agent_pos_on_seg(agent, t):
    """에이전트의 세그먼트 내 위치. seg=None이면 None."""
    if agent.seg is None:
        return None
    return agent.pos(t)


def _path_gap(a, b, oht_map, t):
    """
    두 에이전트 간의 경로 기반 거리 계산.
    같은 세그먼트 → pos 차이.
    인접 세그먼트 → 세그먼트 잔여거리 + 상대 위치.
    그 외 → 유클리드 거리 (경로 관계 없음).
    """
    # Case 1: 같은 세그먼트
    if a.seg is not None and a.seg is b.seg:
        return abs(a.pos(t) - b.pos(t))

    # Case 2: 둘 다 노드 위
    if a.seg is None and b.seg is None:
        if a.cur_node == b.cur_node:
            return 0.0
        # 인접 세그먼트 체크
        seg_ab = oht_map.segments.get((a.cur_node, b.cur_node))
        if seg_ab:
            return seg_ab.length
        seg_ba = oht_map.segments.get((b.cur_node, a.cur_node))
        if seg_ba:
            return seg_ba.length

    # Case 3: 하나는 세그먼트, 하나는 노드
    if a.seg is not None and b.seg is None:
        # b가 a의 세그먼트 끝 노드에 있으면
        if b.cur_node == a.seg.to_id:
            return a.seg.length - a.pos(t)
        # b가 a의 세그먼트 시작 노드에 있으면
        if b.cur_node == a.seg.from_id:
            return a.pos(t)
    if b.seg is not None and a.seg is None:
        if a.cur_node == b.seg.to_id:
            return b.seg.length - b.pos(t)
        if a.cur_node == b.seg.from_id:
            return b.pos(t)

    # Case 4: 인접 세그먼트 (a의 다음이 b의 세그먼트, 또는 그 반대)
    if a.seg is not None and b.seg is not None:
        # a → (a.to_id) → b.seg
        if a.seg.to_id == b.seg.from_id:
            return (a.seg.length - a.pos(t)) + b.pos(t)
        if b.seg.to_id == a.seg.from_id:
            return (b.seg.length - b.pos(t)) + a.pos(t)

    # Fallback: 유클리드 거리
    ax, ay, _ = a.xy(t, oht_map.nodes)
    bx, by, _ = b.xy(t, oht_map.nodes)
    return math.hypot(ax - bx, ay - by)


def _pairwise_gap(agents, oht_map, t):
    """
    모든 에이전트 쌍의 경로 기반 거리 계산.
    Returns list of (dist, agent_a, agent_b).
    """
    results = []
    n = len(agents)
    for i in range(n):
        a = agents[i]
        for j in range(i + 1, n):
            b = agents[j]
            dist = _path_gap(a, b, oht_map, t)
            results.append((dist, a, b))
    return results


# ── 위반 감지기 ───────────────────────────────────────────────────────────────

class ViolationDetector:
    """OHT 간 gap violation 및 동일 노드 점유를 실시간 감지."""

    def __init__(self, oht_map, agents, watch_nodes=None):
        self.oht_map = oht_map
        self.agents = agents
        self.h_min = oht_map.h_min
        self.watch_nodes = set(watch_nodes) if watch_nodes else set()

        # 위반 기록
        self.gap_violations = []      # (t, dist, a_id, b_id, type)
        self.node_violations = []     # (t, node_id, [agent_ids])
        self.watch_events = []        # (t, node_id, agent_id, state)

        # 최소 gap 추적
        self.min_gap = float('inf')
        self.min_gap_info = None

    def check(self, t):
        """현재 시점의 위반 검사."""
        # 1) 같은 세그먼트 내 순서 역전 감지 (가장 심각한 위반)
        for a in self.agents:
            if a.seg is None:
                continue
            q = a.seg.queue
            for i in range(len(q) - 1):
                front = q[i]
                back = q[i + 1]
                gap = front.pos(t) - back.pos(t)
                if gap < self.min_gap:
                    self.min_gap = gap
                    self.min_gap_info = (t, front.id, back.id, front.state, back.state)
                if gap < 0:
                    self.gap_violations.append(
                        (t, gap, back.id, front.id, 'OVERTAKE'))
                elif gap < self.h_min - 1.0:
                    self.gap_violations.append(
                        (t, gap, back.id, front.id, 'SAME_SEG'))

        # 2) 동일 노드 점유 체크
        node_agents = collections.defaultdict(list)
        for a in self.agents:
            if a.seg is None and a.state != DONE:
                node_agents[a.cur_node].append(a.id)
        for nid, aids in node_agents.items():
            if len(aids) > 1:
                self.node_violations.append((t, nid, list(aids)))

        # 3) watch node 모니터링
        if self.watch_nodes:
            for a in self.agents:
                if a.seg is None and a.cur_node in self.watch_nodes:
                    self.watch_events.append((t, a.cur_node, a.id, a.state))

    def _classify(self, a, b):
        """gap violation 유형 분류."""
        if a.seg is None and b.seg is None:
            if a.cur_node == b.cur_node:
                return 'BOTH_NODE'
            return 'DIFF_NODE'
        if a.seg is not None and b.seg is not None and a.seg is b.seg:
            return 'SAME_SEG'
        return 'CROSS_SEG'

    def report(self):
        """테스트 결과 요약."""
        lines = []
        lines.append(f'  Min gap: {self.min_gap:.1f}mm')
        if self.min_gap_info:
            t, a, b, sa, sb = self.min_gap_info
            lines.append(f'    at t={t:.2f}s, agents {a}({sa}) & {b}({sb})')

        if self.gap_violations:
            lines.append(f'  Gap violations (< h_min={self.h_min:.0f}mm): {len(self.gap_violations)}')
            # 유형별 집계
            by_type = collections.Counter(v[4] for v in self.gap_violations)
            for vtype, cnt in by_type.most_common():
                lines.append(f'    {vtype}: {cnt}')
            # 최악 5개
            worst = sorted(self.gap_violations, key=lambda v: v[1])[:5]
            for t, dist, a, b, vtype in worst:
                lines.append(f'    t={t:.2f}s dist={dist:.1f}mm agents={a},{b} type={vtype}')
        else:
            lines.append(f'  Gap violations: 0 (PASS)')

        if self.node_violations:
            lines.append(f'  Same-node violations: {len(self.node_violations)}')
            seen = set()
            for t, nid, aids in self.node_violations[:10]:
                key = (nid, tuple(sorted(aids)))
                if key not in seen:
                    seen.add(key)
                    lines.append(f'    t={t:.2f}s node={nid} agents={aids}')
        else:
            lines.append(f'  Same-node violations: 0 (PASS)')

        return '\n'.join(lines)


# ── 테스트 실행 ───────────────────────────────────────────────────────────────

def run_single(seed, n_agents, duration, watch_nodes=None, verbose=False):
    """단일 seed 테스트 실행. 위반 정보 반환."""
    rng = random.Random(seed)

    oht_map = OHTMap(JSON_FILE, area='OHT_A')
    env = OHTEnvironmentDES(oht_map, cross_segment=True)

    # 에이전트 배치
    agents = []
    excluded = set()
    nodes_list = list(oht_map.nodes.keys())
    for aid in range(n_agents):
        rng.shuffle(nodes_list)
        for start in nodes_list:
            if start in excluded:
                continue
            path = oht_map.bfs_path(start)
            if len(path) > 1:
                a = OHTAgent(aid, (180, 80, 200), path,
                             oht_map.vehicle_length * (1000/1108))
                excluded |= oht_map.nearby_nodes(start, oht_map.h_min)
                agents.append(a)
                env.add_agent(a, t_start=0.0)
                break

    detector = ViolationDetector(oht_map, agents, watch_nodes)

    # 시뮬레이션 실행
    sim_time = 0.0
    check_interval = 0.1   # 100ms 간격 체크
    reassign_count = 0

    while sim_time < duration:
        sim_time += check_interval
        env.step(sim_time)

        # DONE 에이전트 재할당
        for a in agents:
            if a.state == DONE:
                path = oht_map.bfs_path(a.cur_node)
                if len(path) > 1:
                    env.reassign(a, path, sim_time)
                    reassign_count += 1

        # 위반 검사
        detector.check(sim_time)

    return {
        'seed': seed,
        'n_agents': len(agents),
        'duration': duration,
        'reassigns': reassign_count,
        'events': env.event_count,
        'detector': detector,
    }


def main():
    parser = argparse.ArgumentParser(description='OHT DES collision test')
    parser.add_argument('--seeds', type=int, default=20,
                        help='Number of random seeds (default: 20)')
    parser.add_argument('--agents', type=int, default=5,
                        help='Number of OHT agents (default: 5)')
    parser.add_argument('--duration', type=float, default=300.0,
                        help='Simulation duration in seconds (default: 300)')
    parser.add_argument('--watch', nargs='*', default=[],
                        help='Node IDs to monitor (e.g. --watch 10022 10023)')
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()

    watch = args.watch if args.watch else None

    print(f'OHT DES Collision Test')
    print(f'  Seeds:    {args.seeds}')
    print(f'  Agents:   {args.agents}')
    print(f'  Duration: {args.duration}s')
    if watch:
        print(f'  Watch:    {watch}')
    print()

    import time
    total_gap_violations = 0
    total_node_violations = 0
    all_min_gaps = []
    violation_seeds = []
    wall_start = time.perf_counter()

    for seed in range(args.seeds):
        result = run_single(seed, args.agents, args.duration, watch, args.verbose)
        det = result['detector']

        n_gap = len(det.gap_violations)
        n_node = len(det.node_violations)
        total_gap_violations += n_gap
        total_node_violations += n_node
        all_min_gaps.append(det.min_gap)

        status = 'PASS' if (n_gap == 0 and n_node == 0) else 'FAIL'
        marker = '' if status == 'PASS' else ' <<<'

        print(f'  Seed {seed:3d}: min_gap={det.min_gap:7.1f}mm  '
              f'gap_viol={n_gap:3d}  node_viol={n_node:3d}  '
              f'events={result["events"]:6d}  reassigns={result["reassigns"]:4d}  '
              f'[{status}]{marker}')

        if status == 'FAIL':
            violation_seeds.append(seed)
            if args.verbose:
                print(det.report())
                print()

        # watch node 이벤트 출력
        if watch and det.watch_events and args.verbose:
            print(f'    Watch events ({len(det.watch_events)}):')
            seen = set()
            for t, nid, aid, state in det.watch_events[:20]:
                key = (round(t, 1), nid, aid)
                if key not in seen:
                    seen.add(key)
                    print(f'      t={t:.1f}s node={nid} agent={aid} state={state}')

    wall_elapsed = time.perf_counter() - wall_start

    print(f'\n{"="*70}')
    print(f'SUMMARY ({args.seeds} seeds, {args.agents} agents, {args.duration}s each)')
    print(f'{"="*70}')
    print(f'  Total wall time:     {wall_elapsed:.2f}s')
    print(f'  Sim time per seed:   {args.duration}s')
    print(f'  Total sim time:      {args.seeds * args.duration:.0f}s')
    print(f'  Speed:               {args.seeds * args.duration / wall_elapsed:.0f}x real-time')
    print()
    print(f'  Min gap (global):    {min(all_min_gaps):.1f}mm')
    print(f'  h_min:               {OHTMap(JSON_FILE).h_min:.0f}mm')
    print(f'  Gap violations:      {total_gap_violations}')
    print(f'  Node violations:     {total_node_violations}')
    print(f'  Failed seeds:        {len(violation_seeds)}/{args.seeds}')
    if violation_seeds:
        print(f'  Failed seed list:    {violation_seeds}')

    # 상세 보기: 실패한 seed 중 첫 번째를 verbose로 재실행
    if violation_seeds and not args.verbose:
        print(f'\n--- Detail for seed {violation_seeds[0]} ---')
        result = run_single(violation_seeds[0], args.agents, args.duration, watch)
        print(result['detector'].report())

    passed = total_gap_violations == 0 and total_node_violations == 0
    print(f'\n  Result: {"ALL PASS" if passed else "VIOLATIONS FOUND"}')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
