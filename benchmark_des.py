"""
benchmark_des.py — 시뮬레이션 속도 벤치마크.

1) Headless (렌더링 없음): 순수 DES 이벤트 처리 속도
2) With animation (pygame 60 FPS): 렌더링 포함 속도

실행:
  python benchmark_des.py
"""
from __future__ import annotations
import sys, os, time, random, json, math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from env_oht_des import (OHTMap, OHTAgent, OHTEnvironmentDES,
                          IDLE, DONE)
from env_tapg import (TAPGAgent, TAPGEnvironment,
                      DONE as AGV_DONE)
from pkl_loader import PklMapGraph
from pkl_prioritized_planner import PklPrioritizedPlanner
from env_3ds import FloorGraph
from elevator import ElevatorController

JSON_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'Maps', 'KaistTB.map_latest.json')
AMR_PKL   = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'KaistTB_AMR_A.pkl')
S3D_FLOOR_IDS = ['3DS_F1', '3DS_F2', '3DS_F3']

# ── 3DS 좌표 변환 (vis_combined.py와 동일) ──────────────────────────────────

_3DS_W, _3DS_H = 4620 - 1115, 17411 - 12721
_3DS_SCALE = 3.8
_3DS_CX = 1115 + _3DS_W / 2
_3DS_CY = 12721 + _3DS_H / 2

AREA_OFFSETS = {
    'OHT_A':  (0, 0),
    'AMR_A':  (0, 0),
    '3DS_F1': (-15000, -40000),
    '3DS_F2': (-15000, -25000),
    '3DS_F3': (-15000, -10000),
}

def _transform_3ds(x, y, area):
    dx, dy = AREA_OFFSETS.get(area, (0, 0))
    sx = _3DS_CX + (x - _3DS_CX) * _3DS_SCALE + dx
    sy = _3DS_CY + (y - _3DS_CY) * _3DS_SCALE + dy
    return sx, sy


# ── 시뮬레이션 환경 구성 (pygame 없이) ────────────────────────────────────────

def build_simulation(n_oht=5, n_agv=3, n_s3d=2, seed=42):
    """CombinedSimulator와 동일한 시뮬레이션 환경을 pygame 없이 구성."""
    rng = random.Random(seed)

    # OHT
    oht_map = OHTMap(JSON_FILE, area='OHT_A')
    oht_env = OHTEnvironmentDES(oht_map, cross_segment=True)
    oht_agents = []
    excluded = set()
    nodes_list = list(oht_map.nodes.keys())
    for aid in range(n_oht):
        rng.shuffle(nodes_list)
        for start in nodes_list:
            if start in excluded:
                continue
            path = oht_map.bfs_path(start)
            if len(path) > 1:
                a = OHTAgent(aid, (180, 80, 200), path,
                             oht_map.vehicle_length * (1000/1108))
                excluded |= oht_map.nearby_nodes(start, oht_map.h_min)
                oht_agents.append(a)
                oht_env.add_agent(a, t_start=0.0)
                break

    # AGV
    amr_graph = PklMapGraph(AMR_PKL)
    agv_planner = PklPrioritizedPlanner(amr_graph)
    agv_env = TAPGEnvironment(amr_graph, accel=500.0, decel=500.0)

    port_nodes = list(agv_planner._port_nodes)
    rng.shuffle(port_nodes)
    agv_positions = {}
    for i in range(min(n_agv, len(port_nodes))):
        agv_positions[100 + i] = port_nodes[i]

    agv_goals = agv_planner.assign_random_goals(agv_positions)
    agv_result = agv_planner.plan(agv_positions, agv_goals,
                                   start_times={aid: 0.0 for aid in agv_positions})
    agv_agents = []
    if agv_result and agv_result.paths:
        for aid in sorted(agv_result.paths):
            raw_path = agv_result.paths[aid]
            if len(raw_path) >= 2:
                a = TAPGAgent(aid, (100, 220, 120), raw_path)
                agv_agents.append(a)
        if agv_agents:
            agv_env.setup(agv_agents, t_start=0.0)

    # 3DS (per floor)
    s3d_envs = {}
    s3d_agents_all = []
    s3d_next_id = 200
    for fid in S3D_FLOOR_IDS:
        pkl_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                f'KaistTB_{fid}.pkl')
        if not os.path.exists(pkl_path):
            continue
        graph = PklMapGraph(pkl_path)
        for node in graph.nodes.values():
            node.x, node.y = _transform_3ds(node.x, node.y, fid)

        planner = PklPrioritizedPlanner(graph)
        tapg_env = TAPGEnvironment(graph, accel=500.0, decel=500.0)

        ports = list(planner._port_nodes)
        rng.shuffle(ports)
        positions = {}
        for k in range(min(n_s3d, len(ports))):
            positions[s3d_next_id] = ports[k]
            s3d_next_id += 1

        goals = planner.assign_random_goals(positions)
        result = planner.plan(positions, goals,
                               start_times={aid: 0.0 for aid in positions})
        agents = []
        if result and result.paths:
            for aid in sorted(result.paths):
                raw_path = result.paths[aid]
                if len(raw_path) >= 2:
                    a = TAPGAgent(aid, (80, 180, 255), raw_path)
                    agents.append(a)
                    s3d_agents_all.append(a)
            if agents:
                tapg_env.setup(agents, t_start=0.0)
        s3d_envs[fid] = {'env': tapg_env, 'agents': agents}

    # Elevator
    lift_heap = []
    lift_ctrl = ElevatorController(lift_heap)
    with open(JSON_FILE, 'r', encoding='utf-8') as f:
        jdata = json.load(f)
    area_heights = {'1': 0.0, '2': 3000.0, '3': 6000.0}
    for lift_data in jdata.get('lifts', []):
        lift_ctrl.add_lift_from_map(lift_data, area_heights,
                                    speed=1000.0, xfer_duration=3.0, capacity=1)

    return {
        'oht_env': oht_env, 'oht_agents': oht_agents, 'oht_map': oht_map,
        'agv_env': agv_env, 'agv_agents': agv_agents,
        's3d_envs': s3d_envs, 's3d_agents': s3d_agents_all,
        'lift_heap': lift_heap, 'lift_ctrl': lift_ctrl,
    }


def reassign_done_oht(oht_env, oht_agents, oht_map, sim_time):
    """DONE OHT 에이전트에 새 경로 할당."""
    for a in oht_agents:
        if a.state == DONE:
            path = oht_map.bfs_path(a.cur_node)
            if len(path) > 1:
                oht_env.reassign(a, path, sim_time)


# ── Headless 벤치마크 ────────────────────────────────────────────────────────

def benchmark_headless(sim_duration: float, n_oht=5, n_agv=3, n_s3d=2):
    """
    렌더링 없이 순수 DES 실행.
    이벤트 큐에서 다음 이벤트 시간으로 점프하는 방식.
    """
    print(f'\n{"="*60}')
    print(f' HEADLESS benchmark (no rendering)')
    print(f' sim_duration={sim_duration}s, OHT={n_oht}, AGV={n_agv}, 3DS={n_s3d}')
    print(f'{"="*60}')

    sim = build_simulation(n_oht, n_agv, n_s3d)

    import heapq

    sim_time = 0.0
    event_count = 0
    wall_start = time.perf_counter()

    # 1초 간격으로 step (프레임 루프 없이)
    dt_step = 1.0
    while sim_time < sim_duration:
        sim_time += dt_step

        sim['oht_env'].step(sim_time)
        sim['agv_env'].step(sim_time)
        for fid, fd in sim['s3d_envs'].items():
            fd['env'].step(sim_time)

        # Elevator
        while sim['lift_heap'] and sim['lift_heap'][0].t <= sim_time:
            ev = heapq.heappop(sim['lift_heap'])
            sim['lift_ctrl'].handle_event(ev)

        # OHT reassign
        reassign_done_oht(sim['oht_env'], sim['oht_agents'],
                          sim['oht_map'], sim_time)

        event_count += 1

    wall_elapsed = time.perf_counter() - wall_start

    ratio = sim_duration / wall_elapsed if wall_elapsed > 0 else float('inf')
    print(f'\n  Sim time:  {sim_duration:.1f}s')
    print(f'  Wall time: {wall_elapsed:.4f}s')
    print(f'  Speed:     {ratio:.1f}x real-time')
    print(f'  Steps:     {event_count}')

    # OHT 상태 확인
    oht_done = sum(1 for a in sim['oht_agents'] if a.state == DONE)
    print(f'  OHT agents: {len(sim["oht_agents"])} ({oht_done} DONE)')

    # AGV 상태
    agv_done = sum(1 for a in sim['agv_agents'] if a.state == AGV_DONE)
    print(f'  AGV agents: {len(sim["agv_agents"])} ({agv_done} DONE)')

    # 3DS 상태
    s3d_done = sum(1 for a in sim['s3d_agents'] if a.state == AGV_DONE)
    print(f'  3DS agents: {len(sim["s3d_agents"])} ({s3d_done} DONE)')

    return ratio


def benchmark_headless_event_jump(sim_duration: float, n_oht=5, n_agv=3, n_s3d=2):
    """
    렌더링 없이 순수 DES 이벤트 점프.
    1초 간격이 아니라 가능한 한 크게 점프하면서 실행.
    """
    print(f'\n{"="*60}')
    print(f' HEADLESS EVENT-JUMP benchmark (max time skip)')
    print(f' sim_duration={sim_duration}s, OHT={n_oht}, AGV={n_agv}, 3DS={n_s3d}')
    print(f'{"="*60}')

    sim = build_simulation(n_oht, n_agv, n_s3d)

    import heapq

    sim_time = 0.0
    step_count = 0
    wall_start = time.perf_counter()

    # 10초 간격으로 큰 점프 (이벤트 큐가 알아서 처리)
    dt_step = 10.0
    while sim_time < sim_duration:
        sim_time += dt_step
        sim_time = min(sim_time, sim_duration)

        sim['oht_env'].step(sim_time)
        sim['agv_env'].step(sim_time)
        for fid, fd in sim['s3d_envs'].items():
            fd['env'].step(sim_time)

        while sim['lift_heap'] and sim['lift_heap'][0].t <= sim_time:
            ev = heapq.heappop(sim['lift_heap'])
            sim['lift_ctrl'].handle_event(ev)

        reassign_done_oht(sim['oht_env'], sim['oht_agents'],
                          sim['oht_map'], sim_time)

        step_count += 1

    wall_elapsed = time.perf_counter() - wall_start

    ratio = sim_duration / wall_elapsed if wall_elapsed > 0 else float('inf')
    print(f'\n  Sim time:  {sim_duration:.1f}s')
    print(f'  Wall time: {wall_elapsed:.4f}s')
    print(f'  Speed:     {ratio:.1f}x real-time')
    print(f'  Steps:     {step_count}')

    oht_done = sum(1 for a in sim['oht_agents'] if a.state == DONE)
    agv_done = sum(1 for a in sim['agv_agents'] if a.state == AGV_DONE)
    s3d_done = sum(1 for a in sim['s3d_agents'] if a.state == AGV_DONE)
    print(f'  OHT: {len(sim["oht_agents"])} ({oht_done} DONE)')
    print(f'  AGV: {len(sim["agv_agents"])} ({agv_done} DONE)')
    print(f'  3DS: {len(sim["s3d_agents"])} ({s3d_done} DONE)')

    return ratio


def benchmark_with_pygame(sim_duration: float, n_oht=5, n_agv=3, n_s3d=2):
    """
    pygame 렌더링 포함 (60 FPS).
    실제 CombinedSimulator 실행.
    """
    print(f'\n{"="*60}')
    print(f' PYGAME benchmark (60 FPS rendering)')
    print(f' sim_duration={sim_duration}s, OHT={n_oht}, AGV={n_agv}, 3DS={n_s3d}')
    print(f'{"="*60}')

    try:
        import pygame
    except ImportError:
        print('  pygame not installed — skipping')
        return None

    # vis_combined.py에서 CombinedSimulator를 import
    from vis_combined import CombinedSimulator, OHTMap

    oht_map = OHTMap(JSON_FILE, area='OHT_A')
    amr_graph = PklMapGraph(AMR_PKL)

    sim = CombinedSimulator(oht_map, amr_graph,
                            n_oht=n_oht, n_agv=n_agv, n_s3d=n_s3d)
    sim.running = True
    sim.spd_idx = 7  # 16x speed

    wall_start = time.perf_counter()
    frame_count = 0

    while sim.sim_time < sim_duration:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                return None

        # 16x speed로 실행
        dt_real = sim.clock.tick(60) / 1000.0
        sim.update(dt_real)
        sim.render()
        frame_count += 1

    wall_elapsed = time.perf_counter() - wall_start
    pygame.quit()

    ratio = sim_duration / wall_elapsed if wall_elapsed > 0 else float('inf')
    print(f'\n  Sim time:  {sim_duration:.1f}s')
    print(f'  Wall time: {wall_elapsed:.4f}s')
    print(f'  Speed:     {ratio:.1f}x real-time')
    print(f'  Frames:    {frame_count} ({frame_count/wall_elapsed:.1f} FPS)')

    return ratio


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    SIM_DURATION = 300.0   # 시뮬레이션 5분

    print('Simulation Benchmark')
    print(f'Duration: {SIM_DURATION}s simulated time')

    # 1) Headless (1초 간격)
    r1 = benchmark_headless(SIM_DURATION)

    # 2) Headless (10초 점프)
    r2 = benchmark_headless_event_jump(SIM_DURATION)

    # 3) With pygame
    r3 = benchmark_with_pygame(SIM_DURATION)

    print(f'\n{"="*60}')
    print(f' SUMMARY')
    print(f'{"="*60}')
    print(f'  Headless (1s step):   {r1:.1f}x real-time')
    print(f'  Headless (10s jump):  {r2:.1f}x real-time')
    if r3 is not None:
        print(f'  Pygame (16x, 60FPS):  {r3:.1f}x real-time')
    if r3 and r1:
        print(f'\n  Headless speedup vs pygame: {r1/r3:.1f}x')
