"""
bench_tapg.py — TAPGEnvironment vs TAPGEnvironmentDES 수행 시간 비교.

같은 CBS 플랜을 두 환경에서 실행하고 wall-clock 시간을 측정합니다.

Usage
─────
    python mapf_edu/bench_tapg.py                         # 기본 (77.pkl, 4 agents)
    python mapf_edu/bench_tapg.py 77.pkl 6                # pkl 경로, agent 수 지정
    python mapf_edu/bench_tapg.py c10_map.pkl 4 --repeat 10
"""
from __future__ import annotations
import sys, os, time, argparse, copy, math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'solvers'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'map_gen'))

from pkl_loader       import PklMapGraph
from planner          import PklPlanner
from env_tapg         import TAPGAgent, TAPGEnvironment, DONE
from env_tapg_des     import TAPGEnvironmentDES


# ── 실행 헬퍼 ─────────────────────────────────────────────────────────────────

def _make_agents(raw_paths: list, colors=None) -> list:
    """raw_paths → TAPGAgent 목록 (환경마다 독립 복사본 생성)."""
    default_colors = [(220, 80, 80), (80, 180, 220), (100, 220, 100),
                      (220, 180, 60), (180, 80, 220), (80, 220, 180)]
    agents = []
    for i, rp in enumerate(raw_paths):
        c = (colors[i] if colors and i < len(colors)
             else default_colors[i % len(default_colors)])
        agents.append(TAPGAgent(i + 1, c, list(rp)))
    return agents


def run_timestepped(graph, raw_paths, accel, decel, dt=1/60) -> dict:
    """
    TAPGEnvironment: dt=1/60 고정 간격 (60 FPS 렌더링 시뮬레이션).
    wall-clock 시간과 step 횟수를 반환합니다.
    """
    agents = _make_agents(raw_paths)
    env    = TAPGEnvironment(graph, accel=accel, decel=decel)
    env.setup(agents)

    steps    = 0
    sim_time = 0.0
    t0       = time.perf_counter()

    while not env.all_done():
        sim_time += dt
        env.step(sim_time)
        steps += 1
        if sim_time > 3600:   # 안전 상한 (1시간 sim)
            break

    wall = time.perf_counter() - t0
    return {'wall_s': wall, 'steps': steps, 'sim_end': sim_time,
            'stats': env.tapg_stats()}


def run_des_fixed(graph, raw_paths, accel, decel, dt=0.5) -> dict:
    """
    TAPGEnvironmentDES: 고정 dt (기본 0.5초) 간격으로 step 호출.
    dt 루프는 없지만 step 간격을 고정해 dt의 영향을 측정합니다.
    """
    agents = _make_agents(raw_paths)
    env    = TAPGEnvironmentDES(graph, accel=accel, decel=decel)
    env.setup(agents)

    steps    = 0
    sim_time = 0.0
    t0       = time.perf_counter()

    while not env.all_done():
        sim_time += dt
        env.step(sim_time)
        steps += 1
        if sim_time > 3600:
            break

    wall = time.perf_counter() - t0
    return {'wall_s': wall, 'steps': steps, 'sim_end': sim_time,
            'stats': env.tapg_stats()}


def run_des_event_driven(graph, raw_paths, accel, decel) -> dict:
    """
    TAPGEnvironmentDES: 다음 이벤트 시각으로 직접 점프 (최소 step 수).
    step 횟수 ≈ 이벤트 수.
    """
    agents = _make_agents(raw_paths)
    env    = TAPGEnvironmentDES(graph, accel=accel, decel=decel)
    env.setup(agents)

    steps    = 0
    sim_time = 0.0
    t0       = time.perf_counter()

    while not env.all_done():
        # 큐에 이벤트가 있으면 다음 이벤트 시각으로 점프
        if env._eq:
            sim_time = env._eq[0].time
        else:
            # 이벤트 없이 WAITING이 남아 있는 경우 → 주기적 wakeup 간격만큼 전진
            sim_time += env.PERIODIC_INTERVAL
        env.step(sim_time)
        steps += 1
        if sim_time > 3600:
            break

    wall = time.perf_counter() - t0
    return {'wall_s': wall, 'steps': steps, 'sim_end': sim_time,
            'stats': env.tapg_stats()}


# ── 출력 ──────────────────────────────────────────────────────────────────────

def _fmt(r: dict, label: str):
    s = r['stats']
    return (f"  {label:<30s} | "
            f"wall={r['wall_s']*1000:8.2f} ms | "
            f"steps={r['steps']:6d} | "
            f"sim_end={r['sim_end']:7.2f}s | "
            f"nodes={s['total']:4d}  cross={s['cross_edges']:4d}")


def benchmark(pkl_path: str, n_agents: int, accel: float, decel: float,
              repeat: int = 5):
    sep = '─' * 85

    print(f'\n{sep}')
    print(f'  Benchmark: TAPGEnvironment vs TAPGEnvironmentDES')
    print(f'  pkl={os.path.basename(pkl_path)}  agents={n_agents}  '
          f'accel={accel}  decel={decel}  repeat={repeat}')
    print(sep)

    # ── 그래프 & 플래너 로드 ─────────────────────────────────────────────────
    print('  Loading graph and planner...', end='', flush=True)
    graph   = PklMapGraph(pkl_path)
    planner = PklPlanner(pkl_path)
    print(' done.')

    results = {'ts': [], 'des_fixed': [], 'des_event': []}

    for trial in range(1, repeat + 1):
        # 같은 플랜으로 세 가지 환경 비교
        raw_paths = planner._env.find_solution.__self__ if False else None  # dummy

        # CBS 플래닝 (매 trial 새로운 랜덤 시나리오)
        node_paths = planner.plan_random(n_agents=n_agents, max_time=30.0)
        if node_paths is None:
            print(f'  [Trial {trial}] CBS failed — skip')
            continue

        raw_paths = planner._last_raw_paths   # [(sid, t), ...] per agent

        print(f'\n  Trial {trial}/{repeat}')
        r_ts  = run_timestepped(graph, raw_paths, accel, decel, dt=1/60)
        r_df  = run_des_fixed(graph, raw_paths, accel, decel, dt=0.5)
        r_de  = run_des_event_driven(graph, raw_paths, accel, decel)

        print(_fmt(r_ts, 'TimeStep  (dt=1/60 s)'))
        print(_fmt(r_df, 'DES fixed (dt=0.5  s)'))
        print(_fmt(r_de, 'DES event (next evt)'))

        results['ts'].append(r_ts['wall_s'])
        results['des_fixed'].append(r_df['wall_s'])
        results['des_event'].append(r_de['wall_s'])

    if not results['ts']:
        print('\n  No successful trials.')
        return

    def avg(lst): return sum(lst) / len(lst) * 1000  # → ms

    print(f'\n{sep}')
    print(f'  Average wall-clock time over {len(results["ts"])} trials:')
    print(f'    TimeStep  (dt=1/60 s) : {avg(results["ts"]):8.2f} ms')
    print(f'    DES fixed (dt=0.5  s) : {avg(results["des_fixed"]):8.2f} ms  '
          f'({avg(results["ts"]) / avg(results["des_fixed"]):.1f}x faster)')
    print(f'    DES event (next evt)  : {avg(results["des_event"]):8.2f} ms  '
          f'({avg(results["ts"]) / avg(results["des_event"]):.1f}x faster)')
    print(sep)


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='TAPG environment benchmark')
    parser.add_argument('pkl',     nargs='?',
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                             '..', '77.pkl'),
                        help='Path to .pkl file (default: ../77.pkl)')
    parser.add_argument('agents',  nargs='?', type=int, default=4,
                        help='Number of agents (default: 4)')
    parser.add_argument('--accel', type=float, default=math.inf,
                        help='Acceleration mm/s² (default: inf)')
    parser.add_argument('--decel', type=float, default=math.inf,
                        help='Deceleration mm/s² (default: inf)')
    parser.add_argument('--repeat', type=int, default=5,
                        help='Number of random scenarios to average (default: 5)')
    args = parser.parse_args()

    benchmark(
        pkl_path=os.path.abspath(args.pkl),
        n_agents=args.agents,
        accel=args.accel,
        decel=args.decel,
        repeat=args.repeat,
    )
