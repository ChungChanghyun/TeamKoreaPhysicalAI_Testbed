"""
Headless test: MCS 작업 할당 + SIPP 경로 계획이 연속적으로 동작하는지 확인.
pygame 없이 MCS 엔진과 SIPP planner만 사용.
"""
import os, sys, random

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pkl_prioritized_planner import PklMapGraph, PklPrioritizedPlanner
from env_tapg import TAPGEnvironment, TAPGAgent, DONE as AGV_DONE
from mcs_unified import (MCSEngine, VehicleJobState, LoadState,
                         post_vehicle_arrived)
import heapq

# ── 설정 ──
SEED = 823019
N_AGV = 10
SIM_DURATION = 600.0  # 초
DT = 0.5  # step 간격

random.seed(SEED)

# ── 맵 로드 ──
pkl_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'KaistTB_AMR_A.pkl')
graph = PklMapGraph(pkl_path)
planner = PklPrioritizedPlanner(graph)
env = TAPGEnvironment(graph, accel=500.0, decel=500.0)

port_nodes = list(planner._port_nodes)
random.shuffle(port_nodes)

# ── AGV 초기화 ──
agents = []
agv_goals = {}
agv_pushed = set()
agv_pending_replan = set()
agv_arrival_idx = {}
agv_arrived_notified = set()
agv_done_notified = set()

for i in range(min(N_AGV, len(port_nodes))):
    aid = 100 + i
    nid = port_nodes[i]
    stop_sid = f'S,{nid},0'
    # find actual stop state
    for sid in graph.stop_states_raw:
        if sid.split(',')[1] == nid:
            stop_sid = sid
            break
    a = TAPGAgent(aid, (200, 200, 200), [(stop_sid, 0.0)])
    agents.append(a)

env.setup(agents, t_start=0.0)
agent_map = {a.id: a for a in agents}

# ── MCS 초기화 ──
mcs_heap = []
mcs_seq = [0]
mcs = MCSEngine(heap=mcs_heap, seq_counter=mcs_seq, dwell_time=3.0, seed=SEED)

mcs.register_system(
    system='AGV',
    port_nodes=[p for p in port_nodes[:N_AGV]],
    on_dispatch=lambda vid, goal, t: dispatch_agv(vid, goal, t),
    is_vehicle_free=lambda vid: agent_map[vid].state == AGV_DONE if vid in agent_map else False,
    get_vehicle_node=lambda vid: get_agv_node(vid),
    get_distance=None,
    port_prod_rate=0.3,
)

for a in agents:
    mcs.register_vehicle(a.id, 'AGV')

# ── 헬퍼 함수 ──
def get_agv_node(vid):
    a = agent_map.get(vid)
    if a is None:
        return None
    idx = a.path_idx
    if idx < len(a.raw_path):
        return a.raw_path[idx][0].split(',')[1]
    elif a.raw_path:
        return a.raw_path[-1][0].split(',')[1]
    return None

def find_stop_state(nid):
    for sid in graph.stop_states_raw:
        if sid.split(',')[1] == nid:
            return sid
    return f'S,{nid},0'

sipp_fail_count = 0
sipp_success_count = 0
exit_count = 0

def dispatch_agv(vid, goal_node, t):
    global sipp_fail_count, sipp_success_count
    a = agent_map.get(vid)
    if a is None:
        return
    cur_node = get_agv_node(vid)
    if cur_node is None:
        return
    if cur_node == goal_node:
        b = mcs.bindings.get(vid)
        if b:
            post_vehicle_arrived(mcs_heap, mcs_seq, vid, b.token, t)
        return

    agv_goals[vid] = goal_node
    agv_pushed.discard(vid)
    agv_pending_replan.add(vid)
    agv_done_notified.discard(vid)
    agv_arrived_notified.discard(vid)
    agv_arrival_idx.pop(vid, None)
    replan_done_agvs(t)

def replan_done_agvs(sim_time):
    global sipp_fail_count, sipp_success_count, exit_count
    done_ids = set(agv_pending_replan)
    if not done_ids:
        return

    env.recompute_earliest_schedule(current_time=sim_time)

    # 0.5) EXIT extension
    mcs_goals_pre = {agv_goals.get(aid) for aid in done_ids if agv_goals.get(aid)}
    if mcs_goals_pre:
        extend_with_exit(done_ids, mcs_goals_pre, sim_time)

    # 1) Build constraints
    all_constraints = []
    for a in agents:
        if a.id in done_ids:
            continue
        idx = a.path_idx
        if idx >= len(a.raw_path):
            idx = len(a.raw_path) - 1
        cur_sid, cur_t = a.raw_path[idx]
        if idx + 1 < len(a.raw_path):
            t_end_cur = max(a.raw_path[idx + 1][1], sim_time)
        else:
            t_end_cur = float('inf')
        all_constraints.append({'agent': a.id, 'loc': cur_sid, 'timestep': (sim_time, t_end_cur)})

        unclaimed_path = a.raw_path[a.claim_idx:]
        if unclaimed_path:
            cs = planner._build_constraints(unclaimed_path, a.id)
            all_constraints.extend(cs)

    # 2) positions
    positions_to_plan = {}
    idle_aids = set()
    for a in agents:
        if a.id in done_ids:
            positions_to_plan[a.id] = a.raw_path[-1][0].split(',')[1]

    # blocker detection
    mcs_goals_set = {agv_goals.get(aid) for aid in done_ids if agv_goals.get(aid)}
    for a in agents:
        if a.id in done_ids:
            continue
        if a.path_idx >= len(a.raw_path):
            nid = a.raw_path[-1][0].split(',')[1]
            if nid in mcs_goals_set:
                positions_to_plan[a.id] = nid
                idle_aids.add(a.id)
                done_ids.add(a.id)

    # 3) goals
    occupied = set()
    for a in agents:
        if a.id not in done_ids and a.raw_path:
            occupied.add(a.raw_path[-1][0].split(',')[1])
    targeted = {g for aid, g in agv_goals.items() if aid not in done_ids}
    used = occupied | targeted | set(positions_to_plan.values())

    new_goals = {}
    for aid, cur in positions_to_plan.items():
        mcs_goal = agv_goals.get(aid)
        if mcs_goal and mcs_goal != cur and aid not in agv_pushed:
            new_goals[aid] = mcs_goal
            used.add(mcs_goal)
            continue
        if aid in idle_aids or aid in agv_pushed:
            agv_goals.pop(aid, None)
            for p in port_nodes:
                if p != cur and p not in used:
                    new_goals[aid] = p
                    used.add(p)
                    break
            else:
                new_goals[aid] = cur
            continue
        for p in port_nodes:
            if p != cur and p not in used:
                new_goals[aid] = p
                used.add(p)
                break
        else:
            for p in port_nodes:
                if p != cur:
                    new_goals[aid] = p
                    break

    # 4) Plan
    result = planner.plan(
        positions_to_plan, new_goals,
        existing_constraints=all_constraints,
        start_times={aid: sim_time for aid in positions_to_plan},
    )

    if result is None or not result.paths:
        sipp_fail_count += 1
        agv_pending_replan.clear()
        return

    sipp_success_count += 1

    # 5) dwell
    dwell = mcs.dwell_time
    for aid, new_path in result.paths.items():
        if aid in idle_aids or aid in agv_pushed:
            continue
        if new_path and len(new_path) >= 2:
            arr_idx = len(new_path) - 1
            last_sid, last_t = new_path[-1]
            new_path.append((last_sid, last_t + dwell))
            agv_arrival_idx[aid] = arr_idx
            agv_arrived_notified.discard(aid)

    # 6) extend
    for aid, goal in new_goals.items():
        agv_goals[aid] = goal
        if aid in idle_aids:
            agv_pushed.add(aid)

    batch = {}
    for aid, new_path in result.paths.items():
        if not new_path or len(new_path) < 2:
            continue
        if aid not in agent_map:
            continue
        batch[aid] = new_path

    if batch:
        env.extend_agents_batch(batch, sim_time)
        for aid in batch:
            if aid in idle_aids or aid in agv_pushed:
                agv_done_notified.discard(aid)

    agv_pending_replan.clear()


def extend_with_exit(done_ids, mcs_goals, sim_time):
    global exit_count
    used = set(mcs_goals)
    for aid, g in agv_goals.items():
        if g:
            used.add(g)
    for a in agents:
        if a.raw_path:
            used.add(a.raw_path[-1][0].split(',')[1])

    for a in agents:
        if a.id in done_ids:
            continue
        if a.path_idx >= len(a.raw_path):
            continue
        agv_goal = agv_goals.get(a.id)
        if not agv_goal or agv_goal not in mcs_goals:
            continue

        exit_port = None
        b = mcs.bindings.get(a.id)

        if a.id in agv_pushed:
            reason = 'PUSHED'
            for p in port_nodes:
                if p != agv_goal and p not in used:
                    exit_port = p
                    break
        elif b and b.load:
            if b.phase == VehicleJobState.TO_PICKUP:
                exit_port = b.load.dst_port
                reason = f'TO_PICKUP→dst={exit_port}'
            elif b.phase == VehicleJobState.TO_DELIVERY:
                reason = 'TO_DELIVERY→idle'
                for p in port_nodes:
                    if p != agv_goal and p not in used:
                        exit_port = p
                        break
            else:
                continue
        else:
            continue

        if exit_port is None or exit_port == agv_goal:
            continue

        last_sid, last_t = a.raw_path[-1]
        arrival_node = last_sid.split(',')[1]

        exit_constraints = []
        for other in agents:
            if other.id == a.id or other.id in done_ids:
                continue
            if other.path_idx >= len(other.raw_path):
                o_sid = other.raw_path[-1][0]
                exit_constraints.append({
                    'agent': other.id, 'loc': o_sid,
                    'timestep': (last_t, float('inf')),
                })

        result = planner.plan(
            {a.id: arrival_node}, {a.id: exit_port},
            existing_constraints=exit_constraints,
            start_times={a.id: last_t},
        )

        if result and result.paths.get(a.id):
            exit_path = result.paths[a.id]
            if len(exit_path) >= 2:
                ext = exit_path[1:]
                a.raw_path.extend(ext)
                used.add(exit_port)
                for k in range(len(a.raw_path) - len(ext), len(a.raw_path)):
                    sid, t = a.raw_path[k]
                    nk = env._nk(sid, a.id, t)
                    duration = (float('inf') if k == len(a.raw_path) - 1
                                else a.raw_path[k + 1][1] - t)
                    env.G.add_node(nk, agv_id=a.id, start_time=t, duration=duration)
                    if k > 0:
                        prev_sid, prev_t = a.raw_path[k - 1]
                        prev_nk = env._nk(prev_sid, a.id, prev_t)
                        env._add_edge(prev_nk, nk)
                exit_count += 1
                print(f'  [EXIT] A{a.id}: {reason} {agv_goal}→{exit_port} (+{len(ext)} steps)')


# ── 메인 시뮬레이션 루프 ──
mcs.start_production(t=0.0)
sim_time = 0.0
step_count = 0

while sim_time < SIM_DURATION:
    sim_time += DT
    env.step(sim_time)

    # DONE 감지 → MCS 통보
    for a in agents:
        # arrival 감지
        if a.id not in agv_arrived_notified:
            arr_idx = agv_arrival_idx.get(a.id)
            if arr_idx is not None and a.path_idx >= arr_idx:
                b = mcs.bindings.get(a.id)
                if b and b.load is not None:
                    post_vehicle_arrived(mcs_heap, mcs_seq, a.id, b.token, sim_time)
                agv_arrived_notified.add(a.id)

        # DONE 감지
        if a.state == AGV_DONE and a.id not in agv_done_notified:
            if a.id in agv_pushed:
                agv_goals.pop(a.id, None)
                agv_done_notified.add(a.id)
                continue
            if a.id in agv_arrived_notified:
                agv_done_notified.add(a.id)
                continue
            b = mcs.bindings.get(a.id)
            if b and b.load is not None:
                post_vehicle_arrived(mcs_heap, mcs_seq, a.id, b.token, sim_time)
            agv_done_notified.add(a.id)

    # MCS step
    mcs.handle_all(sim_time)
    step_count += 1

# ── 결과 ──
stats = mcs.stats_summary(sim_time)
print(f'\n{"="*60}')
print(f'Simulation: {SIM_DURATION}s, {step_count} steps, seed={SEED}')
print(f'SIPP: {sipp_success_count} success, {sipp_fail_count} fail')
print(f'EXIT extensions: {exit_count}')
print(f'MCS: waiting={stats["waiting"]} active={stats["active"]} '
      f'completed={stats["completed"]}')
print(f'  throughput={stats["throughput"]:.2f}/min  '
      f'avg_cycle={stats["avg_cycle"]:.1f}s  '
      f'utilization={stats["utilization"]:.0%}')
print(f'{"="*60}')
