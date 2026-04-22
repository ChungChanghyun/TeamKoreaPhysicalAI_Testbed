"""
env_tapg.py — TAPG (Temporal Action Precedence Graph) DES environment.

CBS time-based plan을 action-dependency DAG로 변환합니다.
각 에이전트는 모든 cross-agent 선행 의존 작업이 완료된 시점에만 M/R 액션을
실행하므로, 실제 주행 시간의 차이가 있어도 충돌이 발생하지 않습니다.

TAPG 노드
─────────
  (state_id, agv_id, cbs_start_time)  — ALL 상태 포함 (S, M, R)

엣지
────
  순차 엣지   : 같은 에이전트 내 연속 상태 사이
  교차 에이전트 : non-S 상태 쌍 중 affect_state 충돌 + t1 <= t2 조건 만족 시
                agent j의 path에서 t2 >= t1인 가장 첫 번째 non-S 충돌 상태만 연결

클레임 조건
────────────
  nk의 모든 predecessor 중 다른 에이전트의 것이 모두 completed_nodes에 있을 때
"""
from __future__ import annotations
import math
import heapq
import networkx as nx
INF = math.inf
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Any

from pkl_loader import PklMapGraph

# ── 상수 ──────────────────────────────────────────────────────────────────────
ROTATION_TIME_90  = 2.0                                 # seconds per 90° turn
ANGULAR_SPEED     = math.radians(90.0) / ROTATION_TIME_90  # rad/s

IDLE     = 'idle'
MOVING   = 'moving'
ROTATING = 'rotating'
WAITING  = 'waiting'
DONE     = 'done'


# ── 이벤트 ────────────────────────────────────────────────────────────────────

@dataclass(order=True)
class Event:
    time:     float
    seq:      int
    kind:     str  = field(compare=False)
    agent_id: int  = field(compare=False)
    data:     dict = field(compare=False, default_factory=dict)


# ── TAPG 에이전트 ─────────────────────────────────────────────────────────────

class TAPGAgent:
    """
    CBS raw_path [(state_id, cbs_cost), ...] 를 따라 이동하는 에이전트.
    S 상태는 즉시 통과하고, M/R 상태는 TAPG 선행 조건을 확인합니다.
    """

    def __init__(self, agent_id: int, color: tuple, raw_path: list):
        self.id       = agent_id
        self.color    = color
        self.raw_path = raw_path   # [(state_id, cbs_cost), ...]
        self.path_idx = 0
        self.state    = IDLE

        # 위치 / 자세
        self.x:     float = 0.0
        self.y:     float = 0.0
        self.theta: float = 0.0
        self.v:     float = 0.0

        # MOVING — 위치 기반 도달 감지
        self.from_x:        float = 0.0
        self.from_y:        float = 0.0
        self.to_x:          float = 0.0
        self.to_y:          float = 0.0
        self.dist_traveled: float = 0.0   # 현재 엣지에서 이동한 거리 (mm)
        self.edge_length:   float = 0.0   # 현재 엣지 전체 길이 (mm)
        self.max_speed:     float = 0.0   # 현재 엣지 최고 속도 (mm/s)

        # ROTATING — 각도 기반 완료 감지
        self.from_theta:      float = 0.0
        self.to_theta:        float = 0.0
        self.angle_traversed: float = 0.0   # 이번 회전에서 회전한 각도 (rad)
        self.angle_total:     float = 0.0   # 목표 회전 각도 (rad)

        # 현재 실행 중인 TAPG 노드 (도달 시 complete 처리에 사용)
        self._tapg_node: tuple = None

        # Claim: 실제 주행이 확정된 구간의 끝 인덱스 (exclusive)
        # path_idx ~ claim_idx 구간은 멈출 수 없음
        self.claim_idx: int = 0

    @property
    def cur_state_id(self) -> str:
        if self.path_idx < len(self.raw_path):
            return self.raw_path[self.path_idx][0]
        return self.raw_path[-1][0] if self.raw_path else ''

    @property
    def node_path(self) -> List[str]:
        """시각화용 node_id 경로 (S 상태 기반, 연속 중복 제거)."""
        nodes: List[str] = []
        for sid, _ in self.raw_path:
            if sid.startswith('S,'):
                nid = sid.split(',')[1]
                if not nodes or nodes[-1] != nid:
                    nodes.append(nid)
        return nodes


# ── TAPG 환경 ─────────────────────────────────────────────────────────────────

class TAPGEnvironment:
    """
    TAPG 기반 DES 실행 환경.

    setup(raw_paths, agent_ids, colors) 으로 초기화하고
    step(sim_time) 을 매 프레임 호출합니다.
    """

    PERIODIC_INTERVAL = 0.5   # sim-seconds: WAITING 에이전트 재확인 주기

    def __init__(self, graph: PklMapGraph,
                 accel: float = INF,
                 decel: float = INF):
        """
        Parameters
        ----------
        accel : mm/s²  가속도. INF → 즉시 최고속도(상수 속도 모드)
        decel : mm/s²  감속도. INF → 즉시 정지(위치 도달 순간 스냅)
        """
        self.graph           = graph
        self.accel           = accel
        self.decel           = decel
        self._eq: list       = []
        self._seq            = 0
        self.agents:         Dict[int, TAPGAgent]  = {}
        self.sim_time        = 0.0
        self._last_check     = 0.0

        # TAPG
        self.G:               nx.DiGraph            = nx.DiGraph()
        self.wait_queues:     Dict[Any, List[int]]  = {}  # tapg_node -> [agent_id]

    def _add_edge(self, u, v):
        """self-loop 방지 래퍼. DAG에 자기 참조 엣지는 절대 추가하지 않는다."""
        if u == v:
            import traceback
            print(f'  [TAPG] self-loop BLOCKED: {u}')
            traceback.print_stack(limit=4)
            return
        self.G.add_edge(u, v)

    # ── 초기화 ────────────────────────────────────────────────────────────────

    def setup(self, agents: list, t_start: float = 0.0):
        """
        agents: 외부에서 생성한 TAPGAgent 객체 목록 (이 객체를 직접 업데이트함).
        같은 객체를 시각화쪽에서 참조해야 위치/상태가 반영된다.
        """
        self.agents = {a.id: a for a in agents}
        self.wait_queues.clear()
        self.G.clear()
        self._eq        = []
        self._seq       = 0
        self.sim_time   = t_start
        self._last_check = t_start

        raw_paths = [a.raw_path for a in agents]
        agent_ids = [a.id      for a in agents]

        # 에이전트 상태 초기화 + 시작 위치 설정
        for a in agents:
            a.path_idx = 0
            a.claim_idx = 0
            a.state    = IDLE
            a.v        = 0.0
            if a.raw_path:
                nid = self._node_of(a.raw_path[0][0])
                if nid and nid in self.graph.nodes:
                    n = self.graph.nodes[nid]
                    a.x, a.y = n.x, n.y
                a.theta = self._heading_of(a.raw_path[0][0])

        self._build_tapg(raw_paths, agent_ids)

        for aid in agent_ids:
            self._schedule(t_start, 'TRY_ADVANCE', aid)

    # ── TAPG 구성 ─────────────────────────────────────────────────────────────

    def _build_tapg(self, raw_paths: list, agent_ids: list):
        """
        Automod prioritized solver의 construct_temporal_graph 와 동일한 논리.

        노드 키: (state_id, agv_id, cbs_start_time)
        """
        # ① 모든 상태 노드 + 같은 에이전트 내 순차 엣지
        for aid, path in zip(agent_ids, raw_paths):
            for k, (sid, t) in enumerate(path):
                nk = self._nk(sid, aid, t)
                duration = (float('inf') if k == len(path) - 1
                            else path[k + 1][1] - t)
                self.G.add_node(nk, agv_id=aid, start_time=t, duration=duration)
                if k > 0:
                    prev_sid, prev_t = path[k - 1]
                    self._add_edge(self._nk(prev_sid, aid, prev_t), nk)

        # ② 교차-에이전트 엣지 (S/M/R 모두 포함)
        for i, (ai, pi) in enumerate(zip(agent_ids, raw_paths)):
            for j, (aj, pj) in enumerate(zip(agent_ids, raw_paths)):
                if i == j:
                    continue
                for k1 in range(len(pi) - 1, -1, -1):
                    s1, t1 = pi[k1]
                    affect1 = self._state_affect_set(s1)
                    if not affect1:
                        continue

                    for k2, (s2, t2) in enumerate(pj):
                        if t2 <= t1:
                            continue
                        affect2 = self._state_affect_set(s2)
                        if s2 in affect1 or s1 in affect2:
                            self._add_edge(self._nk(s1, ai, t1), self._nk(s2, aj, t2))
                            break

    # ── 메인 루프 ──────────────────────────────────────────────────────────────

    def step(self, sim_time: float):
        dt            = max(0.0, sim_time - self.sim_time)
        self.sim_time = sim_time

        # TRY_ADVANCE 등 예약 이벤트 처리
        while self._eq and self._eq[0].time <= sim_time:
            ev = heapq.heappop(self._eq)
            self._process(ev)

        # 위치 기반 이동/도달 처리 (센서 시뮬레이션)
        self._update_positions(dt, sim_time)

        # 주기적으로 WAITING 에이전트 재확인 (event miss 방어)
        if sim_time - self._last_check >= self.PERIODIC_INTERVAL:
            self._last_check = sim_time
            self._periodic_wakeup(sim_time)

    # ── 이벤트 처리 ────────────────────────────────────────────────────────────

    def _process(self, ev: Event):
        handler = {
            'TRY_ADVANCE': self._on_try_advance,
        }.get(ev.kind)
        if handler:
            handler(ev)

    def _on_try_advance(self, ev: Event):
        agent = self.agents.get(ev.agent_id)
        if agent is None or agent.state in (MOVING, ROTATING, DONE):
            return

        # S 상태는 연속으로 즉시 통과
        while (agent.path_idx < len(agent.raw_path)
               and agent.raw_path[agent.path_idx][0].startswith('S,')):
            sid = agent.raw_path[agent.path_idx][0]
            nid = self._node_of(sid)
            if nid and nid in self.graph.nodes:
                n = self.graph.nodes[nid]
                agent.x, agent.y = n.x, n.y
            agent.theta = self._heading_of(sid)

            # 다음이 M/R이면: claimed 범위 내면 무조건 통과, 아니면 체크
            next_idx = agent.path_idx + 1
            if next_idx < len(agent.raw_path):
                next_sid, next_t = agent.raw_path[next_idx]
                if not next_sid.startswith('S,'):
                    if next_idx < agent.claim_idx:
                        pass  # 이미 claimed → 무조건 통과
                    else:
                        next_nk = self._nk(next_sid, agent.id, next_t)
                        if not self._is_claimable(next_nk, agent.id):
                            break
                        if not self._try_claim_next(agent):
                            break

            agent.path_idx += 1

        if agent.path_idx >= len(agent.raw_path):
            agent.state = DONE
            agent.v     = 0.0
            return

        sid, cbs_t = agent.raw_path[agent.path_idx]
        nk = self._nk(sid, agent.id, cbs_t)

        # S state에서 대기 → WAITING 등록
        if sid.startswith('S,'):
            next_idx = agent.path_idx + 1
            if next_idx < len(agent.raw_path):
                next_sid, next_t = agent.raw_path[next_idx]
                next_nk = self._nk(next_sid, agent.id, next_t)
                if next_nk in self.G:
                    for pred in self.G.predecessors(next_nk):
                        if pred[1] != agent.id:
                            q = self.wait_queues.setdefault(pred, [])
                            if agent.id not in q:
                                q.append(agent.id)
            agent.state = WAITING
            agent.v     = 0.0
            return

        # Claimed 범위 내면 무조건 실행
        if agent.path_idx < agent.claim_idx:
            if sid.startswith('M,'):
                self._start_move(agent, sid, cbs_t, ev.time)
            elif sid.startswith('R,'):
                self._start_rotate(agent, sid, cbs_t, ev.time)
            else:
                self._complete_node(nk, ev.time)
                agent.path_idx += 1
                self._schedule(ev.time, 'TRY_ADVANCE', agent.id)
            return

        # Claimed 범위 밖: claimable 체크 + claim 확장
        if not self._is_claimable(nk, agent.id):
            agent.state = WAITING
            agent.v     = 0.0
            for pred in self.G.predecessors(nk):
                if pred[1] != agent.id:
                    q = self.wait_queues.setdefault(pred, [])
                    if agent.id not in q:
                        q.append(agent.id)
            return

        if not self._try_claim_next(agent):
            agent.state = WAITING
            agent.v     = 0.0
            return

        # 액션 실행
        if sid.startswith('M,'):
            self._start_move(agent, sid, cbs_t, ev.time)
        elif sid.startswith('R,'):
            self._start_rotate(agent, sid, cbs_t, ev.time)
        else:
            # 알 수 없는 상태: 건너뜀
            self._complete_node(nk, ev.time)
            agent.path_idx += 1
            self._schedule(ev.time, 'TRY_ADVANCE', agent.id)

    def _start_move(self, agent: TAPGAgent, sid: str,
                    cbs_t: float, cur_time: float,
                    v_init: float = None):
        """
        v_init: 연속 주행 체이닝 시 이전 엣지에서 이어받는 속도.
                None이면 가속도 설정에 따라 0 또는 max_speed에서 출발.
        """
        parts        = sid.split(',')
        from_n, to_n = parts[1], parts[2]
        edge         = self.graph.get_edge(from_n, to_n)
        if edge is None:
            nk = self._nk(sid, agent.id, cbs_t)
            self._complete_node(nk, cur_time)
            agent.path_idx += 1
            self._schedule(cur_time, 'TRY_ADVANCE', agent.id)
            return

        fn = self.graph.nodes[from_n]
        tn = self.graph.nodes[to_n]

        agent.from_x        = fn.x
        agent.from_y        = fn.y
        agent.to_x          = tn.x
        agent.to_y          = tn.y
        agent.theta         = edge.angle
        agent.max_speed     = edge.max_speed
        if v_init is not None:
            # 연속 주행: 이전 속도 유지 (max_speed 초과 방지)
            agent.v = min(v_init, edge.max_speed)
        elif math.isfinite(self.accel):
            agent.v = 0.0           # 유한 가속도: 정지 상태에서 출발
        else:
            agent.v = edge.max_speed  # 즉시 최고속도
        agent.dist_traveled = 0.0
        agent.edge_length   = edge.length
        agent._tapg_node    = self._nk(sid, agent.id, cbs_t)
        agent.state         = MOVING
        # ACTION_DONE 이벤트 없음 — _update_positions에서 위치 기반으로 도달 감지

    def _start_rotate(self, agent: TAPGAgent, sid: str,
                      cbs_t: float, cur_time: float):
        parts    = sid.split(',')
        from_deg = float(parts[2])
        to_deg   = float(parts[3])

        # 최단 경로 방향의 부호 있는 각도 계산
        # CCW(+) vs CW(-) 중 짧은 쪽을 선택
        diff_ccw = (to_deg - from_deg) % 360   # 반시계 방향으로 얼마나 돌아야 하는지
        if diff_ccw <= 180:
            signed_diff = math.radians(diff_ccw)   # CCW (+)
        else:
            signed_diff = math.radians(diff_ccw - 360)  # CW (-)

        from_rad = math.radians(from_deg)

        agent.from_theta      = from_rad
        agent.to_theta        = from_rad + signed_diff  # 최단 경로 기준 도달 각도
        agent.theta           = from_rad
        agent.v               = 0.0
        agent.angle_traversed = 0.0
        agent.angle_total     = abs(signed_diff)
        agent._tapg_node      = self._nk(sid, agent.id, cbs_t)
        agent.state           = ROTATING
        # ACTION_DONE 이벤트 없음 — _update_positions에서 각도 기반으로 완료 감지

    # ── 주기적 재확인 ──────────────────────────────────────────────────────────

    def _periodic_wakeup(self, sim_time: float):
        """
        WAITING 에이전트 중 선행 조건이 해소된 에이전트를 깨웁니다.
        event-driven wakeup이 누락되는 경우를 방어합니다.
        """
        for agent in list(self.agents.values()):
            if agent.state != WAITING:
                continue
            if agent.path_idx >= len(agent.raw_path):
                agent.state = DONE
                continue
            sid, cbs_t = agent.raw_path[agent.path_idx]
            nk = self._nk(sid, agent.id, cbs_t)
            if self._is_claimable(nk, agent.id):
                agent.state = IDLE
                self._schedule(sim_time, 'TRY_ADVANCE', agent.id)

    # ── 완료 처리 ──────────────────────────────────────────────────────────────

    def _complete_node(self, nk: tuple, cur_time: float):
        """TAPG 노드를 DAG에서 제거하고 대기 중인 에이전트를 깨웁니다."""
        # 대기 중인 agent 깨우기 (제거 전에 처리)
        waiters = self.wait_queues.pop(nk, [])

        # DAG에서 노드 제거 → 후행 노드의 in-edge가 자동으로 사라짐
        if self.G.has_node(nk):
            self.G.remove_node(nk)

        for wid in waiters:
            wa = self.agents.get(wid)
            if wa and wa.state == WAITING:
                wa.state = IDLE
                self._schedule(cur_time + 1e-9, 'TRY_ADVANCE', wid)

    # ── TAPG 클레임 확인 ───────────────────────────────────────────────────────

    def _is_claimable(self, nk: tuple, agent_id: int) -> bool:
        """다른 에이전트의 선행 TAPG 노드가 DAG에 없으면(제거됨=완료) True."""
        if nk not in self.G:
            return True
        for pred in self.G.predecessors(nk):
            if pred[1] != agent_id:
                return False
        return True

    def _try_claim_next(self, agent: TAPGAgent) -> bool:
        """다음 M/R action까지 claim 확장 시도.
        TAPG claimable 체크를 통과하면 claim 확장."""
        # 다음 M/R의 인덱스 찾기
        target_idx = agent.claim_idx
        while target_idx < len(agent.raw_path):
            sid = agent.raw_path[target_idx][0]
            if sid.startswith('M,') or sid.startswith('R,'):
                # 이 M/R이 TAPG상 claimable인지 확인
                t = agent.raw_path[target_idx][1]
                nk = self._nk(sid, agent.id, t)
                if not self._is_claimable(nk, agent.id):
                    return False

                # Claim 확장: M/R + 그 다음 S까지
                end_idx = target_idx + 1
                while end_idx < len(agent.raw_path) and agent.raw_path[end_idx][0].startswith('S,'):
                    end_idx += 1
                agent.claim_idx = end_idx
                return True
            target_idx += 1

        return False

    # ── 위치 기반 이동 / 도달 감지 (센서 시뮬레이션) ──────────────────────────

    def _update_positions(self, dt: float, sim_time: float):
        """
        dt 동안 각 에이전트를 물리적으로 전진시키고,
        목적지 도달 여부를 거리/각도로 판단합니다.
        CBS 플래닝 타임과 무관하게 실제 이동량 기준으로 완료를 결정합니다.
        """
        if dt <= 0.0:
            return

        arrivals = []

        for agent in self.agents.values():
            if agent.state == MOVING:
                # 사다리꼴 속도 프로파일 (accel/decel이 유한할 때만 적용)
                d_rem   = agent.edge_length - agent.dist_traveled
                d_brake = (agent.v ** 2 / (2.0 * self.decel)
                           if math.isfinite(self.decel) else 0.0)
                if math.isfinite(self.decel) and d_rem <= d_brake + 1e-6:
                    # 제동 구간 — 하지만 다음 Move가 이미 claimable이면 감속 생략
                    if (math.isfinite(self.accel)
                            and self._next_move_claimable(agent)):
                        # 연속 주행: 감속 없이 통과
                        if agent.v < agent.max_speed:
                            agent.v = min(agent.max_speed,
                                          agent.v + self.accel * dt)
                    else:
                        agent.v = max(0.0, agent.v - self.decel * dt)
                elif math.isfinite(self.accel) and agent.v < agent.max_speed:
                    # 가속 구간
                    agent.v = min(agent.max_speed, agent.v + self.accel * dt)
                # else: 최고속도 유지 (cruise)

                agent.dist_traveled += agent.v * dt
                if agent.dist_traveled >= agent.edge_length - 1e-6:
                    # 도달 — 목적지로 스냅 (센서 트리거)
                    agent.x = agent.to_x
                    agent.y = agent.to_y
                    arrivals.append(agent)
                else:
                    frac    = agent.dist_traveled / agent.edge_length
                    agent.x = agent.from_x + (agent.to_x - agent.from_x) * frac
                    agent.y = agent.from_y + (agent.to_y - agent.from_y) * frac

            elif agent.state == ROTATING:
                agent.angle_traversed += ANGULAR_SPEED * dt
                if agent.angle_total <= 0 or agent.angle_traversed >= agent.angle_total:
                    # 회전 완료 — 목표 각도로 스냅
                    agent.theta = agent.to_theta
                    arrivals.append(agent)
                else:
                    frac        = agent.angle_traversed / agent.angle_total
                    agent.theta = (agent.from_theta
                                   + (agent.to_theta - agent.from_theta) * frac)

        for agent in arrivals:
            self._handle_arrival(agent, sim_time)

    def _handle_arrival(self, agent: TAPGAgent, sim_time: float):
        """에이전트가 목적지(또는 회전 목표)에 도달했을 때 호출됩니다.
        완료된 M/R action + 그 이전의 S state들을 DAG에서 한꺼번에 제거."""
        nk               = agent._tapg_node
        carry_v          = agent.v
        agent._tapg_node = None
        agent.path_idx  += 1
        agent.state      = IDLE

        # 현재 완료된 action(M/R) + 그 이전의 모든 S state를 DAG에서 제거
        if nk:
            # path에서 현재 path_idx 이전의 모든 state 제거
            for i in range(agent.path_idx):
                sid_i, t_i = agent.raw_path[i]
                nk_i = self._nk(sid_i, agent.id, t_i)
                if self.G.has_node(nk_i):
                    # wait_queues에서 대기자 깨우기
                    for wid in self.wait_queues.pop(nk_i, []):
                        wa = self.agents.get(wid)
                        if wa and wa.state == WAITING:
                            wa.state = IDLE
                            self._schedule(sim_time + 1e-9, 'TRY_ADVANCE', wid)
                    self.G.remove_node(nk_i)

        # 유한 가속도 모드 + Move 완료 시 연속 주행 시도
        if math.isfinite(self.accel) and carry_v > 0 and nk and nk[0].startswith('M,'):
            if self._try_chain_move(agent, carry_v, sim_time):
                return

        agent.v = 0.0
        self._schedule(sim_time, 'TRY_ADVANCE', agent.id)

    # ── 연속 주행 헬퍼 ────────────────────────────────────────────────────────

    def _peek_next_move(self, agent: TAPGAgent):
        """현재 path_idx 이후의 첫 번째 M 상태 (S 상태 건너뜀) 반환. 없으면 None."""
        idx = agent.path_idx + 1
        while idx < len(agent.raw_path):
            sid, t = agent.raw_path[idx]
            if sid.startswith('S,'):
                idx += 1
                continue
            return (sid, t) if sid.startswith('M,') else None
        return None

    def _next_move_claimable(self, agent: TAPGAgent) -> bool:
        """다음 M 상태가 이미 claimed이거나 claim 가능한지 확인."""
        nm = self._peek_next_move(agent)
        if nm is None:
            return False
        # 이미 claimed 범위 내면 OK
        nm_idx = None
        for i in range(agent.path_idx + 1, len(agent.raw_path)):
            if agent.raw_path[i][0] == nm[0] and abs(agent.raw_path[i][1] - nm[1]) < 1e-6:
                nm_idx = i
                break
        if nm_idx is not None and nm_idx < agent.claim_idx:
            return True
        # Claim 확장 시도
        return self._try_claim_next(agent)

    def _try_chain_move(self, agent: TAPGAgent,
                        carry_v: float, sim_time: float) -> bool:
        """
        도달 직후 S 상태를 inline으로 처리하고,
        다음 M이 claimable이면 멈추지 않고 바로 _start_move 진입.

        성공하면 True, 체이닝 불가(R 상태·claimable 아님·경로 끝)이면 False.
        """
        # S 상태 inline 통과 — 위치만 갱신, DAG 제거는 다음 M 완료 시
        while (agent.path_idx < len(agent.raw_path)
               and agent.raw_path[agent.path_idx][0].startswith('S,')):
            sid, t = agent.raw_path[agent.path_idx]
            nid = self._node_of(sid)
            if nid and nid in self.graph.nodes:
                n = self.graph.nodes[nid]
                agent.x, agent.y = n.x, n.y
            agent.theta = self._heading_of(sid)

            next_idx = agent.path_idx + 1
            if next_idx < len(agent.raw_path):
                next_sid, next_t = agent.raw_path[next_idx]
                if not next_sid.startswith('S,'):
                    next_nk = self._nk(next_sid, agent.id, next_t)
                    if not self._is_claimable(next_nk, agent.id):
                        return False

            agent.path_idx += 1

        if agent.path_idx >= len(agent.raw_path):
            agent.state = DONE
            agent.v     = 0.0
            return True   # 경로 완료

        next_sid, next_t = agent.raw_path[agent.path_idx]
        next_nk = (next_sid, agent.id, next_t)

        if next_sid.startswith('M,') and self._is_claimable(next_nk, agent.id):
            # 속도를 유지하며 다음 엣지로 바로 진입
            self._start_move(agent, next_sid, next_t, sim_time, v_init=carry_v)
            return True

        return False  # 체이닝 불가 — 호출자가 v=0 처리

    # ── 헬퍼 ──────────────────────────────────────────────────────────────────

    @staticmethod
    def _round_t(t: float) -> float:
        """TAPG 노드 키의 시간을 반올림하여 부동소수점 불일치를 방지."""
        return round(t, 6)

    @staticmethod
    def _nk(sid: str, aid: int, t: float) -> tuple:
        """TAPG 노드 키 생성 — 시간을 반올림하여 부동소수점 불일치 방지."""
        return (sid, aid, round(t, 6))

    def _get_state_obj(self, state_id: str):
        # R 상태는 별도 dict 없이 Move_state에 같이 저장됨
        if state_id.startswith('M,') or state_id.startswith('R,'):
            return self.graph.move_states_raw.get(state_id)
        if state_id.startswith('S,'):
            return self.graph.stop_states_raw.get(state_id)
        return None

    def _state_affect_set(self, state_id: str) -> set:
        """state_id의 affect_state set 반환."""
        obj = self._get_state_obj(state_id)
        return set(getattr(obj, 'affect_state', [])) if obj else set()

    def _effective_state_ids(self, state_id: str) -> list:
        """충돌 체크에서 이 state가 상대방 affect_set에 있는지 확인할 ID 목록."""
        return [state_id]

    @staticmethod
    def _node_of(state_id: str) -> Optional[str]:
        parts = state_id.split(',')
        return parts[1] if len(parts) >= 2 else None

    @staticmethod
    def _heading_of(state_id: str) -> float:
        parts = state_id.split(',')
        if parts[0] == 'S' and len(parts) >= 3:
            try:
                return math.radians(float(parts[2]))
            except ValueError:
                pass
        return 0.0

    def _schedule(self, time: float, kind: str, agent_id: int, **data):
        ev = Event(time, self._seq, kind, agent_id, data)
        self._seq += 1
        heapq.heappush(self._eq, ev)

    # ── 상태 조회 ──────────────────────────────────────────────────────────────

    # ── 동적 경로 확장 (incremental replan) ─────────────────────────────────

    def extend_agents_batch(self, agent_paths: dict, t_start: float):
        """
        여러 DONE agent에 새 경로를 한번에 부여하고 TAPG DAG를 확장합니다.

        1) 모든 새 경로의 agent 상태 초기화 + DAG 노드/순차엣지 추가
        2) 모든 새 경로가 등록된 후 cross-agent 엣지 빌드
        3) TRY_ADVANCE 스케줄

        Parameters
        ----------
        agent_paths : {agent_id: new_raw_path} — 새 경로를 받을 agent들
        t_start     : 현재 sim_time
        """
        new_paths = {}  # aid → raw_path (등록된 것만)

        # ── Phase 0: 이전 경로의 DAG 노드 정리 ──────────────────────────
        for aid, new_raw_path in agent_paths.items():
            agent = self.agents.get(aid)
            if agent is None or not new_raw_path:
                continue
            # 이전 경로의 남은 노드를 DAG에서 제거 (DONE 시 정리 안 된 것 포함)
            old_path = agent.raw_path
            if old_path:
                for k, (sid, t) in enumerate(old_path):
                    old_nk = self._nk(sid, aid, t)
                    if self.G.has_node(old_nk):
                        self.G.remove_node(old_nk)
                # wait_queues에서도 제거
                for nk, waiters in list(self.wait_queues.items()):
                    if aid in waiters:
                        waiters.remove(aid)
                    if not waiters:
                        self.wait_queues.pop(nk, None)

        # ── Phase 1: agent 상태 초기화 + DAG 노드/순차엣지 추가 ──────────
        for aid, new_raw_path in agent_paths.items():
            agent = self.agents.get(aid)
            if agent is None or not new_raw_path:
                continue

            agent.raw_path = new_raw_path
            agent.path_idx = 0
            agent.claim_idx = 0
            agent.state    = IDLE
            agent.v        = 0.0

            if new_raw_path:
                nid = self._node_of(new_raw_path[0][0])
                if nid and nid in self.graph.nodes:
                    n = self.graph.nodes[nid]
                    agent.x, agent.y = n.x, n.y
                agent.theta = self._heading_of(new_raw_path[0][0])

            for k, (sid, t) in enumerate(new_raw_path):
                nk = self._nk(sid, aid, t)
                # ── DEBUG: Phase1 노드가 이미 그래프에 있는지 확인 ──
                if nk in self.G:
                    old_edges_in = list(self.G.predecessors(nk))
                    old_edges_out = list(self.G.successors(nk))
                    if old_edges_in or old_edges_out:
                        print(f'  [TAPG WARN] Phase1: node {nk} already exists '
                              f'with in={len(old_edges_in)} out={len(old_edges_out)}')
                        for p in old_edges_in[:3]:
                            print(f'    pred: A{p[1]-100 if isinstance(p[1],int) else p[1]} {p[0]} t={p[2]:.2f}')
                        for s in old_edges_out[:3]:
                            print(f'    succ: A{s[1]-100 if isinstance(s[1],int) else s[1]} {s[0]} t={s[2]:.2f}')
                duration = (float('inf') if k == len(new_raw_path) - 1
                            else new_raw_path[k + 1][1] - t)
                self.G.add_node(nk, agv_id=aid, start_time=t, duration=duration)
                if k > 0:
                    prev_sid, prev_t = new_raw_path[k - 1]
                    prev_nk = self._nk(prev_sid, aid, prev_t)
                    if prev_nk != nk:  # self-loop 방지
                        self._add_edge(prev_nk, nk)

            new_paths[aid] = new_raw_path

        # ── Phase 2: cross-agent 엣지 빌드 ──────────────────────────────
        # 새 경로들 + 기존 active 경로들 전체를 대상으로 빌드
        all_paths = {}
        for aid, agent in self.agents.items():
            if agent.raw_path:
                all_paths[aid] = agent.raw_path  # 새 경로 포함 (Phase 1에서 교체됨)

        for ai, pi in new_paths.items():
            for aj, pj in all_paths.items():
                if ai == aj:
                    continue
                # pi vs pj: pi의 state가 pj를 block
                for k1 in range(len(pi) - 1, -1, -1):
                    s1, t1 = pi[k1]
                    affect1 = self._state_affect_set(s1)
                    if not affect1:
                        continue
                    for k2, (s2, t2) in enumerate(pj):
                        if t2 <= t1:
                            continue
                        affect2 = self._state_affect_set(s2)
                        if s2 in affect1 or s1 in affect2:
                            nk1 = self._nk(s1, ai, t1)
                            nk2 = self._nk(s2, aj, t2)
                            if nk1 == nk2:
                                print(f'  [TAPG SELF-LOOP] Phase2-A: {nk1}')
                            if nk1 in self.G and nk2 in self.G:
                                self._add_edge(nk1, nk2)
                            break

                # pj vs pi: pj의 state가 pi를 block
                if aj in new_paths:
                    continue
                for k1 in range(len(pj) - 1, -1, -1):
                    s1, t1 = pj[k1]
                    affect1 = self._state_affect_set(s1)
                    if not affect1:
                        continue
                    for k2, (s2, t2) in enumerate(pi):
                        if t2 <= t1:
                            continue
                        affect2 = self._state_affect_set(s2)
                        if s2 in affect1 or s1 in affect2:
                            nk1 = self._nk(s1, aj, t1)
                            nk2 = self._nk(s2, ai, t2)
                            if nk1 == nk2:
                                print(f'  [TAPG SELF-LOOP] Phase2-B: {nk1}')
                            if nk1 in self.G and nk2 in self.G:
                                self._add_edge(nk1, nk2)
                            break

        # ── Phase 3: TRY_ADVANCE 스케줄 ─────────────────────────────────
        for aid in new_paths:
            self._schedule(t_start, 'TRY_ADVANCE', aid)

    def recompute_earliest_schedule(self, current_time=0.0):
        """
        TAPG DAG의 earliest start schedule을 재계산합니다.

        1) 모든 agent의 DAG 시작 노드 시간 = current_time
        2) 위상 정렬 순서대로 earliest start 전파
        3) M/R의 duration = state.cost (고정)
           S의 duration = 다음 action의 earliest_start - S의 earliest_start
        """
        import networkx as nx

        if self.G.number_of_nodes() == 0:
            return {}

        try:
            topo_order = list(nx.topological_sort(self.G))
        except nx.NetworkXUnfeasible:
            return {}

        # 각 agent의 DAG 시작 노드 찾기 (in-degree 0 중 같은 agent)
        start_nodes = set()
        for v in topo_order:
            same_agent_preds = [p for p in self.G.predecessors(v) if p[1] == v[1]]
            if not same_agent_preds:
                start_nodes.add(v)

        # Earliest start 계산
        new_start = {}
        for v in topo_order:
            state_id = v[0]

            # Cross-agent predecessor의 earliest finish
            cross_pred_finish = []
            for u in self.G.predecessors(v):
                if u[1] != v[1] and u in new_start:
                    # Cross-agent: predecessor의 earliest_start + duration
                    u_sid = u[0]
                    u_state = self._get_state_obj(u_sid)
                    u_dur = u_state.cost if u_state and u_state.cost else 0.0
                    cross_pred_finish.append(new_start[u] + u_dur)

            # Same-agent predecessor의 earliest finish
            same_pred_finish = []
            for u in self.G.predecessors(v):
                if u[1] == v[1] and u in new_start:
                    u_sid = u[0]
                    u_state = self._get_state_obj(u_sid)
                    u_dur = u_state.cost if u_state and u_state.cost else 0.0
                    same_pred_finish.append(new_start[u] + u_dur)

            if v in start_nodes:
                # 시작 노드: current_time, cross-agent 제약 반영
                t_v = current_time
                if cross_pred_finish:
                    t_v = max(t_v, max(cross_pred_finish))
            else:
                all_finish = same_pred_finish + cross_pred_finish
                if all_finish:
                    t_v = max(all_finish)
                else:
                    t_v = current_time

            new_start[v] = t_v

        # 새 그래프 구성
        H = nx.DiGraph()
        old_to_new = {}
        for old_node in topo_order:
            if old_node not in new_start:
                continue
            t_v = new_start[old_node]
            state_id, agv_i, _old_t = old_node
            new_node = self._nk(state_id, agv_i, t_v)
            old_to_new[old_node] = new_node

            attrs = dict(self.G.nodes[old_node])
            attrs['start_time'] = t_v
            # duration 갱신: M/R은 cost, S는 다음 노드에서 결정됨
            state_obj = self._get_state_obj(state_id)
            if state_obj and state_id.startswith('S,'):
                attrs['duration'] = 0  # S의 duration은 후속 노드에 의해 결정
            elif state_obj:
                attrs['duration'] = state_obj.cost
            H.add_node(new_node, **attrs)

        for u, v in self.G.edges():
            if u in old_to_new and v in old_to_new:
                if old_to_new[u] != old_to_new[v]:
                    H.add_edge(old_to_new[u], old_to_new[v])

        self.G = H

        # agent raw_path + _tapg_node 갱신
        for agent in self.agents.values():
            new_raw_path = []
            for idx, (sid, old_t) in enumerate(agent.raw_path):
                old_key = self._nk(sid, agent.id, old_t)
                if old_key in old_to_new:
                    new_raw_path.append((sid, round(old_to_new[old_key][2], 6)))
                else:
                    new_raw_path.append((sid, round(old_t, 6)))
            agent.raw_path = new_raw_path

            if agent._tapg_node and agent._tapg_node in old_to_new:
                agent._tapg_node = old_to_new[agent._tapg_node]

        # wait_queues 키 갱신
        new_wq = {}
        for old_key, waiters in self.wait_queues.items():
            new_key = old_to_new.get(old_key, old_key)
            new_wq[new_key] = waiters
        self.wait_queues = new_wq

        return new_start

    def all_done(self) -> bool:
        return all(a.state == DONE for a in self.agents.values())

    def tapg_stats(self) -> dict:
        total     = self.G.number_of_nodes()
        edges     = self.G.number_of_edges()
        seq_edges = sum(1 for u, v in self.G.edges()
                        if self.G.nodes[u].get('agv_id') == self.G.nodes[v].get('agv_id'))
        return {
            'total':      total,
            'done':       0,
            'remaining':  total - done,
            'edges':      edges,
            'cross_edges': edges - seq_edges,
        }
