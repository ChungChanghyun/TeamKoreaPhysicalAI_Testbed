"""
env_tapg_des.py — TAPG (Temporal Action Precedence Graph) DES environment.
                  Pure DES version: no dt-loop, all motion pre-scheduled.

env_tapg.TAPGEnvironment 와 동일한 외부 인터페이스를 제공하지만,
이동/회전 완료를 EDGE_DONE / ROTATE_DONE 이벤트로 미래 시각에 예약합니다.
시뮬레이션 시간을 실시간보다 훨씬 빠르게 진행할 수 있습니다 (dt 루프 없음).

시각화용 위치는 _update_visuals 에서 SpeedProfile 공식으로 O(1) 계산합니다.

Usage
─────
    from env_tapg_des import TAPGEnvironmentDES, SpeedProfile
    from env_tapg import TAPGAgent   # 동일한 에이전트 클래스 재사용

    env = TAPGEnvironmentDES(graph, accel=500.0, decel=500.0)
    env.setup(agents)
    while not env.all_done():
        env.step(env.sim_time + 0.1)   # 임의 간격으로 진행 가능
"""
from __future__ import annotations
import math
import heapq
import networkx as nx
INF = math.inf
from typing import Dict, List, Optional, Any

from pkl_loader import PklMapGraph

# env_tapg 의 공용 심볼 재사용 (에이전트, 이벤트, 상태 상수)
from env_tapg import (
    TAPGAgent, Event,
    IDLE, MOVING, ROTATING, WAITING, DONE,
    ROTATION_TIME_90, ANGULAR_SPEED,
)


# ── 속도 프로파일 ──────────────────────────────────────────────────────────────

class SpeedProfile:
    """
    사다리꼴 속도 프로파일 (가속 → 정속 → 감속).
    accel=INF, decel=INF 이면 상수 속도 모드 (즉시 최고속도, 즉시 정지).

    모든 값은 mm, mm/s, mm/s² 단위.

    Attributes
    ----------
    t_total : float   전체 소요 시간 (초)
    v_peak  : float   실제 도달한 최고 속도 (mm/s)
    """

    def __init__(self, dist: float, v_init: float, v_max: float,
                 accel: float, decel: float):
        self.dist   = dist
        self.v_max  = v_max
        self.v_init = v_init

        if dist <= 0.0:
            self.v_peak  = 0.0
            self.t1 = self.t2 = self.t_total = 0.0
            self._const = True
            self._a = self._d = 0.0
            self._d_acc = self._d_cru = 0.0
            return

        inf_a = not math.isfinite(accel)
        inf_d = not math.isfinite(decel)

        if inf_a and inf_d:
            # 상수 속도 모드: 즉시 최고속도, 즉시 정지
            self.v_peak  = v_max
            self.t1      = 0.0
            self.t2      = dist / v_max if v_max > 0 else 0.0
            self.t_total = self.t2
            self._const  = True
            self._a = self._d = 0.0
            self._d_acc = self._d_cru = 0.0
        else:
            a = accel if math.isfinite(accel) else 1e18
            d = decel if math.isfinite(decel) else 1e18

            # 거리 제약에 따른 실제 도달 가능한 최고속도
            #   d_acc = (v_p² - v_i²) / (2a),   d_dec = v_p² / (2d)
            #   d_acc + d_dec <= dist
            #   → v_p² <= 2*(a*d/(a+d))*dist + v_i²*(d/(a+d))
            coeff     = 2.0 * a * d / (a + d)
            v_peak_sq = coeff * dist + v_init ** 2 * d / (a + d)
            v_peak    = min(v_max, math.sqrt(max(0.0, v_peak_sq)))

            self.v_peak = v_peak
            self._a     = a
            self._d     = d
            self._const = False

            d_acc = (v_peak ** 2 - v_init ** 2) / (2.0 * a) if v_peak > v_init else 0.0
            d_dec = v_peak ** 2 / (2.0 * d)
            d_cru = max(0.0, dist - d_acc - d_dec)

            t_acc = (v_peak - v_init) / a if v_peak > v_init else 0.0
            t_cru = d_cru / v_peak if v_peak > 0 else 0.0
            t_dec = v_peak / d if d > 0 else 0.0

            self.t1      = t_acc
            self.t2      = t_acc + t_cru
            self.t_total = t_acc + t_cru + t_dec
            self._d_acc  = d_acc
            self._d_cru  = d_cru

    def get_position(self, elapsed: float) -> float:
        """elapsed 초 후의 이동 거리 (mm)."""
        if self.dist <= 0.0 or self.t_total <= 0.0:
            return self.dist
        elapsed = max(0.0, min(elapsed, self.t_total))

        if self._const:
            return min(self.dist, self.v_peak * elapsed)

        if elapsed <= self.t1:
            return self.v_init * elapsed + 0.5 * self._a * elapsed ** 2
        elif elapsed <= self.t2:
            return self._d_acc + self.v_peak * (elapsed - self.t1)
        else:
            dt = elapsed - self.t2
            d_up = self._d_acc + self._d_cru
            return min(self.dist,
                       d_up + self.v_peak * dt - 0.5 * self._d * dt ** 2)

    def get_velocity(self, elapsed: float) -> float:
        """elapsed 초 후의 속도 (mm/s)."""
        if self.t_total <= 0.0:
            return 0.0
        elapsed = max(0.0, min(elapsed, self.t_total))

        if self._const:
            # 상수 속도 모드: t_total 도달 전까지 v_peak 유지
            return self.v_peak if elapsed < self.t_total else 0.0

        if elapsed <= self.t1:
            return self.v_init + self._a * elapsed
        elif elapsed <= self.t2:
            return self.v_peak
        else:
            dt = elapsed - self.t2
            return max(0.0, self.v_peak - self._d * dt)


# ── TAPG DES 환경 ─────────────────────────────────────────────────────────────

class TAPGEnvironmentDES:
    """
    TAPG 기반 순수 DES 실행 환경.

    env_tapg.TAPGEnvironment 와 동일한 외부 인터페이스:
      setup(agents, t_start)
      step(sim_time)         — dt 루프 없음: 이벤트 처리 + 시각화 업데이트만
      all_done()
      tapg_stats()

    이동/회전 완료는 EDGE_DONE / ROTATE_DONE 이벤트로 사전 예약됩니다.
    시뮬레이션 시간을 원하는 만큼 빠르게 진행할 수 있습니다.
    """

    PERIODIC_INTERVAL = 0.5   # sim-seconds: WAITING 에이전트 재확인 주기

    def __init__(self, graph: PklMapGraph,
                 accel: float = INF,
                 decel: float = INF):
        """
        Parameters
        ----------
        accel : mm/s²  가속도. INF → 즉시 최고속도(상수 속도 모드)
        decel : mm/s²  감속도. INF → 즉시 정지
        """
        self.graph  = graph
        self.accel  = accel
        self.decel  = decel

        self._eq: list       = []
        self._seq            = 0
        self.agents:         Dict[int, TAPGAgent]  = {}
        self.sim_time        = 0.0
        self._last_check     = 0.0

        # TAPG
        self.G:               nx.DiGraph            = nx.DiGraph()
        self.completed_nodes: set                   = set()
        self.wait_queues:     Dict[Any, List[int]]  = {}

        # DES 전용 — 에이전트별 속도 프로파일 & 액션 시작 시각
        self._profiles:           Dict[int, SpeedProfile] = {}
        self._move_start_times:   Dict[int, float]        = {}
        self._rotate_start_times: Dict[int, float]        = {}
        self._move_tokens:        Dict[int, int]          = {}   # stale 이벤트 방지용
        self._move_offsets:       Dict[int, float]        = {}   # 패스스루 시 거리 오프셋
        self._chain_carry:        Dict[int, float]        = {}   # 패스스루 도착 속도

    # ── 초기화 ────────────────────────────────────────────────────────────────

    def setup(self, agents: list, t_start: float = 0.0):
        """
        agents: 외부에서 생성한 TAPGAgent 객체 목록.
        같은 객체를 시각화쪽에서 참조하면 위치/상태가 자동으로 반영됩니다.
        """
        self.agents = {a.id: a for a in agents}
        self.completed_nodes.clear()
        self.wait_queues.clear()
        self.G.clear()
        self._eq        = []
        self._seq       = 0
        self.sim_time   = t_start
        self._last_check = t_start
        self._profiles.clear()
        self._move_start_times.clear()
        self._rotate_start_times.clear()
        self._move_tokens.clear()
        self._move_offsets.clear()
        self._chain_carry.clear()

        raw_paths = [a.raw_path for a in agents]
        agent_ids = [a.id      for a in agents]

        # 에이전트 상태 초기화 + 시작 위치 설정
        for a in agents:
            a.path_idx = 0
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

    # ── TAPG 구성 (env_tapg.py 와 동일) ───────────────────────────────────────

    def _build_tapg(self, raw_paths: list, agent_ids: list):
        """
        Automod prioritized solver 의 construct_temporal_graph 와 동일한 논리.
        노드 키: (state_id, agv_id, cbs_start_time)
        """
        # ① 모든 상태 노드 + 같은 에이전트 내 순차 엣지
        for aid, path in zip(agent_ids, raw_paths):
            for k, (sid, t) in enumerate(path):
                nk = (sid, aid, t)
                duration = (float('inf') if k == len(path) - 1
                            else path[k + 1][1] - t)
                self.G.add_node(nk, agv_id=aid, start_time=t, duration=duration)
                if k > 0:
                    prev_sid, prev_t = path[k - 1]
                    self.G.add_edge((prev_sid, aid, prev_t), nk)

        # ② 교차-에이전트 엣지 (non-S 상태만)
        for i, (ai, pi) in enumerate(zip(agent_ids, raw_paths)):
            for j, (aj, pj) in enumerate(zip(agent_ids, raw_paths)):
                if i == j:
                    continue
                # i 의 path를 역순으로 순회 (원본 코드와 동일)
                for k1 in range(len(pi) - 1, -1, -1):
                    s1, t1 = pi[k1]
                    if s1.startswith('S,'):
                        continue
                    affect1 = self._state_affect_set(s1)
                    if not affect1:
                        continue
                    # j 의 path를 순방향으로: t2 >= t1인 non-S 상태 중
                    # 실제로 충돌하는 첫 번째 액션만 연결
                    for k2, (s2, t2) in enumerate(pj):
                        if t2 < t1:
                            continue
                        if s2.startswith('S,'):
                            continue
                        affect2 = self._state_affect_set(s2)
                        if s2 in affect1 or s1 in affect2:
                            self.G.add_edge((s1, ai, t1), (s2, aj, t2))
                            break

    # ── 메인 루프 ──────────────────────────────────────────────────────────────

    def step(self, sim_time: float):
        self.sim_time = sim_time

        # 예약 이벤트 처리 (dt 루프 없음)
        while self._eq and self._eq[0].time <= sim_time:
            ev = heapq.heappop(self._eq)
            self._process(ev)

        # 시각화용 위치 업데이트 (SpeedProfile 공식으로 O(1))
        self._update_visuals(sim_time)

        # 주기적으로 WAITING 에이전트 재확인 (event miss 방어)
        if sim_time - self._last_check >= self.PERIODIC_INTERVAL:
            self._last_check = sim_time
            self._periodic_wakeup(sim_time)

    # ── 이벤트 처리 ────────────────────────────────────────────────────────────

    def _process(self, ev: Event):
        handler = {
            'TRY_ADVANCE': self._on_try_advance,
            'EDGE_DONE':   self._on_edge_done,
            'ROTATE_DONE': self._on_rotate_done,
            'CHECK_CHAIN': self._on_check_chain,
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
            nk_s = (sid, agent.id, agent.raw_path[agent.path_idx][1])
            self._complete_node(nk_s, ev.time)
            agent.path_idx += 1

        if agent.path_idx >= len(agent.raw_path):
            agent.state = DONE
            agent.v     = 0.0
            return

        sid, cbs_t = agent.raw_path[agent.path_idx]
        nk = (sid, agent.id, cbs_t)

        # TAPG 클레임 가능 여부 확인
        if not self._is_claimable(nk, agent.id):
            agent.state = WAITING
            agent.v     = 0.0
            for pred in self.G.predecessors(nk):
                if pred[1] != agent.id and pred not in self.completed_nodes:
                    q = self.wait_queues.setdefault(pred, [])
                    if agent.id not in q:
                        q.append(agent.id)
            return

        # 액션 실행
        if sid.startswith('M,'):
            self._start_move(agent, sid, cbs_t, ev.time)
        elif sid.startswith('R,'):
            self._start_rotate(agent, sid, cbs_t, ev.time)
        else:
            self._complete_node(nk, ev.time)
            agent.path_idx += 1
            self._schedule(ev.time, 'TRY_ADVANCE', agent.id)

    def _on_edge_done(self, ev: Event):
        """EDGE_DONE 이벤트: 이동 완료 처리."""
        agent = self.agents.get(ev.agent_id)
        if agent is None or agent.state != MOVING:
            return
        if ev.data.get('token') != self._move_tokens.get(ev.agent_id):
            return  # CHECK_CHAIN 이 이미 토큰을 갱신한 stale 이벤트
        # 목적지로 스냅
        agent.x = agent.to_x
        agent.y = agent.to_y
        # 패스스루 도착이면 carry_v > 0, 일반 감속 도착이면 0
        carry_v = self._chain_carry.pop(ev.agent_id, 0.0)
        agent.v = carry_v
        self._handle_arrival(agent, ev.time, carry_v=carry_v)

    def _on_check_chain(self, ev: Event):
        """
        CHECK_CHAIN 이벤트: 제동 시작 시점에 다음 Move 가 claimable 이면
        감속을 취소하고 v_peak 로 나머지 거리를 일정 속도로 통과합니다.

        패스스루 조건을 만족하면:
          1) 현재까지의 이동 거리를 _move_offsets 에 저장
          2) 나머지 d_dec 를 v_peak 일정 속도로 달리는 새 SpeedProfile 생성
          3) 새 토큰으로 EDGE_DONE 재예약
          4) _chain_carry 에 v_peak 저장 (도착 후 체이닝)
        """
        agent = self.agents.get(ev.agent_id)
        if agent is None or agent.state != MOVING:
            return
        if ev.data.get('token') != self._move_tokens.get(ev.agent_id):
            return  # stale

        if not self._next_move_claimable(agent):
            return  # 아직 클레임 불가 — 원래 EDGE_DONE 대로 감속

        profile = self._profiles[agent.id]
        v_peak  = profile.v_peak
        if v_peak <= 0:
            return

        # 원래 감속 거리·시간 계산
        d_dec    = v_peak ** 2 / (2.0 * self.decel)
        t_remain = d_dec / v_peak   # = v_peak / (2 * decel)

        # 제동 시작 시점까지의 이동 거리를 오프셋으로 저장
        t0 = self._move_start_times[agent.id]
        self._move_offsets[agent.id] = profile.get_position(ev.time - t0)

        # 일정 속도 v_peak 로 d_dec 를 달리는 새 프로파일
        new_profile = SpeedProfile(d_dec, v_peak, v_peak, INF, INF)
        self._profiles[agent.id]         = new_profile
        self._move_start_times[agent.id] = ev.time
        agent.v = v_peak

        # 새 토큰으로 EDGE_DONE 재예약 (기존 EDGE_DONE 은 stale 처리)
        tok = self._move_tokens.get(agent.id, 0) + 1
        self._move_tokens[agent.id] = tok
        self._chain_carry[agent.id] = v_peak   # 도착 후 체이닝용 속도
        self._schedule(ev.time + t_remain, 'EDGE_DONE', agent.id, token=tok)

    def _on_rotate_done(self, ev: Event):
        """ROTATE_DONE 이벤트: 회전 완료 처리."""
        agent = self.agents.get(ev.agent_id)
        if agent is None or agent.state != ROTATING:
            return
        agent.theta = agent.to_theta
        agent.v     = 0.0
        self._handle_arrival(agent, ev.time, carry_v=0.0)

    # ── 액션 시작 ─────────────────────────────────────────────────────────────

    def _start_move(self, agent: TAPGAgent, sid: str,
                    cbs_t: float, cur_time: float,
                    v_init: float = None):
        """
        이동 시작: SpeedProfile 을 계산하고 EDGE_DONE 을 예약합니다.
        v_init: 체이닝 시 이전 속도. None 이면 accel 설정에 따라 0 또는 max_speed.
        """
        parts        = sid.split(',')
        from_n, to_n = parts[1], parts[2]
        edge         = self.graph.get_edge(from_n, to_n)
        if edge is None:
            nk = (sid, agent.id, cbs_t)
            self._complete_node(nk, cur_time)
            agent.path_idx += 1
            self._schedule(cur_time, 'TRY_ADVANCE', agent.id)
            return

        fn = self.graph.nodes[from_n]
        tn = self.graph.nodes[to_n]

        agent.from_x      = fn.x
        agent.from_y      = fn.y
        agent.to_x        = tn.x
        agent.to_y        = tn.y
        agent.theta       = edge.angle
        agent.max_speed   = edge.max_speed
        agent.edge_length = edge.length
        agent._tapg_node  = (sid, agent.id, cbs_t)
        agent.state       = MOVING

        if v_init is None:
            v_init = 0.0 if math.isfinite(self.accel) else edge.max_speed
        else:
            v_init = min(v_init, edge.max_speed)

        agent.v = v_init

        profile = SpeedProfile(edge.length, v_init, edge.max_speed,
                               self.accel, self.decel)
        self._profiles[agent.id]         = profile
        self._move_start_times[agent.id] = cur_time
        self._move_offsets[agent.id]     = 0.0

        # 토큰 발급 — stale 이벤트 방지
        tok = self._move_tokens.get(agent.id, 0) + 1
        self._move_tokens[agent.id] = tok

        # EDGE_DONE 을 이동 완료 시각에 예약
        self._schedule(cur_time + profile.t_total, 'EDGE_DONE', agent.id, token=tok)

        # 유한 감속 + 감속 구간이 존재하면 제동 시작 시점에 연속 주행 체크
        if math.isfinite(self.decel) and profile.t2 < profile.t_total:
            self._schedule(cur_time + profile.t2, 'CHECK_CHAIN', agent.id, token=tok)

    def _start_rotate(self, agent: TAPGAgent, sid: str,
                      cbs_t: float, cur_time: float):
        """회전 시작: ROTATE_DONE 을 예약합니다."""
        parts    = sid.split(',')
        from_deg = float(parts[2])
        to_deg   = float(parts[3])

        # 최단 경로 방향의 부호 있는 각도
        diff_ccw = (to_deg - from_deg) % 360
        if diff_ccw <= 180:
            signed_diff = math.radians(diff_ccw)    # CCW (+)
        else:
            signed_diff = math.radians(diff_ccw - 360)  # CW (-)

        from_rad = math.radians(from_deg)

        agent.from_theta  = from_rad
        agent.to_theta    = from_rad + signed_diff
        agent.theta       = from_rad
        agent.v           = 0.0
        agent.angle_total = abs(signed_diff)
        agent._tapg_node  = (sid, agent.id, cbs_t)
        agent.state       = ROTATING

        duration = agent.angle_total / ANGULAR_SPEED if ANGULAR_SPEED > 0 else 0.0
        self._rotate_start_times[agent.id] = cur_time

        # ROTATE_DONE 을 회전 완료 시각에 예약
        self._schedule(cur_time + duration, 'ROTATE_DONE', agent.id)

    # ── 시각화 업데이트 (dt 루프 없음) ────────────────────────────────────────

    def _update_visuals(self, sim_time: float):
        """
        모든 MOVING/ROTATING 에이전트의 위치/각도를
        SpeedProfile 공식으로 갱신합니다. O(agents) — 루프 없음.
        """
        for agent in self.agents.values():
            if agent.state == MOVING:
                profile = self._profiles.get(agent.id)
                t0      = self._move_start_times.get(agent.id, sim_time)
                if profile is None or agent.edge_length <= 0:
                    continue
                elapsed = sim_time - t0
                d       = self._move_offsets.get(agent.id, 0.0) + profile.get_position(elapsed)
                frac    = max(0.0, min(1.0, d / agent.edge_length))
                agent.x = agent.from_x + (agent.to_x - agent.from_x) * frac
                agent.y = agent.from_y + (agent.to_y - agent.from_y) * frac
                agent.v = profile.get_velocity(elapsed)

            elif agent.state == ROTATING:
                t0 = self._rotate_start_times.get(agent.id, sim_time)
                elapsed = sim_time - t0
                if agent.angle_total > 0:
                    frac = min(1.0, ANGULAR_SPEED * elapsed / agent.angle_total)
                    agent.theta = (agent.from_theta
                                   + (agent.to_theta - agent.from_theta) * frac)

    # ── 주기적 재확인 ──────────────────────────────────────────────────────────

    def _periodic_wakeup(self, sim_time: float):
        """WAITING 에이전트 중 선행 조건이 해소된 에이전트를 깨웁니다."""
        for agent in list(self.agents.values()):
            if agent.state != WAITING:
                continue
            if agent.path_idx >= len(agent.raw_path):
                agent.state = DONE
                continue
            sid, cbs_t = agent.raw_path[agent.path_idx]
            nk = (sid, agent.id, cbs_t)
            if self._is_claimable(nk, agent.id):
                agent.state = IDLE
                self._schedule(sim_time, 'TRY_ADVANCE', agent.id)

    # ── 완료 처리 ──────────────────────────────────────────────────────────────

    def _complete_node(self, nk: tuple, cur_time: float):
        """TAPG 노드를 완료 처리하고 대기 중인 에이전트를 깨웁니다."""
        self.completed_nodes.add(nk)
        for wid in self.wait_queues.pop(nk, []):
            wa = self.agents.get(wid)
            if wa and wa.state == WAITING:
                wa.state = IDLE
                self._schedule(cur_time + 1e-9, 'TRY_ADVANCE', wid)

    def _handle_arrival(self, agent: TAPGAgent, sim_time: float,
                        carry_v: float):
        """에이전트가 목적지(또는 회전 목표)에 도달했을 때 호출됩니다."""
        nk               = agent._tapg_node
        agent._tapg_node = None
        agent.path_idx  += 1
        agent.state      = IDLE
        if nk:
            self._complete_node(nk, sim_time)

        # 유한 가속도 모드 + Move 완료 시 연속 주행 시도
        if math.isfinite(self.accel) and carry_v > 0 and nk and nk[0].startswith('M,'):
            if self._try_chain_move(agent, carry_v, sim_time):
                return

        agent.v = 0.0
        self._schedule(sim_time, 'TRY_ADVANCE', agent.id)

    # ── 연속 주행 헬퍼 ────────────────────────────────────────────────────────

    def _peek_next_move(self, agent: TAPGAgent):
        """현재 path_idx 이후의 첫 번째 M 상태 (S 상태 건너뜀) 반환."""
        idx = agent.path_idx + 1
        while idx < len(agent.raw_path):
            sid, t = agent.raw_path[idx]
            if sid.startswith('S,'):
                idx += 1
                continue
            return (sid, t) if sid.startswith('M,') else None
        return None

    def _next_move_claimable(self, agent: TAPGAgent) -> bool:
        """다음 M 상태가 현재 이미 claimable 인지 확인."""
        nm = self._peek_next_move(agent)
        if nm is None:
            return False
        nk = (nm[0], agent.id, nm[1])
        return self._is_claimable(nk, agent.id)

    def _try_chain_move(self, agent: TAPGAgent,
                        carry_v: float, sim_time: float) -> bool:
        """
        도달 직후 S 상태를 inline으로 처리하고,
        다음 M이 claimable이면 멈추지 않고 바로 _start_move 진입.
        """
        while (agent.path_idx < len(agent.raw_path)
               and agent.raw_path[agent.path_idx][0].startswith('S,')):
            sid, t = agent.raw_path[agent.path_idx]
            nid = self._node_of(sid)
            if nid and nid in self.graph.nodes:
                n = self.graph.nodes[nid]
                agent.x, agent.y = n.x, n.y
            agent.theta = self._heading_of(sid)
            self._complete_node((sid, agent.id, t), sim_time)
            agent.path_idx += 1

        if agent.path_idx >= len(agent.raw_path):
            agent.state = DONE
            agent.v     = 0.0
            return True

        next_sid, next_t = agent.raw_path[agent.path_idx]
        next_nk = (next_sid, agent.id, next_t)

        if next_sid.startswith('M,') and self._is_claimable(next_nk, agent.id):
            self._start_move(agent, next_sid, next_t, sim_time, v_init=carry_v)
            return True

        return False

    # ── TAPG 클레임 확인 ───────────────────────────────────────────────────────

    def _is_claimable(self, nk: tuple, agent_id: int) -> bool:
        """다른 에이전트의 모든 선행 TAPG 노드가 완료됐으면 True."""
        if nk not in self.G:
            return True
        for pred in self.G.predecessors(nk):
            if pred[1] != agent_id and pred not in self.completed_nodes:
                return False
        return True

    # ── 헬퍼 ──────────────────────────────────────────────────────────────────

    def _get_state_obj(self, state_id: str):
        if state_id.startswith('M,') or state_id.startswith('R,'):
            return self.graph.move_states_raw.get(state_id)
        if state_id.startswith('S,'):
            return self.graph.stop_states_raw.get(state_id)
        return None

    def _state_affect_set(self, state_id: str) -> set:
        """state_id 의 affect_state set 반환."""
        obj = self._get_state_obj(state_id)
        return set(getattr(obj, 'affect_state', [])) if obj else set()

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

    def all_done(self) -> bool:
        return all(a.state == DONE for a in self.agents.values())

    def tapg_stats(self) -> dict:
        total     = self.G.number_of_nodes()
        done      = len(self.completed_nodes)
        edges     = self.G.number_of_edges()
        seq_edges = sum(1 for u, v in self.G.edges()
                        if self.G.nodes[u].get('agv_id') == self.G.nodes[v].get('agv_id'))
        return {
            'total':       total,
            'done':        done,
            'remaining':   total - done,
            'edges':       edges,
            'cross_edges': edges - seq_edges,
        }
