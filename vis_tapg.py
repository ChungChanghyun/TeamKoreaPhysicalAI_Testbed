"""
vis_tapg.py — TAPG (Temporal Action Precedence Graph) visualizer for pkl maps.

CBS 플랜을 action-dependency DAG로 변환하여 실행합니다.
각 에이전트는 cross-agent 선행 의존 작업이 모두 완료된 시점에 다음 액션을
수행하므로 실제 주행 시간 차이가 있어도 충돌이 발생하지 않습니다.

Keys
────
  SPACE   Start / Pause
  R       Reset
  F       Toggle stop-region overlay
  M       Toggle move-region overlay
  S       Shuffle — BFS 경로로 재시작 (DES 모드)
  C       Plan (CBS+TAPG)
  N / P   Add / Remove one vehicle
  +/-     Sim speed up / down
  [/]     Accel preset down / up
  ,/.     Decel preset down / up

Agent state colours
───────────────────
  MOVING   — colored rect, speed tint
  ROTATING — yellow rect (제자리 회전 중)
  WAITING  — colored rect + orange outer ring (TAPG 선행 조건 대기)
  IDLE     — colored rect, dimmed
  DONE     — dark rect
"""
from __future__ import annotations
import sys, os, math, random, json
import pygame

SCENARIO_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'last_scenario.json')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pkl_loader  import PklMapGraph
from env_cont    import AGENT_COLORS
from env_des     import DESAgent, DESEnvironment, IDLE as DES_IDLE, \
                        MOVING as DES_MOVING, WAITING as DES_WAITING, DONE as DES_DONE
from env_tapg    import TAPGAgent, TAPGEnvironment, \
                        IDLE, MOVING, ROTATING, WAITING, DONE
from env_tapg_des import TAPGEnvironmentDES
from planner     import PklPlanner
from vis_pkl     import (_bfs_path, _shapely_to_screen,
                          COL_STOP_FILL, COL_STOP_BORDER,
                          COL_MOVE_FILL, COL_MOVE_BORDER,
                          MAX_VEHICLES)
from vis_cont    import (_draw_rotated_rect, _draw_arrow, _speed_tint, Button,
                          BG, SIDE_BG, COL_EDGE, COL_EDGE_A, COL_NODE, COL_NODE_B,
                          COL_PORT, COL_TEXT, COL_DIM, COL_BTN,
                          COL_DANGER, COL_WHITE, COL_YELLOW,
                          NODE_R_MM, MAP_W, SIDE_W, WIN_W, WIN_H, MAP_PAD, FPS,
                          SIM_SPEEDS, SIM_SPEED_LABELS, DEFAULT_SPEED_IDX)

COL_WAITING  = (255, 165,   0)   # orange
COL_ROTATING = (255, 220,  60)   # yellow

# 가속/감속 프리셋 (mm/s²) — 낮을수록 느림, 높을수록 빠름, inf = 즉시
import math as _math
KINEMATIC_PRESETS = [200.0, 500.0, 1000.0, 2000.0, 5000.0, _math.inf]
KINEMATIC_LABELS  = ['200', '500', '1000', '2000', '5000', '∞']


class TAPGSimulator:
    """
    Pygame visualizer — DES mode (no plan) → TAPG mode (after CBS plan).

    DES mode : BFS 경로 + DESEnvironment (충돌 회피 없음, 예약 기반)
    TAPG mode: CBS 경로 + TAPGEnvironment (action-dependency 기반 충돌 방지)
    """

    def __init__(self, graph: PklMapGraph, n_agents: int = 3,
                 pkl_path: str = None):
        self._show_stop_regions = True
        self._show_move_regions = False
        self._region_surf_dirty = True
        self._plan_status       = ''
        self._mode              = 'des'   # 'des' | 'tapg'
        self._cbs_markers       = []

        self.graph     = graph
        self._next_id  = 0
        self._n_target = max(1, min(n_agents, MAX_VEHICLES))

        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption('TAPG MAPF Simulator')
        self.clock  = pygame.time.Clock()

        self.font_s = pygame.font.SysFont('Consolas', 13)
        self.font_m = pygame.font.SysFont('Consolas', 15)
        self.font_b = pygame.font.SysFont('Consolas', 16, bold=True)

        self.sim_time  = 0.0
        self.running   = False
        self.spd_idx   = DEFAULT_SPEED_IDX
        self.accel_idx = len(KINEMATIC_PRESETS) - 1   # default: inf
        self.decel_idx = len(KINEMATIC_PRESETS) - 1   # default: inf

        self._build_transform()
        self._build_buttons()

        # DES 초기 상태
        self._des_agents = self._create_des_agents(self._n_target)
        self._des_env    = self._make_des_env(self._des_agents, 0.0)
        self.agents      = self._des_agents  # 현재 활성 에이전트 목록

        # TAPG 환경 (CBS 플랜 후 초기화)
        self._tapg_env: TAPGEnvironment | None = None
        self._tapg_agents: list = []
        self._use_des_engine: bool = False   # T 키로 토글: False=TimeStep, True=DES

        # CBS planner
        self._planner: PklPlanner | None = None
        if pkl_path:
            try:
                print('Initializing CBS planner...')
                self._planner    = PklPlanner(pkl_path)
                self._plan_status = 'Planner ready'
                print('Planner ready.')
            except Exception as e:
                self._plan_status = f'Planner error: {e}'
                print(f'Planner init failed: {e}')

    # ── 좌표 변환 ─────────────────────────────────────────────────────────────

    def _build_transform(self):
        x0, y0, x1, y1 = self.graph.bbox
        mw, mh = x1 - x0, y1 - y0
        avail_w = MAP_W - 2 * MAP_PAD
        avail_h = WIN_H - 2 * MAP_PAD
        self._scale = min(avail_w / mw, avail_h / mh) if mw > 0 and mh > 0 else 0.05
        self._ox = MAP_W / 2.0 - (x0 + x1) / 2.0 * self._scale
        self._oy = WIN_H / 2.0 + (y0 + y1) / 2.0 * self._scale

    def to_screen(self, x_mm: float, y_mm: float) -> tuple:
        return (int(self._ox + x_mm * self._scale),
                int(self._oy - y_mm * self._scale))

    def px(self, mm: float) -> float:
        return mm * self._scale

    # ── 버튼 레이아웃 ─────────────────────────────────────────────────────────

    def _build_buttons(self):
        sx = MAP_W + 10
        sw = SIDE_W - 20
        y  = 12
        H, G = 30, 6

        self.btn_start = Button((sx, y, sw, H), '▶ Start / Pause')
        y += H + G
        self.btn_reset = Button((sx, y, sw, H), '↺  Reset Time', base=COL_DANGER)
        y += H + G + 6

        bw = (sw + 2) // 4
        self.btns_speed = []
        for row_i, row in enumerate([SIM_SPEED_LABELS[:4], SIM_SPEED_LABELS[4:]]):
            for col_i, lbl in enumerate(row):
                idx = row_i * 4 + col_i
                b   = Button((sx + col_i * bw, y, bw - 2, H - 4), lbl, toggle=True)
                b.active = (idx == self.spd_idx)
                self.btns_speed.append(b)
            y += H - 4 + 2
        y += G + 4

        half = (sw - G) // 2
        self.btn_stop_reg = Button((sx,            y, half, H - 6), 'F: Stop Rgn', toggle=True)
        self.btn_move_reg = Button((sx + half + G, y, half, H - 6), 'M: Move Rgn', toggle=True)
        self.btn_stop_reg.active = self._show_stop_regions
        self.btn_move_reg.active = self._show_move_regions
        y += H - 6 + G

        bw2 = 30
        self.btn_del_agent = Button((sx,            y, bw2,           H - 6), '-', base=COL_DANGER)
        self.btn_add_agent = Button((sx + sw - bw2, y, bw2,           H - 6), '+')
        self._count_rect   = pygame.Rect(sx + bw2 + G, y, sw - 2 * bw2 - 2 * G, H - 6)
        y += H - 6 + G

        self.btn_shuffle = Button((sx, y, sw, H - 6), 'S: Shuffle (DES)')
        y += H - 6 + G
        self.btn_plan    = Button((sx, y, sw, H - 6), 'C: Plan (CBS+TAPG)')
        y += H - 6 + G

        self._info_y    = y + 4
        self.all_buttons = ([self.btn_start, self.btn_reset, *self.btns_speed,
                              self.btn_stop_reg, self.btn_move_reg,
                              self.btn_del_agent, self.btn_add_agent,
                              self.btn_shuffle, self.btn_plan])

    # ── DES 에이전트 팩토리 ───────────────────────────────────────────────────

    def _fresh_color(self, agent_id: int) -> tuple:
        return AGENT_COLORS[agent_id % len(AGENT_COLORS)]

    def _create_des_agent(self, agent_id: int, occupied: set) -> DESAgent | None:
        free = [n for n in self.graph.nodes if n not in occupied]
        if not free:
            return None
        random.shuffle(free)
        for start in free:
            path = _bfs_path(self.graph, start)
            if path:
                return DESAgent(agent_id, self._fresh_color(agent_id), path)
        return None

    def _create_des_agents(self, n: int) -> list:
        occupied, agents = set(), []
        for _ in range(n):
            a = self._create_des_agent(self._next_id, occupied)
            if a is None:
                break
            occupied.add(a.node_path[0])
            self._next_id += 1
            agents.append(a)
        return agents

    def _make_tapg_env(self):
        """_use_des_engine 플래그에 따라 TAPGEnvironment 또는 TAPGEnvironmentDES 반환."""
        cls = TAPGEnvironmentDES if self._use_des_engine else TAPGEnvironment
        return cls(
            self.graph,
            accel=KINEMATIC_PRESETS[self.accel_idx],
            decel=KINEMATIC_PRESETS[self.decel_idx],
        )

    def _make_des_env(self, agents: list, t_start: float) -> DESEnvironment:
        env = DESEnvironment(self.graph)
        for a in agents:
            a.path_idx = 0
            a.state    = DES_IDLE
            a.reserved = []
            env.add_agent(a, t_start=t_start)
        return env

    # ── 업데이트 ──────────────────────────────────────────────────────────────

    def update(self, dt_real: float):
        if not self.running:
            return
        self.sim_time += dt_real * SIM_SPEEDS[self.spd_idx]

        if self._mode == 'tapg':
            self._tapg_env.step(self.sim_time)
            if self._tapg_env.all_done():
                self.running = False
        else:
            self._des_env.step(self.sim_time)
            for agent in self._des_agents:
                if agent.state == DES_DONE:
                    self._reassign_des_agent(agent)

    # ── 리셋 ──────────────────────────────────────────────────────────────────

    def _reset(self):
        self.sim_time = 0.0
        self.running  = False
        if self._mode == 'tapg':
            self._tapg_env = self._make_tapg_env()
            self._tapg_env.setup(self._tapg_agents, t_start=0.0)
        else:
            self._des_env = self._make_des_env(self._des_agents, 0.0)

    # ── DES 에이전트 재배정 ───────────────────────────────────────────────────

    def _reassign_des_agent(self, agent: DESAgent):
        cur  = agent.cur_node
        path = _bfs_path(self.graph, cur)
        if not path:
            path = _bfs_path(self.graph, random.choice(list(self.graph.nodes)))
        if not path:
            return
        self._des_env.remove_agent(agent.id)
        agent.node_path = path
        agent.path_idx  = 0
        agent.state     = DES_IDLE
        agent.reserved  = []
        self._des_env.add_agent(agent, t_start=self.sim_time)

    # ── Shuffle ───────────────────────────────────────────────────────────────

    def _shuffle_paths(self):
        self._mode        = 'des'
        self._cbs_markers = []
        self._next_id     = 0
        all_nodes = list(self.graph.nodes)
        random.shuffle(all_nodes)
        occupied = set()
        for agent in self._des_agents:
            free = [n for n in all_nodes if n not in occupied]
            if not free:
                free = all_nodes
            for start in free:
                path = _bfs_path(self.graph, start)
                if path:
                    agent.node_path = path
                    occupied.add(start)
                    break
        self._des_env = self._make_des_env(self._des_agents, self.sim_time)
        self.agents   = self._des_agents

    # ── CBS + TAPG 플랜 ───────────────────────────────────────────────────────

    def _plan_cbs(self):
        if self._planner is None:
            self._plan_status = 'No planner (pkl_path needed)'
            return

        n = self._n_target
        self._plan_status = f'Planning {n} agents...'
        self.render()

        try:
            node_paths = self._planner.plan_random(n_agents=n, max_time=1.0)
        except Exception as e:
            self._plan_status = f'Plan error: {e}'
            print(f'CBS error: {e}')
            return

        if node_paths is None:
            self._plan_status = 'No solution found'
            starts = getattr(self._planner, '_last_starts', None)
            goals  = getattr(self._planner, '_last_goals',  None)
            print('No solution found.')
            if starts and goals:
                for i, (s, g) in enumerate(zip(starts, goals)):
                    print(f'  Agent {i+1}: {s} -> {g}')
            return

        raw_paths = getattr(self._planner, '_last_raw_paths', None)
        if raw_paths is None or len(raw_paths) != len(node_paths):
            self._plan_status = 'Internal error: raw_paths missing'
            return

        # TAPG 에이전트 생성
        self._tapg_agents = []
        self._cbs_markers = []
        for i, (raw, node_path) in enumerate(zip(raw_paths, node_paths)):
            color = self._fresh_color(i)
            a     = TAPGAgent(i, color, raw)
            self._tapg_agents.append(a)
            if node_path:
                self._cbs_markers.append((node_path[0], node_path[-1], color))

        # TAPG 환경 구성 — 같은 객체를 env와 시각화 양쪽에서 공유
        self._tapg_env = self._make_tapg_env()
        self._tapg_env.setup(self._tapg_agents, t_start=self.sim_time)

        # 초기 TRY_ADVANCE 이벤트를 즉시 처리 (paused 상태에서도 첫 tick 실행)
        self._tapg_env.step(self.sim_time)

        # TAPG 통계 출력
        stats = self._tapg_env.tapg_stats()
        print(f'TAPG: {stats["total"]} nodes, {stats["cross_edges"]} cross-agent edges')

        self._mode        = 'tapg'
        self.agents       = self._tapg_agents
        self._plan_status = (f'TAPG ok: {len(self._tapg_agents)} agents, '
                             f'{stats["cross_edges"]} dep edges')
        self.running      = False

        # 시나리오 저장 (재현용)
        self._save_scenario()

    # ── 시나리오 저장 / 로드 ──────────────────────────────────────────────────

    def _save_scenario(self):
        if self._planner is None:
            return
        starts = getattr(self._planner, '_last_starts', None)
        goals  = getattr(self._planner, '_last_goals',  None)
        if not starts or not goals:
            return
        scenario = {'starts': starts, 'goals': goals}
        with open(SCENARIO_FILE, 'w') as f:
            json.dump(scenario, f, indent=2)
        print(f'[Scenario saved] {SCENARIO_FILE}')
        print(f'  starts: {starts}')
        print(f'  goals : {goals}')

    def _plan_fixed(self, scenario_path: str):
        """JSON 파일에서 시나리오를 읽어 그 start/goal로 CBS+TAPG 플래닝."""
        if self._planner is None:
            self._plan_status = 'No planner (pkl_path needed)'
            return
        try:
            with open(scenario_path) as f:
                sc = json.load(f)
            starts, goals = sc['starts'], sc['goals']
        except Exception as e:
            self._plan_status = f'Scenario load error: {e}'
            print(f'Scenario load failed: {e}')
            return

        n = len(starts)
        self._plan_status = f'Replaying scenario ({n} agents)...'
        self.render()

        try:
            node_paths = self._planner.plan_fixed(starts, goals, max_time=30.0)
        except Exception as e:
            self._plan_status = f'Plan error: {e}'
            print(f'CBS error: {e}')
            return

        if node_paths is None:
            self._plan_status = 'No solution found'
            starts = getattr(self._planner, '_last_starts', None)
            goals  = getattr(self._planner, '_last_goals',  None)
            print('No solution found.')
            if starts and goals:
                for i, (s, g) in enumerate(zip(starts, goals)):
                    print(f'  Agent {i+1}: {s} -> {g}')
            return

        raw_paths = getattr(self._planner, '_last_raw_paths', None)
        if raw_paths is None or len(raw_paths) != len(node_paths):
            self._plan_status = 'Internal error: raw_paths missing'
            return

        self._tapg_agents = []
        self._cbs_markers = []
        for i, (raw, node_path) in enumerate(zip(raw_paths, node_paths)):
            color = self._fresh_color(i)
            a     = TAPGAgent(i, color, raw)
            self._tapg_agents.append(a)
            if node_path:
                self._cbs_markers.append((node_path[0], node_path[-1], color))

        self._tapg_env = self._make_tapg_env()
        self._tapg_env.setup(self._tapg_agents, t_start=self.sim_time)
        self._tapg_env.step(self.sim_time)

        stats = self._tapg_env.tapg_stats()
        print(f'TAPG: {stats["total"]} nodes, {stats["cross_edges"]} cross-agent edges')

        self._mode        = 'tapg'
        self.agents       = self._tapg_agents
        self._plan_status = (f'Replay ok: {n} agents, '
                             f'{stats["cross_edges"]} dep edges')
        self.running      = False

    # ── 에이전트 수 조정 ──────────────────────────────────────────────────────

    def _add_agent(self):
        if self._mode == 'tapg':
            # TAPG 모드: 다음 플래닝에 쓸 대수만 변경
            self._n_target = min(self._n_target + 1, MAX_VEHICLES)
            return
        if len(self._des_agents) >= MAX_VEHICLES:
            return
        occupied = {a.node_path[0] for a in self._des_agents}
        a = self._create_des_agent(self._next_id, occupied)
        if a:
            self._des_agents.append(a)
            self._next_id += 1
            self._des_env.add_agent(a, t_start=self.sim_time)
            self.agents = self._des_agents

    def _remove_agent(self):
        if self._mode == 'tapg':
            # TAPG 모드: 다음 플래닝에 쓸 대수만 변경
            self._n_target = max(self._n_target - 1, 1)
            return
        if self._des_agents:
            a = self._des_agents.pop()
            self._des_env.remove_agent(a.id)
            self.agents = self._des_agents

    # ── 속도 ──────────────────────────────────────────────────────────────────

    def _set_speed(self, idx: int):
        self.spd_idx = idx
        for i, b in enumerate(self.btns_speed):
            b.active = (i == idx)

    # ── 이벤트 ────────────────────────────────────────────────────────────────

    def handle_events(self):
        mpos = pygame.mouse.get_pos()
        for b in self.all_buttons:
            b.update_hover(mpos)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()

            if event.type == pygame.KEYDOWN:
                k = event.key
                if k == pygame.K_SPACE:
                    self.running = not self.running
                elif k == pygame.K_r:
                    self._reset()
                elif k in (pygame.K_EQUALS, pygame.K_PLUS):
                    self._set_speed(min(self.spd_idx + 1, len(SIM_SPEEDS) - 1))
                elif k == pygame.K_MINUS:
                    self._set_speed(max(self.spd_idx - 1, 0))
                elif k == pygame.K_f:
                    self._toggle_stop_regions()
                elif k == pygame.K_m:
                    self._toggle_move_regions()
                elif k == pygame.K_s:
                    self._shuffle_paths()
                elif k == pygame.K_c:
                    self._plan_cbs()
                elif k == pygame.K_l:
                    self._plan_fixed(SCENARIO_FILE)
                elif k == pygame.K_n:
                    self._add_agent()
                elif k == pygame.K_p:
                    self._remove_agent()
                elif k == pygame.K_t:
                    self._use_des_engine = not self._use_des_engine
                    engine = 'DES' if self._use_des_engine else 'TimeStep'
                    self._plan_status = f'Engine: {engine} (re-plan to apply)'
                elif k == pygame.K_LEFTBRACKET:
                    self.accel_idx = max(self.accel_idx - 1, 0)
                elif k == pygame.K_RIGHTBRACKET:
                    self.accel_idx = min(self.accel_idx + 1, len(KINEMATIC_PRESETS) - 1)
                elif k == pygame.K_COMMA:
                    self.decel_idx = max(self.decel_idx - 1, 0)
                elif k == pygame.K_PERIOD:
                    self.decel_idx = min(self.decel_idx + 1, len(KINEMATIC_PRESETS) - 1)

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                pos = event.pos
                if self.btn_start.clicked(pos):
                    self.running = not self.running
                elif self.btn_reset.clicked(pos):
                    self._reset()
                elif self.btn_stop_reg.clicked(pos):
                    self._toggle_stop_regions()
                elif self.btn_move_reg.clicked(pos):
                    self._toggle_move_regions()
                elif self.btn_shuffle.clicked(pos):
                    self._shuffle_paths()
                elif self.btn_plan.clicked(pos):
                    self._plan_cbs()
                elif self.btn_add_agent.clicked(pos):
                    self._add_agent()
                elif self.btn_del_agent.clicked(pos):
                    self._remove_agent()
                else:
                    for i, b in enumerate(self.btns_speed):
                        if b.clicked(pos):
                            self._set_speed(i)

    # ── 오버레이 토글 ─────────────────────────────────────────────────────────

    def _toggle_stop_regions(self):
        self._show_stop_regions  = not self._show_stop_regions
        self.btn_stop_reg.active = self._show_stop_regions
        self._region_surf_dirty  = True

    def _toggle_move_regions(self):
        self._show_move_regions  = not self._show_move_regions
        self.btn_move_reg.active = self._show_move_regions
        self._region_surf_dirty  = True

    # ── 지역 서피스 캐시 ──────────────────────────────────────────────────────

    def _rebuild_region_surf(self):
        surf = pygame.Surface((MAP_W, WIN_H), pygame.SRCALPHA)
        if self._show_move_regions:
            for sid, poly in self.graph.move_regions.items():
                pts = _shapely_to_screen(poly, self.to_screen)
                if len(pts) >= 3:
                    pygame.draw.polygon(surf, COL_MOVE_FILL,   pts)
                    pygame.draw.polygon(surf, COL_MOVE_BORDER, pts, 1)
        if self._show_stop_regions:
            for sid, poly in self.graph.stop_regions.items():
                pts = _shapely_to_screen(poly, self.to_screen)
                if len(pts) >= 3:
                    pygame.draw.polygon(surf, COL_STOP_FILL,   pts)
                    pygame.draw.polygon(surf, COL_STOP_BORDER, pts, 1)
        self._region_surf       = surf
        self._region_surf_dirty = False

    # ── 렌더 ──────────────────────────────────────────────────────────────────

    def render(self):
        self.screen.fill(BG)
        pygame.draw.rect(self.screen, SIDE_BG, pygame.Rect(MAP_W, 0, SIDE_W, WIN_H))

        self._draw_edges()

        if self._region_surf_dirty:
            self._rebuild_region_surf()
        if self._show_stop_regions or self._show_move_regions:
            self.screen.blit(self._region_surf, (0, 0))

        self._draw_agent_paths()
        self._draw_nodes()
        if self._cbs_markers:
            self._draw_cbs_markers()
        self._draw_agents()
        self._draw_sidebar()
        pygame.display.flip()

    # ── 맵 그리기 ─────────────────────────────────────────────────────────────

    def _draw_edges(self):
        for (fn, tn), edge in self.graph.edges.items():
            fn_n = self.graph.nodes[fn]
            tn_n = self.graph.nodes[tn]
            p0   = self.to_screen(fn_n.x, fn_n.y)
            p1   = self.to_screen(tn_n.x, tn_n.y)
            pygame.draw.line(self.screen, COL_EDGE, p0, p1, 2)
            mx      = p0[0] + (p1[0] - p0[0]) * 0.6
            my      = p0[1] + (p1[1] - p0[1]) * 0.6
            scr_ang = math.atan2(-(tn_n.y - fn_n.y), tn_n.x - fn_n.x)
            _draw_arrow(self.screen, COL_EDGE_A, mx, my, scr_ang, size=7)

    def _draw_nodes(self):
        r_px = max(5, min(int(self.px(NODE_R_MM)), 10))
        for nid, node in self.graph.nodes.items():
            sx, sy  = self.to_screen(node.x, node.y)
            is_port = nid in self.graph.ports.values()
            col     = COL_PORT if is_port else COL_NODE
            pygame.draw.circle(self.screen, col,        (sx, sy), r_px)
            pygame.draw.circle(self.screen, COL_NODE_B, (sx, sy), r_px, 1)
            lbl = self.font_s.render(nid, True, COL_DIM)
            self.screen.blit(lbl, (sx + r_px + 2, sy - 7))

    def _draw_agent_paths(self):
        surf       = pygame.Surface((MAP_W, WIN_H), pygame.SRCALPHA)
        is_tapg    = (self._mode == 'tapg')
        line_alpha = 160 if is_tapg else 80
        node_alpha = 200 if is_tapg else 100
        line_width = 3   if is_tapg else 2

        for agent in self.agents:
            path = agent.node_path
            pts  = [self.to_screen(self.graph.nodes[n].x, self.graph.nodes[n].y)
                    for n in path if n in self.graph.nodes]
            if len(pts) < 2:
                continue

            col_line = (*agent.color, line_alpha)
            col_node = (*agent.color, node_alpha)
            pygame.draw.lines(surf, col_line, False, pts, line_width)

            for i in range(len(pts) - 1):
                x0, y0 = pts[i]
                x1, y1 = pts[i + 1]
                mx = x0 + (x1 - x0) * 0.6
                my = y0 + (y1 - y0) * 0.6
                ang = math.atan2(y1 - y0, x1 - x0)
                size = 6
                cos_a, sin_a = math.cos(ang), math.sin(ang)
                tip = (mx + cos_a * size,       my + sin_a * size)
                bl  = (mx - cos_a * size * 0.5 + (-sin_a) * size * 0.6,
                       my - sin_a * size * 0.5 + cos_a    * size * 0.6)
                br  = (mx - cos_a * size * 0.5 - (-sin_a) * size * 0.6,
                       my - sin_a * size * 0.5 - cos_a    * size * 0.6)
                pygame.draw.polygon(surf, col_line, [tip, bl, br])

            for pt in pts[1:-1]:
                pygame.draw.circle(surf, col_node, pt, 4)

        self.screen.blit(surf, (0, 0))

    # ── 에이전트 그리기 ───────────────────────────────────────────────────────

    def _draw_agents(self):
        w_px  = max(self.px(self.graph.vehicle_width),  8.0)
        l_px  = max(self.px(self.graph.vehicle_length), 12.0)
        v_max = max((e.max_speed for e in self.graph.edges.values()), default=1000.0)

        for agent in self.agents:
            sx, sy  = self.to_screen(agent.x, agent.y)
            scr_ang = math.degrees(-agent.theta)

            state = agent.state

            if state == DONE:
                fill   = tuple(c // 3 for c in agent.color)
                border = COL_DIM
            elif state == WAITING:
                fill   = agent.color
                border = COL_WHITE
            elif state == ROTATING:
                fill   = COL_ROTATING
                border = COL_WHITE
            elif state == MOVING:
                vf   = min(agent.v / v_max, 1.0) if v_max > 0 else 0.0
                fill = _speed_tint(agent.color, vf)
                border = COL_WHITE
            else:  # IDLE
                fill   = tuple(max(0, c - 40) for c in agent.color)
                border = COL_DIM

            _draw_rotated_rect(self.screen, fill, sx, sy,
                               l_px, w_px, scr_ang,
                               border=border, border_w=2)

            # 헤드라이트 점
            rad = math.radians(scr_ang)
            hx  = sx + math.cos(rad) * l_px / 2
            hy  = sy + math.sin(rad) * l_px / 2
            pygame.draw.circle(self.screen, (255, 255, 180),
                               (int(hx), int(hy)), max(3, int(w_px * 0.18)))

            # 상태 지시 링
            if state == WAITING:
                pygame.draw.circle(self.screen, COL_WAITING,
                                   (sx, sy), int(l_px / 2) + 5, 3)
            elif state == ROTATING:
                # 회전 진행도 아크 (각도 기반)
                frac = 0.0
                if hasattr(agent, 'angle_total') and agent.angle_total > 0:
                    traversed = abs(agent.theta - agent.from_theta)
                    frac = min(1.0, traversed / agent.angle_total)
                r_ring = int(l_px / 2) + 4
                rect   = pygame.Rect(sx - r_ring, sy - r_ring, r_ring * 2, r_ring * 2)
                pygame.draw.arc(self.screen, COL_ROTATING, rect,
                                0, frac * 2 * math.pi + 0.01, 3)
            elif state == MOVING:
                pygame.draw.circle(self.screen, agent.color,
                                   (sx + int(l_px / 2) - 2, sy - int(w_px / 2) + 2), 4)
            elif state == IDLE:
                pygame.draw.circle(self.screen, COL_DIM,
                                   (sx, sy), int(l_px / 2) + 2, 2)

            # 에이전트 ID
            lbl = self.font_s.render(str(agent.id), True,
                                     COL_WHITE if state != DONE else COL_DIM)
            self.screen.blit(lbl, lbl.get_rect(center=(sx, sy)))

    # ── CBS 마커 ──────────────────────────────────────────────────────────────

    def _draw_cbs_markers(self):
        surf = pygame.Surface((MAP_W, WIN_H), pygame.SRCALPHA)
        R    = 14
        for start_nid, goal_nid, color in self._cbs_markers:
            if start_nid in self.graph.nodes:
                n      = self.graph.nodes[start_nid]
                sx, sy = self.to_screen(n.x, n.y)
                pygame.draw.circle(surf, (*color, 180),       (sx, sy), R)
                pygame.draw.circle(surf, (255, 255, 255, 220),(sx, sy), R, 2)
                lbl = self.font_s.render('S', True, (255, 255, 255))
                surf.blit(lbl, lbl.get_rect(center=(sx, sy)))
            if goal_nid in self.graph.nodes:
                n      = self.graph.nodes[goal_nid]
                gx, gy = self.to_screen(n.x, n.y)
                pygame.draw.circle(surf, (*color, 60),  (gx, gy), R + 5)
                pygame.draw.circle(surf, (*color, 180), (gx, gy), R + 5, 2)
                pygame.draw.circle(surf, (*color, 180), (gx, gy), R - 4, 2)
                lbl = self.font_s.render('G', True, (*color, 255))
                surf.blit(lbl, lbl.get_rect(center=(gx, gy)))
        self.screen.blit(surf, (0, 0))

    # ── 사이드바 ──────────────────────────────────────────────────────────────

    def _draw_sidebar(self):
        for b in self.all_buttons:
            b.draw(self.screen, self.font_s)

        sp_lbl = self.font_s.render('Sim speed:', True, COL_DIM)
        self.screen.blit(sp_lbl, (MAP_W + 10, self.btns_speed[0].rect.y - 16))

        n_cur = len(self.agents)
        n_tgt = self._n_target
        if self._mode == 'tapg' and n_tgt != n_cur:
            count_txt = f'{n_cur} vehicles (next: {n_tgt})'
            count_col = COL_WAITING  # 변경 예정임을 주황으로 표시
        else:
            count_txt = f'{n_tgt} vehicle{"s" if n_tgt != 1 else ""}'
            count_col = COL_TEXT
        lbl = self.font_s.render(count_txt, True, count_col)
        self.screen.blit(lbl, lbl.get_rect(center=self._count_rect.center))

        y  = self._info_y
        sx = MAP_W + 10

        if self._plan_status:
            col = ((100, 220, 100) if ('ok' in self._plan_status or 'ready' in self._plan_status)
                   else (220, 100, 100) if ('error' in self._plan_status or 'No' in self._plan_status)
                   else COL_DIM)
            self.screen.blit(self.font_s.render(self._plan_status, True, col), (sx, y))
            y += 18

        def line(text, color=COL_TEXT, font=None):
            nonlocal y
            f = font or self.font_m
            self.screen.blit(f.render(text, True, color), (sx, y))
            y += f.size(text)[1] + 3

        line('── Simulation ──', font=self.font_b)
        line(f'Time   : {self.sim_time:8.2f} s')
        line(f'Speed  : {SIM_SPEED_LABELS[self.spd_idx]}')
        mode_label = 'TAPG' if self._mode == 'tapg' else 'DES'
        line(f'Mode   : {mode_label}',
             color=(100, 220, 220) if self._mode == 'tapg' else COL_DIM)
        if self._mode == 'tapg':
            engine_label = 'DES' if self._use_des_engine else 'TimeStep'
            engine_color = (100, 255, 160) if self._use_des_engine else (180, 180, 180)
            line(f'Engine : {engine_label}  [T]', color=engine_color)
        state_str = '▶ Running' if self.running else '|| Paused'
        line(state_str, color=(100, 220, 100) if self.running else (170, 170, 170))
        y += 4

        a_lbl = KINEMATIC_LABELS[self.accel_idx]
        d_lbl = KINEMATIC_LABELS[self.decel_idx]
        line(f'Accel  : {a_lbl} mm/s² [/]', color=COL_DIM, font=self.font_s)
        line(f'Decel  : {d_lbl} mm/s² ,/.', color=COL_DIM, font=self.font_s)
        y += 4

        if self._mode == 'tapg' and self._tapg_env:
            stats = self._tapg_env.tapg_stats()
            line('── TAPG Stats ──', font=self.font_b)
            line(f'Nodes  : {stats["total"]}',
                 color=COL_DIM, font=self.font_s)
            line(f'Done   : {stats["done"]}',
                 color=(100, 220, 100), font=self.font_s)
            line(f'Remain : {stats["remaining"]}',
                 color=COL_TEXT if stats["remaining"] else COL_DIM, font=self.font_s)
            line(f'XEdges : {stats["cross_edges"]}',
                 color=COL_DIM, font=self.font_s)
            y += 6

        # 에이전트별 상태
        n_moving   = sum(1 for a in self.agents if a.state == MOVING)
        n_rotating = sum(1 for a in self.agents if a.state == ROTATING)
        n_waiting  = sum(1 for a in self.agents if a.state == WAITING)
        n_done     = sum(1 for a in self.agents if a.state == DONE)
        line('── Agents ──', font=self.font_b)
        line(f'Moving  : {n_moving}',   color=COL_TEXT,    font=self.font_s)
        line(f'Rotating: {n_rotating}', color=COL_ROTATING if n_rotating else COL_DIM,
             font=self.font_s)
        line(f'Waiting : {n_waiting}',
             color=COL_WAITING if n_waiting else COL_DIM, font=self.font_s)
        line(f'Done    : {n_done}',     color=COL_DIM,     font=self.font_s)
        y += 6

        STATE_ICON = {MOVING: '>', ROTATING: '@', WAITING: '||', IDLE: 'o', DONE: 'v'}
        STATE_COL  = {MOVING: COL_TEXT, ROTATING: COL_ROTATING,
                      WAITING: COL_WAITING, IDLE: COL_DIM, DONE: COL_DIM}
        for agent in self.agents:
            pygame.draw.rect(self.screen, agent.color,
                             pygame.Rect(sx, y + 3, 11, 11), border_radius=2)
            icon = STATE_ICON.get(agent.state, '?')
            scol = STATE_COL.get(agent.state, COL_TEXT)
            self.screen.blit(
                self.font_s.render(f' A{agent.id} {icon}  v={agent.v/1000:.2f}m/s',
                                   True, scol),
                (sx + 13, y))
            y += 16
            self.screen.blit(
                self.font_s.render(
                    f'   ({agent.x/1000:.1f},{agent.y/1000:.1f})m '
                    f'th={math.degrees(agent.theta):.0f}deg',
                    True, COL_DIM),
                (sx, y))
            y += 18
        y += 4

        line('── Map ──', font=self.font_b)
        line(f'Nodes  : {len(self.graph.nodes)}', color=COL_DIM, font=self.font_s)
        line(f'Edges  : {len(self.graph.edges)}', color=COL_DIM, font=self.font_s)
        vl = self.graph.vehicle_length / 1000
        vw = self.graph.vehicle_width  / 1000
        line(f'AGV    : {vl:.2f}m x {vw:.2f}m', color=COL_DIM, font=self.font_s)
        y += 4

        line('── Keys ──', font=self.font_b)
        for hint in ['SPACE - start/pause', 'R - reset',
                     '+/- - sim speed',     'F/M - overlays',
                     '[/] - accel preset',  ',/. - decel preset',
                     'S - shuffle (DES)',   'C - CBS+TAPG plan',
                     'L - replay last scenario',
                     'T - toggle engine (DES/TS)',
                     'N/P - add/remove AGV (DES)']:
            line(hint, color=COL_DIM, font=self.font_s)

    # ── 메인 루프 ─────────────────────────────────────────────────────────────

    def run(self):
        while True:
            dt_real = self.clock.tick(FPS) / 1000.0
            self.handle_events()
            self.update(dt_real)
            self.render()


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--pkl',      default='grid_15x15_map.pkl')
    parser.add_argument('--agents',   type=int, default=6)
    parser.add_argument('--scenario', default=None,
                        help='재현할 시나리오 JSON 파일 경로')
    args = parser.parse_args()

    PKL_FILE = os.path.join(os.path.dirname(__file__), '..', args.pkl)

    print('Loading pkl...')
    graph = PklMapGraph(PKL_FILE)
    print(graph)
    print(f'Vehicle: {graph.vehicle_length:.0f}mm x {graph.vehicle_width:.0f}mm')
    bbox = graph.bbox
    print(f'Bbox: x=[{bbox[0]/1000:.1f}, {bbox[2]/1000:.1f}]m  '
          f'y=[{bbox[1]/1000:.1f}, {bbox[3]/1000:.1f}]m')

    sim = TAPGSimulator(graph, n_agents=args.agents, pkl_path=PKL_FILE)

    if args.scenario:
        print(f'Loading scenario: {args.scenario}')
        sim._plan_fixed(args.scenario)
    elif os.path.exists(SCENARIO_FILE):
        print(f'(Last scenario available: {SCENARIO_FILE}  →  press L to replay)')

    sim.run()
