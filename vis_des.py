"""
vis_des.py — DES (Discrete Event Simulation) visualizer for pkl-based maps.

Discrete-event approach: agent positions are linearly interpolated between
node-arrival events rather than computed from a kinematic model.

Keys
────
  SPACE   Start / Pause simulation
  R       Reset (all agents restart from beginning of their paths)
  F       Toggle stop-region overlay
  M       Toggle move-region overlay
  S       Shuffle — reassign all agent paths randomly
  C       Plan (CBS) — CBS+SIPP conflict-free planning
  N / P   Add / Remove one vehicle
  +/-     Sim speed up / down

Agent state colours
───────────────────
  MOVING  — colored rect, speed tint
  WAITING — colored rect + orange outer ring (blocked by another agent)
  IDLE    — colored rect, dimmed (briefly at a node between events)
  DONE    — dark rect (path completed)
"""
from __future__ import annotations
import sys, os, math, random
import pygame

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pkl_loader  import PklMapGraph
from env_cont    import AGENT_COLORS
from env_des     import DESAgent, DESEnvironment, IDLE, MOVING, WAITING, DONE
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

COL_WAITING = (255, 165, 0)   # orange ring for WAITING agents


class DESSimulator:
    """
    Pygame visualizer for DES-based MAPF simulation on pkl maps.

    Uses DESEnvironment (event queue + reservation table) instead of
    per-agent kinematic models.  Positions are linearly interpolated
    between TRY_DEPART and ARRIVE events for smooth rendering.
    """

    def __init__(self, graph: PklMapGraph, n_agents: int = 3,
                 pkl_path: str = None):
        self._show_stop_regions = True
        self._show_move_regions = False
        self._region_surf_dirty = True
        self._plan_status       = ''
        self._cbs_mode          = False
        self._cbs_markers       = []   # [(start_node, goal_node, color), ...]

        self.graph    = graph
        self._next_id = 0
        self._n_target = max(1, min(n_agents, MAX_VEHICLES))

        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption('DES MAPF Simulator')
        self.clock = pygame.time.Clock()

        self.font_s = pygame.font.SysFont('Consolas', 13)
        self.font_m = pygame.font.SysFont('Consolas', 15)
        self.font_b = pygame.font.SysFont('Consolas', 16, bold=True)

        self.sim_time = 0.0
        self.running  = False
        self.spd_idx  = DEFAULT_SPEED_IDX

        self._build_transform()
        self._build_buttons()

        # Create agents and initial DES environment
        self.agents   = self._create_agents(self._n_target)
        self._des_env = self._make_des_env(self.agents, t_start=0.0)

        # CBS planner (optional — needs pkl_path)
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

    # ── Coordinate transform ──────────────────────────────────────────────────

    def _build_transform(self):
        x0, y0, x1, y1 = self.graph.bbox
        map_w, map_h = x1 - x0, y1 - y0
        avail_w = MAP_W - 2 * MAP_PAD
        avail_h = WIN_H - 2 * MAP_PAD
        self._scale = min(avail_w / map_w, avail_h / map_h) if map_w > 0 and map_h > 0 else 0.05
        map_cx, map_cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        self._ox = MAP_W / 2.0 - map_cx * self._scale
        self._oy = WIN_H / 2.0 + map_cy * self._scale

    def to_screen(self, x_mm: float, y_mm: float) -> tuple:
        return (int(self._ox + x_mm * self._scale),
                int(self._oy - y_mm * self._scale))

    def px(self, mm: float) -> float:
        return mm * self._scale

    # ── Button layout ─────────────────────────────────────────────────────────

    def _build_buttons(self):
        sx  = MAP_W + 10
        sw  = SIDE_W - 20
        y   = 12
        H, G = 30, 6

        self.btn_start = Button((sx, y, sw, H), '▶ Start / Pause')
        y += H + G
        self.btn_reset = Button((sx, y, sw, H), '↺  Reset Time', base=COL_DANGER)
        y += H + G + 6

        # Speed buttons (2 rows of 4)
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

        # Region overlay toggles
        half = (sw - G) // 2
        self.btn_stop_reg = Button((sx,            y, half, H - 6), 'F: Stop Rgn', toggle=True)
        self.btn_move_reg = Button((sx + half + G, y, half, H - 6), 'M: Move Rgn', toggle=True)
        self.btn_stop_reg.active = self._show_stop_regions
        self.btn_move_reg.active = self._show_move_regions
        y += H - 6 + G

        # Vehicle count row  [−]  «n vehicles»  [+]
        bw2 = 30
        self.btn_del_agent = Button((sx,            y, bw2,           H - 6), '−', base=COL_DANGER)
        self.btn_add_agent = Button((sx + sw - bw2, y, bw2,           H - 6), '+')
        self._count_rect   = pygame.Rect(sx + bw2 + G, y, sw - 2 * bw2 - 2 * G, H - 6)
        y += H - 6 + G

        # Shuffle + CBS
        self.btn_shuffle = Button((sx, y, sw, H - 6), 'S: Shuffle Paths')
        y += H - 6 + G
        self.btn_plan    = Button((sx, y, sw, H - 6), 'C: Plan (CBS)')
        y += H - 6 + G

        self._info_y     = y + 4
        self.all_buttons = ([self.btn_start, self.btn_reset, *self.btns_speed,
                             self.btn_stop_reg, self.btn_move_reg,
                             self.btn_del_agent, self.btn_add_agent,
                             self.btn_shuffle,  self.btn_plan])

    # ── Agent factory ─────────────────────────────────────────────────────────

    def _fresh_color(self, agent_id: int) -> tuple:
        return AGENT_COLORS[agent_id % len(AGENT_COLORS)]

    def _create_agent(self, agent_id: int, occupied: set) -> DESAgent | None:
        free = [n for n in self.graph.nodes if n not in occupied]
        if not free:
            return None
        random.shuffle(free)
        for start in free:
            path = _bfs_path(self.graph, start)
            if path:
                return DESAgent(agent_id, self._fresh_color(agent_id), path)
        return None

    def _create_agents(self, n: int) -> list:
        occupied, agents = set(), []
        for _ in range(n):
            a = self._create_agent(self._next_id, occupied)
            if a is None:
                break
            occupied.add(a.node_path[0])
            self._next_id += 1
            agents.append(a)
        return agents

    # ── DES environment factory ───────────────────────────────────────────────

    def _make_des_env(self, agents: list, t_start: float = 0.0) -> DESEnvironment:
        """Create a fresh DES environment and register all agents."""
        env = DESEnvironment(self.graph)
        for agent in agents:
            # Reset agent state before re-adding
            agent.path_idx = 0
            agent.state    = IDLE
            agent.reserved = []
            env.add_agent(agent, t_start=t_start)
        return env

    # ── DES agent reassignment (non-CBS mode) ─────────────────────────────────

    def _reassign_agent(self, agent: DESAgent):
        """Assign a new random path to a DONE agent and reinsert into DES env."""
        cur  = agent.cur_node
        path = _bfs_path(self.graph, cur)
        if not path:
            path = _bfs_path(self.graph, random.choice(list(self.graph.nodes)))
        if not path:
            return
        self._des_env.remove_agent(agent.id)
        agent.node_path = path
        agent.path_idx  = 0
        agent.state     = IDLE
        agent.reserved  = []
        self._des_env.add_agent(agent, t_start=self.sim_time)

    # ── Update ────────────────────────────────────────────────────────────────

    def update(self, dt_real: float):
        if not self.running:
            return
        self.sim_time += dt_real * SIM_SPEEDS[self.spd_idx]
        self._des_env.step(self.sim_time)

        if self._cbs_mode:
            if self._des_env.all_done():
                self.running = False
        else:
            for agent in self.agents:
                if agent.state == DONE:
                    self._reassign_agent(agent)

    # ── Reset ─────────────────────────────────────────────────────────────────

    def _reset(self):
        self.sim_time = 0.0
        self.running  = False
        self._des_env = self._make_des_env(self.agents, t_start=0.0)

    # ── Shuffle ───────────────────────────────────────────────────────────────

    def _shuffle_paths(self):
        self._cbs_mode    = False
        self._cbs_markers = []
        all_nodes = list(self.graph.nodes)
        random.shuffle(all_nodes)
        occupied = set()
        for agent in self.agents:
            free = [n for n in all_nodes if n not in occupied]
            if not free:
                free = all_nodes
            for start in free:
                path = _bfs_path(self.graph, start)
                if path:
                    agent.node_path = path
                    occupied.add(start)
                    break
        self._des_env = self._make_des_env(self.agents, t_start=self.sim_time)

    # ── CBS planning ──────────────────────────────────────────────────────────

    def _plan_cbs(self):
        if self._planner is None:
            self._plan_status = 'No planner (pkl_path needed)'
            return

        n = len(self.agents) or self._n_target
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
            return

        self.agents       = []
        self._next_id     = 0
        self._cbs_markers = []

        for i, node_path in enumerate(node_paths):
            if len(node_path) < 1 or node_path[0] not in self.graph.nodes:
                continue
            color = self._fresh_color(self._next_id)
            a     = DESAgent(self._next_id, color, node_path)
            self.agents.append(a)
            self._cbs_markers.append((node_path[0], node_path[-1], color))
            self._next_id += 1

        self._des_env     = self._make_des_env(self.agents, t_start=self.sim_time)
        self._cbs_mode    = True
        self._plan_status = f'CBS ok: {len(self.agents)} agents planned'
        self.running      = False   # user presses SPACE to start

    # ── Agent count controls ──────────────────────────────────────────────────

    def _add_agent(self):
        if len(self.agents) >= MAX_VEHICLES:
            return
        occupied = {a.node_path[0] for a in self.agents}
        a = self._create_agent(self._next_id, occupied)
        if a:
            self.agents.append(a)
            self._next_id += 1
            self._des_env.add_agent(a, t_start=self.sim_time)

    def _remove_agent(self):
        if self.agents:
            a = self.agents.pop()
            self._des_env.remove_agent(a.id)

    # ── Speed ─────────────────────────────────────────────────────────────────

    def _set_speed(self, idx: int):
        self.spd_idx = idx
        for i, b in enumerate(self.btns_speed):
            b.active = (i == idx)

    # ── Events ────────────────────────────────────────────────────────────────

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
                elif k == pygame.K_n:
                    self._add_agent()
                elif k == pygame.K_p:
                    self._remove_agent()

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

    # ── Region toggles ────────────────────────────────────────────────────────

    def _toggle_stop_regions(self):
        self._show_stop_regions  = not self._show_stop_regions
        self.btn_stop_reg.active = self._show_stop_regions
        self._region_surf_dirty  = True

    def _toggle_move_regions(self):
        self._show_move_regions  = not self._show_move_regions
        self.btn_move_reg.active = self._show_move_regions
        self._region_surf_dirty  = True

    # ── Cached region surface ─────────────────────────────────────────────────

    def _rebuild_region_surf(self):
        surf = pygame.Surface((MAP_W, WIN_H), pygame.SRCALPHA)
        if self._show_move_regions:
            for state_id, poly in self.graph.move_regions.items():
                pts = _shapely_to_screen(poly, self.to_screen)
                if len(pts) >= 3:
                    pygame.draw.polygon(surf, COL_MOVE_FILL,   pts)
                    pygame.draw.polygon(surf, COL_MOVE_BORDER, pts, 1)
        if self._show_stop_regions:
            for state_id, poly in self.graph.stop_regions.items():
                pts = _shapely_to_screen(poly, self.to_screen)
                if len(pts) >= 3:
                    pygame.draw.polygon(surf, COL_STOP_FILL,   pts)
                    pygame.draw.polygon(surf, COL_STOP_BORDER, pts, 1)
        self._region_surf       = surf
        self._region_surf_dirty = False

    # ── Render ────────────────────────────────────────────────────────────────

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

    # ── Map drawing ───────────────────────────────────────────────────────────

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
        surf = pygame.Surface((MAP_W, WIN_H), pygame.SRCALPHA)
        # CBS 모드에서는 경로를 더 뚜렷하게 표시
        line_alpha = 160 if self._cbs_mode else 80
        node_alpha = 200 if self._cbs_mode else 100
        line_width = 3   if self._cbs_mode else 2

        for agent in self.agents:
            path = agent.node_path
            pts  = [self.to_screen(self.graph.nodes[n].x, self.graph.nodes[n].y)
                    for n in path if n in self.graph.nodes]
            if len(pts) < 2:
                continue

            col_line = (*agent.color, line_alpha)
            col_node = (*agent.color, node_alpha)

            pygame.draw.lines(surf, col_line, False, pts, line_width)

            # 각 엣지 중간에 방향 화살표
            for i in range(len(pts) - 1):
                x0, y0 = pts[i]
                x1, y1 = pts[i + 1]
                mx = x0 + (x1 - x0) * 0.6
                my = y0 + (y1 - y0) * 0.6
                ang = math.atan2(y1 - y0, x1 - x0)
                size = 6
                cos_a, sin_a = math.cos(ang), math.sin(ang)
                tip = (mx + cos_a  * size,        my + sin_a  * size)
                bl  = (mx - cos_a  * size * 0.5 + (-sin_a) * size * 0.6,
                       my - sin_a  * size * 0.5 + cos_a    * size * 0.6)
                br  = (mx - cos_a  * size * 0.5 - (-sin_a) * size * 0.6,
                       my - sin_a  * size * 0.5 - cos_a    * size * 0.6)
                pygame.draw.polygon(surf, col_line, [tip, bl, br])

            # 중간 경유 노드 (시작/끝 제외)
            for pt in pts[1:-1]:
                pygame.draw.circle(surf, col_node, pt, 4)
        self.screen.blit(surf, (0, 0))

    # ── Agent drawing ─────────────────────────────────────────────────────────

    def _draw_agents(self):
        w_px = max(self.px(self.graph.vehicle_width),  8.0)
        l_px = max(self.px(self.graph.vehicle_length), 12.0)
        v_max = max((e.max_speed for e in self.graph.edges.values()), default=1000.0)

        for agent in self.agents:
            sx, sy  = self.to_screen(agent.x, agent.y)
            scr_ang = math.degrees(-agent.theta)

            # Fill / border colour by state
            if agent.state == DONE:
                fill   = tuple(c // 3 for c in agent.color)
                border = COL_DIM
            elif agent.state == WAITING:
                fill   = agent.color
                border = COL_WHITE
            elif agent.state == MOVING:
                vf     = min(agent.v / v_max, 1.0) if v_max > 0 else 0.0
                fill   = _speed_tint(agent.color, vf)
                border = COL_WHITE
            else:  # IDLE
                fill   = tuple(max(0, c - 40) for c in agent.color)
                border = COL_DIM

            _draw_rotated_rect(self.screen, fill, sx, sy,
                               l_px, w_px, scr_ang,
                               border=border, border_w=2)

            # Headlight dot
            rad = math.radians(scr_ang)
            hx  = sx + math.cos(rad) * l_px / 2
            hy  = sy + math.sin(rad) * l_px / 2
            pygame.draw.circle(self.screen, (255, 255, 180),
                               (int(hx), int(hy)), max(3, int(w_px * 0.18)))

            # State indicator ring
            if agent.state == WAITING:
                # Orange ring around whole body — clearly visible "blocked" signal
                pygame.draw.circle(self.screen, COL_WAITING,
                                   (sx, sy), int(l_px / 2) + 5, 3)
            elif agent.state == MOVING:
                # Small colored dot at the front corner
                pygame.draw.circle(self.screen, agent.color,
                                   (sx + int(l_px / 2) - 2, sy - int(w_px / 2) + 2), 4)
            elif agent.state == IDLE:
                # Dim ring — "parked"
                pygame.draw.circle(self.screen, COL_DIM,
                                   (sx, sy), int(l_px / 2) + 2, 2)

            # Agent ID label
            lbl = self.font_s.render(str(agent.id), True,
                                     COL_WHITE if agent.state != DONE else COL_DIM)
            self.screen.blit(lbl, lbl.get_rect(center=(sx, sy)))

    # ── CBS start / goal markers ──────────────────────────────────────────────

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

    # ── Sidebar ───────────────────────────────────────────────────────────────

    def _draw_sidebar(self):
        for b in self.all_buttons:
            b.draw(self.screen, self.font_s)

        # Speed section label
        sp_lbl = self.font_s.render('Sim speed:', True, COL_DIM)
        self.screen.blit(sp_lbl, (MAP_W + 10, self.btns_speed[0].rect.y - 16))

        # Vehicle count label centred in its rect
        n = len(self.agents)
        lbl = self.font_s.render(f'{n} vehicle{"s" if n != 1 else ""}', True, COL_TEXT)
        self.screen.blit(lbl, lbl.get_rect(center=self._count_rect.center))

        y  = self._info_y
        sx = MAP_W + 10

        # Planner status line
        if self._plan_status:
            col = ((100, 220, 100) if 'ok'    in self._plan_status or 'ready' in self._plan_status
                   else (220, 100, 100) if 'error' in self._plan_status or 'No' in self._plan_status
                   else COL_DIM)
            self.screen.blit(self.font_s.render(self._plan_status, True, col), (sx, y))
            y += 18

        def line(text, color=COL_TEXT, font=None):
            nonlocal y
            f = font or self.font_m
            self.screen.blit(f.render(text, True, color), (sx, y))
            y += f.size(text)[1] + 3

        # Simulation info
        line('── Simulation ──', font=self.font_b)
        line(f'Time   : {self.sim_time:8.2f} s')
        line(f'Speed  : {SIM_SPEED_LABELS[self.spd_idx]}')
        state_str = '▶ Running' if self.running else '⏸ Paused'
        line(state_str, color=(100, 220, 100) if self.running else (170, 170, 170))
        y += 6

        # DES summary
        n_moving  = sum(1 for a in self.agents if a.state == MOVING)
        n_waiting = sum(1 for a in self.agents if a.state == WAITING)
        n_done    = sum(1 for a in self.agents if a.state == DONE)
        line('── DES Stats ──', font=self.font_b)
        line(f'Moving : {n_moving}',  color=COL_TEXT, font=self.font_s)
        line(f'Waiting: {n_waiting}',
             color=COL_WAITING if n_waiting else COL_DIM, font=self.font_s)
        line(f'Done   : {n_done}',    color=COL_DIM,  font=self.font_s)
        line(f'Reserv : {self._des_env.reservation_count()}',
             color=COL_DIM, font=self.font_s)
        y += 6

        # Per-agent list
        line('── Agents ──', font=self.font_b)
        STATE_ICON = {MOVING: '▶', WAITING: '⏸', IDLE: '○', DONE: '✓'}
        STATE_COL  = {MOVING: COL_TEXT, WAITING: COL_WAITING, IDLE: COL_DIM, DONE: COL_DIM}
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
                    f'   ({agent.x/1000:.1f}, {agent.y/1000:.1f}) m  '
                    f'θ={math.degrees(agent.theta):.0f}°',
                    True, COL_DIM),
                (sx, y))
            y += 18
        y += 6

        # Map info
        line('── Map ──', font=self.font_b)
        line(f'Nodes  : {len(self.graph.nodes)}', color=COL_DIM, font=self.font_s)
        line(f'Edges  : {len(self.graph.edges)}', color=COL_DIM, font=self.font_s)
        vl = self.graph.vehicle_length / 1000
        vw = self.graph.vehicle_width  / 1000
        line(f'AGV    : {vl:.2f}m × {vw:.2f}m', color=COL_DIM, font=self.font_s)
        y += 6

        # Key hints
        line('── Keys ──', font=self.font_b)
        for hint in ['SPACE — start/pause',  'R — reset',
                     '+/- — sim speed',      'F/M — region overlays',
                     'S — shuffle paths',    'C — CBS plan',
                     'N/P — add/remove AGV']:
            line(hint, color=COL_DIM, font=self.font_s)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        while True:
            dt_real = self.clock.tick(FPS) / 1000.0
            self.handle_events()
            self.update(dt_real)
            self.render()


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    PKL_FILE = os.path.join(os.path.dirname(__file__), '..', 'c10_map.pkl')
    N_AGENTS = 3

    print('Loading pkl...')
    graph = PklMapGraph(PKL_FILE)
    print(graph)
    print(f'Vehicle: {graph.vehicle_length:.0f}mm × {graph.vehicle_width:.0f}mm')
    bbox = graph.bbox
    print(f'Bbox: x=[{bbox[0]/1000:.1f}, {bbox[2]/1000:.1f}]m  '
          f'y=[{bbox[1]/1000:.1f}, {bbox[3]/1000:.1f}]m')

    DESSimulator(graph, n_agents=N_AGENTS, pkl_path=PKL_FILE).run()
