"""
vis_pkl.py — Visualizer for collision-profile .pkl maps.

Extends vis_cont.py rendering style.

Keys
────
  SPACE   Start / Pause simulation
  R       Reset sim_time to 0
  F       Toggle stop-region overlay  (footprints at each node)
  M       Toggle move-region overlay  (swept areas on edges)
  S       Shuffle — reassign all agent paths randomly (random BFS)
  C       Plan (CBS) — CBS+SIPP conflict-free planning
  N / P   Add / Remove one vehicle
  +/-     Sim speed up / down
"""
from __future__ import annotations
import sys, os, math, random
from collections import deque
import pygame
from shapely.geometry import Polygon

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pkl_loader  import PklMapGraph
from env_cont    import KinematicAgent, IDLE, MOVING, A_MAX, trapezoid_params, AGENT_COLORS
from planner     import PklPlanner
from vis_cont    import (ContSimulator, _draw_rotated_rect, _draw_arrow,
                         _speed_tint, Button,
                         BG, SIDE_BG, COL_EDGE, COL_EDGE_A, COL_NODE,
                         COL_NODE_B, COL_PORT, COL_TEXT, COL_DIM,
                         COL_BTN, COL_HOVER, COL_ACTIVE, COL_DANGER, COL_WHITE,
                         COL_YELLOW, NODE_R_MM,
                         MAP_W, SIDE_W, WIN_W, WIN_H, MAP_PAD, FPS,
                         SIM_SPEEDS, SIM_SPEED_LABELS, DEFAULT_SPEED_IDX)


# ── Overlay colours ────────────────────────────────────────────────────────────
COL_STOP_FILL   = (100, 160, 255, 35)    # RGBA  — stop region fill
COL_STOP_BORDER = (100, 160, 255, 120)   # RGBA  — stop region border
COL_MOVE_FILL   = (255, 200,  80, 25)    # RGBA  — move region fill
COL_MOVE_BORDER = (255, 200,  80, 90)    # RGBA  — move region border

MAX_VEHICLES = len(AGENT_COLORS)
MIN_HOPS = 3
MAX_HOPS = 10


# ── Path utilities ─────────────────────────────────────────────────────────────

def _bfs_path(graph: PklMapGraph, start: str,
              min_hops: int = MIN_HOPS,
              max_hops: int = MAX_HOPS,
              exclude_start: bool = True) -> list:
    """
    BFS from `start`.  Returns a random path of length in [min_hops, max_hops].
    Returns [] if no path of at least min_hops exists.
    """
    visited = {start: None}
    q = deque([start])
    # collect nodes by BFS depth
    by_depth: dict = {0: [start]}   # depth -> list of node ids
    depth_of: dict = {start: 0}

    while q:
        cur = q.popleft()
        d   = depth_of[cur]
        if d >= max_hops - 1:
            continue
        for nb in graph.neighbors(cur):
            if nb not in visited:
                visited[nb] = cur
                depth_of[nb] = d + 1
                by_depth.setdefault(d + 1, []).append(nb)
                q.append(nb)

    # Pick a random reachable node at depth >= min_hops - 1
    candidates = []
    for d in range(min_hops - 1, max_hops):
        candidates.extend(by_depth.get(d, []))

    if not candidates:
        # fall back: any reachable node
        candidates = [n for n in visited if n != start] if exclude_start else list(visited)
        if not candidates:
            return []

    end = random.choice(candidates)
    path = []
    n = end
    while n is not None:
        path.append(n)
        n = visited[n]
    path.reverse()
    return path


def _shapely_to_screen(poly: Polygon, to_screen_fn) -> list:
    """Convert Shapely polygon exterior coords → list of screen (px, py)."""
    return [to_screen_fn(x, y)
            for x, y in list(poly.exterior.coords)[:-1]]


# ── Simulator ──────────────────────────────────────────────────────────────────

class PklSimulator(ContSimulator):
    """
    ContSimulator extended for pkl-based maps.

    Adds
    ────
    • Stop-region overlay (toggle F)
    • Move-region overlay (toggle M)
    • Random path assignment, auto-reassign when agents go IDLE
    • Vehicle count control (+/- buttons, N/P keys)
    • Shuffle button (S key) — reassign all agents to new random paths
    """

    def __init__(self, graph: PklMapGraph, n_agents: int = 3,
                 pkl_path: str = None):
        self._show_stop_regions = True
        self._show_move_regions = False
        self._region_surf_dirty = True
        self._plan_status       = ''   # 사이드바에 표시할 플래닝 상태 메시지
        self._cbs_mode          = False
        self._cbs_markers       = []   # [(start_node, goal_node, color), ...]

        # self.graph must be set before _create_agents (super() sets it later)
        self.graph     = graph
        self._next_id  = 0
        self._n_target = max(1, min(n_agents, MAX_VEHICLES))
        agents = self._create_agents(self._n_target)

        super().__init__(graph, agents)   # ContSimulator.__init__

        # ── CBS 플래너 초기화 (pkl_path 있을 때만) ─────────────────────────
        self._planner: 'PklPlanner | None' = None
        if pkl_path:
            try:
                print('Initializing CBS planner...')
                self._planner = PklPlanner(pkl_path)
                self._plan_status = 'Planner ready'
                print('Planner ready.')
            except Exception as e:
                self._plan_status = f'Planner error: {e}'
                print(f'Planner init failed: {e}')

    # ── Agent creation helpers ─────────────────────────────────────────────────

    def _fresh_color(self, agent_id: int):
        return AGENT_COLORS[agent_id % len(AGENT_COLORS)]

    def _create_agent(self, agent_id: int, occupied: set) -> 'KinematicAgent | None':
        """Spawn one agent at a random unoccupied start node with a random path."""
        free_nodes = [n for n in self.graph.nodes if n not in occupied]
        if not free_nodes:
            return None
        random.shuffle(free_nodes)
        for start in free_nodes:
            path = _bfs_path(self.graph, start)
            if path:
                edge0 = self.graph.get_edge(path[0], path[1])
                theta = edge0.angle if edge0 else 0.0
                a = KinematicAgent(agent_id, path[0], self.graph, start_theta=theta)
                a.color = self._fresh_color(agent_id)
                a.set_node_path(path, t_start=self.sim_time if hasattr(self, 'sim_time') else 0.0)
                return a
        return None

    def _create_agents(self, n: int) -> list:
        occupied = set()
        agents = []
        for _ in range(n):
            a = self._create_agent(self._next_id, occupied)
            if a is None:
                break
            occupied.add(a.node_path[0])
            self._next_id += 1
            agents.append(a)
        return agents

    def _assign_new_path(self, agent: 'KinematicAgent'):
        """Give `agent` a fresh random path starting at its current node."""
        cur = getattr(agent, 'cur_node', None) or agent.node_path[-1]
        path = _bfs_path(self.graph, cur)
        if not path:
            # try any random node as fallback
            path = _bfs_path(self.graph, random.choice(list(self.graph.nodes)))
        if not path:
            return
        agent.set_node_path(path, t_start=self.sim_time)

    # ── Extend button layout ───────────────────────────────────────────────────

    def _build_buttons(self):
        super()._build_buttons()

        sx = MAP_W + 10
        sw = SIDE_W - 20
        y  = self._info_y
        H  = 24
        G  = 4

        # Region overlays
        half = (sw - G) // 2
        self.btn_stop_reg = Button((sx,             y, half, H),
                                   'F: Stop Rgn', toggle=True)
        self.btn_move_reg = Button((sx + half + G,  y, half, H),
                                   'M: Move Rgn', toggle=True)
        self.btn_stop_reg.active = self._show_stop_regions
        self.btn_move_reg.active = self._show_move_regions
        y += H + G

        # Vehicle count row  [−]  «n vehicles»  [+]
        bw = 30
        self.btn_del_agent = Button((sx,           y, bw,       H),
                                    '−', base=COL_DANGER)
        self.btn_add_agent = Button((sx + sw - bw, y, bw,       H),
                                    '+')
        self._count_rect   = pygame.Rect(sx + bw + G, y,
                                         sw - 2 * bw - 2 * G, H)
        y += H + G

        # Shuffle button
        self.btn_shuffle = Button((sx, y, sw, H), 'S: Shuffle Paths')
        y += H + G

        # CBS plan button
        self.btn_plan = Button((sx, y, sw, H), 'C: Plan (CBS)')
        y += H + G

        self._info_y = y + 4
        self.all_buttons += [self.btn_stop_reg, self.btn_move_reg,
                             self.btn_del_agent, self.btn_add_agent,
                             self.btn_shuffle, self.btn_plan]

    # ── Events ─────────────────────────────────────────────────────────────────

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

    def _toggle_stop_regions(self):
        self._show_stop_regions = not self._show_stop_regions
        self.btn_stop_reg.active = self._show_stop_regions
        self._region_surf_dirty  = True

    def _toggle_move_regions(self):
        self._show_move_regions = not self._show_move_regions
        self.btn_move_reg.active = self._show_move_regions
        self._region_surf_dirty  = True

    def _shuffle_paths(self):
        """Teleport all agents to new random start nodes and assign new paths."""
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
                    node0 = self.graph.nodes[path[0]]
                    edge0 = self.graph.get_edge(path[0], path[1]) if len(path) >= 2 else None
                    agent.theta = edge0.angle if edge0 else 0.0
                    agent.x     = node0.x
                    agent.y     = node0.y
                    agent.v     = 0.0
                    agent.set_node_path(path, t_start=self.sim_time)
                    occupied.add(path[0])
                    break

    def _plan_cbs(self):
        """CBS+SIPP로 현재 에이전트 수만큼 충돌 없는 경로를 계획합니다."""
        if self._planner is None:
            self._plan_status = 'No planner (pkl_path needed)'
            return

        n = len(self.agents) or self._n_target
        self._plan_status = f'Planning {n} agents...'
        self.render()          # 상태 메시지를 즉시 화면에 표시
        pygame.display.flip()

        try:
            node_paths = self._planner.plan_random(n_agents=n, max_time=1.0)
        except Exception as e:
            self._plan_status = f'Plan error: {e}'
            print(f'CBS error: {e}')
            return

        if node_paths is None:
            self._plan_status = 'No solution found'
            return

        # 기존 에이전트를 CBS 경로로 교체
        self.agents    = []
        self._next_id  = 0
        self._cbs_markers = []
        for i, node_path in enumerate(node_paths):
            if len(node_path) < 1:
                continue
            if node_path[0] not in self.graph.nodes:
                continue
            edge0 = (self.graph.get_edge(node_path[0], node_path[1])
                     if len(node_path) >= 2 else None)
            theta = edge0.angle if edge0 else 0.0
            color = self._fresh_color(self._next_id)
            a = KinematicAgent(self._next_id, node_path[0], self.graph,
                               start_theta=theta)
            a.color = color
            a.set_node_path(node_path, t_start=self.sim_time)
            self.agents.append(a)
            self._cbs_markers.append((node_path[0], node_path[-1], color))
            self._next_id += 1

        self._cbs_mode    = True
        self._plan_status = f'CBS ok: {len(self.agents)} agents planned'
        self.running      = False   # 확인 후 직접 시작하도록 일시정지

    def _add_agent(self):
        if len(self.agents) >= MAX_VEHICLES:
            return
        occupied = {a.node_path[0] for a in self.agents}
        a = self._create_agent(self._next_id, occupied)
        if a:
            self._next_id += 1
            self.agents.append(a)

    def _remove_agent(self):
        if self.agents:
            self.agents.pop()

    # ── Update (auto-reassign idle agents) ─────────────────────────────────────

    def update(self, dt_real: float):
        if not self.running:
            return
        self.sim_time += dt_real * SIM_SPEEDS[self.spd_idx]
        for agent in self.agents:
            agent.update(self.sim_time)
            # CBS 모드에서는 경로가 끝나도 재할당 안 함
            if agent.status == IDLE and not self._cbs_mode:
                self._assign_new_path(agent)

    # ── Region surface (cached) ────────────────────────────────────────────────

    def _rebuild_region_surf(self):
        """Pre-render all region overlays onto a cached SRCALPHA surface."""
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

        self._region_surf        = surf
        self._region_surf_dirty  = False

    # ── Render override ────────────────────────────────────────────────────────

    def render(self):
        self.screen.fill(BG)
        pygame.draw.rect(self.screen, SIDE_BG,
                         pygame.Rect(MAP_W, 0, SIDE_W, WIN_H))

        self._draw_edges()

        # Region overlay
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

    # ── Sidebar override ───────────────────────────────────────────────────────

    def _draw_sidebar(self):
        super()._draw_sidebar()
        # Vehicle count label in the count rect
        n   = len(self.agents)
        lbl = self.font_s.render(f'{n} vehicle{"s" if n != 1 else ""}', True, COL_TEXT)
        self.screen.blit(lbl, lbl.get_rect(center=self._count_rect.center))
        # Planner status
        if self._plan_status:
            col = ((100, 220, 100) if 'ok' in self._plan_status or 'ready' in self._plan_status
                   else (220, 100, 100) if 'error' in self._plan_status or 'No' in self._plan_status
                   else COL_DIM)
            st_lbl = self.font_s.render(self._plan_status, True, col)
            self.screen.blit(st_lbl, (MAP_W + 10, self._info_y - 18))

    # ── Edge drawing: use node IDs (strings) ──────────────────────────────────

    def _draw_edges(self):
        for (fn, tn), edge in self.graph.edges.items():
            fn_n = self.graph.nodes[fn]
            tn_n = self.graph.nodes[tn]
            p0   = self.to_screen(fn_n.x, fn_n.y)
            p1   = self.to_screen(tn_n.x, tn_n.y)
            pygame.draw.line(self.screen, COL_EDGE, p0, p1, 2)
            mx = p0[0] + (p1[0] - p0[0]) * 0.6
            my = p0[1] + (p1[1] - p0[1]) * 0.6
            scr_ang = math.atan2(-(tn_n.y - fn_n.y), tn_n.x - fn_n.x)
            _draw_arrow(self.screen, COL_EDGE_A, mx, my, scr_ang, size=7)

    # ── Node drawing: label with node ID ──────────────────────────────────────

    def _draw_nodes(self):
        r_px = max(5, min(int(self.px(NODE_R_MM)), 10))
        for nid, node in self.graph.nodes.items():
            sx, sy   = self.to_screen(node.x, node.y)
            is_port  = nid in self.graph.ports.values()
            col      = COL_PORT if is_port else COL_NODE
            pygame.draw.circle(self.screen, col,        (sx, sy), r_px)
            pygame.draw.circle(self.screen, COL_NODE_B, (sx, sy), r_px, 1)
            lbl = self.font_s.render(nid, True, COL_DIM)
            self.screen.blit(lbl, (sx + r_px + 2, sy - 7))

    # ── CBS start / goal markers ───────────────────────────────────────────────

    def _draw_cbs_markers(self):
        surf = pygame.Surface((MAP_W, WIN_H), pygame.SRCALPHA)
        R = 14

        for start_nid, goal_nid, color in self._cbs_markers:
            # ── Start: filled circle + "S" ──────────────────────────────────
            if start_nid in self.graph.nodes:
                n = self.graph.nodes[start_nid]
                sx, sy = self.to_screen(n.x, n.y)
                pygame.draw.circle(surf, (*color, 180), (sx, sy), R)
                pygame.draw.circle(surf, (255, 255, 255, 220), (sx, sy), R, 2)
                lbl = self.font_s.render('S', True, (255, 255, 255))
                surf.blit(lbl, lbl.get_rect(center=(sx, sy)))

            # ── Goal: concentric rings + "G" ────────────────────────────────
            if goal_nid in self.graph.nodes:
                n = self.graph.nodes[goal_nid]
                gx, gy = self.to_screen(n.x, n.y)
                pygame.draw.circle(surf, (*color, 60),  (gx, gy), R + 5)
                pygame.draw.circle(surf, (*color, 180), (gx, gy), R + 5, 2)
                pygame.draw.circle(surf, (*color, 180), (gx, gy), R - 4, 2)
                lbl = self.font_s.render('G', True, (*color, 255))
                surf.blit(lbl, lbl.get_rect(center=(gx, gy)))

        self.screen.blit(surf, (0, 0))


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    PKL_FILE  = os.path.join(os.path.dirname(__file__), '..', 'c10_map.pkl')
    N_AGENTS  = 3   # initial vehicle count

    print('Loading pkl...')
    graph = PklMapGraph(PKL_FILE)
    print(graph)
    print(f'Vehicle: {graph.vehicle_length:.0f}mm × {graph.vehicle_width:.0f}mm')
    bbox = graph.bbox
    print(f'Bbox: x=[{bbox[0]/1000:.1f}, {bbox[2]/1000:.1f}]m  '
          f'y=[{bbox[1]/1000:.1f}, {bbox[3]/1000:.1f}]m')

    PklSimulator(graph, n_agents=N_AGENTS, pkl_path=PKL_FILE).run()
