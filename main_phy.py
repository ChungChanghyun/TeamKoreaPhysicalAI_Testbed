"""
main_phy.py — Physical MAPF Simulator (Pygame)

Agents have:
  - Physical size (rotated rectangle body)
  - Discrete heading: E / N / W / S
  - Discrete speed levels v ∈ {0, 1, 2, 3}  (cells per tick)
  - In-place rotation only when v = 0
  - Acceleration / deceleration constrained to ±1 per tick

Controls:
  SPACE       Start / Pause
  T           Add random task
  R           Replan all agents
  Left-click  Grid interaction (mode-dependent)
"""
import sys, math
import pygame

from core_phy import (GridWorld, PhysAgent, TaskManager,
                      IDLE, TO_PICKUP, TO_DELIVERY,
                      DIRS, SCREEN_ANGLES, THETA_NAMES, V_MAX, AGENT_COLORS)
from pathfinder_phy import (plan_phy_all, plan_phy_cbs,
                             get_footprint, find_phy_conflict,
                             get_swept_poly, PhysAgent_BODY_W, PhysAgent_BODY_L)

# ── Layout ─────────────────────────────────────────────────────────────────────
ROWS, COLS   = 15, 15
CELL         = 40
PAD          = 10
SIDE_W       = 265
WIN_W        = PAD + COLS*CELL + PAD + SIDE_W + PAD
WIN_H        = PAD + ROWS*CELL + PAD
GRID_LEFT    = PAD
GRID_TOP     = PAD
SIDE_LEFT    = GRID_LEFT + COLS*CELL + PAD

FPS          = 30
MAX_AGENTS   = 8
TASK_INTERVAL= 60

SPEED_LABELS = ['▶', '▶▶', '▶▶▶', '⚡']
SPEED_FRAMES = [20, 10,  4,   1]

# ── Colours ────────────────────────────────────────────────────────────────────
BG         = (18,  18,  24)
SIDE_BG    = (26,  26,  34)
CELL_FREE  = (225, 225, 228)
CELL_OBS   = (50,  50,  60)
GRID_LINE  = (155, 155, 165)
COL_PICKUP = (45,  210,  80)
COL_DELIV  = (220,  60,  60)
COL_TEXT   = (215, 215, 225)
COL_DIM    = (115, 115, 130)
COL_BTN    = (50,  72, 105)
COL_HOVER  = (70, 100, 148)
COL_ACTIVE = (85, 130, 190)
COL_DANGER = (130,  45,  45)
COL_WHITE  = (255, 255, 255)


# ── Drawing helpers ────────────────────────────────────────────────────────────

def draw_rotated_rect(surf, color, cx, cy, length_px, width_px, screen_angle_deg,
                      border=None, border_w=2):
    """Draw a filled rectangle centred at (cx, cy), rotated by screen_angle_deg."""
    rad   = math.radians(screen_angle_deg)
    cos_a = math.cos(rad);  sin_a = math.sin(rad)
    fw    = ( cos_a,  sin_a)   # forward unit vector
    rt    = (-sin_a,  cos_a)   # right   unit vector
    hl, hw = length_px / 2, width_px / 2

    corners = []
    for fl, rw in [(-hl,-hw), (-hl,hw), (hl,hw), (hl,-hw)]:
        corners.append((cx + fl*fw[0] + rw*rt[0],
                        cy + fl*fw[1] + rw*rt[1]))

    pygame.draw.polygon(surf, color, corners)
    if border:
        pygame.draw.polygon(surf, border, corners, border_w)
    return corners


def draw_arrow(surf, color, cx, cy, screen_angle_deg, size=7):
    """Draw a small filled arrowhead."""
    rad   = math.radians(screen_angle_deg)
    fw    = ( math.cos(rad),  math.sin(rad))
    rt    = (-math.sin(rad),  math.cos(rad))
    tip   = (cx + fw[0]*size,       cy + fw[1]*size)
    bl    = (cx - fw[0]*size*0.5 + rt[0]*size*0.55,
             cy - fw[1]*size*0.5 + rt[1]*size*0.55)
    br    = (cx - fw[0]*size*0.5 - rt[0]*size*0.55,
             cy - fw[1]*size*0.5 - rt[1]*size*0.55)
    pygame.draw.polygon(surf, color, [tip, bl, br])


def lerp_angle(a, b, t):
    """Shortest-path angular interpolation."""
    diff = ((b - a + 180) % 360) - 180
    return a + diff * t


def speed_tint(base_color, vel_frac):
    """Tint agent colour toward orange-red at high speed."""
    r, g, b = base_color
    return (min(255, int(r + (255-r)*vel_frac*0.45)),
            min(255, int(g * (1 - vel_frac*0.35))),
            max(0,   int(b * (1 - vel_frac*0.50))))


# ── Button ─────────────────────────────────────────────────────────────────────

class Button:
    def __init__(self, rect, label, base=None, toggle=False):
        self.rect   = pygame.Rect(rect)
        self.label  = label
        self.base   = base or COL_BTN
        self.toggle = toggle
        self.active = False
        self._hover = False

    def update_hover(self, pos): self._hover = self.rect.collidepoint(pos)
    def clicked(self, pos):      return self.rect.collidepoint(pos)

    def draw(self, surf, font):
        col = COL_ACTIVE if (self.toggle and self.active) else \
              COL_HOVER  if self._hover else self.base
        pygame.draw.rect(surf, col, self.rect, border_radius=5)
        pygame.draw.rect(surf, (175,185,210), self.rect, 1, border_radius=5)
        surf.blit(font.render(self.label, True, COL_WHITE),
                  font.render(self.label, True, COL_WHITE).get_rect(center=self.rect.center))


# ── Simulator ──────────────────────────────────────────────────────────────────

class PhysSimulator:

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption('Physical MAPF Simulator')
        self.clock  = pygame.font.SysFont  # placeholder, overwritten below
        self.clock  = pygame.time.Clock()

        self.font_s = pygame.font.SysFont('Consolas', 13)
        self.font_m = pygame.font.SysFont('Consolas', 15)
        self.font_b = pygame.font.SysFont('Consolas', 16, bold=True)

        self._init_state()
        self._build_buttons()

    # ── State ─────────────────────────────────────────────────────────────────

    def _init_state(self):
        self.grid        = GridWorld(ROWS, COLS)
        self.agents      = []
        self.task_mgr    = TaskManager(self.grid)
        self.mode        = 'obstacle'      # 'obstacle' | 'agent'
        self.algorithm   = 'prioritized'   # 'prioritized' | 'cbs'
        self.cbs_ok      = True
        self.place_theta = 0               # heading for next placed agent (0=E)
        self.running     = False
        self.tick        = 0
        self.spd_idx     = 1
        self.frame_cnt   = 0
        self.anim_t      = 0.0

    # ── Button layout ─────────────────────────────────────────────────────────

    def _build_buttons(self):
        sx   = SIDE_LEFT
        sw   = SIDE_W - PAD
        y    = PAD
        H, G = 30, 6
        half = (sw - 4) // 2

        self.btn_obs   = Button((sx,        y, half, H), '✏ Obstacle', toggle=True)
        self.btn_agent = Button((sx+half+4, y, half, H), '🤖 Agent',   toggle=True)
        self.btn_obs.active = True
        y += H + G

        self.btn_start  = Button((sx, y, sw, H), '▶ Start / Pause');  y += H + G
        self.btn_task   = Button((sx, y, sw, H), '＋ Add Task  [T]'); y += H + G
        self.btn_replan = Button((sx, y, sw, H), '↺  Replan   [R]');  y += H + G

        self.btn_prio = Button((sx,        y, half, H), 'Prioritized', toggle=True)
        self.btn_cbs  = Button((sx+half+4, y, half, H), 'CBS',         toggle=True)
        self.btn_prio.active = True
        y += H + G

        # Heading selector (shown when in Agent mode)
        hw4 = sw // 4
        self.btns_theta = []
        for i, name in enumerate(THETA_NAMES):
            b = Button((sx + i*hw4, y, hw4-2, H-4), name, toggle=True)
            b.active = (i == 0)
            self.btns_theta.append(b)
        y += H + G

        pw = (sw - 4) // 2
        self.btn_preset = Button((sx,      y, pw, H), '⬛ Warehouse')
        self.btn_maze   = Button((sx+pw+4, y, pw, H), '🌀 Maze')
        y += H + G
        self.btn_clear  = Button((sx, y, sw, H), '🗑  Clear All', base=COL_DANGER)
        y += H + G + 2

        bw = sw // len(SPEED_LABELS)
        self.btns_speed = []
        for i, lbl in enumerate(SPEED_LABELS):
            b = Button((sx + i*bw, y, bw-2, H-4), lbl, toggle=True)
            b.active = (i == self.spd_idx)
            self.btns_speed.append(b)
        y += H + G

        self._info_y = y + 8

        self.all_buttons = [
            self.btn_obs, self.btn_agent,
            self.btn_start, self.btn_task, self.btn_replan,
            self.btn_prio, self.btn_cbs,
            *self.btns_theta,
            self.btn_preset, self.btn_maze, self.btn_clear,
            *self.btns_speed,
        ]

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def cell_at(self, px, py):
        c = (px - GRID_LEFT) // CELL;  r = (py - GRID_TOP) // CELL
        return (r, c) if 0 <= r < ROWS and 0 <= c < COLS else None

    def cell_center_px(self, r, c):
        return (GRID_LEFT + c*CELL + CELL//2,
                GRID_TOP  + r*CELL + CELL//2)

    def cell_rect(self, r, c):
        return pygame.Rect(GRID_LEFT+c*CELL+1, GRID_TOP+r*CELL+1, CELL-2, CELL-2)

    # ── Event handling ────────────────────────────────────────────────────────

    def handle_events(self):
        mpos = pygame.mouse.get_pos()
        for btn in self.all_buttons: btn.update_hover(mpos)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()

            if event.type == pygame.KEYDOWN:
                if   event.key == pygame.K_SPACE: self._toggle_run()
                elif event.key == pygame.K_t:     self._add_task()
                elif event.key == pygame.K_r:     self._replan()

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                pos = event.pos

                # Mode
                if self.btn_obs.clicked(pos):
                    self.mode = 'obstacle'
                    self.btn_obs.active = True; self.btn_agent.active = False
                elif self.btn_agent.clicked(pos):
                    self.mode = 'agent'
                    self.btn_obs.active = False; self.btn_agent.active = True

                # Controls
                elif self.btn_start.clicked(pos):  self._toggle_run()
                elif self.btn_task.clicked(pos):   self._add_task()
                elif self.btn_replan.clicked(pos): self._replan()

                # Algorithm
                elif self.btn_prio.clicked(pos):
                    self.algorithm = 'prioritized'
                    self.btn_prio.active = True; self.btn_cbs.active = False
                elif self.btn_cbs.clicked(pos):
                    self.algorithm = 'cbs'
                    self.btn_prio.active = False; self.btn_cbs.active = True

                # Heading selector
                elif any(b.clicked(pos) for b in self.btns_theta):
                    for i, b in enumerate(self.btns_theta):
                        if b.clicked(pos):
                            self.place_theta = i
                        b.active = (b == self.btns_theta[self.place_theta])

                # Presets / clear
                elif self.btn_preset.clicked(pos): self._load_preset('warehouse')
                elif self.btn_maze.clicked(pos):   self._load_preset('maze')
                elif self.btn_clear.clicked(pos):  self._clear_all()

                # Speed
                else:
                    for i, b in enumerate(self.btns_speed):
                        if b.clicked(pos):
                            self.spd_idx = i
                            for j, sb in enumerate(self.btns_speed):
                                sb.active = (j == i)

                    # Grid click
                    cell = self.cell_at(*pos)
                    if cell:
                        r, c = cell
                        agent_pos = {(a.r, a.c) for a in self.agents}
                        task_pos  = ({t.pickup   for t in self.task_mgr.active()} |
                                     {t.delivery for t in self.task_mgr.active()})

                        if self.mode == 'obstacle':
                            if (r,c) not in agent_pos and (r,c) not in task_pos:
                                self.grid.toggle(r, c)

                        elif self.mode == 'agent':
                            if (len(self.agents) < MAX_AGENTS and
                                    self.grid.is_free(r, c) and
                                    (r, c) not in agent_pos):
                                self.agents.append(
                                    PhysAgent(len(self.agents), r, c, self.place_theta))

    # ── Actions ───────────────────────────────────────────────────────────────

    def _toggle_run(self):
        self.running = not self.running

    def _add_task(self):
        agent_pos = [(a.r, a.c) for a in self.agents]
        task = self.task_mgr.generate(agent_pos)
        if task:
            self.task_mgr.assign_pending(self.agents)
            self._replan()

    def _replan(self):
        if self.algorithm == 'cbs':
            self.cbs_ok = plan_phy_cbs(self.grid, self.agents)
        else:
            plan_phy_all(self.grid, self.agents)
            self.cbs_ok = True

    def _load_preset(self, name):
        self._clear_all(); self.grid.load_preset(name)

    def _clear_all(self):
        self.running = False
        self._init_state()
        self._build_buttons()

    # ── Simulation step ───────────────────────────────────────────────────────

    def update(self):
        if not self.running: return
        self.frame_cnt += 1
        self.anim_t = min(self.frame_cnt / SPEED_FRAMES[self.spd_idx], 1.0)
        if self.frame_cnt >= SPEED_FRAMES[self.spd_idx]:
            self.frame_cnt = 0
            self.anim_t    = 0.0
            self._sim_step()

    def _sim_step(self):
        self.tick += 1

        # ── Physical conflict-aware movement ──────────────────────────────────
        intentions = {a.id: a.peek_next() for a in self.agents}
        claimed_fps = []   # list of (Shapely polygon, agent_id)
        will_move   = {}

        for agent in sorted(self.agents, key=lambda a: a.id):
            nr, nc, nth, nv = intentions[agent.id]
            nxt_fp = get_footprint(nr, nc, nth, agent.BODY_W, agent.BODY_L)

            blocked = any(nxt_fp.intersects(fp) for fp, _ in claimed_fps
                         if (nr, nc) != (agent.r, agent.c))  # allow staying

            if not blocked:
                will_move[agent.id] = True
                claimed_fps.append((nxt_fp, agent.id))
            else:
                will_move[agent.id] = False
                stay_fp = get_footprint(agent.r, agent.c, agent.theta,
                                        agent.BODY_W, agent.BODY_L)
                claimed_fps.append((stay_fp, agent.id))

        for agent in self.agents:
            if will_move[agent.id]: agent.advance()
            else:                   agent.stay()

        # ── Check arrivals ─────────────────────────────────────────────────────
        replan_needed = False
        for agent in self.agents:
            if not agent.at_destination(): continue

            if agent.status == TO_PICKUP and agent.task:
                if (agent.r, agent.c) == agent.task.pickup:
                    agent.status = TO_DELIVERY
                    replan_needed = True

            elif agent.status == TO_DELIVERY and agent.task:
                if (agent.r, agent.c) == agent.task.delivery:
                    self.task_mgr.complete_task(agent)

        # ── Assign + auto-generate ─────────────────────────────────────────────
        prev_idle = sum(1 for a in self.agents if a.status == IDLE)
        self.task_mgr.assign_pending(self.agents)
        if sum(1 for a in self.agents if a.status == IDLE) < prev_idle:
            replan_needed = True

        if self.tick % TASK_INTERVAL == 0 and self.agents:
            task = self.task_mgr.generate([(a.r, a.c) for a in self.agents])
            if task:
                self.task_mgr.assign_pending(self.agents)
                replan_needed = True

        # ── Replan if needed ───────────────────────────────────────────────────
        if not replan_needed:
            replan_needed = any(
                a.status != IDLE and not a.has_path()
                and a.goal() and (a.r, a.c) != a.goal()
                for a in self.agents)
        if replan_needed:
            self._replan()

    # ── Rendering ─────────────────────────────────────────────────────────────

    def render(self):
        self.screen.fill(BG)
        self._draw_grid()
        self._draw_tasks()
        self._draw_paths()
        self._draw_agents()
        self._draw_sidebar()
        pygame.display.flip()

    def _draw_grid(self):
        for r in range(ROWS):
            for c in range(COLS):
                col = CELL_OBS if self.grid.cells[r][c] else CELL_FREE
                pygame.draw.rect(self.screen, col, self.cell_rect(r, c), border_radius=2)
        for r in range(ROWS + 1):
            y = GRID_TOP + r*CELL
            pygame.draw.line(self.screen, GRID_LINE,
                             (GRID_LEFT, y), (GRID_LEFT+COLS*CELL, y))
        for c in range(COLS + 1):
            x = GRID_LEFT + c*CELL
            pygame.draw.line(self.screen, GRID_LINE,
                             (x, GRID_TOP), (x, GRID_TOP+ROWS*CELL))

    def _draw_tasks(self):
        agents_by_id = {a.id: a for a in self.agents}
        surf = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)
        for task in self.task_mgr.active():
            ac = (agents_by_id[task.agent_id].color
                  if task.agent_id in agents_by_id else None)
            for (tr, tc), marker, def_col in [
                    (task.pickup,   'P', COL_PICKUP),
                    (task.delivery, 'D', COL_DELIV)]:
                col = (*( ac or def_col), 175)
                rect = self.cell_rect(tr, tc)
                pygame.draw.rect(surf, col, rect, border_radius=3)
                lbl = self.font_s.render(marker, True, COL_WHITE)
                surf.blit(lbl, lbl.get_rect(center=rect.center))
                id_lbl = self.font_s.render(f'T{task.id}', True, (255,240,80))
                surf.blit(id_lbl, (rect.x+2, rect.y+1))
        self.screen.blit(surf, (0, 0))

    def _interp_state(self, agent):
        """Return (px, py, angle_deg, vel_frac) interpolated for current anim_t."""
        t = min(self.anim_t, 1.0)
        pr, pc = agent.prev_r, agent.prev_c
        cr, cc = agent.r, agent.c
        px = GRID_LEFT + (pc + (cc-pc)*t)*CELL + CELL/2
        py = GRID_TOP  + (pr + (cr-pr)*t)*CELL + CELL/2
        angle = lerp_angle(SCREEN_ANGLES[agent.prev_theta],
                           SCREEN_ANGLES[agent.theta], t)
        vf = (agent.prev_vel + (agent.vel - agent.prev_vel)*t) / V_MAX
        return int(px), int(py), angle, vf

    def _draw_paths(self):
        surf = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)
        for agent in self.agents:
            remaining = agent.path[agent.path_idx:]
            if not remaining: continue

            # Draw thin line connecting waypoints
            pts = [self.cell_center_px(agent.r, agent.c)]
            for s in remaining:
                pts.append(self.cell_center_px(s[0], s[1]))
            if len(pts) > 1:
                pygame.draw.lines(surf, (*agent.color, 60), False, pts, 2)

            # Draw direction arrows at each waypoint
            prev_theta = agent.theta
            for s in remaining:
                wr, wc, wth, wv = s
                wx, wy = self.cell_center_px(wr, wc)
                scr_ang = SCREEN_ANGLES[wth]
                # arrow color: brighter for higher speed
                alpha = 80 + int(wv / V_MAX * 120)
                draw_arrow(surf, (*agent.color, alpha), wx, wy, scr_ang, size=8)
                prev_theta = wth

        self.screen.blit(surf, (0, 0))

    def _draw_agents(self):
        bw = PhysAgent_BODY_W
        bl = PhysAgent_BODY_L
        px_w = bw * CELL
        px_l = bl * CELL

        for agent in self.agents:
            cx, cy, angle, vf = self._interp_state(agent)

            # Body fill (tinted by speed)
            fill = speed_tint(agent.color, vf)
            corners = draw_rotated_rect(self.screen, fill, cx, cy,
                                        px_l, px_w, angle,
                                        border=COL_WHITE, border_w=2)

            # "Headlights" — small dot at the front centre
            rad = math.radians(angle)
            front_x = cx + math.cos(rad) * px_l/2
            front_y = cy + math.sin(rad) * px_l/2
            pygame.draw.circle(self.screen, (255,255,180),
                               (int(front_x), int(front_y)), 4)

            # Agent ID centred in body
            lbl = self.font_m.render(str(agent.id), True, COL_WHITE)
            self.screen.blit(lbl, lbl.get_rect(center=(cx, cy)))

            # Velocity bar (small bar on top of agent)
            bar_len = int(px_l * vf)
            if bar_len > 0:
                bar_col = speed_tint((80, 180, 255), vf)
                # Draw along forward axis
                for i in range(bar_len):
                    bx = int(cx - px_l/2*math.cos(rad) + i*math.cos(rad))
                    by = int(cy - px_l/2*math.sin(rad) + i*math.sin(rad))
                # simpler: draw a small rect above agent
                pygame.draw.rect(self.screen, bar_col,
                                 (cx - 12, cy - int(px_w/2) - 5, bar_len, 4))

            # Status dot
            dot_col = {IDLE: (130,130,130), TO_PICKUP: COL_PICKUP,
                       TO_DELIVERY: COL_DELIV}.get(agent.status, (130,130,130))
            pygame.draw.circle(self.screen, dot_col,
                               (int(cx + px_l/2 - 4), int(cy - px_w/2 + 4)), 5)

    # ── Sidebar ───────────────────────────────────────────────────────────────

    def _draw_sidebar(self):
        pygame.draw.rect(self.screen, SIDE_BG,
                         pygame.Rect(SIDE_LEFT-PAD//2, 0, SIDE_W+PAD, WIN_H))
        for btn in self.all_buttons:
            btn.draw(self.screen, self.font_s)

        # Heading label
        th_y = self.btns_theta[0].rect.y - 16
        self.screen.blit(self.font_s.render('Place heading:', True, COL_DIM),
                         (SIDE_LEFT, th_y))

        # Speed label
        sp_y = self.btns_speed[0].rect.y - 16
        self.screen.blit(self.font_s.render('Speed:', True, COL_DIM),
                         (SIDE_LEFT, sp_y))

        # ── Stats ─────────────────────────────────────────────────────────────
        y, sx = self._info_y, SIDE_LEFT

        def line(text, color=COL_TEXT, font=None):
            nonlocal y
            f = font or self.font_m
            self.screen.blit(f.render(text, True, color), (sx, y))
            y += f.size(text)[1] + 3

        line('── Stats ──', font=self.font_b)
        line(f'Timestep : {self.tick}')
        state_txt = '▶ Running' if self.running else '⏸ Paused'
        line(state_txt, color=(100,220,100) if self.running else (170,170,170))

        if self.algorithm == 'cbs':
            lbl = 'CBS' if self.cbs_ok else 'CBS→Prioritized (fallback)'
            line(f'Planner  : {lbl}',
                 color=(100,200,255) if self.cbs_ok else (220,160,60))
        else:
            line('Planner  : Prioritized', color=(170,170,170))

        pending = len(self.task_mgr.pending())
        active  = len(self.task_mgr.active())
        done    = self.task_mgr.done_count
        line(f'Tasks    : {active} active, {done} done')
        line(f'           {pending} unassigned')
        y += 6

        # ── Agent list ────────────────────────────────────────────────────────
        line('── Agents ──', font=self.font_b)
        if not self.agents:
            line('(Agent mode → click grid)', color=COL_DIM, font=self.font_s)
        else:
            for agent in self.agents:
                pygame.draw.rect(self.screen, agent.color,
                                 pygame.Rect(sx, y+3, 11, 11), border_radius=2)
                heading = THETA_NAMES[agent.theta]
                vel_bar = '█' * agent.vel + '░' * (V_MAX - agent.vel)
                status_str = {
                    IDLE:        'idle',
                    TO_PICKUP:   f'→P T{agent.task.id}' if agent.task else '→P',
                    TO_DELIVERY: f'→D T{agent.task.id}' if agent.task else '→D',
                }.get(agent.status, '?')
                txt = f' A{agent.id}[{heading}] v={agent.vel} {status_str}'
                self.screen.blit(self.font_s.render(txt, True, COL_TEXT), (sx+13, y))
                y += 17
        y += 8

        # ── Legend ────────────────────────────────────────────────────────────
        line('── Legend ──', font=self.font_b)
        for col, lbl in [(COL_PICKUP,    'P — pickup'),
                         (COL_DELIV,     'D — delivery'),
                         ((80,180,255),  'arrow — heading & speed'),
                         ((200,200,100), '● — headlight (front)')]:
            pygame.draw.rect(self.screen, col, pygame.Rect(sx, y+3, 12, 12), border_radius=2)
            self.screen.blit(self.font_s.render(' '+lbl, True, COL_DIM), (sx+12, y))
            y += 17
        y += 6

        # ── Keys ──────────────────────────────────────────────────────────────
        line('── Keys ──', font=self.font_b)
        for h in ['SPACE — start/pause', 'T — add task', 'R — replan all']:
            line(h, color=COL_DIM, font=self.font_s)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        while True:
            self.handle_events()
            self.update()
            self.render()
            self.clock.tick(FPS)


if __name__ == '__main__':
    PhysSimulator().run()
