"""
main.py — MAPF Educational Simulator (Pygame)

Controls:
  SPACE       Start / Pause
  T           Add a random task
  R           Replan all agents
  Left-click  Place obstacle (Obstacle mode) / Place agent (Agent mode)

UI:
  Left panel  — 15×15 grid
  Right panel — controls & stats
"""
import sys
import pygame

from core import (GridWorld, Agent, TaskManager,
                  IDLE, TO_PICKUP, TO_DELIVERY, AGENT_COLORS)
from pathfinder import plan_all, plan_cbs, astar

# ── Layout constants ───────────────────────────────────────────────────────────
ROWS, COLS   = 15, 15
CELL         = 40           # px per grid cell
PAD          = 10           # outer padding
SIDE_W       = 255          # sidebar width
WIN_W        = PAD + COLS * CELL + PAD + SIDE_W + PAD
WIN_H        = PAD + ROWS * CELL + PAD
GRID_LEFT    = PAD
GRID_TOP     = PAD
SIDE_LEFT    = GRID_LEFT + COLS * CELL + PAD   # sidebar x-start

FPS          = 30
MAX_AGENTS   = 8
TASK_INTERVAL= 60           # sim-ticks between auto task generation

# Speed table: label → frames-per-sim-step
SPEED_LABELS  = ['▶', '▶▶', '▶▶▶', '⚡']
SPEED_FRAMES  = [20,  10,    4,     1]   # frames per sim step

# ── Colour palette ─────────────────────────────────────────────────────────────
BG          = (18,  18,  24)
SIDE_BG     = (26,  26,  34)
CELL_FREE   = (228, 228, 232)
CELL_OBS    = (52,  52,  62)
GRID_LINE   = (160, 160, 170)
COL_PICKUP  = (45,  210, 80)
COL_DELIV   = (220, 60,  60)
COL_TEXT    = (215, 215, 225)
COL_DIM     = (120, 120, 135)
COL_BTN     = (50,  72,  105)
COL_HOVER   = (70,  100, 148)
COL_ACTIVE  = (85,  130, 190)
COL_DANGER  = (140, 50,  50)
COL_WHITE   = (255, 255, 255)


# ── Button ────────────────────────────────────────────────────────────────────

class Button:
    def __init__(self, rect, label, base=None, toggle=False):
        self.rect   = pygame.Rect(rect)
        self.label  = label
        self.base   = base or COL_BTN
        self.toggle = toggle
        self.active = False
        self._hover = False

    def update_hover(self, pos):
        self._hover = self.rect.collidepoint(pos)

    def clicked(self, pos):
        return self.rect.collidepoint(pos)

    def draw(self, surf, font):
        if self.toggle and self.active:
            col = COL_ACTIVE
        elif self._hover:
            col = COL_HOVER
        else:
            col = self.base
        pygame.draw.rect(surf, col,          self.rect, border_radius=5)
        pygame.draw.rect(surf, (180, 190, 210), self.rect, 1, border_radius=5)
        txt = font.render(self.label, True, COL_WHITE)
        surf.blit(txt, txt.get_rect(center=self.rect.center))


# ── Simulator ─────────────────────────────────────────────────────────────────

class Simulator:

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption('MAPF Simulator — Educational')
        self.clock  = pygame.time.Clock()

        self.font_s = pygame.font.SysFont('Consolas', 13)
        self.font_m = pygame.font.SysFont('Consolas', 15)
        self.font_b = pygame.font.SysFont('Consolas', 16, bold=True)

        self._init_state()
        self._build_buttons()

    # ── State initialisation ───────────────────────────────────────────────────

    def _init_state(self):
        self.grid      = GridWorld(ROWS, COLS)
        self.agents    = []
        self.task_mgr  = TaskManager(self.grid)
        self.mode      = 'obstacle'      # 'obstacle' | 'agent'
        self.algorithm = 'prioritized'   # 'prioritized' | 'cbs'
        self.cbs_ok    = True            # False = last CBS fell back to prioritized
        self.running   = False
        self.tick      = 0
        self.spd_idx   = 1            # index into SPEED_FRAMES
        self.frame_cnt = 0            # frames since last sim step
        self.anim_t    = 0.0          # 0→1 within a step (for smooth animation)

    # ── Button layout ──────────────────────────────────────────────────────────

    def _build_buttons(self):
        sx  = SIDE_LEFT
        sw  = SIDE_W - PAD        # usable sidebar width
        y   = PAD
        H   = 30                  # button height
        GAP = 6

        half = (sw - 4) // 2

        self.btn_mode_obs   = Button((sx,          y, half, H), '✏ Obstacle',  toggle=True)
        self.btn_mode_agent = Button((sx+half+4,   y, half, H), '🤖 Agent',    toggle=True)
        self.btn_mode_obs.active = True
        y += H + GAP

        self.btn_start  = Button((sx, y, sw, H), '▶ Start / Pause')
        y += H + GAP
        self.btn_task   = Button((sx, y, sw, H), '＋ Add Task  [T]')
        y += H + GAP
        self.btn_replan = Button((sx, y, sw, H), '↺  Replan   [R]')
        y += H + GAP

        # Algorithm toggle
        self.btn_prio = Button((sx,          y, half, H), 'Prioritized', toggle=True)
        self.btn_cbs  = Button((sx+half+4,   y, half, H), 'CBS',         toggle=True)
        self.btn_prio.active = True
        y += H + GAP

        # Preset & Clear on same row
        pw = (sw - 4) // 2
        self.btn_preset = Button((sx,      y, pw, H), '⬛ Warehouse')
        self.btn_maze   = Button((sx+pw+4, y, pw, H), '🌀 Maze')
        y += H + GAP
        self.btn_clear  = Button((sx, y, sw, H), '🗑  Clear All', base=COL_DANGER)
        y += H + GAP + 2

        # Speed buttons
        bw = sw // len(SPEED_LABELS)
        self.btns_speed = []
        for i, lbl in enumerate(SPEED_LABELS):
            b = Button((sx + i * bw, y, bw - 2, H - 4), lbl, toggle=True)
            b.active = (i == self.spd_idx)
            self.btns_speed.append(b)
        y += H + GAP

        self._side_info_y = y + 10  # where stats begin

        self.all_buttons = [
            self.btn_mode_obs, self.btn_mode_agent,
            self.btn_start, self.btn_task, self.btn_replan,
            self.btn_prio, self.btn_cbs,
            self.btn_preset, self.btn_maze, self.btn_clear,
            *self.btns_speed,
        ]

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def cell_at(self, px, py):
        c = (px - GRID_LEFT) // CELL
        r = (py - GRID_TOP)  // CELL
        if 0 <= r < ROWS and 0 <= c < COLS:
            return r, c
        return None

    def cell_px(self, r, c):
        """Top-left pixel of cell (r,c)."""
        return GRID_LEFT + c * CELL, GRID_TOP + r * CELL

    def cell_center(self, r, c):
        x, y = self.cell_px(r, c)
        return x + CELL // 2, y + CELL // 2

    def cell_rect(self, r, c):
        x, y = self.cell_px(r, c)
        return pygame.Rect(x + 1, y + 1, CELL - 2, CELL - 2)

    # ── Event handling ────────────────────────────────────────────────────────

    def handle_events(self):
        mpos = pygame.mouse.get_pos()
        for btn in self.all_buttons:
            btn.update_hover(mpos)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()

            # ── Keyboard shortcuts ────────────────────────────────────────────
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self._toggle_run()
                elif event.key == pygame.K_t:
                    self._add_task()
                elif event.key == pygame.K_r:
                    self._replan()

            # ── Mouse click ───────────────────────────────────────────────────
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                pos = event.pos

                # — Mode buttons
                if self.btn_mode_obs.clicked(pos):
                    self.mode = 'obstacle'
                    self.btn_mode_obs.active   = True
                    self.btn_mode_agent.active = False

                elif self.btn_mode_agent.clicked(pos):
                    self.mode = 'agent'
                    self.btn_mode_obs.active   = False
                    self.btn_mode_agent.active = True

                # — Control buttons
                elif self.btn_start.clicked(pos):
                    self._toggle_run()
                elif self.btn_task.clicked(pos):
                    self._add_task()
                elif self.btn_replan.clicked(pos):
                    self._replan()

                # — Algorithm toggle
                elif self.btn_prio.clicked(pos):
                    self.algorithm = 'prioritized'
                    self.btn_prio.active = True
                    self.btn_cbs.active  = False
                elif self.btn_cbs.clicked(pos):
                    self.algorithm = 'cbs'
                    self.btn_prio.active = False
                    self.btn_cbs.active  = True

                elif self.btn_preset.clicked(pos):
                    self._load_preset('warehouse')
                elif self.btn_maze.clicked(pos):
                    self._load_preset('maze')
                elif self.btn_clear.clicked(pos):
                    self._clear_all()

                # — Speed buttons
                else:
                    for i, b in enumerate(self.btns_speed):
                        if b.clicked(pos):
                            self.spd_idx = i
                            for j, sb in enumerate(self.btns_speed):
                                sb.active = (j == i)

                    # — Grid click
                    cell = self.cell_at(*pos)
                    if cell:
                        r, c = cell
                        agent_pos = {a.pos for a in self.agents}
                        task_pos  = ({t.pickup   for t in self.task_mgr.active()} |
                                     {t.delivery for t in self.task_mgr.active()})

                        if self.mode == 'obstacle':
                            if (r, c) not in agent_pos and (r, c) not in task_pos:
                                self.grid.toggle(r, c)

                        elif self.mode == 'agent':
                            can_place = (
                                len(self.agents) < MAX_AGENTS and
                                self.grid.is_free(r, c) and
                                (r, c) not in agent_pos
                            )
                            if can_place:
                                self.agents.append(Agent(len(self.agents), (r, c)))

    # ── Actions ───────────────────────────────────────────────────────────────

    def _toggle_run(self):
        self.running = not self.running

    def _add_task(self):
        agent_pos = [a.pos for a in self.agents]
        task = self.task_mgr.generate(agent_pos)
        if task:
            self.task_mgr.assign_pending(self.agents)
            self._replan()

    def _replan(self):
        if self.algorithm == 'cbs':
            self.cbs_ok = plan_cbs(self.grid, self.agents)
        else:
            plan_all(self.grid, self.agents)
            self.cbs_ok = True

    def _load_preset(self, name):
        self._clear_all()
        self.grid.load_preset(name)

    def _clear_all(self):
        self.running = False
        self._init_state()
        self._build_buttons()   # reset button states

    # ── Simulation step ───────────────────────────────────────────────────────

    def update(self):
        if not self.running:
            return

        self.frame_cnt += 1
        self.anim_t = min(self.frame_cnt / SPEED_FRAMES[self.spd_idx], 1.0)

        if self.frame_cnt >= SPEED_FRAMES[self.spd_idx]:
            self.frame_cnt = 0
            self.anim_t    = 0.0
            self._sim_step()

    def _sim_step(self):
        self.tick += 1

        # ── Conflict-aware movement (priority = lower agent id) ────────────────
        # First pass: each agent declares its intended next cell
        intentions = {a.id: a.peek_next() for a in self.agents}

        # Cells claimed by higher-priority agents (processed first)
        claimed = set()
        will_move = {}   # agent.id → True/False

        for agent in sorted(self.agents, key=lambda a: a.id):
            nxt = intentions[agent.id]
            if nxt == agent.pos:
                # Staying (no path / waiting)
                will_move[agent.id] = False
                claimed.add(nxt)
            elif nxt not in claimed:
                will_move[agent.id] = True
                claimed.add(nxt)
                claimed.discard(agent.pos)
            else:
                # Blocked — wait
                will_move[agent.id] = False
                claimed.add(agent.pos)

        # Apply movement
        for agent in self.agents:
            if will_move[agent.id]:
                agent.advance()
            else:
                agent.stay()

        # ── Check arrivals ─────────────────────────────────────────────────────
        replan_needed = False
        for agent in self.agents:
            if not agent.at_destination():
                continue

            if agent.status == TO_PICKUP and agent.task:
                if agent.pos == agent.task.pickup:
                    agent.status = TO_DELIVERY
                    replan_needed = True   # need path to delivery

            elif agent.status == TO_DELIVERY and agent.task:
                if agent.pos == agent.task.delivery:
                    self.task_mgr.complete_task(agent)

        # ── Assign pending tasks to newly idle agents ──────────────────────────
        prev_idle = sum(1 for a in self.agents if a.status == IDLE)
        self.task_mgr.assign_pending(self.agents)
        new_assigned = prev_idle - sum(1 for a in self.agents if a.status == IDLE)
        if new_assigned > 0:
            replan_needed = True

        # ── Auto-generate tasks periodically ──────────────────────────────────
        if self.tick % TASK_INTERVAL == 0 and self.agents:
            agent_pos = [a.pos for a in self.agents]
            task = self.task_mgr.generate(agent_pos)
            if task:
                self.task_mgr.assign_pending(self.agents)
                replan_needed = True

        # ── Re-plan when needed ────────────────────────────────────────────────
        # Also replan if any agent has a task but no path left
        if not replan_needed:
            replan_needed = any(
                a.status != IDLE and not a.has_path()
                and a.goal() and a.pos != a.goal()
                for a in self.agents
            )

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
                rect = self.cell_rect(r, c)
                col  = CELL_OBS if self.grid.cells[r][c] else CELL_FREE
                pygame.draw.rect(self.screen, col, rect, border_radius=2)

        # Grid lines
        for r in range(ROWS + 1):
            y = GRID_TOP + r * CELL
            pygame.draw.line(self.screen, GRID_LINE,
                             (GRID_LEFT, y), (GRID_LEFT + COLS * CELL, y))
        for c in range(COLS + 1):
            x = GRID_LEFT + c * CELL
            pygame.draw.line(self.screen, GRID_LINE,
                             (x, GRID_TOP), (x, GRID_TOP + ROWS * CELL))

    def _draw_tasks(self):
        agents_by_id = {a.id: a for a in self.agents}
        surf = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)

        for task in self.task_mgr.active():
            # Colour: agent colour if assigned, else default pickup/delivery colours
            agent_col = None
            if task.agent_id is not None and task.agent_id in agents_by_id:
                agent_col = agents_by_id[task.agent_id].color

            p_col = (*( agent_col or COL_PICKUP),  180)
            d_col = (*( agent_col or COL_DELIV),   180)

            # Pickup square
            pr, pc   = task.pickup
            p_rect   = self.cell_rect(pr, pc)
            pygame.draw.rect(surf, p_col,  p_rect, border_radius=3)
            p_txt = self.font_s.render('P', True, COL_WHITE)
            surf.blit(p_txt, p_txt.get_rect(center=p_rect.center))

            # Delivery square
            dr, dc   = task.delivery
            d_rect   = self.cell_rect(dr, dc)
            pygame.draw.rect(surf, d_col,  d_rect, border_radius=3)
            d_txt = self.font_s.render('D', True, COL_WHITE)
            surf.blit(d_txt, d_txt.get_rect(center=d_rect.center))

            # Task ID label (top-left of cell)
            id_lbl = self.font_s.render(f'T{task.id}', True, (255, 240, 80))
            surf.blit(id_lbl, (p_rect.x + 2, p_rect.y + 1))
            surf.blit(id_lbl, (d_rect.x + 2, d_rect.y + 1))

        self.screen.blit(surf, (0, 0))

    def _draw_paths(self):
        surf = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)
        for agent in self.agents:
            remaining = agent.path[agent.path_idx:]
            if not remaining:
                continue
            pts = [self.cell_center(*agent.pos)] + [self.cell_center(*p) for p in remaining]
            if len(pts) > 1:
                pygame.draw.lines(surf, (*agent.color, 70), False, pts, 2)
            for pt in pts[1:]:
                pygame.draw.circle(surf, (*agent.color, 110), pt, 4)
        self.screen.blit(surf, (0, 0))

    def _interp_center(self, agent):
        """Smooth pixel position interpolated between prev_pos and pos."""
        t       = min(self.anim_t, 1.0)
        pr, pc  = agent.prev_pos
        cr, cc  = agent.pos
        px = GRID_LEFT + (pc + (cc - pc) * t) * CELL + CELL / 2
        py = GRID_TOP  + (pr + (cr - pr) * t) * CELL + CELL / 2
        return int(px), int(py)

    def _draw_agents(self):
        R = CELL // 2 - 4
        for agent in self.agents:
            px, py = self._interp_center(agent)

            # Drop shadow
            pygame.draw.circle(self.screen, (0, 0, 0), (px + 2, py + 2), R)

            # Body
            pygame.draw.circle(self.screen, agent.color,   (px, py), R)
            pygame.draw.circle(self.screen, COL_WHITE,     (px, py), R, 2)

            # ID
            lbl = self.font_m.render(str(agent.id), True, COL_WHITE)
            self.screen.blit(lbl, lbl.get_rect(center=(px, py)))

            # Status dot (top-right of circle)
            dot_col = {IDLE: (140, 140, 140),
                       TO_PICKUP:   COL_PICKUP,
                       TO_DELIVERY: COL_DELIV}.get(agent.status, (140,140,140))
            pygame.draw.circle(self.screen, dot_col, (px + R - 3, py - R + 3), 5)

    # ── Sidebar ───────────────────────────────────────────────────────────────

    def _draw_sidebar(self):
        # Background panel
        pygame.draw.rect(self.screen, SIDE_BG,
                         pygame.Rect(SIDE_LEFT - PAD//2, 0, SIDE_W + PAD, WIN_H))

        # Buttons
        for btn in self.all_buttons:
            btn.draw(self.screen, self.font_s)

        # Speed label above speed buttons
        spd_y = self.btns_speed[0].rect.y - 18
        self.screen.blit(self.font_s.render('Speed:', True, COL_DIM),
                         (SIDE_LEFT, spd_y))

        # ── Stats ─────────────────────────────────────────────────────────────
        y  = self._side_info_y
        sx = SIDE_LEFT

        def line(text, color=COL_TEXT, font=None):
            nonlocal y
            f = font or self.font_m
            self.screen.blit(f.render(text, True, color), (sx, y))
            y += f.size(text)[1] + 3

        line('── Stats ──', font=self.font_b)
        line(f'Timestep : {self.tick}')
        state = '▶ Running' if self.running else '⏸ Paused'
        line(state, color=(100, 220, 100) if self.running else (180, 180, 180))

        if self.algorithm == 'cbs':
            cbs_label = 'CBS' if self.cbs_ok else 'CBS→Prioritized'
            cbs_col   = (100, 200, 255) if self.cbs_ok else (220, 160, 60)
            line(f'Planner  : {cbs_label}', color=cbs_col)
        else:
            line('Planner  : Prioritized', color=(180, 180, 180))

        pending = len(self.task_mgr.pending())
        active  = len(self.task_mgr.active())
        done    = self.task_mgr.done_count
        line(f'Tasks    : {active} active, {done} done')
        line(f'           {pending} waiting assignment')
        y += 6

        # ── Agent list ────────────────────────────────────────────────────────
        line('── Agents ──', font=self.font_b)

        if not self.agents:
            line('(Switch to Agent mode,', color=COL_DIM, font=self.font_s)
            line(' click grid to add)',   color=COL_DIM, font=self.font_s)
        else:
            for agent in self.agents:
                # Colour swatch
                pygame.draw.rect(self.screen, agent.color,
                                 pygame.Rect(sx, y + 3, 11, 11), border_radius=2)
                status_str = {
                    IDLE:        'idle',
                    TO_PICKUP:   f'→ P  T{agent.task.id}' if agent.task else '→ P',
                    TO_DELIVERY: f'→ D  T{agent.task.id}' if agent.task else '→ D',
                }.get(agent.status, '?')
                txt = f' A{agent.id}: {status_str}  [{agent.tasks_done}✓]'
                self.screen.blit(self.font_s.render(txt, True, COL_TEXT), (sx + 13, y))
                y += 17

        y += 8

        # ── Legend ────────────────────────────────────────────────────────────
        line('── Legend ──', font=self.font_b)

        def legend_row(col, label):
            nonlocal y
            pygame.draw.rect(self.screen, col, pygame.Rect(sx, y+3, 12, 12), border_radius=2)
            self.screen.blit(self.font_s.render(' ' + label, True, COL_DIM), (sx+12, y))
            y += 17

        legend_row(COL_PICKUP,  'P  — Task pickup')
        legend_row(COL_DELIV,   'D  — Task delivery')
        legend_row((120,120,120),'·  — Planned path')

        y += 8
        # ── Key hints ─────────────────────────────────────────────────────────
        line('── Keys ──', font=self.font_b)
        for hint in ['SPACE — start/pause',
                     'T     — add task',
                     'R     — replan all']:
            line(hint, color=COL_DIM, font=self.font_s)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        while True:
            self.handle_events()
            self.update()
            self.render()
            self.clock.tick(FPS)


if __name__ == '__main__':
    Simulator().run()
