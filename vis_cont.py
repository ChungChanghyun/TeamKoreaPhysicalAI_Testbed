"""
vis_cont.py — Pygame visualizer for the continuous-time MAPF environment.

sim_time is a float (seconds) advanced by  dt_real × sim_speed  every frame.
Agent positions are interpolated at the exact sim_time — no discrete steps.

Controls
--------
  SPACE   Start / Pause
  R       Reset sim_time to 0
  +/-     Increase / decrease sim speed
"""
from __future__ import annotations
import sys
import math
import pygame

from map_loader import MapGraph
from env_cont import (
    KinematicAgent, MoveManeuver, RotateManeuver, WaitManeuver,
    MOVING, A_MAX,
)

# ── Window layout ──────────────────────────────────────────────────────────────
MAP_W   = 820       # pixels reserved for the map canvas
SIDE_W  = 280       # sidebar width
WIN_W   = MAP_W + SIDE_W
WIN_H   = 700
MAP_PAD = 50        # padding inside the map canvas (px)
FPS     = 60

SIM_SPEEDS = [0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0]
SIM_SPEED_LABELS = ['×0.1', '×0.25', '×0.5', '×1', '×2', '×5', '×10', '×20']
DEFAULT_SPEED_IDX = 3   # ×1

# ── Colour palette ─────────────────────────────────────────────────────────────
BG         = (18,  18,  24)
SIDE_BG    = (26,  26,  34)
COL_EDGE   = (65,  65,  85)
COL_EDGE_A = (100, 100, 130)   # arrow on edge
COL_NODE   = (180, 180, 195)
COL_NODE_B = (220, 220, 235)   # node border
COL_PORT   = (45,  210,  80)
COL_PATH   = None              # filled per-agent (agent.color with alpha)
COL_TEXT   = (215, 215, 225)
COL_DIM    = (110, 110, 130)
COL_BTN    = (50,  72,  105)
COL_HOVER  = (70,  100, 148)
COL_ACTIVE = (85,  130, 190)
COL_DANGER = (130,  45,  45)
COL_WHITE  = (255, 255, 255)
COL_YELLOW = (255, 235,  80)

NODE_R_MM  = 200.0   # visual radius of nodes in map-mm


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
        col = (COL_ACTIVE if (self.toggle and self.active) else
               COL_HOVER  if self._hover else self.base)
        pygame.draw.rect(surf, col,              self.rect, border_radius=5)
        pygame.draw.rect(surf, (175, 185, 210),  self.rect, 1, border_radius=5)
        lbl = font.render(self.label, True, COL_WHITE)
        surf.blit(lbl, lbl.get_rect(center=self.rect.center))


# ── Simulator ──────────────────────────────────────────────────────────────────

class ContSimulator:
    """
    Pygame visualizer for a continuous-time MAPF environment.

    Parameters
    ----------
    graph  : MapGraph
    agents : list[KinematicAgent]
    """

    def __init__(self, graph: MapGraph, agents: list):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption('Continuous MAPF Simulator')
        self.clock = pygame.time.Clock()

        self.font_s = pygame.font.SysFont('Consolas', 13)
        self.font_m = pygame.font.SysFont('Consolas', 15)
        self.font_b = pygame.font.SysFont('Consolas', 16, bold=True)

        self.graph   = graph
        self.agents  = agents

        self.sim_time = 0.0
        self.running  = False
        self.spd_idx  = DEFAULT_SPEED_IDX

        self._build_transform()
        self._build_buttons()

    # ── Coordinate transform ───────────────────────────────────────────────────

    def _build_transform(self):
        """Compute scale + offset to fit the map in the canvas with padding."""
        x0, y0, x1, y1 = self.graph.bbox
        map_w = x1 - x0
        map_h = y1 - y0

        avail_w = MAP_W  - 2 * MAP_PAD
        avail_h = WIN_H  - 2 * MAP_PAD

        if map_w > 0 and map_h > 0:
            self._scale = min(avail_w / map_w, avail_h / map_h)
        else:
            self._scale = 0.05   # fallback

        # Offset so the map centre aligns with the canvas centre
        map_cx = (x0 + x1) / 2.0
        map_cy = (y0 + y1) / 2.0
        # screen: y increases downward → flip y
        self._ox = MAP_W / 2.0 - map_cx * self._scale
        self._oy = WIN_H / 2.0 + map_cy * self._scale

    def to_screen(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        """Map-space mm → screen pixels (y-axis flipped)."""
        return (int(self._ox + x_mm * self._scale),
                int(self._oy - y_mm * self._scale))

    def px(self, mm: float) -> float:
        """Convert a mm distance to pixels."""
        return mm * self._scale

    # ── Button layout ──────────────────────────────────────────────────────────

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
        bw   = (sw + 2) // 4
        row1 = SIM_SPEED_LABELS[:4]
        row2 = SIM_SPEED_LABELS[4:]
        self.btns_speed = []
        for row_i, row in enumerate([row1, row2]):
            for col_i, lbl in enumerate(row):
                idx = row_i * 4 + col_i
                b   = Button((sx + col_i * bw, y, bw - 2, H - 4), lbl, toggle=True)
                b.active = (idx == self.spd_idx)
                self.btns_speed.append(b)
            y += H - 4 + 2
        y += G + 4

        self._info_y    = y
        self.all_buttons = [self.btn_start, self.btn_reset, *self.btns_speed]

    # ── Events ─────────────────────────────────────────────────────────────────

    def handle_events(self):
        mpos = pygame.mouse.get_pos()
        for b in self.all_buttons:
            b.update_hover(mpos)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.running = not self.running
                elif event.key == pygame.K_r:
                    self._reset()
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    self._set_speed(min(self.spd_idx + 1, len(SIM_SPEEDS) - 1))
                elif event.key == pygame.K_MINUS:
                    self._set_speed(max(self.spd_idx - 1, 0))

            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                pos = event.pos
                if self.btn_start.clicked(pos):
                    self.running = not self.running
                elif self.btn_reset.clicked(pos):
                    self._reset()
                else:
                    for i, b in enumerate(self.btns_speed):
                        if b.clicked(pos):
                            self._set_speed(i)

    def _set_speed(self, idx: int):
        self.spd_idx = idx
        for i, b in enumerate(self.btns_speed):
            b.active = (i == idx)

    def _reset(self):
        self.sim_time = 0.0
        self.running  = False
        for agent in self.agents:
            agent.update(0.0)

    # ── Update ─────────────────────────────────────────────────────────────────

    def update(self, dt_real: float):
        if not self.running:
            return
        self.sim_time += dt_real * SIM_SPEEDS[self.spd_idx]
        for agent in self.agents:
            agent.update(self.sim_time)

    # ── Render ─────────────────────────────────────────────────────────────────

    def render(self):
        self.screen.fill(BG)
        pygame.draw.rect(self.screen, SIDE_BG,
                         pygame.Rect(MAP_W, 0, SIDE_W, WIN_H))

        self._draw_edges()
        self._draw_agent_paths()
        self._draw_nodes()
        self._draw_agents()
        self._draw_sidebar()
        pygame.display.flip()

    # ── Map drawing ────────────────────────────────────────────────────────────

    def _draw_edges(self):
        for (fn, tn), edge in self.graph.edges.items():
            fn_n = self.graph.nodes[fn]
            tn_n = self.graph.nodes[tn]
            p0   = self.to_screen(fn_n.x, fn_n.y)
            p1   = self.to_screen(tn_n.x, tn_n.y)

            pygame.draw.line(self.screen, COL_EDGE, p0, p1, 2)

            # Direction arrow at 60% along the edge
            mx = p0[0] + (p1[0] - p0[0]) * 0.6
            my = p0[1] + (p1[1] - p0[1]) * 0.6
            # screen-space angle: flip y (map y-up → screen y-down)
            scr_ang = math.atan2(-(tn_n.y - fn_n.y), tn_n.x - fn_n.x)
            _draw_arrow(self.screen, COL_EDGE_A, mx, my, scr_ang, size=7)

    def _draw_nodes(self):
        r_px = max(5, int(self.px(NODE_R_MM)))
        r_px = min(r_px, 14)

        for nid, node in self.graph.nodes.items():
            sx, sy = self.to_screen(node.x, node.y)
            is_port = nid in self.graph.ports.values()
            col = COL_PORT if is_port else COL_NODE
            pygame.draw.circle(self.screen, col,     (sx, sy), r_px)
            pygame.draw.circle(self.screen, COL_NODE_B, (sx, sy), r_px, 1)

            # Node label
            lbl = self.font_s.render(nid, True, COL_DIM)
            self.screen.blit(lbl, (sx + r_px + 2, sy - 7))

    def _draw_agent_paths(self):
        """Draw planned-path edges for each agent with direction arrows."""
        import math as _math
        surf = pygame.Surface((MAP_W, WIN_H), pygame.SRCALPHA)
        cbs_mode = getattr(self, '_cbs_mode', False)
        line_alpha = 160 if cbs_mode else 80
        node_alpha = 200 if cbs_mode else 100
        line_width = 3   if cbs_mode else 2

        for agent in self.agents:
            path = agent.node_path
            if len(path) < 2:
                continue
            pts = [self.to_screen(self.graph.nodes[n].x,
                                  self.graph.nodes[n].y)
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
                ang = _math.atan2(y1 - y0, x1 - x0)
                size = 6
                cos_a, sin_a = _math.cos(ang), _math.sin(ang)
                tip = (mx + cos_a * size,       my + sin_a * size)
                bl  = (mx - cos_a * size * 0.5 + (-sin_a) * size * 0.6,
                       my - sin_a * size * 0.5 + cos_a    * size * 0.6)
                br  = (mx - cos_a * size * 0.5 - (-sin_a) * size * 0.6,
                       my - sin_a * size * 0.5 - cos_a    * size * 0.6)
                pygame.draw.polygon(surf, col_line, [tip, bl, br])

            for pt in pts[1:-1]:
                pygame.draw.circle(surf, col_node, pt, 4)

        self.screen.blit(surf, (0, 0))

    # ── Agent drawing ──────────────────────────────────────────────────────────

    def _draw_agents(self):
        w_px = self.px(self.graph.vehicle_width)
        l_px = self.px(self.graph.vehicle_length)
        # Clamp to a visible minimum
        w_px = max(w_px, 8)
        l_px = max(l_px, 12)

        # Max speed across all edges (for velocity tint normalisation)
        v_max_all = max((e.max_speed for e in self.graph.edges.values()),
                        default=1000.0)

        for agent in self.agents:
            sx, sy  = self.to_screen(agent.x, agent.y)
            # y-flip the angle for screen space
            scr_ang = math.degrees(-agent.theta)

            vf   = min(agent.v / v_max_all, 1.0) if v_max_all > 0 else 0.0
            fill = _speed_tint(agent.color, vf)

            _draw_rotated_rect(self.screen, fill, sx, sy,
                               l_px, w_px, scr_ang,
                               border=COL_WHITE, border_w=2)

            # Headlight dot at the front
            rad   = math.radians(scr_ang)
            hx    = sx + math.cos(rad) * l_px / 2
            hy    = sy + math.sin(rad) * l_px / 2
            pygame.draw.circle(self.screen, (255, 255, 180),
                               (int(hx), int(hy)), max(3, int(w_px * 0.18)))

            # Agent ID
            lbl = self.font_s.render(str(agent.id), True, COL_WHITE)
            self.screen.blit(lbl, lbl.get_rect(center=(sx, sy)))

            # Status ring
            man  = agent.maneuver_type()
            ring = {'move': agent.color, 'rotate': COL_YELLOW}.get(man, COL_DIM)
            pygame.draw.circle(self.screen, ring,
                               (sx + int(l_px / 2) - 2, sy - int(w_px / 2) + 2), 4)

    # ── Sidebar ────────────────────────────────────────────────────────────────

    def _draw_sidebar(self):
        for b in self.all_buttons:
            b.draw(self.screen, self.font_s)

        # Speed label
        sp_lbl = self.font_s.render('Sim speed:', True, COL_DIM)
        sp_y   = self.btns_speed[0].rect.y - 16
        self.screen.blit(sp_lbl, (MAP_W + 10, sp_y))

        y  = self._info_y
        sx = MAP_W + 10

        def line(text, color=COL_TEXT, font=None):
            nonlocal y
            f = font or self.font_m
            self.screen.blit(f.render(text, True, color), (sx, y))
            y += f.size(text)[1] + 3

        # ── Simulation info ───────────────────────────────────────────────────
        line('── Simulation ──', font=self.font_b)
        line(f'Time   : {self.sim_time:8.2f} s')
        spd_lbl = SIM_SPEED_LABELS[self.spd_idx]
        line(f'Speed  : {spd_lbl}')
        state = '▶ Running' if self.running else '⏸ Paused'
        line(state, color=(100, 220, 100) if self.running else (170, 170, 170))
        y += 6

        # ── Agents ────────────────────────────────────────────────────────────
        line('── Agents ──', font=self.font_b)
        for agent in self.agents:
            pygame.draw.rect(self.screen, agent.color,
                             pygame.Rect(sx, y + 3, 11, 11), border_radius=2)
            man  = agent.maneuver_type()
            icon = {'move': '▶', 'rotate': '↻', 'wait': '■'}.get(man, '?')
            txt  = (f' A{agent.id} {icon}  '
                    f'v={agent.v/1000:.2f}m/s')
            self.screen.blit(self.font_s.render(txt, True, COL_TEXT), (sx + 13, y))
            y += 16
            pos_txt = (f'   ({agent.x/1000:.1f}, {agent.y/1000:.1f}) m  '
                       f'θ={math.degrees(agent.theta):.0f}°')
            self.screen.blit(self.font_s.render(pos_txt, True, COL_DIM), (sx, y))
            y += 18
        y += 6

        # ── Map info ──────────────────────────────────────────────────────────
        line('── Map ──', font=self.font_b)
        line(f'Nodes  : {len(self.graph.nodes)}',  color=COL_DIM, font=self.font_s)
        line(f'Edges  : {len(self.graph.edges)}',  color=COL_DIM, font=self.font_s)
        line(f'Ports  : {len(self.graph.ports)}',  color=COL_DIM, font=self.font_s)
        vw = self.graph.vehicle_width  / 1000
        vl = self.graph.vehicle_length / 1000
        line(f'AGV    : {vl:.2f}m × {vw:.2f}m', color=COL_DIM, font=self.font_s)
        y += 6

        # ── Keys ──────────────────────────────────────────────────────────────
        line('── Keys ──', font=self.font_b)
        for hint in ['SPACE  — start / pause',
                     'R      — reset time',
                     '+/-    — sim speed']:
            line(hint, color=COL_DIM, font=self.font_s)

    # ── Main loop ──────────────────────────────────────────────────────────────

    def run(self):
        while True:
            dt_real = self.clock.tick(FPS) / 1000.0
            self.handle_events()
            self.update(dt_real)
            self.render()


# ── Drawing helpers ────────────────────────────────────────────────────────────

def _draw_rotated_rect(surf, color, cx, cy, length_px, width_px, angle_deg,
                       border=None, border_w=2):
    """Filled (+ optional border) rectangle centred at (cx,cy), rotated angle_deg."""
    rad  = math.radians(angle_deg)
    fwd  = (math.cos(rad), math.sin(rad))
    rgt  = (-math.sin(rad), math.cos(rad))
    hl, hw = length_px / 2.0, width_px / 2.0
    corners = []
    for fl, rw in [(-hl, -hw), (-hl, hw), (hl, hw), (hl, -hw)]:
        corners.append((cx + fl * fwd[0] + rw * rgt[0],
                        cy + fl * fwd[1] + rw * rgt[1]))
    pygame.draw.polygon(surf, color, corners)
    if border:
        pygame.draw.polygon(surf, border, corners, border_w)


def _draw_arrow(surf, color, cx, cy, angle_rad, size=7):
    """Small filled arrowhead at (cx, cy) pointing in angle_rad direction."""
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    fwd   = (cos_a,  sin_a)
    rgt   = (-sin_a, cos_a)
    tip   = (cx + fwd[0] * size,          cy + fwd[1] * size)
    bl    = (cx - fwd[0] * size * 0.5 + rgt[0] * size * 0.55,
             cy - fwd[1] * size * 0.5 + rgt[1] * size * 0.55)
    br    = (cx - fwd[0] * size * 0.5 - rgt[0] * size * 0.55,
             cy - fwd[1] * size * 0.5 - rgt[1] * size * 0.55)
    pygame.draw.polygon(surf, color, [tip, bl, br])


def _speed_tint(base_color, vf: float) -> tuple:
    """Tint the agent body toward orange-red at high speed."""
    r, g, b = base_color
    return (min(255, int(r + (255 - r) * vf * 0.45)),
            min(255, int(g * (1 - vf * 0.35))),
            max(0,   int(b * (1 - vf * 0.50))))


# ── Demo ───────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import os, sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

    from map_loader import MapGraph
    from env_cont   import KinematicAgent

    MAP_FILE = os.path.join(os.path.dirname(__file__),
                            '..', 'Maps', 'grid_5x5_cont.json')
    graph = MapGraph(MAP_FILE)
    print(graph)

    # ── Agent 0: straight horizontal run → no deceleration at intermediate nodes
    #   r0c0 → r0c1 → r0c2 → r0c3 → r0c4
    a0 = KinematicAgent(0, 'r0c0', graph,
                        start_theta=graph.get_edge('r0c0', 'r0c1').angle)
    a0.set_node_path(['r0c0', 'r0c1', 'r0c2', 'r0c3', 'r0c4'], t_start=0.0)

    # ── Agent 1: straight vertical run → no deceleration at intermediate nodes
    #   r4c2 → r3c2 → r2c2 → r1c2 → r0c2
    a1 = KinematicAgent(1, 'r4c2', graph,
                        start_theta=graph.get_edge('r4c2', 'r3c2').angle)
    a1.set_node_path(['r4c2', 'r3c2', 'r2c2', 'r1c2', 'r0c2'], t_start=0.0)

    # ── Agent 2: L-shaped path → decelerates at the corner, then re-accelerates
    #   r4c0 → r3c0 → r2c0 → r2c1 → r2c2 → r2c3 → r2c4
    a2 = KinematicAgent(2, 'r4c0', graph,
                        start_theta=graph.get_edge('r4c0', 'r3c0').angle)
    a2.set_node_path(['r4c0', 'r3c0', 'r2c0', 'r2c1', 'r2c2', 'r2c3', 'r2c4'],
                     t_start=0.5)

    for a in [a0, a1, a2]:
        print(f'A{a.id}: path ends at t={a.path_end_time():.2f}s')

    ContSimulator(graph, [a0, a1, a2]).run()
