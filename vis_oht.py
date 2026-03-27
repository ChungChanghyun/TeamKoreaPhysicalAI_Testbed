"""
vis_oht.py — OHT DES visualizer (KaistTB OHT_A sub-network)

Supports two engines (toggle with Z key):
  Front-detection : env_oht_des — Automod-style kinematic catch-up
  Zone-based      : env_oht_zone — block signaling, claim/release only

Controls
────────
  SPACE     : Start / Pause
  R         : Reset
  S         : Shuffle paths
  Z         : Toggle engine (Front-detection ↔ Zone-based)
  E         : Toggle early release (zone mode)
  N / P     : Add / Remove OHT vehicle
  +/-       : Sim speed up / down
  Mouse drag: Pan
  Wheel     : Zoom
  Q / ESC   : Quit

Agent colour coding
───────────────────
  MOVING    : agent colour (bright)
  FOLLOWING : yellow (front-detection only)
  BLOCKED   : red
  IDLE      : dimmed
  DONE      : dark
"""
from __future__ import annotations
import sys, os, math, random
import pygame

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from env_oht_des import (OHTMap, OHTAgent, OHTEnvironmentDES,
                          IDLE, MOVING, FOLLOWING, BLOCKED, DONE)
from env_oht_zone import (ZoneMap, ZoneAgent, ZoneEnvironmentDES,
                           IDLE as Z_IDLE, MOVING as Z_MOVING,
                           BLOCKED as Z_BLOCKED, DONE as Z_DONE)

JSON_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '..', 'Maps', 'KaistTB.map_latest.json')

# ── Colors ────────────────────────────────────────────────────────────────────
BG          = (18,  20,  28)
COL_SEG     = (50,  70,  90)
COL_SEG_ARR = (80, 110, 140)
COL_NODE    = (90, 110, 130)
COL_PORT    = (60, 220, 120)
COL_TEXT    = (200, 210, 220)
COL_DIM     = (80,  90, 100)
COL_WHITE   = (255, 255, 255)

COL_FOLLOWING = (255, 210,  50)
COL_BLOCKED   = (220,  60,  60)
COL_HEADLIGHT = (255, 255, 180)
COL_ZCU_FREE  = (60,  180,  60)
COL_ZCU_HELD  = (220, 100,  40)
COL_ZONE_NODE = (60,  120, 180)   # intermediate zone-node colour
COL_ZONE_HELD = (180, 60,  60)    # zone-node held by vehicle

AGENT_COLORS = [
    (100, 180, 255), (100, 255, 160), (255, 160, 80),
    (220, 100, 220), (80,  220, 220), (255, 120, 120),
    (160, 255, 100), (200, 160, 255), (255, 200, 80),
    (80,  160, 255),
]

SIM_SPEEDS       = [0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0]
SIM_SPEED_LABELS = ['0.1×', '0.25×', '0.5×', '1×', '2×', '4×', '8×', '16×']
DEFAULT_SPD_IDX  = 3

WIN_W, WIN_H = 1300, 820
SIDE_W       = 240
MAP_W        = WIN_W - SIDE_W
MAP_PAD      = 40
FPS          = 60
MAX_AGENTS   = 10


# ── Camera ────────────────────────────────────────────────────────────────────

class Camera:
    def __init__(self, bbox, w, h, pad=MAP_PAD):
        x0, y0, x1, y1 = bbox
        mw, mh = x1 - x0, y1 - y0
        sx = (w - 2*pad) / mw
        sy = (h - 2*pad) / mh
        self.scale  = min(sx, sy)
        self.offset = [
            pad + (w  - 2*pad - mw*self.scale)/2 - x0*self.scale,
            pad + (h  - 2*pad - mh*self.scale)/2 + (y0+mh)*self.scale,
        ]
        self._drag_start  = None
        self._offset_start = None

    def to_screen(self, mx, my):
        return (int(mx * self.scale + self.offset[0]),
                int(-my * self.scale + self.offset[1]))

    def on_down(self, pos):
        self._drag_start   = pos
        self._offset_start = list(self.offset)

    def on_move(self, pos):
        if self._drag_start is None:
            return
        self.offset[0] = self._offset_start[0] + pos[0] - self._drag_start[0]
        self.offset[1] = self._offset_start[1] + pos[1] - self._drag_start[1]

    def on_up(self):
        self._drag_start = None

    def on_scroll(self, pos, up: bool):
        f = 1.15 if up else 1/1.15
        self.offset[0] = pos[0] + (self.offset[0] - pos[0]) * f
        self.offset[1] = pos[1] + (self.offset[1] - pos[1]) * f
        self.scale *= f


# ── Button ────────────────────────────────────────────────────────────────────

class Button:
    def __init__(self, rect, label, toggle=False, base=(60, 70, 85)):
        self.rect   = pygame.Rect(rect)
        self.label  = label
        self.toggle = toggle
        self.base   = base
        self.active = False
        self.hover  = False

    def update_hover(self, pos):
        self.hover = self.rect.collidepoint(pos)

    def clicked(self, pos) -> bool:
        return self.rect.collidepoint(pos)

    def draw(self, surf, font):
        col = (min(self.base[0]+40, 255),
               min(self.base[1]+40, 255),
               min(self.base[2]+40, 255)) if self.hover else self.base
        if self.toggle and self.active:
            col = (min(col[0]+30, 255), min(col[1]+30, 255), col[2])
        pygame.draw.rect(surf, col,         self.rect, border_radius=4)
        pygame.draw.rect(surf, COL_DIM,     self.rect, 1, border_radius=4)
        lbl = font.render(self.label, True, COL_TEXT)
        surf.blit(lbl, lbl.get_rect(center=self.rect.center))


# ── Arrow drawing ─────────────────────────────────────────────────────────────

def draw_arrow(surf, color, p1, p2, width=1, head=5):
    dx, dy = p2[0]-p1[0], p2[1]-p1[1]
    L = math.hypot(dx, dy)
    if L < 2:
        return
    ratio = max(0, (L - head*1.5)) / L
    tip   = (p1[0]+dx*ratio, p1[1]+dy*ratio)
    pygame.draw.line(surf, color, p1, (int(tip[0]), int(tip[1])), width)
    ux, uy = dx/L, dy/L
    px, py = -uy, ux
    h = head
    pts = [
        (int(p2[0]-ux*h*1.5+px*h*0.6), int(p2[1]-uy*h*1.5+py*h*0.6)),
        (int(p2[0]),                    int(p2[1])),
        (int(p2[0]-ux*h*1.5-px*h*0.6), int(p2[1]-uy*h*1.5-py*h*0.6)),
    ]
    pygame.draw.polygon(surf, color, pts)


def draw_rotated_rect(surf, color, cx, cy, length, width, angle_deg,
                      border=None, border_w=1):
    rad = math.radians(angle_deg)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    hw, hl = width/2, length/2
    corners = [
        ( hl*cos_a - hw*sin_a,  hl*sin_a + hw*cos_a),
        (-hl*cos_a - hw*sin_a, -hl*sin_a + hw*cos_a),
        (-hl*cos_a + hw*sin_a, -hl*sin_a - hw*cos_a),
        ( hl*cos_a + hw*sin_a,  hl*sin_a - hw*cos_a),
    ]
    pts = [(int(cx+dx), int(cy+dy)) for dx, dy in corners]
    pygame.draw.polygon(surf, color, pts)
    if border:
        pygame.draw.polygon(surf, border, pts, border_w)


# ── Simulator ─────────────────────────────────────────────────────────────────

class OHTSimulator:
    def __init__(self, oht_map: OHTMap, zone_map: ZoneMap,
                 n_agents: int = 4):
        self.map      = oht_map
        self.zone_map = zone_map
        self._next_id = 0
        self._n_target = max(1, min(n_agents, MAX_AGENTS))

        # Engine mode: 'front' (front-detection) or 'zone' (block signaling)
        self._engine = 'front'
        self._early_release = True

        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
        pygame.display.set_caption('OHT DES Simulator — KaistTB OHT_A')
        self.clock  = pygame.time.Clock()
        self.font_s = pygame.font.SysFont('Consolas', 12)
        self.font_m = pygame.font.SysFont('Consolas', 14)
        self.font_b = pygame.font.SysFont('Consolas', 15, bold=True)

        self.sim_time = 0.0
        self.running  = False
        self.spd_idx  = DEFAULT_SPD_IDX

        # Metrics for comparison
        self._front_events = 0
        self._zone_events  = 0

        self.cam = Camera(oht_map.bbox, MAP_W, WIN_H)
        self._build_buttons()
        self._init_agents()

    # ── Agent setup ───────────────────────────────────────────────────────────

    def _fresh_color(self, aid: int) -> tuple:
        return AGENT_COLORS[aid % len(AGENT_COLORS)]

    def _make_agent(self, agent_id: int, excluded: set) -> OHTAgent | None:
        nodes = list(self.map.nodes.keys())
        random.shuffle(nodes)
        for start in nodes:
            if start in excluded:
                continue
            path = self.map.bfs_path(start)
            if len(path) > 1:
                return OHTAgent(agent_id, self._fresh_color(agent_id),
                                path, self.map.vehicle_length * (1000/1108))
        return None

    def _init_agents(self):
        if self._engine == 'front':
            self._init_front_agents()
        else:
            self._init_zone_agents()

    def _init_front_agents(self):
        self.agents: list = []
        self.env = OHTEnvironmentDES(self.map, cross_segment=True)
        self.zone_env = None
        excluded = set()
        for _ in range(self._n_target):
            a = self._make_agent(self._next_id, excluded)
            if a:
                excluded |= self.map.nearby_nodes(a.node_path[0], self.map.h_min)
                self._next_id += 1
                self.agents.append(a)
                self.env.add_agent(a, t_start=0.0)

    def _init_zone_agents(self):
        self.agents: list = []
        self.env = None
        self.zone_env = ZoneEnvironmentDES(self.zone_map,
                                           early_release=self._early_release)
        orig_nodes = self.zone_map.startable_node_ids()
        excluded = set()
        for _ in range(self._n_target):
            a = self._make_zone_agent(self._next_id, excluded, orig_nodes)
            if a:
                excluded |= self.zone_map.nearby_nodes(
                    a.zone_path[0], self.zone_map.h_min)
                self._next_id += 1
                self.agents.append(a)
                self.zone_env.add_agent(a, t_start=0.0)

    def _make_zone_agent(self, agent_id: int, excluded: set,
                         start_nodes: list = None) -> ZoneAgent | None:
        if start_nodes is None:
            start_nodes = self.zone_map.startable_node_ids()
        random.shuffle(start_nodes)
        for start in start_nodes:
            if start in excluded:
                continue
            # BFS directly on zone graph
            zone_path = self.zone_map.bfs_path(start)
            if len(zone_path) > 1:
                return ZoneAgent(agent_id, self._fresh_color(agent_id),
                                 zone_path, self.map.vehicle_length * (1000/1108))
        return None

    # ── Buttons ───────────────────────────────────────────────────────────────

    def _build_buttons(self):
        sx, sw = MAP_W + 10, SIDE_W - 20
        y, H, G = 12, 28, 5

        self.btn_start = Button((sx, y, sw, H), '▶ Start / Pause'); y += H+G
        self.btn_reset = Button((sx, y, sw, H), '↺  Reset',
                                base=(80, 40, 40)); y += H+G+4

        bw = (sw+2)//4
        self.btns_spd = []
        for row in range(2):
            for col in range(4):
                idx = row*4 + col
                if idx >= len(SIM_SPEED_LABELS):
                    break
                b = Button((sx+col*bw, y, bw-2, H-4), SIM_SPEED_LABELS[idx], toggle=True)
                b.active = (idx == self.spd_idx)
                self.btns_spd.append(b)
            y += H-4+2
        y += G+4

        half = (sw-G)//2
        self.btn_add = Button((sx+half+G, y, half, H-4), '+');
        self.btn_del = Button((sx,        y, half, H-4), '-', base=(80,40,40))
        self._count_rect = pygame.Rect(sx, y, sw, H-4)
        y += H-4+G

        self.btn_shuffle = Button((sx, y, sw, H-4), 'S: Shuffle'); y += H-4+G

        self._info_y = y + 4
        self.all_btns = [self.btn_start, self.btn_reset,
                         *self.btns_spd,
                         self.btn_add, self.btn_del, self.btn_shuffle]

    # ── Update ────────────────────────────────────────────────────────────────

    def update(self, dt_real: float):
        if not self.running:
            return
        self.sim_time += dt_real * SIM_SPEEDS[self.spd_idx]

        if self._engine == 'front' and self.env:
            self.env.step(self.sim_time)
            self._front_events = getattr(self.env, 'event_count', 0)
            for a in list(self.agents):
                if a.state == DONE:
                    path = self.map.bfs_path(a.cur_node)
                    if len(path) > 1:
                        self.env.reassign(a, path, self.sim_time)
        elif self._engine == 'zone' and self.zone_env:
            self.zone_env.step(self.sim_time)
            self._zone_events = self.zone_env.event_count
            for a in list(self.agents):
                if a.state == Z_DONE:
                    zone_path = self.zone_map.bfs_path(a.cur_node)
                    if len(zone_path) > 1:
                        self.zone_env.reassign(a, zone_path, self.sim_time)

    # ── Input ─────────────────────────────────────────────────────────────────

    def handle_events(self):
        ww, wh = self.screen.get_size()
        mpos = pygame.mouse.get_pos()
        for b in self.all_btns:
            b.update_hover(mpos)

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            elif ev.type == pygame.KEYDOWN:
                k = ev.key
                if k in (pygame.K_q, pygame.K_ESCAPE):
                    pygame.quit(); sys.exit()
                elif k == pygame.K_SPACE:
                    self.running = not self.running
                elif k == pygame.K_r:
                    self._reset()
                elif k == pygame.K_s:
                    self._shuffle()
                elif k == pygame.K_z:
                    self._toggle_engine()
                elif k == pygame.K_e:
                    self._early_release = not self._early_release
                    if self._engine == 'zone':
                        self._reset()
                elif k == pygame.K_n:
                    self._add_agent()
                elif k == pygame.K_p:
                    self._del_agent()
                elif k in (pygame.K_EQUALS, pygame.K_PLUS):
                    self._set_spd(min(self.spd_idx+1, len(SIM_SPEEDS)-1))
                elif k == pygame.K_MINUS:
                    self._set_spd(max(self.spd_idx-1, 0))
                elif k == pygame.K_k:
                    self.cam = Camera(self.map.bbox, ww-SIDE_W, wh)
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.button == 1:
                    pos = ev.pos
                    if self.btn_start.clicked(pos):
                        self.running = not self.running
                    elif self.btn_reset.clicked(pos):
                        self._reset()
                    elif self.btn_shuffle.clicked(pos):
                        self._shuffle()
                    elif self.btn_add.clicked(pos):
                        self._add_agent()
                    elif self.btn_del.clicked(pos):
                        self._del_agent()
                    else:
                        for i, b in enumerate(self.btns_spd):
                            if b.clicked(pos):
                                self._set_spd(i)
                    if ev.pos[0] < MAP_W:
                        self.cam.on_down(ev.pos)
                elif ev.button == 4:
                    self.cam.on_scroll(ev.pos, up=True)
                elif ev.button == 5:
                    self.cam.on_scroll(ev.pos, up=False)
            elif ev.type == pygame.MOUSEBUTTONUP:
                if ev.button == 1:
                    self.cam.on_up()
            elif ev.type == pygame.MOUSEMOTION:
                self.cam.on_move(ev.pos)
            elif ev.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode(ev.size, pygame.RESIZABLE)

    def _reset(self):
        self.sim_time = 0.0
        self.running  = False
        self._next_id = 0
        self.agents   = []
        # clear segment queues from previous run
        for seg in self.map.segments.values():
            seg.queue.clear()
        self._init_agents()

    def _shuffle(self):
        if self._engine == 'front' and self.env:
            nodes = list(self.map.nodes.keys())
            random.shuffle(nodes)
            excluded = set()
            for a in self.agents:
                for start in nodes:
                    if start not in excluded:
                        path = self.map.bfs_path(start)
                        if len(path) > 1:
                            excluded |= self.map.nearby_nodes(start, self.map.h_min)
                            self.env.reassign(a, path, self.sim_time)
                            break
        elif self._engine == 'zone' and self.zone_env:
            start_nodes = self.zone_map.startable_node_ids()
            random.shuffle(start_nodes)
            excluded = set()
            for a in self.agents:
                for start in start_nodes:
                    if start not in excluded:
                        zone_path = self.zone_map.bfs_path(start)
                        if len(zone_path) > 1:
                            excluded |= self.zone_map.nearby_nodes(
                                start, self.zone_map.h_min)
                            self.zone_env.reassign(a, zone_path, self.sim_time)
                            break

    def _toggle_engine(self):
        """Switch between front-detection and zone-based engines."""
        if self._engine == 'front':
            self._engine = 'zone'
        else:
            self._engine = 'front'
        self._reset()

    def _add_agent(self):
        if len(self.agents) >= MAX_AGENTS:
            return
        if self._engine == 'front' and self.env:
            excluded = set()
            for ea in self.agents:
                excluded |= self.map.nearby_nodes(ea.cur_node, self.map.h_min)
            a = self._make_agent(self._next_id, excluded)
            if a:
                self._next_id += 1
                self.agents.append(a)
                self.env.add_agent(a, t_start=self.sim_time)
        elif self._engine == 'zone' and self.zone_env:
            excluded = set()
            for ea in self.agents:
                excluded |= self.zone_map.nearby_nodes(
                    ea.cur_node, self.zone_map.h_min)
            a = self._make_zone_agent(self._next_id, excluded)
            if a:
                self._next_id += 1
                self.agents.append(a)
                self.zone_env.add_agent(a, t_start=self.sim_time)
        self._n_target = len(self.agents)

    def _del_agent(self):
        if not self.agents:
            return
        a = self.agents.pop()
        if self._engine == 'front' and self.env:
            self.env.remove_agent(a.id)
        elif self._engine == 'zone' and self.zone_env:
            self.zone_env.remove_agent(a.id)
        self._n_target = len(self.agents)

    def _set_spd(self, idx: int):
        self.spd_idx = idx
        for i, b in enumerate(self.btns_spd):
            b.active = (i == idx)

    # ── Render ────────────────────────────────────────────────────────────────

    def render(self):
        ww, wh = self.screen.get_size()
        self.screen.fill(BG)
        pygame.draw.rect(self.screen, (24, 26, 36),
                         pygame.Rect(MAP_W, 0, SIDE_W, wh))

        self._draw_network(ww, wh)
        self._draw_agents(ww, wh)
        self._draw_sidebar(wh)
        pygame.display.flip()

    def _draw_network(self, ww, wh):
        seg_w  = max(1, int(self.cam.scale * 0.0006))
        h_size = max(4, int(self.cam.scale * 0.002))

        for seg in self.map.segments.values():
            pts = seg.path_points
            if not pts:
                continue
            # cull: skip if bounding box entirely off-screen
            spts = [self.cam.to_screen(x, y) for x, y in pts]
            xs = [p[0] for p in spts]; ys = [p[1] for p in spts]
            if min(xs) > ww or max(xs) < 0 or min(ys) > wh or max(ys) < 0:
                continue
            # draw polyline
            if len(spts) >= 2:
                pygame.draw.lines(self.screen, COL_SEG, False, spts, seg_w)
            # arrowhead at ~60% along the path
            mid_idx = int(len(spts) * 0.6)
            mid_idx = max(1, min(mid_idx, len(spts) - 1))
            draw_arrow(self.screen, COL_SEG_ARR,
                       spts[mid_idx - 1], spts[mid_idx],
                       width=seg_w, head=h_size)

        node_r = max(3, int(self.cam.scale * 0.004))

        if self._engine == 'zone' and self.zone_env:
            # Draw zone-node markers (intermediate + original)
            zr = max(2, node_r - 1)
            for nid, zn in self.zone_map.nodes.items():
                sx, sy = self.cam.to_screen(zn.x, zn.y)
                if sx < -10 or sx > ww+10 or sy < -10 or sy > wh+10:
                    continue
                holder = self.zone_env._holders.get(nid)
                if holder is not None:
                    col = holder.color
                    r = zr + 2 if zn.is_original else zr + 1
                    pygame.draw.circle(self.screen, col, (sx, sy), r)
                    pygame.draw.circle(self.screen, COL_WHITE, (sx, sy), r, 1)
                elif zn.is_original:
                    col = COL_PORT if nid in self.zone_map.port_nodes else COL_NODE
                    pygame.draw.circle(self.screen, col,       (sx, sy), node_r)
                    pygame.draw.circle(self.screen, COL_WHITE, (sx, sy), node_r, 1)
                else:
                    pygame.draw.circle(self.screen, COL_ZONE_NODE, (sx, sy), zr)
        else:
            # Front-detection mode: original nodes + ZCU markers
            for nid, node in self.map.nodes.items():
                sx, sy = self.cam.to_screen(node.x, node.y)
                if sx < -10 or sx > ww+10 or sy < -10 or sy > wh+10:
                    continue
                if nid in self.map.zcu_node_ids and self.env:
                    zone_held = any(
                        z for z in self.map.zcu_zones
                        if nid in {t for _, t in z.entry_segs | z.exit_segs}
                        and self.env._zcu_holders.get(z.id) is not None
                    )
                    zcu_col = COL_ZCU_HELD if zone_held else COL_ZCU_FREE
                    r = node_r + 2
                    diamond = [(sx, sy-r), (sx+r, sy), (sx, sy+r), (sx-r, sy)]
                    pygame.draw.polygon(self.screen, zcu_col,    diamond)
                    pygame.draw.polygon(self.screen, COL_WHITE,  diamond, 1)
                    lbl = self.font_s.render('Z', True, COL_WHITE)
                    self.screen.blit(lbl, lbl.get_rect(center=(sx, sy)))
                else:
                    col = COL_PORT if nid in self.map.port_nodes else COL_NODE
                    pygame.draw.circle(self.screen, col,       (sx, sy), node_r)
                    pygame.draw.circle(self.screen, COL_WHITE, (sx, sy), node_r, 1)

    def _draw_agents(self, ww, wh):
        v_len = max(self.map.vehicle_length * self.cam.scale, 10.0)
        v_wid = max(self.map.vehicle_width  * self.cam.scale,  5.0)

        for a in self.agents:
            sx, sy = self.cam.to_screen(a.x, a.y)
            if sx < -50 or sx > ww+50 or sy < -50 or sy > wh+50:
                continue

            scr_ang = math.degrees(-a.theta)   # screen y is flipped

            if a.state == DONE:
                fill   = tuple(c//4 for c in a.color)
                border = COL_DIM
            elif a.state == BLOCKED:
                fill   = COL_BLOCKED
                border = COL_WHITE
            elif a.state == FOLLOWING:
                fill   = COL_FOLLOWING
                border = COL_WHITE
            elif a.state == MOVING:
                vf     = min(a.v / self.map.vehicle_length, 1.0)
                bright = int(40 * vf)
                fill   = tuple(min(c+bright, 255) for c in a.color)
                border = COL_WHITE
            else:  # IDLE
                fill   = tuple(max(c-50, 0) for c in a.color)
                border = COL_DIM

            draw_rotated_rect(self.screen, fill, sx, sy,
                              v_len, v_wid, scr_ang,
                              border=border, border_w=2)

            # headlight
            rad = math.radians(scr_ang)
            hx  = sx + math.cos(rad) * v_len / 2
            hy  = sy + math.sin(rad) * v_len / 2
            pygame.draw.circle(self.screen, COL_HEADLIGHT,
                               (int(hx), int(hy)),
                               max(2, int(v_wid * 0.2)))

            # state ring
            if a.state == BLOCKED:
                pygame.draw.circle(self.screen, COL_BLOCKED,
                                   (sx, sy), int(v_len/2)+4, 2)
            elif a.state == FOLLOWING:
                pygame.draw.circle(self.screen, COL_FOLLOWING,
                                   (sx, sy), int(v_len/2)+4, 2)

            # agent ID
            lbl = self.font_s.render(str(a.id), True, COL_WHITE)
            self.screen.blit(lbl, lbl.get_rect(center=(sx, sy)))

    def _draw_sidebar(self, wh):
        for b in self.all_btns:
            b.draw(self.screen, self.font_s)

        y  = self._info_y
        sx = MAP_W + 10

        def line(txt, col=COL_TEXT, f=None):
            nonlocal y
            fnt = f or self.font_m
            self.screen.blit(fnt.render(txt, True, col), (sx, y))
            y += fnt.get_linesize() + 2

        # Engine label
        eng_name = 'Front-Detection' if self._engine == 'front' else 'Zone-Based'
        eng_col  = (220, 180, 80) if self._engine == 'front' else (80, 200, 220)
        line(f'Engine: {eng_name}', col=eng_col, f=self.font_b)
        if self._engine == 'zone':
            er = 'ON' if self._early_release else 'OFF'
            line(f'  Early release: {er} [E]', col=COL_DIM, f=self.font_s)
        y += 2

        line('── Simulation ──', f=self.font_b)
        line(f'Time  : {self.sim_time:8.2f} s')
        line(f'Speed : {SIM_SPEED_LABELS[self.spd_idx]}')
        state = '▶ Running' if self.running else '|| Paused'
        line(state, col=(100,220,100) if self.running else (160,160,160))

        # Event count
        if self._engine == 'front':
            ev_cnt = self._front_events
        else:
            ev_cnt = self._zone_events
        line(f'Events: {ev_cnt}', col=COL_DIM, f=self.font_s)
        if self._engine == 'zone' and self.zone_env:
            util = self.zone_env.node_utilization()
            line(f'Node util: {util:.1%}', col=COL_DIM, f=self.font_s)
        y += 4

        n_mov = sum(1 for a in self.agents if a.state == MOVING or a.state == Z_MOVING)
        n_fol = sum(1 for a in self.agents if a.state == FOLLOWING)
        n_blk = sum(1 for a in self.agents if a.state == BLOCKED or a.state == Z_BLOCKED)
        n_idk = sum(1 for a in self.agents if a.state == IDLE or a.state == Z_IDLE)
        line('── Agents ──', f=self.font_b)
        line(f'Moving   : {n_mov}')
        if self._engine == 'front':
            line(f'Following: {n_fol}',  col=COL_FOLLOWING if n_fol else COL_DIM, f=self.font_s)
        line(f'Blocked  : {n_blk}',  col=COL_BLOCKED   if n_blk else COL_DIM, f=self.font_s)
        line(f'Idle     : {n_idk}',  col=COL_DIM, f=self.font_s)
        y += 4

        STATE_COL = {
            MOVING: COL_TEXT, Z_MOVING: COL_TEXT,
            FOLLOWING: COL_FOLLOWING,
            BLOCKED: COL_BLOCKED, Z_BLOCKED: COL_BLOCKED,
            IDLE: COL_DIM, Z_IDLE: COL_DIM,
            DONE: COL_DIM, Z_DONE: COL_DIM,
        }
        for a in self.agents:
            scol = STATE_COL.get(a.state, COL_TEXT)
            pygame.draw.rect(self.screen, a.color,
                             pygame.Rect(sx, y+2, 10, 10), border_radius=2)
            self.screen.blit(
                self.font_s.render(
                    f' A{a.id:02d} {a.state:<9s} v={a.v/1000:.2f}m/s',
                    True, scol),
                (sx+12, y))
            y += 15

        y += 6
        if self._engine == 'front' and self.env and self.map.zcu_zones:
            line('── ZCU ──', f=self.font_b)
            for z in self.map.zcu_zones:
                holder = self.env._zcu_holders.get(z.id)
                n_wait = len(self.env._zcu_waitlists.get(z.id, []))
                name = z.id.replace('ZCU_', '')
                if holder is not None:
                    txt = f'{name}: A{holder.id:02d}'
                    if n_wait:
                        txt += f' ({n_wait}w)'
                    line(txt, col=COL_ZCU_HELD, f=self.font_s)
                else:
                    line(f'{name}: free', col=COL_ZCU_FREE, f=self.font_s)
            y += 4

        line('── Map ──', f=self.font_b)
        if self._engine == 'zone':
            line(f'Orig nodes: {self.zone_map.n_original_nodes}',
                 col=COL_DIM, f=self.font_s)
            line(f'Zone nodes: {len(self.zone_map.nodes)}',
                 col=COL_ZONE_NODE, f=self.font_s)
            line(f'Zone segs : {len(self.zone_map.segments)}',
                 col=COL_ZONE_NODE, f=self.font_s)
            line(f'Zone len  : ~{self.zone_map.zone_target:.0f}mm',
                 col=COL_DIM, f=self.font_s)
        else:
            line(f'Nodes  : {len(self.map.nodes)}',    col=COL_DIM, f=self.font_s)
            line(f'Segs   : {len(self.map.segments)}', col=COL_DIM, f=self.font_s)
        vl = self.map.vehicle_length / 1000
        vw = self.map.vehicle_width  / 1000
        line(f'OHT    : {vl:.2f}m × {vw:.2f}m',  col=COL_DIM, f=self.font_s)
        line(f'h_min  : {self.map.h_min:.0f}mm',  col=COL_DIM, f=self.font_s)
        y += 6
        line('── Keys ──', f=self.font_b)
        for hint in ['SPACE - start/pause', 'R - reset', 'S - shuffle',
                     'Z - toggle engine',
                     'E - early release (zone)',
                     '+/- - sim speed', 'N/P - add/remove', 'drag - pan',
                     'wheel - zoom', 'Q - quit']:
            line(hint, col=COL_DIM, f=self.font_s)

    # ── Run ───────────────────────────────────────────────────────────────────

    def run(self):
        while True:
            dt = self.clock.tick(FPS) / 1000.0
            self.handle_events()
            self.update(dt)
            self.render()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--agents', type=int, default=8)
    ap.add_argument('--engine', choices=['front', 'zone'], default='front',
                    help='Starting engine (front=front-detection, zone=zone-based)')
    args = ap.parse_args()

    print('Loading OHT map (front-detection)...')
    oht_map = OHTMap(JSON_FILE, area='OHT_A')
    print(f'  {len(oht_map.nodes)} nodes, {len(oht_map.segments)} segments')
    print(f'  OHT: {oht_map.vehicle_length}mm × {oht_map.vehicle_width}mm  h_min={oht_map.h_min}mm')

    print('Building zone map (block signaling)...')
    zone_map = ZoneMap(JSON_FILE, area='OHT_A')
    print(f'  {zone_map.n_original_nodes} orig nodes → {len(zone_map.nodes)} zone nodes')
    print(f'  {zone_map.n_original_segs} orig segs  → {len(zone_map.segments)} zone segs')
    print(f'  Zone target: {zone_map.zone_target:.0f}mm')

    sim = OHTSimulator(oht_map, zone_map, n_agents=args.agents)
    if args.engine == 'zone':
        sim._engine = 'zone'
        sim._reset()
    sim.run()
