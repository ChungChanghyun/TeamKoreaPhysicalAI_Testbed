"""
vis_songdo.py — AGV 시뮬레이터 for 송도공장 layout.

Uses the same TAPG+SIPP planning infrastructure as vis_combined's AGV portion,
but adapted for the 송도공장 JSON layout + pkl.

Usage:
    1. First generate pkl:  python gen_songdo_pkl.py
    2. Then run simulator:  python vis_songdo.py

Controls
────────
  SPACE     : Start / Pause
  R         : Reset
  S         : Shuffle paths
  N / P     : Add / Remove AGV
  +/-       : Sim speed up / down
  D         : Dump status
  Mouse drag: Pan
  Wheel     : Zoom
  Q / ESC   : Quit
"""
from __future__ import annotations
import sys, os, json, math, random, collections
import pygame

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from env_tapg import (TAPGAgent, TAPGEnvironment,
                      IDLE as AGV_IDLE, MOVING as AGV_MOVING,
                      WAITING as AGV_WAITING, ROTATING as AGV_ROTATING,
                      DONE as AGV_DONE)
from pkl_loader import PklMapGraph
from pkl_prioritized_planner import PklPrioritizedPlanner

# ── Paths ─────────────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
JSON_FILE = os.path.join(BASE_DIR, '송도공장_Final_v0_bi.json')
PKL_FILE  = os.path.join(BASE_DIR, '송도공장.pkl')
COLLISION_LOG = os.path.join(BASE_DIR, 'songdo_collision_log.txt')

# ── Colors ────────────────────────────────────────────────────────────────────
BG          = (18,  20,  28)
COL_SEG     = (40,  55,  70)
COL_SEG_ARR = (60,  80, 100)
COL_NODE    = (70,  85, 100)
COL_PORT    = (60, 220, 120)
COL_TEXT    = (200, 210, 220)
COL_DIM     = (80,  90, 100)
COL_WHITE   = (255, 255, 255)

AGV_COLORS = [
    (100, 220, 120), (255, 200,  60), (255, 140,  60),
    (80,  200, 200), (200, 255, 100), (180,  80, 200),
    (140, 100, 255), (200, 120, 255), (120,  80, 220),
    (255, 100, 150),
]

SIM_SPEEDS       = [0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0]
SIM_SPEED_LABELS = ['0.1×', '0.25×', '0.5×', '1×', '2×', '4×', '8×', '16×']
DEFAULT_SPD_IDX  = 3

WIN_W, WIN_H = 1400, 900
SIDE_W       = 260
MAP_W        = WIN_W - SIDE_W
MAP_PAD      = 40
FPS          = 60
MAX_AGENTS   = 10


# ── Load JSON background data ────────────────────────────────────────────────

def load_songdo_map(json_path: str):
    with open(json_path, 'r', encoding='utf-8') as f:
        d = json.load(f)
    nodes = {}
    for n in d['nodes']:
        if 'location' in n:
            loc = n['location']
            nodes[n['id']] = {'id': n['id'], 'x': loc[0], 'y': loc[1]}
        else:
            nodes[n['id']] = {'id': n['id'], 'x': n['x'], 'y': n['y']}

    # Support both 'edges' and 'segments' keys
    if 'edges' in d:
        edges = d['edges']
    else:
        edges = []
        seen = set()
        for s in d.get('segments', []):
            start = s['startNodeId']
            end = s['endNodeId']
            pair = tuple(sorted([start, end]))
            bidir = pair in seen
            if not bidir:
                seen.add(pair)
            edges.append({
                'start': start, 'end': end,
                'type': s.get('type', ''),
                'bidirectional': bidir,
            })

    # Port nodes
    if 'port_nodes' in d:
        port_ids = set(d['port_nodes'])
    else:
        port_ids = set(p['nodeId'] for p in d.get('ports', []))

    ports = {}
    for p in d.get('ports', []):
        pid = p.get('id', p.get('nodeId', ''))
        ports[pid] = p

    params = d.get('parameters', {})
    xs = [n['x'] for n in nodes.values()]
    ys = [n['y'] for n in nodes.values()]
    bbox = (min(xs), min(ys), max(xs), max(ys))
    return nodes, edges, port_ids, ports, params, bbox


# ── Camera ────────────────────────────────────────────────────────────────────

class Camera:
    def __init__(self, bbox, w, h, pad=MAP_PAD):
        x0, y0, x1, y1 = bbox
        mw, mh = x1 - x0, y1 - y0
        sx = (w - 2*pad) / mw if mw > 0 else 1
        sy = (h - 2*pad) / mh if mh > 0 else 1
        self.scale  = min(sx, sy)
        self.offset = [
            pad + (w - 2*pad - mw*self.scale)/2 - x0*self.scale,
            pad + (h - 2*pad - mh*self.scale)/2 + (y0+mh)*self.scale,
        ]
        self._drag_start  = None
        self._offset_start = None

    def to_screen(self, mx, my):
        return (int(mx * self.scale + self.offset[0]),
                int(-my * self.scale + self.offset[1]))

    def px(self, mm):
        return mm * self.scale

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
        col = tuple(min(c+40, 255) for c in self.base) if self.hover else self.base
        if self.toggle and self.active:
            col = (min(col[0]+30, 255), min(col[1]+30, 255), col[2])
        pygame.draw.rect(surf, col, self.rect, border_radius=4)
        pygame.draw.rect(surf, COL_DIM, self.rect, 1, border_radius=4)
        lbl = font.render(self.label, True, COL_TEXT)
        surf.blit(lbl, lbl.get_rect(center=self.rect.center))


# ── Drawing helpers ──────────────────────────────────────────────────────────

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


# ── Simulator ────────────────────────────────────────────────────────────────

class SongdoSimulator:
    def __init__(self, n_agv: int = 3):
        # Load background map
        self.bg_nodes, self.bg_edges, self.port_ids, self.bg_ports, \
            self.params, self.bg_bbox = load_songdo_map(JSON_FILE)

        # Load pkl graph
        if not os.path.exists(PKL_FILE):
            print(f'ERROR: {PKL_FILE} not found. Run gen_songdo_pkl.py first.')
            sys.exit(1)
        self.amr_graph = PklMapGraph(PKL_FILE)
        print(f'Loaded graph: {self.amr_graph}')

        self._n_agv = min(n_agv, MAX_AGENTS)

        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
        pygame.display.set_caption('송도공장 AGV Simulator')
        self.clock  = pygame.time.Clock()
        self.font_s = pygame.font.SysFont('Consolas', 12)
        self.font_m = pygame.font.SysFont('Consolas', 14)
        self.font_b = pygame.font.SysFont('Consolas', 15, bold=True)

        self.sim_time = 0.0
        self.running  = False
        self.spd_idx  = DEFAULT_SPD_IDX

        self.cam = Camera(self.bg_bbox, MAP_W, WIN_H)
        self._build_buttons()

        # AGV agents
        self._agv_next_id = 0
        self.agv_agents: list[TAPGAgent] = []
        self.agv_env = TAPGEnvironment(self.amr_graph, accel=50.0, decel=50.0)
        self.agv_planner = PklPrioritizedPlanner(self.amr_graph)
        self._agv_goals: dict = {}
        self._agv_pending_replan: set = set()
        self._plan_status = ''
        self._collision_log_f = open(COLLISION_LOG, 'w', encoding='utf-8')
        self._collision_pairs_logged: set = set()
        self._collision_count = 0
        self._replan_history = {}

        self._init_agv_agents()

    def _init_agv_agents(self):
        port_nodes = list(self.agv_planner._port_nodes)
        random.shuffle(port_nodes)
        self._agv_start_positions = {}
        n = min(self._n_agv, len(port_nodes))
        for i in range(n):
            aid = self._agv_next_id
            self._agv_start_positions[aid] = port_nodes[i]
            self._agv_next_id += 1
        self._plan_agv_paths(0.0)

    def _plan_agv_paths(self, t_start: float):
        positions = {}
        if self.agv_agents:
            for a in self.agv_agents:
                nid = a.raw_path[-1][0].split(',')[1] if a.state == AGV_DONE else \
                      a.raw_path[a.path_idx][0].split(',')[1]
                positions[a.id] = nid
        else:
            positions = dict(self._agv_start_positions)

        goals = self.agv_planner.assign_random_goals(positions)
        self._agv_goals = goals

        result = self.agv_planner.plan(positions, goals, start_times={
            aid: t_start for aid in positions
        })

        if result is None:
            self._plan_status = 'Plan FAILED'
            return

        self.agv_agents = []
        for i, aid in enumerate(sorted(result.paths)):
            raw_path = result.paths[aid]
            if len(raw_path) < 2:
                continue
            color = AGV_COLORS[aid % len(AGV_COLORS)]
            a = TAPGAgent(aid, color, raw_path)
            self.agv_agents.append(a)

        self.agv_env = TAPGEnvironment(self.amr_graph, accel=50.0, decel=50.0)
        if self.agv_agents:
            self.agv_env.setup(self.agv_agents, t_start=t_start)

        self._plan_status = f'Planned {len(self.agv_agents)} AGVs'

    def _replan_done_agvs(self, sim_time: float):
        done_ids = set(self._agv_pending_replan)
        self.agv_env.recompute_earliest_schedule(current_time=sim_time)

        # Build constraints from active agents
        all_constraints = []
        for a in self.agv_agents:
            if a.id in done_ids:
                continue
            idx = a.path_idx
            if idx >= len(a.raw_path):
                continue

            cur_sid, cur_t = a.raw_path[idx]
            cur_state = self.agv_planner._get_state(cur_sid)
            if cur_state is not None:
                if idx + 1 < len(a.raw_path):
                    t_end_cur = max(a.raw_path[idx + 1][1], sim_time)
                else:
                    t_end_cur = float('inf')
                all_constraints.append({
                    'agent': a.id, 'loc': cur_sid,
                    'timestep': (sim_time, t_end_cur),
                })
                for aff_id in cur_state.affect_state:
                    aff = self.agv_planner._get_state(aff_id)
                    aff_cost = aff.cost if aff else 0.0
                    all_constraints.append({
                        'agent': a.id, 'loc': aff_id,
                        'timestep': (max(0.0, sim_time - aff_cost), t_end_cur),
                    })

            claimed_path = a.raw_path[idx:a.claim_idx]
            if claimed_path:
                for ci, (sid, t) in enumerate(claimed_path):
                    state = self.agv_planner._get_state(sid)
                    if state is None:
                        continue
                    if ci + 1 < len(claimed_path):
                        t_end = claimed_path[ci + 1][1]
                    else:
                        unclaimed_rest = a.raw_path[a.claim_idx:]
                        if unclaimed_rest:
                            t_end = unclaimed_rest[0][1]
                        else:
                            t_end = t + (state.cost if state.cost else 0)
                    all_constraints.append({
                        'agent': a.id, 'loc': sid,
                        'timestep': (sim_time, t_end),
                    })
                    for aff_id in state.affect_state:
                        aff = self.agv_planner._get_state(aff_id)
                        aff_cost = aff.cost if aff else 0.0
                        all_constraints.append({
                            'agent': a.id, 'loc': aff_id,
                            'timestep': (max(0.0, sim_time - aff_cost), t_end),
                        })

            unclaimed_path = a.raw_path[a.claim_idx:]
            if unclaimed_path:
                cs = self.agv_planner._build_constraints(unclaimed_path, a.id)
                all_constraints.extend(cs)

        # Done agents' positions
        positions_to_plan = {}
        for a in self.agv_agents:
            if a.id in done_ids:
                last_sid = a.raw_path[-1][0]
                nid = last_sid.split(',')[1]
                positions_to_plan[a.id] = nid

        # Assign goals
        occupied = set()
        for a in self.agv_agents:
            if a.id not in done_ids and a.raw_path:
                occupied.add(a.raw_path[-1][0].split(',')[1])
        targeted = {g for aid, g in self._agv_goals.items() if aid not in done_ids}
        used = occupied | targeted | set(positions_to_plan.values())
        ports = list(self.agv_planner._port_nodes)
        random.shuffle(ports)

        new_goals = {}
        for aid, cur in positions_to_plan.items():
            for p in ports:
                if p != cur and p not in used:
                    new_goals[aid] = p
                    used.add(p)
                    break
            else:
                for p in ports:
                    if p != cur:
                        new_goals[aid] = p
                        break

        result = self.agv_planner.plan(
            positions_to_plan, new_goals,
            existing_constraints=all_constraints,
            start_times={aid: sim_time for aid in positions_to_plan},
        )

        if result is None or not result.paths:
            self._plan_status = 'Replan FAILED'
            self._agv_pending_replan.clear()
            return

        for aid, goal in new_goals.items():
            self._agv_goals[aid] = goal

        batch = {}
        for aid, new_path in result.paths.items():
            if not new_path or len(new_path) < 2:
                continue
            if self.agv_env.agents.get(aid) is None:
                continue
            batch[aid] = new_path

        if batch:
            self.agv_env.extend_agents_batch(batch, sim_time)

        self._plan_status = f'Replanned {len(result.paths)} AGV @ t={sim_time:.0f}s'
        self._agv_pending_replan.clear()

    def _check_agv_collisions(self):
        if not self.agv_agents:
            return
        THRESHOLD = self.amr_graph.vehicle_length
        t = self.sim_time
        t_bucket = int(t * 2)

        active = [(a, a.x, a.y) for a in self.agv_agents]
        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                ai, ax, ay = active[i]
                aj, bx, by = active[j]
                d = math.hypot(ax - bx, ay - by)
                if d < THRESHOLD:
                    pair_key = (min(ai.id, aj.id), max(ai.id, aj.id), t_bucket)
                    if pair_key in self._collision_pairs_logged:
                        continue
                    self._collision_pairs_logged.add(pair_key)
                    self._collision_count += 1
                    msg = (f'[COLLISION #{self._collision_count}] t={t:.2f}s '
                           f'dist={d:.1f} A{ai.id} vs A{aj.id}')
                    self._collision_log_f.write(msg + '\n')
                    self._collision_log_f.flush()
                    print(msg)

    # ── Buttons ──────────────────────────────────────────────────────────────

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
                b = Button((sx+col*bw, y, bw-2, H-4), SIM_SPEED_LABELS[idx],
                           toggle=True)
                b.active = (idx == self.spd_idx)
                self.btns_spd.append(b)
            y += H-4+2
        y += G+4

        # AGV controls
        self.btn_agv_del = Button((sx, y, 30, H-4), '-', base=(80,40,40))
        self.btn_agv_add = Button((sx+sw-30, y, 30, H-4), '+')
        self._agv_count_rect = pygame.Rect(sx+32, y, sw-64, H-4)
        y += H-4+G

        self.btn_shuffle = Button((sx, y, sw, H-4), 'S: Shuffle All'); y += H-4+G

        self._info_y = y + 4
        self.all_btns = [self.btn_start, self.btn_reset, *self.btns_spd,
                         self.btn_agv_add, self.btn_agv_del, self.btn_shuffle]

    # ── Update ───────────────────────────────────────────────────────────────

    def update(self, dt_real: float):
        if not self.running:
            return
        dt_sim = dt_real * SIM_SPEEDS[self.spd_idx]
        self.sim_time += dt_sim

        self.agv_env.step(self.sim_time)

        self._check_agv_collisions()

        done_agents = [a for a in self.agv_agents if a.state == AGV_DONE
                       and a.id not in self._agv_pending_replan]
        if done_agents:
            for a in done_agents:
                self._agv_pending_replan.add(a.id)
            self._replan_done_agvs(self.sim_time)

    # ── Input ────────────────────────────────────────────────────────────────

    def handle_events(self):
        ww, wh = self.screen.get_size()
        mpos = pygame.mouse.get_pos()
        for b in self.all_btns:
            b.update_hover(mpos)

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                return False
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    return False
                elif ev.key == pygame.K_SPACE:
                    self.running = not self.running
                elif ev.key == pygame.K_r:
                    self._reset()
                elif ev.key == pygame.K_s:
                    self._shuffle()
                elif ev.key == pygame.K_n:
                    self._add_agv()
                elif ev.key == pygame.K_p:
                    self._remove_agv()
                elif ev.key == pygame.K_EQUALS or ev.key == pygame.K_PLUS:
                    self.spd_idx = min(self.spd_idx + 1, len(SIM_SPEEDS) - 1)
                    self._update_speed_btns()
                elif ev.key == pygame.K_MINUS:
                    self.spd_idx = max(self.spd_idx - 1, 0)
                    self._update_speed_btns()
                elif ev.key == pygame.K_d:
                    self._dump_status()
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.button == 1:
                    if mpos[0] < MAP_W:
                        self.cam.on_down(mpos)
                    else:
                        self._handle_button_click(mpos)
                elif ev.button in (4, 5):
                    self.cam.on_scroll(mpos, ev.button == 4)
            elif ev.type == pygame.MOUSEBUTTONUP:
                if ev.button == 1:
                    self.cam.on_up()
            elif ev.type == pygame.MOUSEMOTION:
                self.cam.on_move(mpos)
            elif ev.type == pygame.MOUSEWHEEL:
                self.cam.on_scroll(mpos, ev.y > 0)
            elif ev.type == pygame.VIDEORESIZE:
                pass
        return True

    def _handle_button_click(self, pos):
        if self.btn_start.clicked(pos):
            self.running = not self.running
        elif self.btn_reset.clicked(pos):
            self._reset()
        elif self.btn_shuffle.clicked(pos):
            self._shuffle()
        elif self.btn_agv_add.clicked(pos):
            self._add_agv()
        elif self.btn_agv_del.clicked(pos):
            self._remove_agv()
        else:
            for i, b in enumerate(self.btns_spd):
                if b.clicked(pos):
                    self.spd_idx = i
                    self._update_speed_btns()
                    break

    def _update_speed_btns(self):
        for i, b in enumerate(self.btns_spd):
            b.active = (i == self.spd_idx)

    def _reset(self):
        self.sim_time = 0.0
        self.running = False
        self._agv_pending_replan.clear()
        self._agv_next_id = 0
        self.agv_agents = []
        self._agv_goals = {}
        self._collision_count = 0
        self._collision_pairs_logged.clear()
        self._agv_start_positions = {}
        self._init_agv_agents()

    def _shuffle(self):
        self.sim_time = 0.0
        self.running = False
        self._agv_pending_replan.clear()
        old_n = len(self.agv_agents)
        self._agv_next_id = 0
        self.agv_agents = []
        self._agv_goals = {}
        self._agv_start_positions = {}
        self._n_agv = old_n
        self._init_agv_agents()

    def _add_agv(self):
        if len(self.agv_agents) >= MAX_AGENTS:
            return
        self._n_agv = len(self.agv_agents) + 1
        self._reset()

    def _remove_agv(self):
        if self._n_agv <= 1:
            return
        self._n_agv -= 1
        self._reset()

    def _dump_status(self):
        print(f'\n{"="*60}')
        print(f't={self.sim_time:.2f}s  AGVs={len(self.agv_agents)}  '
              f'collisions={self._collision_count}')
        for a in self.agv_agents:
            goal = self._agv_goals.get(a.id, '?')
            print(f'  A{a.id}: state={a.state} goal={goal} '
                  f'pos=({a.x:.1f},{a.y:.1f}) v={a.v:.1f} '
                  f'idx={a.path_idx}/{len(a.raw_path)}')
        print(f'{"="*60}\n')

    # ── Draw ─────────────────────────────────────────────────────────────────

    def draw(self):
        self.screen.fill(BG)

        # Draw map
        self._draw_map()

        # Draw AGVs
        self._draw_agvs()

        # Side panel
        self._draw_side_panel()

        pygame.display.flip()

    def _draw_map(self):
        cam = self.cam

        # Edges
        for edge in self.bg_edges:
            sn = self.bg_nodes.get(edge['start'])
            en = self.bg_nodes.get(edge['end'])
            if not sn or not en:
                continue
            p1 = cam.to_screen(sn['x'], sn['y'])
            p2 = cam.to_screen(en['x'], en['y'])

            etype = edge.get('type', '')
            if etype == 'internal':
                col = (50, 60, 75)
            elif etype == 'port':
                col = (50, 100, 70)
            else:
                col = COL_SEG

            bidir = edge.get('bidirectional', False)
            if bidir:
                pygame.draw.line(self.screen, col, p1, p2, 1)
            else:
                draw_arrow(self.screen, COL_SEG_ARR, p1, p2, 1, 4)

        # Nodes
        for nid, nd in self.bg_nodes.items():
            p = cam.to_screen(nd['x'], nd['y'])
            if nid in self.port_ids:
                pygame.draw.circle(self.screen, COL_PORT, p, 5)
                lbl = self.font_s.render(nid, True, COL_PORT)
                self.screen.blit(lbl, (p[0] + 6, p[1] - 6))
            else:
                pygame.draw.circle(self.screen, COL_NODE, p, 2)

    def _draw_agvs(self):
        cam = self.cam
        vl = self.amr_graph.vehicle_length
        vw = self.amr_graph.vehicle_width

        for a in self.agv_agents:
            sx, sy = cam.to_screen(a.x, a.y)
            length_px = max(cam.px(vl), 6)
            width_px  = max(cam.px(vw), 4)

            angle_deg = math.degrees(a.theta)

            # State-based coloring
            if a.state == AGV_WAITING:
                color = (255, 210, 50)
            elif a.state == AGV_DONE:
                color = (100, 100, 100)
            else:
                color = a.color

            draw_rotated_rect(self.screen, color, sx, sy,
                              length_px, width_px, -angle_deg,
                              border=COL_WHITE, border_w=1)

            # Agent label
            lbl = self.font_s.render(f'A{a.id}', True, COL_WHITE)
            self.screen.blit(lbl, (sx - lbl.get_width()//2,
                                    sy - lbl.get_height() - 2))

            # Path preview (next few nodes)
            if a.state not in (AGV_DONE,):
                node_path = a.node_path
                nidx = 0
                for ni, nid in enumerate(node_path):
                    node = self.amr_graph.nodes.get(nid)
                    if node:
                        np_scr = cam.to_screen(node.x, node.y)
                        if ni > 0:
                            prev_node = self.amr_graph.nodes.get(node_path[ni-1])
                            if prev_node:
                                pp = cam.to_screen(prev_node.x, prev_node.y)
                                col = tuple(max(c//3, 30) for c in a.color)
                                pygame.draw.line(self.screen, col, pp, np_scr, 1)

            # Goal marker
            goal = self._agv_goals.get(a.id)
            if goal:
                gnode = self.amr_graph.nodes.get(goal)
                if gnode:
                    gp = cam.to_screen(gnode.x, gnode.y)
                    pygame.draw.circle(self.screen, a.color, gp, 4, 1)

    def _draw_side_panel(self):
        sx = MAP_W
        pygame.draw.rect(self.screen, (25, 28, 35),
                         (sx, 0, SIDE_W, self.screen.get_height()))

        # Buttons
        for b in self.all_btns:
            b.draw(self.screen, self.font_s)

        # AGV count display
        pygame.draw.rect(self.screen, (35, 40, 50), self._agv_count_rect,
                         border_radius=3)
        cnt = self.font_m.render(f'AGV: {len(self.agv_agents)}', True, COL_TEXT)
        self.screen.blit(cnt, cnt.get_rect(center=self._agv_count_rect.center))

        # Info
        y = self._info_y
        info_lines = [
            f'Time: {self.sim_time:.1f}s',
            f'Speed: {SIM_SPEED_LABELS[self.spd_idx]}',
            f'Status: {"Running" if self.running else "Paused"}',
            f'',
            f'{self._plan_status}',
            f'Collisions: {self._collision_count}',
            f'',
        ]
        # Per-agent info
        for a in self.agv_agents:
            goal = self._agv_goals.get(a.id, '?')
            info_lines.append(f'A{a.id}: {a.state} → {goal}')

        for line in info_lines:
            lbl = self.font_s.render(line, True, COL_TEXT)
            self.screen.blit(lbl, (sx + 10, y))
            y += 16

    # ── Main loop ────────────────────────────────────────────────────────────

    def run(self):
        while True:
            dt = self.clock.tick(FPS) / 1000.0
            if not self.handle_events():
                break
            self.update(dt)
            self.draw()

        self._collision_log_f.close()
        pygame.quit()


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--agvs', type=int, default=3, help='Number of AGVs')
    args = ap.parse_args()

    sim = SongdoSimulator(n_agv=args.agvs)
    sim.run()


if __name__ == '__main__':
    main()
