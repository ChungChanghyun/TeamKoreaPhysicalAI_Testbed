"""
vis_combined.py — OHT + AGV 통합 시뮬레이터 (KaistTB map)

OHT: OHT_A 서브네트워크, OHTEnvironmentDES (세그먼트 큐 + 전방감지)
AGV: AMR_A PklMapGraph, PklPrioritizedPlanner + TAPGEnvironment (SIPP + TAPG DAG)

Controls
────────
  SPACE     : Start / Pause
  R         : Reset
  S         : Shuffle paths (both OHT & AGV)
  O / L     : Add / Remove OHT vehicle
  N / P     : Add / Remove AGV vehicle
  +/-       : Sim speed up / down
  Mouse drag: Pan
  Wheel     : Zoom
  Q / ESC   : Quit
"""
from __future__ import annotations
import sys, os, json, math, random, collections
import pygame

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from env_oht_des import (OHTMap, OHTAgent, OHTEnvironmentDES,
                          IDLE, MOVING, FOLLOWING, BLOCKED, DONE)
from env_tapg import (TAPGAgent, TAPGEnvironment,
                      IDLE as AGV_IDLE, MOVING as AGV_MOVING,
                      WAITING as AGV_WAITING, ROTATING as AGV_ROTATING,
                      DONE as AGV_DONE)
from pkl_loader import PklMapGraph
from pkl_prioritized_planner import PklPrioritizedPlanner
from env_3ds import FloorGraph
from elevator import (Elevator, ElevatorController, LiftRequest,
                      IDLE as LIFT_IDLE, MOVING as LIFT_MOVING,
                      LOADING as LIFT_LOADING, UNLOADING as LIFT_UNLOADING)

JSON_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'Maps', 'KaistTB.map_latest.json')
AMR_PKL   = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'KaistTB_AMR_A.pkl')
COLLISION_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'collision_log.txt')

# ── Colors ────────────────────────────────────────────────────────────────────
BG          = (18,  20,  28)
COL_SEG     = (40,  55,  70)
COL_SEG_ARR = (60,  80, 100)
COL_NODE    = (70,  85, 100)
COL_PORT    = (60, 220, 120)
COL_TEXT    = (200, 210, 220)
COL_DIM     = (80,  90, 100)
COL_WHITE   = (255, 255, 255)

COL_FOLLOWING = (255, 210,  50)
COL_BLOCKED   = (220,  60,  60)
COL_HEADLIGHT = (255, 255, 180)
COL_ZCU_FREE  = (60,  180,  60)
COL_ZCU_HELD  = (220, 100,  40)

# area → 배경 노드/세그먼트 색상
AREA_COLORS = {
    '3DS_F1': (35, 60, 80),
    '3DS_F2': (35, 70, 55),
    '3DS_F3': (65, 50, 35),
    'OHT_A':  (60, 35, 70),
    'AMR_A':  (65, 65, 30),
    '':       (50, 55, 65),
}
AREA_SEG_COLORS = {
    '3DS_F1': (50, 80, 110),
    '3DS_F2': (50, 95,  70),
    '3DS_F3': (90, 70,  45),
    'OHT_A':  (85, 50, 100),
    'AMR_A':  (90, 90,  35),
    '':       (60, 70,  80),
}

# OHT agent colors (purple / blue tones)
OHT_COLORS = [
    (180,  80, 200), (140, 100, 255), (200, 120, 255),
    (120,  80, 220), (180, 140, 255),
]
# AGV agent colors (green / yellow / orange tones)
AGV_COLORS = [
    (100, 220, 120), (255, 200,  60), (255, 140,  60),
    (80,  200, 200), (200, 255, 100),
]

# 3DS shuttle colors per floor
S3D_COLORS = {
    '3DS_F1': (80,  180, 255),
    '3DS_F2': (80,  230, 130),
    '3DS_F3': (255, 180,  80),
}
S3D_FLOOR_IDS = ['3DS_F1', '3DS_F2', '3DS_F3']

# ── Area layout offsets ──────────────────────────────────────────────────────
# 각 area를 겹치지 않게 2D에 펼치기 위한 (dx, dy) 오프셋 (mm 단위).
# 원본 좌표 범위:
#   AMR_A : x=[530,18152]  y=[740,14284]   w=17622 h=13544
#   OHT_A : x=[1870,17890] y=[890,3993]    w=16020 h=3103
#   3DS_*:  x=[1115,11115] y=[12721,14721] w=10000 h=2000 (3층 겹침)
#
# 레이아웃:  AMR_A(좌)  |  3DS(우, 세로 3층)  |  OHT_A(하단)
GAP = 1500  # area 간 간격 (mm)

# 레이아웃:
#   좌측 세로: AMR_A(위) → OHT_A(아래, 밀착)
#   우측 가로: 3DS_F1 | 3DS_F2 | 3DS_F3
#
# 원본 좌표:
#   AMR_A : x=[530,18152]  y=[740,14284]   w=17622 h=13544
#   OHT_A : x=[1870,17890] y=[890,3993]    w=16020 h=3103
#   3DS_*:  x=[1115,11115] y=[12721,14721] w=10000 h=2000
_3DS_W = 10000  # 3DS 한 층 원본 폭
_3DS_H = 2000   # 3DS 한 층 원본 높이
_3DS_SCALE = 1.5 # 3DS 표시 스케일 (실제 좌표 대비)

# 레이아웃 (위→아래):
#   3DS (아이소메트릭 적층: F3 위, F2 중간, F1 아래)
#   AMR_A
#   OHT_A
_AMR_TOP = 14284
_3DS_BOT = 12721
_3DS_BASE_Y = _AMR_TOP + GAP - _3DS_BOT
_3DS_BASE_X = -1115 + 530

# 아이소메트릭 층간 오프셋 (스케일 적용된 크기 기준)
_ISO_DX = 2000                       # 층당 x 이동
_ISO_DY = _3DS_H * _3DS_SCALE + GAP  # 층당 y 이동 (스케일된 높이 + 간격)

AREA_OFFSETS = {
    'AMR_A':  (0, 0),
    'OHT_A':  (0, 740 - GAP - 3993),
    '3DS_F1': (_3DS_BASE_X, _3DS_BASE_Y),
    '3DS_F2': (_3DS_BASE_X + _ISO_DX, _3DS_BASE_Y + _ISO_DY),
    '3DS_F3': (_3DS_BASE_X + 2*_ISO_DX, _3DS_BASE_Y + 2*_ISO_DY),
}
# area 미지정 노드 (포트 등)
_DEFAULT_OFFSET = (0, 0)

SIM_SPEEDS       = [0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0]
SIM_SPEED_LABELS = ['0.1×', '0.25×', '0.5×', '1×', '2×', '4×', '8×', '16×']
DEFAULT_SPD_IDX  = 3

WIN_W, WIN_H = 1400, 900
SIDE_W       = 260
MAP_W        = WIN_W - SIDE_W
MAP_PAD      = 40
FPS          = 60
MAX_AGENTS   = 10


# ── Full-map background data ────────────────────────────────────────────────

def _area_offset(area: str):
    """Return (dx, dy) offset for the given area."""
    return AREA_OFFSETS.get(area, _DEFAULT_OFFSET)


def _is_3ds_area(area: str) -> bool:
    return area.startswith('3DS_F')


# 3DS 좌표 중심 (스케일 기준점)
_3DS_CX = 1115 + _3DS_W / 2   # 원본 x 중심
_3DS_CY = 12721 + _3DS_H / 2  # 원본 y 중심


def _transform_3ds(x: float, y: float, area: str):
    """3DS 노드 좌표에 스케일 + offset 적용."""
    dx, dy = _area_offset(area)
    # 중심 기준 스케일
    sx = _3DS_CX + (x - _3DS_CX) * _3DS_SCALE + dx
    sy = _3DS_CY + (y - _3DS_CY) * _3DS_SCALE + dy
    return sx, sy


def load_full_map(json_path: str):
    """Load all nodes & segments for background rendering, applying area offsets."""
    with open(json_path, 'r', encoding='utf-8') as f:
        d = json.load(f)
    nodes = {}
    for n in d['nodes']:
        area = n.get('area', '')
        if _is_3ds_area(area):
            nx, ny = _transform_3ds(n['x'], n['y'], area)
            nodes[n['id']] = {**n, 'x': nx, 'y': ny}
        else:
            dx, dy = _area_offset(area)
            nodes[n['id']] = {**n, 'x': n['x'] + dx, 'y': n['y'] + dy}
    segments = d['segments']
    ports = {p['nodeId']: p for p in d.get('ports', [])}
    chargers = {c['nodeId'] for c in d.get('chargers', [])}
    xs = [n['x'] for n in nodes.values()]
    ys = [n['y'] for n in nodes.values()]
    bbox = (min(xs), min(ys), max(xs), max(ys))
    return nodes, segments, ports, chargers, bbox


# ── Camera ────────────────────────────────────────────────────────────────────

class Camera:
    def __init__(self, bbox, w, h, pad=MAP_PAD):
        x0, y0, x1, y1 = bbox
        mw, mh = x1 - x0, y1 - y0
        sx = (w - 2*pad) / mw if mw > 0 else 1
        sy = (h - 2*pad) / mh if mh > 0 else 1
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


def draw_dashed_line(surf, color, p1, p2, width=1, dash_len=8, gap_len=5):
    """Draw a dashed line from p1 to p2."""
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    dist = math.hypot(dx, dy)
    if dist < 1:
        return
    ux, uy = dx / dist, dy / dist
    drawn = 0.0
    while drawn < dist:
        seg_end = min(drawn + dash_len, dist)
        x1 = int(p1[0] + ux * drawn)
        y1 = int(p1[1] + uy * drawn)
        x2 = int(p1[0] + ux * seg_end)
        y2 = int(p1[1] + uy * seg_end)
        pygame.draw.line(surf, color, (x1, y1), (x2, y2), width)
        drawn += dash_len + gap_len


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


# ── AMR_A sub-network is loaded from pre-generated pkl ──────────────────────
# Run gen_amr_pkl.py to generate KaistTB_AMR_A.pkl before first use.


# ── Combined Simulator ───────────────────────────────────────────────────────

class CombinedSimulator:
    def __init__(self, oht_map: OHTMap, amr_graph: PklMapGraph,
                 n_oht: int = 5, n_agv: int = 3, n_s3d: int = 2):
        self.oht_map   = oht_map
        self.amr_graph = amr_graph
        self._n_oht  = min(n_oht, MAX_AGENTS)
        self._n_agv  = min(n_agv, MAX_AGENTS)
        self._n_s3d  = n_s3d

        # Load full map for background
        self.bg_nodes, self.bg_segments, self.bg_ports, self.bg_chargers, self.bg_bbox = \
            load_full_map(JSON_FILE)
        self._node_area = {n['id']: n.get('area', '') for n in self.bg_nodes.values()}

        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
        pygame.display.set_caption('OHT + AGV Combined Simulator — KaistTB')
        self.clock  = pygame.time.Clock()
        self.font_s = pygame.font.SysFont('Consolas', 12)
        self.font_m = pygame.font.SysFont('Consolas', 14)
        self.font_b = pygame.font.SysFont('Consolas', 15, bold=True)

        self.sim_time = 0.0
        self.running  = False
        self.spd_idx  = DEFAULT_SPD_IDX

        self.cam = Camera(self.bg_bbox, MAP_W, WIN_H)
        self._build_buttons()

        # OHT agents (segment-queue DES)
        self._oht_next_id = 0
        self.oht_agents: list[OHTAgent] = []
        self.oht_env = OHTEnvironmentDES(oht_map, cross_segment=True)
        self._init_oht_agents()

        # AGV agents (prioritized SIPP + TAPG execution)
        self._agv_next_id = 100   # offset to avoid ID collisions
        self.agv_agents: list[TAPGAgent] = []
        self.agv_env = TAPGEnvironment(amr_graph, accel=500.0, decel=500.0)
        self.agv_planner = PklPrioritizedPlanner(amr_graph)
        self._agv_goals: dict = {}       # aid → current goal node
        self._agv_pending_replan: set = set()   # agents awaiting replan
        self._plan_status = ''
        self._collision_log_f = open(COLLISION_LOG, 'w', encoding='utf-8')
        self._collision_pairs_logged: set = set()
        self._collision_count = 0
        self._plan_log_counter = 0
        # 로그 초기화
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
        os.makedirs(log_dir, exist_ok=True)
        plan_log = os.path.join(log_dir, 'plan_log.txt')
        with open(plan_log, 'w', encoding='utf-8') as f:
            f.write(f'Plan Log started\n')
        self._init_agv_agents()

        # 3DS shuttles (one per floor, simple path-following)
        self._init_3ds()

    # ── 3DS setup (SIPP + TAPG, same as AGV) ──────────────────────────────

    def _init_3ds(self):
        """Load 3DS pkl per floor → SIPP planner + TAPG env (AGV와 동일 구조)."""
        self._s3d_next_id = 200
        self.s3d_floor_data = {}
        self.s3d_agents = []

        for fid in S3D_FLOOR_IDS:
            pkl_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                f'KaistTB_{fid}.pkl')
            if not os.path.exists(pkl_path):
                print(f'[WARN] {pkl_path} not found — run gen_3ds_pkl.py first')
                continue

            # PklMapGraph (정밀 collision profile)
            graph = PklMapGraph(pkl_path)

            # Area offset + scale 적용
            for node in graph.nodes.values():
                node.x, node.y = _transform_3ds(node.x, node.y, fid)

            planner  = PklPrioritizedPlanner(graph)
            tapg_env = TAPGEnvironment(graph, accel=500.0, decel=500.0)

            # 셔틀 배치
            port_nodes = list(planner._port_nodes)
            random.shuffle(port_nodes)
            n_shuttles = min(self._n_s3d, len(port_nodes))

            start_positions = {}
            for k in range(n_shuttles):
                aid = self._s3d_next_id
                self._s3d_next_id += 1
                start_positions[aid] = port_nodes[k]

            goals = planner.assign_random_goals(start_positions)
            result = planner.plan(start_positions, goals,
                                  start_times={aid: 0.0 for aid in start_positions})

            agents = []
            if result and result.paths:
                base_col = S3D_COLORS.get(fid, (150, 150, 150))
                for i, aid in enumerate(sorted(result.paths)):
                    raw_path = result.paths[aid]
                    if len(raw_path) < 2:
                        continue
                    brightness = 1.0 if i == 0 else 0.7
                    color = tuple(max(0, min(255, int(c * brightness)))
                                  for c in base_col)
                    a = TAPGAgent(aid, color, raw_path)
                    agents.append(a)
                    self.s3d_agents.append(a)
                if agents:
                    tapg_env.setup(agents, t_start=0.0)

            self.s3d_floor_data[fid] = {
                'graph': graph,
                'planner': planner,
                'env': tapg_env,
                'agents': agents,
                'goals': goals,
                'pending_replan': set(),
            }

        # ── Elevators ──
        self._init_elevators()

    def _init_elevators(self):
        """KaistTB 맵에서 엘리베이터 생성."""
        import json as _json
        with open(JSON_FILE, 'r', encoding='utf-8') as f:
            jdata = _json.load(f)

        # 독립 힙 (3DS TAPG 힙과 별도 — 엘리베이터는 시간 기반 DES)
        self._lift_heap = []
        self.lift_ctrl = ElevatorController(self._lift_heap)

        # 층 높이 (areas.viewShift.y 또는 노드 z 좌표)
        area_heights = {'1': 0.0, '2': 3000.0, '3': 6000.0}

        for lift_data in jdata.get('lifts', []):
            self.lift_ctrl.add_lift_from_map(
                lift_data, area_heights,
                speed=1000.0,       # mm/s (수직)
                xfer_duration=3.0,  # 적재/하역 3초
                capacity=1,
            )

        # 주기적 자동 요청 타이머
        self._lift_next_request_t = 5.0   # 첫 요청 시각
        self._lift_request_interval = 15.0  # 요청 간격

        n_lifts = len(self.lift_ctrl.lifts)
        if n_lifts:
            print(f'  Elevators: {n_lifts} lifts loaded')

    def _step_elevators(self, sim_time: float):
        """엘리베이터 DES step + 주기적 자동 요청."""
        # 힙 이벤트 처리
        import heapq as _hq
        while self._lift_heap and self._lift_heap[0].t <= sim_time:
            ev = _hq.heappop(self._lift_heap)
            self.lift_ctrl.handle_event(ev)

        # 주기적 자동 요청 (데모용)
        if sim_time >= self._lift_next_request_t:
            self._lift_next_request_t = sim_time + self._lift_request_interval
            floors = ['1', '2', '3']
            from_f = random.choice(floors)
            to_f = random.choice([f for f in floors if f != from_f])
            result = self.lift_ctrl.request_auto(from_f, to_f, sim_time)
            if result:
                lid, rid = result
                lift = self.lift_ctrl.get_lift(lid)

    def _replan_3ds_floor(self, fid: str, sim_time: float):
        """한 층의 DONE 셔틀을 순차 replan — 한 대씩 계획 후 constraint 추가."""
        fd = self.s3d_floor_data[fid]
        planner = fd['planner']
        tapg_env = fd['env']
        agents = fd['agents']
        done_ids = list(fd['pending_replan'])

        if not done_ids:
            return

        # 기존 active agent의 constraint 수집
        base_constraints = []
        for a in agents:
            if a.id in done_ids:
                continue
            idx = a.path_idx
            if idx >= len(a.raw_path):
                # active지만 경로 끝 → 현재 위치를 영구 block
                nid = a.raw_path[-1][0].split(',')[1]
                stop_sid = f'S,{nid},0'
                base_constraints.append({
                    'agent': a.id, 'loc': stop_sid,
                    'timestep': (sim_time, float('inf')),
                })
                # affect state도 block
                state = planner._get_state(stop_sid)
                if state:
                    for aff_id in state.affect_state:
                        base_constraints.append({
                            'agent': a.id, 'loc': aff_id,
                            'timestep': (sim_time, float('inf')),
                        })
                continue
            remaining = a.raw_path[idx:]
            cs = planner._build_constraints(remaining, a.id)
            base_constraints.extend(cs)

        # DONE 셔틀들의 현재 위치도 constraint로 추가
        # (아직 replan 안 된 셔틀이 앉아있는 노드 보호)
        done_positions = {}
        for aid in done_ids:
            a = tapg_env.agents.get(aid)
            if a and a.raw_path:
                nid = a.raw_path[-1][0].split(',')[1]
                done_positions[aid] = nid

        # 점유/목표 노드 추적 (AGV와 동일 로직)
        occupied = set()
        for a in agents:
            if a.id not in done_ids and a.raw_path:
                occupied.add(a.raw_path[-1][0].split(',')[1])
        targeted = {g for aid, g in fd['goals'].items() if aid not in done_ids}
        used = occupied | targeted | set(done_positions.values())

        # 한 대씩 순차 replan
        all_constraints = list(base_constraints)

        # 아직 replan 안 된 DONE 셔틀의 위치를 임시 constraint로 추가
        pending_position_constraints = {}
        for aid in done_ids:
            nid = done_positions.get(aid)
            if nid is None:
                continue
            cs = []
            stop_sid = f'S,{nid},0'
            cs.append({'agent': aid, 'loc': stop_sid,
                       'timestep': (sim_time, float('inf'))})
            state = planner._get_state(stop_sid)
            if state:
                for aff_id in state.affect_state:
                    cs.append({'agent': aid, 'loc': aff_id,
                               'timestep': (sim_time, float('inf'))})
            pending_position_constraints[aid] = cs

        for aid in done_ids:
            a = tapg_env.agents.get(aid)
            if a is None:
                continue
            nid = done_positions.get(aid, '')
            used.add(nid)

            # 다른 DONE 셔틀의 위치를 constraint에 포함
            plan_constraints = list(all_constraints)
            for other_aid, cs in pending_position_constraints.items():
                if other_aid != aid:
                    plan_constraints.extend(cs)

            # 새 목표 선택
            ports = list(planner._port_nodes)
            random.shuffle(ports)
            goal = None
            for p in ports:
                if p != nid and p not in used:
                    goal = p
                    break
            if goal is None:
                for p in ports:
                    if p != nid:
                        goal = p
                        break
            if goal is None:
                continue

            result = planner.plan(
                {aid: nid}, {aid: goal},
                existing_constraints=plan_constraints,
                start_times={aid: sim_time},
            )

            if result and result.paths.get(aid):
                new_path = result.paths[aid]
                if len(new_path) >= 2:
                    fd['goals'][aid] = goal
                    used.add(goal)
                    tapg_env.extend_agents_batch({aid: new_path}, sim_time)
                    # 이 경로를 다음 셔틀의 constraint로 추가
                    cs = planner._build_constraints(new_path, aid)
                    all_constraints.extend(cs)
                    # 이 셔틀은 이제 움직이므로 위치 constraint 제거
                    pending_position_constraints.pop(aid, None)

        fd['pending_replan'].clear()

    # ── Agent setup ──────────────────────────────────────────────────────────

    def _make_oht_agent(self, aid: int, excluded: set) -> OHTAgent | None:
        nodes = list(self.oht_map.nodes.keys())
        random.shuffle(nodes)
        for start in nodes:
            if start in excluded:
                continue
            path = self.oht_map.bfs_path(start)
            if len(path) > 1:
                color = OHT_COLORS[aid % len(OHT_COLORS)]
                return OHTAgent(aid, color, path,
                                self.oht_map.vehicle_length * (1000/1108))
        return None

    def _init_oht_agents(self):
        excluded = set()
        for _ in range(self._n_oht):
            a = self._make_oht_agent(self._oht_next_id, excluded)
            if a:
                excluded |= self.oht_map.nearby_nodes(a.node_path[0],
                                                       self.oht_map.h_min)
                self._oht_next_id += 1
                self.oht_agents.append(a)
                self.oht_env.add_agent(a, t_start=0.0)

    def _init_agv_agents(self):
        """Create AGV agents at port positions, plan paths to other ports."""
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
        """Run prioritized SIPP → create TAPGAgents → setup TAPGEnvironment."""
        if not self._agv_start_positions and not self.agv_agents:
            return

        # Current positions: from start_positions (init) or current agent positions
        positions = {}
        if self.agv_agents:
            for a in self.agv_agents:
                # Extract node from current state
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

        # Create TAPGAgents from planned state-level paths
        self.agv_agents = []
        for i, aid in enumerate(sorted(result.paths)):
            raw_path = result.paths[aid]
            if len(raw_path) < 2:
                continue
            color = AGV_COLORS[(aid - 100) % len(AGV_COLORS)]
            a = TAPGAgent(aid, color, raw_path)
            self.agv_agents.append(a)

        # Setup TAPG environment
        self.agv_env = TAPGEnvironment(self.amr_graph, accel=500.0, decel=500.0)
        if self.agv_agents:
            self.agv_env.setup(self.agv_agents, t_start=t_start)

        n_planned = len(self.agv_agents)
        self._plan_status = f'Planned {n_planned} AGVs'
        self._save_plan_snapshot('initial_plan')

    def _save_plan_snapshot(self, label: str):
        """Planning 시점의 전체 AGV 경로 + TAPG + D-key dump를 단일 로그 파일에 추가."""
        import pickle
        self._plan_log_counter += 1
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')

        # 1) 텍스트 로그: 단일 파일에 append
        log_path = os.path.join(log_dir, 'plan_log.txt')
        with open(log_path, 'a', encoding='utf-8') as f:
            f.write(f'\n{"="*80}\n')
            f.write(f'[PLAN #{self._plan_log_counter}] {label} | t={self.sim_time:.2f}s\n')
            f.write(f'{"="*80}\n\n')
            for a in self.agv_agents:
                goal = self._agv_goals.get(a.id, '?')
                f.write(f'A{a.id-100} state={a.state} goal={goal} '
                        f'path_idx={a.path_idx}/{len(a.raw_path)} '
                        f'claim_idx={a.claim_idx} '
                        f'pos=({a.x:.0f},{a.y:.0f})\n')
                for idx, (sid, t) in enumerate(a.raw_path):
                    markers = []
                    if idx == a.path_idx: markers.append('CURRENT')
                    if a.path_idx <= idx < a.claim_idx: markers.append('CLAIMED')
                    marker = f' <<< {",".join(markers)}' if markers else ''
                    f.write(f'  [{idx:3d}] {sid:35s} t={t:.4f}{marker}\n')
                f.write('\n')

            # D-key dump: TAPG 상태 + 각 agent의 BLOCKED_BY
            G = self.agv_env.G
            f.write(f'TAPG: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges\n\n')

            f.write('--- Agent dependency ---\n')
            for a in self.agv_agents:
                idx = a.path_idx
                if idx >= len(a.raw_path): continue
                # Find next M/R
                next_idx = idx
                while next_idx < len(a.raw_path) and a.raw_path[next_idx][0].startswith('S,'):
                    next_idx += 1
                if next_idx >= len(a.raw_path): continue
                ns, nt = a.raw_path[next_idx]
                nk = self.agv_env._nk(ns, a.id, nt)
                blockers = []
                if nk in G:
                    blockers = [f'A{p[1]-100}:{p[0][:20]}'
                                for p in G.predecessors(nk) if p[1] != a.id]
                if blockers:
                    f.write(f'  A{a.id-100} wants {ns} BLOCKED_BY={blockers}\n')
                else:
                    f.write(f'  A{a.id-100} wants {ns} (free)\n')
            f.write('\n')

        # 2) TAPG DAG pickle (최신 1개만 유지)
        try:
            tapg_data = {
                'G_nodes': list(self.agv_env.G.nodes(data=True)),
                'G_edges': list(self.agv_env.G.edges()),
                'agent_paths': {a.id: a.raw_path for a in self.agv_agents},
                'agent_states': {a.id: {
                    'state': a.state, 'path_idx': a.path_idx,
                    'claim_idx': a.claim_idx,
                    'x': a.x, 'y': a.y, 'v': a.v,
                    'goal': self._agv_goals.get(a.id),
                    '_tapg_node': a._tapg_node,
                } for a in self.agv_agents},
                'sim_time': self.sim_time,
                'label': label,
            }
            pkl_path = os.path.join(log_dir, 'latest_tapg.pkl')
            with open(pkl_path, 'wb') as f:
                pickle.dump(tapg_data, f)
        except Exception as e:
            print(f'[WARN] Failed to save TAPG: {e}')

    def _dump_agv_status(self):
        """D키: 현재 AGV 경로 + TAPG 상태를 콘솔에 출력."""
        print(f'\n{"="*80}')
        print(f'[DUMP] t={self.sim_time:.2f}s  AGVs={len(self.agv_agents)}  '
              f'TAPG nodes={self.agv_env.G.number_of_nodes()} '
              f'edges={self.agv_env.G.number_of_edges()}')
        print(f'{"="*80}')

        for a in self.agv_agents:
            goal = self._agv_goals.get(a.id, '?')
            print(f'\nA{a.id-100}: state={a.state} goal={goal} '
                  f'pos=({a.x:.0f},{a.y:.0f}) v={a.v:.0f} '
                  f'path_idx={a.path_idx}/{len(a.raw_path)}')

            # Remaining path (from current index)
            for i in range(a.path_idx, min(len(a.raw_path), a.path_idx + 15)):
                sid, t = a.raw_path[i]
                nk = self.agv_env._nk(sid, a.id, t)
                in_g = nk in self.agv_env.G
                # Cross predecessors still in DAG
                cross_pred = []
                if in_g:
                    cross_pred = [f'A{p[1]-100}:{p[0][:20]}'
                                  for p in self.agv_env.G.predecessors(nk)
                                  if p[1] != a.id]
                marker = ' <<< CURRENT' if i == a.path_idx else ''
                blocked = f' BLOCKED_BY={cross_pred}' if cross_pred else ''
                dag = '' if in_g else ' [NOT_IN_DAG]'
                print(f'  [{i:3d}] {sid:35s} t={t:.2f}{dag}{blocked}{marker}')

            if len(a.raw_path) > a.path_idx + 15:
                print(f'  ... +{len(a.raw_path) - a.path_idx - 15} more states')

        # TAPG cross edges summary
        G = self.agv_env.G
        cross_edges = [(u, v) for u, v in G.edges() if u[1] != v[1]]
        if cross_edges:
            print(f'\n--- Cross-agent edges ({len(cross_edges)}) ---')
            for u, v in cross_edges[:20]:
                print(f'  A{u[1]-100}:{u[0][:20]}(t={u[2]:.1f}) -> '
                      f'A{v[1]-100}:{v[0][:20]}(t={v[2]:.1f})')
            if len(cross_edges) > 20:
                print(f'  ... +{len(cross_edges)-20} more')

        print(f'{"="*80}\n')
        self._save_plan_snapshot('manual_dump')

    def _check_agv_collisions(self):
        """AGV 간 물리적 근접 감지 → collision_log.txt에 상세 정보 기록."""
        if not self.agv_agents:
            return
        THRESHOLD = self.amr_graph.vehicle_length  # 780mm
        t = self.sim_time
        t_bucket = int(t * 2)  # 0.5초 단위 dedup

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

                    # 상세 정보 수집
                    def agent_info(a):
                        idx = a.path_idx
                        cur_sid = a.raw_path[idx][0] if idx < len(a.raw_path) else '?'
                        cur_t = a.raw_path[idx][1] if idx < len(a.raw_path) else 0
                        tapg_node = a._tapg_node
                        goal = self._agv_goals.get(a.id, '?')
                        return (f'A{a.id-100} state={a.state} '
                                f'pos=({a.x:.0f},{a.y:.0f}) v={a.v:.0f} '
                                f'cur_state={cur_sid} path_idx={idx}/{len(a.raw_path)} '
                                f'tapg={tapg_node} goal={goal}')

                    # TAPG dependency 확인
                    tapg_info = ''
                    G = self.agv_env.G
                    # 현재 실행중인 TAPG 노드 간 edge 확인
                    if ai._tapg_node and aj._tapg_node:
                        has_ij = G.has_edge(ai._tapg_node, aj._tapg_node) if (ai._tapg_node in G and aj._tapg_node in G) else False
                        has_ji = G.has_edge(aj._tapg_node, ai._tapg_node) if (ai._tapg_node in G and aj._tapg_node in G) else False
                        ij_done = ai._tapg_node not in G if ai._tapg_node else True
                        ji_done = aj._tapg_node not in G if aj._tapg_node else True
                        tapg_info = (f'  TAPG edge {ai.id}->{aj.id}={has_ij} '
                                     f'{aj.id}->{ai.id}={has_ji} '
                                     f'completed: {ai.id}={ij_done} {aj.id}={ji_done}')

                    # raw_path에서 현재 state의 TAPG 노드 존재 여부 확인
                    for label, a in [('ai', ai), ('aj', aj)]:
                        idx = a.path_idx
                        if idx < len(a.raw_path):
                            sid, ct = a.raw_path[idx]
                            nk = (sid, a.id, round(ct, 6))
                            in_g = nk in G
                            is_completed = nk not in G
                            preds = []
                            if in_g:
                                preds = [(p[0][:25], p[1], f'{p[2]:.2f}',
                                          p not in G)
                                         for p in G.predecessors(nk) if p[1] != a.id]
                            tapg_info += (f'\n  {label} nk={nk} in_G={in_g} '
                                          f'completed={is_completed} '
                                          f'cross_pred={preds}')

                    msg = (f'[COLLISION #{self._collision_count}] t={t:.2f}s dist={d:.0f}mm\n'
                           f'  {agent_info(ai)}\n'
                           f'  {agent_info(aj)}\n'
                           f'{tapg_info}\n')

                    self._collision_log_f.write(msg + '\n')
                    self._collision_log_f.flush()
                    print(msg)

    def _replan_done_agvs(self, sim_time: float):
        """Incrementally replan finished AGVs. Active agents' TAPG is untouched."""
        done_ids = set(self._agv_pending_replan)

        # 0) TAPG 시간 보정: 실제 실행 시간과 계획 시간의 차이를 해소
        self.agv_env.recompute_earliest_schedule(current_time=sim_time)

        # 1) Build constraints from ALL active (non-done) agents' remaining paths
        #    Claimed 구간: 시간을 0~inf로 확장 (절대 침범 불가)
        #    Unclaimed 구간: 계획 시간 기반 constraint
        all_constraints = []
        for a in self.agv_agents:
            if a.id in done_ids:
                continue
            idx = a.path_idx
            if idx >= len(a.raw_path):
                continue

            # 현재 위치 물리적 점유: 에이전트가 지금 있는 위치를 sim_time부터
            # 무조건 block (claimed/unclaimed 시간 갭 방지)
            cur_sid, cur_t = a.raw_path[idx]
            cur_state = self.agv_planner._get_state(cur_sid)
            if cur_state is not None:
                # 현재 상태가 M/R이면: 출발+도착 노드 모두 block
                # 현재 상태가 S이면: 해당 노드 block
                # t_end: 다음 상태 시작 or inf
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

            # Claimed 구간 (path_idx ~ claim_idx):
            # 첫 action 시작 시간 ~ 각 action 종료 시간으로 block
            claimed_path = a.raw_path[idx:a.claim_idx]
            if claimed_path:
                t_claim_start = sim_time  # 현재 시간부터 점유

                for ci, (sid, t) in enumerate(claimed_path):
                    state = self.agv_planner._get_state(sid)
                    if state is None:
                        continue
                    # 이 action의 종료 시간
                    if ci + 1 < len(claimed_path):
                        t_end = claimed_path[ci + 1][1]
                    else:
                        # 마지막 claimed step → unclaimed 구간 시작까지 연장
                        # (에이전트가 해당 노드에서 대기 중이므로 점유 지속)
                        unclaimed_rest = a.raw_path[a.claim_idx:]
                        if unclaimed_rest:
                            t_end = unclaimed_rest[0][1]
                        else:
                            t_end = t + (state.cost if state.cost else 0)

                    all_constraints.append({
                        'agent': a.id, 'loc': sid,
                        'timestep': (t_claim_start, t_end),
                    })
                    for aff_id in state.affect_state:
                        aff = self.agv_planner._get_state(aff_id)
                        aff_cost = aff.cost if aff else 0.0
                        all_constraints.append({
                            'agent': a.id, 'loc': aff_id,
                            'timestep': (max(0.0, t_claim_start - aff_cost), t_end),
                        })

            # Unclaimed 구간 (claim_idx ~): 시간 기반 constraint
            unclaimed_path = a.raw_path[a.claim_idx:]
            if unclaimed_path:
                cs = self.agv_planner._build_constraints(unclaimed_path, a.id)
                all_constraints.extend(cs)

        # 2) Current positions of done agents
        positions_to_plan = {}
        for a in self.agv_agents:
            if a.id in done_ids:
                last_sid = a.raw_path[-1][0]
                nid = last_sid.split(',')[1]
                positions_to_plan[a.id] = nid

        # 3) Assign new port goals (avoid other AGVs' destinations + positions)
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

        # 4) Plan only the done agents (active paths are constraints)
        result = self.agv_planner.plan(
            positions_to_plan, new_goals,
            existing_constraints=all_constraints,
            start_times={aid: sim_time for aid in positions_to_plan},
        )

        if result is None or not result.paths:
            self._plan_status = 'Replan FAILED'
            self._agv_pending_replan.clear()
            return

        # 5) Update goals & extend TAPG — batch all new paths at once
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

        n_replanned = len(result.paths)
        self._plan_status = f'Replanned {n_replanned} AGV @ t={sim_time:.0f}s'
        self._agv_pending_replan.clear()
        self._save_plan_snapshot(f'replan_{n_replanned}agv')

        # Cycle detection
        import networkx as nx
        try:
            nx.find_cycle(self.agv_env.G)
            cycles = list(nx.simple_cycles(self.agv_env.G))
            print(f'\n[CYCLE DETECTED] t={sim_time:.2f}s after replan, {len(cycles)} cycles')
            # Find minimal cycle
            min_cyc = min(cycles, key=len)
            print(f'  Minimal cycle ({len(min_cyc)} nodes):')
            for n in min_cyc:
                print(f'    A{n[1]-100}: {n[0]} t={n[2]:.2f}')
            self._save_plan_snapshot(f'CYCLE_DETECTED')
        except nx.NetworkXNoCycle:
            pass

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

        # OHT controls
        half = (sw-G)//2
        self.btn_oht_del = Button((sx, y, 30, H-4), '-', base=(80,40,40))
        self.btn_oht_add = Button((sx+sw-30, y, 30, H-4), '+')
        self._oht_count_rect = pygame.Rect(sx+32, y, sw-64, H-4)
        y += H-4+G

        # AGV controls
        self.btn_agv_del = Button((sx, y, 30, H-4), '-', base=(80,40,40))
        self.btn_agv_add = Button((sx+sw-30, y, 30, H-4), '+')
        self._agv_count_rect = pygame.Rect(sx+32, y, sw-64, H-4)
        y += H-4+G

        self.btn_shuffle = Button((sx, y, sw, H-4), 'S: Shuffle All'); y += H-4+G

        self._info_y = y + 4
        self.all_btns = [self.btn_start, self.btn_reset, *self.btns_spd,
                         self.btn_oht_add, self.btn_oht_del,
                         self.btn_agv_add, self.btn_agv_del,
                         self.btn_shuffle]

    # ── Update ───────────────────────────────────────────────────────────────

    def update(self, dt_real: float):
        if not self.running:
            return
        dt_sim = dt_real * SIM_SPEEDS[self.spd_idx]
        self.sim_time += dt_sim

        # step all DES engines
        self.oht_env.step(self.sim_time)
        self.agv_env.step(self.sim_time)
        # Step all 3DS TAPG environments + elevators
        for fid, fd in self.s3d_floor_data.items():
            fd['env'].step(self.sim_time)
        self._step_elevators(self.sim_time)

        # reassign DONE OHT agents
        for a in self.oht_agents:
            if a.state == DONE:
                path = self.oht_map.bfs_path(a.cur_node)
                if len(path) > 1:
                    self.oht_env.reassign(a, path, self.sim_time)

        # 3DS: replan DONE shuttles per floor
        for fid, fd in self.s3d_floor_data.items():
            done = [a for a in fd['agents'] if a.state == AGV_DONE
                    and a.id not in fd['pending_replan']]
            if done:
                for a in done:
                    fd['pending_replan'].add(a.id)
                self._replan_3ds_floor(fid, self.sim_time)

        # AGV collision detection
        self._check_agv_collisions()

        # Incremental AGV replanning: when ANY AGV finishes → replan that one
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
                elif k == pygame.K_d:
                    self._dump_agv_status()
                elif k == pygame.K_o:
                    self._add_oht()
                elif k == pygame.K_l:
                    self._del_oht()
                elif k == pygame.K_n:
                    self._add_agv()
                elif k == pygame.K_p:
                    self._del_agv()
                elif k in (pygame.K_EQUALS, pygame.K_PLUS):
                    self._set_spd(min(self.spd_idx+1, len(SIM_SPEEDS)-1))
                elif k == pygame.K_MINUS:
                    self._set_spd(max(self.spd_idx-1, 0))
                elif k == pygame.K_k:
                    self.cam = Camera(self.bg_bbox, ww-SIDE_W, wh)
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.button == 1:
                    pos = ev.pos
                    if self.btn_start.clicked(pos):
                        self.running = not self.running
                    elif self.btn_reset.clicked(pos):
                        self._reset()
                    elif self.btn_shuffle.clicked(pos):
                        self._shuffle()
                    elif self.btn_oht_add.clicked(pos):
                        self._add_oht()
                    elif self.btn_oht_del.clicked(pos):
                        self._del_oht()
                    elif self.btn_agv_add.clicked(pos):
                        self._add_agv()
                    elif self.btn_agv_del.clicked(pos):
                        self._del_agv()
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

        # reset OHT
        self._oht_next_id = 0
        self.oht_agents = []
        for seg in self.oht_map.segments.values():
            seg.queue.clear()
        self.oht_env = OHTEnvironmentDES(self.oht_map, cross_segment=True)
        self._init_oht_agents()

        # reset AGV
        self._agv_next_id = 100
        self.agv_agents = []
        self._agv_start_positions = {}
        self.agv_env = TAPGEnvironment(self.amr_graph, accel=500.0, decel=500.0)
        self._agv_goals = {}
        self._agv_pending_replan = set()
        self._plan_status = ''
        self._init_agv_agents()

        # reset 3DS (SIPP+TAPG per floor) + elevators
        self.s3d_agents = []
        self._init_3ds()  # _init_elevators() 포함

    def _shuffle(self):
        # OHT
        nodes = list(self.oht_map.nodes.keys())
        random.shuffle(nodes)
        excluded = set()
        for a in self.oht_agents:
            for start in nodes:
                if start not in excluded:
                    path = self.oht_map.bfs_path(start)
                    if len(path) > 1:
                        excluded |= self.oht_map.nearby_nodes(start, self.oht_map.h_min)
                        self.oht_env.reassign(a, path, self.sim_time)
                        break
        # AGV — replan with new port destinations
        self._plan_agv_paths(self.sim_time)
        # 3DS — reinit (re-plan all floors)
        self.s3d_agents = []
        self._init_3ds()

    def _add_oht(self):
        if len(self.oht_agents) >= MAX_AGENTS:
            return
        excluded = set()
        for a in self.oht_agents:
            excluded |= self.oht_map.nearby_nodes(a.cur_node, self.oht_map.h_min)
        a = self._make_oht_agent(self._oht_next_id, excluded)
        if a:
            self._oht_next_id += 1
            self.oht_agents.append(a)
            self.oht_env.add_agent(a, t_start=self.sim_time)
        self._n_oht = len(self.oht_agents)

    def _del_oht(self):
        if not self.oht_agents:
            return
        a = self.oht_agents.pop()
        self.oht_env.remove_agent(a.id)
        self._n_oht = len(self.oht_agents)

    def _add_agv(self):
        if len(self.agv_agents) >= MAX_AGENTS:
            return
        # Record current positions for replanning
        occupied = set()
        for a in self.agv_agents:
            nid = a.raw_path[-1][0].split(',')[1] if a.state == AGV_DONE else \
                  a.raw_path[a.path_idx][0].split(',')[1]
            occupied.add(nid)
        nodes = list(self.amr_graph.nodes.keys())
        random.shuffle(nodes)
        for start in nodes:
            if start not in occupied:
                aid = self._agv_next_id
                self._agv_start_positions[aid] = start
                self._agv_next_id += 1
                break
        self._n_agv += 1
        self._plan_agv_paths(self.sim_time)

    def _del_agv(self):
        if not self.agv_agents:
            return
        a = self.agv_agents.pop()
        self.agv_env.remove_agent(a.id)
        self._agv_goals.pop(a.id, None)
        self._n_agv = len(self.agv_agents)

    def _set_spd(self, idx: int):
        self.spd_idx = idx
        for i, b in enumerate(self.btns_spd):
            b.active = (i == idx)

    # ── Render ───────────────────────────────────────────────────────────────

    def render(self):
        ww, wh = self.screen.get_size()
        self.screen.fill(BG)
        pygame.draw.rect(self.screen, (24, 26, 36),
                         pygame.Rect(MAP_W, 0, SIDE_W, wh))

        self._draw_background(ww, wh)
        self._draw_oht_network(ww, wh)
        self._draw_agv_network(ww, wh)
        self._draw_agents(ww, wh)
        self._draw_sidebar(wh)
        pygame.display.flip()

    def _draw_background(self, ww, wh):
        """Draw 3DS floor segments/nodes as dim background."""
        seg_w  = max(1, int(self.cam.scale * 0.0004))
        node_r = max(2, int(self.cam.scale * 0.002))

        for seg in self.bg_segments:
            n1 = self.bg_nodes.get(seg['startNodeId'])
            n2 = self.bg_nodes.get(seg['endNodeId'])
            if not n1 or not n2:
                continue
            a1 = n1.get('area', '')
            a2 = n2.get('area', '')
            # skip OHT_A and AMR_A — drawn separately with detail
            if a1 in ('OHT_A', 'AMR_A') or a2 in ('OHT_A', 'AMR_A'):
                continue
            p1 = self.cam.to_screen(n1['x'], n1['y'])
            p2 = self.cam.to_screen(n2['x'], n2['y'])
            if max(p1[0], p2[0]) < 0 or min(p1[0], p2[0]) > ww:
                continue
            if max(p1[1], p2[1]) < 0 or min(p1[1], p2[1]) > wh:
                continue
            col = AREA_SEG_COLORS.get(a1, COL_SEG)
            pygame.draw.line(self.screen, col, p1, p2, seg_w)

        for nid, node in self.bg_nodes.items():
            area = node.get('area', '')
            if area in ('OHT_A', 'AMR_A'):
                continue
            sx, sy = self.cam.to_screen(node['x'], node['y'])
            if sx < -10 or sx > ww+10 or sy < -10 or sy > wh+10:
                continue
            col = AREA_COLORS.get(area, COL_NODE)
            pygame.draw.circle(self.screen, col, (sx, sy), node_r)

        # Area labels
        area_centers = {}
        for nid, node in self.bg_nodes.items():
            area = node.get('area', '')
            if not area:
                continue
            area_centers.setdefault(area, {'xs': [], 'ys': []})
            area_centers[area]['xs'].append(node['x'])
            area_centers[area]['ys'].append(node['y'])
        for area, coords in area_centers.items():
            cx = sum(coords['xs']) / len(coords['xs'])
            cy = max(coords['ys']) + 500
            lx, ly = self.cam.to_screen(cx, cy)
            col = AREA_SEG_COLORS.get(area, COL_DIM)
            lbl = self.font_b.render(area, True, col)
            self.screen.blit(lbl, lbl.get_rect(center=(lx, ly)))

        # ── Elevator shafts: 층간 gate 노드를 연결하는 선 ──
        self._draw_elevator_shafts(ww, wh)

    def _draw_elevator_shafts(self, ww, wh):
        """Draw elevator shafts connecting 3DS floor gate nodes."""
        import json as _json
        if not hasattr(self, '_lift_gate_cache'):
            # 캐시: lift별 [(floor_id, gate_node_id)] — offset 적용된 좌표
            with open(JSON_FILE, 'r', encoding='utf-8') as f:
                jdata = _json.load(f)
            self._lift_gate_cache = []
            for lift in jdata.get('lifts', []):
                shaft = []
                for fl in lift.get('floors', []):
                    floor_id = '3DS_F' + fl['id']
                    for g in fl.get('gates', []):
                        for nid in g.get('entryNodes', []):
                            node = self.bg_nodes.get(nid)
                            if node:
                                shaft.append((floor_id, nid,
                                              node['x'], node['y']))
                if shaft:
                    self._lift_gate_cache.append({
                        'id': lift['id'], 'floors': shaft})

        shaft_w = max(2, int(self.cam.scale * 0.001))
        gate_r  = max(4, int(self.cam.scale * 0.006))
        col_shaft = (180, 100, 100)
        col_gate  = (255, 100, 100)

        for lift in self._lift_gate_cache:
            floors = lift['floors']
            # 층간 샤프트 선
            screen_pts = []
            for fid, nid, x, y in floors:
                sp = self.cam.to_screen(x, y)
                screen_pts.append(sp)

            if len(screen_pts) >= 2:
                # 점선 스타일 샤프트
                for i in range(len(screen_pts) - 1):
                    p1, p2 = screen_pts[i], screen_pts[i + 1]
                    pygame.draw.line(self.screen, col_shaft, p1, p2, shaft_w)

                # 샤프트 양 옆에 레일 (입체감)
                for i in range(len(screen_pts) - 1):
                    p1, p2 = screen_pts[i], screen_pts[i + 1]
                    off = max(3, int(self.cam.scale * 0.004))
                    pygame.draw.line(self.screen, (col_shaft[0]//2, col_shaft[1]//2, col_shaft[2]//2),
                                     (p1[0]-off, p1[1]), (p2[0]-off, p2[1]), 1)
                    pygame.draw.line(self.screen, (col_shaft[0]//2, col_shaft[1]//2, col_shaft[2]//2),
                                     (p1[0]+off, p1[1]), (p2[0]+off, p2[1]), 1)

            # gate 노드 마커
            for fid, nid, x, y in floors:
                sp = self.cam.to_screen(x, y)
                pygame.draw.circle(self.screen, col_gate, sp, gate_r)
                pygame.draw.circle(self.screen, COL_WHITE, sp, gate_r, 1)

            # Lift 현재 위치 표시 (MOVING 시 보간)
            lift_id = lift['id']
            if hasattr(self, 'lift_ctrl'):
                elev = self.lift_ctrl.get_lift(lift_id)
                if elev and len(screen_pts) >= 2:
                    floor_idx = {'1': 0, '2': 1, '3': 2}
                    box_r = max(6, int(self.cam.scale * 0.008))

                    if elev.state == LIFT_MOVING and elev.move_from_floor and elev.move_to_floor:
                        # 보간: 출발층↔도착층 사이
                        fi = floor_idx.get(elev.move_from_floor, 0)
                        ti = floor_idx.get(elev.move_to_floor, 0)
                        if fi < len(screen_pts) and ti < len(screen_pts):
                            prog = elev.move_progress(self.sim_time)
                            fx, fy = screen_pts[fi]
                            tx, ty = screen_pts[ti]
                            ex = int(fx + (tx - fx) * prog)
                            ey = int(fy + (ty - fy) * prog)
                        else:
                            ci = floor_idx.get(elev.cur_floor, 0)
                            ex, ey = screen_pts[min(ci, len(screen_pts)-1)]
                        box_col = (255, 200, 60)
                    else:
                        ci = floor_idx.get(elev.cur_floor, 0)
                        ex, ey = screen_pts[min(ci, len(screen_pts)-1)]
                        if elev.state in (LIFT_LOADING, LIFT_UNLOADING):
                            box_col = (60, 200, 255)
                        else:
                            box_col = (180, 180, 180)

                    # 엘리베이터 박스
                    pygame.draw.rect(
                        self.screen, box_col,
                        (ex - box_r, ey - box_r, box_r*2, box_r*2),
                        border_radius=3)
                    pygame.draw.rect(
                        self.screen, COL_WHITE,
                        (ex - box_r, ey - box_r, box_r*2, box_r*2),
                        1, border_radius=3)

                    # 화물 표시
                    if elev.cargo_count > 0:
                        pygame.draw.circle(
                            self.screen, (255, 100, 100),
                            (ex, ey), max(3, box_r // 2))

                    # 라벨
                    state_short = elev.state[:4]
                    lbl = self.font_s.render(
                        f'{lift_id} F{elev.cur_floor} {state_short}',
                        True, box_col)
                    self.screen.blit(lbl, (ex + box_r + 3, ey - 6))

    def _draw_oht_network(self, ww, wh):
        """Draw OHT sub-network with polyline segments and ZCU nodes."""
        omap = self.oht_map
        seg_col, arr_col = (85, 50, 100), (110, 70, 140)
        seg_w  = max(1, int(self.cam.scale * 0.0006))
        h_size = max(4, int(self.cam.scale * 0.002))
        node_r = max(3, int(self.cam.scale * 0.004))

        for seg in omap.segments.values():
            pts = seg.path_points
            if not pts:
                continue
            spts = [self.cam.to_screen(x, y) for x, y in pts]
            xs = [p[0] for p in spts]; ys = [p[1] for p in spts]
            if min(xs) > ww or max(xs) < 0 or min(ys) > wh or max(ys) < 0:
                continue
            if len(spts) >= 2:
                pygame.draw.lines(self.screen, seg_col, False, spts, seg_w)
            mid_idx = max(1, min(int(len(spts) * 0.6), len(spts) - 1))
            draw_arrow(self.screen, arr_col,
                       spts[mid_idx - 1], spts[mid_idx],
                       width=seg_w, head=h_size)

        for nid, node in omap.nodes.items():
            sx, sy = self.cam.to_screen(node.x, node.y)
            if sx < -10 or sx > ww+10 or sy < -10 or sy > wh+10:
                continue
            if nid in omap.zcu_node_ids:
                zone_held = any(
                    z for z in omap.zcu_zones
                    if nid in {t for _, t in z.entry_segs | z.exit_segs}
                    and self.oht_env._zcu_holders.get(z.id) is not None
                )
                zcu_col = COL_ZCU_HELD if zone_held else COL_ZCU_FREE
                r = node_r + 2
                diamond = [(sx, sy-r), (sx+r, sy), (sx, sy+r), (sx-r, sy)]
                pygame.draw.polygon(self.screen, zcu_col, diamond)
                pygame.draw.polygon(self.screen, COL_WHITE, diamond, 1)
            else:
                col = COL_PORT if nid in omap.port_nodes else COL_NODE
                pygame.draw.circle(self.screen, col, (sx, sy), node_r)
                pygame.draw.circle(self.screen, COL_WHITE, (sx, sy), node_r, 1)

    def _draw_agv_network(self, ww, wh):
        """Draw AGV (AMR_A) sub-network with edges and nodes."""
        graph = self.amr_graph
        seg_col, arr_col = (90, 90, 35), (120, 120, 50)
        seg_w  = max(1, int(self.cam.scale * 0.0006))
        h_size = max(4, int(self.cam.scale * 0.002))
        node_r = max(3, int(self.cam.scale * 0.004))

        for (fn, tn), edge in graph.edges.items():
            n1, n2 = graph.nodes[fn], graph.nodes[tn]
            p1 = self.cam.to_screen(n1.x, n1.y)
            p2 = self.cam.to_screen(n2.x, n2.y)
            if max(p1[0], p2[0]) < 0 or min(p1[0], p2[0]) > ww:
                continue
            if max(p1[1], p2[1]) < 0 or min(p1[1], p2[1]) > wh:
                continue
            draw_arrow(self.screen, arr_col, p1, p2,
                       width=seg_w, head=h_size)

        port_nids = set(graph.ports.values()) if graph.ports else set()
        port_r = max(5, int(self.cam.scale * 0.008))
        for nid, node in graph.nodes.items():
            sx, sy = self.cam.to_screen(node.x, node.y)
            if sx < -10 or sx > ww+10 or sy < -10 or sy > wh+10:
                continue
            if nid in port_nids:
                # Port: larger, brighter, square shape with label
                r = port_r
                pygame.draw.rect(self.screen, COL_PORT,
                                 (sx - r, sy - r, r*2, r*2), border_radius=2)
                pygame.draw.rect(self.screen, COL_WHITE,
                                 (sx - r, sy - r, r*2, r*2), 1, border_radius=2)
                # Label
                lbl = self.font_s.render(nid.replace('na.', ''), True, COL_PORT)
                self.screen.blit(lbl, (sx + r + 2, sy - 6))
            else:
                pygame.draw.circle(self.screen, COL_NODE, (sx, sy), node_r)
                pygame.draw.circle(self.screen, COL_WHITE, (sx, sy), node_r, 1)

    def _draw_agents(self, ww, wh):
        """Draw all OHT + AGV + 3DS agents."""
        # Goal lines (dashed) — 먼저 그려서 차량 아래에
        self._draw_goal_lines(ww, wh)

        # 3DS shuttles (draw first — background layer)
        self._draw_3ds_agents(ww, wh)

        # OHT vehicles (longer, narrower — overhead rail)
        oht_len = max(self.oht_map.vehicle_length * self.cam.scale, 10.0)
        oht_wid = max(self.oht_map.vehicle_width  * self.cam.scale,  5.0)
        self._draw_oht_agents(oht_len, oht_wid, ww, wh)

        # AGV vehicles (squarish — ground robot)
        agv_len = max(self.amr_graph.vehicle_length * self.cam.scale, 8.0)
        agv_wid = max(self.amr_graph.vehicle_width  * self.cam.scale, 8.0)
        self._draw_agv_agents(agv_len, agv_wid, ww, wh)

    def _draw_goal_lines(self, ww, wh):
        """Draw dashed lines from each vehicle to its goal + diamond marker."""
        r = max(5, int(self.cam.scale * 0.006))
        dim = 0.4   # 점선 밝기 비율

        # ── OHT: 경로 끝 노드가 목적지
        for a in self.oht_agents:
            if a.state == DONE or len(a.node_path) < 2:
                continue
            goal_nid = a.node_path[-1]
            gn = self.oht_map.nodes.get(goal_nid)
            if not gn:
                continue
            ax, ay = self.cam.to_screen(a.x, a.y)
            gx, gy = self.cam.to_screen(gn.x, gn.y)
            col = tuple(max(0, int(c * dim)) for c in a.color)
            draw_dashed_line(self.screen, col, (ax, ay), (gx, gy), 1, 6, 4)
            diamond = [(gx, gy-r), (gx+r, gy), (gx, gy+r), (gx-r, gy)]
            pygame.draw.polygon(self.screen, a.color, diamond, 2)

        # ── AGV
        for a in self.agv_agents:
            goal = self._agv_goals.get(a.id)
            if not goal or a.state == AGV_DONE:
                continue
            node = self.amr_graph.nodes.get(goal)
            if not node:
                continue
            ax, ay = self.cam.to_screen(a.x, a.y)
            gx, gy = self.cam.to_screen(node.x, node.y)
            col = tuple(max(0, int(c * dim)) for c in a.color)
            draw_dashed_line(self.screen, col, (ax, ay), (gx, gy), 1, 6, 4)
            diamond = [(gx, gy-r), (gx+r, gy), (gx, gy+r), (gx-r, gy)]
            pygame.draw.polygon(self.screen, a.color, diamond, 2)

        # ── 3DS
        for fid, fd in self.s3d_floor_data.items():
            graph = fd['graph']
            for a in fd['agents']:
                goal = fd['goals'].get(a.id)
                if not goal or a.state == AGV_DONE:
                    continue
                node = graph.nodes.get(goal)
                if not node:
                    continue
                ax, ay = self.cam.to_screen(a.x, a.y)
                gx, gy = self.cam.to_screen(node.x, node.y)
                col = tuple(max(0, int(c * dim)) for c in a.color)
                draw_dashed_line(self.screen, col, (ax, ay), (gx, gy), 1, 6, 4)
                diamond = [(gx, gy-r), (gx+r, gy), (gx, gy+r), (gx-r, gy)]
                pygame.draw.polygon(self.screen, a.color, diamond, 2)

    def _draw_oht_agents(self, v_len, v_wid, ww, wh):
        """Draw OHT agents (MOVING/FOLLOWING/BLOCKED/IDLE/DONE)."""
        for a in self.oht_agents:
            sx, sy = self.cam.to_screen(a.x, a.y)
            if sx < -50 or sx > ww+50 or sy < -50 or sy > wh+50:
                continue
            scr_ang = math.degrees(-a.theta)

            if a.state == DONE:
                fill, border = tuple(c//4 for c in a.color), COL_DIM
            elif a.state == BLOCKED:
                fill, border = COL_BLOCKED, COL_WHITE
            elif a.state == FOLLOWING:
                fill, border = COL_FOLLOWING, COL_WHITE
            elif a.state == MOVING:
                vf = min(a.v / 1500.0, 1.0)
                fill = tuple(min(c + int(40*vf), 255) for c in a.color)
                border = COL_WHITE
            else:
                fill, border = tuple(max(c-50, 0) for c in a.color), COL_DIM

            draw_rotated_rect(self.screen, fill, sx, sy,
                              v_len, v_wid, scr_ang, border=border, border_w=2)

            # headlight
            rad = math.radians(scr_ang)
            hx = sx + math.cos(rad) * v_len / 2
            hy = sy + math.sin(rad) * v_len / 2
            pygame.draw.circle(self.screen, COL_HEADLIGHT,
                               (int(hx), int(hy)), max(2, int(v_wid * 0.2)))

            if a.state == BLOCKED:
                pygame.draw.circle(self.screen, COL_BLOCKED,
                                   (sx, sy), int(v_len/2)+4, 2)
            elif a.state == FOLLOWING:
                pygame.draw.circle(self.screen, COL_FOLLOWING,
                                   (sx, sy), int(v_len/2)+4, 2)

            lbl = self.font_s.render(f'H{a.id}', True, COL_WHITE)
            self.screen.blit(lbl, lbl.get_rect(center=(sx, sy)))

    def _draw_agv_agents(self, v_len, v_wid, ww, wh):
        """Draw AGV agents (TAPG states: idle/moving/rotating/waiting/done)."""
        COL_AGV_WAITING  = (255, 165, 0)   # orange — TAPG dependency wait
        COL_AGV_ROTATING = (255, 220, 60)  # yellow — in-place rotation
        for a in self.agv_agents:
            sx, sy = self.cam.to_screen(a.x, a.y)
            if sx < -50 or sx > ww+50 or sy < -50 or sy > wh+50:
                continue
            scr_ang = math.degrees(-a.theta)

            if a.state == AGV_DONE:
                fill, border = tuple(c//4 for c in a.color), COL_DIM
            elif a.state == AGV_WAITING:
                fill, border = COL_AGV_WAITING, COL_WHITE
            elif a.state == AGV_ROTATING:
                fill, border = COL_AGV_ROTATING, COL_WHITE
            elif a.state == AGV_MOVING:
                vf = min(a.v / 1500.0, 1.0)
                fill = tuple(min(c + int(40*vf), 255) for c in a.color)
                border = COL_WHITE
            else:  # idle
                fill, border = tuple(max(c-50, 0) for c in a.color), COL_DIM

            draw_rotated_rect(self.screen, fill, sx, sy,
                              v_len, v_wid, scr_ang, border=border, border_w=2)

            # headlight
            rad = math.radians(scr_ang)
            hx = sx + math.cos(rad) * v_len / 2
            hy = sy + math.sin(rad) * v_len / 2
            pygame.draw.circle(self.screen, COL_HEADLIGHT,
                               (int(hx), int(hy)), max(2, int(v_wid * 0.2)))

            if a.state == AGV_WAITING:
                pygame.draw.circle(self.screen, COL_AGV_WAITING,
                                   (sx, sy), int(v_len/2)+4, 2)
            elif a.state == AGV_ROTATING:
                pygame.draw.circle(self.screen, COL_AGV_ROTATING,
                                   (sx, sy), int(v_len/2)+4, 2)

            display_id = a.id - 100
            lbl = self.font_s.render(f'A{display_id}', True, COL_WHITE)
            self.screen.blit(lbl, lbl.get_rect(center=(sx, sy)))

    def _draw_3ds_agents(self, ww, wh):
        """Draw 3DS shuttle agents (TAPG-based, same rendering as AGV)."""
        v_size = max(16, int(self.cam.scale * 850 * 0.001 * _3DS_SCALE * 4))  # 850mm × scale × 4
        COL_S3D_WAITING  = (255, 165, 0)
        COL_S3D_ROTATING = (255, 220, 60)
        for a in self.s3d_agents:
            sx, sy = self.cam.to_screen(a.x, a.y)
            if sx < -50 or sx > ww+50 or sy < -50 or sy > wh+50:
                continue
            scr_ang = math.degrees(-a.theta)

            if a.state == AGV_DONE:
                fill = tuple(c // 4 for c in a.color)
                border = COL_DIM
            elif a.state == AGV_WAITING:
                fill, border = COL_S3D_WAITING, COL_WHITE
            elif a.state == AGV_ROTATING:
                fill, border = COL_S3D_ROTATING, COL_WHITE
            elif a.state == AGV_MOVING:
                vf = min(a.v / 1500.0, 1.0)
                fill = tuple(min(c + int(40*vf), 255) for c in a.color)
                border = COL_WHITE
            else:  # idle
                fill = tuple(max(c - 50, 0) for c in a.color)
                border = COL_DIM

            draw_rotated_rect(self.screen, fill, sx, sy,
                              v_size, v_size, scr_ang,
                              border=border, border_w=2)

            # headlight
            rad = math.radians(scr_ang)
            hx = sx + math.cos(rad) * v_size / 2
            hy = sy + math.sin(rad) * v_size / 2
            pygame.draw.circle(self.screen, COL_HEADLIGHT,
                               (int(hx), int(hy)), max(2, int(v_size * 0.15)))

            if a.state == AGV_WAITING:
                pygame.draw.circle(self.screen, COL_S3D_WAITING,
                                   (sx, sy), int(v_size/2)+4, 2)

            display_id = a.id - 200
            lbl = self.font_s.render(f'S{display_id}', True, COL_WHITE)
            self.screen.blit(lbl, lbl.get_rect(center=(sx, sy)))

    def _draw_sidebar(self, wh):
        for b in self.all_btns:
            b.draw(self.screen, self.font_s)

        sx = MAP_W + 10
        sw = SIDE_W - 20

        # OHT count display
        r = self._oht_count_rect
        pygame.draw.rect(self.screen, (40, 25, 50), r, border_radius=3)
        lbl = self.font_s.render(f'OHT: {len(self.oht_agents)}', True,
                                 OHT_COLORS[0])
        self.screen.blit(lbl, lbl.get_rect(center=r.center))

        # AGV count display
        r = self._agv_count_rect
        pygame.draw.rect(self.screen, (40, 40, 20), r, border_radius=3)
        lbl = self.font_s.render(f'AGV: {len(self.agv_agents)}', True,
                                 AGV_COLORS[0])
        self.screen.blit(lbl, lbl.get_rect(center=r.center))

        y = self._info_y

        def line(txt, col=COL_TEXT, f=None):
            nonlocal y
            fnt = f or self.font_m
            self.screen.blit(fnt.render(txt, True, col), (sx, y))
            y += fnt.get_linesize() + 2

        line('── Simulation ──', f=self.font_b)
        line(f'Time  : {self.sim_time:8.2f} s')
        line(f'Speed : {SIM_SPEED_LABELS[self.spd_idx]}')
        state = '▶ Running' if self.running else '|| Paused'
        line(state, col=(100,220,100) if self.running else (160,160,160))
        y += 6

        # OHT stats
        line('── OHT ──', f=self.font_b, col=OHT_COLORS[0])
        n_mov = sum(1 for a in self.oht_agents if a.state == MOVING)
        n_fol = sum(1 for a in self.oht_agents if a.state == FOLLOWING)
        n_blk = sum(1 for a in self.oht_agents if a.state == BLOCKED)
        line(f'  Mov:{n_mov} Fol:{n_fol} Blk:{n_blk}', f=self.font_s)

        for a in self.oht_agents:
            state_col = {
                MOVING: COL_TEXT, FOLLOWING: COL_FOLLOWING,
                BLOCKED: COL_BLOCKED, IDLE: COL_DIM, DONE: COL_DIM,
            }.get(a.state, COL_TEXT)
            pygame.draw.rect(self.screen, a.color,
                             pygame.Rect(sx, y+2, 10, 10), border_radius=2)
            self.screen.blit(
                self.font_s.render(
                    f' H{a.id:02d} {a.state:<9s} v={a.v/1000:.2f}',
                    True, state_col),
                (sx+12, y))
            y += 14
        y += 4

        # AGV stats
        COL_AGV_WAITING_S = (255, 165, 0)
        line('── AGV (SIPP+TAPG) ──', f=self.font_b, col=AGV_COLORS[0])
        if self._plan_status:
            line(f'  {self._plan_status}', f=self.font_s, col=COL_DIM)
        n_mov = sum(1 for a in self.agv_agents if a.state == AGV_MOVING)
        n_rot = sum(1 for a in self.agv_agents if a.state == AGV_ROTATING)
        n_wai = sum(1 for a in self.agv_agents if a.state == AGV_WAITING)
        n_idl = sum(1 for a in self.agv_agents if a.state == AGV_IDLE)
        line(f'  Mov:{n_mov} Rot:{n_rot} Wait:{n_wai} Idle:{n_idl}', f=self.font_s)

        for a in self.agv_agents:
            state_col = {
                AGV_MOVING:   COL_TEXT,
                AGV_ROTATING: (255, 220, 60),
                AGV_WAITING:  COL_AGV_WAITING_S,
                AGV_IDLE:     COL_DIM,
                AGV_DONE:     COL_DIM,
            }.get(a.state, COL_TEXT)
            pygame.draw.rect(self.screen, a.color,
                             pygame.Rect(sx, y+2, 10, 10), border_radius=2)
            display_id = a.id - 100
            goal = self._agv_goals.get(a.id, '?')
            goal_short = goal.replace('na.', '') if isinstance(goal, str) else '?'
            st_short = a.state[:4] if a.state else '?'
            self.screen.blit(
                self.font_s.render(
                    f' A{display_id:02d} {st_short:<5s} v={a.v/1000:.1f} ->{goal_short}',
                    True, state_col),
                (sx+12, y))
            y += 14
        y += 6

        # 3DS stats
        line('── 3DS Shuttle ──', f=self.font_b, col=S3D_COLORS['3DS_F1'])
        n_mov_s = sum(1 for a in self.s3d_agents if a.state == AGV_MOVING)
        n_wai_s = sum(1 for a in self.s3d_agents if a.state == AGV_WAITING)
        line(f'  Mov:{n_mov_s} Wait:{n_wai_s}', f=self.font_s)
        for fid in S3D_FLOOR_IDS:
            fd = self.s3d_floor_data.get(fid)
            if not fd:
                continue
            for a in fd['agents']:
                state_col = {
                    AGV_MOVING: COL_TEXT, AGV_WAITING: (255, 165, 0),
                    AGV_ROTATING: (255, 220, 60),
                    AGV_IDLE: COL_DIM, AGV_DONE: COL_DIM,
                }.get(a.state, COL_TEXT)
                pygame.draw.rect(self.screen, a.color,
                                 pygame.Rect(sx, y+2, 10, 10), border_radius=2)
                display_id = a.id - 200
                floor_short = fid.replace('3DS_', '')
                goal = fd['goals'].get(a.id, '?')
                if isinstance(goal, str):
                    goal = goal[-4:]
                st_short = a.state[:4] if a.state else '?'
                self.screen.blit(
                    self.font_s.render(
                        f' S{display_id} {floor_short} {st_short:<5s} v={a.v/1000:.1f}',
                        True, state_col),
                    (sx+12, y))
                y += 14
        y += 6

        # Elevators
        if hasattr(self, 'lift_ctrl') and self.lift_ctrl.lifts:
            COL_LIFT_MOV  = (255, 200, 60)
            COL_LIFT_XFER = (60, 200, 255)
            COL_LIFT_IDLE = (180, 180, 180)
            line('── Elevator ──', f=self.font_b, col=COL_LIFT_XFER)
            for lid in sorted(self.lift_ctrl.lifts):
                elev = self.lift_ctrl.get_lift(lid)
                if elev.state == LIFT_MOVING:
                    scol = COL_LIFT_MOV
                elif elev.state in (LIFT_LOADING, LIFT_UNLOADING):
                    scol = COL_LIFT_XFER
                else:
                    scol = COL_LIFT_IDLE
                cargo = ' [C]' if elev.cargo_count > 0 else ''
                queue = f' q={elev.queue_length}' if elev.queue_length > 0 else ''
                line(f'  {lid} F{elev.cur_floor} {elev.state:<9s}{cargo}{queue}',
                     f=self.font_s, col=scol)
            y += 6

        # ZCU
        if self.oht_map.zcu_zones:
            line('── ZCU ──', f=self.font_b)
            for z in self.oht_map.zcu_zones:
                holder = self.oht_env._zcu_holders.get(z.id)
                n_wait = len(self.oht_env._zcu_waitlists.get(z.id, []))
                name = z.id.replace('ZCU_', '')
                if holder is not None:
                    txt = f'{name}: H{holder.id:02d}'
                    if n_wait:
                        txt += f' ({n_wait}w)'
                    line(txt, col=COL_ZCU_HELD, f=self.font_s)
                else:
                    line(f'{name}: free', col=COL_ZCU_FREE, f=self.font_s)
            y += 4

        # Map info
        line('── Map ──', f=self.font_b)
        line(f'OHT nodes: {len(self.oht_map.nodes)}  segs: {len(self.oht_map.segments)}',
             col=COL_DIM, f=self.font_s)
        line(f'AGV: {len(self.amr_graph.nodes)}n {len(self.amr_graph.edges)}e '
             f'{len(self.amr_graph.move_states_raw)}s',
             col=COL_DIM, f=self.font_s)
        y += 4

        line('── Keys ──', f=self.font_b)
        for hint in ['SPACE - start/pause', 'R - reset', 'S - shuffle',
                     '+/- - sim speed', 'O/L - OHT +/-', 'N/P - AGV +/-',
                     'drag - pan', 'wheel - zoom', 'Q - quit']:
            line(hint, col=COL_DIM, f=self.font_s)

        # area legend
        y += 6
        line('── Areas ──', f=self.font_b)
        for area, col in [('OHT_A', OHT_COLORS[0]), ('AMR_A', AGV_COLORS[0]),
                          ('3DS_F1', AREA_COLORS['3DS_F1']),
                          ('3DS_F2', AREA_COLORS['3DS_F2']),
                          ('3DS_F3', AREA_COLORS['3DS_F3'])]:
            pygame.draw.circle(self.screen, col, (sx+6, y+6), 5)
            self.screen.blit(self.font_s.render(area, True, COL_DIM),
                             (sx+15, y))
            y += 16

    # ── Run ──────────────────────────────────────────────────────────────────

    def run(self):
        while True:
            dt = self.clock.tick(FPS) / 1000.0
            self.handle_events()
            self.update(dt)
            self.render()


# ── Entry point ──────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--oht', type=int, default=8, help='Number of OHT vehicles')
    ap.add_argument('--agv', type=int, default=10, help='Number of AGV vehicles')
    ap.add_argument('--s3d', type=int, default=4, help='Number of 3DS shuttles per floor')
    ap.add_argument('--seed', type=int, default=None, help='Random seed for reproducibility')
    args = ap.parse_args()

    # Seed 설정 및 저장
    if args.seed is not None:
        seed = args.seed
    else:
        seed = random.randint(0, 999999)
    random.seed(seed)
    print(f'Random seed: {seed}  (reproduce with --seed {seed})')

    print('Loading maps...')
    oht_map = OHTMap(JSON_FILE, area='OHT_A')
    print(f'  OHT: {len(oht_map.nodes)} nodes, {len(oht_map.segments)} segments')

    amr_graph = PklMapGraph(AMR_PKL)
    print(f'  AGV: {len(amr_graph.nodes)} nodes, {len(amr_graph.edges)} edges')
    print(f'  AGV states: {len(amr_graph.move_states_raw)} move+rot, '
          f'{len(amr_graph.stop_states_raw)} stop')

    # Apply area offsets to OHTMap nodes
    dx_oht, dy_oht = _area_offset('OHT_A')
    for node in oht_map.nodes.values():
        node.x += dx_oht
        node.y += dy_oht
    # Rebuild segment path_points with offset
    for seg in oht_map.segments.values():
        if seg.path_points:
            seg.path_points = [(px + dx_oht, py + dy_oht)
                               for px, py in seg.path_points]

    # Apply area offset to AMR_A (PklMapGraph) nodes
    dx_amr, dy_amr = _area_offset('AMR_A')
    for node in amr_graph.nodes.values():
        node.x += dx_amr
        node.y += dy_amr

    print(f'Starting combined simulator: {args.oht} OHT + {args.agv} AGV + {args.s3d} 3DS/floor')
    sim = CombinedSimulator(oht_map, amr_graph,
                            n_oht=args.oht, n_agv=args.agv, n_s3d=args.s3d)
    sim.run()
