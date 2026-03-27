"""
test_3ds.py — 3DS 셔틀 환경 단독 테스트 (pygame 시각화).

KaistTB_map.json에서 3DS_F1/F2/F3 층 그래프를 로드하고,
각 층 셔틀 1대씩 랜덤 경로를 주행합니다.

Controls
────────
  SPACE : Start / Pause
  R     : 랜덤 경로 재할당
  1/2/3 : 해당 층 셔틀만 랜덤 경로 할당
  +/-   : 속도 조절
  Mouse : 드래그(팬), 휠(줌)
  Q/ESC : 종료
"""
import sys, os, json, math, random
import pygame

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from env_3ds import Env3DS, FloorGraph, FloorNode, FloorEdge, Shuttle, IDLE, MOVING, DONE

# ── 설정 ─────────────────────────────────────────────────────────────────────
JSON_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'KaistTB_map.json')

FLOOR_IDS     = ['3DS_F1', '3DS_F2', '3DS_F3']
FLOOR_COLORS  = {
    '3DS_F1': {'node': (50, 80, 110), 'edge': (40, 60, 85), 'shuttle': (80, 180, 255)},
    '3DS_F2': {'node': (50, 95,  70), 'edge': (40, 75, 55), 'shuttle': (80, 230, 130)},
    '3DS_F3': {'node': (90, 70,  45), 'edge': (70, 55, 35), 'shuttle': (255, 180, 80)},
}
FLOOR_LABELS = {'3DS_F1': 'Floor 1', '3DS_F2': 'Floor 2', '3DS_F3': 'Floor 3'}

BG         = (18, 20, 28)
COL_TEXT   = (200, 210, 220)
COL_DIM    = (80, 90, 100)
COL_PATH   = (255, 255, 100)
COL_GATE   = (255, 100, 100)

WIN_W, WIN_H = 1200, 800
SIDE_W = 250
SIM_SPEEDS = [0.5, 1.0, 2.0, 5.0, 10.0, 20.0]
SIM_LABELS = ['0.5x', '1x', '2x', '5x', '10x', '20x']


# ── 맵 로드 ─────────────────────────────────────────────────────────────────

def load_map():
    with open(JSON_FILE) as f:
        data = json.load(f)

    # 노드 → area 매핑
    node_area = {}
    for nd in data['nodes']:
        node_area[nd['id']] = nd.get('area', '')

    # 층별 그래프 생성
    floors = {}
    for fid in FLOOR_IDS:
        fg = FloorGraph(fid)
        for nd in data['nodes']:
            if nd.get('area') == fid:
                fg.add_node(nd['id'], nd['x'], nd['y'])
        for seg in data['segments']:
            sid = seg.get('startNodeId', '')
            eid = seg.get('endNodeId', '')
            if sid in fg.nodes and eid in fg.nodes:
                spd = seg.get('speed', 1000)
                fg.add_edge(sid, eid, spd)
        floors[fid] = fg

    # lift gate 노드
    gate_nodes = set()
    for lift in data.get('lifts', []):
        for fl in lift.get('floors', []):
            for g in fl.get('gates', []):
                for n in g.get('entryNodes', []) + g.get('exitNodes', []):
                    gate_nodes.add(n)

    return floors, gate_nodes, node_area


# ── 카메라 ───────────────────────────────────────────────────────────────────

class Camera:
    def __init__(self, bbox, w, h):
        x0, y0, x1, y1 = bbox
        cx, cy = (x0+x1)/2, (y0+y1)/2
        rx, ry = (x1-x0) or 1, (y1-y0) or 1
        self.scale = min(w / rx, h / ry) * 0.85
        self.ox = w/2 - cx * self.scale
        self.oy = h/2 + cy * self.scale
        self._drag = False
        self._last = (0, 0)

    def to_screen(self, x, y):
        return (int(x * self.scale + self.ox),
                int(-y * self.scale + self.oy))

    def on_down(self, pos):
        self._drag = True
        self._last = pos

    def on_up(self):
        self._drag = False

    def on_move(self, pos):
        if self._drag:
            dx = pos[0] - self._last[0]
            dy = pos[1] - self._last[1]
            self.ox += dx
            self.oy += dy
            self._last = pos

    def on_scroll(self, pos, up):
        factor = 1.15 if up else 1/1.15
        sx, sy = pos
        self.ox = sx - (sx - self.ox) * factor
        self.oy = sy - (sy - self.oy) * factor
        self.scale *= factor


# ── 메인 ─────────────────────────────────────────────────────────────────────

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
    pygame.display.set_caption('3DS Shuttle Test')
    clock = pygame.time.Clock()
    font_s = pygame.font.SysFont('consolas', 13)
    font_m = pygame.font.SysFont('consolas', 15)
    font_b = pygame.font.SysFont('consolas', 16, bold=True)

    # 맵 로드
    floors, gate_nodes, node_area = load_map()

    # 전체 bbox
    all_nodes = []
    for fg in floors.values():
        for n in fg.nodes.values():
            all_nodes.append((n.x, n.y))
    if not all_nodes:
        print("No 3DS nodes found")
        return
    xs = [p[0] for p in all_nodes]
    ys = [p[1] for p in all_nodes]
    bbox = (min(xs), min(ys), max(xs), max(ys))
    cam = Camera(bbox, WIN_W - SIDE_W, WIN_H)

    # 환경 생성
    env = Env3DS(floors, accel=500.0, decel=500.0)

    # 셔틀 등록 — 각 층 첫 번째 노드에 배치
    shuttle_info = {}
    for i, fid in enumerate(FLOOR_IDS):
        fg = floors[fid]
        start = list(fg.nodes.keys())[0]
        sid = i
        color = FLOOR_COLORS[fid]['shuttle']
        env.add_shuttle(fid, sid, start, color=color, max_speed=1000.0)
        shuttle_info[sid] = {'floor': fid, 'start': start, 'path': []}

    def assign_random(sid):
        s = env.get_shuttle(sid)
        fg = floors[s.floor_id]
        nodes = list(fg.nodes.keys())
        if len(nodes) < 2:
            return
        goal = random.choice([n for n in nodes if n != s.cur_node])
        path = fg.bfs(s.cur_node, goal)
        env.assign(sid, path, sim_time)
        shuttle_info[sid]['path'] = path

    def assign_all():
        for sid in shuttle_info:
            assign_random(sid)

    # 초기 경로 할당
    sim_time = 0.0
    assign_all()

    running = False
    spd_idx = 1

    # ── 메인 루프 ─────────────────────────────────────────────────────────

    while True:
        dt_real = clock.tick(60) / 1000.0
        ww, wh = screen.get_size()

        # 이벤트 처리
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); return
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    pygame.quit(); return
                elif ev.key == pygame.K_SPACE:
                    running = not running
                elif ev.key == pygame.K_r:
                    assign_all()
                elif ev.key == pygame.K_1:
                    assign_random(0)
                elif ev.key == pygame.K_2:
                    assign_random(1)
                elif ev.key == pygame.K_3:
                    assign_random(2)
                elif ev.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    spd_idx = min(spd_idx + 1, len(SIM_SPEEDS) - 1)
                elif ev.key == pygame.K_MINUS:
                    spd_idx = max(spd_idx - 1, 0)
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.button == 1 and ev.pos[0] < ww - SIDE_W:
                    cam.on_down(ev.pos)
                elif ev.button == 4:
                    cam.on_scroll(ev.pos, up=True)
                elif ev.button == 5:
                    cam.on_scroll(ev.pos, up=False)
            elif ev.type == pygame.MOUSEBUTTONUP:
                if ev.button == 1:
                    cam.on_up()
            elif ev.type == pygame.MOUSEMOTION:
                cam.on_move(ev.pos)

        # 시뮬레이션
        if running:
            dt_sim = dt_real * SIM_SPEEDS[spd_idx]
            sim_time += dt_sim
            env.step(sim_time, dt_sim)

            # DONE → 자동 재할당
            for sid in shuttle_info:
                s = env.get_shuttle(sid)
                if s.state == DONE:
                    assign_random(sid)

        # ── 렌더링 ─────────────────────────────────────────────────────────

        screen.fill(BG)

        # 사이드바 배경
        pygame.draw.rect(screen, (24, 26, 36),
                         pygame.Rect(ww - SIDE_W, 0, SIDE_W, wh))

        # 층별 그래프 그리기
        edge_w = max(1, int(cam.scale * 0.0005))
        node_r = max(2, int(cam.scale * 0.003))
        gate_r = max(3, int(cam.scale * 0.005))

        for fid, fg in floors.items():
            ecol = FLOOR_COLORS[fid]['edge']
            ncol = FLOOR_COLORS[fid]['node']

            # 엣지
            for (fi, ti), edge in fg.edges.items():
                n1, n2 = fg.nodes[fi], fg.nodes[ti]
                p1 = cam.to_screen(n1.x, n1.y)
                p2 = cam.to_screen(n2.x, n2.y)
                pygame.draw.line(screen, ecol, p1, p2, edge_w)

            # 노드
            for nid, node in fg.nodes.items():
                sx, sy = cam.to_screen(node.x, node.y)
                if nid in gate_nodes:
                    pygame.draw.circle(screen, COL_GATE, (sx, sy), gate_r)
                else:
                    pygame.draw.circle(screen, ncol, (sx, sy), node_r)

        # 셔틀 경로 + 셔틀 그리기
        shuttle_r = max(5, int(cam.scale * 0.008))

        for sid, info in shuttle_info.items():
            s = env.get_shuttle(sid)
            fg = floors[s.floor_id]

            # 남은 경로 표시
            if s.path_idx < len(s.node_path):
                for i in range(s.path_idx, len(s.node_path) - 1):
                    n1 = fg.nodes.get(s.node_path[i])
                    n2 = fg.nodes.get(s.node_path[i + 1])
                    if n1 and n2:
                        p1 = cam.to_screen(n1.x, n1.y)
                        p2 = cam.to_screen(n2.x, n2.y)
                        pygame.draw.line(screen, COL_PATH, p1, p2,
                                         max(1, edge_w + 1))

            # 셔틀 본체
            sx, sy = cam.to_screen(s.x, s.y)
            pygame.draw.circle(screen, s.color, (sx, sy), shuttle_r)
            pygame.draw.circle(screen, (255, 255, 255), (sx, sy), shuttle_r, 1)

            # 방향 표시
            hx = sx + int(shuttle_r * 1.5 * math.cos(s.theta))
            hy = sy - int(shuttle_r * 1.5 * math.sin(s.theta))
            pygame.draw.line(screen, (255, 255, 255), (sx, sy), (hx, hy), 2)

            # ID 라벨
            lbl = font_s.render(f'S{sid}', True, (255, 255, 255))
            screen.blit(lbl, (sx + shuttle_r + 2, sy - 6))

        # ── 사이드바 ──────────────────────────────────────────────────────

        sx = ww - SIDE_W + 10
        y = 15

        def line(txt, col=COL_TEXT, f=None):
            nonlocal y
            fnt = f or font_m
            screen.blit(fnt.render(txt, True, col), (sx, y))
            y += fnt.get_linesize() + 2

        line('── 3DS Shuttle Test ──', f=font_b)
        y += 4
        line(f'Time : {sim_time:.2f} s')
        line(f'Speed: {SIM_LABELS[spd_idx]}')
        state_txt = '▶ Running' if running else '|| Paused'
        line(state_txt, col=(100, 220, 100) if running else COL_DIM)
        y += 10

        line('── Controls ──', f=font_b, col=COL_DIM)
        line('SPACE: Start/Pause', f=font_s, col=COL_DIM)
        line('R: Randomize all', f=font_s, col=COL_DIM)
        line('1/2/3: Randomize floor', f=font_s, col=COL_DIM)
        line('+/-: Speed', f=font_s, col=COL_DIM)
        y += 10

        for sid in sorted(shuttle_info.keys()):
            s = env.get_shuttle(sid)
            fid = s.floor_id
            col = FLOOR_COLORS[fid]['shuttle']
            line(f'── {FLOOR_LABELS[fid]} ──', f=font_b, col=col)

            # 상태
            state_col = {IDLE: COL_DIM, MOVING: COL_TEXT, DONE: COL_DIM
                         }.get(s.state, COL_TEXT)
            line(f'  S{sid} {s.state:<7s} v={s.v:.0f} mm/s',
                 f=font_s, col=state_col)
            line(f'  Node: {s.cur_node}', f=font_s, col=COL_DIM)

            # 경로 진행
            total = len(s.node_path)
            prog = s.path_idx
            if total > 1:
                bar_w = SIDE_W - 40
                bar_h = 8
                frac = prog / (total - 1) if total > 1 else 1.0
                pygame.draw.rect(screen, (40, 40, 50),
                                 pygame.Rect(sx, y, bar_w, bar_h),
                                 border_radius=3)
                pygame.draw.rect(screen, col,
                                 pygame.Rect(sx, y, int(bar_w * frac), bar_h),
                                 border_radius=3)
                y += bar_h + 4

            # 층 그래프 정보
            fg = floors[fid]
            line(f'  Nodes: {len(fg.nodes)}  '
                 f'Edges: {len(fg.edges)}',
                 f=font_s, col=COL_DIM)
            y += 6

        # 범례
        y = wh - 50
        pygame.draw.circle(screen, COL_GATE, (sx + 5, y + 5), 5)
        screen.blit(font_s.render('Lift gate', True, COL_DIM), (sx + 15, y))

        pygame.display.flip()


if __name__ == '__main__':
    main()
