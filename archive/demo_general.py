"""
demo_general.py — GeneralGraph + TAPGEnvironmentDES 데모.

PKL 파일 없이 임의 방향 그래프를 정의하고 TAPG 시뮬레이션을 실행합니다.

Keys
────
  SPACE   Start / Pause
  R       Reset
  +/-     Sim speed up / down
  Q/ESC   Quit

Usage
─────
    python mapf_edu/demo_general.py
    python mapf_edu/demo_general.py --no-vis   # 시각화 없이 CLI 출력만
    python mapf_edu/demo_general.py --headon   # head-on 규칙만 사용
"""
from __future__ import annotations
import sys, os, math, argparse, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from general_graph   import GeneralGraph
from general_planner import plan_agents, bfs_path, path_to_raw
from env_tapg        import TAPGAgent, IDLE, MOVING, ROTATING, WAITING, DONE
from env_tapg_des    import TAPGEnvironmentDES


# ── 예제 그래프 정의 ──────────────────────────────────────────────────────────
#
#         P1            P2
#          \            /
#     A ── B ── C ── D
#     |  ╲ |    | ╱  |
#     E ── F ── G ── H
#     |  ╱ |    | ╲  |
#     I ── J ── K ── L
#          /            \
#         P3            P4
#
#  A~L: 내부 노드 (3×4 배열, 비등간격 배치)
#  P1~P4: 포트 노드 (출발/목적지)
#  대각 엣지가 포함돼 일반 그래프임을 명확히 합니다.

_S = 4000   # grid spacing mm

NODES_XY = {
    # 내부 노드 (비등간격으로 배치 — grid 아님을 강조)
    'A': (0,      2*_S),
    'B': (1*_S,   2*_S + 800),   # 약간 위로
    'C': (2*_S,   2*_S),
    'D': (3*_S,   2*_S + 400),
    'E': (0,      _S),
    'F': (1*_S,   _S - 600),     # 약간 아래로
    'G': (2*_S,   _S),
    'H': (3*_S,   _S + 200),
    'I': (0,      0),
    'J': (1*_S,   0),
    'K': (2*_S,   0),
    'L': (3*_S,   0),
    # 포트 노드
    'P1': (-_S,   2*_S + 1000),
    'P2': (4*_S,  2*_S + 1000),
    'P3': (-_S,   -_S),
    'P4': (4*_S,  -_S),
}

# 단방향 엣지 목록 (양방향은 두 줄로 표기)
_EDGE_PAIRS = [
    # 수평
    ('A','B'), ('B','A'),
    ('B','C'), ('C','B'),
    ('C','D'), ('D','C'),
    ('E','F'), ('F','E'),
    ('F','G'), ('G','F'),
    ('G','H'), ('H','G'),
    ('I','J'), ('J','I'),
    ('J','K'), ('K','J'),
    ('K','L'), ('L','K'),
    # 수직
    ('A','E'), ('E','A'),
    ('E','I'), ('I','E'),
    ('B','F'), ('F','B'),
    ('F','J'), ('J','F'),
    ('C','G'), ('G','C'),
    ('G','K'), ('K','G'),
    ('D','H'), ('H','D'),
    ('H','L'), ('L','H'),
    # 대각 (비대칭 — 일반 그래프 특성)
    ('A','F'), ('F','C'),
    ('B','G'), ('G','D'),
    ('E','J'), ('J','G'),
    ('F','K'), ('K','H'),
    # 포트 연결
    ('P1','A'), ('A','P1'),
    ('P2','D'), ('D','P2'),
    ('P3','I'), ('I','P3'),
    ('P4','L'), ('L','P4'),
]

# 출발-목적지 쌍
OD_PAIRS = [
    ('P1', 'P4'),
    ('P2', 'P3'),
    ('P3', 'P2'),
    ('P4', 'P1'),
    ('P1', 'P3'),
    ('P2', 'P4'),
]


# ── 에이전트 색상 ─────────────────────────────────────────────────────────────
AGENT_COLORS = [
    (220,  80,  80),
    ( 80, 180, 220),
    (100, 220, 100),
    (220, 180,  60),
    (180,  80, 220),
    ( 80, 220, 180),
]


# ── CLI 실행 (시각화 없음) ─────────────────────────────────────────────────────

def run_cli(graph, raw_paths, accel, decel):
    agents = [TAPGAgent(i+1, AGENT_COLORS[i % len(AGENT_COLORS)], list(r))
              for i, r in enumerate(raw_paths)]
    env = TAPGEnvironmentDES(graph, accel=accel, decel=decel)
    env.setup(agents)

    stats = env.tapg_stats()
    print(f'\nTAPG: {stats["total"]} nodes, '
          f'{stats["cross_edges"]} cross-agent edges\n')

    t0 = time.perf_counter()
    while not env.all_done():
        if env._eq:
            env.step(env._eq[0].time)
        else:
            env.step(env.sim_time + 0.5)
        if env.sim_time > 3600:
            print('Timeout!')
            break

    wall = (time.perf_counter() - t0) * 1000
    print(f'Simulation finished: sim_time={env.sim_time:.2f}s  '
          f'wall={wall:.2f}ms')

    for a in agents:
        print(f'  Agent {a.id}: {a.state}  '
              f'pos=({a.x/1000:.1f}m, {a.y/1000:.1f}m)')


# ── Pygame 시각화 ─────────────────────────────────────────────────────────────

def run_vis(graph, raw_paths, accel, decel):
    import pygame

    # ── 레이아웃 상수 ────────────────────────────────────────────────────────
    MAP_W, WIN_H = 900, 700
    SIDE_W       = 240
    WIN_W        = MAP_W + SIDE_W
    PAD          = 50
    FPS          = 60
    SIM_SPEEDS   = [0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 50.0]
    SPD_LABELS   = ['0.25x','0.5x','1x','2x','5x','10x','50x']
    SPD_DEFAULT  = 2   # index

    BG       = (30,  30,  35)
    SIDE_BG  = (22,  22,  28)
    C_EDGE   = (70,  70,  90)
    C_ARROW  = (100, 100, 130)
    C_NODE   = (80,  130, 200)
    C_PORT   = (220, 160,  60)
    C_TEXT   = (200, 200, 210)
    C_DIM    = (100, 100, 110)
    C_WHITE  = (255, 255, 255)
    C_WAIT   = (255, 165,   0)
    C_DONE   = (50,   50,  60)

    # ── 좌표 변환 ────────────────────────────────────────────────────────────
    x0, y0, x1, y1 = graph.bbox
    mw, mh = x1 - x0, y1 - y0
    scale  = min((MAP_W - 2*PAD) / mw, (WIN_H - 2*PAD) / mh) if mw>0 and mh>0 else 0.05
    ox     = MAP_W / 2 - (x0 + x1) / 2 * scale
    oy     = WIN_H / 2 + (y0 + y1) / 2 * scale

    def to_screen(x_mm, y_mm):
        return (int(ox + x_mm * scale), int(oy - y_mm * scale))

    def px(mm):
        return mm * scale

    # ── Pygame 초기화 ─────────────────────────────────────────────────────────
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption('GeneralGraph TAPG Demo')
    clock  = pygame.font.SysFont('Consolas', 13)
    font_s = pygame.font.SysFont('Consolas', 13)
    font_m = pygame.font.SysFont('Consolas', 15)
    font_b = pygame.font.SysFont('Consolas', 16, bold=True)
    clk    = pygame.time.Clock()

    # ── 에이전트 + 환경 초기화 ────────────────────────────────────────────────
    def make_env():
        agents = [TAPGAgent(i+1, AGENT_COLORS[i % len(AGENT_COLORS)], list(r))
                  for i, r in enumerate(raw_paths)]
        env = TAPGEnvironmentDES(graph, accel=accel, decel=decel)
        env.setup(agents)
        env.step(0.0)   # 첫 TRY_ADVANCE 즉시 처리
        return env, agents

    env, agents = make_env()
    sim_time = 0.0
    running  = False
    spd_idx  = SPD_DEFAULT
    tapg_stats_cache = env.tapg_stats()

    # ── 헬퍼 그리기 함수 ──────────────────────────────────────────────────────
    def draw_arrow(surf, color, cx, cy, angle_rad, size=8):
        cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
        tip = (cx + cos_a * size,       cy - sin_a * size)
        bl  = (cx - cos_a*size*0.5 + sin_a*size*0.6,
               cy + sin_a*size*0.5 + cos_a*size*0.6)
        br  = (cx - cos_a*size*0.5 - sin_a*size*0.6,
               cy + sin_a*size*0.5 - cos_a*size*0.6)
        pygame.draw.polygon(surf, color, [tip, bl, br])

    def draw_rotated_rect(surf, color, cx, cy, length, width, angle_deg,
                          border=None, border_w=2):
        l2, w2 = length / 2, width / 2
        corners_local = [(-l2,-w2),(l2,-w2),(l2,w2),(-l2,w2)]
        rad = math.radians(angle_deg)
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        pts = [(int(cx + x*cos_a - y*sin_a),
                int(cy + x*sin_a + y*cos_a))
               for x, y in corners_local]
        pygame.draw.polygon(surf, color, pts)
        if border:
            pygame.draw.polygon(surf, border, pts, border_w)

    # ── 메인 루프 ─────────────────────────────────────────────────────────────
    while True:
        dt_real = clk.tick(FPS) / 1000.0

        # 이벤트 처리
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            if event.type == pygame.KEYDOWN:
                k = event.key
                if k in (pygame.K_q, pygame.K_ESCAPE):
                    pygame.quit(); sys.exit()
                elif k == pygame.K_SPACE:
                    running = not running
                elif k == pygame.K_r:
                    env, agents = make_env()
                    sim_time = 0.0
                    running  = False
                    tapg_stats_cache = env.tapg_stats()
                elif k in (pygame.K_EQUALS, pygame.K_PLUS):
                    spd_idx = min(spd_idx + 1, len(SIM_SPEEDS) - 1)
                elif k == pygame.K_MINUS:
                    spd_idx = max(spd_idx - 1, 0)

        # 시뮬레이션 전진
        if running and not env.all_done():
            sim_time += dt_real * SIM_SPEEDS[spd_idx]
            env.step(sim_time)
            tapg_stats_cache = env.tapg_stats()
        elif running and env.all_done():
            running = False

        # ── 렌더링 ────────────────────────────────────────────────────────────
        screen.fill(BG)
        pygame.draw.rect(screen, SIDE_BG, pygame.Rect(MAP_W, 0, SIDE_W, WIN_H))

        # 엣지
        for (fn, tn), edge in graph.edges.items():
            fn_n, tn_n = graph.nodes[fn], graph.nodes[tn]
            p0 = to_screen(fn_n.x, fn_n.y)
            p1 = to_screen(tn_n.x, tn_n.y)
            pygame.draw.line(screen, C_EDGE, p0, p1, 2)
            mx = p0[0] + (p1[0]-p0[0]) * 0.6
            my = p0[1] + (p1[1]-p0[1]) * 0.6
            scr_ang = math.atan2(-(tn_n.y - fn_n.y), tn_n.x - fn_n.x)
            draw_arrow(screen, C_ARROW, mx, my, scr_ang, size=6)

        # 노드
        r_px = max(6, min(int(px(400)), 12))
        for nid, node in graph.nodes.items():
            sx, sy  = to_screen(node.x, node.y)
            is_port = nid in graph.ports.values()
            col     = C_PORT if is_port else C_NODE
            pygame.draw.circle(screen, col, (sx, sy), r_px)
            pygame.draw.circle(screen, C_WHITE, (sx, sy), r_px, 1)
            lbl = font_s.render(nid, True, C_DIM)
            screen.blit(lbl, (sx + r_px + 2, sy - 7))

        # 에이전트 경로 (희미하게)
        path_surf = pygame.Surface((MAP_W, WIN_H), pygame.SRCALPHA)
        for agent in agents:
            pts = []
            for sid, _ in agent.raw_path:
                if sid.startswith('S,'):
                    nid = sid.split(',')[1]
                    if nid in graph.nodes:
                        pts.append(to_screen(graph.nodes[nid].x, graph.nodes[nid].y))
            if len(pts) >= 2:
                pygame.draw.lines(path_surf, (*agent.color, 60), False, pts, 2)
        screen.blit(path_surf, (0, 0))

        # 에이전트
        v_max = max((e.max_speed for e in graph.edges.values()), default=1000.0)
        l_px  = max(int(px(graph.vehicle_length)), 12)
        w_px  = max(int(px(graph.vehicle_width)),   6)

        for agent in agents:
            sx, sy   = to_screen(agent.x, agent.y)
            scr_ang  = math.degrees(-agent.theta)

            if agent.state == DONE:
                fill, border = C_DONE, C_DIM
            elif agent.state == WAITING:
                fill, border = agent.color, C_WAIT
            elif agent.state == MOVING:
                vf   = min(agent.v / v_max, 1.0)
                tint = tuple(int(c * (0.5 + 0.5 * vf)) for c in agent.color)
                fill, border = tint, C_WHITE
            else:
                fill, border = tuple(max(0,c-50) for c in agent.color), C_DIM

            draw_rotated_rect(screen, fill, sx, sy, l_px, w_px,
                              scr_ang, border=border)

            if agent.state == WAITING:
                pygame.draw.circle(screen, C_WAIT, (sx, sy), l_px//2 + 5, 3)

            # 헤드라이트
            rad = math.radians(scr_ang)
            hx  = sx + math.cos(rad) * l_px / 2
            hy  = sy + math.sin(rad) * l_px / 2
            pygame.draw.circle(screen, (255,255,180), (int(hx), int(hy)),
                               max(2, int(w_px * 0.2)))

            # ID
            lbl = font_s.render(str(agent.id), True,
                                C_WHITE if agent.state != DONE else C_DIM)
            screen.blit(lbl, lbl.get_rect(center=(sx, sy)))

        # ── 사이드바 ──────────────────────────────────────────────────────────
        sx_side = MAP_W + 10
        y = 12

        def line(text, color=C_TEXT, font=None):
            nonlocal y
            f = font or font_m
            screen.blit(f.render(text, True, color), (sx_side, y))
            y += f.size(text)[1] + 3

        line('GeneralGraph TAPG Demo', font=font_b)
        y += 6

        line('── Simulation ──', font=font_b)
        line(f'Time   : {sim_time:7.2f} s')
        line(f'Speed  : {SPD_LABELS[spd_idx]}')
        state_str = '▶ Running' if running else ('Done' if env.all_done() else '|| Paused')
        line(state_str, color=(100,220,100) if running else
             ((180,180,60) if env.all_done() else C_DIM))
        y += 6

        s = tapg_stats_cache
        line('── TAPG Stats ──', font=font_b)
        line(f'Nodes  : {s["total"]}',     color=C_DIM,  font=font_s)
        line(f'Done   : {s["done"]}',      color=(100,220,100), font=font_s)
        line(f'Remain : {s["remaining"]}', color=C_TEXT if s["remaining"] else C_DIM,
             font=font_s)
        line(f'XEdges : {s["cross_edges"]}', color=C_DIM, font=font_s)
        y += 6

        line('── Agents ──', font=font_b)
        ST_ICON = {MOVING:'>', WAITING:'||', IDLE:'o', DONE:'✓', ROTATING:'@'}
        ST_COL  = {MOVING:C_TEXT, WAITING:C_WAIT, IDLE:C_DIM, DONE:C_DIM}
        for agent in agents:
            pygame.draw.rect(screen, agent.color,
                             pygame.Rect(sx_side, y+3, 10, 10), border_radius=2)
            icon = ST_ICON.get(agent.state, '?')
            col  = ST_COL.get(agent.state, C_TEXT)
            screen.blit(
                font_s.render(f' A{agent.id} {icon}  v={agent.v/1000:.2f}m/s',
                              True, col),
                (sx_side + 12, y))
            y += 16

        y += 8
        line('── Graph ──', font=font_b)
        line(f'Nodes : {len(graph.nodes)}', color=C_DIM, font=font_s)
        line(f'Edges : {len(graph.edges)}', color=C_DIM, font=font_s)
        y += 8

        line('── Keys ──', font=font_b)
        for hint in ['SPACE - start/pause', 'R     - reset',
                     '+/-   - sim speed',   'Q/ESC - quit']:
            line(hint, color=C_DIM, font=font_s)

        pygame.display.flip()


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-vis',  action='store_true',
                        help='시각화 없이 CLI 출력만')
    parser.add_argument('--node-sharing', dest='headon', action='store_true',
                        help='node_sharing 충돌 규칙 사용 (주의: TAPG deadlock 가능)')
    parser.add_argument('--agents',  type=int, default=4,
                        help='에이전트 수 (최대 6, 기본 4)')
    parser.add_argument('--accel',   type=float, default=math.inf)
    parser.add_argument('--decel',   type=float, default=math.inf)
    parser.add_argument('--bfs',     action='store_true',
                        help='BFS 경로만 사용 (space-time A* 비활성)')
    args = parser.parse_args()

    # node_sharing은 TAPG에 순환 의존성을 유발할 수 있으므로 기본 head_on 사용
    # space-time A*가 노드 점유 충돌을 플래닝 단계에서 해소하기 때문
    conflict_rule = 'node_sharing' if args.headon else 'head_on'
    n_agents      = max(1, min(args.agents, len(OD_PAIRS)))

    print(f'GeneralGraph Demo')
    print(f'  nodes={len(NODES_XY)}  edges={len(_EDGE_PAIRS)}  '
          f'agents={n_agents}  conflict_rule={conflict_rule}')
    print(f'  accel={args.accel}  decel={args.decel}')

    graph = GeneralGraph(
        nodes_xy       = NODES_XY,
        edges          = _EDGE_PAIRS,
        max_speed      = 1000.0,
        vehicle_length = 1500.0,
        vehicle_width  =  700.0,
        conflict_rule  = conflict_rule,
        port_nodes     = ['P1', 'P2', 'P3', 'P4'],
    )
    print(f'\n{graph}')

    print(f'\nPlanning {n_agents} agents...')
    od = OD_PAIRS[:n_agents]
    try:
        raw_paths = plan_agents(graph, od,
                                use_spacetime=not args.bfs,
                                dt=0.1)
    except RuntimeError as e:
        print(f'Planning failed: {e}')
        sys.exit(1)

    print(f'\nPlanning done. {len(raw_paths)} paths generated.')

    if args.no_vis:
        run_cli(graph, raw_paths, args.accel, args.decel)
    else:
        run_vis(graph, raw_paths, args.accel, args.decel)
