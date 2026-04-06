"""
test_large_oht_des.py — env_oht_des on oht.large.map.json
Same visualization as test_graph_v6.py, but using env_oht_des engine.

Controls: same as test_graph_v6.py
  RClick-drag / Arrow keys : Pan
  Mouse wheel              : Zoom
  Left-click               : Select vehicle
  Space                    : Pause / Resume
  +/-                      : Speed up / slow down
  R                        : Reset view
  N                        : Toggle node dots
  Z                        : Toggle ZCU markers
  ESC                      : Quit
"""
import sys, os, math, random, collections, json
import pygame

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'TeamKoreaPhysicalAI_Testbed'))
from env_oht_des import (
    OHTNode, OHTSegment, OHTMap, OHTAgent, OHTEnvironmentDES,
    ZCUZone, IDLE, MOVING, FOLLOWING, BLOCKED, DONE,
    _interp_path,
)

# ── Colors ────────────────────────────────────────────────────────────────────

BG       = (18, 18, 24)
TRACK_C  = (50, 100, 170)
NODE_C   = (60, 60, 80)
TEXT     = (200, 200, 200)
DIM      = (120, 120, 150)

STATE_C = {
    IDLE:      (100, 100, 100),
    MOVING:    (60, 200, 60),
    FOLLOWING: (255, 210, 50),
    BLOCKED:   (255, 60, 60),
    DONE:      (60, 60, 60),
}

VEHICLE_COLORS = [
    (255, 80, 80), (80, 200, 255), (255, 200, 60), (100, 255, 130),
    (255, 130, 200), (200, 130, 255), (255, 160, 80), (130, 255, 220),
    (180, 180, 255), (255, 255, 150),
]

N_VEHICLES = 200


# ── Build OHTMap from large layout ───────────────────────────────────────────

def build_large_oht_map(json_path: str) -> OHTMap:
    """Build OHTMap from oht.large.map.json (no area filter, auto ZCU)."""
    with open(json_path, 'r', encoding='utf-8') as f:
        d = json.load(f)

    m = object.__new__(OHTMap)
    m.nodes = {}
    m.segments = {}
    m.adj = {}

    for n in d['nodes']:
        m.nodes[n['id']] = OHTNode(n['id'], n['x'], n['y'])
    m.adj = {nid: [] for nid in m.nodes}

    for s in d['segments']:
        fn, tn = s['startNodeId'], s['endNodeId']
        if fn not in m.nodes or tn not in m.nodes:
            continue
        n1, n2 = m.nodes[fn], m.nodes[tn]
        length = math.hypot(n2.x - n1.x, n2.y - n1.y)
        pts = [(n1.x, n1.y), (n2.x, n2.y)]
        seg = OHTSegment(s['id'], fn, tn, length, s.get('speed', 3600), pts)
        m.segments[(fn, tn)] = seg
        m.adj[fn].append(tn)

    m.vehicle_length = 750
    m.vehicle_width = 460
    m.h_min = m.vehicle_length + 400
    m.accel = 500
    m.decel = 500
    max_v = max((s.max_speed for s in m.segments.values()), default=3600)
    m.max_brake = max_v ** 2 / (2.0 * m.decel)
    m.port_nodes = set()

    # Auto-detect merge/diverge ZCU
    in_deg = collections.Counter()
    out_deg = collections.Counter()
    for (fn, tn) in m.segments:
        in_deg[tn] += 1
        out_deg[fn] += 1

    m.zcu_zones = []
    m._entry_zcu = {}
    m._exit_zcu = {}
    m.zcu_node_ids = set()
    m.merge_nodes = set()
    m.diverge_nodes = set()

    for nid in m.nodes:
        if in_deg[nid] >= 2:
            entry = frozenset(k for k in m.segments if k[1] == nid)
            exit_ = frozenset(k for k in m.segments if k[0] == nid)
            if entry and exit_:
                z = ZCUZone(f'merge_{nid}', entry, exit_)
                m.zcu_zones.append(z)
                for k in entry:
                    m._entry_zcu[k] = z
                for k in exit_:
                    m._exit_zcu[k] = z
                m.zcu_node_ids.add(nid)
                m.merge_nodes.add(nid)

        if out_deg[nid] >= 2:
            entry = frozenset(k for k in m.segments if k[1] == nid)
            exit_ = frozenset(k for k in m.segments if k[0] == nid)
            if entry and exit_:
                z = ZCUZone(f'diverge_{nid}', entry, exit_)
                m.zcu_zones.append(z)
                for k in entry:
                    if k not in m._entry_zcu:
                        m._entry_zcu[k] = z
                for k in exit_:
                    if k not in m._exit_zcu:
                        m._exit_zcu[k] = z
                m.zcu_node_ids.add(nid)
                m.diverge_nodes.add(nid)

    xs = [n.x for n in m.nodes.values()]
    ys = [n.y for n in m.nodes.values()]
    pad = m.vehicle_length
    m.bbox = (min(xs)-pad, min(ys)-pad, max(xs)+pad, max(ys)+pad)

    print(f'Map: {len(m.nodes)} nodes, {len(m.segments)} segments, '
          f'{len(m.zcu_zones)} ZCU ({len(m.merge_nodes)} merge, {len(m.diverge_nodes)} diverge)')
    return m


def random_walk_path(oht_map, start, length=500):
    path = [start]
    cur = start
    prev = None
    for _ in range(length):
        nbs = oht_map.adj.get(cur, [])
        if not nbs:
            break
        cands = [n for n in nbs if n != prev] or nbs
        nxt = random.choice(cands)
        path.append(nxt)
        prev = cur
        cur = nxt
    return path


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    random.seed(42)

    map_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "oht.large.map.json")
    oht_map = build_large_oht_map(map_path)

    env = OHTEnvironmentDES(oht_map, cross_segment=True)

    # Spawn
    agents = []
    excluded = set()
    all_nodes = list(oht_map.nodes.keys())

    for i in range(N_VEHICLES):
        for _ in range(200):
            start = random.choice(all_nodes)
            if start in excluded:
                continue
            path = random_walk_path(oht_map, start, length=500)
            if len(path) < 10:
                continue
            color = VEHICLE_COLORS[i % len(VEHICLE_COLORS)]
            a = OHTAgent(i, color, path, max_speed=3600)
            agents.append(a)
            excluded |= oht_map.nearby_nodes(start, oht_map.h_min)
            env.add_agent(a, t_start=0.0)
            break

    print(f'Spawned {len(agents)} OHTs (env_oht_des)')

    # Pygame
    pygame.init()
    disp = pygame.display.Info()
    SW = min(1600, disp.current_w - 100)
    SH = min(900, disp.current_h - 100)
    screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)
    pygame.display.set_caption(f"env_oht_des Large Map ({len(agents)} OHTs)")
    clock = pygame.time.Clock()
    font_s = pygame.font.SysFont("consolas", 12)

    bx0, by0, bx1, by1 = oht_map.bbox
    world_cx = (bx0 + bx1) / 2
    world_cy = (by0 + by1) / 2
    scale = min(SW / (bx1 - bx0 + 100), SH / (by1 - by0 + 100)) * 0.9
    cam_x, cam_y = world_cx, world_cy
    init_scale, init_cx, init_cy = scale, cam_x, cam_y

    node_xy = {nid: (n.x, n.y) for nid, n in oht_map.nodes.items()}

    sim_time = 0.0
    sim_speed = 3.0
    paused = False
    show_nodes = False
    show_zcu = True
    selected = None
    dragging = False
    drag_start = None

    running = True
    while running:
        dt_real = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    cam_x, cam_y, scale = init_cx, init_cy, init_scale
                    selected = None
                elif event.key == pygame.K_n:
                    show_nodes = not show_nodes
                elif event.key == pygame.K_z:
                    show_zcu = not show_zcu
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    sim_speed = max(0.5, sim_speed / 1.5)
                elif event.key in (pygame.K_EQUALS, pygame.K_KP_PLUS):
                    sim_speed = min(50.0, sim_speed * 1.5)
                elif pygame.K_1 <= event.key <= pygame.K_9:
                    sim_speed = float(event.key - pygame.K_0)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3:
                    dragging = True
                    drag_start = event.pos
                elif event.button == 1:
                    mx, my = event.pos
                    best, bd = None, 25
                    for a in agents:
                        if a.seg and a.seg.path_points:
                            pos = a.pos(sim_time)
                            px, py, _ = _interp_path(a.seg.path_points, max(0, pos))
                        else:
                            nid = a.node_path[min(a.path_idx, len(a.node_path)-1)]
                            n = oht_map.nodes.get(nid)
                            if not n: continue
                            px, py = n.x, n.y
                        sx = int((px - cam_x) * scale + SW/2)
                        sy = int(-(py - cam_y) * scale + SH/2)
                        d = math.hypot(sx - mx, sy - my)
                        if d < bd:
                            best, bd = a, d
                    selected = best
                elif event.button == 4:
                    mx, my = event.pos
                    wx = (mx - SW/2) / scale + cam_x
                    wy = -(my - SH/2) / scale + cam_y
                    scale *= 1.15
                    cam_x = wx - (mx - SW/2) / scale
                    cam_y = wy + (my - SH/2) / scale
                elif event.button == 5:
                    mx, my = event.pos
                    wx = (mx - SW/2) / scale + cam_x
                    wy = -(my - SH/2) / scale + cam_y
                    scale /= 1.15
                    cam_x = wx - (mx - SW/2) / scale
                    cam_y = wy + (my - SH/2) / scale
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3:
                    dragging = False
            elif event.type == pygame.MOUSEMOTION:
                if dragging:
                    dx = event.pos[0] - drag_start[0]
                    dy = event.pos[1] - drag_start[1]
                    cam_x -= dx / scale
                    cam_y += dy / scale
                    drag_start = event.pos
            elif event.type == pygame.VIDEORESIZE:
                SW, SH = event.w, event.h
                screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)

        keys = pygame.key.get_pressed()
        pan = 500 / scale
        if keys[pygame.K_LEFT]:  cam_x -= pan
        if keys[pygame.K_RIGHT]: cam_x += pan
        if keys[pygame.K_UP]:    cam_y += pan
        if keys[pygame.K_DOWN]:  cam_y -= pan

        if selected and keys[pygame.K_f]:
            if selected.seg and selected.seg.path_points:
                pos = selected.pos(sim_time)
                px, py, _ = _interp_path(selected.seg.path_points, max(0, pos))
                cam_x, cam_y = px, py

        def w2s(wx, wy):
            return (int((wx - cam_x) * scale + SW/2),
                    int(-(wy - cam_y) * scale + SH/2))

        # Step
        if not paused:
            dt_sim = min(dt_real * sim_speed, 0.3)
            sim_time += dt_sim
            env.step(sim_time)

            # Reassign done agents
            for a in agents:
                if a.state == DONE:
                    nid = a.node_path[min(a.path_idx, len(a.node_path)-1)]
                    new_path = random_walk_path(oht_map, nid, 500)
                    if len(new_path) >= 5:
                        env.reassign(a, new_path, sim_time)

        # Draw
        screen.fill(BG)

        vl_, vt_ = (0 - SW/2) / scale + cam_x, -(0 - SH/2) / scale + cam_y
        vr_, vb_ = (SW - SW/2) / scale + cam_x, -(SH - SH/2) / scale + cam_y
        margin = 5000
        vl_ -= margin; vr_ += margin
        vy_min = min(vt_, vb_) - margin
        vy_max = max(vt_, vb_) + margin

        lw = max(1, min(3, int(scale * 80)))
        for seg in oht_map.segments.values():
            p0 = seg.path_points[0]
            pn = seg.path_points[-1]
            mx = (p0[0] + pn[0]) / 2
            my = (p0[1] + pn[1]) / 2
            if mx < vl_ or mx > vr_ or my < vy_min or my > vy_max:
                continue
            pts_s = [w2s(p[0], p[1]) for p in seg.path_points]
            if len(pts_s) >= 2:
                pygame.draw.lines(screen, TRACK_C, False, pts_s, lw)

        if show_nodes:
            nr = max(2, int(scale * 60))
            for nid, (nx, ny) in node_xy.items():
                if nx < vl_ or nx > vr_ or ny < vy_min or ny > vy_max:
                    continue
                pygame.draw.circle(screen, NODE_C, w2s(nx, ny), nr)

        if show_zcu:
            mr = max(3, int(scale * 120))
            for nid in oht_map.merge_nodes:
                nx, ny = node_xy[nid]
                if nx < vl_ or nx > vr_ or ny < vy_min or ny > vy_max:
                    continue
                pygame.draw.circle(screen, (255, 60, 60), w2s(nx, ny), mr)
            for nid in oht_map.diverge_nodes:
                nx, ny = node_xy[nid]
                if nx < vl_ or nx > vr_ or ny < vy_min or ny > vy_max:
                    continue
                pygame.draw.circle(screen, (255, 180, 60), w2s(nx, ny), mr)

        # Agents
        for a in agents:
            if a.seg and a.seg.path_points:
                pos = a.pos(sim_time)
                px, py, theta = _interp_path(a.seg.path_points, max(0, pos))
            else:
                nid = a.node_path[min(a.path_idx, len(a.node_path)-1)]
                n = oht_map.nodes.get(nid)
                if not n: continue
                px, py = n.x, n.y
                theta = 0

            sx, sy = w2s(px, py)
            if sx < -50 or sx > SW + 50 or sy < -50 or sy > SH + 50:
                continue

            fill = STATE_C.get(a.state, a.color)
            is_sel = selected and a.id == selected.id

            ct, st_ = math.cos(theta), math.sin(theta)
            vhl, vhw = 375, 250
            corners = [w2s(px + lx*ct - ly*st_, py + lx*st_ + ly*ct)
                       for lx, ly in [(vhl, vhw), (vhl, -vhw), (-vhl, -vhw), (-vhl, vhw)]]
            pygame.draw.polygon(screen, fill, corners)
            if is_sel:
                pygame.draw.polygon(screen, (255, 255, 100), corners, 2)

            fx = px + vhl * 0.7 * ct
            fy = py + vhl * 0.7 * st_
            pygame.draw.circle(screen, (255, 255, 255), w2s(fx, fy), max(2, int(scale * 120)))

            if scale > 0.015:
                screen.blit(font_s.render(str(a.id), True, (200, 200, 200)), (sx - 4, sy + 8))

        # HUD
        sc = collections.Counter(a.state for a in agents)
        lines = [
            f"t={sim_time:.1f}s x{sim_speed:.1f} {'PAUSED' if paused else ''} [env_oht_des]",
            f"Events:{env.event_count}",
            f"MOV:{sc.get(MOVING,0)} FOL:{sc.get(FOLLOWING,0)} "
            f"BLK:{sc.get(BLOCKED,0)} IDL:{sc.get(IDLE,0)} DONE:{sc.get(DONE,0)}",
            f"Vehicles:{len(agents)} Segs:{len(oht_map.segments)} ZCU:{len(oht_map.zcu_zones)}",
        ]
        if selected:
            a = selected
            lines.append("")
            lines.append(f"#{a.id} {a.state} v={a.v:.0f} seg={a.seg.id if a.seg else '-'}")
            lines.append(f"PathIdx:{a.path_idx}/{len(a.node_path)}")

        pw = 420
        ph = 18 * len(lines) + 16
        panel = pygame.Surface((pw, ph), pygame.SRCALPHA)
        panel.fill((30, 30, 45, 200))
        screen.blit(panel, (8, 8))
        for i, line in enumerate(lines):
            screen.blit(font_s.render(line, True, TEXT), (14, 14 + i * 18))

        screen.blit(font_s.render("Space:Pause +/-:Speed R:Reset N:Nodes Z:ZCU F:Follow",
                                  True, DIM), (10, SH - 20))

        pygame.display.flip()

    pygame.quit()


if __name__ == '__main__':
    main()
