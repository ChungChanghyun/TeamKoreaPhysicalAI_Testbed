"""Circular test using graph_des_v5 engine — pygame visualizer."""
import sys, os, math, random, collections
import pygame

from graph_des_v5 import (
    GraphMap, GraphDESv5, Vehicle,
    IDLE, ACCEL, CRUISE, DECEL, STOP, LOADING,
)

N_VEH = 30
RADIUS = 20000
BG=(18,18,24); TRACK_C=(50,100,170); NODE_C=(60,60,80); TEXT=(200,200,200); DIM=(120,120,150)
STATE_C = {IDLE:(100,100,100), ACCEL:(80,255,80), CRUISE:(60,200,60),
           DECEL:(255,200,60), STOP:(255,60,60), LOADING:(200,60,200)}


def main():
    random.seed(42)

    # ── Load circular map ─────────────────────────────────────────────────
    map_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "circular.map.json")
    gmap = GraphMap(map_path)
    n_nodes = len(gmap.nodes)
    print(f"Loaded circular map: {n_nodes} nodes, {len(gmap.segments)} segments")

    # Build ordered node ring for spawning
    ring = [str(i) for i in range(n_nodes)]

    # ── Force some nodes as ZCU nodes (every 10th) ────────────────────────
    ZCU_INTERVAL = 10
    for i in range(0, n_nodes, ZCU_INTERVAL):
        gmap.zcu_nodes.add(str(i))
    print(f"Injected {len(gmap.zcu_nodes)} ZCU nodes: {sorted(gmap.zcu_nodes, key=lambda x: int(x))}")

    # ── Create DES engine ─────────────────────────────────────────────────
    des = GraphDESv5(gmap)

    # ── Spawn vehicles evenly around the ring ─────────────────────────────
    vehicles = []
    spacing = n_nodes / N_VEH  # nodes apart

    for i in range(N_VEH):
        start_idx = int(i * spacing) % n_nodes
        start_node = ring[start_idx]
        # Build a long circular path
        path = []
        for j in range(500):
            path.append(ring[(start_idx + j) % n_nodes])
        c = pygame.Color(0); c.hsla = (i * 360 / N_VEH, 80, 60, 100)
        v = Vehicle(i, gmap, path, (c.r, c.g, c.b))
        vehicles.append(v)
        des.add_vehicle(v)

    des.start_all()
    print(f"Spawned {N_VEH} vehicles")

    # ── Vehicle 0: stop at node "10" (a ZCU node) ────────────────────────
    vehicles[0].dest_node = "10"
    print(f"Vehicle 0: dest_node='10' (will stop there)")

    # ── Pygame init ───────────────────────────────────────────────────────
    pygame.init()
    SW, SH = 1200, 900
    screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)
    pygame.display.set_caption("Circular DES v5 — Graph Engine")
    clock = pygame.time.Clock()
    font_s = pygame.font.SysFont("consolas", 12)

    scale = min(SW, SH) / (RADIUS * 2.6)
    cam_x = cam_y = 0.0
    sim_time = 0.0; sim_speed = 3.0; paused = False; selected = None
    dragging = False; drag_start = None
    total_viol = 0; min_gap = float('inf'); gap_hist = [0] * 20
    last_reassign = 0.0

    def w2s(wx, wy):
        return (int((wx - cam_x) * scale + SW / 2),
                int(-(wy - cam_y) * scale + SH / 2))

    def s2w(sx, sy):
        return ((sx - SW / 2) / scale + cam_x,
                -(sy - SH / 2) / scale + cam_y)

    # Node screen positions
    node_xy = {nid: (n.x, n.y) for nid, n in gmap.nodes.items()}

    # ── Main loop ─────────────────────────────────────────────────────────
    running = True
    while running:
        dt_real = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: running = False
                elif event.key == pygame.K_SPACE: paused = not paused
                elif event.key == pygame.K_r:
                    cam_x = cam_y = 0; scale = min(SW, SH) / (RADIUS * 2.6)
                elif event.key == pygame.K_MINUS: sim_speed = max(0.5, sim_speed / 1.5)
                elif event.key == pygame.K_EQUALS: sim_speed = min(50, sim_speed * 1.5)
                elif pygame.K_1 <= event.key <= pygame.K_9:
                    sim_speed = float(event.key - pygame.K_0)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3: dragging = True; drag_start = event.pos
                elif event.button == 1:
                    mx, my = event.pos; best, bd = None, 25
                    for v in vehicles:
                        d = math.hypot(w2s(v.x, v.y)[0] - mx, w2s(v.x, v.y)[1] - my)
                        if d < bd: best, bd = v, d
                    selected = best
                elif event.button == 4:
                    mx, my = event.pos; wx, wy = s2w(mx, my); scale *= 1.15
                    cam_x = wx - (mx - SW / 2) / scale; cam_y = wy + (my - SH / 2) / scale
                elif event.button == 5:
                    mx, my = event.pos; wx, wy = s2w(mx, my); scale /= 1.15
                    cam_x = wx - (mx - SW / 2) / scale; cam_y = wy + (my - SH / 2) / scale
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3: dragging = False
            elif event.type == pygame.MOUSEMOTION:
                if dragging:
                    dx = event.pos[0] - drag_start[0]; dy = event.pos[1] - drag_start[1]
                    cam_x -= dx / scale; cam_y += dy / scale; drag_start = event.pos
            elif event.type == pygame.VIDEORESIZE:
                SW, SH = event.w, event.h
                screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)

        if not paused:
            dt_sim = min(dt_real * sim_speed, 2.0)
            sim_time += dt_sim
            des.step(sim_time)

            # Vehicle 0: release after 120s of sim time
            v0 = vehicles[0]
            if v0.dest_reached and v0.dest_node is not None and sim_time > 120.0:
                print(f"t={sim_time:.1f}s: Vehicle 0 released from dest '{v0.dest_node}'")
                v0.dest_node = None
                v0.dest_reached = False

            # Periodic leader reassignment
            if sim_time - last_reassign >= 2.0:
                des.reassign_leaders_periodic(sim_time)
                last_reassign = sim_time

            for v in vehicles:
                g = v.gap_to_leader
                if g < float('inf') and g < 200000:
                    b = min(int(g / 100), 19)
                    if b >= 0: gap_hist[b] += 1
                    min_gap = min(min_gap, g)
                    if g < v.length: total_viol += 1

        # ── Draw ──────────────────────────────────────────────────────────
        screen.fill(BG)

        # Draw track segments
        lw = max(1, min(3, int(scale * 80)))
        for (fn, tn), seg in gmap.segments.items():
            if seg.path_points and len(seg.path_points) >= 2:
                pts_s = [w2s(p[0], p[1]) for p in seg.path_points]
                pygame.draw.lines(screen, TRACK_C, False, pts_s, lw)

        # Draw nodes
        nr = max(2, int(scale * 60))
        for nid, (nx, ny) in node_xy.items():
            if nid in gmap.zcu_nodes:
                pygame.draw.circle(screen, (255, 60, 60), w2s(nx, ny), nr + 2)
            else:
                pygame.draw.circle(screen, NODE_C, w2s(nx, ny), nr)

        # Draw vehicles
        vhl, vhw = 750 / 2, 500 / 2
        for v in vehicles:
            sx, sy = w2s(v.x, v.y)
            th = v.theta
            ct, st = math.cos(th), math.sin(th)
            corners = [w2s(v.x + lx * ct - ly * st, v.y + lx * st + ly * ct)
                       for lx, ly in [(vhl, vhw), (vhl, -vhw), (-vhl, -vhw), (-vhl, vhw)]]
            clr = STATE_C.get(v.state, v.color)
            is_sel = selected and v.id == selected.id
            pygame.draw.polygon(screen, clr, corners)
            if is_sel: pygame.draw.polygon(screen, (255, 255, 100), corners, 2)
            fx = v.x + vhl * 0.7 * ct; fy = v.y + vhl * 0.7 * st
            pygame.draw.circle(screen, (255, 255, 255), w2s(fx, fy), max(2, int(scale * 120)))

            # Gap bar
            g = v.gap_to_leader
            if g < float('inf') and g < 50000:
                bw = 30; ratio = min(g / v.h_min, 2.0); fill = int(bw * min(ratio, 1.0))
                bc = (255, 0, 0) if g < v.length else (255, 200, 60) if g < v.h_min else (60, 200, 60)
                bx, by = sx - bw // 2, sy - 18
                pygame.draw.rect(screen, (40, 40, 40), (bx, by, bw, 4))
                pygame.draw.rect(screen, bc, (bx, by, fill, 4))
                screen.blit(font_s.render(f"{g:.0f}", True, bc), (bx + bw + 2, by - 3))

            if scale > 0.015:
                screen.blit(font_s.render(str(v.id), True, (200, 200, 200)), (sx - 4, sy + 8))

        # X markers
        for v in vehicles:
            if v.x_marker_node is not None and v.x_marker_pidx >= 0:
                pidx = v.x_marker_pidx
                stop_off = v.x_marker_offset
                if pidx < len(v.path) - 1:
                    seg = v.gmap.segment_between(v.path[pidx], v.path[pidx + 1])
                    if seg and seg.path_points:
                        from graph_des_v5 import _interp_path
                        sx2, sy2, _ = _interp_path(seg.path_points, max(0, stop_off))
                        ssx, ssy = w2s(sx2, sy2)
                        r = max(3, int(scale * 100))
                        pygame.draw.line(screen, v.color, (ssx - r, ssy - r), (ssx + r, ssy + r), 2)
                        pygame.draw.line(screen, v.color, (ssx - r, ssy + r), (ssx + r, ssy - r), 2)
                        vsx, vsy = w2s(v.x, v.y)
                        pygame.draw.line(screen, (*v.color[:3], 100), (vsx, vsy), (ssx, ssy), 1)

        # Leader arrows for stopped vehicles
        for v in vehicles:
            if v.state in (STOP, LOADING) and v.leader:
                s1, s2 = w2s(v.x, v.y), w2s(v.leader.x, v.leader.y)
                pygame.draw.line(screen, (255, 80, 80), s1, s2, 1)
        if selected and selected.leader:
            s1, s2 = w2s(selected.x, selected.y), w2s(selected.leader.x, selected.leader.y)
            pygame.draw.line(screen, (255, 255, 100), s1, s2, 2)

        # Info panel
        sc = collections.Counter(v.state for v in vehicles)
        lines = [
            f"t={sim_time:.1f}s x{sim_speed:.1f} {'PAUSED' if paused else ''} [circular-v5]",
            f"Ev:{des.event_count}",
            f"A:{sc.get(ACCEL, 0)} C:{sc.get(CRUISE, 0)} D:{sc.get(DECEL, 0)} "
            f"S:{sc.get(STOP, 0)} L:{sc.get(LOADING, 0)} I:{sc.get(IDLE, 0)}",
            f"Violations:{total_viol} MinGap:{min_gap:.0f}mm",
        ]
        if selected:
            v = selected; lines.append("")
            lines.append(f"#{v.id} {v.state} v={v.vel_at(sim_time):.0f} a={v.acc:.0f}")
            lines.append(f"Gap:{v.gap_to_leader:.0f} PathIdx:{v.path_idx}/{len(v.path)}")
            lines.append(f"Seg:{v.seg_from}->{v.seg_to}")
            if v.leader: lines.append(f"Leader:#{v.leader.id}({v.leader.state})")
            if v.stop_dist is not None: lines.append(f"StopDist:{v.stop_dist:.0f}")
            if v.x_marker_node: lines.append(f"XMarker@{v.x_marker_node}")
            if v.dest_node: lines.append(f"Dest:{v.dest_node} {'REACHED' if v.dest_reached else ''}")

        pw = 380; ph = 18 * len(lines) + 16
        p = pygame.Surface((pw, ph), pygame.SRCALPHA); p.fill((30, 30, 45, 200))
        screen.blit(p, (8, 8))
        for i, line in enumerate(lines):
            screen.blit(font_s.render(line, True, TEXT), (14, 14 + i * 18))

        # Gap histogram
        hx, hy, hw, hh = SW - 220, SH - 160, 200, 140
        p2 = pygame.Surface((hw + 10, hh + 10), pygame.SRCALPHA); p2.fill((30, 30, 45, 180))
        screen.blit(p2, (hx - 5, hy - 5))
        screen.blit(font_s.render("Gap(mm)", True, TEXT), (hx, hy - 2))
        mx_c = max(gap_hist) if any(gap_hist) else 1
        bw2 = hw // 20
        for i, c in enumerate(gap_hist):
            bh = int((c / mx_c) * (hh - 20)) if mx_c > 0 else 0
            bc = (255, 0, 0) if i * 100 < 750 else (255, 200, 60) if i * 100 < 1150 else (60, 200, 60)
            pygame.draw.rect(screen, bc, (hx + i * bw2, hy + hh - bh, bw2 - 1, bh))

        screen.blit(font_s.render("Space:Pause +/-:Speed R:Reset", True, DIM), (10, SH - 20))
        pygame.display.flip()

    pygame.quit()


if __name__ == '__main__':
    main()
