"""
test_circular_v6_graph.py — Circular track test for graph_des_v6 (pure DES).

Uses circular.map.json (40-node ring, no merge/diverge).
Tests: leader-follower gap maintenance, event self-sustaining, collision-free.

Controls:
  Space        : Pause / Resume
  +/-          : Speed up / slow down
  1~9          : Set sim speed
  Left-click   : Select vehicle
  RClick-drag  : Pan
  Mouse wheel  : Zoom
  F (hold)     : Follow selected
  D            : Toggle dest-stop on vehicle 0
  R            : Reset view
  ESC          : Quit
"""

import sys, os, math, random, collections
import pygame

from graph_des_v5 import GraphMap, random_safe_path, _interp_path
from graph_des_v6 import (
    GraphDESv6, Vehicle,
    IDLE, ACCEL, CRUISE, DECEL, STOP, LOADING,
)

N_VEHICLES = 20
RADIUS = 20000

BG = (18, 18, 24)
TRACK_C = (50, 100, 170)
NODE_C = (60, 60, 80)
TEXT = (200, 200, 200)
DIM = (120, 120, 150)

STATE_C = {
    IDLE:    (100, 100, 100),
    ACCEL:   (80, 255, 80),
    CRUISE:  (60, 200, 60),
    DECEL:   (255, 200, 60),
    STOP:    (255, 60, 60),
    LOADING: (200, 60, 200),
}


def build_circular_path(gmap: GraphMap, start_node: str, laps: int = 10):
    """Build a multi-lap path around the circular track."""
    # Find the ring order
    ring = [start_node]
    cur = start_node
    visited = {start_node}
    for _ in range(len(gmap.nodes) * laps):
        neighbors = gmap.adj.get(cur, [])
        nxt = None
        for n in neighbors:
            if n not in visited:
                nxt = n
                break
        if nxt is None:
            # Wrap around — restart visited for next lap
            for n in neighbors:
                if n == ring[0] or n in gmap.adj.get(cur, []):
                    nxt = n
                    break
            if nxt is None:
                break
            visited.clear()
            visited.add(nxt)
        else:
            visited.add(nxt)
        ring.append(nxt)
        cur = nxt
    return ring


def main():
    random.seed(42)

    # ── Load circular map ─────────────────────────────────────────────────
    map_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "circular.map.json")
    gmap = GraphMap(map_path)
    n_nodes = len(gmap.nodes)
    print(f"Circular map: {n_nodes} nodes, {len(gmap.segments)} segments")
    print(f"  ZCU nodes: {len(gmap.zcu_nodes)} (expected 0 for simple ring)")

    # Node order around the ring
    ring_order = []
    cur = "0"
    for _ in range(n_nodes):
        ring_order.append(cur)
        neighbors = gmap.adj.get(cur, [])
        if neighbors:
            cur = neighbors[0]

    # ── Create engine ─────────────────────────────────────────────────────
    des = GraphDESv6(gmap)

    # ── Spawn vehicles evenly around the ring ─────────────────────────────
    vehicles = []
    spacing = n_nodes / N_VEHICLES
    for i in range(N_VEHICLES):
        node_idx = int(i * spacing) % n_nodes
        start = ring_order[node_idx]

        # Build long path: repeat ring multiple times
        path = []
        idx = node_idx
        for _ in range(n_nodes * 20):
            path.append(ring_order[idx % n_nodes])
            idx += 1
        path.append(ring_order[idx % n_nodes])

        c = pygame.Color(0)
        c.hsla = (i * 360 / N_VEHICLES, 80, 60, 100)
        v = Vehicle(i, gmap, path, (c.r, c.g, c.b))
        vehicles.append(v)
        des.add_vehicle(v)

    des.start_all()
    print(f"Spawned {N_VEHICLES} vehicles evenly around ring")

    # ── Pygame init ───────────────────────────────────────────────────────
    pygame.init()
    SW, SH = 1200, 900
    screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)
    pygame.display.set_caption("Circular DES v6 — Pure DES on ring track")
    clock = pygame.time.Clock()
    font_s = pygame.font.SysFont("consolas", 12)
    font_m = pygame.font.SysFont("consolas", 14)

    node_xy = {nid: (n.x, n.y) for nid, n in gmap.nodes.items()}

    scale = min(SW, SH) / (RADIUS * 2.6)
    cam_x, cam_y = 0.0, 0.0
    init_scale, init_cx, init_cy = scale, cam_x, cam_y

    sim_time = 0.0
    sim_speed = 3.0
    paused = False
    selected = None
    dragging = False
    drag_start = None
    total_viol = 0
    min_gap = float('inf')
    gap_hist = [0] * 20
    dest_mode = False

    def w2s(wx, wy):
        return (int((wx - cam_x) * scale + SW / 2),
                int(-(wy - cam_y) * scale + SH / 2))

    def s2w(sx, sy):
        return ((sx - SW / 2) / scale + cam_x,
                -(sy - SH / 2) / scale + cam_y)

    # ── Main loop ─────────────────────────────────────────────────────────
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
                elif event.key == pygame.K_d:
                    # Toggle destination on vehicle 0
                    v0 = vehicles[0]
                    if v0.dest_node is None:
                        # Set dest to node ~halfway around
                        target_idx = (ring_order.index(v0.seg_from) + n_nodes // 2) % n_nodes
                        v0.dest_node = ring_order[target_idx]
                        v0.dest_reached = False
                        dest_mode = True
                        print(f"V0 dest set to {v0.dest_node}")
                    else:
                        v0.dest_node = None
                        v0.dest_reached = False
                        dest_mode = False
                        print("V0 dest cleared")
                elif event.key == pygame.K_MINUS or event.key == pygame.K_KP_MINUS:
                    sim_speed = max(0.5, sim_speed / 1.5)
                elif event.key == pygame.K_EQUALS or event.key == pygame.K_KP_PLUS:
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
                    for v in vehicles:
                        sx, sy = w2s(v.x, v.y)
                        d = math.hypot(sx - mx, sy - my)
                        if d < bd:
                            best, bd = v, d
                    selected = best
                elif event.button == 4:
                    mx, my = event.pos
                    wx, wy = s2w(mx, my)
                    scale *= 1.15
                    cam_x = wx - (mx - SW / 2) / scale
                    cam_y = wy + (my - SH / 2) / scale
                elif event.button == 5:
                    mx, my = event.pos
                    wx, wy = s2w(mx, my)
                    scale /= 1.15
                    cam_x = wx - (mx - SW / 2) / scale
                    cam_y = wy + (my - SH / 2) / scale
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
            cam_x, cam_y = selected.x, selected.y

        # ── Sim ───────────────────────────────────────────────────────────
        if not paused:
            dt_sim = min(dt_real * sim_speed, 0.3)
            sim_time += dt_sim

            des.run_until(sim_time)
            des.query_positions(sim_time)

            # Path extension
            for v in vehicles:
                if v.needs_path_extension():
                    cur_node = v.path[-1]
                    idx = ring_order.index(cur_node) if cur_node in ring_order else 0
                    ext = []
                    for j in range(1, n_nodes * 5 + 1):
                        ext.append(ring_order[(idx + j) % n_nodes])
                    v.extend_path(ext)

            # Stats
            for v in vehicles:
                g = v.gap_to_leader
                if g < float('inf') and g < 200000:
                    b = min(int(g / 100), 19)
                    if b >= 0:
                        gap_hist[b] += 1
                    min_gap = min(min_gap, g)
                    if g < v.length:
                        total_viol += 1

        # ── Draw ──────────────────────────────────────────────────────────
        screen.fill(BG)

        # Track segments
        lw = max(1, min(4, int(scale * 100)))
        for (fn, tn), seg in gmap.segments.items():
            if not seg.path_points:
                continue
            pts_s = [w2s(p[0], p[1]) for p in seg.path_points]
            if len(pts_s) >= 2:
                pygame.draw.lines(screen, TRACK_C, False, pts_s, lw)

        # Nodes
        nr = max(3, int(scale * 100))
        for nid, (nx, ny) in node_xy.items():
            pygame.draw.circle(screen, NODE_C, w2s(nx, ny), nr)
            if scale > 0.01:
                lbl = font_s.render(nid, True, DIM)
                sx, sy = w2s(nx, ny)
                screen.blit(lbl, (sx + nr + 2, sy - 6))

        # Dest marker
        if dest_mode and vehicles[0].dest_node:
            dn = node_xy.get(vehicles[0].dest_node)
            if dn:
                dsx, dsy = w2s(*dn)
                r = max(8, int(scale * 400))
                pygame.draw.circle(screen, (255, 60, 255), (dsx, dsy), r, 3)
                lbl = font_m.render(f"DEST {vehicles[0].dest_node}", True, (255, 60, 255))
                screen.blit(lbl, (dsx + r + 4, dsy - 8))

        # Vehicles
        vhl, vhw = 750 / 2, 500 / 2
        for v in vehicles:
            sx, sy = w2s(v.x, v.y)
            if sx < -100 or sx > SW + 100 or sy < -100 or sy > SH + 100:
                continue

            th = v.theta
            ct, st = math.cos(th), math.sin(th)
            corners = [w2s(v.x + lx * ct - ly * st, v.y + lx * st + ly * ct)
                       for lx, ly in [(vhl, vhw), (vhl, -vhw), (-vhl, -vhw), (-vhl, vhw)]]

            clr = STATE_C.get(v.state, v.color)
            is_sel = selected and v.id == selected.id
            pygame.draw.polygon(screen, clr, corners)
            if is_sel:
                pygame.draw.polygon(screen, (255, 255, 100), corners, 2)

            # Front dot
            fx = v.x + vhl * 0.7 * ct
            fy = v.y + vhl * 0.7 * st
            pygame.draw.circle(screen, (255, 255, 255), w2s(fx, fy),
                               max(2, int(scale * 150)))

            # ID label
            if scale > 0.008:
                screen.blit(font_s.render(str(v.id), True, TEXT), (sx - 4, sy + 10))

        # Leader line for selected
        if selected and selected.leader:
            s1 = w2s(selected.x, selected.y)
            s2 = w2s(selected.leader.x, selected.leader.y)
            pygame.draw.line(screen, (255, 255, 100), s1, s2, 2)
            lx, ly = s2
            pygame.draw.circle(screen, (255, 255, 100), (lx, ly),
                               max(4, int(scale * 250)), 2)

        # X markers + connection line to OHT
        for v in vehicles:
            if v.x_marker_pidx >= 0:
                pidx = v.x_marker_pidx
                if pidx < len(v.path) - 1:
                    seg = v.gmap.segment_between(v.path[pidx], v.path[pidx + 1])
                    if seg and seg.path_points:
                        mx2, my2, _ = _interp_path(seg.path_points,
                                                    max(0, v.x_marker_offset))
                        msx, msy = w2s(mx2, my2)
                        vsx, vsy = w2s(v.x, v.y)
                        # Connection line from OHT to X marker
                        pygame.draw.line(screen, (*v.color[:3],), (vsx, vsy), (msx, msy), 1)
                        # X marker
                        r = max(3, int(scale * 120))
                        pygame.draw.line(screen, v.color,
                                         (msx - r, msy - r), (msx + r, msy + r), 2)
                        pygame.draw.line(screen, v.color,
                                         (msx - r, msy + r), (msx + r, msy - r), 2)

        # ── Info panel ────────────────────────────────────────────────────
        sc = collections.Counter(v.state for v in vehicles)
        lines = [
            f"t={sim_time:.1f}s x{sim_speed:.1f} {'PAUSED' if paused else ''} "
            f"[v6 pure-DES circular]",
            f"Events:{des.event_count}  SimTime:{des.sim_time:.2f}",
            f"A:{sc.get(ACCEL,0)} C:{sc.get(CRUISE,0)} D:{sc.get(DECEL,0)} "
            f"S:{sc.get(STOP,0)} I:{sc.get(IDLE,0)}",
            f"Violations:{total_viol}  MinGap:{min_gap:.0f}mm",
            f"Vehicles:{N_VEHICLES}  Nodes:{n_nodes}",
        ]
        if selected:
            v = selected
            lines.append("")
            lines.append(f"#{v.id} {v.state} v={v.vel_at(sim_time):.0f} a={v.acc:.0f}")
            lines.append(f"Gap:{v.gap_to_leader:.0f} Seg:{v.seg_from}->{v.seg_to}")
            lines.append(f"PathIdx:{v.path_idx}/{len(v.path)} off={v.seg_offset:.0f}")
            if v.leader:
                lines.append(f"Leader:#{v.leader.id} v={v.leader.vel_at(sim_time):.0f} "
                             f"L.seg:{v.leader.seg_from}->{v.leader.seg_to}")
            else:
                lines.append("Leader: None")
            if v.dest_node:
                lines.append(f"Dest:{v.dest_node} "
                             f"{'REACHED' if v.dest_reached else 'en-route'}")

        pw = 420
        ph = 18 * len(lines) + 16
        panel = pygame.Surface((pw, ph), pygame.SRCALPHA)
        panel.fill((30, 30, 45, 200))
        screen.blit(panel, (8, 8))
        for i, line in enumerate(lines):
            screen.blit(font_s.render(line, True, TEXT), (14, 14 + i * 18))

        # Gap histogram
        hx, hy, hw, hh = SW - 220, SH - 160, 200, 140
        p2 = pygame.Surface((hw + 10, hh + 10), pygame.SRCALPHA)
        p2.fill((30, 30, 45, 180))
        screen.blit(p2, (hx - 5, hy - 5))
        screen.blit(font_s.render("Gap(mm) 0~2000", True, TEXT), (hx, hy - 2))
        mx_c = max(gap_hist) if any(gap_hist) else 1
        bw2 = hw // 20
        for i, c in enumerate(gap_hist):
            bh = int((c / mx_c) * (hh - 20)) if mx_c > 0 else 0
            bc = ((255, 0, 0) if i * 100 < 750 else
                  (255, 200, 60) if i * 100 < 1150 else
                  (60, 200, 60))
            pygame.draw.rect(screen, bc, (hx + i * bw2, hy + hh - bh, bw2 - 1, bh))

        # Controls
        screen.blit(font_s.render(
            "Space:Pause +/-:Speed D:Dest F:Follow R:Reset", True, DIM),
            (10, SH - 20))

        pygame.display.flip()

    pygame.quit()


if __name__ == '__main__':
    main()
