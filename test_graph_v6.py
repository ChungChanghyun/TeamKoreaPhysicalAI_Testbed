"""
test_graph_v6.py — Graph DES v6 (Pure DES) visualizer for oht.large.map.json.

v6 changes from v5:
  - Pure DES: events self-sustain, no external step() pump required
  - ZCU wait-queue with event-driven wakeup (no polling)
  - Leader→follower O(k) notification
  - Rendering separated from simulation via query_positions()
  - Backwards-compatible step() wrapper for this visualizer

Controls:
  RClick-drag / Arrow keys : Pan
  Mouse wheel              : Zoom
  Left-click               : Select vehicle
  Space                    : Pause / Resume
  +/-                      : Speed up / slow down sim
  1~9                      : Set sim speed multiplier
  R                        : Reset view
  N                        : Toggle node dots
  Z                        : Toggle ZCU markers
  F (hold)                 : Follow selected vehicle
  ESC                      : Quit
"""

import sys, os, math, random, collections
import pygame

from graph_des_v5 import (
    GraphMap, random_safe_path, _interp_path,
)
from graph_des_v6 import (
    GraphDESv6, Vehicle,
    IDLE, ACCEL, CRUISE, DECEL, STOP, LOADING,
)

# ── Colors ────────────────────────────────────────────────────────────────────

BG        = (18, 18, 24)
TRACK_C   = (50, 100, 170)
NODE_C    = (60, 60, 80)
TEXT      = (200, 200, 200)
DIM       = (120, 120, 150)
ZCU_MERGE_C  = (255, 60, 60)
ZCU_DIVERGE_C = (255, 180, 60)
ZCU_CURVE_C  = (255, 100, 50)
ZCU_STRAIGHT_C = (80, 220, 255)

STATE_C = {
    IDLE:    (100, 100, 100),
    ACCEL:   (80, 255, 80),
    CRUISE:  (60, 200, 60),
    DECEL:   (255, 200, 60),
    STOP:    (255, 60, 60),
    LOADING: (200, 60, 200),
}

VEHICLE_COLORS = [
    (255, 80,  80),
    (80,  200, 255),
    (255, 200, 60),
    (100, 255, 130),
    (255, 130, 200),
    (200, 130, 255),
    (255, 160, 80),
    (130, 255, 220),
    (180, 180, 255),
    (255, 255, 150),
]

N_VEHICLES = 200


def main():
    random.seed(42)

    # ── Load map ──────────────────────────────────────────────────────────
    map_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "oht.large.map.json")
    gmap = GraphMap(map_path)
    print(f"Loaded: {len(gmap.nodes)} nodes, {len(gmap.segments)} segments, "
          f"main loop: {len(gmap.main_loop)} nodes")

    # Good start candidates
    degree = collections.Counter()
    for fn, tn in gmap.segments:
        degree[fn] += 1
        degree[tn] += 1
    good_starts = [nid for nid in gmap.main_loop
                   if degree[nid] >= 2 and gmap.adj.get(nid)]
    if not good_starts:
        good_starts = [nid for nid in gmap.main_loop if gmap.adj.get(nid)]
    print(f"Good start nodes: {len(good_starts)}")

    # ── Create DES engine (v6 — pure DES) ─────────────────────────────────
    des = GraphDESv6(gmap)

    # ── Spawn vehicles ────────────────────────────────────────────────────
    used_nodes = set()
    vehicles = []

    def spawn_vehicles(count):
        nonlocal used_nodes
        for i in range(count):
            vid = len(vehicles)
            candidates = [n for n in good_starts if n not in used_nodes]
            if not candidates:
                candidates = good_starts
            start = random.choice(candidates)

            path = random_safe_path(gmap, start, length=200)
            color = VEHICLE_COLORS[vid % len(VEHICLE_COLORS)]
            v = Vehicle(vid, gmap, path, color)
            vehicles.append(v)
            des.add_vehicle(v)

            used_nodes.add(start)
            for nb in gmap.adj.get(start, []):
                used_nodes.add(nb)

    spawn_vehicles(N_VEHICLES)
    des.start_all()
    print(f"Spawned {N_VEHICLES} vehicles (pure DES v6)")

    # ── Pygame init ───────────────────────────────────────────────────────
    pygame.init()
    disp = pygame.display.Info()
    SW = min(1600, disp.current_w - 100)
    SH = min(900, disp.current_h - 100)
    screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)
    pygame.display.set_caption("Graph DES v6 — Pure DES + ZCU wait-queue")
    clock = pygame.time.Clock()
    font_s = pygame.font.SysFont("consolas", 12)

    # ── Camera ────────────────────────────────────────────────────────────
    bx0, by0, bx1, by1 = gmap.bbox
    world_cx = (bx0 + bx1) / 2
    world_cy = (by0 + by1) / 2
    world_w = bx1 - bx0
    world_h = by1 - by0
    scale = min(SW / (world_w * 1.1), SH / (world_h * 1.1))
    cam_x, cam_y = world_cx, world_cy
    init_scale, init_cx, init_cy = scale, cam_x, cam_y

    node_xy = {nid: (n.x, n.y) for nid, n in gmap.nodes.items()}

    # ── State ─────────────────────────────────────────────────────────────
    sim_time = 0.0
    sim_speed = 3.0
    paused = True
    show_nodes = True
    show_zcu = True
    selected = None
    dragging = False
    drag_start = None
    hovered_zcu_node = None
    total_viol = 0
    min_gap = float('inf')
    gap_hist = [0] * 20

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
                elif event.key == pygame.K_n:
                    show_nodes = not show_nodes
                elif event.key == pygame.K_z:
                    show_zcu = not show_zcu
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
                elif show_zcu:
                    mx, my = event.pos
                    hovered_zcu_node = None
                    best_d = 20
                    # Hover both ZCU nodes and boundary nodes
                    all_zcu = gmap.zcu_nodes | des._boundary_nodes
                    for nid in all_zcu:
                        if nid not in node_xy:
                            continue
                        nx, ny = node_xy[nid]
                        sx, sy = w2s(nx, ny)
                        d = math.hypot(sx - mx, sy - my)
                        if d < best_d:
                            best_d = d
                            hovered_zcu_node = nid

            elif event.type == pygame.VIDEORESIZE:
                SW, SH = event.w, event.h
                screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)

        # Arrow key pan
        keys = pygame.key.get_pressed()
        pan = 500 / scale
        if keys[pygame.K_LEFT]:  cam_x -= pan
        if keys[pygame.K_RIGHT]: cam_x += pan
        if keys[pygame.K_UP]:    cam_y += pan
        if keys[pygame.K_DOWN]:  cam_y -= pan

        # Follow selected
        if selected and keys[pygame.K_f]:
            cam_x, cam_y = selected.x, selected.y

        # ── Sim step ──────────────────────────────────────────────────────
        if not paused:
            dt_sim = min(dt_real * sim_speed, 0.3)
            sim_time += dt_sim

            # Pure DES: process events, then query positions for rendering
            des.run_until(sim_time)
            des.query_positions(sim_time)

            # Track stats
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

        # Visible world rect for culling
        vl, vt = s2w(0, 0)
        vr, vb = s2w(SW, SH)
        margin = 5000
        vl -= margin; vr += margin
        vt_y = min(vt, vb) - margin
        vb_y = max(vt, vb) + margin

        # Draw segments
        lw = max(1, min(3, int(scale * 80)))
        for (fn, tn), seg in gmap.segments.items():
            if not seg.path_points:
                continue
            p0 = seg.path_points[0]
            pn = seg.path_points[-1]
            mid_x = (p0[0] + pn[0]) / 2
            mid_y = (p0[1] + pn[1]) / 2
            if mid_x < vl or mid_x > vr or mid_y < vt_y or mid_y > vb_y:
                continue

            pts_s = [w2s(p[0], p[1]) for p in seg.path_points]
            if len(pts_s) >= 2:
                pygame.draw.lines(screen, TRACK_C, False, pts_s, lw)

        # Draw nodes
        if show_nodes:
            nr = max(2, int(scale * 60))
            for nid, (nx, ny) in node_xy.items():
                if nx < vl or nx > vr or ny < vt_y or ny > vb_y:
                    continue
                pygame.draw.circle(screen, NODE_C, w2s(nx, ny), nr)

        # ── ZCU markers ──────────────────────────────────────────────────
        if show_zcu:
            mr = max(3, int(scale * 120))
            # Merge nodes (red)
            for nid in gmap.merge_nodes:
                nx, ny = node_xy[nid]
                if nx < vl or nx > vr or ny < vt_y or ny > vb_y:
                    continue
                pygame.draw.circle(screen, ZCU_MERGE_C, w2s(nx, ny), mr)
            # Diverge nodes (orange)
            for nid in gmap.diverge_nodes:
                nx, ny = node_xy[nid]
                if nx < vl or nx > vr or ny < vt_y or ny > vb_y:
                    continue
                pygame.draw.circle(screen, ZCU_DIVERGE_C, w2s(nx, ny), mr)
            # Boundary nodes (small yellow diamond) — only those not already merge/diverge
            br = max(2, int(scale * 80))
            for nid in des._boundary_nodes:
                if nid in gmap.merge_nodes or nid in gmap.diverge_nodes:
                    continue
                if nid not in node_xy:
                    continue
                nx, ny = node_xy[nid]
                if nx < vl or nx > vr or ny < vt_y or ny > vb_y:
                    continue
                bsx, bsy = w2s(nx, ny)
                # Show locked boundary as filled, free as outline
                is_locked = any(des._zone_lock.get(lid) is not None
                                for _, lid in des._boundary_to_zones.get(nid, []))
                bc = (255, 255, 60) if is_locked else (120, 120, 60)
                dm = [(bsx, bsy-br), (bsx+br, bsy), (bsx, bsy+br), (bsx-br, bsy)]
                if is_locked:
                    pygame.draw.polygon(screen, bc, dm)
                else:
                    pygame.draw.polygon(screen, bc, dm, 1)

            # Hovered ZCU detail + lock status
            if hovered_zcu_node:
                zcu_lw = max(3, min(6, int(scale * 200)))
                hn = node_xy[hovered_zcu_node]
                hsx, hsy = w2s(*hn)

                r = max(6, int(scale * 300))
                is_merge = hovered_zcu_node in gmap.merge_nodes
                is_diverge = hovered_zcu_node in gmap.diverge_nodes
                is_boundary = hovered_zcu_node in des._boundary_nodes

                if is_merge:
                    diamond_c = ZCU_MERGE_C
                elif is_diverge:
                    diamond_c = ZCU_DIVERGE_C
                else:
                    diamond_c = (200, 200, 60)  # boundary node
                diamond = [(hsx, hsy-r), (hsx+r, hsy), (hsx, hsy+r), (hsx-r, hsy)]
                pygame.draw.polygon(screen, diamond_c, diamond)
                pygame.draw.polygon(screen, (255,255,255), diamond, 2)

                # Draw entry/exit segments
                if is_merge:
                    for pred in gmap.adj_rev.get(hovered_zcu_node, []):
                        seg_key = (pred, hovered_zcu_node)
                        seg = gmap.segments.get(seg_key)
                        if seg and seg.path_points:
                            pts_s = [w2s(p[0], p[1]) for p in seg.path_points]
                            if len(pts_s) >= 2:
                                is_curve = seg_key in gmap.merge_curve_entries
                                c = ZCU_CURVE_C if is_curve else ZCU_STRAIGHT_C
                                pygame.draw.lines(screen, c, False, pts_s, zcu_lw)
                        if pred in node_xy:
                            pygame.draw.circle(screen, (255,200,60), w2s(*node_xy[pred]), mr+2, 2)
                if is_diverge:
                    for succ in gmap.adj.get(hovered_zcu_node, []):
                        seg_key = (hovered_zcu_node, succ)
                        seg = gmap.segments.get(seg_key)
                        if seg and seg.path_points:
                            pts_s = [w2s(p[0], p[1]) for p in seg.path_points]
                            if len(pts_s) >= 2:
                                is_curve = seg_key in gmap.diverge_curve_exits
                                c = ZCU_CURVE_C if is_curve else ZCU_STRAIGHT_C
                                pygame.draw.lines(screen, c, False, pts_s, zcu_lw)
                        if succ in node_xy:
                            pygame.draw.circle(screen, (255,200,60), w2s(*node_xy[succ]), mr+2, 2)

                # ── Lock status info panel ────────────────────────────
                info_lines = []
                node_label = hovered_zcu_node
                types = []
                if is_merge: types.append("MERGE")
                if is_diverge: types.append("DIVERGE")
                if is_boundary: types.append("BOUNDARY")
                info_lines.append(f"{'+'.join(types) if types else 'NODE'} {node_label}")

                # Find all zones related to this node
                # 1. As a zone center (merge/diverge)
                for zone in gmap.zcu_zones:
                    if zone.node_id == hovered_zcu_node:
                        lock_id = f"{zone.node_id}_{zone.kind}"
                        holder = des._zone_lock.get(lock_id)
                        waiters = des._zone_waiters.get(lock_id, [])
                        holder_str = f"V#{holder.id}" if holder else "FREE"
                        wait_str = ",".join(f"V#{w.id}" for w in waiters) if waiters else "-"
                        info_lines.append(f"  {zone.kind} lock={holder_str} wait=[{wait_str}]")

                # 2. As a boundary node
                for zone, lock_id in des._boundary_to_zones.get(hovered_zcu_node, []):
                    holder = des._zone_lock.get(lock_id)
                    waiters = des._zone_waiters.get(lock_id, [])
                    holder_str = f"V#{holder.id}" if holder else "FREE"
                    wait_str = ",".join(f"V#{w.id}" for w in waiters) if waiters else "-"
                    info_lines.append(f"  bnd->{zone.node_id}_{zone.kind} lock={holder_str} wait=[{wait_str}]")

                # 3. As an exit node
                for zone, lock_id in des._exit_to_zones.get(hovered_zcu_node, []):
                    holder = des._zone_lock.get(lock_id)
                    if holder:
                        info_lines.append(f"  exit<-{lock_id} holder=V#{holder.id}")

                # Vehicles waiting at this node's zones
                waiting_here = [v for v in vehicles
                                if v.waiting_at_zcu and
                                hovered_zcu_node in (v.waiting_at_zcu, )]
                # Also check if waiting_at_zcu matches any lock_id containing this node
                for v in vehicles:
                    if v.waiting_at_zcu and hovered_zcu_node in v.waiting_at_zcu \
                       and v not in waiting_here:
                        waiting_here.append(v)

                if waiting_here:
                    info_lines.append(f"  OHTs waiting: {[f'V#{v.id}' for v in waiting_here]}")

                # Draw arrows from ZCU node to lock holders
                drawn_holders = set()
                all_zone_entries = list(des._boundary_to_zones.get(hovered_zcu_node, []))
                for zone in gmap.zcu_zones:
                    if zone.node_id == hovered_zcu_node:
                        lid = f"{zone.node_id}_{zone.kind}"
                        all_zone_entries.append((zone, lid))
                for zone, lock_id in all_zone_entries:
                    holder = des._zone_lock.get(lock_id)
                    if holder and holder.id not in drawn_holders:
                        drawn_holders.add(holder.id)
                        hx2, hy2 = w2s(holder.x, holder.y)
                        # Arrow from ZCU to holder
                        pygame.draw.line(screen, (255, 100, 255), (hsx, hsy), (hx2, hy2), 2)
                        pygame.draw.circle(screen, (255, 100, 255), (hx2, hy2),
                                           max(5, int(scale * 250)), 2)
                        # Label
                        lbl = font_s.render(f"V#{holder.id}", True, (255, 100, 255))
                        screen.blit(lbl, (hx2 + 8, hy2 - 14))

                # Draw info panel near the diamond
                if info_lines:
                    pw = max(len(line) * 7 + 10 for line in info_lines)
                    ph = 14 * len(info_lines) + 8
                    px = hsx + r + 8
                    py = hsy - ph // 2
                    panel = pygame.Surface((pw, ph), pygame.SRCALPHA)
                    panel.fill((20, 20, 35, 220))
                    screen.blit(panel, (px, py))
                    for i, line in enumerate(info_lines):
                        c = (255, 255, 255) if i == 0 else (200, 200, 200)
                        if "FREE" in line:
                            c = (80, 255, 80)
                        elif "V#" in line and "lock=" in line:
                            c = (255, 200, 60)
                        screen.blit(font_s.render(line, True, c), (px + 4, py + 4 + i * 14))

        # ── Stuck holder highlight (before vehicles so it's behind) ─────
        STUCK_C = (255, 0, 255)  # magenta for stuck lock holders
        stuck_holders = set()
        for lid, holder in des._zone_lock.items():
            if holder and holder.state == STOP:
                stuck_holders.add(holder.id)

        # Draw vehicles
        vhl, vhw = 750 / 2, 500 / 2
        for v in vehicles:
            sx, sy = w2s(v.x, v.y)
            if sx < -50 or sx > SW + 50 or sy < -50 or sy > SH + 50:
                continue

            th = v.theta
            ct, st = math.cos(th), math.sin(th)
            corners = [w2s(v.x + lx * ct - ly * st, v.y + lx * st + ly * ct)
                       for lx, ly in [(vhl, vhw), (vhl, -vhw), (-vhl, -vhw), (-vhl, vhw)]]

            clr = STATE_C.get(v.state, v.color)
            is_sel = selected and v.id == selected.id

            # Stuck holder: magenta outline
            if v.id in stuck_holders:
                pygame.draw.polygon(screen, STUCK_C, corners, 3)
            pygame.draw.polygon(screen, clr, corners)
            if is_sel:
                pygame.draw.polygon(screen, (255, 255, 100), corners, 2)

            # Front dot
            fx = v.x + vhl * 0.7 * ct
            fy = v.y + vhl * 0.7 * st
            pygame.draw.circle(screen, (255, 255, 255), w2s(fx, fy), max(2, int(scale * 120)))

            # Vehicle ID
            if scale > 0.015:
                screen.blit(font_s.render(str(v.id), True, (200, 200, 200)), (sx - 4, sy + 8))

        # X markers
        for v in vehicles:
            if v.x_marker_pidx >= 0:
                pidx = v.x_marker_pidx
                stop_off = v.x_marker_offset

                if pidx < len(v.path) - 1:
                    seg = v.gmap.segment_between(v.path[pidx], v.path[pidx + 1])
                    if seg and seg.path_points:
                        sx2, sy2, _ = _interp_path(seg.path_points, max(0, stop_off))
                        ssx, ssy = w2s(sx2, sy2)
                        if 0 <= ssx <= SW and 0 <= ssy <= SH:
                            r = max(3, int(scale * 100))
                            pygame.draw.line(screen, v.color,
                                             (ssx - r, ssy - r), (ssx + r, ssy + r), 2)
                            pygame.draw.line(screen, v.color,
                                             (ssx - r, ssy + r), (ssx + r, ssy - r), 2)
                            vsx, vsy = w2s(v.x, v.y)
                            pygame.draw.line(screen, (*v.color[:3], 100),
                                             (vsx, vsy), (ssx, ssy), 1)
                            if scale > 0.02 and v.x_marker_node:
                                lbl = font_s.render(v.x_marker_node, True, v.color)
                                screen.blit(lbl, (ssx + r + 2, ssy - 6))

        # ── Selected OHT: path visualization ─────────────────────────
        if selected:
            v = selected
            path_lw = max(1, int(scale * 60))
            # Draw forward path (next 30 segments) in vehicle color
            for i in range(v.path_idx, min(v.path_idx + 30, len(v.path) - 1)):
                seg = v.gmap.segment_between(v.path[i], v.path[i + 1])
                if seg and seg.path_points:
                    pts_s = [w2s(p[0], p[1]) for p in seg.path_points]
                    if len(pts_s) >= 2:
                        c = (*v.color[:3],) if hasattr(v, 'color') else (255, 200, 100)
                        pygame.draw.lines(screen, c, False, pts_s, path_lw)

        # Leader arrow for selected
        if selected and selected.leader:
            s1 = w2s(selected.x, selected.y)
            s2 = w2s(selected.leader.x, selected.leader.y)
            pygame.draw.line(screen, (255, 255, 100), s1, s2, 2)
            lx, ly = w2s(selected.leader.x, selected.leader.y)
            pygame.draw.circle(screen, (255, 255, 100), (lx, ly), max(4, int(scale * 200)), 2)

        # ── Info panel ────────────────────────────────────────────────────
        sc = collections.Counter(v.state for v in vehicles)
        # Count ZCU waiters
        zcu_waiting = sum(1 for v in vehicles if v.waiting_at_zcu is not None)
        lines = [
            f"t={sim_time:.1f}s x{sim_speed:.1f} {'PAUSED' if paused else ''} [graph-des-v6 pure-DES]",
            f"Events:{des.event_count}  SimTime:{des.sim_time:.2f}",
            f"A:{sc.get(ACCEL, 0)} C:{sc.get(CRUISE, 0)} D:{sc.get(DECEL, 0)} "
            f"S:{sc.get(STOP, 0)} L:{sc.get(LOADING, 0)} I:{sc.get(IDLE, 0)}",
            f"ZCU-waiting:{zcu_waiting}  Violations:{total_viol} MinGap:{min_gap:.0f}mm",
            f"Vehicles:{len(vehicles)} Segs:{len(gmap.segments)} ZCU:{len(gmap.zcu_nodes)}",
        ]
        if selected:
            v = selected
            lines.append("")
            lines.append(f"#{v.id} {v.state} v={v.vel_at(sim_time):.0f} a={v.acc:.0f}")
            lines.append(f"Gap:{v.gap_to_leader:.0f} PathIdx:{v.path_idx}/{len(v.path)}")
            lines.append(f"Seg:{v.seg_from}->{v.seg_to} off={v.seg_offset:.0f}")
            if v.leader:
                lines.append(f"Leader:#{v.leader.id} {v.leader.state} "
                             f"v={v.leader.vel_at(sim_time):.0f}")
                lines.append(f"  L.seg:{v.leader.seg_from}->{v.leader.seg_to}")
            else:
                lines.append("Leader: None")
            if v.stop_dist is not None:
                lines.append(f"StopDist:{v.stop_dist:.0f}")
            if v.x_marker_node:
                lines.append(f"XMarker@{v.x_marker_node} (ZCU)")
            if v.waiting_at_zcu:
                lines.append(f"Waiting@ZCU:{v.waiting_at_zcu}")
            # Held locks
            held = [lid for lid, h in des._zone_lock.items() if h is v]
            if held:
                lines.append(f"Locks({len(held)}):{','.join(held[:4])}")
                if len(held) > 4:
                    lines.append(f"  +{len(held)-4} more")
            if v.passed_zcu:
                lines.append(f"Passed:{list(v.passed_zcu)[:4]}")
            if v.dest_node:
                lines.append(f"Dest:{v.dest_node} {'REACHED' if v.dest_reached else ''}")

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
        screen.blit(font_s.render("Gap(mm)", True, TEXT), (hx, hy - 2))
        mx_c = max(gap_hist) if any(gap_hist) else 1
        bw2 = hw // 20
        for i, c in enumerate(gap_hist):
            bh = int((c / mx_c) * (hh - 20)) if mx_c > 0 else 0
            bc = ((255, 0, 0) if i * 100 < 750 else
                  (255, 200, 60) if i * 100 < 1150 else
                  (60, 200, 60))
            pygame.draw.rect(screen, bc, (hx + i * bw2, hy + hh - bh, bw2 - 1, bh))

        # Controls hint
        screen.blit(font_s.render("Space:Pause +/-:Speed R:Reset N:Nodes Z:ZCU F:Follow",
                                  True, DIM), (10, SH - 20))

        pygame.display.flip()

    pygame.quit()


if __name__ == '__main__':
    main()
