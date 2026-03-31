"""
Circular DES v2 — pygame visualizer with S-curve physics.
"""

import sys, os, math, random, time, collections
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'TeamKoreaPhysicalAI_Testbed'))

import pygame
from oht_physics import IDLE, ACCEL, CRUISE, DECEL, STOP
from circular_des_v2 import CircularTrack, CircularDES, Vehicle

# ── Config ────────────────────────────────────────────────────────────────────
N_VEHICLES = 25
N_NODES    = 40
RADIUS     = 20000
STOP_PROB  = 0.02
STOP_DUR   = (2.0, 5.0)
SEED       = 42

# Colors
BG        = (18, 18, 24)
TRACK_CLR = (50, 100, 170)
NODE_CLR  = (60, 60, 80)
TEXT       = (200, 200, 200)
DIM        = (120, 120, 150)
STATE_CLR  = {IDLE: (100,100,100), ACCEL: (80,255,80), CRUISE: (60,200,60),
              DECEL: (255,200,60), STOP: (255,60,60)}


def main():
    random.seed(SEED)
    track = CircularTrack(N_NODES, RADIUS, seg_speed=3600)
    des = CircularDES(track, stop_prob=STOP_PROB, stop_duration=STOP_DUR)

    # Create vehicles evenly spaced
    vehicles = []
    for i in range(N_VEHICLES):
        start_idx = int(i * N_NODES / N_VEHICLES) % N_NODES
        color = pygame.Color(0)
        color.hsla = (i * 360 / N_VEHICLES, 80, 60, 100)
        v = Vehicle(i, (color.r, color.g, color.b), track, f"C{start_idx}")
        vehicles.append(v)
        des.add_vehicle(v)

    des.assign_leaders()

    # ── Pygame ────────────────────────────────────────────────────────────
    pygame.init()
    SW, SH = 1200, 900
    screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)
    pygame.display.set_caption("Circular DES v2 — S-Curve Physics")
    clock = pygame.time.Clock()
    font_s = pygame.font.SysFont("consolas", 12)
    font_m = pygame.font.SysFont("consolas", 14)

    scale = min(SW, SH) / (RADIUS * 2.6)
    cam_x, cam_y = 0.0, 0.0
    sim_time = 0.0
    sim_speed = 3.0
    paused = False
    selected = None
    dragging = False
    drag_start = None

    # Metrics
    total_violations = 0
    min_gap = float('inf')
    gap_hist = [0] * 20

    def w2s(wx, wy):
        return (int((wx - cam_x) * scale + SW / 2),
                int(-(wy - cam_y) * scale + SH / 2))

    def s2w(sx, sy):
        return ((sx - SW / 2) / scale + cam_x,
                -(sy - SH / 2) / scale + cam_y)

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
                    cam_x, cam_y = 0.0, 0.0
                    scale = min(SW, SH) / (RADIUS * 2.6)
                elif event.key == pygame.K_MINUS: sim_speed = max(0.5, sim_speed / 1.5)
                elif event.key == pygame.K_EQUALS: sim_speed = min(50, sim_speed * 1.5)
                elif pygame.K_1 <= event.key <= pygame.K_9:
                    sim_speed = float(event.key - pygame.K_0)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3:
                    dragging = True; drag_start = event.pos
                elif event.button == 1:
                    mx, my = event.pos
                    best, bd = None, 25
                    for v in vehicles:
                        sx, sy = w2s(v.x, v.y)
                        d = math.hypot(sx-mx, sy-my)
                        if d < bd: best, bd = v, d
                    selected = best
                elif event.button == 4:
                    mx, my = event.pos; wx, wy = s2w(mx, my)
                    scale *= 1.15
                    cam_x = wx - (mx - SW/2) / scale
                    cam_y = wy + (my - SH/2) / scale
                elif event.button == 5:
                    mx, my = event.pos; wx, wy = s2w(mx, my)
                    scale /= 1.15
                    cam_x = wx - (mx - SW/2) / scale
                    cam_y = wy + (my - SH/2) / scale
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3: dragging = False
            elif event.type == pygame.MOUSEMOTION:
                if dragging:
                    dx = event.pos[0] - drag_start[0]
                    dy = event.pos[1] - drag_start[1]
                    cam_x -= dx / scale; cam_y += dy / scale
                    drag_start = event.pos
            elif event.type == pygame.VIDEORESIZE:
                SW, SH = event.w, event.h
                screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)

        # ── Sim step ──────────────────────────────────────────────────────
        if not paused:
            dt_sim = min(dt_real * sim_speed, 2.0)
            sim_time += dt_sim
            des.step(sim_time)

            # Gap check
            for v in vehicles:
                gap = v.gap_to_leader
                if gap < float('inf'):
                    bucket = min(int(gap / 100), 19)
                    if bucket >= 0:
                        gap_hist[bucket] += 1
                    min_gap = min(min_gap, gap)
                    if gap < v.length:
                        total_violations += 1

        # ── Draw ──────────────────────────────────────────────────────────
        screen.fill(BG)

        # Track
        lw = max(1, min(3, int(scale * 80)))
        for i in range(N_NODES):
            fn, tn = f"C{i}", f"C{(i+1)%N_NODES}"
            s1, s2 = w2s(*track.node_xy[fn]), w2s(*track.node_xy[tn])
            pygame.draw.line(screen, TRACK_CLR, s1, s2, lw)

        # Nodes
        nr = max(2, int(scale * 60))
        for nid, xy in track.node_xy.items():
            pygame.draw.circle(screen, NODE_CLR, w2s(*xy), nr)

        # Vehicles
        vhl = 750 / 2
        vhw = 500 / 2
        for v in vehicles:
            sx, sy = w2s(v.x, v.y)
            th = v.theta
            cos_t, sin_t = math.cos(th), math.sin(th)
            corners = []
            for lx, ly in [(vhl,vhw),(vhl,-vhw),(-vhl,-vhw),(-vhl,vhw)]:
                wx = v.x + lx*cos_t - ly*sin_t
                wy = v.y + lx*sin_t + ly*cos_t
                corners.append(w2s(wx, wy))

            clr = STATE_CLR.get(v.state, v.color)
            is_sel = selected and v.id == selected.id
            pygame.draw.polygon(screen, clr, corners)
            if is_sel:
                pygame.draw.polygon(screen, (255,255,100), corners, 2)

            # Direction dot
            fx = v.x + vhl*0.7*cos_t
            fy = v.y + vhl*0.7*sin_t
            pygame.draw.circle(screen, (255,255,255), w2s(fx,fy), max(2, int(scale*120)))

            # Gap bar
            gap = v.gap_to_leader
            if gap < float('inf'):
                bar_w = 30
                ratio = min(gap / v.h_min, 2.0)
                fill = int(bar_w * min(ratio, 1.0))
                if gap < v.length:
                    bc = (255,0,0)
                elif gap < v.h_min:
                    bc = (255,200,60)
                else:
                    bc = (60,200,60)
                bx, by = sx - bar_w//2, sy - 18
                pygame.draw.rect(screen, (40,40,40), (bx,by,bar_w,4))
                pygame.draw.rect(screen, bc, (bx,by,fill,4))
                lbl = font_s.render(f"{gap:.0f}", True, bc)
                screen.blit(lbl, (bx+bar_w+2, by-3))

            # ID
            if scale > 0.015:
                screen.blit(font_s.render(str(v.id), True, (200,200,200)),
                            (sx-4, sy+8))

        # ── Leader arrows ─────────────────────────────────────────────────
        # Draw line from each vehicle to its leader
        for v in vehicles:
            if v.leader is None:
                continue
            sx1, sy1 = w2s(v.x, v.y)
            sx2, sy2 = w2s(v.leader.x, v.leader.y)
            gap = v.gap_to_leader
            if gap < v.h_min:
                color = (255, 60, 60)
            elif gap < v.h_min * 2:
                color = (255, 200, 60, 100)
            else:
                color = (40, 80, 40)
            # Only draw if STOP state (blocked)
            if v.state == STOP:
                pygame.draw.line(screen, (255, 80, 80), (sx1, sy1), (sx2, sy2), 2)
                # Arrowhead
                ddx, ddy = sx2-sx1, sy2-sy1
                dist_s = math.hypot(ddx, ddy)
                if dist_s > 10:
                    ux, uy = ddx/dist_s, ddy/dist_s
                    ahx, ahy = sx2-ux*8, sy2-uy*8
                    px, py = -uy*5, ux*5
                    pygame.draw.polygon(screen, (255,80,80), [
                        (sx2,sy2), (int(ahx+px),int(ahy+py)), (int(ahx-px),int(ahy-py))])

        # Selected vehicle: always show leader line
        if selected and selected.leader:
            sx1, sy1 = w2s(selected.x, selected.y)
            sx2, sy2 = w2s(selected.leader.x, selected.leader.y)
            pygame.draw.line(screen, (255, 255, 100), (sx1, sy1), (sx2, sy2), 2)

        # ── Info panel ────────────────────────────────────────────────────
        sc = collections.Counter(v.state for v in vehicles)
        lines = [
            f"t={sim_time:.1f}s  x{sim_speed:.1f}  {'PAUSED' if paused else ''}",
            f"Events: {des.event_count}  Stops: {des.stops_executed}",
            f"A:{sc.get(ACCEL,0)} C:{sc.get(CRUISE,0)} D:{sc.get(DECEL,0)} "
            f"S:{sc.get(STOP,0)} I:{sc.get(IDLE,0)}",
            f"Violations: {total_violations}  Min gap: {min_gap:.0f}mm",
        ]
        if selected:
            v = selected
            lines.append("")
            lines.append(f"#{v.id} {v.state}  v={v.vel:.0f}mm/s")
            lines.append(f"Gap: {v.gap_to_leader:.0f}mm  Safe: {v.safe_gap():.0f}mm")
            if v.leader:
                lines.append(f"Leader: #{v.leader.id} ({v.leader.state})")
            if v.at_node:
                lines.append(f"At node: {v.at_node}")
            elif v.seg_from:
                lines.append(f"Seg: {v.seg_from}->{v.seg_to} pos={v.pos:.0f}/{v.seg_length:.0f}")

        pw = 350
        ph = 18 * len(lines) + 16
        panel = pygame.Surface((pw, ph), pygame.SRCALPHA)
        panel.fill((30,30,45,200))
        screen.blit(panel, (8,8))
        for i, line in enumerate(lines):
            screen.blit(font_s.render(line, True, TEXT), (14, 14+i*18))

        # Histogram
        hx, hy, hw, hh = SW-220, SH-160, 200, 140
        p2 = pygame.Surface((hw+10,hh+10), pygame.SRCALPHA)
        p2.fill((30,30,45,180))
        screen.blit(p2, (hx-5,hy-5))
        screen.blit(font_s.render("Gap (mm)", True, TEXT), (hx, hy-2))
        mx_c = max(gap_hist) if any(gap_hist) else 1
        bw = hw // 20
        for i, c in enumerate(gap_hist):
            bh = int((c/mx_c)*(hh-20)) if mx_c > 0 else 0
            if i*100 < 750: bc = (255,0,0)
            elif i*100 < 950: bc = (255,200,60)
            else: bc = (60,200,60)
            pygame.draw.rect(screen, bc, (hx+i*bw, hy+hh-bh, bw-1, bh))

        screen.blit(font_s.render("Space:Pause +/-:Speed Click:Select", True, DIM),
                    (10, SH-20))
        pygame.display.flip()

    pygame.quit()


if __name__ == '__main__':
    main()
