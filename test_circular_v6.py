"""Circular test using circular_des_v6 engine — pygame visualizer."""
import sys, os, math, random, collections
import pygame

from circular_des_v6 import (
    Track, CircularDESv6, Vehicle,
    IDLE, ACCEL, CRUISE, DECEL, STOP, DWELL,
)

N_VEH = 30
RADIUS = 20000
BG=(18,18,24); TRACK_C=(50,100,170); NODE_C=(60,60,80); TEXT=(200,200,200); DIM=(120,120,150)
STATE_C = {IDLE:(100,100,100), ACCEL:(80,255,80), CRUISE:(60,200,60),
           DECEL:(255,200,60), STOP:(255,60,60), DWELL:(100,60,255)}


def main():
    random.seed(42)

    track = Track(40, RADIUS)
    des = CircularDESv6(track, zcu_interval=10)
    print(f"Track: {track.n} nodes, length={track.length:.0f}mm")
    print(f"ZCU positions: {[f'{p:.0f}' for p in des.zcu_positions]}")

    vehicles = []
    for i in range(N_VEH):
        pos = track.length * i / N_VEH
        c = pygame.Color(0); c.hsla = (i * 360 / N_VEH, 80, 60, 100)
        v = Vehicle(i, track, pos, (c.r, c.g, c.b))
        vehicles.append(v)
        des.add_vehicle(v)

    # Vehicle 0: stop at ZCU node 10
    vehicles[0].dest_pos = track.node_cum_dist[10]
    des.start_all()
    print(f"Spawned {N_VEH} vehicles, v0 dest at node 10")

    pygame.init()
    SW, SH = 1200, 900
    screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)
    pygame.display.set_caption("Circular DES v6 — Trajectory Confirmed")
    clock = pygame.time.Clock()
    font_s = pygame.font.SysFont("consolas", 12)

    scale = min(SW, SH) / (RADIUS * 2.6)
    cam_x = cam_y = 0.0
    sim_time = 0.0; sim_speed = 3.0; paused = False; selected = None
    dragging = False; drag_start = None
    total_viol = 0; min_gap = float('inf'); gap_hist = [0] * 20

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

            v0 = vehicles[0]
            if v0.state == DWELL and sim_time > 120.0:
                print(f"t={sim_time:.1f}s: Vehicle 0 departing from DWELL")
                des.depart(v0.id, sim_time)

            for v in vehicles:
                g = v.gap_to_leader
                if g < float('inf') and g < 200000:
                    b = min(int(g / 100), 19)
                    if b >= 0: gap_hist[b] += 1
                    min_gap = min(min_gap, g)
                    if g < v.length: total_viol += 1

        # ── Draw ──────────────────────────────────────────────────────────
        screen.fill(BG)

        # Draw track
        lw = max(1, min(3, int(scale * 80)))
        for i in range(track.n):
            p1 = track.node_xy[i]; p2 = track.node_xy[(i+1) % track.n]
            pygame.draw.line(screen, TRACK_C, w2s(*p1), w2s(*p2), lw)

        # Draw nodes
        nr = max(2, int(scale * 60))
        for i, xy in enumerate(track.node_xy):
            is_zcu = track.node_cum_dist[i] in des.zcu_positions
            c = (255, 60, 60) if is_zcu else NODE_C
            pygame.draw.circle(screen, c, w2s(*xy), nr + (2 if is_zcu else 0))

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
            if v.state not in (STOP, IDLE, DWELL) and v.x_marker_dist > 10:
                marker_abs = v.pos_at(sim_time) + v.x_marker_dist
                mx2, my2, _ = track.xy_at(marker_abs)
                ssx, ssy = w2s(mx2, my2)
                r = max(3, int(scale * 100))
                pygame.draw.line(screen, v.color, (ssx - r, ssy - r), (ssx + r, ssy + r), 2)
                pygame.draw.line(screen, v.color, (ssx - r, ssy + r), (ssx + r, ssy - r), 2)
                vsx, vsy = w2s(v.x, v.y)
                pygame.draw.line(screen, (*v.color[:3], 100), (vsx, vsy), (ssx, ssy), 1)

        # Leader arrow for selected
        if selected and selected.leader:
            s1, s2 = w2s(selected.x, selected.y), w2s(selected.leader.x, selected.leader.y)
            pygame.draw.line(screen, (255, 255, 100), s1, s2, 2)
            pygame.draw.circle(screen, (255, 255, 100), s2, max(4, int(scale * 200)), 2)

        # Info panel
        sc = collections.Counter(v.state for v in vehicles)
        lines = [
            f"t={sim_time:.1f}s x{sim_speed:.1f} {'PAUSED' if paused else ''} [circular-v6]",
            f"Ev:{des.event_count}",
            f"A:{sc.get(ACCEL, 0)} C:{sc.get(CRUISE, 0)} D:{sc.get(DECEL, 0)} "
            f"S:{sc.get(STOP, 0)} DW:{sc.get(DWELL, 0)} I:{sc.get(IDLE, 0)}",
            f"Violations:{total_viol} MinGap:{min_gap:.0f}mm",
        ]
        if selected:
            v = selected; lines.append("")
            lines.append(f"#{v.id} {v.state} v={v.vel_at(sim_time):.0f} a={v.acc_at(sim_time):.0f}")
            lines.append(f"Gap:{v.gap_to_leader:.0f}")
            lines.append(f"Pos:{v.pos_at(sim_time):.0f}")
            if v.leader:
                lines.append(f"Leader:#{v.leader.id} {v.leader.state} v={v.leader.vel_at(sim_time):.0f}")
            else:
                lines.append("Leader: None")
            marker_dist = v.x_marker_dist
            lines.append(f"Marker dist:{marker_dist:.0f}")
            if v.state == DWELL: lines.append("** DWELL **")

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
