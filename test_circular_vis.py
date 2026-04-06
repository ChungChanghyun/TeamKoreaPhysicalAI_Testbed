"""
Circular loop DES — pygame visualizer with gap violation monitoring.

Controls:
  Space     : Pause / Resume
  +/-       : Sim speed
  1~9       : Sim speed multiplier
  R         : Reset view
  Click     : Select vehicle (shows gap info)
  V         : Toggle violation markers
  ESC       : Quit
"""

import sys, os, math, random, time, collections

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'TeamKoreaPhysicalAI_Testbed'))

import pygame
from env_oht_des import (OHTMap, OHTAgent, OHTEnvironmentDES, OHTNode, OHTSegment,
                          IDLE, MOVING, FOLLOWING, BLOCKED, DONE)

# ── Reuse from test_circular_des ──────────────────────────────────────────────
from test_circular_des import make_circular_map, CircularTestEnv


# ── Colors ────────────────────────────────────────────────────────────────────
BG        = (18, 18, 24)
TRACK     = (50, 100, 170)
NODE_CLR  = (60, 60, 80)
TEXT      = (200, 200, 200)
DIM       = (120, 120, 150)

STATE_CLR = {
    IDLE:      (100, 100, 100),
    MOVING:    (60, 200, 60),
    FOLLOWING: (255, 200, 60),
    BLOCKED:   (255, 60, 60),
    DONE:      (80, 80, 80),
}

VIOLATION_CLR = (255, 0, 0)
GAP_OK_CLR    = (60, 200, 60)
GAP_WARN_CLR  = (255, 200, 60)


def main():
    # ── Config ────────────────────────────────────────────────────────────
    N_VEHICLES = 25
    N_NODES    = 40
    STOP_PROB  = 0.02
    STOP_DUR   = (2.0, 5.0)
    RADIUS     = 20000

    random.seed(42)
    oht_map, track_len = make_circular_map(n_nodes=N_NODES, radius=RADIUS)
    seg_len = track_len / N_NODES

    env = CircularTestEnv(oht_map, stop_prob=STOP_PROB, stop_duration=STOP_DUR,
                          cross_segment=True)

    node_list = [f"C{i}" for i in range(N_NODES)]
    node_pos = {nid: (n.x, n.y) for nid, n in oht_map.nodes.items()}
    agents = []
    for i in range(N_VEHICLES):
        start_idx = int(i * N_NODES / N_VEHICLES) % N_NODES
        path = []
        for lap in range(1000):
            for j in range(N_NODES):
                path.append(node_list[(start_idx + j + lap * N_NODES) % N_NODES])
        color = pygame.Color(0)
        color.hsla = (i * 360 / N_VEHICLES, 80, 60, 100)
        agent = OHTAgent(i, (color.r, color.g, color.b), path, max_speed=3600)
        agents.append(agent)
        env.add_agent(agent, t_start=0.0)

    # ── Pygame ────────────────────────────────────────────────────────────
    pygame.init()
    SW, SH = 1200, 900
    screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)
    pygame.display.set_caption("Circular DES — Gap Monitor")
    clock = pygame.time.Clock()
    font_s = pygame.font.SysFont("consolas", 12)
    font_m = pygame.font.SysFont("consolas", 14)

    # Camera
    scale = min(SW, SH) / (RADIUS * 2.6)
    cam_x, cam_y = 0.0, 0.0
    init_scale, init_cx, init_cy = scale, cam_x, cam_y

    sim_time = 0.0
    sim_speed = 3.0
    paused = False
    selected = None
    show_violations = True
    dragging = False
    drag_start = None

    # Metrics
    violation_log = []     # [(t, agent_a_id, agent_b_id, gap)]
    max_violations = 500
    total_violations = 0
    min_gap_ever = float('inf')
    gap_histogram = [0] * 20  # 0-100, 100-200, ..., 1900-2000

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
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    cam_x, cam_y, scale = init_cx, init_cy, init_scale
                elif event.key == pygame.K_v:
                    show_violations = not show_violations
                elif event.key == pygame.K_MINUS:
                    sim_speed = max(0.5, sim_speed / 1.5)
                elif event.key == pygame.K_EQUALS:
                    sim_speed = min(50.0, sim_speed * 1.5)
                elif pygame.K_1 <= event.key <= pygame.K_9:
                    sim_speed = float(event.key - pygame.K_0)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3:
                    dragging = True
                    drag_start = event.pos
                elif event.button == 1:
                    mx, my = event.pos
                    best, best_d = None, 25
                    for a in agents:
                        sx, sy = w2s(a.x, a.y)
                        d = math.hypot(sx - mx, sy - my)
                        if d < best_d:
                            best, best_d = a, d
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

        # ── Sim step ──────────────────────────────────────────────────────
        if not paused:
            dt_sim = min(dt_real * sim_speed, 2.0)
            sim_time += dt_sim
            env.step(sim_time)

            # Gap check
            seg_agents = collections.defaultdict(list)
            for a in agents:
                if a.seg:
                    seg_agents[(a.seg.from_id, a.seg.to_id)].append(a)

            for key, ags in seg_agents.items():
                if len(ags) < 2:
                    continue
                positions = sorted([(a, a.pos(sim_time)) for a in ags],
                                   key=lambda x: -x[1])
                for i in range(len(positions) - 1):
                    af, pf = positions[i]
                    ab, pb = positions[i + 1]
                    gap = pf - pb
                    # Histogram
                    bucket = min(int(gap / 100), 19)
                    if bucket >= 0:
                        gap_histogram[bucket] += 1
                    min_gap_ever = min(min_gap_ever, gap)
                    if gap < oht_map.vehicle_length:
                        total_violations += 1
                        if len(violation_log) < max_violations:
                            violation_log.append((sim_time, af.id, ab.id, gap))

        # ── Draw ──────────────────────────────────────────────────────────
        screen.fill(BG)

        # Track segments
        lw = max(1, min(3, int(scale * 80)))
        for (fn, tn), seg in oht_map.segments.items():
            s1 = w2s(*node_pos[fn])
            s2 = w2s(*node_pos[tn])
            pygame.draw.line(screen, TRACK, s1, s2, lw)

        # Nodes
        nr = max(2, int(scale * 60))
        for nid, (nx, ny) in node_pos.items():
            pygame.draw.circle(screen, NODE_CLR, w2s(nx, ny), nr)

        # ── Vehicles + gap bars ───────────────────────────────────────────
        vhl = oht_map.vehicle_length / 2
        vhw = oht_map.vehicle_width / 2

        # Compute per-vehicle gap to leader
        vehicle_gaps = {}
        for key, ags in seg_agents.items():
            if len(ags) < 2:
                continue
            positions = sorted([(a, a.pos(sim_time)) for a in ags],
                               key=lambda x: -x[1])
            for i in range(len(positions) - 1):
                af, pf = positions[i]
                ab, pb = positions[i + 1]
                vehicle_gaps[ab.id] = pf - pb

        for agent in agents:
            sx, sy = w2s(agent.x, agent.y)
            th = agent.theta
            cos_t, sin_t = math.cos(th), math.sin(th)

            corners = []
            for lx, ly in [(vhl, vhw), (vhl, -vhw), (-vhl, -vhw), (-vhl, vhw)]:
                wx = agent.x + lx * cos_t - ly * sin_t
                wy = agent.y + lx * sin_t + ly * cos_t
                corners.append(w2s(wx, wy))

            color = agent.color
            is_sel = selected and agent.id == selected.id

            # Color by state
            if agent.state == BLOCKED:
                color = (255, 60, 60)
            elif agent.state == FOLLOWING:
                color = (255, 200, 60)

            pygame.draw.polygon(screen, color, corners)
            if is_sel:
                pygame.draw.polygon(screen, (255, 255, 100), corners, 2)

            # Direction dot
            fx = agent.x + vhl * 0.7 * cos_t
            fy = agent.y + vhl * 0.7 * sin_t
            pygame.draw.circle(screen, (255, 255, 255), w2s(fx, fy),
                               max(2, int(scale * 120)))

            # Gap indicator bar above vehicle
            gap = vehicle_gaps.get(agent.id)
            if gap is not None and show_violations:
                bar_w = 30
                bar_h = 4
                ratio = min(gap / oht_map.h_min, 2.0)
                fill_w = int(bar_w * min(ratio, 1.0))
                if gap < oht_map.vehicle_length:
                    bar_color = VIOLATION_CLR
                elif gap < oht_map.h_min:
                    bar_color = GAP_WARN_CLR
                else:
                    bar_color = GAP_OK_CLR
                bx, by = sx - bar_w // 2, sy - 18
                pygame.draw.rect(screen, (40, 40, 40), (bx, by, bar_w, bar_h))
                pygame.draw.rect(screen, bar_color, (bx, by, fill_w, bar_h))
                # Gap text
                lbl = font_s.render(f"{gap:.0f}", True, bar_color)
                screen.blit(lbl, (bx + bar_w + 2, by - 3))

            # ID label
            if scale > 0.02:
                lbl = font_s.render(str(agent.id), True, (200, 200, 200))
                screen.blit(lbl, (sx - 4, sy + 8))

        # ── Violation flash markers ───────────────────────────────────────
        if show_violations:
            recent = [v for v in violation_log if sim_time - v[0] < 2.0]
            for vt, va, vb, vgap in recent[-20:]:
                a = agents[vb]
                sx, sy = w2s(a.x, a.y)
                alpha = max(0, 1.0 - (sim_time - vt) / 2.0)
                r = max(8, int(scale * 500))
                pygame.draw.circle(screen, (int(255 * alpha), 0, 0),
                                   (sx, sy), r, 2)

        # ── Info panel ────────────────────────────────────────────────────
        lines = [
            f"Time: {sim_time:.1f}s  Speed: x{sim_speed:.1f}  "
            f"{'PAUSED' if paused else 'RUNNING'}",
            f"Vehicles: {N_VEHICLES}  Events: {env.event_count}  "
            f"Stops: {env.stops_executed}",
        ]
        sc = collections.Counter(a.state for a in agents)
        lines.append(f"MOV:{sc[MOVING]} FOL:{sc[FOLLOWING]} "
                     f"BLK:{sc[BLOCKED]} IDL:{sc[IDLE]}")
        lines.append(f"Violations: {total_violations}  "
                     f"Min gap: {min_gap_ever:.0f}mm  "
                     f"h_min: {oht_map.h_min}mm")

        if selected:
            a = selected
            lines.append("")
            lines.append(f"#{a.id} {a.state}  v={a.vel(sim_time):.0f}mm/s")
            gap = vehicle_gaps.get(a.id)
            if gap is not None:
                lines.append(f"  Gap to leader: {gap:.0f}mm")
            if a.leader:
                lines.append(f"  Leader: #{a.leader.id} ({a.leader.state})")
            if a.state == BLOCKED and a.block_reason:
                lines.append(f"  Block: {a.block_reason}")
                if a.blocked_by:
                    lines.append(f"  By: #{a.blocked_by.id}")

        pw, ph = 380, 18 * len(lines) + 16
        panel = pygame.Surface((pw, ph), pygame.SRCALPHA)
        panel.fill((30, 30, 45, 200))
        screen.blit(panel, (8, 8))
        for i, line in enumerate(lines):
            screen.blit(font_s.render(line, True, TEXT), (14, 14 + i * 18))

        # ── Gap histogram (bottom right) ──────────────────────────────────
        hist_x = SW - 220
        hist_y = SH - 160
        hist_w = 200
        hist_h = 140
        panel2 = pygame.Surface((hist_w + 10, hist_h + 10), pygame.SRCALPHA)
        panel2.fill((30, 30, 45, 180))
        screen.blit(panel2, (hist_x - 5, hist_y - 5))

        screen.blit(font_s.render("Gap Distribution (mm)", True, TEXT),
                    (hist_x, hist_y - 2))
        max_count = max(gap_histogram) if any(gap_histogram) else 1
        bar_w = hist_w // 20
        for i, count in enumerate(gap_histogram):
            bh = int((count / max_count) * (hist_h - 20)) if max_count > 0 else 0
            bx = hist_x + i * bar_w
            by = hist_y + hist_h - bh
            if i * 100 < oht_map.vehicle_length:
                bc = VIOLATION_CLR
            elif i * 100 < oht_map.h_min:
                bc = GAP_WARN_CLR
            else:
                bc = GAP_OK_CLR
            pygame.draw.rect(screen, bc, (bx, by, bar_w - 1, bh))
        # Labels
        screen.blit(font_s.render("0", True, DIM), (hist_x, hist_y + hist_h + 2))
        screen.blit(font_s.render("2000", True, DIM),
                    (hist_x + hist_w - 30, hist_y + hist_h + 2))

        # Help
        help_txt = "Space:Pause +/-:Speed Click:Select V:Violations R:Reset"
        screen.blit(font_s.render(help_txt, True, DIM), (10, SH - 20))

        pygame.display.flip()

    pygame.quit()


if __name__ == '__main__':
    main()
