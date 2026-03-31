"""
Circular loop test using the ORIGINAL env_oht_des engine.
Same track, same visualization, but with the proven DES engine.
"""

import sys, os, math, random, collections

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'TeamKoreaPhysicalAI_Testbed'))

import pygame
from env_oht_des import (OHTMap, OHTAgent, OHTEnvironmentDES, OHTNode, OHTSegment,
                          _polyline_length, IDLE, MOVING, FOLLOWING, BLOCKED, DONE)
from test_circular_des import make_circular_map, CircularTestEnv

N_VEHICLES = 25
N_NODES    = 40
RADIUS     = 20000
STOP_PROB  = 0.02
STOP_DUR   = (2.0, 5.0)

BG = (18, 18, 24)
TRACK_CLR = (50, 100, 170)
NODE_CLR = (60, 60, 80)
TEXT = (200, 200, 200)
DIM = (120, 120, 150)
STATE_CLR = {IDLE:(100,100,100), MOVING:(60,200,60), FOLLOWING:(255,200,60),
             BLOCKED:(255,60,60), DONE:(80,80,80)}


def main():
    random.seed(42)
    oht_map, track_len = make_circular_map(n_nodes=N_NODES, radius=RADIUS)
    env = CircularTestEnv(oht_map, stop_prob=STOP_PROB, stop_duration=STOP_DUR,
                          cross_segment=True)

    node_pos = {nid: (n.x, n.y) for nid, n in oht_map.nodes.items()}
    node_list = [f"C{i}" for i in range(N_NODES)]

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

    pygame.init()
    SW, SH = 1200, 900
    screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)
    pygame.display.set_caption("Circular Loop — Original Engine")
    clock = pygame.time.Clock()
    font_s = pygame.font.SysFont("consolas", 12)

    scale = min(SW, SH) / (RADIUS * 2.6)
    cam_x, cam_y = 0.0, 0.0
    sim_time = 0.0
    sim_speed = 3.0
    paused = False
    selected = None
    dragging = False
    drag_start = None

    total_violations = 0
    min_gap = float('inf')
    gap_hist = [0] * 20

    def w2s(wx, wy):
        return (int((wx - cam_x) * scale + SW / 2),
                int(-(wy - cam_y) * scale + SH / 2))

    def s2w(sx, sy):
        return ((sx - SW / 2) / scale + cam_x, -(sy - SH / 2) / scale + cam_y)

    running = True
    while running:
        dt_real = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: running = False
                elif event.key == pygame.K_SPACE: paused = not paused
                elif event.key == pygame.K_r:
                    cam_x, cam_y = 0, 0; scale = min(SW,SH)/(RADIUS*2.6)
                elif event.key == pygame.K_MINUS: sim_speed = max(0.5, sim_speed/1.5)
                elif event.key == pygame.K_EQUALS: sim_speed = min(50, sim_speed*1.5)
                elif pygame.K_1 <= event.key <= pygame.K_9:
                    sim_speed = float(event.key - pygame.K_0)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3: dragging = True; drag_start = event.pos
                elif event.button == 1:
                    mx, my = event.pos; best, bd = None, 25
                    for a in agents:
                        d = math.hypot(w2s(a.x,a.y)[0]-mx, w2s(a.x,a.y)[1]-my)
                        if d < bd: best, bd = a, d
                    selected = best
                elif event.button == 4:
                    mx,my = event.pos; wx,wy = s2w(mx,my); scale *= 1.15
                    cam_x = wx-(mx-SW/2)/scale; cam_y = wy+(my-SH/2)/scale
                elif event.button == 5:
                    mx,my = event.pos; wx,wy = s2w(mx,my); scale /= 1.15
                    cam_x = wx-(mx-SW/2)/scale; cam_y = wy+(my-SH/2)/scale
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3: dragging = False
            elif event.type == pygame.MOUSEMOTION:
                if dragging:
                    dx = event.pos[0]-drag_start[0]; dy = event.pos[1]-drag_start[1]
                    cam_x -= dx/scale; cam_y += dy/scale; drag_start = event.pos
            elif event.type == pygame.VIDEORESIZE:
                SW, SH = event.w, event.h
                screen = pygame.display.set_mode((SW,SH), pygame.RESIZABLE)

        if not paused:
            dt_sim = min(dt_real * sim_speed, 2.0)
            sim_time += dt_sim
            env.step(sim_time)

            # Gap check on same segment
            seg_agents = collections.defaultdict(list)
            for a in agents:
                if a.seg:
                    seg_agents[(a.seg.from_id, a.seg.to_id)].append(a)
            for key, ags in seg_agents.items():
                if len(ags) < 2: continue
                positions = sorted([(a, a.pos(sim_time)) for a in ags], key=lambda x:-x[1])
                for i in range(len(positions)-1):
                    af, pf = positions[i]
                    ab, pb = positions[i+1]
                    gap = pf - pb
                    bucket = min(int(gap/100), 19)
                    if bucket >= 0: gap_hist[bucket] += 1
                    min_gap = min(min_gap, gap)
                    if gap < oht_map.vehicle_length:
                        total_violations += 1

        screen.fill(BG)

        # Track
        lw = max(1, min(3, int(scale*80)))
        for i in range(N_NODES):
            fn, tn = f"C{i}", f"C{(i+1)%N_NODES}"
            pygame.draw.line(screen, TRACK_CLR, w2s(*node_pos[fn]), w2s(*node_pos[tn]), lw)

        # Nodes
        nr = max(2, int(scale*60))
        for nid, xy in node_pos.items():
            pygame.draw.circle(screen, NODE_CLR, w2s(*xy), nr)

        # Vehicles
        vhl, vhw = 750/2, 500/2
        for a in agents:
            sx, sy = w2s(a.x, a.y)
            th = a.theta
            cos_t, sin_t = math.cos(th), math.sin(th)
            corners = [w2s(a.x+lx*cos_t-ly*sin_t, a.y+lx*sin_t+ly*cos_t)
                       for lx, ly in [(vhl,vhw),(vhl,-vhw),(-vhl,-vhw),(-vhl,vhw)]]
            clr = STATE_CLR.get(a.state, a.color)
            is_sel = selected and a.id == selected.id
            pygame.draw.polygon(screen, clr, corners)
            if is_sel:
                pygame.draw.polygon(screen, (255,255,100), corners, 2)
            fx = a.x + vhl*0.7*cos_t; fy = a.y + vhl*0.7*sin_t
            pygame.draw.circle(screen, (255,255,255), w2s(fx,fy), max(2, int(scale*120)))
            if scale > 0.015:
                screen.blit(font_s.render(str(a.id), True, (200,200,200)), (sx-4, sy+8))

        # Leader arrows for BLOCKED/FOLLOWING
        for a in agents:
            if a.state in (BLOCKED, FOLLOWING) and a.leader:
                sx1, sy1 = w2s(a.x, a.y)
                sx2, sy2 = w2s(a.leader.x, a.leader.y)
                clr = (255,60,60) if a.state == BLOCKED else (255,200,60)
                pygame.draw.line(screen, clr, (sx1,sy1), (sx2,sy2), 1)

        if selected and selected.leader:
            sx1, sy1 = w2s(selected.x, selected.y)
            sx2, sy2 = w2s(selected.leader.x, selected.leader.y)
            pygame.draw.line(screen, (255,255,100), (sx1,sy1), (sx2,sy2), 2)

        # Info
        sc = collections.Counter(a.state for a in agents)
        lines = [
            f"t={sim_time:.1f}s  x{sim_speed:.1f}  {'PAUSED' if paused else ''}  "
            f"[Original Engine]",
            f"Events: {env.event_count}  Stops: {env.stops_executed}",
            f"MOV:{sc[MOVING]} FOL:{sc[FOLLOWING]} BLK:{sc[BLOCKED]} IDL:{sc[IDLE]}",
            f"Violations: {total_violations}  Min gap: {min_gap:.0f}mm",
        ]
        if selected:
            a = selected
            lines.append(f"")
            lines.append(f"#{a.id} {a.state}  v={a.vel(sim_time):.0f}mm/s")
            if a.leader:
                lines.append(f"Leader: #{a.leader.id} ({a.leader.state})")
            if a.state == BLOCKED:
                lines.append(f"Block: {a.block_reason}")

        pw = 380; ph = 18*len(lines)+16
        panel = pygame.Surface((pw,ph), pygame.SRCALPHA); panel.fill((30,30,45,200))
        screen.blit(panel, (8,8))
        for i, line in enumerate(lines):
            screen.blit(font_s.render(line, True, TEXT), (14, 14+i*18))

        # Histogram
        hx, hy, hw, hh = SW-220, SH-160, 200, 140
        p2 = pygame.Surface((hw+10,hh+10), pygame.SRCALPHA); p2.fill((30,30,45,180))
        screen.blit(p2, (hx-5,hy-5))
        screen.blit(font_s.render("Gap (mm)", True, TEXT), (hx, hy-2))
        mx_c = max(gap_hist) if any(gap_hist) else 1
        bw = hw//20
        for i, c in enumerate(gap_hist):
            bh = int((c/mx_c)*(hh-20)) if mx_c > 0 else 0
            bc = (255,0,0) if i*100<750 else (255,200,60) if i*100<950 else (60,200,60)
            pygame.draw.rect(screen, bc, (hx+i*bw, hy+hh-bh, bw-1, bh))

        screen.blit(font_s.render("Space:Pause +/-:Speed Click:Select", True, DIM), (10, SH-20))
        pygame.display.flip()

    pygame.quit()

if __name__ == '__main__':
    main()
