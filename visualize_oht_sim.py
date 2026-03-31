"""
LargeOHT Simulation Visualizer
- OHT DES engine integrated with pygame rendering
- Vehicles drive along the layout with collision avoidance

Controls:
  RClick-drag / Arrow keys : Pan
  Mouse wheel              : Zoom
  Left-click               : Select vehicle / node
  Space                    : Pause / Resume
  +/-                      : Speed up / slow down sim
  1~9                      : Set sim speed multiplier
  A                        : Add 5 more vehicles
  R                        : Reset view
  G                        : Toggle grid
  N                        : Toggle node dots
  ESC                      : Quit
"""

import sys, os, math, json, random, collections

# Add engine path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'TeamKoreaPhysicalAI_Testbed'))

import pygame
from env_oht_des import (OHTMap, OHTAgent, OHTEnvironmentDES, OHTNode, OHTSegment,
                          IDLE, MOVING, FOLLOWING, BLOCKED, DONE,
                          _polyline_length)

# ── Custom map loader (LargeOHT has no area filter, no vehicleModel) ─────────

class LargeOHTMap(OHTMap):
    """Override to load LargeOHT.json which has area='LargeOHT' and no vehicleModels."""

    def __init__(self, json_path: str):
        # bypass parent __init__ entirely
        from env_oht_des import _build_path_points, _polyline_length

        with open(json_path, 'r', encoding='utf-8') as f:
            d = json.load(f)

        # Load ALL nodes (no area filter)
        self.nodes = {
            n['id']: OHTNode(n['id'], n['x'], n['y'])
            for n in d['nodes']
        }

        self.segments = {}
        self.adj = {nid: [] for nid in self.nodes}

        for s in d['segments']:
            fn, tn = s['startNodeId'], s['endNodeId']
            if fn in self.nodes and tn in self.nodes:
                n1, n2 = self.nodes[fn], self.nodes[tn]
                start_xy = (n1.x, n1.y)
                parts = s.get('parts')
                if parts:
                    path_pts = _build_path_points(parts, start_xy)
                    length = _polyline_length(path_pts)
                else:
                    path_pts = [start_xy, (n2.x, n2.y)]
                    length = math.hypot(n2.x - n1.x, n2.y - n1.y)
                if length < 0.1:
                    continue
                seg = OHTSegment(s['id'], fn, tn, length,
                                 s.get('speed', 2000), path_pts)
                self.segments[(fn, tn)] = seg
                self.adj[fn].append(tn)

        # ── Split curve segments at midpoint ───────────────────────────────
        # Adds a midpoint node on each curve so that:
        # 1. Vehicle at midpoint is visible as node_blocker to straight vehicles
        # 2. Only one vehicle per half-curve (front-detection handles spacing)
        self._split_curves_at_midpoint(_build_path_points, _polyline_length)

        # ── Compute main loop (largest SCC) ───────────────────────────────
        self.main_loop = self._compute_main_scc()

        # Vehicle model from JSON or defaults
        oht_vm = next((v for v in d.get('vehicleModels', []) if v['id'] == 'OHT'), {})
        dim = oht_vm.get('dimension', {})
        self.vehicle_length = dim.get('length', 750)
        self.vehicle_width  = dim.get('width',  500)
        self.h_min = self.vehicle_length + 200
        self.accel = oht_vm.get('acceleration', 500)
        self.decel = oht_vm.get('deceleration', 500)
        max_v = max((s.get('speed', 2000) for s in d['segments']), default=2000)
        self.max_brake = max_v ** 2 / (2.0 * self.decel) if self.decel > 0 else 10000

        # Port nodes
        self.port_nodes = {
            p['nodeId'] for p in d.get('ports', [])
            if p.get('nodeId') in self.nodes
        }

        # ── Auto-detect ZCU zones at merge nodes (in-degree >= 2) ─────────
        self._build_zcu_zones()

        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        pad = self.vehicle_length
        self.bbox = (min(xs)-pad, min(ys)-pad, max(xs)+pad, max(ys)+pad)

    def _is_uturn(self, fn, tn):
        """Check if a curve segment is a U-turn by comparing incoming/outgoing
        straight segment directions. U-turn = direction reversal (angle > 120)."""
        # Get incoming straight direction to fn
        in_dirs = []
        for pred in self._radj_cache.get(fn, []):
            seg = self.segments.get((pred, fn))
            if seg and len(seg.path_points) <= 2:  # straight
                pn = self.nodes[pred]
                n = self.nodes[fn]
                dx, dy = n.x - pn.x, n.y - pn.y
                l = math.hypot(dx, dy)
                if l > 1:
                    in_dirs.append((dx/l, dy/l))
        # Get outgoing straight direction from tn
        out_dirs = []
        for succ in self.adj.get(tn, []):
            seg = self.segments.get((tn, succ))
            if seg and len(seg.path_points) <= 2:
                n = self.nodes[tn]
                sn = self.nodes[succ]
                dx, dy = sn.x - n.x, sn.y - n.y
                l = math.hypot(dx, dy)
                if l > 1:
                    out_dirs.append((dx/l, dy/l))
        if not in_dirs or not out_dirs:
            return False
        dot = in_dirs[0][0]*out_dirs[0][0] + in_dirs[0][1]*out_dirs[0][1]
        angle = math.degrees(math.acos(max(-1, min(1, dot))))
        return angle > 120

    def _split_curves_at_midpoint(self, _build_path_points, _polyline_length):
        """Split U-turn curve segments at midpoint. S-curves are NOT split
        because there's no physical space to stop mid-shortcut."""
        # Build reverse adjacency for direction detection
        self._radj_cache = collections.defaultdict(list)
        for (fn, tn) in self.segments:
            self._radj_cache[tn].append(fn)

        to_split = []
        for (fn, tn), seg in list(self.segments.items()):
            if seg.path_points and len(seg.path_points) > 2:
                if self._is_uturn(fn, tn):
                    to_split.append((fn, tn, seg))

        mid_id_counter = 0
        for fn, tn, seg in to_split:
            pts = seg.path_points
            total_len = _polyline_length(pts)
            if total_len < 100:
                continue  # too short to split

            # Find midpoint along polyline
            half = total_len / 2.0
            accum = 0.0
            mid_x, mid_y = pts[-1]
            split_idx = len(pts) - 1
            for i in range(len(pts) - 1):
                dx = pts[i+1][0] - pts[i][0]
                dy = pts[i+1][1] - pts[i][1]
                seg_len = math.hypot(dx, dy)
                if accum + seg_len >= half:
                    t = (half - accum) / seg_len if seg_len > 0 else 0.5
                    mid_x = pts[i][0] + dx * t
                    mid_y = pts[i][1] + dy * t
                    split_idx = i + 1
                    break
                accum += seg_len

            # Create midpoint node
            mid_nid = f"_curve_mid_{mid_id_counter}"
            mid_id_counter += 1
            self.nodes[mid_nid] = OHTNode(mid_nid, mid_x, mid_y)
            self.adj[mid_nid] = []

            # Split path_points
            pts_first = list(pts[:split_idx]) + [(mid_x, mid_y)]
            pts_second = [(mid_x, mid_y)] + list(pts[split_idx:])

            len_first = _polyline_length(pts_first)
            len_second = _polyline_length(pts_second)

            # Remove original segment
            del self.segments[(fn, tn)]
            self.adj[fn].remove(tn)

            # Create two new segments
            seg1 = OHTSegment(f"{seg.id}_a", fn, mid_nid, len_first,
                              seg.max_speed, pts_first)
            seg2 = OHTSegment(f"{seg.id}_b", mid_nid, tn, len_second,
                              seg.max_speed, pts_second)

            self.segments[(fn, mid_nid)] = seg1
            self.segments[(mid_nid, tn)] = seg2
            self.adj[fn].append(mid_nid)
            self.adj[mid_nid].append(tn)

            # Link both halves as a curve group (for occupancy check)
            gid = f"_cg_{mid_id_counter-1}"
            seg1._curve_group = gid
            seg2._curve_group = gid

        # Build curve_group → [segment_keys] mapping
        self.curve_groups = collections.defaultdict(list)
        for (fn, tn), seg in self.segments.items():
            gid = getattr(seg, '_curve_group', None)
            if gid:
                self.curve_groups[gid].append((fn, tn))

        if to_split:
            print(f"Curve midpoints added: {len(to_split)} curves split -> "
                  f"{len(self.nodes)} nodes, {len(self.segments)} segments")

    def _build_zcu_zones(self):
        """Build bypass zones at merge/diverge points.

        A bypass zone protects a curve segment that connects a diverge to a merge.
        Rules:
        - Curve segments: one vehicle at a time. Lock acquired at entry,
          released when vehicle arrives at merge node.
        - Straight segments to same merge: free queue, BUT blocked while
          any curve segment in the same zone is occupied.
        - Straight segments from same diverge: same rule from diverge side.

        Implementation: each curve segment becomes a ZCU zone.
        Entry = the curve segment itself + all straight segments to the same merge
                + all straight segments from the same diverge.
        This way, when a curve is occupied, all related entries are blocked.
        When a straight enters, it acquires the lock — but same-segment
        followers are handled by the engine's 'holder is not agent' check
        plus segment queue logic.

        Actually, simplest correct approach:
        - Zone entry = ALL segments into the merge node (curve + straight)
        - Lock held by curve segment occupant only
        - Straight entries: skip lock check (free queue)
        - Curve entries: acquire/release lock normally

        We mark segments as 'bypass_straight' or 'bypass_curve' for the engine.
        """
        from env_oht_des import ZCUZone

        radj = collections.defaultdict(list)
        for (fn, tn) in self.segments:
            radj[tn].append(fn)

        present = set(self.segments.keys())
        self.zcu_zones = []
        self._entry_zcu = {}
        self._exit_zcu = {}
        self.zcu_node_ids = set()

        # bypass_curve_segs: set of (fn, tn) that are curve entries
        self.bypass_curve_segs = set()
        # bypass_straight_segs: set of (fn, tn) that are straight entries
        #   to a merge that also has a curve entry
        self.bypass_straight_segs = set()

        for nid in self.nodes:
            preds = radj[nid]
            if len(preds) < 2:
                continue

            entries = [(p, nid) for p in preds if (p, nid) in present]
            if len(entries) < 2:
                continue

            # Classify entries
            curve_entries = []
            straight_entries = []
            for key in entries:
                seg = self.segments[key]
                if seg.path_points and len(seg.path_points) > 2:
                    curve_entries.append(key)
                else:
                    straight_entries.append(key)

            # Create zone with ALL entries
            all_entries = frozenset(entries)
            zone = ZCUZone(f'ZCU_{nid}', all_entries, all_entries)
            self.zcu_zones.append(zone)
            for k in all_entries:
                self._entry_zcu[k] = zone
                self._exit_zcu[k] = zone
            self.zcu_node_ids.add(nid)

            # Mark curve vs straight
            for k in curve_entries:
                self.bypass_curve_segs.add(k)
            if curve_entries:
                # Only mark straights as bypass if there's a curve counterpart
                for k in straight_entries:
                    self.bypass_straight_segs.add(k)

        # ── Diverge protection (NOT ZCU — no lock, occupancy-based) ────────
        # For each diverge node, classify outgoing segments as curve/straight.
        # Engine will check: if a curve exit is occupied, block other exits.
        self.diverge_curve_exits = set()   # (fn, tn) outgoing curve segs from diverge
        self.diverge_straight_exits = set()
        self.diverge_nodes = set()
        for nid in self.nodes:
            succs = self.adj.get(nid, [])
            if len(succs) < 2:
                continue
            exits = [(nid, s) for s in succs if (nid, s) in present]
            if len(exits) < 2:
                continue
            curve_exits = [k for k in exits
                           if self.segments[k].path_points and len(self.segments[k].path_points) > 2]
            straight_exits = [k for k in exits
                              if not (self.segments[k].path_points and len(self.segments[k].path_points) > 2)]
            if curve_exits and straight_exits:
                self.diverge_nodes.add(nid)
                for k in curve_exits:
                    self.diverge_curve_exits.add(k)
                for k in straight_exits:
                    self.diverge_straight_exits.add(k)

        print(f"ZCU zones: {len(self.zcu_zones)} merge "
              f"(curve entries: {len(self.bypass_curve_segs)}, "
              f"straight entries: {len(self.bypass_straight_segs)})")
        print(f"Diverge protection: {len(self.diverge_nodes)} nodes "
              f"(curve exits: {len(self.diverge_curve_exits)}, "
              f"straight exits: {len(self.diverge_straight_exits)})")

    def _compute_main_scc(self) -> set:
        """Kosaraju's algorithm to find the largest strongly connected component."""
        # Pass 1: forward DFS, record finish order
        order = []
        vis = set()
        for start in self.nodes:
            if start in vis:
                continue
            stack = [(start, False)]
            while stack:
                node, done = stack.pop()
                if done:
                    order.append(node)
                    continue
                if node in vis:
                    continue
                vis.add(node)
                stack.append((node, True))
                for nb in self.adj.get(node, []):
                    if nb not in vis:
                        stack.append((nb, False))

        # Build reverse adjacency
        radj = collections.defaultdict(list)
        for (fn, tn) in self.segments:
            radj[tn].append(fn)

        # Pass 2: reverse DFS in reverse finish order
        vis2 = set()
        largest = []
        for node in reversed(order):
            if node in vis2:
                continue
            comp = []
            stack = [node]
            while stack:
                n = stack.pop()
                if n in vis2:
                    continue
                vis2.add(n)
                comp.append(n)
                for nb in radj.get(n, []):
                    if nb not in vis2:
                        stack.append(nb)
            if len(comp) > len(largest):
                largest = comp

        return set(largest)


def random_safe_path(oht_map, start_node, length=200):
    """Generate a random walk staying within the main loop (SCC).
    Only follows edges whose target is in the main loop, preventing
    vehicles from entering dead-end branches they can't exit."""
    main = oht_map.main_loop
    path = [start_node]
    cur = start_node

    for _ in range(length):
        neighbors = oht_map.adj.get(cur, [])
        # Only allow neighbors in main loop
        safe = [n for n in neighbors if n in main]
        if not safe:
            break
        # Prefer unvisited for variety
        visited = set(path[-20:])  # only recent to allow revisits on loops
        unvisited = [n for n in safe if n not in visited]
        nxt = random.choice(unvisited) if unvisited else random.choice(safe)
        path.append(nxt)
        cur = nxt

    return path


# ── Colors ────────────────────────────────────────────────────────────────────

BG_COLOR       = (18, 18, 24)
GRID_COLOR     = (35, 35, 50)
GRID_MAJOR     = (50, 50, 70)
PATH_COLOR     = (50, 100, 170)
NODE_COLOR     = (60, 60, 80)
TEXT_COLOR      = (200, 200, 200)
DIM_TEXT        = (120, 120, 150)
ZCU_ENTRY_COLOR = (255, 100, 50)   # orange-red for ZCU entry segments
ZCU_NODE_COLOR  = (255, 60, 60)    # red diamond for merge nodes

VEHICLE_COLORS = [
    (255, 80,  80),   # red
    (80,  200, 255),  # cyan
    (255, 200, 60),   # yellow
    (100, 255, 130),  # green
    (255, 130, 200),  # pink
    (200, 130, 255),  # purple
    (255, 160, 80),   # orange
    (130, 255, 220),  # mint
    (180, 180, 255),  # lavender
    (255, 255, 150),  # light yellow
]

STATE_COLORS = {
    IDLE:      (100, 100, 100),
    MOVING:    (60, 200, 60),
    FOLLOWING: (255, 200, 60),
    BLOCKED:   (255, 60, 60),
    DONE:      (80, 80, 80),
}


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    pygame.init()
    disp = pygame.display.Info()
    SW = min(1600, disp.current_w - 100)
    SH = min(900, disp.current_h - 100)
    screen = pygame.display.set_mode((SW, SH), pygame.RESIZABLE)
    pygame.display.set_caption("LargeOHT Simulation")
    clock = pygame.time.Clock()
    font_s = pygame.font.SysFont("consolas", 12)
    font_m = pygame.font.SysFont("consolas", 14)

    # ── Load map ──────────────────────────────────────────────────────────
    map_path = os.path.join(os.path.dirname(__file__), "oht.large.map.json")
    oht_map = LargeOHTMap(map_path)
    print(f"Loaded: {len(oht_map.nodes)} nodes, {len(oht_map.segments)} segments")

    # Precompute node positions
    node_pos = {nid: (n.x, n.y) for nid, n in oht_map.nodes.items()}

    # Build ZCU merge node → zone lookup
    zcu_by_node = {}
    for zone in oht_map.zcu_zones:
        for (_, tn) in zone.entry_segs:
            zcu_by_node[tn] = zone

    # Find good start candidates: main loop nodes with outgoing edges
    main_loop = oht_map.main_loop
    degree = collections.Counter()
    for fn, tn in oht_map.segments:
        degree[fn] += 1
        degree[tn] += 1
    good_starts = [nid for nid in main_loop if degree[nid] >= 2 and oht_map.adj.get(nid)]
    if not good_starts:
        good_starts = [nid for nid in main_loop if oht_map.adj.get(nid)]
    print(f"Main loop: {len(main_loop)} nodes, dead-end branches: {len(oht_map.nodes) - len(main_loop)} nodes")

    # ── Create environment & agents ───────────────────────────────────────
    env = OHTEnvironmentDES(oht_map, cross_segment=True)

    n_vehicles = 200
    agents = []

    def spawn_vehicles(count, t_start=0.0):
        nonlocal agents
        used_nodes = set()
        for agent in agents:
            nearby = oht_map.nearby_nodes(agent.cur_node, oht_map.h_min * 2)
            used_nodes |= nearby

        for i in range(count):
            aid = len(agents)
            # Find a start node not too close to existing vehicles
            candidates = [n for n in good_starts if n not in used_nodes]
            if not candidates:
                candidates = good_starts
            start = random.choice(candidates)

            # Generate a long random path within main loop only
            path = random_safe_path(oht_map, start, length=300)

            color = VEHICLE_COLORS[aid % len(VEHICLE_COLORS)]
            agent = OHTAgent(aid, color, path, max_speed=3600)
            agents.append(agent)
            env.add_agent(agent, t_start=t_start)

            # Mark nearby nodes as used
            nearby = oht_map.nearby_nodes(start, oht_map.h_min * 3)
            used_nodes |= nearby

    spawn_vehicles(n_vehicles)
    print(f"Spawned {n_vehicles} vehicles")

    # ── Camera ────────────────────────────────────────────────────────────
    xs = [n.x for n in oht_map.nodes.values()]
    ys = [n.y for n in oht_map.nodes.values()]
    world_cx = (min(xs) + max(xs)) / 2
    world_cy = (min(ys) + max(ys)) / 2
    world_w = max(xs) - min(xs)
    world_h = max(ys) - min(ys)
    scale = min(SW / (world_w * 1.1), SH / (world_h * 1.1))
    cam_x, cam_y = world_cx, world_cy
    init_scale, init_cx, init_cy = scale, cam_x, cam_y

    # Toggle states
    show_grid = True
    show_nodes = True
    show_zcu = True
    paused = False
    hovered_zcu = None  # ZCUZone currently under mouse
    sim_speed = 3.0
    sim_time = 0.0

    # Interaction
    dragging = False
    drag_start = None
    selected_agent = None

    def w2s(wx, wy):
        return (int((wx - cam_x) * scale + SW / 2),
                int(-(wy - cam_y) * scale + SH / 2))

    def s2w(sx, sy):
        return ((sx - SW / 2) / scale + cam_x,
                -(sy - SH / 2) / scale + cam_y)

    # ── Path reassignment for DONE agents ─────────────────────────────────

    def reassign_done_agents(t):
        for agent in agents:
            if agent.state == DONE:
                start = agent.cur_node
                if not oht_map.adj.get(start):
                    continue
                path = random_safe_path(oht_map, start, length=300)
                env.reassign(agent, path, t)

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
                    selected_agent = None
                elif event.key == pygame.K_g:
                    show_grid = not show_grid
                elif event.key == pygame.K_n:
                    show_nodes = not show_nodes
                elif event.key == pygame.K_z:
                    show_zcu = not show_zcu
                elif event.key == pygame.K_a:
                    spawn_vehicles(5, sim_time)
                    print(f"Total vehicles: {len(agents)}")
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
                    # Find closest vehicle
                    mx, my = event.pos
                    best, best_d = None, 30
                    for a in agents:
                        sx, sy = w2s(a.x, a.y)
                        d = math.hypot(sx - mx, sy - my)
                        if d < best_d:
                            best, best_d = a, d
                    selected_agent = best
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
                # Detect ZCU hover (find nearest merge node within 20 screen pixels)
                if show_zcu and not dragging:
                    mx, my = event.pos
                    hovered_zcu = None
                    best_dist = 20  # pixels
                    for nid, zone in zcu_by_node.items():
                        nx, ny = node_pos[nid]
                        sx, sy = w2s(nx, ny)
                        d = math.hypot(sx - mx, sy - my)
                        if d < best_dist:
                            best_dist = d
                            hovered_zcu = zone

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

        # ── Sim step ──────────────────────────────────────────────────────
        if not paused:
            import time as _time
            # Cap sim advance to prevent event storms from freezing UI
            dt_sim = min(dt_real * sim_speed, 2.0)  # max 2s sim per frame
            sim_time += dt_sim
            ev_before = env.event_count
            t_start = _time.perf_counter()
            env.step(sim_time)
            t_elapsed = _time.perf_counter() - t_start
            ev_processed = env.event_count - ev_before
            # Log if step took too long
            if t_elapsed > 0.3:
                heap_size = len(env._heap)
                print(f"[SLOW] step={t_elapsed:.2f}s  events={ev_processed}  "
                      f"heap={heap_size}  sim_t={sim_time:.1f}  dt_sim={dt_sim:.2f}")
                sc = collections.Counter(a.state for a in agents)
                print(f"  states: {dict(sc)}")
            # Reassign DONE agents periodically
            reassign_done_agents(sim_time)

            # Detect and log deadlocks (BLOCKED chains that form cycles)
            if int(sim_time * 2) % 20 == 0:  # check every 10s sim time
                blocked = [a for a in agents if a.state == BLOCKED]
                if len(blocked) > 5:
                    # Check for cycles
                    for ba in blocked:
                        visited = set()
                        cur = ba
                        chain = []
                        while cur and cur.blocked_by and cur.id not in visited:
                            visited.add(cur.id)
                            chain.append(f"#{cur.id}({cur.block_reason})")
                            cur = cur.blocked_by
                        if cur and cur.id in visited:
                            chain.append(f"#{cur.id}(CYCLE)")
                            print(f"[DEADLOCK t={sim_time:.1f}] {' -> '.join(chain)}")
                            break  # one is enough

        # ── Follow selected agent ─────────────────────────────────────────
        if selected_agent and keys[pygame.K_f]:
            cam_x, cam_y = selected_agent.x, selected_agent.y

        # ── Draw ──────────────────────────────────────────────────────────
        screen.fill(BG_COLOR)

        # Visible world rect for culling
        vl, vt = s2w(0, 0)
        vr, vb = s2w(SW, SH)
        vis_x0 = min(vl, vr) - 2000
        vis_x1 = max(vl, vr) + 2000
        vis_y0 = min(vt, vb) - 2000
        vis_y1 = max(vt, vb) + 2000

        # Grid
        if show_grid:
            visible_w = SW / scale
            nice = [100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000]
            spacing = nice[0]
            for n in nice:
                if n >= visible_w / 15:
                    spacing = n
                    break
            else:
                spacing = nice[-1]

            sx_start = int(vis_x0 // spacing) * spacing
            sy_start = int(vis_y0 // spacing) * spacing
            x = sx_start
            while x <= vis_x1:
                px, _ = w2s(x, 0)
                c = GRID_MAJOR if x % (spacing * 5) == 0 else GRID_COLOR
                pygame.draw.line(screen, c, (px, 0), (px, SH), 1)
                x += spacing
            y = sy_start
            while y <= vis_y1:
                _, py_ = w2s(0, y)
                c = GRID_MAJOR if y % (spacing * 5) == 0 else GRID_COLOR
                pygame.draw.line(screen, c, (0, py_), (SW, py_), 1)
                y += spacing

        # Segments (draw polyline for curved paths)
        lw = max(1, min(3, int(scale * 80)))
        for (fn, tn), seg in oht_map.segments.items():
            pts = seg.path_points
            if not pts:
                continue
            # Quick bounding box cull using first/last points
            xs_seg = [p[0] for p in pts]
            ys_seg = [p[1] for p in pts]
            if max(xs_seg) < vis_x0 or min(xs_seg) > vis_x1:
                continue
            if max(ys_seg) < vis_y0 or min(ys_seg) > vis_y1:
                continue
            screen_pts = [w2s(p[0], p[1]) for p in pts]
            if len(screen_pts) >= 2:
                pygame.draw.lines(screen, PATH_COLOR, False, screen_pts, lw)

        # ZCU zones — show merge node markers; highlight hovered zone's edges
        if show_zcu:
            # Draw small markers on merge nodes (red) and diverge nodes (orange)
            mr = max(3, int(scale * 120))
            for nid in zcu_by_node:
                nx, ny = node_pos[nid]
                if nx < vis_x0 or nx > vis_x1 or ny < vis_y0 or ny > vis_y1:
                    continue
                sx, sy = w2s(nx, ny)
                pygame.draw.circle(screen, ZCU_NODE_COLOR, (sx, sy), mr)
            for nid in getattr(oht_map, 'diverge_nodes', set()):
                if nid not in node_pos:
                    continue
                nx, ny = node_pos[nid]
                if nx < vis_x0 or nx > vis_x1 or ny < vis_y0 or ny > vis_y1:
                    continue
                sx, sy = w2s(nx, ny)
                pygame.draw.circle(screen, (255, 180, 60), (sx, sy), mr)

            # Highlight hovered zone's entry/exit segments
            if hovered_zcu:
                zcu_lw = max(3, min(6, int(scale * 200)))
                # Entry segments in orange-red
                for (fn, tn) in hovered_zcu.entry_segs:
                    seg = oht_map.segments.get((fn, tn))
                    if seg and seg.path_points:
                        pts_s = [w2s(p[0], p[1]) for p in seg.path_points]
                        if len(pts_s) >= 2:
                            pygame.draw.lines(screen, ZCU_ENTRY_COLOR, False, pts_s, zcu_lw)
                    # Draw start node of entry
                    if fn in node_pos:
                        sx, sy = w2s(*node_pos[fn])
                        pygame.draw.circle(screen, (255, 200, 60), (sx, sy), mr + 2, 2)
                # Exit segments in cyan
                for (fn, tn) in hovered_zcu.exit_segs:
                    seg = oht_map.segments.get((fn, tn))
                    if seg and seg.path_points:
                        pts_s = [w2s(p[0], p[1]) for p in seg.path_points]
                        if len(pts_s) >= 2:
                            pygame.draw.lines(screen, (80, 220, 255), False, pts_s, zcu_lw)
                # Merge node diamond
                for (_, tn) in hovered_zcu.entry_segs:
                    if tn in node_pos:
                        sx, sy = w2s(*node_pos[tn])
                        r = max(6, int(scale * 300))
                        diamond = [(sx, sy-r), (sx+r, sy), (sx, sy+r), (sx-r, sy)]
                        pygame.draw.polygon(screen, ZCU_NODE_COLOR, diamond)
                        pygame.draw.polygon(screen, (255,255,255), diamond, 2)
                        # Label
                        lbl = font_s.render(hovered_zcu.id, True, (255,255,255))
                        screen.blit(lbl, (sx + r + 4, sy - 8))
                    break  # only one merge node per zone

        # Nodes
        if show_nodes:
            nr = max(1, min(3, int(scale * 40)))
            for nid, (nx, ny) in node_pos.items():
                if nx < vis_x0 or nx > vis_x1 or ny < vis_y0 or ny > vis_y1:
                    continue
                pygame.draw.circle(screen, NODE_COLOR, w2s(nx, ny), nr)

        # ── Vehicles ──────────────────────────────────────────────────────
        veh_half_l = oht_map.vehicle_length / 2
        veh_half_w = oht_map.vehicle_width / 2

        for agent in agents:
            if agent.state == DONE and agent.x == 0 and agent.y == 0:
                continue
            ax, ay = agent.x, agent.y
            if ax < vis_x0 or ax > vis_x1 or ay < vis_y0 or ay > vis_y1:
                continue

            sx, sy = w2s(ax, ay)
            th = agent.theta

            # Draw vehicle as rotated rectangle
            cos_t, sin_t = math.cos(th), math.sin(th)
            # Corners in world space (relative to center)
            corners_local = [
                ( veh_half_l,  veh_half_w),
                ( veh_half_l, -veh_half_w),
                (-veh_half_l, -veh_half_w),
                (-veh_half_l,  veh_half_w),
            ]
            screen_corners = []
            for lx, ly in corners_local:
                wx = ax + lx * cos_t - ly * sin_t
                wy = ay + lx * sin_t + ly * cos_t
                screen_corners.append(w2s(wx, wy))

            # Color by agent color, brightness by state
            color = agent.color
            if agent.state == BLOCKED:
                color = (min(255, color[0] + 60), max(0, color[1] - 40), max(0, color[2] - 40))
            elif agent.state == FOLLOWING:
                color = (color[0], min(255, color[1] + 30), color[2])

            # Selected highlight
            is_selected = (selected_agent and agent.id == selected_agent.id)
            if is_selected:
                # Draw selection glow
                pygame.draw.polygon(screen, (255, 255, 100), screen_corners, 3)

            pygame.draw.polygon(screen, color, screen_corners)

            # Direction indicator (front dot)
            fx = ax + veh_half_l * 0.7 * cos_t
            fy = ay + veh_half_l * 0.7 * sin_t
            fsx, fsy = w2s(fx, fy)
            dr = max(2, int(scale * 150))
            pygame.draw.circle(screen, (255, 255, 255), (fsx, fsy), dr)

            # State indicator dot (small)
            state_c = STATE_COLORS.get(agent.state, (100, 100, 100))
            pygame.draw.circle(screen, state_c, (sx, sy), max(2, int(scale * 100)))

            # ID label when zoomed in
            if scale > 0.03:
                label = font_s.render(str(agent.id), True, (255, 255, 255))
                screen.blit(label, (sx + 8, sy - 6))

        # ── Blocking chain arrows ─────────────────────────────────────────
        # For ALL BLOCKED vehicles, draw a small arrow to their blocker
        for ba in agents:
            if ba.state != BLOCKED or not ba.blocked_by:
                continue
            blocker = ba.blocked_by
            # Skip if blocker is no longer blocking (stale)
            if blocker.state not in (BLOCKED, IDLE, DONE) and ba.block_reason != 'zcu':
                continue
            sx1, sy1 = w2s(ba.x, ba.y)
            sx2, sy2 = w2s(blocker.x, blocker.y)
            if abs(sx1) > SW*2 or abs(sy1) > SH*2:
                continue
            pygame.draw.line(screen, (255, 80, 80, 150), (sx1, sy1), (sx2, sy2), 1)

        # Selected vehicle: draw full chain with labels
        if selected_agent and selected_agent.state in (BLOCKED, FOLLOWING):
            chain_colors = [(255,100,100), (255,180,60), (255,255,80), (180,255,80)]
            visited_chain = set()
            cur_a = selected_agent
            depth = 0
            while cur_a and depth < 20:
                # Find current blocker (prefer blocked_by, fall back to leader)
                blocker = cur_a.blocked_by if cur_a.state == BLOCKED else cur_a.leader
                if not blocker:
                    break
                if blocker.id in visited_chain:
                    # Deadlock cycle
                    sx1, sy1 = w2s(cur_a.x, cur_a.y)
                    sx2, sy2 = w2s(blocker.x, blocker.y)
                    pygame.draw.line(screen, (255, 0, 0), (sx1, sy1), (sx2, sy2), 3)
                    mx, my = (sx1+sx2)//2, (sy1+sy2)//2
                    dl_lbl = font_m.render("DEADLOCK", True, (255, 0, 0))
                    screen.blit(dl_lbl, (mx-30, my-10))
                    break
                visited_chain.add(cur_a.id)
                sx1, sy1 = w2s(cur_a.x, cur_a.y)
                sx2, sy2 = w2s(blocker.x, blocker.y)
                color = chain_colors[min(depth, len(chain_colors)-1)]
                pygame.draw.line(screen, color, (sx1, sy1), (sx2, sy2), 2)
                # Arrowhead
                ddx, ddy = sx2-sx1, sy2-sy1
                dist_s = math.hypot(ddx, ddy)
                if dist_s > 10:
                    ux, uy = ddx/dist_s, ddy/dist_s
                    ahx, ahy = sx2 - ux*10, sy2 - uy*10
                    px, py = -uy*6, ux*6
                    pygame.draw.polygon(screen, color, [
                        (sx2, sy2), (int(ahx+px), int(ahy+py)), (int(ahx-px), int(ahy-py))])
                # Label
                reason = blocker.block_reason if blocker.state == BLOCKED else blocker.state
                lbl = font_s.render(f"#{blocker.id} {reason}", True, color)
                screen.blit(lbl, (sx2+8, sy2-14))
                br = max(4, int(scale * 250))
                pygame.draw.circle(screen, color, (sx2, sy2), br, 2)
                cur_a = blocker if blocker.state in (BLOCKED, FOLLOWING) else None
                depth += 1

        # ── Info panel ────────────────────────────────────────────────────
        state_counts = collections.Counter(a.state for a in agents)
        info_lines = [
            f"Time: {sim_time:.1f}s  Speed: x{sim_speed:.1f}  {'PAUSED' if paused else 'RUNNING'}",
            f"Vehicles: {len(agents)}  Events: {env.event_count}",
            f"MOVING:{state_counts[MOVING]}  FOLLOW:{state_counts[FOLLOWING]}  "
            f"BLOCKED:{state_counts[BLOCKED]}  IDLE:{state_counts[IDLE]}  DONE:{state_counts[DONE]}",
        ]

        if selected_agent:
            a = selected_agent
            info_lines.append("")
            info_lines.append(f"Vehicle #{a.id}  State: {a.state}")
            info_lines.append(f"  Pos: ({a.x:.0f}, {a.y:.0f})  Speed: {a.vel(sim_time):.0f} mm/s")
            info_lines.append(f"  Path: {a.path_idx}/{len(a.node_path)-1} nodes")
            if a.leader:
                info_lines.append(f"  Following: #{a.leader.id}")
            if a.state == BLOCKED and a.block_reason:
                reason = a.block_reason
                if a.blocked_by:
                    reason += f" by #{a.blocked_by.id} ({a.blocked_by.state})"
                if a.block_zcu_id:
                    reason += f" [{a.block_zcu_id}]"
                info_lines.append(f"  BLOCKED: {reason}")
            # Show next segment info
            if a.path_idx + 1 < len(a.node_path):
                nxt_from = a.node_path[a.path_idx]
                nxt_to = a.node_path[a.path_idx + 1]
                nxt_seg = oht_map.segments.get((nxt_from, nxt_to))
                if nxt_seg:
                    info_lines.append(f"  Next seg: {nxt_from}->{nxt_to} len={nxt_seg.length:.0f}")
                    if nxt_seg.queue:
                        q_ids = [f"#{q.id}({q.state[0]})" for q in nxt_seg.queue]
                        info_lines.append(f"  Queue: {' '.join(q_ids)}")
                    # Check node blocker at destination
                    nb = env._node_blocker(nxt_to, a)
                    if nb:
                        info_lines.append(f"  NodeBlocker@{nxt_to}: #{nb.id}({nb.state})")
                else:
                    info_lines.append(f"  Next seg: {nxt_from}->{nxt_to} NOT FOUND")

        pw = 420
        ph = 18 * len(info_lines) + 16
        panel = pygame.Surface((pw, ph), pygame.SRCALPHA)
        panel.fill((30, 30, 45, 200))
        screen.blit(panel, (8, 8))
        for i, line in enumerate(info_lines):
            screen.blit(font_s.render(line, True, TEXT_COLOR), (14, 14 + i * 18))

        # Help
        help_txt = "Space:Pause  +/-:SimSpeed  A:Add  Click:Select  R:Reset  G:Grid  N:Nodes  Z:ZCU  F:Follow"
        screen.blit(font_s.render(help_txt, True, DIM_TEXT), (10, SH - 22))

        # Legend
        legend_x = SW - 200
        for i, (state, color) in enumerate(STATE_COLORS.items()):
            ly = 14 + i * 18
            pygame.draw.circle(screen, color, (legend_x, ly + 6), 5)
            screen.blit(font_s.render(state, True, TEXT_COLOR), (legend_x + 12, ly))

        pygame.display.flip()

    pygame.quit()


if __name__ == '__main__':
    main()
