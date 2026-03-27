"""
pathfinder_phy.py — Path planning for physical agents with (r, c, θ, v) state

Obstacle collision: cell-based (fast)
Agent-agent collision: Shapely polygon footprints (physically accurate)
"""
import heapq, math
from shapely.geometry import Polygon
from shapely.ops import unary_union
from core_phy import DIRS, V_MAX, SCREEN_ANGLES, TO_PICKUP, TO_DELIVERY


# ── Footprint geometry ─────────────────────────────────────────────────────────

def _footprint(r, c, theta, body_w, body_l):
    """
    Shapely Polygon for agent at (r, c) facing theta.
    Shapely coords: x = col + 0.5,  y = row + 0.5
    """
    cx = c + 0.5;  cy = r + 0.5
    rad = math.radians(SCREEN_ANGLES[theta])
    fw  = (math.cos(rad),  math.sin(rad))   # forward unit vector
    rt  = (-math.sin(rad), math.cos(rad))   # right unit vector
    hl, hw = body_l / 2, body_w / 2

    pts = []
    for fl, rw in [(-hl,-hw), (-hl,hw), (hl,hw), (hl,-hw)]:
        pts.append((cx + fl*fw[0] + rw*rt[0],
                    cy + fl*fw[1] + rw*rt[1]))
    return Polygon(pts)


def get_footprint(r, c, theta, body_w, body_l):
    return _footprint(r, c, theta, body_w, body_l)


def get_swept_poly(r0, c0, r1, c1, theta, v, body_w, body_l):
    """Union of footprints at each integer step of the movement."""
    if r0 == r1 and c0 == c1:
        return _footprint(r0, c0, theta, body_w, body_l)
    dr = DIRS[theta][0];  dc = DIRS[theta][1]
    polys = [_footprint(r0 + i*dr, c0 + i*dc, theta, body_w, body_l)
             for i in range(v + 1)]
    return unary_union(polys)


# ── Obstacle check (cell-based, fast) ─────────────────────────────────────────

def _path_clear(grid, r, c, theta, v):
    """Check that all v cells along the forward path are free in the grid."""
    dr, dc = DIRS[theta]
    for step in range(1, v + 1):
        if not grid.is_free(r + step*dr, c + step*dc):
            return False
    return True


# ── A* over physical state space ──────────────────────────────────────────────

def _heuristic(r, c, goal_r, goal_c, vel):
    dist = abs(r - goal_r) + abs(c - goal_c)
    # Lower bound: max(time to stop, time to cover distance at V_MAX)
    return max(vel, math.ceil(dist / V_MAX) if dist else 0)


def phy_astar(grid, start, goal_rc, v_constraints=None, e_constraints=None):
    """
    A* over (r, c, theta, vel) state space.

    start    : (r, c, theta, vel)
    goal_rc  : (goal_r, goal_c)  — any theta, vel must be 0 at goal

    Transition model (per tick):
      v=0: wait | rotate ±90° | accelerate→move 1 cell
      v>0: brake(stay, v→0) | decel(move v-1 cells) | maintain | accel(move v+1 cells)

    Returns list of states [start, ..., goal] or [].
    """
    goal_r, goal_c = goal_rc

    if (start[0], start[1]) == (goal_r, goal_c) and start[3] == 0:
        return [start]

    v_con = v_constraints or set()
    e_con = e_constraints or set()
    max_t = grid.rows * grid.cols * 3   # generous time limit

    h0 = _heuristic(start[0], start[1], goal_r, goal_c, start[3])
    heap = [(h0, 0, start, 0)]
    came_from = {}
    g_score   = {(start, 0): 0}

    while heap:
        f, g, state, t = heapq.heappop(heap)
        r, c, theta, vel = state

        # Goal: at target cell, stopped
        if (r, c) == (goal_r, goal_c) and vel == 0:
            path = []
            key  = (state, t)
            while key in came_from:
                path.append(key[0]); key = came_from[key]
            path.append(start); path.reverse()
            return path

        if g > g_score.get((state, t), float('inf')):
            continue
        if t >= max_t:
            continue

        next_t = t + 1
        succs  = []
        dr, dc = DIRS[theta]

        if vel == 0:
            # Stationary: rotate or start moving
            succs.append((r, c, (theta - 1) % 4, 0))  # rotate CW
            succs.append((r, c, (theta + 1) % 4, 0))  # rotate CCW
            succs.append((r, c, theta, 0))              # wait
            # Accelerate to v=1
            r1, c1 = r + dr, c + dc
            if _path_clear(grid, r, c, theta, 1):
                succs.append((r1, c1, theta, 1))
        else:
            # Moving: brake / decel / maintain / accel
            succs.append((r, c, theta, 0))              # full brake → stay

            v_d = vel - 1
            if v_d > 0:
                rd, cd = r + v_d*dr, c + v_d*dc
                if _path_clear(grid, r, c, theta, v_d):
                    succs.append((rd, cd, theta, v_d))  # decelerate

            if _path_clear(grid, r, c, theta, vel):
                rm, cm = r + vel*dr, c + vel*dc
                succs.append((rm, cm, theta, vel))      # maintain

            if vel < V_MAX:
                v_a = vel + 1
                if _path_clear(grid, r, c, theta, v_a):
                    ra, ca = r + v_a*dr, c + v_a*dc
                    succs.append((ra, ca, theta, v_a))  # accelerate

        for nxt in succs:
            if v_con and (nxt, next_t) in v_con:
                continue
            if e_con and (state, nxt, t) in e_con:
                continue

            new_g = g + 1
            key   = (nxt, next_t)
            if new_g < g_score.get(key, float('inf')):
                g_score[key]   = new_g
                came_from[key] = (state, t)
                h = _heuristic(nxt[0], nxt[1], goal_r, goal_c, nxt[3])
                heapq.heappush(heap, (new_g + h, new_g, nxt, next_t))

    return []


# ── Conflict detection (Shapely footprint-based) ──────────────────────────────

def _pos_at(path, t):
    """State at time t (agent stays at final state after path ends)."""
    return path[min(t, len(path) - 1)]


def find_phy_conflict(paths, body_w, body_l):
    """
    Find first timestep where any two agents' footprints intersect.
    Returns (a1_id, a2_id, state_a1, t) or None.
    """
    ids  = list(paths.keys())
    maxt = max(len(p) for p in paths.values())

    for t in range(maxt + 1):
        fps = {aid: _footprint(*_pos_at(paths[aid], t)[:3], body_w, body_l)
               for aid in ids}
        for i, a1 in enumerate(ids):
            for a2 in ids[i + 1:]:
                if fps[a1].intersects(fps[a2]):
                    return (a1, a2, _pos_at(paths[a1], t), t)
    return None


# ── CBS (physical) ────────────────────────────────────────────────────────────

def phy_cbs(grid, start_goal_map, body_w, body_l, max_iter=800):
    """
    CBS for physical agents.
    start_goal_map: {agent_id: (start_state, goal_rc)}
    Returns {agent_id: path} or None.
    """
    ids = list(start_goal_map.keys())

    # Root node
    root_v = {aid: set() for aid in ids}
    root_e = {aid: set() for aid in ids}
    root_paths = {}
    for aid, (start, goal_rc) in start_goal_map.items():
        path = phy_astar(grid, start, goal_rc)
        if not path:
            return None
        root_paths[aid] = path

    root_cost = sum(len(p) for p in root_paths.values())
    counter   = 0
    open_set  = [(root_cost, counter, root_v, root_e, root_paths)]
    counter  += 1

    for _ in range(max_iter):
        if not open_set:
            break
        cost, _, v_con, e_con, paths = heapq.heappop(open_set)

        conflict = find_phy_conflict(paths, body_w, body_l)
        if conflict is None:
            return paths   # ✓ solution

        a1, a2, conflicting_state, t = conflict

        for blamed in (a1, a2):
            new_v = {aid: set(s) for aid, s in v_con.items()}
            new_e = {aid: set(s) for aid, s in e_con.items()}
            new_v[blamed].add((conflicting_state, t))

            start_b, goal_b = start_goal_map[blamed]
            new_path = phy_astar(grid, start_b, goal_b,
                                 v_constraints=new_v[blamed],
                                 e_constraints=new_e[blamed])
            if not new_path:
                continue

            new_paths          = dict(paths)
            new_paths[blamed]  = new_path
            new_cost           = sum(len(p) for p in new_paths.values())
            heapq.heappush(open_set,
                (new_cost, counter, new_v, new_e, new_paths))
            counter += 1

    return None


# ── Public planning interfaces ─────────────────────────────────────────────────

def plan_phy_all(grid, agents):
    """Prioritized planning for physical agents (fast)."""
    for agent in sorted(agents, key=lambda a: a.id):
        goal = agent.goal()
        if goal is None:
            continue
        if (agent.r, agent.c) == goal and agent.vel == 0:
            continue
        path = phy_astar(grid, agent.state, goal)
        if path:
            agent.set_path(path)


def plan_phy_cbs(grid, agents):
    """
    CBS for physical agents.
    Returns True if CBS succeeded, False if fell back to prioritized.
    """
    bw = PhysAgent_BODY_W  # avoid import cycle — set below
    bl = PhysAgent_BODY_L

    active = [a for a in agents
              if a.goal() is not None and (a.r, a.c) != a.goal()]
    if not active:
        return True

    start_goal = {a.id: (a.state, a.goal()) for a in active}
    result = phy_cbs(grid, start_goal, bw, bl)

    if result is None:
        plan_phy_all(grid, agents)
        return False

    for agent in active:
        if agent.id in result:
            agent.set_path(result[agent.id])
    return True


# body size constants (avoids importing PhysAgent in circular way)
from core_phy import PhysAgent as _PA
PhysAgent_BODY_W = _PA.BODY_W
PhysAgent_BODY_L = _PA.BODY_L
