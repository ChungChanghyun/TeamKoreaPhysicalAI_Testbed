ㅈ"""
pathfinder.py — A* / Space-time A* / CBS

Two planning modes:
  plan_all(grid, agents)   — Prioritized planning (fast, suboptimal)
  plan_cbs(grid, agents)   — CBS (optimal collision-free, slower)
"""
import heapq


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# ── Standard A* (space only) ──────────────────────────────────────────────────

def astar(grid, start, goal, avoid=None):
    """
    A* on GridWorld (no time dimension).
    avoid : set of (r,c) blocked cells (start and goal are always passable).
    Returns [start, ..., goal] or [] if unreachable.
    """
    if start == goal:
        return [start]

    avoid = (avoid or set()) - {start, goal}
    heap = [(heuristic(start, goal), 0, start)]
    came_from = {}
    g = {start: 0}

    while heap:
        f, cost, cur = heapq.heappop(heap)

        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur); cur = came_from[cur]
            path.append(start); path.reverse()
            return path

        if cost > g.get(cur, float('inf')):
            continue

        for nxt in grid.neighbors(*cur):
            if nxt in avoid:
                continue
            new_g = cost + 1
            if new_g < g.get(nxt, float('inf')):
                g[nxt] = new_g
                came_from[nxt] = cur
                heapq.heappush(heap, (new_g + heuristic(nxt, goal), new_g, nxt))

    return []


# ── Space-Time A* (for CBS low-level) ────────────────────────────────────────

def space_time_astar(grid, start, goal, v_constraints=None, e_constraints=None):
    """
    A* in space-time.

    v_constraints : set of (cell, t) — agent cannot be at cell at time t
    e_constraints : set of (cell_a, cell_b, t) — cannot move a→b at time t
                    (handles swap conflicts)

    Returns [start, ..., goal] where repeated consecutive cells = wait action.
    Returns [] if no path found within time limit.
    """
    if start == goal:
        return [start]

    v_constr = v_constraints or set()
    e_constr = e_constraints or set()
    max_t    = grid.rows * grid.cols  # generous upper bound

    # heap: (f, g, cell, t)
    heap = [(heuristic(start, goal), 0, start, 0)]
    came_from = {}          # (cell, t) -> (prev_cell, prev_t)
    g_score   = {(start, 0): 0}

    while heap:
        f, g, cell, t = heapq.heappop(heap)

        if cell == goal:
            path = []
            state = (cell, t)
            while state in came_from:
                path.append(state[0]); state = came_from[state]
            path.append(start); path.reverse()
            return path

        if g > g_score.get((cell, t), float('inf')):
            continue
        if t >= max_t:
            continue

        next_t = t + 1
        # Actions: move to neighbor OR wait in place
        candidates = list(grid.neighbors(*cell)) + [cell]

        for nxt in candidates:
            # Vertex constraint: forbidden to be at nxt at next_t
            if (nxt, next_t) in v_constr:
                continue
            # Edge constraint: forbidden swap (cell→nxt at time t)
            if (cell, nxt, t) in e_constr:
                continue

            new_g = g + 1
            if new_g < g_score.get((nxt, next_t), float('inf')):
                g_score[(nxt, next_t)] = new_g
                came_from[(nxt, next_t)] = (cell, t)
                heapq.heappush(heap,
                    (new_g + heuristic(nxt, goal), new_g, nxt, next_t))

    return []  # No path found


# ── Conflict detection ────────────────────────────────────────────────────────

def _get_pos(path, t):
    """Return position of agent at time t (stays at goal after path ends)."""
    if not path:
        return None
    return path[min(t, len(path) - 1)]


def find_first_conflict(paths):
    """
    Find the first conflict among space-time paths.

    paths : {agent_id: [cell, ...]}

    Returns (a1, a2, 'vertex', cell, t)
         or (a1, a2, 'edge',  (cell_a, cell_b), t)
         or None if no conflict.
    """
    if len(paths) < 2:
        return None

    ids  = list(paths.keys())
    maxt = max(len(p) for p in paths.values())

    for t in range(maxt + 1):
        # ── Vertex conflicts ──────────────────────────────────────────────────
        at_cell = {}
        for aid in ids:
            pos = _get_pos(paths[aid], t)
            if pos is None:
                continue
            if pos in at_cell:
                return (at_cell[pos], aid, 'vertex', pos, t)
            at_cell[pos] = aid

        # ── Edge (swap) conflicts ─────────────────────────────────────────────
        if t + 1 <= maxt:
            for i, a1 in enumerate(ids):
                for a2 in ids[i + 1:]:
                    p1_t  = _get_pos(paths[a1], t)
                    p1_t1 = _get_pos(paths[a1], t + 1)
                    p2_t  = _get_pos(paths[a2], t)
                    p2_t1 = _get_pos(paths[a2], t + 1)
                    if None in (p1_t, p1_t1, p2_t, p2_t1):
                        continue
                    if p1_t == p2_t1 and p2_t == p1_t1:
                        return (a1, a2, 'edge', (p1_t, p1_t1), t)

    return None


# ── CBS high-level search ─────────────────────────────────────────────────────

def cbs(grid, start_goal_map, max_iter=2000):
    """
    Conflict-Based Search.

    start_goal_map : {agent_id: (start_cell, goal_cell)}

    Returns {agent_id: path} (conflict-free) or None if failed.

    Each CT node stores:
      v_constr : {agent_id: set of (cell, t)}
      e_constr : {agent_id: set of (cell_a, cell_b, t)}
      paths    : {agent_id: [cell, ...]}
      cost     : sum of path lengths (SIC)
    """
    ids = list(start_goal_map.keys())

    # ── Root node ─────────────────────────────────────────────────────────────
    root_v = {aid: set() for aid in ids}
    root_e = {aid: set() for aid in ids}
    root_paths = {}

    for aid, (start, goal) in start_goal_map.items():
        path = space_time_astar(grid, start, goal)
        if not path:
            return None   # No individual path exists
        root_paths[aid] = path

    root_cost = sum(len(p) for p in root_paths.values())

    # Priority queue: (cost, node_id, v_constr, e_constr, paths)
    counter = 0
    open_set = [(root_cost, counter, root_v, root_e, root_paths)]
    counter += 1

    for _ in range(max_iter):
        if not open_set:
            break

        cost, _, v_constr, e_constr, paths = heapq.heappop(open_set)

        conflict = find_first_conflict(paths)
        if conflict is None:
            return paths   # ✓ Conflict-free solution

        a1, a2, ctype, cdata, t = conflict

        # ── Branch: blame a1, then blame a2 ──────────────────────────────────
        for blamed in (a1, a2):
            new_v = {aid: set(s) for aid, s in v_constr.items()}
            new_e = {aid: set(s) for aid, s in e_constr.items()}

            if ctype == 'vertex':
                new_v[blamed].add((cdata, t))
            else:   # edge swap conflict
                cell_from, cell_to = cdata
                new_e[blamed].add((cell_from, cell_to, t))
                # Also forbid being at cell_from at t+1 (the other side of swap)
                new_v[blamed].add((cell_from, t + 1))

            start_b, goal_b = start_goal_map[blamed]
            new_path = space_time_astar(
                grid, start_b, goal_b,
                v_constraints=new_v[blamed],
                e_constraints=new_e[blamed],
            )
            if not new_path:
                continue   # This branch has no valid path; prune

            new_paths = dict(paths)
            new_paths[blamed] = new_path
            new_cost = sum(len(p) for p in new_paths.values())

            heapq.heappush(open_set,
                (new_cost, counter, new_v, new_e, new_paths))
            counter += 1

    return None   # CBS exhausted (no solution or max_iter reached)


# ── Public planning interfaces ────────────────────────────────────────────────

def plan_all(grid, agents):
    """
    Prioritized planning (fast).
    Agents with lower id plan first; higher-id agents avoid their reserved cells.
    """
    reserved = set()

    for agent in sorted(agents, key=lambda a: a.id):
        goal = agent.goal()
        if goal is None or (agent.pos == goal and not agent.has_path()):
            continue

        path = astar(grid, agent.pos, goal, avoid=reserved)
        if not path:
            path = astar(grid, agent.pos, goal)   # fallback: ignore reservations

        agent.set_path(path)
        reserved.update(path)


def plan_cbs(grid, agents):
    """
    CBS planning (optimal collision-free, slower).
    Plans for all non-idle agents simultaneously.
    Falls back to prioritized planning if CBS fails or times out.

    Returns True if CBS succeeded, False if fallback was used.
    """
    active = [a for a in agents if a.goal() is not None and a.pos != a.goal()]
    if not active:
        return True

    start_goal = {a.id: (a.pos, a.goal()) for a in active}
    result = cbs(grid, start_goal)

    if result is None:
        # CBS failed — fall back to prioritized
        plan_all(grid, agents)
        return False

    for agent in active:
        if agent.id in result:
            agent.set_path(result[agent.id])

    return True
