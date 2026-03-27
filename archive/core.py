"""
core.py — MAPF Simulator: Grid, Agent, Task, TaskManager
"""
import random

# ── Grid ──────────────────────────────────────────────────────────────────────

class GridWorld:
    EMPTY    = 0
    OBSTACLE = 1

    # Preset obstacle layouts (row, col) for 15×15 grid
    PRESETS = {
        'warehouse': (
            [(r, 3)  for r in range(2, 7)]  + [(r, 3)  for r in range(8, 13)] +
            [(r, 7)  for r in range(2, 7)]  + [(r, 7)  for r in range(8, 13)] +
            [(r, 11) for r in range(2, 7)]  + [(r, 11) for r in range(8, 13)]
        ),
        'maze': (
            [(2, c)  for c in range(1, 10)] +
            [(4, c)  for c in range(5, 14)] +
            [(6, c)  for c in range(1, 10)] +
            [(8, c)  for c in range(5, 14)] +
            [(10, c) for c in range(1, 10)] +
            [(12, c) for c in range(5, 14)]
        ),
    }

    def __init__(self, rows=15, cols=15):
        self.rows  = rows
        self.cols  = cols
        self.cells = [[self.EMPTY] * cols for _ in range(rows)]

    def toggle(self, r, c):
        if self.is_valid(r, c):
            self.cells[r][c] ^= 1  # 0 ↔ 1

    def is_valid(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_free(self, r, c):
        return self.is_valid(r, c) and self.cells[r][c] == self.EMPTY

    def neighbors(self, r, c):
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = r + dr, c + dc
            if self.is_free(nr, nc):
                yield (nr, nc)

    def free_cells(self):
        return [(r, c) for r in range(self.rows)
                       for c in range(self.cols) if self.is_free(r, c)]

    def load_preset(self, name):
        self.cells = [[self.EMPTY] * self.cols for _ in range(self.rows)]
        for r, c in self.PRESETS.get(name, []):
            if self.is_valid(r, c):
                self.cells[r][c] = self.OBSTACLE

    def clear(self):
        self.cells = [[self.EMPTY] * self.cols for _ in range(self.rows)]


# ── Agent ─────────────────────────────────────────────────────────────────────

IDLE        = 'IDLE'
TO_PICKUP   = 'TO_PICKUP'
TO_DELIVERY = 'TO_DELIVERY'

AGENT_COLORS = [
    (70,  130, 180),   # Steel blue
    (220,  80,  80),   # Red
    (80,  200, 100),   # Green
    (210, 170,  50),   # Gold
    (170,  90, 210),   # Purple
    (60,  200, 200),   # Cyan
    (230, 130,  50),   # Orange
    (200,  80, 150),   # Pink
]

class Agent:
    def __init__(self, agent_id, start_pos):
        self.id       = agent_id
        self.pos      = start_pos    # (row, col) — logical grid position
        self.prev_pos = start_pos    # position before last step (for animation)
        self.color    = AGENT_COLORS[agent_id % len(AGENT_COLORS)]
        self.path     = []           # list of (row, col) to visit (start excluded)
        self.path_idx = 0
        self.status   = IDLE
        self.task     = None
        self.tasks_done = 0

    # ── Path control ──────────────────────────────────────────────────────────

    def set_path(self, path):
        """Assign new path (list from start to goal). Drops start cell."""
        self.path     = path[1:] if len(path) > 1 else []
        self.path_idx = 0

    def has_path(self):
        return self.path_idx < len(self.path)

    def peek_next(self):
        """Next cell without moving. Returns current pos if idle/done."""
        if self.has_path():
            return self.path[self.path_idx]
        return self.pos

    def at_destination(self):
        return not self.has_path()

    # ── Movement (called by simulator) ────────────────────────────────────────

    def advance(self):
        """Move one step forward along path."""
        self.prev_pos = self.pos
        if self.has_path():
            self.pos = self.path[self.path_idx]
            self.path_idx += 1

    def stay(self):
        """Wait in place (animation stays still this tick)."""
        self.prev_pos = self.pos

    def goal(self):
        """Current pathfinding goal based on status."""
        if self.status == TO_PICKUP and self.task:
            return self.task.pickup
        if self.status == TO_DELIVERY and self.task:
            return self.task.delivery
        return None


# ── Task ──────────────────────────────────────────────────────────────────────

_task_counter = 0

class Task:
    def __init__(self, pickup, delivery):
        global _task_counter
        self.id       = _task_counter
        _task_counter += 1
        self.pickup   = pickup    # (row, col)
        self.delivery = delivery  # (row, col)
        self.agent_id = None
        self.done     = False


# ── TaskManager ───────────────────────────────────────────────────────────────

class TaskManager:
    def __init__(self, grid):
        self.grid       = grid
        self.tasks      = []
        self.done_count = 0

    def generate(self, agent_positions=None):
        """Generate a random task on free, unoccupied cells."""
        blocked = set(agent_positions or [])
        for t in self.tasks:
            if not t.done:
                blocked.add(t.pickup)
                blocked.add(t.delivery)

        available = [c for c in self.grid.free_cells() if c not in blocked]
        if len(available) < 2:
            return None

        pickup, delivery = random.sample(available, 2)
        task = Task(pickup, delivery)
        self.tasks.append(task)
        return task

    def pending(self):
        return [t for t in self.tasks if not t.done and t.agent_id is None]

    def active(self):
        return [t for t in self.tasks if not t.done]

    def assign_pending(self, agents):
        """Assign pending tasks to nearest idle agents (greedy nearest)."""
        idle = [a for a in agents if a.status == IDLE]
        for task in self.pending():
            if not idle:
                break
            nearest = min(idle, key=lambda a: (
                abs(a.pos[0] - task.pickup[0]) + abs(a.pos[1] - task.pickup[1])
            ))
            task.agent_id = nearest.id
            nearest.task   = task
            nearest.status = TO_PICKUP
            idle.remove(nearest)

    def complete_task(self, agent):
        """Mark agent's current task as done, reset agent to IDLE."""
        if agent.task:
            agent.task.done = True
            self.done_count += 1
            agent.tasks_done += 1
        agent.task   = None
        agent.status = IDLE
        agent.path   = []
        agent.path_idx = 0
