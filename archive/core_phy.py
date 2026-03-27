"""
core_phy.py — Physical MAPF simulator: grid, agent with (r, c, θ, v) state
"""
import math, random

# ── Motion constants ───────────────────────────────────────────────────────────
V_MAX = 3          # max speed (cells per tick)

# θ index:  0=East  1=North  2=West  3=South
DIRS = [
    ( 0,  1),   # East  → +col
    (-1,  0),   # North → -row
    ( 0, -1),   # West  → -col
    ( 1,  0),   # South → +row
]
THETA_NAMES = ['E', 'N', 'W', 'S']

# Screen angle (degrees, clockwise from x-axis / East)
# East=0°  South=90°  West=180°  North=270°
SCREEN_ANGLES = [0, 270, 180, 90]

IDLE        = 'IDLE'
TO_PICKUP   = 'TO_PICKUP'
TO_DELIVERY = 'TO_DELIVERY'

AGENT_COLORS = [
    ( 70, 130, 180), (220,  80,  80), ( 80, 200, 100),
    (210, 170,  50), (170,  90, 210), ( 60, 200, 200),
    (230, 130,  50), (200,  80, 150),
]


# ── Grid ──────────────────────────────────────────────────────────────────────

class GridWorld:
    EMPTY    = 0
    OBSTACLE = 1

    PRESETS = {
        'warehouse': (
            [(r, 3)  for r in range(2,  7)] + [(r, 3)  for r in range(8, 13)] +
            [(r, 7)  for r in range(2,  7)] + [(r, 7)  for r in range(8, 13)] +
            [(r, 11) for r in range(2,  7)] + [(r, 11) for r in range(8, 13)]
        ),
        'maze': (
            [(2, c) for c in range(1, 10)] + [(4, c) for c in range(5, 14)] +
            [(6, c) for c in range(1, 10)] + [(8, c) for c in range(5, 14)] +
            [(10,c) for c in range(1, 10)] + [(12,c) for c in range(5, 14)]
        ),
    }

    def __init__(self, rows=15, cols=15):
        self.rows  = rows
        self.cols  = cols
        self.cells = [[self.EMPTY] * cols for _ in range(rows)]

    def toggle(self, r, c):
        if self.is_valid(r, c):
            self.cells[r][c] ^= 1

    def is_valid(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_free(self, r, c):
        return self.is_valid(r, c) and self.cells[r][c] == self.EMPTY

    def free_cells(self):
        return [(r, c) for r in range(self.rows)
                       for c in range(self.cols) if self.is_free(r, c)]

    def load_preset(self, name):
        self.cells = [[self.EMPTY]*self.cols for _ in range(self.rows)]
        for r, c in self.PRESETS.get(name, []):
            if self.is_valid(r, c):
                self.cells[r][c] = self.OBSTACLE

    def clear(self):
        self.cells = [[self.EMPTY]*self.cols for _ in range(self.rows)]


# ── Physical Agent ─────────────────────────────────────────────────────────────

class PhysAgent:
    BODY_W = 0.50   # width  in grid cells  (perpendicular to heading)
    BODY_L = 0.85   # length in grid cells  (along heading)

    def __init__(self, agent_id, start_r, start_c, start_theta=0):
        self.id    = agent_id
        self.color = AGENT_COLORS[agent_id % len(AGENT_COLORS)]

        # Current logical state
        self.r, self.c     = start_r, start_c
        self.theta         = start_theta  # 0-3
        self.vel           = 0

        # Previous state (for smooth animation interpolation)
        self.prev_r, self.prev_c   = start_r, start_c
        self.prev_theta            = start_theta
        self.prev_vel              = 0

        # Planned path: list of (r, c, theta, vel) tuples
        self.path     = []
        self.path_idx = 0

        # Task
        self.task       = None
        self.status     = IDLE
        self.tasks_done = 0

    @property
    def state(self):
        return (self.r, self.c, self.theta, self.vel)

    def goal(self):
        if self.status == TO_PICKUP   and self.task: return self.task.pickup
        if self.status == TO_DELIVERY and self.task: return self.task.delivery
        return None

    def pos(self):
        return (self.r, self.c)

    # ── Path management ───────────────────────────────────────────────────────

    def set_path(self, path):
        """Set new path (list of states). Drops the first entry (= current state)."""
        self.path     = path[1:] if len(path) > 1 else []
        self.path_idx = 0

    def has_path(self):
        return self.path_idx < len(self.path)

    def peek_next(self):
        if self.has_path():
            return self.path[self.path_idx]
        return self.state

    def at_destination(self):
        return not self.has_path()

    # ── Movement ──────────────────────────────────────────────────────────────

    def advance(self):
        """Move to next planned state."""
        self.prev_r, self.prev_c = self.r, self.c
        self.prev_theta          = self.theta
        self.prev_vel            = self.vel
        if self.has_path():
            self.r, self.c, self.theta, self.vel = self.path[self.path_idx]
            self.path_idx += 1

    def stay(self):
        """Wait in place (update prev for animation reset)."""
        self.prev_r, self.prev_c = self.r, self.c
        self.prev_theta          = self.theta
        self.prev_vel            = self.vel


# ── Task ──────────────────────────────────────────────────────────────────────

_task_counter = [0]

class Task:
    def __init__(self, pickup, delivery):
        self.id       = _task_counter[0]; _task_counter[0] += 1
        self.pickup   = pickup    # (r, c)
        self.delivery = delivery  # (r, c)
        self.agent_id = None
        self.done     = False


class TaskManager:
    def __init__(self, grid):
        self.grid       = grid
        self.tasks      = []
        self.done_count = 0

    def generate(self, agent_positions=None):
        blocked = set(agent_positions or [])
        for t in self.tasks:
            if not t.done:
                blocked.add(t.pickup); blocked.add(t.delivery)
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
        idle = [a for a in agents if a.status == IDLE]
        for task in self.pending():
            if not idle: break
            nearest = min(idle, key=lambda a: abs(a.r-task.pickup[0]) + abs(a.c-task.pickup[1]))
            task.agent_id     = nearest.id
            nearest.task      = task
            nearest.status    = TO_PICKUP
            idle.remove(nearest)

    def complete_task(self, agent):
        if agent.task:
            agent.task.done = True
            self.done_count += 1
            agent.tasks_done += 1
        agent.task, agent.status = None, IDLE
        agent.path, agent.path_idx, agent.vel = [], 0, 0
