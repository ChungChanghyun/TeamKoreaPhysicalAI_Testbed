"""
state_graph.py — Fixed-cost S/M/R state machine for AGV path planning.

Planning layer uses FIXED costs (max-speed based), decoupled from kinematics.

State types
───────────
StopState(node_id, theta)            — agent stopped at node, facing theta
MoveState(from_id, to_id)            — agent traversing an edge
RotateState(node_id, theta0, theta1) — agent rotating in place

Fixed costs
───────────
  MoveState   : edge.length / edge.max_speed          (seconds)
  RotateState : |Δθ| / OMEGA_MAX                      (seconds)
  StopState   : 0  (wait duration determined by planner)

The execution layer (KinematicAgent.set_plan) receives a plan as a list of
  [(state, t_depart), ...]
and physically executes each state with the kinematic model, departing at
the planned t_depart for each state.
"""
from __future__ import annotations
import math
from typing import Dict, List, Tuple

from map_loader import MapGraph
from env_cont import OMEGA_MAX, angle_diff


# ── State classes ──────────────────────────────────────────────────────────────

class StopState:
    """Agent is stopped at a node, facing theta."""

    def __init__(self, node_id: str, theta: float, x: float, y: float):
        self.node_id = node_id
        self.theta   = theta    # radians
        self.x       = x        # mm (for convenience)
        self.y       = y
        self.cost    = 0.0      # planner sets the wait duration
        self.id      = f"S,{node_id},{math.degrees(theta):.1f}"

    def __repr__(self):
        return self.id


class MoveState:
    """Agent moves from from_id to to_id along a directed edge."""

    def __init__(self, from_id: str, to_id: str,
                 theta: float, length: float, max_speed: float):
        self.from_id   = from_id
        self.to_id     = to_id
        self.theta     = theta          # heading = edge angle
        self.length    = length         # mm
        self.max_speed = max_speed      # mm/s
        self.cost      = length / max_speed   # FIXED cost (seconds)
        self.id        = f"M,{from_id},{to_id}"

    def __repr__(self):
        return f"{self.id}  cost={self.cost:.3f}s"


class RotateState:
    """Agent rotates in place at node_id from theta_start to theta_end."""

    def __init__(self, node_id: str, theta_start: float, theta_end: float,
                 x: float, y: float):
        self.node_id     = node_id
        self.theta_start = theta_start
        self.theta_end   = theta_end
        self.x           = x
        self.y           = y
        d_angle          = abs(angle_diff(theta_end, theta_start))
        self.cost        = d_angle / OMEGA_MAX   # FIXED cost (seconds)
        self.id = (f"R,{node_id},"
                   f"{math.degrees(theta_start):.1f},"
                   f"{math.degrees(theta_end):.1f}")

    def __repr__(self):
        return f"{self.id}  cost={self.cost:.3f}s"


# type alias
State = object   # Union[StopState, MoveState, RotateState]


# ── State graph ────────────────────────────────────────────────────────────────

class StateGraph:
    """
    Builds the complete S/M/R state machine from a MapGraph.

    Attributes
    ──────────
    stop_states    : {(node_id, theta)           -> StopState}
    move_states    : {(from_id, to_id)           -> MoveState}
    rotate_states  : {(node_id, theta0, theta1)  -> RotateState}
    """

    def __init__(self, graph: MapGraph):
        self.graph = graph
        self.stop_states:   Dict[tuple, StopState]   = {}
        self.move_states:   Dict[tuple, MoveState]   = {}
        self.rotate_states: Dict[tuple, RotateState] = {}
        self._build()

    # ── Construction ──────────────────────────────────────────────────────────

    def _build(self):
        # Collect all headings valid at each node
        # (departure heading for outgoing edges, arrival heading = edge.angle)
        node_headings: Dict[str, set] = {nid: set() for nid in self.graph.nodes}
        for (fn, tn), edge in self.graph.edges.items():
            node_headings[fn].add(edge.angle)
            node_headings[tn].add(edge.angle)

        # MoveStates — one per directed edge
        for (fn, tn), edge in self.graph.edges.items():
            self.move_states[(fn, tn)] = MoveState(
                fn, tn, edge.angle, edge.length, edge.max_speed
            )

        # StopStates — one per (node, valid_heading) pair
        for nid, node in self.graph.nodes.items():
            for theta in node_headings[nid]:
                self.stop_states[(nid, theta)] = StopState(
                    nid, theta, node.x, node.y
                )

        # RotateStates — between every pair of headings at the same node
        for nid, node in self.graph.nodes.items():
            headings = sorted(node_headings[nid])
            for t0 in headings:
                for t1 in headings:
                    if abs(angle_diff(t1, t0)) > 1e-4:
                        self.rotate_states[(nid, t0, t1)] = RotateState(
                            nid, t0, t1, node.x, node.y
                        )

    # ── Lookup helpers ────────────────────────────────────────────────────────

    def get_stop(self, node_id: str, theta: float) -> StopState:
        return self.stop_states.get((node_id, theta))

    def get_move(self, from_id: str, to_id: str) -> MoveState:
        return self.move_states.get((from_id, to_id))

    def get_rotate(self, node_id: str, t0: float, t1: float) -> RotateState:
        return self.rotate_states.get((node_id, t0, t1))

    # ── Transition model (for planner) ────────────────────────────────────────

    def next_states(self, state: State) -> List[State]:
        """
        Valid successor states reachable from `state` in one action.

        StopState  →  MoveState (if heading matches an outgoing edge)
                   →  RotateState (any other valid heading)
        MoveState  →  StopState (at destination, same heading)
        RotateState→  StopState (same node, new heading)
        """
        if isinstance(state, StopState):
            succs = []
            for to_id in self.graph.neighbors(state.node_id):
                edge = self.graph.get_edge(state.node_id, to_id)
                if abs(angle_diff(edge.angle, state.theta)) < 1e-4:
                    m = self.move_states.get((state.node_id, to_id))
                    if m:
                        succs.append(m)
            for (nid, t0, t1), rot in self.rotate_states.items():
                if nid == state.node_id and abs(angle_diff(t0, state.theta)) < 1e-4:
                    succs.append(rot)
            return succs

        if isinstance(state, MoveState):
            s = self.stop_states.get((state.to_id, state.theta))
            return [s] if s else []

        if isinstance(state, RotateState):
            s = self.stop_states.get((state.node_id, state.theta_end))
            return [s] if s else []

        return []

    # ── Cost helper ───────────────────────────────────────────────────────────

    def cost(self, state: State) -> float:
        """Fixed cost of executing `state` (seconds)."""
        return getattr(state, 'cost', 0.0)

    # ── Info ──────────────────────────────────────────────────────────────────

    def __repr__(self):
        return (f"StateGraph("
                f"{len(self.stop_states)} stop, "
                f"{len(self.move_states)} move, "
                f"{len(self.rotate_states)} rotate)")
