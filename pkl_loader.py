"""
pkl_loader.py — Load a collision-profile .pkl into a MapGraph + region overlays.

The pkl contains:
  Stop_state    : {state_id -> State}  (S,node_idx,angle)
  Move_state    : {state_id -> State}  (M,from_idx,to_idx)
  stop_regions  : {state_id -> Shapely Polygon}   agent footprint at each Stop
  move_regions  : {state_id -> Shapely Polygon}   swept area for each Move

Node positions are inferred as the centroid of each stop_region polygon.
Edge angles are computed from node-centroid geometry.
Edge max_speed defaults to DEFAULT_SPEED (1000 mm/s) — consistent with pkl data.
"""
from __future__ import annotations
import math
import pickle
import sys
import os
from typing import Dict, Tuple, List

# Need the State class from the solver to unpickle
def _load_pkl(filepath: str) -> dict:
    """Load pkl, injecting solver State class into __main__ for unpickling."""
    solver_dir = os.path.join(os.path.dirname(__file__), 'solvers')
    if solver_dir not in sys.path:
        sys.path.insert(0, solver_dir)
    try:
        from ACS_graph_grid_focal_crisscross_heapcost_backup1028_General_Queuing import (
            State, Point, PointType, Direction
        )
        import __main__
        __main__.State     = State
        __main__.Point     = Point
        __main__.PointType = PointType
        __main__.Direction = Direction
    except ImportError as e:
        raise ImportError(f"Cannot import solver State class: {e}")

    with open(filepath, 'rb') as f:
        return pickle.load(f)


DEFAULT_SPEED = 1000.0   # mm/s  (all edges in the test map use 1 m/s)


# ── Minimal node / edge containers (compatible with MapGraph interface) ────────

class PklNode:
    def __init__(self, node_id: str, x: float, y: float):
        self.id   = node_id
        self.x    = x       # mm
        self.y    = y       # mm
        self.kind = 'Normal'

    def __repr__(self):
        return f'Node({self.id}, {self.x/1000:.2f}m, {self.y/1000:.2f}m)'


class PklEdge:
    def __init__(self, from_id: str, to_id: str,
                 from_node: PklNode, to_node: PklNode,
                 max_speed: float = DEFAULT_SPEED):
        self.id        = f"M,{from_id},{to_id}"
        self.from_id   = from_id
        self.to_id     = to_id
        dx             = to_node.x - from_node.x
        dy             = to_node.y - from_node.y
        self.length    = math.hypot(dx, dy)          # mm
        self.angle     = math.atan2(dy, dx)          # radians (map-space, y-up)
        self.max_speed = max_speed                   # mm/s

    def __repr__(self):
        return f'Edge({self.from_id}→{self.to_id}, L={self.length:.0f}mm)'


class PklMapGraph:
    """
    MapGraph-compatible graph built from a collision-profile .pkl.

    Additional attributes (not in plain MapGraph)
    ──────────────────────────────────────────────
    stop_regions  : {state_id: Shapely Polygon}
    move_regions  : {state_id: Shapely Polygon}
    stop_states   : raw State dict from pkl
    move_states   : raw State dict from pkl
    """

    def __init__(self, pkl_path: str):
        self._load(pkl_path)

    def _load(self, pkl_path: str):
        data = _load_pkl(pkl_path)

        self.stop_regions: Dict[str, object] = data.get('stop_regions', {})
        self.move_regions: Dict[str, object] = data.get('move_regions', {})
        self.stop_states_raw   = data.get('Stop_state',   {})
        self.move_states_raw   = data.get('Move_state',   {})
        self.rotate_states_raw = data.get('Rotate_state', {})

        # ── Node positions from stop_region centroids ──────────────────────
        self.nodes: Dict[str, PklNode] = {}
        for state_id, poly in self.stop_regions.items():
            parts = state_id.split(',')
            if parts[0] != 'S':
                continue
            nid = parts[1]
            if nid not in self.nodes:
                c = poly.centroid
                self.nodes[nid] = PklNode(nid, c.x, c.y)

        # ── Edges from move_state keys ─────────────────────────────────────
        self.edges: Dict[Tuple, PklEdge] = {}
        self.adj:   Dict[str, List[str]] = {nid: [] for nid in self.nodes}
        for state_id in self.move_states_raw:
            parts = state_id.split(',')
            if parts[0] != 'M':
                continue
            fn, tn = parts[1], parts[2]
            if fn not in self.nodes or tn not in self.nodes:
                continue
            edge = PklEdge(fn, tn, self.nodes[fn], self.nodes[tn], DEFAULT_SPEED)
            self.edges[(fn, tn)] = edge
            if tn not in self.adj.get(fn, []):
                self.adj.setdefault(fn, []).append(tn)

        # ── Vehicle dimensions from stop polygon edge lengths ──────────────
        self.vehicle_length = 2000.0   # mm default
        self.vehicle_width  = 1300.0   # mm default
        if self.stop_regions:
            sample_poly = next(iter(self.stop_regions.values()))
            coords = list(sample_poly.exterior.coords)[:-1]
            if len(coords) >= 2:
                d01 = math.hypot(coords[1][0] - coords[0][0],
                                 coords[1][1] - coords[0][1])
                d12 = math.hypot(coords[2][0] - coords[1][0],
                                 coords[2][1] - coords[1][1])
                self.vehicle_length = max(d01, d12)
                self.vehicle_width  = min(d01, d12)

        # ── Ports: nodes that only appear in stop states (no outgoing edges)
        # or nodes explicitly marked — use od_pairs destinations if available
        self.ports: Dict[str, str] = {}
        od = data.get('od_pairs', [])
        for i, (src, dst) in enumerate(od):
            self.ports[str(i)] = str(dst)

    # ── MapGraph-compatible helpers ────────────────────────────────────────────

    def get_edge(self, from_id: str, to_id: str) -> 'PklEdge | None':
        return self.edges.get((from_id, to_id))

    def neighbors(self, node_id: str) -> List[str]:
        return self.adj.get(node_id, [])

    @property
    def bbox(self) -> Tuple[float, float, float, float]:
        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        return min(xs), min(ys), max(xs), max(ys)

    def __repr__(self):
        return (f'PklMapGraph({len(self.nodes)} nodes, '
                f'{len(self.edges)} edges, '
                f'{len(self.stop_regions)} stop_regions, '
                f'{len(self.move_regions)} move_regions)')
