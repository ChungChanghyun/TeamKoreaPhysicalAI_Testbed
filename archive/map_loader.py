"""
map_loader.py — Load xmsmap-v3/v4 JSON into a MapGraph.

Coordinate convention (map-space):
  x = right (mm),  y = up (mm)

Edge angles are stored in map-space radians: atan2(dy, dx).
"""
from __future__ import annotations
import json
import math


class Node:
    def __init__(self, node_id: str, x: float, y: float, kind: str = 'Normal'):
        self.id   = node_id
        self.x    = float(x)   # mm
        self.y    = float(y)   # mm
        self.kind = kind

    def __repr__(self):
        return f'Node({self.id}, x={self.x:.0f}, y={self.y:.0f})'


class Edge:
    def __init__(self, seg_id: int, from_id: str, to_id: str,
                 from_node: Node, to_node: Node, max_speed: float):
        self.id        = seg_id
        self.from_id   = from_id
        self.to_id     = to_id
        dx             = to_node.x - from_node.x
        dy             = to_node.y - from_node.y
        self.length    = math.hypot(dx, dy)           # mm
        self.angle     = math.atan2(dy, dx)           # radians, map-space (y-up)
        self.max_speed = float(max_speed) if max_speed > 0 else 1000.0  # mm/s

    def __repr__(self):
        return f'Edge({self.from_id}→{self.to_id}, L={self.length:.0f}mm, v={self.max_speed:.0f}mm/s)'


class MapGraph:
    """
    Graph loaded from xmsmap JSON (schema v3 or v4).

    Attributes
    ----------
    nodes          : {node_id: Node}
    edges          : {(from_id, to_id): Edge}
    adj            : {from_id: [to_id, ...]}
    ports          : {port_id: node_id}
    vehicle_width  : float  (mm)
    vehicle_length : float  (mm)
    """

    def __init__(self, filepath: str):
        self._load(filepath)

    # ── Loading ───────────────────────────────────────────────────────────────

    def _load(self, filepath: str):
        with open(filepath, 'r') as f:
            data = json.load(f)

        # Nodes
        self.nodes: dict[str, Node] = {}
        for n in data.get('nodes', []):
            kind = n.get('kind', 'Normal')
            self.nodes[str(n['id'])] = Node(str(n['id']), n['x'], n['y'], kind)

        # Edges (segments)
        self.edges: dict[tuple, Edge] = {}
        self.adj:   dict[str, list]   = {nid: [] for nid in self.nodes}
        for s in data.get('segments', []):
            fn = str(s['startNodeId'])
            tn = str(s['endNodeId'])
            if fn not in self.nodes or tn not in self.nodes:
                continue
            edge = Edge(s['id'], fn, tn,
                        self.nodes[fn], self.nodes[tn], s.get('speed', 1000))
            self.edges[(fn, tn)] = edge
            self.adj.setdefault(fn, [])
            if tn not in self.adj[fn]:
                self.adj[fn].append(tn)

        # Ports (stations / pickup-delivery)
        self.ports: dict[str, str] = {}
        for p in data.get('ports', []):
            self.ports[str(p['id'])] = str(p['nodeId'])

        # Vehicle model (first entry wins)
        self.vehicle_width  = 600.0   # mm default
        self.vehicle_length = 750.0   # mm default
        for vm in data.get('vehicleModels', []):
            d = vm.get('dimension', {})
            self.vehicle_width  = float(d.get('width',  self.vehicle_width))
            self.vehicle_length = float(d.get('length', self.vehicle_length))
            break

    # ── Helpers ───────────────────────────────────────────────────────────────

    def get_edge(self, from_id: str, to_id: str) -> 'Edge | None':
        return self.edges.get((from_id, to_id))

    def neighbors(self, node_id: str) -> list[str]:
        return self.adj.get(node_id, [])

    @property
    def bbox(self) -> tuple[float, float, float, float]:
        """(x_min, y_min, x_max, y_max) in mm."""
        xs = [n.x for n in self.nodes.values()]
        ys = [n.y for n in self.nodes.values()]
        return min(xs), min(ys), max(xs), max(ys)

    def __repr__(self):
        return (f'MapGraph({len(self.nodes)} nodes, '
                f'{len(self.edges)} edges, '
                f'{len(self.ports)} ports)')
