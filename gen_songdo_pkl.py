"""
gen_songdo_pkl.py — Generate collision-profile pkl from 송도공장 JSON layout.

The 송도 layout uses a simplified format:
  nodes: [{id, location: [x,y]}, ...]
  edges: [{id, start, end, length, type, bidirectional}, ...]
  port_nodes: [...]
  ports: [{id, location, ...}, ...]
  parameters: {amr_length, safety_gap, speed, ...}

All edges are straight (no arcs). The generated pkl contains:
  Stop_state   : {state_id -> State}   (S,node_id,heading)
  Move_state   : {state_id -> State}   (M,from_id,to_id)
  Rotate_state : {state_id -> State}   (R,node_id,from_angle,to_angle)
  stop_regions : {state_id -> Shapely Polygon}  agent footprint
  move_regions : {state_id -> Shapely Polygon}  swept area
  od_pairs     : [(src_node, dst_node), ...]

Usage:
    python gen_songdo_pkl.py
    python gen_songdo_pkl.py --json path/to/layout.json --out path/to/output.pkl
"""
from __future__ import annotations
import json, math, os, sys, pickle, argparse
from collections import defaultdict
from typing import Dict, List, Tuple, Set
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union

# ── State class (compatible with solver unpickling) ──────────────────────────

class State:
    def __init__(self, type, start_id=None, end_id=None, cost=None,
                 center=None, segment_id=None):
        self.id = type
        self.start_id = start_id
        self.end_id = end_id
        if type[0] == 'S':
            self.cost = 0
        elif type[0] == 'M':
            self.cost = cost if cost is not None else 1
        elif type[0] == 'R':
            a = type.split(',')
            angle_diff = abs(int(a[2]) - int(a[3]))
            if angle_diff == 270:
                angle_diff = 90
            self.cost = angle_diff / 90
        else:
            self.cost = cost if cost is not None else 0
        self.offset = []
        self.sr = 0
        self.sc = 0
        self.center = center
        self.interval_list = [(0, float('inf'))]
        self.type = None
        self.next_state = []
        self.affect_state = []
        self.rsv_veh_list = []
        self.rsv_time_table = []
        self.segment_id = segment_id
        self.heading = 0


# ── Geometry helpers ─────────────────────────────────────────────────────────

def _get_rectangle(cx, cy, width, length, center_of_veh=0):
    """Return 4 corner points of a rectangle centered at (cx, cy).
    length along x-axis, width along y-axis (before rotation)."""
    hl = length / 2
    hw = width / 2
    return [
        (cx - hl, cy - hw),
        (cx + hl, cy - hw),
        (cx + hl, cy + hw),
        (cx - hl, cy + hw),
    ]


def _rotate_rectangle(pts, center, angle_rad):
    """Rotate points around center by angle_rad."""
    cx, cy = center
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    result = []
    for px, py in pts:
        dx, dy = px - cx, py - cy
        result.append((cx + dx * cos_a - dy * sin_a,
                        cy + dx * sin_a + dy * cos_a))
    return result


def normalize_angle(angle_deg):
    return int(round(angle_deg)) % 360


def shortest_rotation(start_angle, end_angle):
    diff = normalize_angle(end_angle - start_angle)
    if diff > 180:
        diff -= 360
    return diff


# ── Layout loader ────────────────────────────────────────────────────────────

class SongdoLayout:
    def __init__(self, json_path: str):
        with open(json_path, 'r', encoding='utf-8') as f:
            self.layout = json.load(f)

        # Node coordinates — support both formats
        self.node_coords: Dict[str, Tuple[float, float]] = {}
        for n in self.layout['nodes']:
            if 'location' in n:
                loc = n['location']
                self.node_coords[n['id']] = (loc[0], loc[1])
            else:
                self.node_coords[n['id']] = (n['x'], n['y'])

        # Parameters — support both formats
        params = self.layout.get('parameters', {})
        vm = self.layout.get('vehicleModels', [{}])[0] if self.layout.get('vehicleModels') else {}
        dim = vm.get('dimension', {})

        self.amr_length = dim.get('length', params.get('amr_length', 20))
        self.amr_width = dim.get('width', params.get('amr_width', self.amr_length))
        self.safety_gap = params.get('safety_gap', 5.0)

        # Speed: from segments or parameters (mm/s in new format)
        if 'segments' in self.layout and self.layout['segments']:
            self.speed = self.layout['segments'][0].get('speed', 1000)
        else:
            self.speed = params.get('speed', 3)

        # Vehicle dimensions
        self.veh_length = self.amr_length
        self.veh_width = self.amr_width

        # Port nodes — support both formats
        if 'port_nodes' in self.layout:
            self.port_node_ids: Set[str] = set(self.layout['port_nodes'])
        else:
            self.port_node_ids: Set[str] = set(p['nodeId'] for p in self.layout.get('ports', []))

        # Edges — support both 'edges' and 'segments'
        if 'edges' in self.layout:
            self._edges = self.layout['edges']
        else:
            self._edges = []
            for s in self.layout.get('segments', []):
                self._edges.append({
                    'start': s['startNodeId'],
                    'end': s['endNodeId'],
                    'type': s.get('type', ''),
                    'speed': s.get('speed', self.speed),
                })

        # Build states
        self.Stop_state: Dict[str, State] = {}
        self.Move_state: Dict[str, State] = {}
        self._build_states()

    def _build_states(self):
        """Build Stop and Move states from edges."""
        pending_stop_states: Set[str] = set()

        for edge in self._edges:
            start = edge['start']
            end = edge['end']
            if start not in self.node_coords or end not in self.node_coords:
                continue

            coord1 = self.node_coords[start]
            coord2 = self.node_coords[end]
            dx = coord2[0] - coord1[0]
            dy = coord2[1] - coord1[1]
            dist = math.hypot(dx, dy)
            if dist < 0.01:
                continue

            heading_deg = normalize_angle(math.degrees(math.atan2(dy, dx)))
            move_id = f"M,{start},{end}"
            move_start = f"S,{start},{heading_deg}"
            move_end = f"S,{end},{heading_deg}"

            travel_time = dist / self.speed if self.speed > 0 else dist
            move_state = State(move_id, move_start, move_end, travel_time,
                               center=None, segment_id=0)
            move_state.next_state.append(move_end)
            self.Move_state[move_id] = move_state

            pending_stop_states.add(move_start)
            pending_stop_states.add(move_end)

        # Create stop states
        for stop_id in pending_stop_states:
            parts = stop_id.split(',')
            heading = int(parts[2])
            state = State(stop_id)
            state.heading = heading
            self.Stop_state[stop_id] = state

        # Create rotation states between different headings at same node
        node_to_headings: Dict[str, Set[int]] = defaultdict(set)
        for stop_id, state in self.Stop_state.items():
            node_id = stop_id.split(',')[1]
            node_to_headings[node_id].add(state.heading)

        rotation_speed = 90  # deg/s
        for node_id, headings in node_to_headings.items():
            headings_list = list(headings)
            for sa in headings_list:
                for ea in headings_list:
                    if sa != ea:
                        rotate_id = f"R,{node_id},{sa},{ea}"
                        move_start = f"S,{node_id},{sa}"
                        move_end = f"S,{node_id},{ea}"
                        angle_diff = abs(sa - ea)
                        if angle_diff > 180:
                            angle_diff = 360 - angle_diff
                        travel_time = angle_diff / rotation_speed
                        x, y = self.node_coords[node_id]
                        rs = State(rotate_id, move_start, move_end,
                                   travel_time, center=(x, y), segment_id=0)
                        rs.next_state.append(move_end)
                        self.Move_state[rotate_id] = rs

        # Wire stop -> move transitions
        for move in self.Move_state.values():
            start_id = move.start_id
            if start_id in self.Stop_state:
                self.Stop_state[start_id].next_state.append(move.id)

        print(f"Built {len(self.Stop_state)} stop states, "
              f"{len(self.Move_state)} move states")

    def _generate_polygons(self):
        """Generate Shapely polygons for all states."""
        half_len = self.veh_length / 2 + self.safety_gap
        half_wid = self.veh_width / 2 + self.safety_gap
        # Use padded dimensions for collision regions
        padded_length = self.veh_length + 2 * self.safety_gap
        padded_width = self.veh_width + 2 * self.safety_gap

        stop_regions: Dict[str, Polygon] = {}
        move_regions: Dict[str, Polygon] = {}

        # Stop regions (stationary footprint)
        for stop_id, state in self.Stop_state.items():
            node_id = stop_id.split(',')[1]
            if node_id not in self.node_coords:
                continue
            x, y = self.node_coords[node_id]
            heading_rad = math.radians(state.heading)
            rect = _get_rectangle(x, y, padded_width, padded_length)
            rotated = _rotate_rectangle(rect, (x, y), heading_rad)
            stop_regions[stop_id] = Polygon(rotated)

        # Move regions (swept area between two nodes)
        for move_id, state in self.Move_state.items():
            parts = move_id.split(',')
            if parts[0] == 'R':
                # Rotation: union of footprint at start and end angles
                node_id = parts[1]
                if node_id not in self.node_coords:
                    continue
                x, y = self.node_coords[node_id]
                sa = int(parts[2])
                ea = int(parts[3])
                shapes = []
                n_steps = max(4, abs(shortest_rotation(sa, ea)) // 10)
                for i in range(n_steps + 1):
                    t = i / n_steps
                    angle = sa + shortest_rotation(sa, ea) * t
                    angle_rad = math.radians(angle)
                    rect = _get_rectangle(x, y, padded_width, padded_length)
                    rotated = _rotate_rectangle(rect, (x, y), angle_rad)
                    shapes.append(Polygon(rotated))
                move_regions[move_id] = unary_union(shapes)
            elif parts[0] == 'M':
                from_node = parts[1]
                to_node = parts[2]
                if from_node not in self.node_coords or to_node not in self.node_coords:
                    continue
                x1, y1 = self.node_coords[from_node]
                x2, y2 = self.node_coords[to_node]
                dx = x2 - x1
                dy = y2 - y1
                heading_rad = math.atan2(dy, dx)
                mx = (x1 + x2) / 2
                my = (y1 + y2) / 2
                seg_length = math.hypot(dx, dy)
                swept_length = seg_length + padded_length
                rect = _get_rectangle(mx, my, padded_width, swept_length)
                rotated = _rotate_rectangle(rect, (mx, my), heading_rad)
                move_regions[move_id] = Polygon(rotated)

        return stop_regions, move_regions

    def _compute_collision_profile(self, stop_regions, move_regions):
        """Find overlapping state pairs (affect_state)."""
        all_regions = {}
        all_regions.update(stop_regions)
        all_regions.update(move_regions)

        keys = list(all_regions.keys())
        n = len(keys)
        collision_profile: Dict[str, List[str]] = {k: [] for k in keys}

        print(f"Computing collision profiles for {n} states...")

        # Use STRtree for faster intersection queries
        from shapely import STRtree
        tree = STRtree([all_regions[k] for k in keys])

        for i, key_i in enumerate(keys):
            poly_i = all_regions[key_i]
            # Query tree for candidates
            candidates = tree.query(poly_i)
            for j in candidates:
                if j <= i:
                    continue
                key_j = keys[j]
                if all_regions[key_j].intersects(poly_i):
                    collision_profile[key_i].append(key_j)
                    collision_profile[key_j].append(key_i)

            if (i + 1) % 100 == 0:
                print(f"  {i+1}/{n} done")

        # Set affect_state on State objects
        for sid, collisions in collision_profile.items():
            if sid in self.Stop_state:
                self.Stop_state[sid].affect_state = collisions
            elif sid in self.Move_state:
                self.Move_state[sid].affect_state = collisions

        print(f"Collision profile complete. Total pairs: "
              f"{sum(len(v) for v in collision_profile.values()) // 2}")
        return collision_profile

    def _generate_od_pairs(self) -> List[Tuple[str, str]]:
        """Generate OD pairs from port nodes (all port-to-port combinations)."""
        ports = sorted(self.port_node_ids)
        od_pairs = []
        for src in ports:
            for dst in ports:
                if src != dst:
                    od_pairs.append((src, dst))
        return od_pairs

    def save_pkl(self, pkl_path: str):
        """Generate and save the collision profile pkl."""
        print("Generating state polygons...")
        stop_regions, move_regions = self._generate_polygons()

        print(f"  {len(stop_regions)} stop regions, {len(move_regions)} move regions")

        collision_profile = self._compute_collision_profile(stop_regions, move_regions)

        od_pairs = self._generate_od_pairs()

        data = {
            'Stop_state': self.Stop_state,
            'Move_state': self.Move_state,
            'Rotate_state': {k: v for k, v in self.Move_state.items()
                             if k.startswith('R,')},
            'stop_regions': stop_regions,
            'move_regions': move_regions,
            'collision_profile': collision_profile,
            'od_pairs': od_pairs,
        }

        os.makedirs(os.path.dirname(pkl_path) or '.', exist_ok=True)
        with open(pkl_path, 'wb') as f:
            pickle.dump(data, f)
        print(f"Saved: {pkl_path}")
        print(f"  Stop states: {len(self.Stop_state)}")
        print(f"  Move states: {len(self.Move_state)}")
        print(f"  Stop regions: {len(stop_regions)}")
        print(f"  Move regions: {len(move_regions)}")
        print(f"  OD pairs: {len(od_pairs)}")


def main():
    ap = argparse.ArgumentParser(description='Generate collision-profile pkl from 송도공장 layout')
    ap.add_argument('--json', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '송도공장_Final_v0_bi.json'),
        help='path to 송도 JSON layout')
    ap.add_argument('--out', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '송도공장.pkl'),
        help='output pkl path')
    args = ap.parse_args()

    layout = SongdoLayout(args.json)
    layout.save_pkl(args.out)


if __name__ == '__main__':
    main()
