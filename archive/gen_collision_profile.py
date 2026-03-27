import sys as _sys, os as _os
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'utils'))
del _sys, _os

import networkx as nx
from collections import defaultdict, Counter
import json
import os
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely.affinity import rotate, translate
from shapley_region import get_rectangle, rotate_rectangle
import random
from matplotlib import cm
from matplotlib.colors import Normalize
import pickle

from matplotlib.patches import Arc
#Json 정보 기반으로map 생성


class State:
    def __init__(self,type,start_id=None, end_id=None, cost=None, center=None, segment_id=None): #type이 stop이면 input 1 = node, input 2 = angle, drive input 1,2는 각각 prev next node,
        self.id = type
        self.start_id = start_id
        self.end_id = end_id
        if type[0] == 'S':
            self.cost = 0
        if type[0] == 'M':
            self.cost = cost if cost is not None else 1
        if type[0] == 'R':
            a = type.split(',')
            Angle = abs(int(a[2])-int(a[3]))
            if Angle == 270:
                Angle = Angle - 180
            self.cost = Angle/90
        self.offset = []
        self.sr = 0
        self.sc = 0
        self.center = center
        self.interval_list = [(0,float('inf'))]
        self.type = None
        self.next_state = []
        self.affect_state = [] #물리적인 경로가 중복되는 list
        self.rsv_veh_list = []
        self.rsv_time_table = []
        self.segment_id = segment_id
#
# class StopState:
#     def __init__(self, id):
#         self.id = id
#         self.next_state = []
#         self.heading = None
#
#
# class MoveState:
#     def __init__(self, id, start_id, end_id, cost, center=None, segment_id=None):
#         self.id = id
#         self.start_id = start_id
#         self.end_id = end_id
#         self.cost = cost
#         self.center = center  # center of arc, if applicable
#         self.segment_id = segment_id  # corresponding segment ID
#         self.next_state = []
#

class CollisionProfile:
    def __init__(self, move_state, resolution=0.2):
        self.move_state = move_state
        self.resolution = resolution
        self.points = self.compute_profile()

    def compute_profile(self):
        if self.move_state.center:
            # Arc segment
            return self._arc_profile()
        else:
            # Straight segment
            return self._line_profile()

    def _line_profile(self):
        x1, y1 = layout.node_coords[self.move_state.start_id]
        x2, y2 = layout.node_coords[self.move_state.end_id]
        dist = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        num_points = max(2, int(dist / self.resolution))
        return [(x1 + (x2 - x1) * i / (num_points - 1),
                 y1 + (y2 - y1) * i / (num_points - 1)) for i in range(num_points)]

    def _arc_profile(self):
        from math import atan2, degrees, radians, cos, sin
        x1, y1 = layout.node_coords[self.move_state.start_id]
        x2, y2 = layout.node_coords[self.move_state.end_id]
        cx, cy = self.move_state.center

        angle1 = atan2(y1 - cy, x1 - cx)
        angle2 = atan2(y2 - cy, x2 - cx)

        angle1_deg = degrees(angle1)
        angle2_deg = degrees(angle2)

        angle1, angle2 = angle1 % (2 * np.pi), angle2 % (2 * np.pi)
        if self.move_state.segment_id > 0:
            if angle2 < angle1:
                angle2 += 2 * np.pi
            angles = np.linspace(angle1, angle2, int(abs(angle2 - angle1) * self.move_state.cost / self.resolution))
        else:
            if angle1 < angle2:
                angle1 += 2 * np.pi
            angles = np.linspace(angle1, angle2, int(abs(angle1 - angle2) * self.move_state.cost / self.resolution))

        r = ((x1 - cx) ** 2 + (y1 - cy) ** 2) ** 0.5
        return [(cx + r * cos(a), cy + r * sin(a)) for a in angles]


# def normalize_angle(angle):
#     return int(round(angle/90.0))*90 % 360

def normalize_angle(angle):
    return int(round(angle))% 360


def reverse_angle(angle):
    return (angle+180)%360

def get_node_num_from_id(node_id):
    parts = node_id.split(',')
    return parts[1]

def shortest_rotation(start_angle, end_angle):
    diff = normalize_angle(end_angle - start_angle)
    if diff > 180:
        diff -= 360
    return diff


def get_arc_heading(cx, cy, px, py, is_ccw):
    """Compute heading at a point (px, py) relative to center (cx, cy).
       If counterclockwise arc, rotate +90°, else rotate -90°."""
    angle_center_to_point = np.arctan2(py - cy, px - cx)  # vector from center to point
    if is_ccw:
        heading = angle_center_to_point + np.pi/2
    else:
        heading = angle_center_to_point - np.pi/2
    heading_deg = np.degrees(heading)
    return int(round(heading_deg % 360))

def generate_move_polygon(move_state, layout, agv_length=2630, agv_width=1380, resolution=0.3):
    if move_state.center:
        # Arc motion (rotation around center)
        x1, y1 = layout.node_coords[get_node_num_from_id(move_state.start_id)]
        x2, y2 = layout.node_coords[get_node_num_from_id(move_state.end_id)]
        cx, cy = move_state.center

        # angles from center to start and end
        # angle1 = np.arctan2(y1 - cy, x1 - cx)
        # angle2 = np.arctan2(y2 - cy, x2 - cx)
        angle1 = layout.Stop_state[move_state.start_id].heading
        angle2 = layout.Stop_state[move_state.end_id].heading
        angle = shortest_rotation(angle1, angle2) * np.pi / 180
        angle1 = angle1 * np.pi / 180
        angle2 = angle2 * np.pi / 180

        # print("Angle",angle2,angle1,angle)
        # if abs(np.degrees(angle2 - angle1)) > 180:
        #     if angle2 < 0:
        #         angle2 += 2*np.pi
        #     if angle1 < 0:
        #         angle1 += 2*np.pi
        # if move_state.start_id == '272' and move_state.end_id == '309':
        #     print('245246',int(np.degrees(angle1)),int(np.degrees(angle2)))
        # angle1, angle2 = angle1 % (2*np.pi), angle2 % (2*np.pi)
        # 메모 시계반시계 임의
        if move_state.segment_id > 0:
            # if angle2 < angle1:
            #     angle2 += 2 * np.pi
            angles = np.linspace(0, angle, abs(int(np.degrees(angle2 - angle1))))
        else:
            # if angle1 < angle2:
            #     angle1 += 2 * np.pi
            angles = np.linspace(0, angle, abs(int(np.degrees(angle1 - angle2))))
        # print(angles)
        # Generate and rotate AGV around center at each angle
        # if abs(np.degrees(angle2-angle1)) >=180:
        #     print("Angle problem")
        #     print(np.degrees(angle1),np.degrees(angle2),move_state.id)
        shapes = []
        radius = ((x1 - cx) ** 2 + (y1 - cy) ** 2) ** 0.5
        rect = get_rectangle(x1, y1, agv_width, agv_length, center_of_veh=0)
        rect = rotate_rectangle(rect, (x1, y1), layout.Stop_state[move_state.start_id].heading * np.pi / 180)
        for a in angles:
            # px = cx + radius * np.cos(a)
            # py = cy + radius * np.sin(a)
            # heading = np.degrees(a)
            # rect = get_rectangle(px, py, agv_width, agv_length, center_of_veh=0)
            rotated = Polygon(rotate_rectangle(rect, (cx, cy), a))
            shapes.append(rotated)

        return unary_union(shapes)

    else:
        # Straight motion as a single swept rectangle
        x1, y1 = layout.node_coords[get_node_num_from_id(move_state.start_id)]
        x2, y2 = layout.node_coords[get_node_num_from_id(move_state.end_id)]
        dx = x2 - x1
        dy = y2 - y1
        heading = np.arctan2(dy, dx)
        mx = (x1 + x2) / 2
        my = (y1 + y2) / 2
        length = ((dx) ** 2 + (dy) ** 2) ** 0.5
        rect = get_rectangle(mx, my, agv_width, length + agv_length, center_of_veh=0)
        rotated = Polygon(rotate_rectangle(rect, (mx, my), heading))
        return rotated


def stop_state_to_polygon(stop_state, layout, agv_length=2630, agv_width=1380):
    x, y = layout.node_coords[get_node_num_from_id(stop_state.id)]
    heading = getattr(stop_state, 'heading', 0)
    rect = get_rectangle(x, y, agv_width, agv_length, center_of_veh=0)
    rotated = Polygon(rotate_rectangle(rect, (x, y), heading * np.pi / 180))
    return rotated


# --- 멀티프로세싱 워커 (Windows spawn에서 pickle 되려면 모듈 최상위에 있어야 함) ---
def _move_polygon_worker(args):
    from types import SimpleNamespace
    move_state, node_coords, stop_headings, agv_length, agv_width = args
    mock_layout = SimpleNamespace(
        node_coords=node_coords,
        Stop_state={sid: SimpleNamespace(heading=h) for sid, h in stop_headings.items()}
    )
    return move_state.id, generate_move_polygon(move_state, mock_layout, agv_length, agv_width)


def _stop_polygon_worker(args):
    from types import SimpleNamespace
    stop_state, node_coords, agv_length, agv_width = args
    mock_layout = SimpleNamespace(node_coords=node_coords)
    return stop_state.id, stop_state_to_polygon(stop_state, mock_layout, agv_length, agv_width)


class LayoutManager:
    def __init__(self,directory,pickle=None):
        self.Stop_state = {}
        self.Move_state = {}
        with open(layout_filename, "r") as f:
            layout_json = json.load(f)
        if pickle:
            self.load_layout(directory)
            self.load_collision_profile(pickle)
        else:
            self.load_layout(directory)
            self.stop_regions = {}
            self.move_regions = {}
            self.collision_profile = {}
            self._generate_state_polygons_and_collisions()


            self.fill_affect_state()
        self.compute_critical_move_frequencies(layout_json)
        self.always_critical = self.get_always_critical_moves_per_origin()

    def load_layouts_from_directory(self, directory_path):
        for filename in os.listdir(directory_path):
            if filename.endswith(".json"):
                full_path = os.path.join(directory_path, filename)
                print(f"Loading layout from: {filename}")
                self.load_layout(full_path)

    def load_layout(self, json_path):
        with open(json_path, 'r') as f:
            layout = json.load(f)
            self.layout = layout
        self.port_node_ids = {port["nodeId"] for port in layout.get("ports", [])}
        if "vehicleModels" in layout and len(layout["vehicleModels"]) > 0:
            model = layout["vehicleModels"][0]  # Assume the first model
            self.width = model.get("dimension", {}).get("width", 1380)
            self.length = model.get("dimension", {}).get("length", 2630)
            print(f"Loaded vehicle size: width={self.width}, length={self.length}")
        else:
            self.width = 1380
            self.length = 2630
            print(f"No vehicle model found, using default size.")
        print("vehwidth,length", self.width, self.length)

        self.Stop_state = {}
        self.Move_state = {}
        self.node_coords = {node['id']: (node['x'], node['y']) for node in layout['nodes']}
        segment_by_id = {s['id']: s for s in layout['segments'] if 'id' in s}

        pending_stop_states = set()

        for segment in layout['segments']:
            if 'startNodeId' not in segment or 'endNodeId' not in segment:
                print(f"Skipping segment without start or end: {segment}")
                continue
            start = segment['startNodeId']
            end = segment['endNodeId']
            heading = segment.get('heading', None)
            if segment['id'] < 0:
                original = segment_by_id.get(abs(segment['id']))
                parts = original.get('parts', []) if original else []
            else:
                parts = segment.get('parts', [])
            is_arc = any(p.get('kind') == 'Arc' for p in parts)

            move_id = f"M,{start},{end}"
            coord1 = self.node_coords[start]
            coord2 = self.node_coords[end]
            dx = coord2[0] - coord1[0]
            dy = coord2[1] - coord1[1]
            heading_angle = np.degrees(np.arctan2(dy, dx))
            move_start = f"S,{start},{int(float(normalize_angle(heading_angle)))}"
            move_end = f"S,{end},{int(float(normalize_angle(heading_angle)))}"

            if is_arc:
                move_id = f"M,{start},{end}"
                coord1 = self.node_coords[start]
                coord2 = self.node_coords[end]

                if start == '291':
                    print('here')


                dx = coord2[0] - coord1[0]
                dy = coord2[1] - coord1[1]
                mx = (coord1[0] + coord2[0]) / 2
                my = (coord1[1] + coord2[1]) / 2
                length = ((dx) ** 2 + (dy) ** 2) ** 0.5
                perp_dx = -(dy / length)
                perp_dy = dx / length
                if segment['id'] > 0:
                    is_ccw = True
                    cx = mx + (perp_dx * (length / 2))
                    cy = my + (perp_dy * (length / 2))
                else:
                    is_ccw = False
                    cx = mx - (perp_dx * (length / 2))
                    cy = my - (perp_dy * (length / 2))
                center = (cx, cy)

                heading_angle1 = get_arc_heading(cx,cy,coord1[0],coord1[1],is_ccw)
                heading_angle2 = get_arc_heading(cx,cy,coord2[0],coord2[1],is_ccw)

                if heading:
                    heading_angle1 = reverse_angle(heading_angle1)
                    heading_angle2 = reverse_angle(heading_angle2)


                move_start = f"S,{start},{int(float(normalize_angle(heading_angle1)))}"
                move_end = f"S,{end},{int(float(normalize_angle(heading_angle2)))}"
                chord = (dx ** 2 + dy ** 2) ** 0.5
                radius = chord / (2 ** 0.5)
                distance = (np.pi / 2) * radius
                pending_stop_states.add(move_start)
                pending_stop_states.add(move_end)

            else:
                move_id = f"M,{start},{end}"
                coord1 = self.node_coords[start]
                coord2 = self.node_coords[end]
                dx = coord2[0] - coord1[0]
                dy = coord2[1] - coord1[1]
                heading_angle = np.degrees(np.arctan2(dy, dx))
                if start =="128":
                    print(start,heading_angle)
                # if heading:
                #     heading_angle = reverse_angle(heading_angle)
                if heading:
                    heading_angle += heading[0]["degree"]
                move_start = f"S,{start},{int(float(normalize_angle(heading_angle)))}"
                move_end = f"S,{end},{int(float(normalize_angle(heading_angle)))}"
                distance = (dx ** 2 + dy ** 2) ** 0.5
                center = None
                pending_stop_states.add(move_start)
                pending_stop_states.add(move_end)

            speed = segment.get('speed', 1.0)
            travel_time = distance / speed

            move_state = State(move_id, move_start, move_end, travel_time, center, segment_id=segment['id'])
            move_state.next_state.append(move_end)

            self.Move_state[move_id] = move_state


        for stop_id in pending_stop_states:
            node_id, heading = stop_id.rsplit(',', 1)
            state = State(stop_id)
            state.heading = int(float(heading))
            self.Stop_state[stop_id] = state


        node_to_headings = defaultdict(set)
        for stop_id in self.Stop_state:
            node_id, heading = stop_id.split(',')[1], int(self.Stop_state[stop_id].heading)
            node_to_headings[node_id].add(heading)

        # 2. For each node, create rotation MoveStates between all pairs of headings
        rotation_speed_deg_per_sec = 90  # assume AGV rotates 90 degrees per second

        #중요 rotation 불가능한 vehicle에선 이거 하면 안됨
        for node_id, headings in node_to_headings.items():
            headings = list(headings)
            for start_angle in headings:
                for end_angle in headings:
                    if start_angle != end_angle:
                        rotate_id = f"R,{node_id},{start_angle},{end_angle}"
                        move_start = f"S,{node_id},{start_angle}"
                        move_end = f"S,{node_id},{end_angle}"

                        # Node coordinate
                        x, y = self.node_coords[node_id]

                        # Rotation center is (x,y)
                        center = (x, y)

                        # Rotation cost proportional to rotation angle
                        angle_diff = abs(start_angle - end_angle)
                        if angle_diff > 180:
                            angle_diff = 360 - angle_diff
                        travel_time = angle_diff / rotation_speed_deg_per_sec

                        move_state = State(
                            type=rotate_id,
                            start_id=move_start,
                            end_id=move_end,
                            cost=travel_time,
                            center=center,
                            segment_id=0  # Same as rotation id
                        )
                        move_state.next_state.append(move_end)
                        self.Move_state[rotate_id] = move_state

        for move in self.Move_state.values():
            self.Stop_state[move.start_id].next_state.append(move.id)
        print(            f"Loaded {len(self.Stop_state)} stop states and {len(self.Move_state)} move states from {os.path.basename(json_path)}.")

    def _affect_aggregate(self, base_map, include_self=True, only_moves=True):
        """
        base_map: {state_id(or move_id): value}
          예) self.move_freq, self.edge_bc_move_ports, self.edge_bc_move 등
        collision_profile를 이용해 '겹치는 상태'로 값을 전파해 누적.
        include_self=True면 자신도 누적 대상.
        only_moves=True면 'M,*' 상태만 결과에 남김(시각화가 move polygon 기준일 때 권장).
        반환: {state_id(or move_id): affect-누적 값}
        """
        acc = Counter()
        if not isinstance(base_map, dict) or not base_map:
            return {}

        for sid, val in base_map.items():
            if not val:
                continue
            # 자신
            if include_self:
                if (not only_moves) or (isinstance(sid, str) and sid.startswith('M')):
                    acc[sid] += val
            # 겹치는 상태들
            for s2 in self.collision_profile.get(sid, []):
                if only_moves and not (isinstance(s2, str) and s2.startswith('M')):
                    continue
                acc[s2] += val

        return dict(acc)

    def _collect_port_stop_states(self):
        """
        JSON(self.layout)에서 ports를 추출하고, 해당 포트에 '도착하는' move의 end_id(=stop-state)를 포트 stop-state로 수집.
        compute_critical_move_frequencies에서 쓰던 로직과 동일한 기준.
        반환: list of stop_id (예: 'S,265,0')
        """
        segments = self.layout["segments"]
        self.port_node_ids = {port["nodeId"] for port in self.layout.get("ports", [])}

        port_states = set()
        for seg in segments:
            end_node = seg["endNodeId"]
            if end_node not in self.port_node_ids:
                continue
            start_node = seg["startNodeId"]
            move_state_id = f"M,{start_node},{end_node}"
            if move_state_id in self.Move_state:
                port_states.add(self.Move_state[move_state_id].end_id)

        return list(port_states)

    def compute_edge_betweenness_ports(self, use_cost_weight=True, directed=True, normalized=True):
        """
        포트 stop-state 집합(균일 수요)으로 subset betweenness를 계산.
        반환: {move_id: bc_value}
        """
        import networkx as nx

        ports = self._collect_port_stop_states()
        # stop->stop 그래프 구성 (평행간선은 최소비용 하나로 집계)
        G = nx.DiGraph() if directed else nx.Graph()
        best = {}  # key: (u,v) 또는 tuple(sorted((u,v))) -> (weight, move_id)

        for stop_id, stop_state in self.Stop_state.items():
            for move_id in getattr(stop_state, "next_state", []):
                mv = self.Move_state.get(move_id)
                if not mv:
                    continue
                for next_stop_id in mv.next_state:
                    w = float(mv.cost) if use_cost_weight else 1.0
                    key = (stop_id, next_stop_id) if directed else tuple(sorted((stop_id, next_stop_id)))
                    if key not in best or w < best[key][0]:
                        best[key] = (w, move_id)

        for (u, v), (w, mid) in best.items():
            G.add_edge(u, v, weight=w, move_id=mid)

        # 그래프에 실제로 존재하는 포트들만
        ports_in_G = [p for p in ports if p in G]
        if len(ports_in_G) < 2:
            edge_bc_move = {mid: 0.0 for mid in self.Move_state.keys()}
            self.edge_bc_move_ports = edge_bc_move
            return edge_bc_move

        weight_key = 'weight' if use_cost_weight else None
        ebc = nx.edge_betweenness_centrality_subset(
            G,
            sources=ports_in_G,
            targets=ports_in_G,
            weight=weight_key,
            normalized=normalized
        )

        # 간선 -> move_id 매핑
        edge_bc_move = {data['move_id']: ebc[(u, v)] for u, v, data in G.edges(data=True)}
        for mid in self.Move_state.keys():  # 누락 채움
            edge_bc_move.setdefault(mid, 0.0)

        self.edge_bc_move_ports = edge_bc_move
        self.edge_bc_graph_directed = directed
        self.edge_bc_graph_weighted = use_cost_weight
        return edge_bc_move

    def visualize_port_bc_vs_cutfreq(self,
                                     title_left="Port-OD Edge Betweenness (affect-aggregated)",
                                     title_right="Critical Affect Frequency",
                                     cmap_name="Reds",
                                     use_p95=True):
        import numpy as np
        import matplotlib.pyplot as plt
        from matplotlib import cm
        from matplotlib.colors import Normalize

        # 1) 값 준비
        if not hasattr(self, "edge_bc_move_ports"):
            self.compute_edge_betweenness_ports(use_cost_weight=True, directed=True, normalized=True)

        # 왼쪽: betweenness의 affect-누적
        bc_map = dict(self.edge_bc_move_ports)
        bc_affect = self._affect_aggregate(bc_map, include_self=True, only_moves=True)

        # 오른쪽: critical frequency의 affect-누적
        cut_freq = getattr(self, "move_freq", {})  # {critical_move: count}
        cut_affect = self._affect_aggregate(cut_freq, include_self=True, only_moves=True)

        # 2) 패널별 정규화(각각 별도 vmax)
        def _vmax(vals):
            arr = np.array(list(vals.values()), dtype=float)
            if arr.size == 0:
                return 1.0
            if use_p95 and (arr > 0).any():
                vm = float(np.quantile(arr, 0.95))
                if vm <= 0: vm = float(arr.max())
            else:
                vm = float(arr.max())
            return vm if vm > 0 else 1.0

        vmax_left = _vmax(bc_affect)
        vmax_right = _vmax(cut_affect)

        norm_left = Normalize(vmin=0.0, vmax=vmax_left)
        norm_right = Normalize(vmin=0.0, vmax=vmax_right)
        cmap = cm.get_cmap(cmap_name)

        # 3) 그림
        fig, axes = plt.subplots(1, 2, figsize=(14, 7), constrained_layout=True)
        pairs = [
            (axes[0], bc_affect, norm_left, f"{title_left}  (max≈{vmax_left:.3g})"),
            (axes[1], cut_affect, norm_right, f"{title_right} (max≈{vmax_right:.3g})"),
        ]

        for ax, metric_map, norm, panel_title in pairs:
            for mid, poly in self.move_regions.items():
                val = float(metric_map.get(mid, 0.0))
                if getattr(poly, "is_empty", False):
                    continue
                if getattr(poly, "geom_type", "") == "Polygon":
                    x, y = poly.exterior.xy
                    ax.fill(x, y, alpha=(0.3 if val > 0 else 0.05), fc=cmap(norm(val)), ec='none')

            # 방향 화살표
            for mv in self.Move_state.values():
                x1, y1 = self.node_coords[get_node_num_from_id(mv.start_id)]
                x2, y2 = self.node_coords[get_node_num_from_id(mv.end_id)]
                ax.arrow(x1, y1, x2 - x1, y2 - y1, head_width=0.15, length_includes_head=True,
                         color='0.5', alpha=0.3)
            ax.set_aspect('equal');
            ax.grid(False);
            ax.set_title(panel_title)

            # 패널별 컬러바
            sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm);
            sm.set_array([])
            cbar = fig.colorbar(sm, ax=ax, fraction=0.046, pad=0.04)
            cbar.set_label('Normalized intensity (per panel)', rotation=270, labelpad=12)

        fig.suptitle("Port-OD Betweenness (affect) vs Critical Affect Frequency — per-panel normalization")
        plt.show()

    def compute_edge_betweenness(self, use_cost_weight=True):
        """
        Stop-state를 노드로, (stop->stop) 전이(edge)를 간선으로 갖는 DiGraph를 만든 뒤,
        엣지-베트위니스 중심성(가중 최단경로 기준)을 계산하여 move_id 단위로 저장.
        self.edge_bc_move: {move_id: bc in [0,1]}
        self.node_bc:      {stop_id: node betweenness in [0,1]}  (참고용)
        """
        G = nx.DiGraph()
        edge_to_move_id = {}

        # stop->move->next_stop 구조에서 stop->next_stop 간선을 만들고 move 비용을 weight로 둔다
        for stop_id, stop_state in self.Stop_state.items():
            for move_id in stop_state.next_state:
                if move_id in self.Move_state:
                    mv = self.Move_state[move_id]
                    for next_stop_id in mv.next_state:
                        w = float(mv.cost) if use_cost_weight else 1.0
                        G.add_edge(stop_id, next_stop_id, weight=w)
                        edge_to_move_id[(stop_id, next_stop_id)] = move_id

        # 엣지 중심성 (가중 최단경로 기반). normalized=True 이면 [0,1] 범위 근사
        ebc = nx.edge_betweenness_centrality(G, normalized=True,
                                             weight=('weight' if use_cost_weight else None))
        nbc = nx.betweenness_centrality(G, normalized=True,
                                        weight=('weight' if use_cost_weight else None))

        # move_id 단위로 매핑
        edge_bc_move = {}
        for (u, v), c in ebc.items():
            mid = edge_to_move_id.get((u, v))
            if mid is not None:
                # 여러 간선이 동일 move_id 로 매핑될 일은 거의 없지만, 있을 경우 최대값 사용
                edge_bc_move[mid] = max(c, edge_bc_move.get(mid, 0.0))

        # 누락된 move는 0으로 채움 (시각화 편의)
        for mid in self.Move_state.keys():
            if mid not in edge_bc_move:
                edge_bc_move[mid] = 0.0

        self.edge_bc_move = edge_bc_move
        self.node_bc = nbc
        return edge_bc_move, nbc

    def _normalize_metric(self, dct):
        """값 사분위-외란을 줄이려고 95퍼센타일로 스케일링 후 [0,1]로 클리핑."""
        if not dct:
            return dct
        vals = np.array(list(dct.values()), dtype=float)
        vmax = np.quantile(vals, 0.95) if np.any(vals > 0) else 1.0
        vmax = float(vmax) if vmax > 0 else 1.0
        return {k: min(v / vmax, 1.0) for k, v in dct.items()}

    def visualize_bc_vs_cutfreq(self, title_left="Edge Betweenness", title_right="Cut-State Frequency"):
        """
        좌: 엣지-베트위니스 중심성 히트맵 (move polygon fill)
        우: cut-state 빈도 히트맵 (self.move_freq 기반; port 수요 균일 가정)
        둘 다 동일 레이아웃 타일 위에 그려 비교.
        """
        # 1) 중심성 없으면 계산
        if not hasattr(self, "edge_bc_move"):
            self.compute_edge_betweenness(use_cost_weight=True)

        # 2) cut-state 빈도 (이미 self.move_freq가 compute_critical_move_frequencies 에서 계산됨)
        cut_freq = getattr(self, "move_freq", {})
        # 빈도 정규화
        if cut_freq:
            maxf = max(cut_freq.values())
            cut_norm = {k: (v / maxf if maxf > 0 else 0.0) for k, v in cut_freq.items()}
        else:
            cut_norm = {k: 0.0 for k in self.Move_state.keys()}

        # 3) 이상치에 둔감한 스케일
        bc_norm = self._normalize_metric(self.edge_bc_move)
        cut_norm = self._normalize_metric(cut_norm)

        # 4) 그림 준비
        fig, axes = plt.subplots(1, 2, figsize=(14, 7), constrained_layout=True)
        cmaps = [cm.viridis, cm.Reds]
        panels = [(axes[0], bc_norm, title_left), (axes[1], cut_norm, title_right)]

        for ax, metric_map, panel_title in panels:
            # move 영역 채우기
            # 값이 클수록 진하게 (alpha는 0.1~0.9 사이로도 조절)
            for mid, poly in self.move_regions.items():
                v = metric_map.get(mid, 0.0)
                if poly.is_empty:
                    continue
                if poly.geom_type == "Polygon":
                    x, y = poly.exterior.xy
                    color = cmaps[panels.index((ax, metric_map, panel_title))](v)  # cmap 적용
                    ax.fill(x, y, alpha=0.75 if v > 0 else 0.05, fc=color, ec='none')

            # stop-state 노드 라벨(번호) 얹기
            for stop_id in self.Stop_state:
                x, y = self.node_coords[get_node_num_from_id(stop_id)]
                ax.text(x, y + 0.3, get_node_num_from_id(stop_id), fontsize=7, ha='center')

            # 방향성 화살표(베이스 지오메트리)는 연한 회색으로
            for mv in self.Move_state.values():
                x1, y1 = self.node_coords[get_node_num_from_id(mv.start_id)]
                x2, y2 = self.node_coords[get_node_num_from_id(mv.end_id)]
                ax.arrow(x1, y1, x2 - x1, y2 - y1, head_width=0.15, length_includes_head=True,
                         color='0.5', alpha=0.3)

            ax.set_aspect('equal')
            ax.set_title(panel_title)
            ax.grid(False)

            # 컬러바
            sm = plt.cm.ScalarMappable(cmap=cmaps[panels.index((ax, metric_map, panel_title))],
                                       norm=Normalize(vmin=0, vmax=1))
            sm.set_array([])
            cbar = fig.colorbar(sm, ax=ax, fraction=0.046, pad=0.04)
            cbar.set_label('Normalized intensity', rotation=270, labelpad=12)

        fig.suptitle("Betweenness vs Cut-State Frequency (uniform port demand)")
        plt.show()

    def _generate_state_polygons_and_collisions(self, agv_length=2630, agv_width=1380):
        from rtree import index as rtree_index
        from concurrent.futures import ProcessPoolExecutor
        import os

        node_coords = self.node_coords
        stop_headings = {sid: s.heading for sid, s in self.Stop_state.items()}
        agv_length, agv_width = self.length, self.width

        # 폴리곤 생성 — CPU 병렬
        move_args = [(ms, node_coords, stop_headings, agv_length, agv_width)
                     for ms in self.Move_state.values()]
        stop_args = [(ss, node_coords, agv_length, agv_width)
                     for ss in self.Stop_state.values()]

        n_workers = max(1, os.cpu_count() or 1)
        print(f"  폴리곤 생성 중... move={len(move_args)}, stop={len(stop_args)}, workers={n_workers}")
        with ProcessPoolExecutor(max_workers=n_workers) as executor:
            for move_id, poly in executor.map(_move_polygon_worker, move_args):
                self.move_regions[move_id] = poly
        with ProcessPoolExecutor(max_workers=n_workers) as executor:
            for stop_id, poly in executor.map(_stop_polygon_worker, stop_args):
                self.stop_regions[stop_id] = poly

        # 충돌 감지 — rtree 공간 인덱스로 후보 사전 필터링 후 정확한 intersects 검사
        all_regions = {**self.stop_regions, **self.move_regions}
        # 빈/degenerate 폴리곤 제거 (1도 미만 회전 등)
        _empty = [sid for sid, poly in all_regions.items()
                  if poly.is_empty or poly.bounds[0] > poly.bounds[2]]
        for sid in _empty:
            all_regions.pop(sid)
            self.stop_regions.pop(sid, None)
            self.move_regions.pop(sid, None)
        if _empty:
            print(f"  Skipped {len(_empty)} empty polygons: {_empty[:4]}...")
        id_list = list(all_regions.keys())
        sid_to_int = {sid: i for i, sid in enumerate(id_list)}

        idx = rtree_index.Index()
        for sid, poly in all_regions.items():
            idx.insert(sid_to_int[sid], poly.bounds)

        import time
        from concurrent.futures import ThreadPoolExecutor
        from collections import defaultdict
        from shapely.prepared import prep

        # Phase 1: rtree로 후보 쌍 수집 (빠름) — 총 검사량 파악
        print(f"  [1/2] rtree 후보 필터링 중... 총 {len(all_regions)}개 상태")
        t0 = time.time()
        seen_pairs = set()
        groups = defaultdict(list)  # sid -> [검사할 oid 목록]
        for sid in all_regions:
            for cid in idx.intersection(all_regions[sid].bounds):
                oid = id_list[cid]
                if oid == sid:
                    continue
                key = (sid, oid) if sid < oid else (oid, sid)
                if key not in seen_pairs:
                    seen_pairs.add(key)
                    groups[key[0]].append(key[1])
        total_pairs = sum(len(v) for v in groups.values())
        print(f"  [1/2] done ({time.time()-t0:.1f}s) - candidate pairs: {total_pairs:,}")

        # Phase 2: intersects 병렬 검사 (prep으로 반복 검사 가속 + 스레드 병렬)
        self.collision_profile = {sid: [] for sid in all_regions}
        done_pairs = 0
        collision_count = 0
        t_start = time.time()
        t_last_print = t_start
        print(f"  [2/2] intersects 병렬 검사 시작 (workers={n_workers})...")

        def check_group(item):
            sid, oids = item
            prepared = prep(all_regions[sid])
            return sid, [oid for oid in oids if prepared.intersects(all_regions[oid])]

        group_items = list(groups.items())
        with ThreadPoolExecutor(max_workers=n_workers) as executor:
            for sid, hit_oids in executor.map(check_group, group_items):
                for oid in hit_oids:
                    self.collision_profile[sid].append(oid)
                    self.collision_profile[oid].append(sid)
                    collision_count += 1
                done_pairs += len(groups[sid])

                now = time.time()
                if now - t_last_print >= 60:
                    elapsed = now - t_start
                    rate = done_pairs / elapsed if elapsed > 0 else 0
                    remaining = (total_pairs - done_pairs) / rate if rate > 0 else float('inf')
                    print(f"  [충돌감지] {done_pairs:,}/{total_pairs:,} 쌍 검사"
                          f" | 경과 {elapsed/60:.1f}분"
                          f" | 예상 잔여 {remaining/60:.1f}분"
                          f" | 충돌쌍 {collision_count:,}개")
                    t_last_print = now

        elapsed = time.time() - t_start
        print(f"  [충돌감지 완료] {total_pairs:,}쌍 검사 | 총 {elapsed/60:.1f}분 | 충돌쌍 {collision_count:,}개")

    def visualize_layout(self):
        plt.figure(figsize=(10, 10))
        coords = {}

        stop_id = random.choice(list(self.stop_regions.keys()))

        straight_ids = [k for k, v in self.Move_state.items() if v.center is None]
        # arc_ids = [k for k, v in self.Move_state.items() if v.center is not None]
        arc_ids = [k for k, v in self.Move_state.items() if v.center is not None]
        straight_id = random.choice(straight_ids)
        arc_id = random.choice(arc_ids)

        print(stop_id, straight_id, arc_id)
        stop_poly = self.stop_regions[stop_id]
        straight_poly = self.move_regions[straight_id]
        arc_poly = self.move_regions[arc_id]

        print(straight_poly, stop_poly, arc_poly)
        # Plot
        fig, ax = plt.subplots(figsize=(10, 10))
        for poly, color, label in zip(
                [stop_poly, straight_poly, arc_poly],
                ['green', 'blue', 'red'],
                ['Stop State', 'Straight Move', 'Arc Move']):
            if poly.geom_type == 'Polygon':
                x, y = poly.exterior.xy
                ax.fill(x, y, alpha=0.5, fc=color, label=label)

        for node_id in self.Stop_state:
            x, y = self.node_coords[get_node_num_from_id(node_id)]
            coords[node_id] = (x, y)
            # plt.plot(x, y, 'ko')
            plt.text(x, y + 0.3, get_node_num_from_id(node_id), fontsize=8, ha='center')

        for move in self.Move_state.values():
            x1, y1 = coords[move.start_id]
            x2, y2 = coords[move.end_id]

            if move.center:
                from matplotlib.patches import Arc
                cx, cy = move.center
                dx1 = x1 - cx
                dy1 = y1 - cy
                dx2 = x2 - cx
                dy2 = y2 - cy
                start_angle = np.degrees(np.arctan2(dy1, dx1)) % 360
                end_angle = np.degrees(np.arctan2(dy2, dx2)) % 360
                radius = (dx1 ** 2 + dy1 ** 2) ** 0.5

                if move.segment_id > 0:
                    # print(move.segment_id)
                    # print(start_angle, end_angle)
                    # plt.plot(cx, cy, 'ro', markersize=4)  # Visualize arc center as red dot/
                    arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                              theta1=start_angle, theta2=end_angle,
                              color='red', lw=2, alpha=0.6)
                    plt.gca().add_patch(arc)
                else:
                    # plt.plot(cx, cy, 'go', markersize=4)  # Visualize arc center as red dot
                    arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                              theta1=end_angle, theta2=start_angle,
                              color='green', lw=2, alpha=0.6)
                    plt.gca().add_patch(arc)

            else:
                if move.segment_id > 0:
                    plt.arrow(x1, y1, x2 - x1, y2 - y1,
                              head_width=0.2, length_includes_head=True,
                              color='blue', alpha=0.6)
                else:
                    plt.arrow(x1, y1, x2 - x1, y2 - y1,
                              head_width=0.2, length_includes_head=True,
                              color='orange', alpha=0.6)

        plt.axis('equal')
        plt.title("Warehouse Layout")
        plt.xlabel("X")
        # plt.ylim(70000, 80000)
        plt.ylabel("Y")
        plt.grid(True)
        plt.show()

    def visualize_layout_with_heatmap(self, move_freq=None):
        plt.figure(figsize=(10, 10))
        coords = {}

        fig, ax = plt.subplots(figsize=(10, 10))
        # Normalize move frequency values

        # norm = Normalize(vmin=0, vmax=max(self.move_freq.values()) if move_freq else 1)
        # cmap = cm.get_cmap("Reds")
        for node_id in self.Stop_state:
            parts = node_id.split(',')
            x, y = self.node_coords[parts[1]]
            coords[node_id] = (x, y)
            # plt.plot(x, y, 'ko')
            plt.text(x, y + 0.3, parts[1], fontsize=8, ha='center')

        for move in self.Move_state.values():

            freq = self.move_freq.get(move.id, 0) if move_freq else 0

            # norm = Normalize(vmin=0, vmax=max(self.move_freq.values()) if move_freq else 1)
            affected_critical = self.collision_profile[move.id]

            if move.id in self.move_freq:
                color = 'red'
                for affect_states in affected_critical:
                    if affect_states[0] == 'S':
                        continue
                    move1 = self.Move_state[affect_states]
                    x1, y1 = coords[move1.start_id]
                    x2, y2 = coords[move1.end_id]
                    if move1.center:
                        cx, cy = move1.center
                        dx1 = x1 - cx
                        dy1 = y1 - cy
                        dx2 = x2 - cx
                        dy2 = y2 - cy
                        start_angle = np.degrees(np.arctan2(dy1, dx1)) % 360
                        end_angle = np.degrees(np.arctan2(dy2, dx2)) % 360
                        radius = (dx1 ** 2 + dy1 ** 2) ** 0.5

                        if move1.segment_id > 0:
                            # print(move.segment_id)
                            # print(start_angle, end_angle)
                            # plt.plot(cx, cy, 'ro', markersize=4)  # Visualize arc center as red dot/
                            arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                                      theta1=start_angle, theta2=end_angle,
                                      color=color, lw=2, alpha=0.1)
                            plt.gca().add_patch(arc)
                        else:
                            # plt.plot(cx, cy, 'go', markersize=4)  # Visualize arc center as red dot
                            arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                                      theta1=end_angle, theta2=start_angle,
                                      color=color, lw=2, alpha=0.1)
                            plt.gca().add_patch(arc)

                    else:
                        if move1.segment_id > 0:
                            plt.arrow(x1, y1, x2 - x1, y2 - y1,
                                      head_width=0.2, length_includes_head=True,
                                      color=color, alpha=0.1)
                        else:
                            plt.arrow(x1, y1, x2 - x1, y2 - y1,
                                      head_width=0.2, length_includes_head=True,
                                      color=color, alpha=0.1)

        ax.set_aspect('equal')
        ax.set_title("Critical MoveState Heatmap-Grid")
        plt.grid(False)
        plt.show()

    def save_collision_profile(self, filepath="collision_profile.pkl"):
        with open(filepath, "wb") as f:
            pickle.dump({
                "Move_state": self.Move_state,
                "Stop_state": self.Stop_state,
                "collision_profile": self.collision_profile,
                "move_regions": self.move_regions,
                "stop_regions": self.stop_regions,
                "critical_nodes":self.od_critical_moves,
                "od_pairs":self.od_pairs,
                "always,critical":self.always_critical
            }, f)

        print(f"Collision profile saved to {filepath}")

    def load_collision_profile(self, filepath="collision_profile.pkl"):
        with open(filepath, "rb") as f:
            data = pickle.load(f)
        print("Data")
        self.collision_profile = data["collision_profile"]
        self.move_regions = data["move_regions"]
        self.stop_regions = data["stop_regions"]
        self.Stop_state = data["Stop_state"]
        self.Move_state = data["Move_state"]
        self.od_critical_moves = data["critical_nodes"]
        self.od_pairs = data["od_pairs"]
        self.always_critical = data["always,critical"]
        print(f"Collision profile loaded from {filepath}")

    def generate_map_info(self):
        return self.Stop_state, self.Move_state, self.collision_profile


    def fill_affect_state(self):
        for stop in self.Stop_state:
            self.Stop_state[stop].affect_state = self.collision_profile.get(stop, [])

        for move in self.Move_state:
            self.Move_state[move].affect_state = self.collision_profile.get(move, [])


    def check_specific_node_arc(self,layout_json,s,g,u,v):
        G = nx.DiGraph()

        for stop_id, stop_state in layout.Stop_state.items():
            for move_id in stop_state.next_state:
                if move_id in layout.Move_state:
                    move_state = layout.Move_state[move_id]
                    for next_stop_id in move_state.next_state:
                        G.add_edge(stop_id, next_stop_id)

        G_tmp = G.copy()
        G_tmp.remove_edge(u, v)
        if not nx.has_path(G_tmp, s, g):
            move_id = edge_to_move_id.get((u, v), f"{u}->{v}")
            od_critical_moves[(s, g)].append(move_id)
        else:
            path = nx.shortest_path(G_tmp, source=s, target=g)
            print("Printinnodearccheck",path)

    def get_always_critical_moves_per_origin(self):
        # Group by source terminal
        from collections import defaultdict

        origin_to_paths = defaultdict(list)
        for (src, dst), moves in self.od_critical_moves.items():
            origin_to_paths[src].append(set(moves))

        # For each origin, take intersection across all (src, *) critical moves
        always_critical = {}
        for src, move_sets in origin_to_paths.items():
            if move_sets:
                common_moves = set.intersection(*move_sets)
                always_critical[src] = common_moves
            else:
                always_critical[src] = set()
        return always_critical

    def _has_path_without_edge(self, G, s, g, skip_u, skip_v):
        """그래프 복사 없이 (skip_u→skip_v) 엣지를 제외하고 s→g 경로 존재 여부를 BFS로 확인."""
        if s == g:
            return True
        visited = {s}
        stack = [s]
        while stack:
            node = stack.pop()
            for nb in G.successors(node):
                if node == skip_u and nb == skip_v:
                    continue
                if nb == g:
                    return True
                if nb not in visited:
                    visited.add(nb)
                    stack.append(nb)
        return False

    #중요 이건 나중에 ACS에도 넣어야함
    def check_blockage(self,origin,destination,block): #input은 state
        G = nx.DiGraph()
        edge_to_move_id = {}

        self.port_node_ids = {port["nodeId"] for port in self.layout.get("ports", [])}
        segments = self.layout["segments"]
        for seg in segments:
            end_node = seg["endNodeId"]
            if end_node not in self.port_node_ids:
                continue
            start_node = seg["startNodeId"]
            move_state_id = f"M,{start_node},{end_node}"
            if end_node == origin:
                s = self.Move_state[move_state_id].end_id
            if end_node == destination:
                g = self.Move_state[move_state_id].end_id
            if end_node == block:
                block = self.Move_state[move_state_id].end_id



        for stop_id, stop_state in self.Stop_state.items():
            for move_id in stop_state.next_state:
                if move_id in self.Move_state:
                    move_state = self.Move_state[move_id]
                    for next_stop_id in move_state.next_state:
                        G.add_edge(stop_id, next_stop_id)
                        edge_to_move_id[(stop_id, move_state.end_id)] = move_id

        G_tmp = G.copy()
        for states in self.collision_profile[block]:
            if states[0] == 'S':
                continue
            u = self.Move_state[states].start_id
            v = self.Move_state[states].end_id
            G_tmp.remove_edge(u, v)
        if not nx.has_path(G_tmp, s, g):
            return True
        else:
            return False

    def compute_critical_move_frequencies(self, layout_json):
        G = nx.DiGraph()
        edge_to_move_id = {}
        segments = layout_json["segments"]
        for stop_id, stop_state in self.Stop_state.items():
            for move_id in stop_state.next_state:
                if move_id in self.Move_state:
                    move_state = self.Move_state[move_id]
                    for next_stop_id in move_state.next_state:
                        G.add_edge(stop_id, next_stop_id)
                        edge_to_move_id[(stop_id, move_state.end_id)] = move_id

        self.port_node_ids = {port["nodeId"] for port in layout_json.get("ports", [])}
        port_states = set()
        for seg in segments:
            end_node = seg["endNodeId"]
            if end_node not in self.port_node_ids:
                continue
            start_node = seg["startNodeId"]
            move_state_id = f"M,{start_node},{end_node}"
            port_states.add(self.Move_state[move_state_id].end_id)

        self.od_pairs_state = [(s, g) for s in port_states for g in port_states if s != g and nx.has_path(G, s, g)]
        self.od_pairs = [(s, g) for s in self.port_node_ids for g in self.port_node_ids]

        # 도달 가능성 사전 계산: 엣지가 해당 OD 경로 위에 있을 수 없으면 건너뜀
        sources = {s for s, g in self.od_pairs_state}
        targets = {g for s, g in self.od_pairs_state}
        forward_reach = {s: nx.descendants(G, s) | {s} for s in sources}
        backward_reach = {g: nx.ancestors(G, g) | {g} for g in targets}

        edges = list(G.edges())
        od_critical_moves = defaultdict(list)
        od_critical_moves_state = defaultdict(list)

        print(f"  critical move 탐색 중... OD쌍={len(self.od_pairs_state)}, 엣지={len(edges)}")
        for s, g in self.od_pairs_state:
            fr = forward_reach[s]
            br = backward_reach[g]
            for u, v in edges:
                # u가 s에서 도달 불가하거나 v가 g로 이어지지 않으면 이 엣지는 s→g 경로 위에 없음
                if u not in fr or v not in br:
                    continue
                # G 복사 없이 BFS로 연결성 확인
                if not self._has_path_without_edge(G, s, g, u, v):
                    move_id = edge_to_move_id.get((u, v), f"{u}->{v}")
                    sa = s.split(',')[1]
                    ga = g.split(',')[1]
                    od_critical_moves[(sa, ga)].append(move_id)
                    od_critical_moves_state[(s, g)].append(move_id)

        move_freq = Counter([m for ms in od_critical_moves.values() for m in ms])
        self.od_critical_moves = od_critical_moves
        self.move_freq = move_freq
        self.od_critical_moves_state = od_critical_moves_state

    # def compute_critical_move_frequencies(self,layout_json):
    #
    #     G = nx.DiGraph()
    #     edge_to_move_id = {}
    #
    #     for segment in layout_json.get("segments", []):
    #         start = segment.get("startNodeId")
    #         end = segment.get("endNodeId")
    #         if start is not None and end is not None:
    #             G.add_edge(start, end)
    #             move_id = f"M,{start},{end}"
    #             edge_to_move_id[(start, end)] = move_id
    #
    #     port_node_ids = {port["nodeId"] for port in layout_json.get("ports", [])}
    #     print(port_node_ids)
    #     self.od_pairs = [(s, g) for s in port_node_ids for g in port_node_ids if s != g and nx.has_path(G, s, g)]
    #
    #     od_critical_moves = defaultdict(list)
    #     for s, g in self.od_pairs:
    #         for u, v in G.edges:
    #             G_tmp = G.copy()
    #             G_tmp.remove_edge(u, v)
    #             if not nx.has_path(G_tmp, s, g):
    #                 move_id = edge_to_move_id.get((u, v), f"{u}->{v}")
    #                 od_critical_moves[(s, g)].append(move_id)
    #             else:
    #                 path = nx.shortest_path(G_tmp, source=s, target=g)
    #
    #     # Frequency count
    #     move_freq = Counter([m for ms in od_critical_moves.values() for m in ms])
    #     self.od_critical_moves = od_critical_moves
    #     self.move_freq = move_freq

    def visualize_all_critical_moves(self, from_id=None, to_id=None):
        plt.figure(figsize=(10,10))
        coords = {}
        current = from_id
        visited = set()
        critical_moves = []
        if from_id == None and to_id == None:
            from_id = self.od_pairs[0][0]
            to_id = self.od_pairs[0][1]


        fig, ax = plt.subplots(figsize=(10,10))


        all_colliding_states = set()
        #
        #     for move in critical_moves:
        #         if move in self.collision_profile:
        #             all_colliding_states.update(self.collision_profile[move])

        print("Length_od_pairs",len(self.od_pairs))
        critical_moves = [m for ms in self.od_critical_moves.values() for m in ms]
        print("Critical_moves",critical_moves)
        collision_exposure = Counter(
            state for m in critical_moves
            for state in self.collision_profile.get(m, [])
        )
        print(collision_exposure)
        norm = Normalize(vmin=0, vmax=max(collision_exposure.values()) if collision_exposure else 1)
        for move in self.Move_state.values():
            freq = collision_exposure.get(move.id, 0) if collision_exposure else 0
            # color = cmap(norm(freq))
            color = norm(freq)/2

            move_id = move.id
            if move_id[0] == 'S':
                poly = self.stop_regions[move_id]
            elif move_id[0] == 'M':
                poly = self.move_regions[move_id]
                x, y = poly.exterior.xy
                ax.fill(x, y, alpha=color, fc='red', ec='black')
            # elif move_id[0] == 'R':
            #     poly = self.move_regions[move_id]
            #     x, y = poly.exterior.xy
            #     ax.fill(x, y, alpha=color, fc='red', ec='black')

        # for (i,j) in self.od_pairs:
        #     from_id = i
        #     to_id = j
        #     critical_moves = self.od_critical_moves[(from_id,to_id)]
        #     # 시각화
        #     all_colliding_states = set()
        #
        #     for move in critical_moves:
        #         if move in self.collision_profile:
        #             all_colliding_states.update(self.collision_profile[move])
        #     print(all_colliding_states)
        #     for move_id in all_colliding_states:
        #         if move_id[0] == 'S':
        #             poly = self.stop_regions[move_id]
        #         elif move_id[0] == 'M':
        #             poly = self.move_regions[move_id]
        #             x, y = poly.exterior.xy
        #             ax.fill(x, y, alpha=0.01, fc='red', ec='black')
        #         elif move_id[0] == 'R':
        #             poly = self.move_regions[move_id]
        #             x, y = poly.exterior.xy
        #             ax.fill(x, y, alpha=0.0025, fc='red', ec='black')
                # x, y = poly.exterior.xy
                # ax.fill(x, y, alpha=0.2, fc='red', ec='black')
                # ax.text(np.mean(x), np.mean(y), move_id, fontsize=8)

        for node_id in self.Stop_state:
            x, y = self.node_coords[get_node_num_from_id(node_id)]
            coords[node_id] = (x, y)
            # plt.plot(x, y, 'ko')
            plt.text(x, y + 0.3, get_node_num_from_id(node_id), fontsize=8, ha='center')

        for move in self.Move_state.values():
            x1, y1 = coords[move.start_id]
            x2, y2 = coords[move.end_id]
            if move.center:
                cx, cy = move.center
                dx1 = x1 - cx
                dy1 = y1 - cy
                dx2 = x2 - cx
                dy2 = y2 - cy
                start_angle = np.degrees(np.arctan2(dy1, dx1)) % 360
                end_angle = np.degrees(np.arctan2(dy2, dx2)) % 360
                radius = (dx1 ** 2 + dy1 ** 2) ** 0.5

                if move.segment_id > 0:
                    # print(move.segment_id)
                    # print(start_angle, end_angle)
                    # plt.plot(cx, cy, 'ro', markersize=4)  # Visualize arc center as red dot/
                    arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                              theta1=start_angle, theta2=end_angle,
                              color='blue', lw=2, alpha=0.6)
                    plt.gca().add_patch(arc)
                else:
                    # plt.plot(cx, cy, 'go', markersize=4)  # Visualize arc center as red dot
                    arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                              theta1=end_angle, theta2=start_angle,
                              color='blue', lw=2, alpha=0.6)
                    plt.gca().add_patch(arc)

            else:
                if move.segment_id > 0:
                    plt.arrow(x1, y1, x2 - x1, y2 - y1,
                              head_width=0.2, length_includes_head=True,
                              color='blue', alpha=0.6)
                else:
                    plt.arrow(x1, y1, x2 - x1, y2 - y1,
                              head_width=0.2, length_includes_head=True,
                              color='blue', alpha=0.6)


        plt.title(f"Critical area heatmap")
        plt.show()

    def visualize_critical_moves_for_fromto(self, from_id=None,to_id =None):
        plt.figure(figsize=(10,2))
        coords = {}
        current = from_id
        visited = set()
        critical_moves = []

        if from_id == None:
            from_id = '2'
            to_id = '14'


        fig, ax = plt.subplots(figsize=(10,2))


        critical_moves = self.od_critical_moves[(from_id,to_id)]
        print("Printcritical",critical_moves)
        # 시각화
        all_colliding_states = set()

        for move in critical_moves:
            if move in self.collision_profile:
                all_colliding_states.update(self.collision_profile[move])
        print(all_colliding_states)
        for move_id in all_colliding_states:
            if move_id[0] == 'S':
                if move_id.split(',')[1] in self.port_node_ids:
                    poly = self.stop_regions[move_id]
                    x, y = poly.exterior.xy
                    if move_id.split(',')[1] == from_id:
                        color = 'red'
                    elif move_id.split(',')[1] == to_id:
                        color = 'blue'
                    else:
                        color = 'green'
                    ax.fill(x, y, alpha=0.5, fc=color, ec='black')
            # elif move_id[0] == 'M':
            #     poly = self.move_regions[move_id]
            #     x, y = poly.exterior.xy
            #     ax.fill(x, y, alpha=0.2, fc='green', ec='black')
            # elif move_id[0] == 'R':
            #     poly = self.move_regions[move_id]
            #     x, y = poly.exterior.xy
            #     ax.fill(x, y, alpha=0.2, fc='green', ec='black')

        for node_id in self.Stop_state:
            x, y = self.node_coords[get_node_num_from_id(node_id)]
            coords[node_id] = (x, y)
            # plt.plot(x, y, 'ko')
            plt.text(x, y + 0.3, get_node_num_from_id(node_id), fontsize=8, ha='center')

        for move in self.Move_state.values():
            x1, y1 = coords[move.start_id]
            x2, y2 = coords[move.end_id]
            if move.id not in critical_moves:
                if move.center:
                    cx, cy = move.center
                    dx1 = x1 - cx
                    dy1 = y1 - cy
                    dx2 = x2 - cx
                    dy2 = y2 - cy
                    start_angle = np.degrees(np.arctan2(dy1, dx1)) % 360
                    end_angle = np.degrees(np.arctan2(dy2, dx2)) % 360
                    radius = (dx1 ** 2 + dy1 ** 2) ** 0.5

                    if move.segment_id > 0:
                        # print(move.segment_id)
                        # print(start_angle, end_angle)
                        # plt.plot(cx, cy, 'ro', markersize=4)  # Visualize arc center as red dot/
                        arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                                  theta1=start_angle, theta2=end_angle,
                                  color='blue', lw=2, alpha=0.4)
                        plt.gca().add_patch(arc)
                    else:
                        # plt.plot(cx, cy, 'go', markersize=4)  # Visualize arc center as red dot
                        arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                                  theta1=end_angle, theta2=start_angle,
                                  color='blue', lw=2, alpha=0.4)
                        plt.gca().add_patch(arc)

                else:
                    if move.segment_id > 0:
                        plt.arrow(x1, y1, x2 - x1, y2 - y1,
                                  head_width=0.2, length_includes_head=True,
                                  color='blue', alpha=0.4)
                    else:
                        plt.arrow(x1, y1, x2 - x1, y2 - y1,
                                  head_width=0.2, length_includes_head=True,
                                  color='blue', alpha=0.4)


        plt.title(f"Blocking terminal states(green) while moving start(red) to goal(blue)")
        plt.show()


    def visualize_always_critical_moves(self, from_id=None,to_id =None):
        plt.figure(figsize=(10,2))
        coords = {}
        current = from_id
        visited = set()
        critical_moves = []

        if from_id == None:
            from_id = self.od_pairs[8][0]


        fig, ax = plt.subplots(figsize=(10,2))


        critical_moves = self.always_critical[from_id]
        print("Printcritical",critical_moves)
        # 시각화
        all_colliding_states = set()

        for move in critical_moves:
            if move in self.collision_profile:
                all_colliding_states.update(self.collision_profile[move])
        print(all_colliding_states)
        for move_id in all_colliding_states:
            if move_id[0] == 'S':
                if move_id.split(',')[1] in self.port_node_ids:
                    poly = self.stop_regions[move_id]
                    x, y = poly.exterior.xy
                    if move_id.split(',')[1] == from_id:
                        color = 'red'
                    else:
                        color = 'green'
                    ax.fill(x, y, alpha=0.5, fc=color, ec='black')
            # elif move_id[0] == 'M':
            #     poly = self.move_regions[move_id]
            #     x, y = poly.exterior.xy
            #     ax.fill(x, y, alpha=0.2, fc='green', ec='black')
            # elif move_id[0] == 'R':
            #     poly = self.move_regions[move_id]
            #     x, y = poly.exterior.xy
            #     ax.fill(x, y, alpha=0.2, fc='green', ec='black')

        for node_id in self.Stop_state:
            x, y = self.node_coords[get_node_num_from_id(node_id)]
            coords[node_id] = (x, y)
            # plt.plot(x, y, 'ko')
            plt.text(x, y + 0.3, get_node_num_from_id(node_id), fontsize=8, ha='center')

        for move in self.Move_state.values():
            x1, y1 = coords[move.start_id]
            x2, y2 = coords[move.end_id]
            if move.id not in critical_moves:
                if move.center:
                    cx, cy = move.center
                    dx1 = x1 - cx
                    dy1 = y1 - cy
                    dx2 = x2 - cx
                    dy2 = y2 - cy
                    start_angle = np.degrees(np.arctan2(dy1, dx1)) % 360
                    end_angle = np.degrees(np.arctan2(dy2, dx2)) % 360
                    radius = (dx1 ** 2 + dy1 ** 2) ** 0.5

                    if move.segment_id > 0:
                        # print(move.segment_id)
                        # print(start_angle, end_angle)
                        # plt.plot(cx, cy, 'ro', markersize=4)  # Visualize arc center as red dot/
                        arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                                  theta1=start_angle, theta2=end_angle,
                                  color='blue', lw=2, alpha=0.4)
                        plt.gca().add_patch(arc)
                    else:
                        # plt.plot(cx, cy, 'go', markersize=4)  # Visualize arc center as red dot
                        arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                                  theta1=end_angle, theta2=start_angle,
                                  color='blue', lw=2, alpha=0.4)
                        plt.gca().add_patch(arc)

                else:
                    if move.segment_id > 0:
                        plt.arrow(x1, y1, x2 - x1, y2 - y1,
                                  head_width=0.2, length_includes_head=True,
                                  color='blue', alpha=0.4)
                    else:
                        plt.arrow(x1, y1, x2 - x1, y2 - y1,
                                  head_width=0.2, length_includes_head=True,
                                  color='blue', alpha=0.4)


        plt.title(f"Mutually locked terminals of terminal state {from_id}")
        plt.show()

    def visualize_toy_case(self, from_id=None,to_id =None):

        plt.figure(figsize=(10, 2))
        coords = {}
        current = from_id
        visited = set()
        critical_moves = []
        if from_id == None and to_id == None:
            from_id = self.od_pairs[8][0]
            to_id = self.od_pairs[8][1]
            from_id = '265'
            to_id = '294'

        fig, ax = plt.subplots(figsize=(10, 2))

        for node_id in self.Stop_state:
            x, y = self.node_coords[get_node_num_from_id(node_id)]
            coords[node_id] = (x, y)
        fromto = [('265','294'),('282','282'),('277','249')]
        colors = ['red', 'green', 'blue']
        for i in range(len(fromto)):
            from_id = fromto[i][0]
            to_id = fromto[i][1]
            for j in range(len(fromto)):
                if i != j:
                    if self.check_blockage(fromto[i][0],fromto[i][1],fromto[j][0]):
                        x1, y1 = self.node_coords[from_id]
                        x2, y2 = self.node_coords[fromto[j][0]]
                        plt.annotate('', xy=(x2, y2), xytext=(x1, y1),
                                     arrowprops=dict(color=colors[i], arrowstyle='-|>', lw=2, alpha=0.9),
                                     )
            if from_id in self.node_coords and to_id in self.node_coords:
                print("Arrow")
                x1, y1 = self.node_coords[from_id]
                x2, y2 = self.node_coords[to_id]
                plt.annotate('', xy=(x2, y2), xytext=(x1, y1),
                             arrowprops=dict(facecolor='black', arrowstyle='->', lw=0.5, alpha=0.5),
                             )
            critical_moves = self.always_critical[from_id]
            print("Printcritical",critical_moves)
            # 시각화
            all_colliding_states = set()

            for move in critical_moves:
                if move in self.collision_profile:
                    all_colliding_states.update(self.collision_profile[move])
            print(all_colliding_states)
            for move_id in all_colliding_states:
                if move_id[0] == 'S':
                    if move_id.split(',')[1] == from_id and move_id.split(',')[1] in self.port_node_ids:
                        poly = self.stop_regions[move_id]
                        x, y = poly.exterior.xy
                        ax.fill(x, y, alpha=0.9, fc=colors[i], ec='black')

            critical_moves = self.always_critical[to_id]
            print("Printcritical", critical_moves)
            # 시각화
            all_colliding_states = set()

            for move in critical_moves:
                if move in self.collision_profile:
                    all_colliding_states.update(self.collision_profile[move])
            print(all_colliding_states)
            for move_id in all_colliding_states:
                if move_id[0] == 'S':
                    if move_id.split(',')[1] == to_id and move_id.split(',')[1] in self.port_node_ids:
                        poly = self.stop_regions[move_id]
                        x, y = poly.exterior.xy
                        ax.fill(x, y, alpha=0.3, fc=colors[i], ec='black')
            critical_moves = self.od_critical_moves[(from_id, to_id)]
            print("Printcritical", critical_moves)
        for move in self.Move_state.values():
            x1, y1 = coords[move.start_id]
            x2, y2 = coords[move.end_id]
            if move.center:
                cx, cy = move.center
                dx1 = x1 - cx
                dy1 = y1 - cy
                dx2 = x2 - cx
                dy2 = y2 - cy
                start_angle = np.degrees(np.arctan2(dy1, dx1)) % 360
                end_angle = np.degrees(np.arctan2(dy2, dx2)) % 360
                radius = (dx1 ** 2 + dy1 ** 2) ** 0.5

                if move.segment_id > 0:
                    # print(move.segment_id)
                    # print(start_angle, end_angle)
                    # plt.plot(cx, cy, 'ro', markersize=4)  # Visualize arc center as red dot/
                    arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                              theta1=start_angle, theta2=end_angle,
                              color='blue', lw=2, alpha=0.5)
                    plt.gca().add_patch(arc)
                else:
                    # plt.plot(cx, cy, 'go', markersize=4)  # Visualize arc center as red dot
                    arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                              theta1=end_angle, theta2=start_angle,
                              color='blue', lw=2, alpha=0.5)
                    plt.gca().add_patch(arc)

            else:
                if move.segment_id > 0:
                    plt.arrow(x1, y1, x2 - x1, y2 - y1,
                              head_width=0.2, length_includes_head=True,
                              color='blue', alpha=0.5)
                else:
                    plt.arrow(x1, y1, x2 - x1, y2 - y1,
                              head_width=0.2, length_includes_head=True,
                              color='blue', alpha=0.5)

                # else:
                #     if move.center:
                #         cx, cy = move.center
                #         dx1 = x1 - cx
                #         dy1 = y1 - cy
                #         dx2 = x2 - cx
                #         dy2 = y2 - cy
                #         start_angle = np.degrees(np.arctan2(dy1, dx1)) % 360
                #         end_angle = np.degrees(np.arctan2(dy2, dx2)) % 360
                #         radius = (dx1 ** 2 + dy1 ** 2) ** 0.5
                #
                #         if move.segment_id > 0:
                #             # print(move.segment_id)
                #             # print(start_angle, end_angle)
                #             # plt.plot(cx, cy, 'ro', markersize=4)  # Visualize arc center as red dot/
                #             arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                #                       theta1=start_angle, theta2=end_angle,
                #                       color=colors[i], lw=2, alpha=0.8, linewidth=3)
                #             plt.gca().add_patch(arc)
                #         else:
                #             # plt.plot(cx, cy, 'go', markersize=4)  # Visualize arc center as red dot
                #             arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                #                       theta1=end_angle, theta2=start_angle,
                #                       color=colors[i], lw=2, alpha=0.8, linewidth=3)
                #             plt.gca().add_patch(arc)
                #
                #     else:
                #
                #         if move.segment_id > 0:
                #             plt.arrow(x1, y1, x2 - x1, y2 - y1,
                #                       head_width=0.2, length_includes_head=True,
                #                       color=colors[i], alpha=0.8, linewidth=3)
                #         else:
                #             plt.arrow(x1, y1, x2 - x1, y2 - y1,
                #                       head_width=0.2, length_includes_head=True,
                #                       color=colors[i], alpha=0.8, linewidth=3)

        # 시각화
        all_colliding_states = set()

        # for move in critical_moves:
        #     if move in self.collision_profile:
        #         all_colliding_states.update(self.collision_profile[move])
        # print(all_colliding_states)
        # for move_id in all_colliding_states:
        #     if move_id[0] == 'S':
        #         poly = self.stop_regions[move_id]
        #     elif move_id[0] == 'M':
        #         poly = self.move_regions[move_id]
        #         x, y = poly.exterior.xy
        #         ax.fill(x, y, alpha=0.1, fc='red', ec='black')
        #     elif move_id[0] == 'R':
        #         poly = self.move_regions[move_id]
        #         x, y = poly.exterior.xy
        #         ax.fill(x, y, alpha=0.025, fc='red', ec='black')

        for node_id in self.Stop_state:
            x, y = self.node_coords[get_node_num_from_id(node_id)]
            coords[node_id] = (x, y)
            # plt.plot(x, y, 'ko')
            # plt.text(x, y + 0.3, get_node_num_from_id(node_id), fontsize=8, ha='center')


        # plt.grid(True)
        plt.title(f"Blockage dependency between AGVs")
        plt.show()

    def visualize_critical_moves(self, from_id=None, to_id=None):
        plt.figure(figsize=(10,2))
        coords = {}
        current = from_id
        visited = set()
        critical_moves = []
        if from_id == None and to_id == None:
            from_id = self.od_pairs[8][0]
            to_id = self.od_pairs[8][1]
            from_id = '2'
            to_id = '14'


        fig, ax = plt.subplots(figsize=(10,2))


        critical_moves = self.od_critical_moves[(from_id,to_id)]
        print("Printcritical",critical_moves)
        # 시각화
        all_colliding_states = set()

        # for move in critical_moves:
        #     if move in self.collision_profile:
        #         all_colliding_states.update(self.collision_profile[move])
        # print(all_colliding_states)
        # for move_id in all_colliding_states:
        #     if move_id[0] == 'S':
        #         poly = self.stop_regions[move_id]
        #     elif move_id[0] == 'M':
        #         poly = self.move_regions[move_id]
        #         x, y = poly.exterior.xy
        #         ax.fill(x, y, alpha=0.1, fc='red', ec='black')
        #     elif move_id[0] == 'R':
        #         poly = self.move_regions[move_id]
        #         x, y = poly.exterior.xy
        #         ax.fill(x, y, alpha=0.025, fc='red', ec='black')

        for node_id in self.Stop_state:
            x, y = self.node_coords[get_node_num_from_id(node_id)]
            coords[node_id] = (x, y)
            # plt.plot(x, y, 'ko')
            plt.text(x, y + 0.3, get_node_num_from_id(node_id), fontsize=8, ha='center')

        for move in self.Move_state.values():

            if move.id == 'M,308,311':

                print("Moveid.111",from_id,to_id,move.id in critical_moves)
            x1, y1 = coords[move.start_id]
            x2, y2 = coords[move.end_id]
            if move.id not in critical_moves:
                if move.center:
                    cx, cy = move.center
                    dx1 = x1 - cx
                    dy1 = y1 - cy
                    dx2 = x2 - cx
                    dy2 = y2 - cy
                    start_angle = np.degrees(np.arctan2(dy1, dx1)) % 360
                    end_angle = np.degrees(np.arctan2(dy2, dx2)) % 360
                    radius = (dx1 ** 2 + dy1 ** 2) ** 0.5

                    if move.segment_id > 0:
                        # print(move.segment_id)
                        # print(start_angle, end_angle)
                        # plt.plot(cx, cy, 'ro', markersize=4)  # Visualize arc center as red dot/
                        arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                                  theta1=start_angle, theta2=end_angle,
                                  color='blue', lw=2, alpha=0.4)
                        plt.gca().add_patch(arc)
                    else:
                        # plt.plot(cx, cy, 'go', markersize=4)  # Visualize arc center as red dot
                        arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                                  theta1=end_angle, theta2=start_angle,
                                  color='blue', lw=2, alpha=0.4)
                        plt.gca().add_patch(arc)

                else:
                    if move.segment_id > 0:
                        plt.arrow(x1, y1, x2 - x1, y2 - y1,
                                  head_width=0.2, length_includes_head=True,
                                  color='blue', alpha=0.4)
                    else:
                        plt.arrow(x1, y1, x2 - x1, y2 - y1,
                                  head_width=0.2, length_includes_head=True,
                                  color='blue', alpha=0.4)

            else:
                if move.center:
                    cx, cy = move.center
                    dx1 = x1 - cx
                    dy1 = y1 - cy
                    dx2 = x2 - cx
                    dy2 = y2 - cy
                    start_angle = np.degrees(np.arctan2(dy1, dx1)) % 360
                    end_angle = np.degrees(np.arctan2(dy2, dx2)) % 360
                    radius = (dx1 ** 2 + dy1 ** 2) ** 0.5

                    if move.segment_id > 0:
                        # print(move.segment_id)
                        # print(start_angle, end_angle)
                        # plt.plot(cx, cy, 'ro', markersize=4)  # Visualize arc center as red dot/
                        arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                                  theta1=start_angle, theta2=end_angle,
                                  color='red', lw=2, alpha=0.8,linewidth = 3)
                        plt.gca().add_patch(arc)
                    else:
                        # plt.plot(cx, cy, 'go', markersize=4)  # Visualize arc center as red dot
                        arc = Arc((cx, cy), 2 * radius, 2 * radius, angle=0,
                                  theta1=end_angle, theta2=start_angle,
                                  color='red', lw=2, alpha=0.8,linewidth = 3)
                        plt.gca().add_patch(arc)

                else:

                    if move.segment_id > 0:
                        plt.arrow(x1, y1, x2 - x1, y2 - y1,
                                  head_width=0.2, length_includes_head=True,
                                  color='red', alpha=0.8,linewidth = 3)
                    else:
                        plt.arrow(x1, y1, x2 - x1, y2 - y1,
                                  head_width=0.2, length_includes_head=True,
                                  color='red', alpha=0.8,linewidth = 3)
        if from_id in self.node_coords and to_id in self.node_coords:
            print("Arrow")
            x1, y1 = self.node_coords[from_id]
            x2, y2 = self.node_coords[to_id]
            plt.annotate('', xy=(x2, y2), xytext=(x1, y1),
                         arrowprops=dict(facecolor='black', arrowstyle='->', lw=2, alpha=0.9),
                         )
        # plt.grid(True)
        plt.title(f"Critical Moves from {from_id} to {to_id}")
        plt.show()

if __name__ == '__main__':
    # ※ 중요: LayoutManager 초기화 시 vehicle rotation 가능 여부 확인할 것

    # --- 입력 / 출력 경로 설정 ---
    # __main__ 블록 설정 확인:
    layout_filename = "Maps/c10_map.json"
    output_pkl      = "77.pkl"

    # --- 핵심 파이프라인 ---
    layout = LayoutManager(layout_filename)
    layout.save_collision_profile(output_pkl)
    print(f"Saved: {output_pkl}")

    # --- 선택적 시각화 (필요 시 주석 해제) ---
    # layout.visualize_layout()t
    # layout.visualize_critical_moves()
    # layout.visualize_all_critical_moves()
    # layout.visualize_always_critical_moves()
    # layout.visualize_critical_moves_for_fromto()
    # layout.compute_edge_betweenness_ports(use_cost_weight=True, directed=True, normalized=True)
    # layout.visualize_port_bc_vs_cutfreq()
    # move_freq = layout.compute_critical_move_frequencies()
    # layout.visualize_layout_with_heatmap(move_freq)
#     for i in range(10):
#         layout.visualize_layout()