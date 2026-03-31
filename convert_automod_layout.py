"""
Convert AutoMod guidepath layout (LargeOHT.txt) to xmsmap-v4.0 JSON format.
Parses GPATH (straight lines and arcs) and CPOINT definitions,
computes coordinates, detects junctions where path endpoints meet other paths,
checks connectivity, and outputs JSON.
"""

import math
import json
from collections import defaultdict

INPUT_FILE = r"C:\Users\user\Desktop\LargeOHT.txt"
OUTPUT_FILE = r"C:\Users\user\Desktop\KAIST\LargeOHT.json"

COORD_TOLERANCE = 10.0  # mm tolerance for matching endpoints (Automod S-curve gaps ~8mm)


def parse_file(filepath):
    """Parse the AutoMod layout file, returning lists of paths and cpoints."""
    paths = []
    cpoints = []

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if line.startswith("GPATH "):
                path = parse_gpath(line)
                if path:
                    paths.append(path)
            elif line.startswith("CPOINT "):
                cp = parse_cpoint(line)
                if cp:
                    cpoints.append(cp)
    return paths, cpoints


def parse_gpath(line):
    """Parse a GPATH line into a dict with name, type (line/arc), and geometry."""
    tokens = line.split()
    data = {}
    i = 0
    while i < len(tokens):
        tok = tokens[i]
        if tok == "name" and i + 1 < len(tokens):
            data["name"] = tokens[i + 1]
            i += 2
        elif tok == "begx" and i + 1 < len(tokens):
            data["begx"] = float(tokens[i + 1])
            i += 2
        elif tok == "begy" and i + 1 < len(tokens):
            data["begy"] = float(tokens[i + 1])
            i += 2
        elif tok == "endx" and i + 1 < len(tokens):
            data["endx"] = float(tokens[i + 1])
            i += 2
        elif tok == "endy" and i + 1 < len(tokens):
            data["endy"] = float(tokens[i + 1])
            i += 2
        elif tok == "cenx" and i + 1 < len(tokens):
            data["cenx"] = float(tokens[i + 1])
            i += 2
        elif tok == "ceny" and i + 1 < len(tokens):
            data["ceny"] = float(tokens[i + 1])
            i += 2
        elif tok == "angle" and i + 1 < len(tokens):
            data["angle"] = float(tokens[i + 1])
            i += 2
        else:
            i += 1

    if "name" not in data or "begx" not in data:
        return None

    if "cenx" in data:
        # Arc path
        cx, cy = data["cenx"], data["ceny"]
        bx, by = data["begx"], data["begy"]
        angle_tenths = data["angle"]

        radius = math.sqrt((bx - cx) ** 2 + (by - cy) ** 2)
        start_angle_rad = math.atan2(by - cy, bx - cx)
        sweep_rad = angle_tenths * math.pi / 1800.0
        end_angle_rad = start_angle_rad + sweep_rad
        ex = cx + radius * math.cos(end_angle_rad)
        ey = cy + radius * math.sin(end_angle_rad)

        arc_length = abs(sweep_rad) * radius

        return {
            "name": data["name"],
            "type": "arc",
            "start": (bx, by),
            "end": (ex, ey),
            "center": (cx, cy),
            "radius": radius,
            "start_angle": start_angle_rad,
            "sweep": sweep_rad,
            "length": arc_length,
        }
    elif "endx" in data:
        # Straight line
        bx, by = data["begx"], data["begy"]
        ex, ey = data["endx"], data["endy"]
        length = math.sqrt((ex - bx) ** 2 + (ey - by) ** 2)
        return {
            "name": data["name"],
            "type": "line",
            "start": (bx, by),
            "end": (ex, ey),
            "length": length,
        }
    return None


def parse_cpoint(line):
    """Parse a CPOINT line into a dict with name, type, path name, distance."""
    tokens = line.split()
    data = {}
    i = 0
    while i < len(tokens):
        tok = tokens[i]
        if tok == "name" and i + 1 < len(tokens):
            data["name"] = tokens[i + 1]
            i += 2
        elif tok == "type" and i + 1 < len(tokens):
            data["cptype"] = tokens[i + 1]
            i += 2
        elif tok == "at" and i + 2 < len(tokens):
            data["path"] = tokens[i + 1]
            data["distance"] = float(tokens[i + 2])
            i += 3
        else:
            i += 1

    if "name" in data and "path" in data and "distance" in data:
        return data
    return None


def point_along_path(path, distance):
    """Compute (x, y) at a given distance along a path from its start."""
    if path["type"] == "line":
        sx, sy = path["start"]
        ex, ey = path["end"]
        total = path["length"]
        if total == 0:
            return (sx, sy)
        t = distance / total
        t = max(0.0, min(1.0, t))
        return (sx + t * (ex - sx), sy + t * (ey - sy))
    else:
        # Arc: distance is arc length from start
        radius = path["radius"]
        if radius == 0:
            return path["start"]
        sign = 1.0 if path["sweep"] >= 0 else -1.0
        angular_disp = sign * distance / radius
        angle = path["start_angle"] + angular_disp
        cx, cy = path["center"]
        return (cx + radius * math.cos(angle), cy + radius * math.sin(angle))


def point_distance_to_line_segment(px, py, x1, y1, x2, y2):
    """
    Compute the minimum distance from point (px,py) to the line segment (x1,y1)-(x2,y2),
    and the parameter t (0..1) of the closest point on the segment.
    Returns (distance, t).
    """
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-12:
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2), 0.0
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq
    t = max(0.0, min(1.0, t))
    cx = x1 + t * dx
    cy = y1 + t * dy
    dist = math.sqrt((px - cx) ** 2 + (py - cy) ** 2)
    return dist, t


def point_distance_to_arc(px, py, path):
    """
    Compute the minimum distance from point (px,py) to an arc path,
    and the distance-along-arc of the closest point.
    Returns (distance, dist_along) or (inf, 0) if not close.
    """
    cx, cy = path["center"]
    radius = path["radius"]
    if radius < 1e-12:
        dist = math.sqrt((px - cx) ** 2 + (py - cy) ** 2)
        return dist, 0.0

    # Angle of point relative to center
    angle_p = math.atan2(py - cy, px - cx)
    start_angle = path["start_angle"]
    sweep = path["sweep"]

    # Normalize angle_p relative to start_angle
    # Check if angle_p is within the arc sweep
    da = angle_p - start_angle
    # Normalize da to be in range matching sweep direction
    if sweep >= 0:
        while da < -1e-9:
            da += 2 * math.pi
        while da > 2 * math.pi + 1e-9:
            da -= 2 * math.pi
        if da <= sweep + 1e-9:
            # Point angle is within arc
            closest_x = cx + radius * math.cos(angle_p)
            closest_y = cy + radius * math.sin(angle_p)
            dist = math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)
            dist_along = da * radius
            return dist, dist_along
    else:
        while da > 1e-9:
            da -= 2 * math.pi
        while da < -2 * math.pi - 1e-9:
            da += 2 * math.pi
        if da >= sweep - 1e-9:
            # Point angle is within arc (negative sweep)
            closest_x = cx + radius * math.cos(angle_p)
            closest_y = cy + radius * math.sin(angle_p)
            dist = math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)
            dist_along = abs(da) * radius
            return dist, dist_along

    # Point is outside arc range, check endpoints
    return float("inf"), 0.0


def coords_match(p1, p2, tol=COORD_TOLERANCE):
    """Check if two (x,y) points are within tolerance."""
    return abs(p1[0] - p2[0]) <= tol and abs(p1[1] - p2[1]) <= tol


def round_coord(val):
    """Round coordinate to integer for JSON output."""
    return round(val)


def find_junctions(paths, tol=COORD_TOLERANCE):
    """
    Find points where a path endpoint lies on the body of another path
    (not at its endpoints). Returns a dict: path_name -> list of (distance, x, y).
    """
    # Collect all path endpoints
    all_endpoints = []
    for p in paths:
        all_endpoints.append((p["start"][0], p["start"][1], p["name"], "start"))
        all_endpoints.append((p["end"][0], p["end"][1], p["name"], "end"))

    # For each endpoint, check if it lies on another path's body
    # Only check LINE paths — arc paths should not be split mid-curve
    junctions = defaultdict(list)  # path_name -> [(distance_along, x, y)]

    for ep_x, ep_y, ep_path_name, ep_type in all_endpoints:
        for path in paths:
            if path["name"] == ep_path_name:
                continue
            # Only split straight lines, never arcs
            if path["type"] != "line":
                continue
            # Skip if this endpoint matches the other path's start or end
            if coords_match((ep_x, ep_y), path["start"], tol):
                continue
            if coords_match((ep_x, ep_y), path["end"], tol):
                continue

            dist, t = point_distance_to_line_segment(
                ep_x, ep_y,
                path["start"][0], path["start"][1],
                path["end"][0], path["end"][1]
            )
            if dist <= tol and 0.001 < t < 0.999:
                dist_along = t * path["length"]
                junctions[path["name"]].append((dist_along, ep_x, ep_y))

    # Deduplicate junction points that are very close together
    for pname in junctions:
        pts = sorted(junctions[pname], key=lambda x: x[0])
        deduped = []
        for pt in pts:
            if not deduped or abs(pt[0] - deduped[-1][0]) > tol:
                deduped.append(pt)
        junctions[pname] = deduped

    return junctions


def build_node_map(paths, cpoints, path_lookup, junctions):
    """
    Build nodes from path endpoints, cpoints, and junction points.
    Returns: list of nodes.
    """
    raw_points = []

    # Path endpoints
    for p in paths:
        raw_points.append((p["start"][0], p["start"][1], ("path_start", p["name"])))
        raw_points.append((p["end"][0], p["end"][1], ("path_end", p["name"])))

    # CPOINT positions — only on straight line paths, not on arcs
    for cp in cpoints:
        pname = cp["path"]
        if pname not in path_lookup:
            continue
        path = path_lookup[pname]
        pos = point_along_path(path, cp["distance"])
        cp["position"] = pos
        if path["type"] == "line":
            raw_points.append((pos[0], pos[1], ("cpoint", cp["name"])))

    # Junction points
    for pname, jpts in junctions.items():
        path = path_lookup[pname]
        for dist_along, jx, jy in jpts:
            # Use the precise position on the path
            pos = point_along_path(path, dist_along)
            raw_points.append((pos[0], pos[1], ("junction", f"{pname}@{dist_along:.1f}")))

    # Merge points by coordinate proximity
    merged_nodes = []

    for px, py, info in raw_points:
        found = False
        for node in merged_nodes:
            if coords_match((px, py), (node[0], node[1])):
                node[2].add(info)
                found = True
                break
        if not found:
            merged_nodes.append([px, py, {info}])

    # Create node list with IDs
    nodes = []
    for idx, (nx, ny, infos) in enumerate(merged_nodes):
        node_id = f"N{idx:06d}"
        node = {
            "id": node_id,
            "x": round_coord(nx),
            "y": round_coord(ny),
            "z": 0,
            "area": "LargeOHT",
            "_raw_x": nx,
            "_raw_y": ny,
            "_infos": infos,
        }
        nodes.append(node)

    return nodes


def find_node_id(nodes, x, y, tol=COORD_TOLERANCE):
    """Find the node ID for a given coordinate."""
    best_dist = float("inf")
    best_id = None
    for n in nodes:
        dx = abs(x - n["_raw_x"])
        dy = abs(y - n["_raw_y"])
        if dx <= tol and dy <= tol:
            d = dx + dy
            if d < best_dist:
                best_dist = d
                best_id = n["id"]
    return best_id


def arc_to_bezier_control(cx, cy, radius, angle_start, angle_end):
    """
    Compute quadratic Bezier control point for a circular arc.
    For a circular arc from angle_start to angle_end with given center/radius,
    the quadratic Bezier control point is at the intersection of the tangent
    lines at the start and end points.

    For arc sweep theta: control = center + radius / cos(theta/2) * mid_direction
    """
    theta = angle_end - angle_start
    half = theta / 2.0
    cos_half = math.cos(half)
    if abs(cos_half) < 1e-9:
        # 180-degree arc, can't represent with single quadratic Bezier
        # Use midpoint on arc as approximation
        mid_angle = (angle_start + angle_end) / 2
        return (cx + radius * math.cos(mid_angle),
                cy + radius * math.sin(mid_angle))
    # Control point at intersection of tangent lines
    mid_angle = (angle_start + angle_end) / 2
    ctrl_r = radius / cos_half
    return (cx + ctrl_r * math.cos(mid_angle),
            cy + ctrl_r * math.sin(mid_angle))


def make_arc_parts(path, dist_start, dist_end):
    """
    Generate 'parts' list for an arc segment between two distances along the path.
    Splits large arcs into sub-arcs of <= 90 degrees for Bezier accuracy.
    Returns list of parts (each Straight or Arc).
    """
    radius = path["radius"]
    cx, cy = path["center"]
    sweep = path["sweep"]
    total_length = path["length"]
    sign = 1.0 if sweep >= 0 else -1.0

    # Angular positions
    ang_start = path["start_angle"] + sign * dist_start / radius
    ang_end = path["start_angle"] + sign * dist_end / radius
    total_sweep = ang_end - ang_start

    # Split into sub-arcs of <= 80 degrees for good Bezier approximation
    max_sweep = math.radians(80)
    n_sub = max(1, int(math.ceil(abs(total_sweep) / max_sweep)))

    parts = []
    for k in range(n_sub):
        a0 = ang_start + total_sweep * k / n_sub
        a1 = ang_start + total_sweep * (k + 1) / n_sub

        ctrl = arc_to_bezier_control(cx, cy, radius, a0, a1)
        end_pt = (cx + radius * math.cos(a1), cy + radius * math.sin(a1))

        parts.append({
            "kind": "Arc",
            "points": [
                {"x": round_coord(ctrl[0]), "y": round_coord(ctrl[1])},
                {"x": round_coord(end_pt[0]), "y": round_coord(end_pt[1])},
            ]
        })

    return parts


def build_segments(paths, cpoints, path_lookup, nodes, junctions):
    """
    Build segments from paths, splitting at CPOINT positions and junction points.
    Arc paths get 'parts' with Bezier curves for smooth rendering.
    """
    # Group cpoints by path
    cpoints_by_path = defaultdict(list)
    for cp in cpoints:
        if "position" in cp:
            cpoints_by_path[cp["path"]].append(cp)

    segments = []
    seg_id = 1

    for path in paths:
        pname = path["name"]

        if path["type"] == "arc":
            # Arc paths: single segment, no splitting — nodes only at endpoints
            start_node = find_node_id(nodes, path["start"][0], path["start"][1])
            end_node = find_node_id(nodes, path["end"][0], path["end"][1])
            if start_node and end_node and start_node != end_node:
                seg = {
                    "id": seg_id,
                    "startNodeId": start_node,
                    "endNodeId": end_node,
                    "speed": 1000,
                    "parts": make_arc_parts(path, 0.0, path["length"]),
                }
                segments.append(seg)
                seg_id += 1
            continue

        # Straight line paths: split at CPOINTs and junction points
        split_points = []

        # CPOINTs
        for cp in cpoints_by_path.get(pname, []):
            split_points.append((cp["distance"], cp["position"]))

        # Junction points
        for dist_along, jx, jy in junctions.get(pname, []):
            pos = point_along_path(path, dist_along)
            split_points.append((dist_along, pos))

        # Sort by distance along path
        split_points.sort(key=lambda x: x[0])

        # Deduplicate very close split points
        deduped = []
        for sp in split_points:
            if not deduped or abs(sp[0] - deduped[-1][0]) > 0.5:
                deduped.append(sp)
        split_points = deduped

        # Build waypoints: start, split points, end
        waypoints = [(0.0, path["start"])]
        for dist, pos in split_points:
            if dist > 0.5 and dist < path["length"] - 0.5:
                waypoints.append((dist, pos))
        waypoints.append((path["length"], path["end"]))

        # Create segments between consecutive waypoints
        for i in range(len(waypoints) - 1):
            d1, pos1 = waypoints[i]
            d2, pos2 = waypoints[i + 1]

            dist = math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)
            if dist < 0.01:
                continue

            start_node = find_node_id(nodes, pos1[0], pos1[1])
            end_node = find_node_id(nodes, pos2[0], pos2[1])

            if start_node and end_node and start_node != end_node:
                segments.append({
                    "id": seg_id,
                    "startNodeId": start_node,
                    "endNodeId": end_node,
                    "speed": 1000,
                })
                seg_id += 1

    return segments


def check_connectivity(nodes, segments):
    """
    Check if the network graph is fully connected.
    Returns (is_connected, num_components, components, isolated_nodes).
    """
    if not nodes:
        return True, 0, [], set()

    adj = defaultdict(set)
    node_ids = {n["id"] for n in nodes}
    nodes_in_segments = set()

    for s in segments:
        adj[s["startNodeId"]].add(s["endNodeId"])
        adj[s["endNodeId"]].add(s["startNodeId"])
        nodes_in_segments.add(s["startNodeId"])
        nodes_in_segments.add(s["endNodeId"])

    isolated_nodes = node_ids - nodes_in_segments

    visited = set()
    components = []

    for node_id in sorted(nodes_in_segments):
        if node_id in visited:
            continue
        component = set()
        queue = [node_id]
        while queue:
            current = queue.pop(0)
            if current in visited:
                continue
            visited.add(current)
            component.add(current)
            for neighbor in adj[current]:
                if neighbor not in visited:
                    queue.append(neighbor)
        components.append(component)

    # Sort components by size descending
    components.sort(key=lambda c: len(c), reverse=True)

    return len(components) <= 1, len(components), components, isolated_nodes


def _seg_approx_length(seg, node_map):
    n1 = node_map[seg["startNodeId"]]
    n2 = node_map[seg["endNodeId"]]
    return math.hypot(n2["x"] - n1["x"], n2["y"] - n1["y"])


def _is_between_branch_nodes(nid, adj_in, adj_out):
    """Check if a pass-through node sits between a diverge and a merge node.
    Trace forward to see if we hit a merge before hitting another diverge/dead-end."""
    # Trace forward
    cur = nid
    for _ in range(20):  # limit depth
        outs = adj_out.get(cur, [])
        if not outs:
            return False
        nxt = outs[0]["endNodeId"]
        nxt_in = len(adj_in.get(nxt, []))
        nxt_out = len(adj_out.get(nxt, []))
        if nxt_in >= 2:
            return True   # reached a merge node
        if nxt_out != 1 or nxt_in != 1:
            return False  # not a simple chain
        cur = nxt
    return False


def merge_curve_transitions(nodes, segments):
    """
    Remove pass-through nodes (in=1, out=1) in two cases:
    1. Curve transitions: at least one adjacent segment is arc, combined < 5000mm
    2. Diverge-to-merge chains: node sits between a diverge and merge node
       (these are CPOINT nodes like BlockClaim/BlockRelease that don't need
       to be graph nodes — removing them extends ZCU entry range)
    """
    node_map = {n["id"]: n for n in nodes}
    adj_out = defaultdict(list)
    adj_in = defaultdict(list)

    for s in segments:
        adj_out[s["startNodeId"]].append(s)
        adj_in[s["endNodeId"]].append(s)

    # Collect all eligible nodes first (don't modify during iteration)
    to_merge = []
    for n in nodes:
        nid = n["id"]
        ins = adj_in.get(nid, [])
        outs = adj_out.get(nid, [])

        if len(ins) != 1 or len(outs) != 1:
            continue

        seg_in = ins[0]
        seg_out = outs[0]

        # Case 1: curve transition (at least one arc, short combined length)
        has_arc = seg_in.get("parts") or seg_out.get("parts")
        if has_arc:
            l1 = _seg_approx_length(seg_in, node_map)
            l2 = _seg_approx_length(seg_out, node_map)
            if l1 + l2 <= 5000:
                to_merge.append(nid)
                continue

        # Case 2: between diverge and merge (CPOINT nodes on ZCU approach)
        if _is_between_branch_nodes(nid, adj_in, adj_out):
            to_merge.append(nid)
            continue

    # Perform merges
    removed = set()
    for nid in to_merge:
        if nid in removed:
            continue
        ins = adj_in.get(nid, [])
        outs = adj_out.get(nid, [])
        if len(ins) != 1 or len(outs) != 1:
            continue

        seg_in = ins[0]
        seg_out = outs[0]

        # Skip if either endpoint was already removed
        if seg_in["startNodeId"] in removed or seg_out["endNodeId"] in removed:
            continue

        # Build merged parts
        merged_parts = []
        if seg_in.get("parts"):
            merged_parts.extend(seg_in["parts"])
        else:
            end_n = node_map[nid]
            merged_parts.append({
                "kind": "Straight",
                "points": [{"x": end_n["x"], "y": end_n["y"]}]
            })

        if seg_out.get("parts"):
            merged_parts.extend(seg_out["parts"])
        else:
            end_n = node_map[seg_out["endNodeId"]]
            merged_parts.append({
                "kind": "Straight",
                "points": [{"x": end_n["x"], "y": end_n["y"]}]
            })

        new_seg = {
            "id": seg_in["id"],
            "startNodeId": seg_in["startNodeId"],
            "endNodeId": seg_out["endNodeId"],
            "speed": seg_in.get("speed", 1000),
            "parts": merged_parts,
        }

        # Replace in lists
        segments.remove(seg_in)
        segments.remove(seg_out)
        adj_out[seg_in["startNodeId"]].remove(seg_in)
        adj_in[nid].remove(seg_in)
        adj_out[nid].remove(seg_out)
        adj_in[seg_out["endNodeId"]].remove(seg_out)

        segments.append(new_seg)
        adj_out[new_seg["startNodeId"]].append(new_seg)
        adj_in[new_seg["endNodeId"]].append(new_seg)

        removed.add(nid)

    # Remove merged nodes
    nodes = [n for n in nodes if n["id"] not in removed]

    # Re-number segment IDs
    for i, s in enumerate(segments, 1):
        s["id"] = i

    return nodes, segments


def merge_arc_passthrough(nodes, segments):
    """
    Remove pass-through nodes (in-degree=1, out-degree=1) where at least one
    adjacent segment is an arc. Merge the two segments into one, concatenating
    parts. This eliminates unnecessary nodes on curves.
    """
    node_map = {n["id"]: n for n in nodes}
    adj_out = defaultdict(list)  # node -> [segment]
    adj_in = defaultdict(list)

    for s in segments:
        adj_out[s["startNodeId"]].append(s)
        adj_in[s["endNodeId"]].append(s)

    changed = True
    while changed:
        changed = False
        for n in list(nodes):
            nid = n["id"]
            ins = adj_in.get(nid, [])
            outs = adj_out.get(nid, [])

            if len(ins) != 1 or len(outs) != 1:
                continue

            seg_in = ins[0]
            seg_out = outs[0]

            # At least one must be arc (has parts)
            if not seg_in.get("parts") and not seg_out.get("parts"):
                continue

            # Merge: seg_in.start -> [nid] -> seg_out.end becomes one segment
            merged_parts = []

            # Parts from incoming segment
            if seg_in.get("parts"):
                merged_parts.extend(seg_in["parts"])
            else:
                # Straight segment: add as Straight part
                end_n = node_map[nid]
                merged_parts.append({
                    "kind": "Straight",
                    "points": [{"x": end_n["x"], "y": end_n["y"]}]
                })

            # Parts from outgoing segment
            if seg_out.get("parts"):
                merged_parts.extend(seg_out["parts"])
            else:
                # Straight segment: add as Straight part
                end_n = node_map[seg_out["endNodeId"]]
                merged_parts.append({
                    "kind": "Straight",
                    "points": [{"x": end_n["x"], "y": end_n["y"]}]
                })

            new_seg = {
                "id": seg_in["id"],
                "startNodeId": seg_in["startNodeId"],
                "endNodeId": seg_out["endNodeId"],
                "speed": seg_in.get("speed", 1000),
                "parts": merged_parts,
            }

            # Remove old segments
            segments.remove(seg_in)
            segments.remove(seg_out)
            adj_out[seg_in["startNodeId"]].remove(seg_in)
            adj_in[nid].remove(seg_in)
            adj_out[nid].remove(seg_out)
            adj_in[seg_out["endNodeId"]].remove(seg_out)

            # Add merged segment
            segments.append(new_seg)
            adj_out[new_seg["startNodeId"]].append(new_seg)
            adj_in[new_seg["endNodeId"]].append(new_seg)

            # Remove the pass-through node
            nodes.remove(n)
            del node_map[nid]
            if nid in adj_out:
                del adj_out[nid]
            if nid in adj_in:
                del adj_in[nid]

            changed = True
            break  # restart iteration since we modified the list

    # Re-number segment IDs
    for i, s in enumerate(segments, 1):
        s["id"] = i

    return nodes, segments


def main():
    print("Parsing input file...")
    paths, cpoints = parse_file(INPUT_FILE)
    print(f"  Parsed {len(paths)} paths, {len(cpoints)} control points")

    # Build path lookup
    path_lookup = {p["name"]: p for p in paths}

    # Verify all cpoint paths exist
    missing_paths = set()
    for cp in cpoints:
        if cp["path"] not in path_lookup:
            missing_paths.add(cp["path"])
    if missing_paths:
        print(f"  WARNING: {len(missing_paths)} CPOINTs reference missing paths: {sorted(missing_paths)[:10]}...")

    print("\nDetecting junction points (where path endpoints meet other path bodies)...")
    junctions = find_junctions(paths)
    total_junctions = sum(len(v) for v in junctions.values())
    print(f"  Found {total_junctions} junction points on {len(junctions)} paths")

    print("\nBuilding nodes...")
    nodes = build_node_map(paths, cpoints, path_lookup, junctions)
    print(f"  Created {len(nodes)} nodes")

    print("\nBuilding segments...")
    segments = build_segments(paths, cpoints, path_lookup, nodes, junctions)
    print(f"  Created {len(segments)} segments")

    print("\nMerging curve transition nodes...")
    prev_count = len(nodes) + 1
    while len(nodes) < prev_count:
        prev_count = len(nodes)
        nodes, segments = merge_curve_transitions(nodes, segments)
    print(f"  After merge: {len(nodes)} nodes, {len(segments)} segments")

    print("\nChecking connectivity...")
    is_connected, num_components, components, isolated_nodes = check_connectivity(nodes, segments)

    print(f"\n{'='*60}")
    print("CONNECTIVITY ANALYSIS")
    print(f"{'='*60}")
    if is_connected and len(isolated_nodes) == 0:
        print("  Network is FULLY CONNECTED (single component)")
    elif is_connected:
        print("  All segment-connected nodes form a SINGLE component")
        if isolated_nodes:
            print(f"  {len(isolated_nodes)} isolated nodes (not in any segment)")
    else:
        print(f"  Network has {num_components} connected components")
        for i, comp in enumerate(components):
            comp_nodes = [n for n in nodes if n["id"] in comp]
            xs = [n["x"] for n in comp_nodes]
            ys = [n["y"] for n in comp_nodes]
            print(f"    Component {i+1}: {len(comp)} nodes, "
                  f"x range [{min(xs)}, {max(xs)}], y range [{min(ys)}, {max(ys)}]")
            if len(comp) <= 10:
                for n in comp_nodes:
                    print(f"      Node {n['id']} at ({n['x']}, {n['y']})")
        if isolated_nodes:
            print(f"\n  {len(isolated_nodes)} isolated nodes (not in any segment):")
            iso_list = [n for n in nodes if n["id"] in isolated_nodes]
            for n in iso_list[:20]:
                infos = n["_infos"]
                info_str = ", ".join(f"{t}:{v}" for t, v in sorted(infos))
                print(f"    Node {n['id']} at ({n['x']}, {n['y']}) from: {info_str}")
            if len(iso_list) > 20:
                print(f"    ... and {len(iso_list) - 20} more")

    # Prepare output JSON
    output_nodes = []
    for n in nodes:
        output_nodes.append({
            "id": n["id"],
            "x": n["x"],
            "y": n["y"],
            "z": n["z"],
            "area": n["area"],
        })

    output = {
        "schema": "xmsmap-v4.0",
        "info": {"name": "LargeOHT", "version": "1.0"},
        "nodes": output_nodes,
        "segments": segments,
    }

    print(f"\nWriting output to {OUTPUT_FILE}...")
    with open(OUTPUT_FILE, "w") as f:
        json.dump(output, f, indent=2)

    print(f"\n{'='*60}")
    print("STATISTICS")
    print(f"{'='*60}")
    print(f"  Paths parsed:      {len(paths)}")
    print(f"    - Line paths:    {sum(1 for p in paths if p['type'] == 'line')}")
    print(f"    - Arc paths:     {sum(1 for p in paths if p['type'] == 'arc')}")
    print(f"  Control points:    {len(cpoints)}")
    print(f"  Junction points:   {total_junctions}")
    print(f"  Nodes created:     {len(nodes)}")
    print(f"  Segments created:  {len(segments)}")
    print(f"  Connected components: {num_components}")
    print(f"  Isolated nodes:    {len(isolated_nodes)}")
    print(f"\nOutput written to: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
