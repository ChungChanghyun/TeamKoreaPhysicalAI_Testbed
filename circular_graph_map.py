"""Generate a circular graph map JSON for testing graph_des_v5."""
import json, math


def make_circular_map(n_nodes=40, radius=20000, speed=3600, out_path="circular.map.json"):
    nodes = []
    segments = []

    for i in range(n_nodes):
        angle = 2 * math.pi * i / n_nodes
        nodes.append({
            "id": str(i),
            "x": radius * math.cos(angle),
            "y": radius * math.sin(angle),
        })

    for i in range(n_nodes):
        j = (i + 1) % n_nodes
        segments.append({
            "id": f"seg_{i}_{j}",
            "startNodeId": str(i),
            "endNodeId": str(j),
            "speed": speed,
        })

    data = {
        "version": "xmsmap-v4.0",
        "nodes": nodes,
        "segments": segments,
        "ports": [],
        "vehicleModels": [{"id": "OHT", "dimension": {"length": 750, "width": 500}}],
    }

    with open(out_path, "w") as f:
        json.dump(data, f)
    print(f"Wrote {out_path}: {n_nodes} nodes, {len(segments)} segments")
    return out_path


if __name__ == "__main__":
    make_circular_map()
