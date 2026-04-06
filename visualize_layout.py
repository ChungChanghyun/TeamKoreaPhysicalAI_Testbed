"""
LargeOHT Layout Visualizer using Pygame
- Pan: right-click drag or arrow keys
- Zoom: mouse wheel
- Click node: show info
- R: reset view
- G: toggle grid
- N: toggle node dots
- L: toggle node labels (when zoomed in)
- C: toggle CPOINT highlight
- ESC: quit
"""

import json
import math
import pygame
import sys

# ── Load data ──────────────────────────────────────────────────────────────
with open("LargeOHT.json", "r") as f:
    data = json.load(f)

nodes = {n["id"]: n for n in data["nodes"]}
segments = data["segments"]

# Precompute node positions as floats
node_pos = {nid: (n["x"], n["y"]) for nid, n in nodes.items()}

# Classify nodes: CPOINT nodes vs junction nodes (by naming convention)
cpoint_nodes = set()
junction_nodes = set()
for nid in nodes:
    if nid.startswith("CP_") or nid.startswith("N"):
        pass  # default
    # We'll just color by degree later

# Precompute degree for each node
node_degree = {nid: 0 for nid in nodes}
for seg in segments:
    node_degree[seg["startNodeId"]] = node_degree.get(seg["startNodeId"], 0) + 1
    node_degree[seg["endNodeId"]] = node_degree.get(seg["endNodeId"], 0) + 1

# Compute bounding box
all_x = [n["x"] for n in data["nodes"]]
all_y = [n["y"] for n in data["nodes"]]
min_x, max_x = min(all_x), max(all_x)
min_y, max_y = min(all_y), max(all_y)
world_w = max_x - min_x
world_h = max_y - min_y
center_x = (min_x + max_x) / 2
center_y = (min_y + max_y) / 2

# ── Pygame setup ───────────────────────────────────────────────────────────
pygame.init()

# Get display size for windowed mode
disp_info = pygame.display.Info()
SCREEN_W = min(1600, disp_info.current_w - 100)
SCREEN_H = min(900, disp_info.current_h - 100)

screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.RESIZABLE)
pygame.display.set_caption("LargeOHT Layout Viewer")

clock = pygame.time.Clock()
font_small = pygame.font.SysFont("consolas", 12)
font_med = pygame.font.SysFont("consolas", 14)
font_large = pygame.font.SysFont("consolas", 18)

# ── Colors ─────────────────────────────────────────────────────────────────
BG_COLOR = (18, 18, 24)
GRID_COLOR = (35, 35, 50)
GRID_COLOR_MAJOR = (50, 50, 70)
PATH_COLOR = (80, 160, 255)
PATH_COLOR_HIGHLIGHT = (120, 220, 255)
NODE_COLOR = (255, 200, 60)
NODE_JUNCTION = (255, 100, 80)
NODE_ENDPOINT = (100, 255, 130)
CPOINT_COLOR = (255, 120, 200)
SELECT_COLOR = (255, 255, 100)
TEXT_COLOR = (200, 200, 200)
INFO_BG = (30, 30, 45, 220)

# ── Camera state ───────────────────────────────────────────────────────────
margin = 1.1
scale = min(SCREEN_W / (world_w * margin), SCREEN_H / (world_h * margin))
cam_x = center_x  # world coordinate at screen center
cam_y = center_y
initial_scale = scale
initial_cam_x = cam_x
initial_cam_y = cam_y

# Toggle states
show_grid = True
show_nodes = True
show_labels = False
show_cpoints = True

# Interaction state
dragging = False
drag_start = None
selected_node = None
hovered_node = None


def world_to_screen(wx, wy):
    sx = (wx - cam_x) * scale + SCREEN_W / 2
    sy = -(wy - cam_y) * scale + SCREEN_H / 2  # flip Y
    return int(sx), int(sy)


def screen_to_world(sx, sy):
    wx = (sx - SCREEN_W / 2) / scale + cam_x
    wy = -(sy - SCREEN_H / 2) / scale + cam_y
    return wx, wy


def draw_grid():
    """Draw adaptive grid based on zoom level."""
    # Determine grid spacing based on zoom
    world_visible_w = SCREEN_W / scale
    target_cells = 15
    raw_spacing = world_visible_w / target_cells

    # Snap to nice intervals
    nice = [100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000]
    spacing = nice[0]
    for n in nice:
        if n >= raw_spacing:
            spacing = n
            break
    else:
        spacing = nice[-1]

    # Visible world range
    w_left, w_top = screen_to_world(0, 0)
    w_right, w_bot = screen_to_world(SCREEN_W, SCREEN_H)
    w_min_y = min(w_top, w_bot)
    w_max_y = max(w_top, w_bot)

    start_x = int(w_left // spacing) * spacing
    start_y = int(w_min_y // spacing) * spacing

    x = start_x
    while x <= w_right:
        sx, _ = world_to_screen(x, 0)
        color = GRID_COLOR_MAJOR if x % (spacing * 5) == 0 else GRID_COLOR
        pygame.draw.line(screen, color, (sx, 0), (sx, SCREEN_H), 1)
        x += spacing

    y = start_y
    while y <= w_max_y:
        _, sy = world_to_screen(0, y)
        color = GRID_COLOR_MAJOR if y % (spacing * 5) == 0 else GRID_COLOR
        pygame.draw.line(screen, color, (0, sy), (SCREEN_W, sy), 1)
        y += spacing


def draw_segments():
    """Draw all path segments as lines."""
    # Frustum cull: compute visible world rect
    w_left, w_top = screen_to_world(0, 0)
    w_right, w_bot = screen_to_world(SCREEN_W, SCREEN_H)
    vis_min_x = min(w_left, w_right) - 1000
    vis_max_x = max(w_left, w_right) + 1000
    vis_min_y = min(w_top, w_bot) - 1000
    vis_max_y = max(w_top, w_bot) + 1000

    line_w = max(1, min(3, int(scale * 50)))

    for seg in segments:
        sid = seg["startNodeId"]
        eid = seg["endNodeId"]
        x1, y1 = node_pos[sid]
        x2, y2 = node_pos[eid]

        # Frustum culling
        seg_min_x = min(x1, x2)
        seg_max_x = max(x1, x2)
        seg_min_y = min(y1, y2)
        seg_max_y = max(y1, y2)
        if seg_max_x < vis_min_x or seg_min_x > vis_max_x:
            continue
        if seg_max_y < vis_min_y or seg_min_y > vis_max_y:
            continue

        s1 = world_to_screen(x1, y1)
        s2 = world_to_screen(x2, y2)

        # Highlight if connected to selected node
        if selected_node and (sid == selected_node or eid == selected_node):
            pygame.draw.line(screen, SELECT_COLOR, s1, s2, line_w + 1)
        else:
            pygame.draw.line(screen, PATH_COLOR, s1, s2, line_w)


def draw_nodes():
    """Draw node dots, colored by connectivity."""
    if not show_nodes:
        return

    # Frustum cull
    w_left, w_top = screen_to_world(0, 0)
    w_right, w_bot = screen_to_world(SCREEN_W, SCREEN_H)
    vis_min_x = min(w_left, w_right) - 500
    vis_max_x = max(w_left, w_right) + 500
    vis_min_y = min(w_top, w_bot) - 500
    vis_max_y = max(w_top, w_bot) + 500

    r = max(2, min(6, int(scale * 80)))
    zoom_threshold_labels = 0.05  # show labels when zoomed in enough

    for nid, (wx, wy) in node_pos.items():
        if wx < vis_min_x or wx > vis_max_x or wy < vis_min_y or wy > vis_max_y:
            continue

        sx, sy = world_to_screen(wx, wy)
        deg = node_degree.get(nid, 0)

        if nid == selected_node:
            color = SELECT_COLOR
            cr = r + 3
        elif nid == hovered_node:
            color = (255, 255, 255)
            cr = r + 2
        elif deg == 1:
            color = NODE_ENDPOINT  # dead end
            cr = r + 1
        elif deg >= 4:
            color = NODE_JUNCTION  # junction
            cr = r + 1
        else:
            color = NODE_COLOR
            cr = r

        pygame.draw.circle(screen, color, (sx, sy), cr)

        # Labels when very zoomed in
        if show_labels and scale > zoom_threshold_labels:
            label = font_small.render(nid, True, TEXT_COLOR)
            screen.blit(label, (sx + cr + 2, sy - 6))


def draw_info_panel():
    """Draw info panel with stats and selected node info."""
    lines = [
        f"Nodes: {len(nodes)}  Segments: {len(segments)}",
        f"Zoom: {scale:.6f}  ({scale/initial_scale:.1f}x)",
        f"Center: ({cam_x:.0f}, {cam_y:.0f})",
    ]

    if selected_node:
        n = nodes[selected_node]
        deg = node_degree.get(selected_node, 0)
        lines.append("")
        lines.append(f"Selected: {selected_node}")
        lines.append(f"  Position: ({n['x']:.1f}, {n['y']:.1f})")
        lines.append(f"  Degree: {deg}")
        # Find connected segments
        connected = []
        for seg in segments:
            if seg["startNodeId"] == selected_node:
                connected.append(f"  -> {seg['endNodeId']} (seg {seg['id']})")
            elif seg["endNodeId"] == selected_node:
                connected.append(f"  <- {seg['startNodeId']} (seg {seg['id']})")
        for c in connected[:8]:
            lines.append(c)
        if len(connected) > 8:
            lines.append(f"  ... +{len(connected)-8} more")

    # Draw panel background
    panel_w = 340
    panel_h = 18 * len(lines) + 16
    panel_surf = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    panel_surf.fill((30, 30, 45, 200))
    screen.blit(panel_surf, (8, 8))

    for i, line in enumerate(lines):
        txt = font_small.render(line, True, TEXT_COLOR)
        screen.blit(txt, (14, 14 + i * 18))


def draw_help():
    """Draw help text at bottom."""
    help_text = "RClick-drag:Pan  Wheel:Zoom  Click:Select  R:Reset  G:Grid  N:Nodes  L:Labels  ESC:Quit"
    txt = font_small.render(help_text, True, (120, 120, 150))
    screen.blit(txt, (10, SCREEN_H - 22))


def find_node_at(sx, sy, threshold=15):
    """Find nearest node to screen position."""
    wx, wy = screen_to_world(sx, sy)
    best = None
    best_dist = float("inf")
    world_thresh = threshold / scale

    for nid, (nx, ny) in node_pos.items():
        d = math.hypot(nx - wx, ny - wy)
        if d < world_thresh and d < best_dist:
            best = nid
            best_dist = d
    return best


# ── Main loop ──────────────────────────────────────────────────────────────
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            elif event.key == pygame.K_r:
                cam_x, cam_y, scale = initial_cam_x, initial_cam_y, initial_scale
                selected_node = None
            elif event.key == pygame.K_g:
                show_grid = not show_grid
            elif event.key == pygame.K_n:
                show_nodes = not show_nodes
            elif event.key == pygame.K_l:
                show_labels = not show_labels
            elif event.key == pygame.K_c:
                show_cpoints = not show_cpoints

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 3:  # right click - start pan
                dragging = True
                drag_start = event.pos
            elif event.button == 1:  # left click - select
                selected_node = find_node_at(*event.pos)
            elif event.button == 4:  # scroll up - zoom in
                mx, my = event.pos
                wx, wy = screen_to_world(mx, my)
                scale *= 1.15
                # Adjust camera so world point stays under cursor
                cam_x = wx - (mx - SCREEN_W / 2) / scale
                cam_y = wy + (my - SCREEN_H / 2) / scale
            elif event.button == 5:  # scroll down - zoom out
                mx, my = event.pos
                wx, wy = screen_to_world(mx, my)
                scale /= 1.15
                cam_x = wx - (mx - SCREEN_W / 2) / scale
                cam_y = wy + (my - SCREEN_H / 2) / scale

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 3:
                dragging = False

        elif event.type == pygame.MOUSEMOTION:
            if dragging:
                dx = event.pos[0] - drag_start[0]
                dy = event.pos[1] - drag_start[1]
                cam_x -= dx / scale
                cam_y += dy / scale
                drag_start = event.pos
            else:
                hovered_node = find_node_at(*event.pos)

        elif event.type == pygame.VIDEORESIZE:
            SCREEN_W, SCREEN_H = event.w, event.h
            screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.RESIZABLE)

    # Arrow key panning
    keys = pygame.key.get_pressed()
    pan_speed = 500 / scale
    if keys[pygame.K_LEFT]:
        cam_x -= pan_speed
    if keys[pygame.K_RIGHT]:
        cam_x += pan_speed
    if keys[pygame.K_UP]:
        cam_y += pan_speed
    if keys[pygame.K_DOWN]:
        cam_y -= pan_speed

    # ── Draw ───────────────────────────────────────────────────────────────
    screen.fill(BG_COLOR)

    if show_grid:
        draw_grid()

    draw_segments()
    draw_nodes()
    draw_info_panel()
    draw_help()

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()
