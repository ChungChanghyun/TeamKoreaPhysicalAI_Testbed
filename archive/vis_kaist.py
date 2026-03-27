"""
vis_kaist.py — KaistTB_map.json 레이아웃 시각화 (경로 탐색 없음)

Controls
--------
  Mouse drag    : 화면 이동 (pan)
  Mouse wheel   : 확대/축소 (zoom)
  R             : 화면 리셋
  A             : area 색상 on/off
  N             : 노드 ID 표시 on/off (줌인 시 자동)
  P             : 포트 강조 on/off
  Q / ESC       : 종료
"""
from __future__ import annotations
import sys, os, json, math
import pygame

JSON_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'KaistTB_map.json')

# ── 색상 ──────────────────────────────────────────────────────────────────────
BG           = (20,  20,  25)
COL_NODE     = (140, 160, 180)
COL_SEG      = (70,  90, 110)
COL_SEG_ARR  = (100, 130, 160)   # arrowhead
COL_TEXT     = (200, 210, 220)
COL_DIM      = (80,  90, 100)

# area → 색상
AREA_COLORS = {
    '3DS_F1': (80,  160, 220),   # 파랑
    '3DS_F2': (80,  200, 130),   # 초록
    '3DS_F3': (200, 130,  80),   # 주황
    'OHT_A':  (180,  80, 200),   # 보라
    'AMR_A':  (200, 200,  60),   # 노랑
    '':       (140, 160, 180),   # 기본
}

# port kind → 색상
PORT_COLORS = {
    'Station': (60,  220, 120),
    'Buffer':  (255, 200,  50),
}

NODE_R   = 4    # 기본 노드 반지름 (px)
PORT_R   = 6    # 포트 노드 반지름 (px)
WIN_W, WIN_H = 1400, 900

# ── 데이터 로드 ───────────────────────────────────────────────────────────────

def load_map(path: str):
    with open(path, 'r', encoding='utf-8') as f:
        d = json.load(f)

    nodes    = {n['id']: n for n in d['nodes']}
    segments = d['segments']
    ports    = d.get('ports', [])
    chargers = d.get('chargers', [])

    port_node_ids    = {p['nodeId']: p for p in ports}
    charger_node_ids = {c['nodeId'] for c in chargers}

    xs = [n['x'] for n in nodes.values()]
    ys = [n['y'] for n in nodes.values()]
    bbox = (min(xs), min(ys), max(xs), max(ys))

    return nodes, segments, port_node_ids, charger_node_ids, bbox

# ── 좌표 변환 (map mm → screen px) ───────────────────────────────────────────

class Camera:
    def __init__(self, bbox, win_w, win_h, pad=40):
        mx0, my0, mx1, my1 = bbox
        mw, mh = mx1 - mx0, my1 - my0
        sx = (win_w - 2 * pad) / mw
        sy = (win_h - 2 * pad) / mh
        self.scale  = min(sx, sy)
        self.offset = [
            pad + (win_w - 2*pad - mw * self.scale) / 2 - mx0 * self.scale,
            pad + (win_h - 2*pad - mh * self.scale) / 2 + (my0 + mh) * self.scale,
        ]
        self._drag_start = None
        self._offset_start = None

    def to_screen(self, mx, my):
        sx = mx * self.scale + self.offset[0]
        sy = -my * self.scale + self.offset[1]   # y-flip
        return int(sx), int(sy)

    def on_mouse_down(self, pos):
        self._drag_start  = pos
        self._offset_start = list(self.offset)

    def on_mouse_move(self, pos):
        if self._drag_start is None:
            return
        dx = pos[0] - self._drag_start[0]
        dy = pos[1] - self._drag_start[1]
        self.offset[0] = self._offset_start[0] + dx
        self.offset[1] = self._offset_start[1] + dy

    def on_mouse_up(self):
        self._drag_start = None

    def on_scroll(self, pos, up: bool):
        factor = 1.15 if up else 1 / 1.15
        mx, my = pos
        self.offset[0] = mx + (self.offset[0] - mx) * factor
        self.offset[1] = my + (self.offset[1] - my) * factor
        self.scale *= factor

# ── 화살표 그리기 ──────────────────────────────────────────────────────────────

def draw_arrow(surf, color, p1, p2, width=1, head_size=6):
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    length = math.hypot(dx, dy)
    if length < 2:
        return
    # 화살표 끝을 노드 중심에서 약간 앞에 위치
    ratio = max(0, (length - head_size * 1.5)) / length
    tip = (p1[0] + dx * ratio, p1[1] + dy * ratio)
    pygame.draw.line(surf, color, p1, (int(tip[0]), int(tip[1])), width)

    # arrowhead
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    h = head_size
    pts = [
        (int(p2[0] - ux*h*1.5 + px*h*0.6),
         int(p2[1] - uy*h*1.5 + py*h*0.6)),
        (int(p2[0]), int(p2[1])),
        (int(p2[0] - ux*h*1.5 - px*h*0.6),
         int(p2[1] - uy*h*1.5 - py*h*0.6)),
    ]
    pygame.draw.polygon(surf, color, pts)


# ── 메인 ──────────────────────────────────────────────────────────────────────

def main():
    nodes, segments, port_node_ids, charger_node_ids, bbox = load_map(JSON_FILE)
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
    pygame.display.set_caption('KaistTB Layout Viewer')
    clock  = pygame.time.Clock()
    font_s = pygame.font.SysFont('consolas', 11)
    font_m = pygame.font.SysFont('consolas', 13)

    cam = Camera(bbox, WIN_W, WIN_H)

    show_area    = True
    show_node_id = False
    show_ports   = True

    def reset_cam():
        nonlocal cam
        cam = Camera(bbox, WIN_W, WIN_H)

    running = True
    while running:
        ww, wh = screen.get_size()

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif ev.key == pygame.K_r:
                    reset_cam()
                elif ev.key == pygame.K_a:
                    show_area = not show_area
                elif ev.key == pygame.K_n:
                    show_node_id = not show_node_id
                elif ev.key == pygame.K_p:
                    show_ports = not show_ports
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.button == 1:
                    cam.on_mouse_down(ev.pos)
                elif ev.button == 4:
                    cam.on_scroll(ev.pos, up=True)
                elif ev.button == 5:
                    cam.on_scroll(ev.pos, up=False)
            elif ev.type == pygame.MOUSEBUTTONUP:
                if ev.button == 1:
                    cam.on_mouse_up()
            elif ev.type == pygame.MOUSEMOTION:
                cam.on_mouse_move(ev.pos)

        screen.fill(BG)

        # ── 세그먼트 그리기 ──────────────────────────────────────────────────
        seg_width = max(1, int(cam.scale * 0.0004))
        head_size = max(4, int(cam.scale * 0.0015))
        for seg in segments:
            n1 = nodes.get(seg['startNodeId'])
            n2 = nodes.get(seg['endNodeId'])
            if not n1 or not n2:
                continue
            p1 = cam.to_screen(n1['x'], n1['y'])
            p2 = cam.to_screen(n2['x'], n2['y'])
            # 화면 밖이면 건너뜀
            if (max(p1[0], p2[0]) < 0 or min(p1[0], p2[0]) > ww or
                max(p1[1], p2[1]) < 0 or min(p1[1], p2[1]) > wh):
                continue
            draw_arrow(screen, COL_SEG_ARR, p1, p2,
                       width=seg_width, head_size=head_size)

        # ── 노드 그리기 ──────────────────────────────────────────────────────
        auto_id = cam.scale > 0.06   # 줌인 시 자동으로 ID 표시
        show_id = show_node_id or auto_id
        node_r  = max(2, int(NODE_R * cam.scale * 0.003))
        port_r  = max(3, int(PORT_R * cam.scale * 0.003))

        for nid, node in nodes.items():
            sx, sy = cam.to_screen(node['x'], node['y'])
            if sx < -20 or sx > ww + 20 or sy < -20 or sy > wh + 20:
                continue

            is_port    = nid in port_node_ids
            is_charger = nid in charger_node_ids

            if is_port and show_ports:
                port_info = port_node_ids[nid]
                col = PORT_COLORS.get(port_info.get('kind', ''), (200, 200, 200))
                pygame.draw.circle(screen, col, (sx, sy), port_r)
                pygame.draw.circle(screen, (255, 255, 255), (sx, sy), port_r, 1)
            elif is_charger:
                pygame.draw.rect(screen, (80, 200, 255),
                                 (sx - port_r, sy - port_r, port_r*2, port_r*2))
            else:
                area  = node.get('area', '')
                col   = AREA_COLORS.get(area, COL_NODE) if show_area else COL_NODE
                pygame.draw.circle(screen, col, (sx, sy), node_r)

            if show_id and cam.scale > 0.04:
                lbl = font_s.render(nid[-5:], True, COL_DIM)
                screen.blit(lbl, (sx + node_r + 1, sy - 6))

        # ── 범례 ─────────────────────────────────────────────────────────────
        legend = []
        if show_area:
            for area, col in AREA_COLORS.items():
                if area:
                    legend.append((col, area))
        if show_ports:
            for kind, col in PORT_COLORS.items():
                legend.append((col, kind))
            legend.append(((80, 200, 255), 'Charger'))

        lx, ly = 12, 12
        for col, label in legend:
            pygame.draw.circle(screen, col, (lx + 6, ly + 6), 5)
            txt = font_s.render(label, True, COL_TEXT)
            screen.blit(txt, (lx + 15, ly))
            ly += 18

        # ── 우측 하단 정보 ────────────────────────────────────────────────────
        info_lines = [
            f'nodes: {len(nodes)}  segs: {len(segments)}  ports: {len(port_node_ids)}',
            f'zoom: {cam.scale:.4f}',
            'drag:pan  wheel:zoom  R:reset  A:area  N:id  P:port  Q:quit',
        ]
        for i, line in enumerate(info_lines):
            txt = font_s.render(line, True, COL_DIM)
            screen.blit(txt, (ww - txt.get_width() - 8,
                               wh - (len(info_lines) - i) * 16 - 4))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == '__main__':
    main()
