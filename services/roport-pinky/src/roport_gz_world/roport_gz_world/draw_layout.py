"""배치 데이터 → 실측 도면 PNG(노드 그래프 포함).

패키지 폴더에서 실행한다(matplotlib 필요, 실행·CI 의존성 아님):
  python3 -m roport_gz_world.draw_layout     # ../../maps/map_measured_<version>.png
"""

import argparse
import os
import sys

from .layout import Layout

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_LAYOUT = os.path.join(PKG_DIR, 'layout', 'roport_airport.yaml')
DEFAULT_MAP_DIR = os.path.normpath(os.path.join(PKG_DIR, '..', '..', 'maps'))

C = {
    'wall': '#2f3640', 'object': '#d9d5c7', 'edge': '#4b4b4b', 'low': '#eeeeee',
    'tape': '#1e6b52', 'guess': '#e1702b', 'dim': '#c0392b', 'zone03': '#eef0fb',
    'node': '#ffffff', 'switch': '#2e4fd8', 'marker': '#b4441c', 'text': '#222222',
}
LABELS = {
    'box_03': '상자 67.4×21', 'camera_tripod': '카메라\n삼각대', 'warehouse_box': '창고박스\n(높이 11)',
    'omx3_base': 'OMX3 받침', 'shelf_02': '선반', 'omx12_left': 'OMX1', 'omx12_right': 'OMX2',
    'omx12_back': '',
}


def _korean_font():
    """Noto Sans CJK(.ttc)를 matplotlib 에 등록하고 글꼴 이름을 돌려준다."""
    from matplotlib import font_manager
    found = None
    for style in ('Regular', 'Bold'):
        path = f'/usr/share/fonts/opentype/noto/NotoSansCJK-{style}.ttc'
        if os.path.exists(path):
            font_manager.fontManager.addfont(path)
            found = found or font_manager.FontProperties(fname=path).get_name()
    if found is None:
        sys.exit('Noto Sans CJK not found (apt install fonts-noto-cjk)')
    return found


def _dim(ax, p0, p1, text, offset=(0, 0), color=C['dim'], rot=0):
    ax.annotate('', xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle='<->', color=color, lw=1.0, shrinkA=0, shrinkB=0))
    mx, my = (p0[0] + p1[0]) / 2 + offset[0], (p0[1] + p1[1]) / 2 + offset[1]
    ax.text(mx, my, text, color=color, fontsize=9, ha='center', va='center', rotation=rot,
            bbox=dict(fc='white', ec='none', pad=0.6, alpha=0.85))


def draw(layout, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    plt.rcParams['font.family'] = _korean_font()
    raw = layout.raw
    x0, y0, x1, y1 = layout.room()
    fig = plt.figure(figsize=(18, 12.5), dpi=110)
    ax = fig.add_axes([0.03, 0.37, 0.94, 0.58])
    ax.set_aspect('equal')
    ax.set_xlim(x0 - 25, x1 + 25)
    ax.set_ylim(y0 - 40, y1 + 30)
    ax.axis('off')

    # 03 공용 내부공간 바탕
    z = next(w for w in raw['walls'] if w['name'] == 'zone03_bottom')
    ax.add_patch(Rectangle((z['from'][0], z['from'][1]), x1 - z['from'][0], y1 - z['from'][1],
                           fc=C['zone03'], ec='none', zorder=0))

    # 물체
    for o in raw.get('objects') or []:
        (ox0, oy0), (ox1, oy1) = o['min'], o['max']
        low = float(o.get('height', layout.object_height)) <= layout.map.lidar_height
        guess = bool(o.get('provisional'))
        ax.add_patch(Rectangle((ox0, oy0), ox1 - ox0, oy1 - oy0,
                               fc=C['low'] if low else C['object'],
                               ec=C['guess'] if guess else C['edge'], lw=1.2,
                               ls='--' if guess or low else '-', zorder=2))
        label = LABELS.get(o['name'], o['name'])
        if label:
            ax.text((ox0 + ox1) / 2, (oy0 + oy1) / 2, label, fontsize=8.5, ha='center',
                    va='center', color=C['text'], zorder=3)

    # 벽
    t = layout.wall_thickness
    for w in raw['walls']:
        (ax_, ay_), (bx_, by_) = w['from'], w['to']
        ax.plot([ax_, bx_], [ay_, by_], color=C['wall'], lw=4, solid_capstyle='projecting',
                zorder=4)
    del t

    # 테이프(노드 연결 + 가로선): 실제 폭으로
    hw = layout.line_width / 2
    for a, b in layout.edges():
        guess = layout.edge_provisional(a, b)
        rx0, rx1 = sorted((a.x, b.x))
        ry0, ry1 = sorted((a.y, b.y))
        ax.add_patch(Rectangle((rx0 - hw, ry0 - hw), rx1 - rx0 + 2 * hw, ry1 - ry0 + 2 * hw,
                               fc=C['guess'] if guess else C['tape'], ec='none', zorder=5))
    half = layout.bar_length / 2
    for n in layout.nodes:
        if not n.bar:
            continue
        w_, h_ = (2 * half, 2 * hw) if n.bar == 'x' else (2 * hw, 2 * half)
        ax.add_patch(Rectangle((n.x - w_ / 2, n.y - h_ / 2), w_, h_,
                               fc=C['guess'] if n.provisional else C['tape'], ec='none',
                               zorder=5))

    # 노드
    for n in layout.nodes:
        edge = C['switch'] if n.nav2_switch else (C['guess'] if n.provisional else C['tape'])
        face = C['switch'] if n.nav2_switch else (edge if n.bar else C['node'])
        ax.add_patch(Circle((n.x, n.y), 4.2, fc=face, ec=edge, lw=1.6, zorder=6))
        dx, dy, ha = 0, 8.5, 'center'
        if n.bar == 'x' and n.y < 60:
            dy = -9
        if n.id in ('N10', 'N02'):
            dx, dy, ha = -6, 0, 'right'
        if n.id in ('N05', 'N07'):
            dx, dy, ha = 6, 6, 'left'
        if n.id == 'N12':
            dx, dy, ha = 7, 0, 'left'
        txt = n.id + (f' ({n.place})' if n.place else '')
        ax.text(n.x + dx, n.y + dy, txt, fontsize=9, fontweight='bold', ha=ha, va='center',
                color=edge, zorder=7,
                bbox=dict(fc='white', ec='none', pad=0.4, alpha=0.8))

    # 마커
    for m in (raw.get('markers') or {}).get('items') or []:
        mx, my = m['center']
        ax.add_patch(Rectangle((mx - 4, my - 1.5), 8, 3, fc=C['marker'], ec='none', zorder=6))
        dy = -6 if my > 200 else 6
        ax.text(mx + (6 if m['name'].startswith('G') else 0), my + dy, m['name'], fontsize=8,
                color=C['marker'], ha='left' if m['name'].startswith('G') else 'center',
                va='center', zorder=7)

    # 구역 이름
    ax.text(260, 200, '03 공용 내부공간 (Nav2)', fontsize=15, color=C['switch'], ha='center',
            fontweight='bold')
    ax.text(66, 120, '01 물류창고 루프', fontsize=11, color=C['tape'], ha='center')
    ax.text(170, 30, '01 | 02', fontsize=9, color='#777777', ha='center')
    ax.text(290, 12, '02 인도장', fontsize=12, color='#8a5a00', ha='center', fontweight='bold')
    ax.text(362, 50, '스마트카트\n대기·충전', fontsize=10, color='#4b3fa0', ha='left',
            va='center')
    ax.text(412, 100, '입구', fontsize=11, color=C['guess'], ha='center', fontweight='bold')

    # 주요 치수(cm)
    _dim(ax, (0, y1 + 18), (138, y1 + 18), 'G 138')
    _dim(ax, (138, y1 + 18), (x1, y1 + 18), 'H 306.5')
    _dim(ax, (x1 + 15, 0), (x1 + 15, y1), 'e 221', rot=90)
    _dim(ax, (150, 92), (150, y1), 'f 129', offset=(5, 25), rot=90)
    _dim(ax, (138, 85), (379.5, 85), 'I 241.5', offset=(-60, 0))
    _dim(ax, (379.5, 85), (x1, 85), '입구 65')
    _dim(ax, (0, 100), (20.5, 100), 'a 20.5', offset=(0, 5))
    _dim(ax, (8, 0), (8, 72), 'b 72', rot=90)
    _dim(ax, (20.5, 145), (113.7, 145), 'D 93.2', offset=(0, 4))
    _dim(ax, (124, 72), (124, 163.3), 'c 91.3', rot=90)
    _dim(ax, (20.5, -22), (379.5, -22), 'O 359 (루프 바깥선 → 간선 끝)')
    _dim(ax, (44.85, 205), (89.35, 205), 'S 44.5', offset=(0, 4))
    _dim(ax, (308.0, -10), (339.5, -10), 'ㅈ 31.5')
    _dim(ax, (339.5, -10), (378.3, -10), 'ㄱ 38.8')
    for nid, label in (('N16', 'R 25'), ('N01', 'Q 29.9'), ('N11', 'W 41'), ('H2', 'U 40')):
        n = next(k for k in layout.nodes if k.id == nid)
        ax.text(n.x + 3, (n.y + (163.3 if nid == 'N16' else 72)) / 2, label, fontsize=8.5,
                color=C['dim'], ha='left', va='center', zorder=7)

    ax.set_title(f'시연 공간 실측 도면 {layout.version} (단위 cm · 위 = 창가, 아래 = 통로 · '
                 f'노드 그래프 {layout.graph_version})', fontsize=15, loc='left', pad=12)
    ax.text(x0 - 20, y0 - 37,
            '원점(0,0) = 콘센트 벽·통로 쪽 모서리 · x = 길이, y = 창가 방향 · 노드 좌표 = 테이프 '
            '중심선(테이프 24 mm, 치수는 선 바깥 기준) · 가로선 12 cm · 진한 초록 = 실측, 주황 = 위치 '
            '추정·가정 · 파란 노드 = Nav2 전환 · 점선 회색 물체 = 라이다(12.5 cm)보다 낮음',
            fontsize=8.5, color='#666666', ha='left', va='bottom')

    # 노드 표
    tax = fig.add_axes([0.03, 0.01, 0.94, 0.34])
    tax.axis('off')
    rows = []
    for n in sorted(layout.nodes, key=lambda k: (k.id[0] != 'N', int(k.id[1:]))):
        note = 'Nav2 전환' if n.nav2_switch else ('위치 추정' if n.provisional else '')
        rows.append([n.id, n.place or '—', n.role, f'{n.x:.1f}', f'{n.y:.1f}',
                     ' · '.join(n.links), note])
    tab = tax.table(cellText=rows, colLabels=['노드', '운영 위치', '역할', 'x (cm)', 'y (cm)',
                                              '연결', '비고'],
                    colWidths=[0.06, 0.08, 0.28, 0.07, 0.07, 0.2, 0.1], loc='upper center',
                    cellLoc='center')
    tab.auto_set_font_size(False)
    tab.set_fontsize(9)
    tab.scale(1, 1.22)
    for (r, _c), cell in tab.get_celld().items():
        cell.set_edgecolor('#bbbbbb')
        if r == 0:
            cell.set_facecolor('#e8ecef')
            cell.set_text_props(fontweight='bold')
        elif rows[r - 1][6] == '위치 추정':
            cell.set_text_props(color=C['guess'])
        elif rows[r - 1][6] == 'Nav2 전환':
            cell.set_text_props(color=C['switch'])

    fig.savefig(path, dpi=110, metadata={'Software': None})
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--layout', default=DEFAULT_LAYOUT)
    ap.add_argument('--output', default=None)
    args = ap.parse_args(argv)
    layout = Layout.load(args.layout)
    out = args.output or os.path.join(DEFAULT_MAP_DIR, f'map_measured_{layout.version}.png')
    draw(layout, out)
    print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
