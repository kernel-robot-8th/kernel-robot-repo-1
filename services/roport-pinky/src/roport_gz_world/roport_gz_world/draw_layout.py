"""배치 데이터 → 실측 도면 PNG(노드 그래프 포함).

패키지 폴더에서 실행한다(matplotlib 필요, 실행·CI 의존성 아님):
  python3 -m roport_gz_world.draw_layout     # ../../maps/map_measured_<version>.png, _02.png
"""

import argparse
import math
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
# 02 인도장 확대도(map_measured_<version>_02.png)에만 그리는 치수(재측정 기록지 번호)
DETAIL_02 = {'[2]', '[3]', '[4]', '[6]', '[8]', '[10]', '[15]', '[13]'}


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


def draw(layout, path, detail=False):
    """detail=True 면 02 인도장 확대도(치수 [2]~[15])만 그린다."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    plt.rcParams['font.family'] = _korean_font()
    raw = layout.raw
    x0, y0, x1, y1 = layout.room()
    if detail:
        fig = plt.figure(figsize=(14, 9.2), dpi=110)
        ax = fig.add_axes([0.02, 0.02, 0.96, 0.9])
        ax.set_aspect('equal')
        ax.set_xlim(128, 296)
        ax.set_ylim(-6, 100)
    else:
        fig = plt.figure(figsize=(18, 17), dpi=110)
        ax = fig.add_axes([0.03, 0.538, 0.94, 0.4265])
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

    # 마커: 벽 = 벽을 따라 긴 막대, 바닥 = 작은 사각형
    markers = (raw.get('markers') or {}).get('items') or []
    for m in markers:
        mx, my = m['center']
        if m.get('floor'):
            # 이름은 노드 이름(P4·P7)과 겹치므로 표에만 적는다
            ax.add_patch(Rectangle((mx - 2, my - 2), 4, 4, fc=C['marker'], ec='none', zorder=8))
            continue
        along_y = abs(math.sin(math.radians(m.get('facing_deg', 0)))) < 0.5
        w_, h_ = (3, 8) if along_y else (8, 3)
        ax.add_patch(Rectangle((mx - w_ / 2, my - h_ / 2), w_, h_, fc=C['marker'], ec='none',
                               zorder=6))
        if along_y:
            dx = 6 if mx < x1 / 2 else -6
            ax.text(mx + dx, my, m['name'], fontsize=8, color=C['marker'],
                    ha='left' if dx > 0 else 'right', va='center', zorder=7)
        else:
            ax.text(mx, my + (-6 if my > y1 / 2 else 6), m['name'], fontsize=8,
                    color=C['marker'], ha='center', va='center', zorder=7)

    # 구역 이름
    ax.text(260, 200, '03 공용 내부공간 (Nav2)', fontsize=15, color=C['switch'], ha='center',
            fontweight='bold')
    ax.text(66, 120, '01 물류창고 루프', fontsize=11, color=C['tape'], ha='center')
    ax.text(262, 11, '02\n인도장', fontsize=11, color='#8a5a00', ha='center', va='center',
            fontweight='bold')
    ax.text(362, 50, '스마트카트\n대기·충전', fontsize=10, color='#4b3fa0', ha='left',
            va='center')
    ax.text(412, 100, '입구', fontsize=11, color=C['guess'], ha='center', fontweight='bold')

    # 주요 치수(cm). A–Z = 실측 메모(2026-11-02 전체 도면) 기호 그대로, 한글 기호·기호 없는 치수는
    # 새 번호(①…)를 매기고 표의 비고에 메모 원래 기호를 적는다.
    n16, n01, h2 = (next(k for k in layout.nodes if k.id == i) for i in ('N16', 'N01', 'H2'))
    hw_ = layout.line_width / 2
    n17, h1, n12 = (next(k for k in layout.nodes if k.id == i) for i in ('N17', 'H1', 'N12'))
    sh, ol, orr, bk = (next(o for o in raw['objects'] if o['name'] == i)
                       for i in ('shelf_02', 'omx12_left', 'omx12_right', 'omx12_back'))
    n11 = next(k for k in layout.nodes if k.id == 'N11')
    bar11 = n11.y - hw_
    t_ = raw['defaults']['wall_thickness'] / 2   # 벽 면 = 중심 ± 0.25 cm
    dims = [
        # (시작, 끝, 기호(None = 새 번호), 값, 구간, 비고, 글자 위치 보정, 회전)
        # [n] = 2026-11-03 저녁 재측정 기록지 번호(Confluence 19136514)
        ((0, y1 + 18), (138, y1 + 18), 'G', '138', '왼쪽 벽 → 03 왼쪽 벽(게이트 벽)', '',
         (0, 0), 0),
        ((138, y1 + 18), (x1, y1 + 18), 'H', '306.5', '03 왼쪽 벽 → 오른쪽 벽', '메모 305 → 재측정',
         (0, 0), 0),
        ((x1 + 15, 0), (x1 + 15, y1), 'e', '221', '통로 벽 → 창가 벽', '', (0, 0), 90),
        ((150, 94), (150, y1), 'f', '127', '03 아래 벽 → 창가 벽',
         '⑤ 기준(e − 94). 메모 129·[19] 129.5와 다름', (0, 25), 90),
        ((138, 85), (379.5, 85), 'I', '241.5', '03 아래 벽 길이(03 왼쪽 벽 → 입구)',
         '메모 243. H − 입구', (-85, 0), 0),
        ((379.5, 85), (x1, 85), '입구', '65', '03 입구 폭', '메모 64.5 → 재측정', (0, 0), 0),
        ((0, 100), (20.5, 100), 'a', '20.5', '왼쪽 벽 → 루프 왼쪽 바깥선', '', (0, 5), 0),
        ((8, 0), (8, 72), 'b', '72', '통로 벽 → 간선 바깥선(통로 쪽)', '', (0, 0), 90),
        ((20.5, 145), (113.7, 145), 'D', '93.2', '루프 가로(바깥선 → 바깥선)', '', (0, 4), 0),
        ((124, 72), (124, 163.3), 'c', '91.3', '루프 세로(바깥선 → 바깥선)', '', (0, 0), 90),
        ((n16.x + 5, 163.3), (n16.x + 5, n16.y + hw_), 'R', '25',
         '루프 위 바깥선 → P5·P6 가로선 바깥선', '', (4, 0), 90),
        ((n01.x + 5, 72), (n01.x + 5, n01.y - hw_), 'Q', '29.9',
         '간선 바깥선(통로 쪽) → P4 가로선 바깥선', '', (4, 0), 90),
        ((h2.x + 5, 72), (h2.x + 5, h2.y - hw_), 'U', '40',
         '간선 바깥선(통로 쪽) → P1·P2 가로선 바깥선', '', (4, 0), 90),
        ((143, 0), (143, 54), 'P', '54', '02 왼쪽 칸막이 길이', '', (0, 0), 90),
        ((285, 0), (285, 49), 'T', '49', '02 오른쪽 칸막이 길이', '메모 49.5 → 재측정', (0, 0), 90),
        ((t_, 200), (n16.x - hw_, 200), '[24]', '44.5', '왼쪽 벽 면 → P5 세로선 왼쪽 가장자리',
         '메모 S 44.5(P5 ↔ P6)는 안 씀', (0, 4), 0),
        ((t_, 210), (n17.x - hw_, 210), '[25]', '91.5', '왼쪽 벽 면 → P6 세로선 왼쪽 가장자리', '',
         (18, 4), 0),
        ((280 + t_, -10), (h1.x - hw_, -10), '[21]', '25',
         '02 오른쪽 칸막이 면 → P1 세로선 왼쪽 가장자리', '', (0, 0), 0),
        ((h1.x - hw_, -10), (h2.x - hw_, -10), '[22]', '33', 'P1 ↔ P2 세로선 왼쪽 가장자리끼리',
         '메모 ㅈ 31.5', (0, 0), 0),
        ((h2.x - hw_, -10), (n12.x + hw_, -10), '[23]', '38.5',
         'P2 세로선 왼쪽 가장자리 → N12 가로선 오른쪽 가장자리', '메모 ㄱ 38.8, O 359와 2.75 다름',
         (0, 0), 0),
        ((138 + t_, 10), (sh['min'][0], 10), '[2]', '45.9', '02 왼쪽 칸막이 면 → 선반 왼쪽 끝', '',
         (0, 3), 0),
        ((sh['min'][0], 5), (sh['max'][0], 5), '[3]', '50', '선반 가로', '', (12, 0), 0),
        ((138 + t_, 40), (ol['min'][0], 40), '[4]', '35.5', '02 왼쪽 칸막이 면 → OMX1 받침 왼쪽 끝',
         '', (0, 3), 0),
        ((ol['max'][0], 46), (orr['min'][0], 46), '[6]', '25.5', 'OMX1 ↔ OMX2 받침 사이(오목 폭)',
         '', (7, 0), 0),
        ((178, t_), (178, ol['max'][1]), '[8]', '50.5', '통로 벽 면 → OMX1·2 받침 앞면', '',
         (0, 12), 90),
        ((201, t_), (201, bk['max'][1]), '[10]', '31.5', '통로 벽 면 → 오목 바닥(가운데 받침 앞면)',
         '', (0, -2), 90),
        ((215, t_), (215, bar11), '[15]', '35.7', '통로 벽 면 → N11 가로선 선반 쪽 가장자리',
         '[16] W 41(간선 → 가로선)과 5 cm 다름', (0, -2), 90),
        ((219, bk['max'][1]), (219, bar11), '[13]', '4', 'N11 가로선 ↔ 오목 바닥(V)', '',
         (8, 0), 0),
        ((20.5, 64), (64.1, 64), None, '43.6', '루프 왼쪽 바깥선 → P4 세로선 바깥선', '',
         (0, -4), 0),
        ((111.3, 64), (205.3, 64), None, '94', 'N07 세로선 바깥선 → P7 세로선 바깥선', '',
         (0, -4), 0),
        ((138, -22), (280, -22), None, '142', '02 인도장 가로(왼쪽 칸막이 → 오른쪽 칸막이)', '',
         (0, 0), 0),
        ((138.25, 125), (196, 125), None, '58', '03 왼쪽 벽 → 카메라 삼각대', '', (0, 4), 0),
        ((293, t_), (293, 94 - t_), None, '93.5', '통로 벽 ↔ 03 아래 벽(안쪽 면 사이)',
         '11-03 Ⓓ, 메모 ㅂ 94.5(두 벽 포함)', (0, -25), 90),
    ]
    drows = []
    num = 0
    for p0, p1, key, value, desc, note, off, rot in dims:
        if key is None:
            key = chr(0x2460 + num)
            num += 1
        if (key in DETAIL_02) == detail:
            _dim(ax, p0, p1, f'{key} {value}', offset=off, rot=rot)
        drows.append([key, value, desc, note + (' (02 확대도)' if key in DETAIL_02 else '')])
    if detail:
        ax.set_title(f'02 인도장 확대 {layout.version} (단위 cm, [n] = 2026-11-03 재측정 기록지 번호 · '
                     f'통로 벽 면 y 0.25, 02 왼쪽 칸막이 면 x 138.25)', fontsize=13, loc='left')
        fig.savefig(path, dpi=110, metadata={'Software': None})
        plt.close(fig)
        return

    ax.set_title(f'시연 공간 실측 도면 {layout.version} (단위 cm · 위 = 창가, 아래 = 통로 · '
                 f'노드 그래프 {layout.graph_version})', fontsize=15, loc='left', pad=12)
    ax.text(x0 - 20, y0 - 37,
            '원점(0,0) = 콘센트 벽·통로 쪽 모서리 · x = 길이, y = 창가 방향 · 노드 좌표 = 테이프 '
            '중심선(테이프 24 mm, 치수는 선 바깥 기준) · 가로선 12 cm · 진한 초록 = 실측, 주황 = 위치 '
            '추정·가정 · 파란 노드 = Nav2 전환 · 점선 회색 물체 = 라이다(12.5 cm)보다 낮음',
            fontsize=8.5, color='#666666', ha='left', va='bottom')

    # 마커 부착 제안 표
    ids = _marker_ids()
    mrows = []
    for m in markers:
        mx, my = m['center']
        where = '바닥' if m.get('floor') else '벽'
        size = m.get('size', (raw.get('markers') or {}).get('defaults', {}).get('size'))
        mrows.append([m['name'], str(ids.get(m['name'], '—')), where, f'{size * 10:g} mm',
                      f'{mx:.1f}', f'{my:.1f}'])
    max_ = fig.add_axes([0.70, 0.27, 0.27, 0.25])
    max_.axis('off')
    mtab = max_.table(cellText=mrows, colLabels=['마커', 'ID', '부착', '크기', 'x (cm)', 'y (cm)'],
                      colWidths=[0.15, 0.12, 0.13, 0.2, 0.2, 0.2], loc='upper center',
                      cellLoc='center')
    _style_table(mtab, [C['guess']] * len(mrows))
    max_.text(0.5, 0.42, 'ArUco 부착 제안 위치(추정)\n벽 마커: 종이 아래 끝을 바닥에 맞춤\n'
              '바닥 마커: 가로선 앞 빈 바닥 가운데',
              fontsize=9, color='#666666', ha='center', va='top', transform=max_.transAxes)

    # 노드 표
    tax = fig.add_axes([0.03, 0.27, 0.65, 0.25])
    tax.axis('off')
    rows = []
    for n in sorted(layout.nodes, key=lambda k: (k.id[0] != 'N', int(k.id[1:]))):
        note = 'Nav2 전환' if n.nav2_switch else ('위치 추정' if n.provisional else '')
        rows.append([n.id, n.place or '—', n.role, f'{n.x:.1f}', f'{n.y:.1f}',
                     ' · '.join(n.links), note])
    tab = tax.table(cellText=rows, colLabels=['노드', '운영 위치', '역할', 'x (cm)', 'y (cm)',
                                              '연결', '비고'],
                    colWidths=[0.07, 0.09, 0.3, 0.08, 0.08, 0.22, 0.1], loc='upper center',
                    cellLoc='center')
    colors = [C['switch'] if r[6] == 'Nav2 전환' else C['guess'] if r[6] == '위치 추정'
              else C['text'] for r in rows]
    _style_table(tab, colors)

    # 치수 표(두 단)
    half_rows = (len(drows) + 1) // 2
    for k, part in enumerate((drows[:half_rows], drows[half_rows:])):
        dax = fig.add_axes([0.03 + k * 0.48, 0.01, 0.46, 0.24])
        dax.axis('off')
        dtab = dax.table(cellText=part, colLabels=['기호', '값 (cm)', '구간', '비고'],
                         colWidths=[0.07, 0.1, 0.46, 0.37], loc='upper center', cellLoc='center')
        _style_table(dtab, [C['text']] * len(part))
        for (r, c), cell in dtab.get_celld().items():
            if r > 0 and c >= 2:
                cell.set_text_props(ha='left')
                cell.PAD = 0.02

    fig.savefig(path, dpi=110, metadata={'Software': None})
    plt.close(fig)


def _style_table(tab, row_colors):
    tab.auto_set_font_size(False)
    tab.set_fontsize(9)
    tab.scale(1, 1.22)
    for (r, _c), cell in tab.get_celld().items():
        cell.set_edgecolor('#bbbbbb')
        if r == 0:
            cell.set_facecolor('#e8ecef')
            cell.set_text_props(fontweight='bold')
        else:
            cell.set_text_props(color=row_colors[r - 1])


def _marker_ids():
    """docs/aruco/markers.yaml(ArUco 원본)에서 이름 → ID. 없으면 빈 표."""
    import yaml
    path = os.path.join(PKG_DIR, '..', '..', '..', '..', 'docs', 'aruco', 'markers.yaml')
    try:
        with open(path, encoding='utf-8') as f:
            return {m['name']: m['id'] for m in yaml.safe_load(f).get('markers') or []}
    except OSError:
        return {}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--layout', default=DEFAULT_LAYOUT)
    ap.add_argument('--output', default=None)
    args = ap.parse_args(argv)
    layout = Layout.load(args.layout)
    out = args.output or os.path.join(DEFAULT_MAP_DIR, f'map_measured_{layout.version}.png')
    draw(layout, out)
    print(f'wrote {out}')
    out02 = os.path.splitext(out)[0] + '_02.png'
    draw(layout, out02, detail=True)
    print(f'wrote {out02}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
