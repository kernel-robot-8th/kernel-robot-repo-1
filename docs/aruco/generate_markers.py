#!/usr/bin/env python3
"""markers.yaml 에서 ArUco 마커 이미지(PNG)와 A4 인쇄 시트(PDF)를 만든다.

    python3 docs/aruco/generate_markers.py          # 생성
    python3 docs/aruco/generate_markers.py --check  # 생성 후 PNG를 다시 인식해 ID 확인

의존성: OpenCV(cv2.aruco), PyYAML. PDF는 외부 라이브러리 없이 직접 쓴다(벡터, 실제 크기).
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
PAGE_W, PAGE_H = 210.0, 297.0          # A4, mm
MARGIN_X, TOP, BOTTOM = 15.0, 32.0, 12.0
GAP, LABEL_H = 6.0, 8.0
CELLS = 6                               # 4x4 비트 + 테두리 1칸씩
PNG_CELL_PX = 50
PT = 72.0 / 25.4                        # mm -> pt


def get_dictionary(name):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def marker_cells(dictionary, marker_id):
    """CELLS x CELLS bool 배열(True = 검은 칸)."""
    generate = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    return generate(dictionary, marker_id, CELLS) < 128


def write_png(path, cells):
    padded = np.pad(cells, 1, constant_values=False)    # 흰 여백 1칸
    img = np.where(padded, 0, 255).astype(np.uint8)
    img = np.kron(img, np.ones((PNG_CELL_PX, PNG_CELL_PX), np.uint8))
    cv2.imwrite(str(path), img)


def detect_ids(img):
    dictionary = get_dictionary(CONFIG['dictionary'])
    if hasattr(cv2.aruco, 'ArucoDetector'):
        _, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(img)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(img, dictionary)
    return [] if ids is None else sorted(int(i) for i in ids.flatten())


class Pdf:
    """사각형과 Helvetica 글자만 쓰는 최소 PDF 작성기. 좌표는 mm, 원점은 왼쪽 위."""

    def __init__(self):
        self.pages = []

    def new_page(self):
        self.pages.append([])

    def _y(self, y):
        return (PAGE_H - y) * PT

    def fill_rect(self, x, y, w, h, gray=0.0):
        self.pages[-1].append(
            f'{gray} g {x * PT:.3f} {self._y(y + h):.3f} {w * PT:.3f} {h * PT:.3f} re f')

    def text(self, x, y, s, size=9):
        s = s.replace('\\', '\\\\').replace('(', '\\(').replace(')', '\\)')
        self.pages[-1].append(
            f'BT /F1 {size} Tf {x * PT:.3f} {self._y(y):.3f} Td ({s}) Tj ET')

    def save(self, path):
        objs = ['<< /Type /Catalog /Pages 2 0 R >>', None,
                '<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>']
        kids = []
        for ops in self.pages:
            stream = '\n'.join(ops).encode('ascii')
            objs.append(b'<< /Length %d >>\nstream\n' % len(stream) + stream + b'\nendstream')
            kids.append(len(objs) + 1)
            objs.append(f'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {PAGE_W * PT:.3f} '
                        f'{PAGE_H * PT:.3f}] /Resources << /Font << /F1 3 0 R >> >> '
                        f'/Contents {len(objs)} 0 R >>')
        objs[1] = f'<< /Type /Pages /Kids [{" ".join(f"{k} 0 R" for k in kids)}] /Count {len(kids)} >>'
        out = bytearray(b'%PDF-1.4\n')
        offsets = []
        for i, obj in enumerate(objs, 1):
            offsets.append(len(out))
            body = obj if isinstance(obj, bytes) else obj.encode('ascii')
            out += b'%d 0 obj\n' % i + body + b'\nendobj\n'
        xref = len(out)
        out += b'xref\n0 %d\n0000000000 65535 f \n' % (len(objs) + 1)
        out += b''.join(b'%010d 00000 n \n' % o for o in offsets)
        out += b'trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n' % (len(objs) + 1, xref)
        Path(path).write_bytes(bytes(out))


def draw_marker(pdf, x, y, cells, size, quiet, label):
    """(x, y) = 자르는 선 왼쪽 위. 검은 칸은 행마다 이어서 그린다.

    자르는 선은 모서리 표시만 둔다. 닫힌 사각형은 검출기가 마커 테두리로 오인한다.
    """
    cell = size / CELLS
    full = size + 2 * quiet
    arm, t = min(4.0, quiet), 0.25
    for cx, sx in ((x, 1), (x + full, -1)):
        for cy, sy in ((y, 1), (y + full, -1)):
            pdf.fill_rect(min(cx, cx + sx * arm), cy - t / 2, arm, t, gray=0.6)
            pdf.fill_rect(cx - t / 2, min(cy, cy + sy * arm), t, arm, gray=0.6)
    for r in range(CELLS):
        c = 0
        while c < CELLS:
            if not cells[r, c]:
                c += 1
                continue
            start = c
            while c < CELLS and cells[r, c]:
                c += 1
            # 행 사이 틈이 인쇄되지 않도록 아래로 0.02 mm 겹친다
            pdf.fill_rect(x + quiet + start * cell, y + quiet + r * cell,
                          (c - start) * cell, cell + (0.02 if r < CELLS - 1 else 0))
    if full >= 50:
        pdf.text(x, y + full + 4.5, '  '.join(label), size=8)
    else:   # 작은 마커는 두 줄로 짧게
        pdf.text(x, y + full + 3.2, f'{label[0]}  {label[1]}', size=6)
        pdf.text(x, y + full + 6.0, label[2], size=6)


def page_header(pdf, title):
    pdf.new_page()
    pdf.text(MARGIN_X, 12, title, size=10)
    # 실제 크기 확인용 100 mm 자
    pdf.fill_rect(MARGIN_X, 18, 100, 0.6)
    for i in range(11):
        pdf.fill_rect(MARGIN_X + i * 10 - 0.15, 18, 0.3, 4 if i % 5 == 0 else 2.5)
    pdf.text(MARGIN_X + 103, 20.5, '100 mm - print at 100% (actual size)', size=8)


def layout(pdf, title, items):
    """items: (cells, size, quiet, (이름, ID, 크기)). 줄 단위로 채우고 넘치면 새 페이지."""
    page_header(pdf, title)
    x, y, row_h = MARGIN_X, TOP, 0.0
    for cells, size, quiet, label in items:
        w = size + 2 * quiet
        h = w + LABEL_H
        if x + w > PAGE_W - MARGIN_X:
            x, y, row_h = MARGIN_X, y + row_h + GAP, 0.0
        if y + h > PAGE_H - BOTTOM:
            page_header(pdf, title + ' (cont.)')
            x, y, row_h = MARGIN_X, TOP, 0.0
        draw_marker(pdf, x, y, cells, size, quiet, label)
        x += w + GAP
        row_h = max(row_h, h)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--check', action='store_true', help='PNG를 다시 인식해 ID 확인')
    args = parser.parse_args()

    dict_name = CONFIG['dictionary']
    dictionary = get_dictionary(dict_name)
    version = CONFIG['version']
    img_dir, print_dir = HERE / 'images', HERE / 'print'
    img_dir.mkdir(exist_ok=True)
    print_dir.mkdir(exist_ok=True)

    ids = [m['id'] for m in CONFIG['markers']]
    if len(ids) != len(set(ids)):
        raise SystemExit('markers.yaml: 중복 ID가 있다')

    groups = {}
    for m in CONFIG['markers']:
        cells = marker_cells(dictionary, m['id'])
        png = img_dir / f"id{m['id']:02d}_{m['name']}.png"
        write_png(png, cells)
        label = (m['name'], f"ID {m['id']}", f"{m['size_mm']} mm  {dict_name}")
        groups.setdefault(m['group'], []).extend(
            [(cells, m['size_mm'], m['quiet_mm'], label)] * m.get('copies', 1))

    pdf = Pdf()
    for group, items in groups.items():
        layout(pdf, f'RO:PORT ArUco v{version} - {group}', items)
    pdf.save(print_dir / f'aruco_v{version}_markers.pdf')

    test = CONFIG['range_test']
    test_cells = marker_cells(dictionary, test['id'])
    write_png(img_dir / f"id{test['id']:02d}_TEST.png", test_cells)
    pdf = Pdf()
    layout(pdf, f'RO:PORT ArUco range test - ID {test["id"]} (not for operation)',
           [(test_cells, s, min(10.0, max(4.0, s / CELLS)), ('TEST', f'ID {test["id"]}', f'{s} mm  {dict_name}'))
            for s in test['sizes_mm']])
    pdf.save(print_dir / 'aruco_range_test.pdf')

    if args.check:
        for png in sorted(img_dir.glob('id*.png')):
            want = int(png.name[2:4])
            got = detect_ids(cv2.imread(str(png), cv2.IMREAD_GRAYSCALE))
            if got != [want]:
                raise SystemExit(f'{png.name}: 기대 ID {want}, 인식 {got}')
        print(f'check OK: {len(list(img_dir.glob("id*.png")))} PNG')
    print(f'생성: {img_dir.relative_to(HERE.parent.parent)}, {print_dir.relative_to(HERE.parent.parent)}')


CONFIG = yaml.safe_load((HERE / 'markers.yaml').read_text(encoding='utf-8'))

if __name__ == '__main__':
    main()
