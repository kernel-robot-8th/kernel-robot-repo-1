"""배치 데이터 → Nav2 map_server 지도(pgm·yaml)와 같은 픽셀의 png 생성.

패키지 폴더에서 실행한다:
  python3 -m roport_gz_world.generate_map            # ../../maps/airport_map.{pgm,yaml,png} 갱신
  python3 -m roport_gz_world.generate_map --check    # 커밋된 지도와 배치 데이터가 같은지 검사

SLAM 으로 만든 지도가 아니라 실측 배치를 그린 지도다. 해상도·원점·크기는 SLAM 지도와 같게 둔다.
- 점유(0): 벽, 라이다 높이(map.lidar_height)보다 높은 물체
- 빈 곳(254): 벽 안쪽
- 미지(205): 벽 바깥
테이프 라인·마커·라이다보다 낮은 물체는 라이다에 안 보이므로 그리지 않는다.
"""

import argparse
import os
import struct
import sys
import zlib

from .layout import Layout, LayoutError

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_LAYOUT = os.path.join(PKG_DIR, 'layout', 'roport_airport.yaml')
DEFAULT_MAP_DIR = os.path.normpath(os.path.join(PKG_DIR, '..', '..', 'maps'))
NAME = 'airport_map'

OCCUPIED, UNKNOWN, FREE = 0, 205, 254


def _rects(layout):
    """지도에 그릴 사각형(실측 cm): [(x0, y0, x1, y1), ...]."""
    t = layout.wall_thickness / 2.0
    out = []
    for w in layout.raw.get('walls') or []:
        (ax, ay), (bx, by) = w['from'], w['to']
        if ax != bx and ay != by:
            raise LayoutError(f"{w.get('name')}: map generation supports axis-aligned walls only")
        out.append((min(ax, bx) - t, min(ay, by) - t, max(ax, bx) + t, max(ay, by) + t))
    for o in layout.raw.get('objects') or []:
        if float(o.get('height', layout.object_height)) > layout.map.lidar_height:
            out.append((o['min'][0], o['min'][1], o['max'][0], o['max'][1]))
    return out


def build_pixels(layout):
    """행 0 = 지도 위쪽(+y 끝). 반환: bytes (width * height)."""
    m = layout.map
    if m is None:
        raise LayoutError('layout has no map section')
    half = m.resolution * 100.0 / 2.0   # 칸 반 변(cm)
    rects = _rects(layout)
    rx0, ry0, rx1, ry1 = layout.room()
    px = bytearray()
    for r in range(m.height):
        for c in range(m.width):
            wx = m.origin[0] + (c + 0.5) * m.resolution
            wy = m.origin[1] + (m.height - 1 - r + 0.5) * m.resolution
            x, y = layout.frame.to_measured(wx, wy)
            # 칸(정사각형)과 사각형이 겹치면 점유
            if any(x0 - half < x < x1 + half and y0 - half < y < y1 + half
                   for x0, y0, x1, y1 in rects):
                px.append(OCCUPIED)
            elif rx0 <= x <= rx1 and ry0 <= y <= ry1:
                px.append(FREE)
            else:
                px.append(UNKNOWN)
    return bytes(px)


def pgm_bytes(layout, pixels):
    m = layout.map
    header = (f'P5\n# CREATOR: roport_gz_world.generate_map (layout {layout.version}) '
              f'{m.resolution:.3f} m/pix\n{m.width} {m.height}\n255\n')
    return header.encode('ascii') + pixels


def yaml_text(layout):
    m = layout.map
    return (f'# 실측 배치 {layout.version}(roport_gz_world/layout/roport_airport.yaml)에서 생성한 지도.\n'
            '# SLAM 지도가 아님. 직접 고치지 말고 python3 -m roport_gz_world.generate_map 으로 만든다.\n'
            f'image: {NAME}.pgm\n'
            'mode: trinary\n'
            f'resolution: {m.resolution:.3f}\n'
            f'origin: [{m.origin[0]:.3f}, {m.origin[1]:.3f}, 0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.196\n')


def _chunk(kind, data):
    body = kind + data
    return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body) & 0xffffffff)


def png_bytes(width, height, pixels):
    """8비트 회색 PNG (외부 라이브러리 없이)."""
    raw = b''.join(b'\x00' + pixels[r * width:(r + 1) * width] for r in range(height))
    return (b'\x89PNG\r\n\x1a\n'
            + _chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
            + _chunk(b'IDAT', zlib.compress(raw, 9))
            + _chunk(b'IEND', b''))


def png_pixels(data):
    """png_bytes 로 쓴 파일의 픽셀을 다시 읽는다(필터 0만 지원)."""
    pos, idat, width, height = 8, b'', 0, 0
    while pos < len(data):
        (length,) = struct.unpack('>I', data[pos:pos + 4])
        kind, body = data[pos + 4:pos + 8], data[pos + 8:pos + 8 + length]
        if kind == b'IHDR':
            width, height = struct.unpack('>II', body[:8])
        elif kind == b'IDAT':
            idat += body
        pos += 12 + length
    raw = zlib.decompress(idat)
    rows = [raw[r * (width + 1):(r + 1) * (width + 1)] for r in range(height)]
    if any(row[0] != 0 for row in rows):
        raise ValueError('unsupported PNG filter')
    return b''.join(row[1:] for row in rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--layout', default=DEFAULT_LAYOUT)
    ap.add_argument('--output-dir', default=DEFAULT_MAP_DIR)
    ap.add_argument('--check', action='store_true',
                    help='do not write; exit 1 if the map files are out of date')
    args = ap.parse_args(argv)

    layout = Layout.load(args.layout)
    pixels = build_pixels(layout)
    files = {
        f'{NAME}.pgm': pgm_bytes(layout, pixels),
        f'{NAME}.yaml': yaml_text(layout).encode('utf-8'),
        f'{NAME}.png': png_bytes(layout.map.width, layout.map.height, pixels),
    }
    if args.check:
        stale = []
        for name, data in files.items():
            path = os.path.join(args.output_dir, name)
            try:
                current = open(path, 'rb').read()
            except OSError:
                stale.append(name)
                continue
            # png 는 zlib 버전에 따라 압축 바이트가 다를 수 있어 픽셀로 비교
            same = (png_pixels(current) == pixels) if name.endswith('.png') else current == data
            if not same:
                stale.append(name)
        if stale:
            print(f'out of date: {", ".join(stale)}; run python3 -m roport_gz_world.generate_map',
                  file=sys.stderr)
            return 1
        print(f'{args.output_dir}/{NAME}.* are up to date')
        return 0
    for name, data in files.items():
        with open(os.path.join(args.output_dir, name), 'wb') as f:
            f.write(data)
        print(f'wrote {os.path.join(args.output_dir, name)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
