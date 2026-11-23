"""「로봇이 아는 내 위치」(읽기 전용): 지도 읽기와 비용 지도 칸 → map 좌표 변환."""
import math
from types import SimpleNamespace

from roport_follow.localization_view import grid_cells, load_map


def test_load_map_png_and_meta(tmp_path):
    from PIL import Image
    Image.new('L', (4, 3), 205).save(tmp_path / 'm.pgm')
    (tmp_path / 'm.yaml').write_text('image: m.pgm\nresolution: 0.05\norigin: [-1.0, -2.0, 0]\nnegate: 0\n'
                                     'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
    meta, png = load_map(tmp_path / 'm.yaml')
    assert meta == {'name': 'm.yaml', 'resolution': 0.05, 'origin': [-1.0, -2.0], 'width': 4, 'height': 3}
    assert png[:4] == b'\x89PNG'


def grid(data, w, h, res=0.1, ox=0.0, oy=0.0):
    return SimpleNamespace(info=SimpleNamespace(width=w, height=h, resolution=res,
                                                origin=SimpleNamespace(position=SimpleNamespace(x=ox, y=oy))), data=data)


def test_grid_cells_transforms_and_filters():
    g = grid([0, 99, 100, 0,
              0, 0, 0, 100], 4, 2)
    cells = grid_cells(g, (1.0, 2.0, 0.0), (1.0, 2.0), radius_m=1.0)
    assert sorted(cells) == [(1.15, 2.05, 99), (1.25, 2.05, 100), (1.35, 2.15, 100)]
    rot = grid_cells(g, (0.0, 0.0, math.pi / 2), (0.0, 0.0), radius_m=1.0)
    assert (-0.05, 0.25, 100) in rot                     # (0.25, 0.05) 를 90° 돌림
    assert grid_cells(g, (5.0, 5.0, 0.0), (0.0, 0.0), radius_m=1.0) == []
