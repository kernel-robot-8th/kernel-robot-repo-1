"""web/dock_core.js 계산(node 로 실행, 없으면 건너뜀): 기본 = 지금 코드(0·90°·0), ①②③ ↔ 자세 왕복."""

import json
import os
import shutil
import subprocess

import pytest

from pinky_test_console.route_view import graph_view

HERE = os.path.dirname(os.path.abspath(__file__))
CORE = os.path.join(HERE, '..', 'web', 'dock_core.js')
LAYOUT = os.path.join(HERE, '..', '..', 'roport_gz_world', 'layout', 'roport_airport.yaml')
CALIB_N11 = [[90.7195837124143, -4.404143506091232, -13054.959929962743],
             [6.361353159085935, -67.34587539101129, 3741.9071349180795],
             [0.03140811910317259, -0.001305337232637611, 1.0]]

SCRIPT = r"""
const C = require(process.argv[1]);
const g = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const L = C.layoutFromGraph(g.graph), H = g.H;
const P = { L: 12, W: 12, frontAxle: 3.8, irAhead: 3.0, halfLine: 1.2, overshoot: 0.5 };
const out = {};
for (const n of ['N11', 'N01', 'N21', 'N22']) {
  const pr = C.predictedDock(L, n, P);
  out[n] = { pred: pr, same: C.plan(pr, { x: pr.x, y: pr.y, th: pr.th }) };
  const pl = { advance_cm: 1.5, rotate_deg: 96, move_cm: -2 };
  out[n].back = C.plan(pr, C.poseFromPlan(pr, pl));
  out[n].inLimits = C.inLimits(pl);
}
out.limits = [{ advance_cm: 6.1, rotate_deg: 90, move_cm: 0 },
  { advance_cm: 0, rotate_deg: 121, move_cm: 0 },
  { advance_cm: NaN, rotate_deg: 90, move_cm: 0 }].map(C.inLimits);
const Hi = C.inv3(H), q = C.apply(Hi, C.apply(H, [209.4, 37.15]));
out.roundtrip = q;
const src = [[200, 30], [220, 30], [220, 50], [200, 50], [210, 40]];
const dst = src.map((p) => C.apply(H, p));
out.fit = C.apply(C.homography(src, dst), [205, 35]);
out.fitWant = C.apply(H, [205, 35]);
out.err = C.resultError({ x: 0, y: 0, th: 0 }, { x: 1, y: 2, th: 0.1 }, -Math.PI / 2);
out.dock_nodes = L.dock_nodes;
console.log(JSON.stringify(out));
"""


@pytest.mark.skipif(shutil.which('node') is None, reason='node not installed')
def test_dock_core():
    data = json.dumps({'graph': graph_view(LAYOUT), 'H': CALIB_N11})
    res = subprocess.run(['node', '-e', SCRIPT, os.path.abspath(CORE)], input=data,
                         capture_output=True, text=True, timeout=30, check=True)
    out = json.loads(res.stdout)
    for n in ('N11', 'N01', 'N21', 'N22'):
        same, back = out[n]['same'], out[n]['back']
        assert abs(same['advance_cm']) < 1e-9 and abs(same['move_cm']) < 1e-9
        assert abs(same['rotate_deg'] - 90) < 1e-9          # 기본 = 지금 코드(0 · 90° · 0)
        assert abs(back['advance_cm'] - 1.5) < 1e-9 and abs(back['rotate_deg'] - 96) < 1e-9
        assert abs(back['move_cm'] + 2) < 1e-9 and out[n]['inLimits']
    # N11: 남쪽(N19 → N11)으로 접근, 축 = 노드 중심에서 북쪽 3.7 cm(1.2 + 3.0 − 0.5), 동쪽을 봄
    p = out['N11']['pred']
    assert abs(p['x'] - 209.4) < 1e-9 and abs(p['y'] - (37.15 + 3.7)) < 1e-9
    assert abs(p['approach'] + 1.5707963) < 1e-6 and abs(p['th']) < 1e-9
    assert out['limits'] == [False, False, False]
    assert abs(out['roundtrip'][0] - 209.4) < 1e-6 and abs(out['roundtrip'][1] - 37.15) < 1e-6
    for i in (0, 1):
        assert abs(out['fit'][i] - out['fitWant'][i]) < 1e-3
    # 접근 남쪽(-90°): 앞 = -y, 왼쪽 = +x
    e = out['err']
    assert abs(e['along_cm'] + 2) < 1e-9 and abs(e['left_cm'] - 1) < 1e-9
    assert abs(e['dtheta_deg'] - 5.729578) < 1e-5
    assert set(out['dock_nodes']) == {'N01', 'N11', 'N21', 'N22'}
