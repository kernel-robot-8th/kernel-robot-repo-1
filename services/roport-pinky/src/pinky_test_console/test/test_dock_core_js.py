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
  out[n].inLimits = C.inLimits(pl, n);
}
out.limits = [{ advance_cm: 6.1, rotate_deg: 90, move_cm: 0 },
  { advance_cm: 0, rotate_deg: 121, move_cm: 0 },
  { advance_cm: NaN, rotate_deg: 90, move_cm: 0 }].map((pl) => C.inLimits(pl, 'N11'));
// ② 노드별 범위: N21·N22 만 −10~10 · 60~120 (양 끝 포함), 180° 쪽은 거부
const ROTS = [-10.1, -10, 0, 10, 10.1, 59.9, 60, 120, 120.1, 169.9, 170, 180, -180, 190, 190.1];
out.bands = {};
for (const n of ['N01', 'N11', 'N21', 'N22']) out.bands[n] = ROTS.map((r) => C.inLimits({ advance_cm: 0, rotate_deg: r, move_cm: 0 }, n));
// 평행(②가 0°): ③ 없이 ①만, 옆 오차는 side_cm 로 알림. 180° 쪽은 범위 밖이라 적용 불가
out.par = {};
for (const n of ['N21', 'N22']) {
  const pr = C.predictedDock(L, n, P);
  const u = [Math.cos(pr.approach), Math.sin(pr.approach)], left = [-u[1], u[0]];
  const at = (a, s, dth) => ({ x: pr.x + u[0] * a + left[0] * s, y: pr.y + u[1] * a + left[1] * s, th: pr.approach + dth });
  out.par[n] = {
    zero: C.plan(pr, at(2.5, 0, 0)), half: C.plan(pr, at(-3, 0, Math.PI)), side: C.plan(pr, at(1, 0.8, 0)),
    near: C.plan(pr, at(1, 0, Math.PI * 185 / 180)),
    back0: C.plan(pr, C.poseFromPlan(pr, { advance_cm: 1.5, rotate_deg: 0, move_cm: -2 })),
  };
  for (const k of ['half', 'near']) { const q = out.par[n][k]; out.par[n][k + 'Ok'] = C.inLimits({ advance_cm: q.advance_cm, rotate_deg: Math.round(q.rotate_deg * 10) / 10, move_cm: q.move_cm }, n); }
}
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
    # ② 노드별 범위(dock.py ROTATE_BANDS 와 같게)
    rots = [-10.1, -10, 0, 10, 10.1, 59.9, 60, 120, 120.1, 169.9, 170, 180, -180, 190, 190.1]
    only_60_120 = [60 <= r <= 120 for r in rots]
    wide = [(-10 <= r <= 10) or (60 <= r <= 120) for r in rots]
    assert out['bands']['N01'] == only_60_120 and out['bands']['N11'] == only_60_120
    assert out['bands']['N21'] == wide and out['bands']['N22'] == wide
    for n in ('N21', 'N22'):
        par = out['par'][n]
        z, h = par['zero'], par['half']
        assert z['parallel'] and z['rotate_deg'] == 0 and abs(z['advance_cm'] - 2.5) < 1e-9
        assert z['move_cm'] == 0 and abs(z['side_cm']) < 1e-9
        assert abs(abs(h['rotate_deg']) - 180) < 1e-9 and not par['halfOk']   # 180° 쪽은 적용 불가
        assert not par['nearOk']
        s = par['side']                                             # 옆 0.8 cm 는 ①로 못 맞춤
        assert s['parallel'] and abs(s['advance_cm'] - 1) < 1e-9 and abs(s['side_cm'] - 0.8) < 1e-9
        b0 = par['back0']                                           # ① 1.5 + ③ −2(같은 방향) = ① −0.5
        assert b0['parallel'] and abs(b0['advance_cm'] + 0.5) < 1e-9 and b0['move_cm'] == 0


def test_dock_routes_match_map():
    """dock.js ROUTES(시험 주행) = N07 출발 → 상차 노드, 이웃끼리 지도 links 로 이어짐. dock.html 목적지와 같은 노드."""
    import re
    src = open(os.path.join(HERE, '..', 'web', 'dock.js'), encoding='utf-8').read()
    m = re.search(r'const ROUTES = \{(.*?)\};', src, re.S)
    assert m, 'ROUTES not found in dock.js'
    routes = {k: re.findall(r'"([A-Z0-9]+)"', v)
              for k, v in re.findall(r'(N\d\d): \[([^\]]*)\]', m.group(1))}
    assert routes == {
        'N11': ['N07', 'N23', 'N19', 'N11'], 'N01': ['N07', 'N18', 'N01'],
        'N21': ['N07', 'N23', 'N21'], 'N22': ['N07', 'N23', 'N19', 'N24', 'N22']}
    g = graph_view(LAYOUT)
    links = {n['id']: set(n['links']) for n in g['nodes']}
    for goal, route in routes.items():
        assert route[0] == 'N07' and route[-1] == goal and goal in g['dock_nodes']
        for a, b in zip(route, route[1:]):
            assert b in links[a] and a in links[b], (goal, a, b)
    html = open(os.path.join(HERE, '..', 'web', 'dock.html'), encoding='utf-8').read()
    assert set(re.findall(r'name="goal" value="(N\d\d)"', html)) == set(routes)
