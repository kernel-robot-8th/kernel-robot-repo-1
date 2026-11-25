// 정밀 도킹 계산(브라우저·node 공용, DOM 없음). 원본: data/dock_tool/dock_zone.src.html 의 Core
// (호모그래피·predictedDock·plan·box·refPoints, 2026-11-07 사용자 도구). 추가: 배치 변환·①②③ → 자세·결과 오차.
// 단위 cm, 각도 rad(반시계 +), 지도 좌표 = 실측(roport_airport.yaml).
"use strict";

const DockCore = (() => {
  function solve(A, b) {               // 가우스 소거(부분 피벗). A n×n, b n
    const n = b.length, M = A.map((r, i) => [...r, b[i]]);
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      if (Math.abs(M[p][c]) < 1e-12) return null;
      [M[c], M[p]] = [M[p], M[c]];
      for (let r = 0; r < n; r++) if (r !== c) {
        const f = M[r][c] / M[c][c];
        for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
      }
    }
    return M.map((r, i) => r[n] / r[i]);
  }
  function normT(pts) {                // Hartley 정규화
    const cx = pts.reduce((s, p) => s + p[0], 0) / pts.length, cy = pts.reduce((s, p) => s + p[1], 0) / pts.length;
    const d = pts.reduce((s, p) => s + Math.hypot(p[0] - cx, p[1] - cy), 0) / pts.length || 1;
    const k = Math.SQRT2 / d;
    return [[k, 0, -k * cx], [0, k, -k * cy], [0, 0, 1]];
  }
  const mul = (A, B) => A.map((r) => B[0].map((_, j) => r.reduce((s, v, k) => s + v * B[k][j], 0)));
  function inv3(m) {
    const [a, b, c] = m[0], [d, e, f] = m[1], [g, h, i] = m[2];
    const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g, det = a * A + b * B + c * C;
    if (Math.abs(det) < 1e-15) return null;
    return [[A / det, -(b * i - c * h) / det, (b * f - c * e) / det],
      [B / det, (a * i - c * g) / det, -(a * f - c * d) / det],
      [C / det, -(a * h - b * g) / det, (a * e - b * d) / det]];
  }
  const apply = (H, p) => {
    const w = H[2][0] * p[0] + H[2][1] * p[1] + H[2][2];
    return [(H[0][0] * p[0] + H[0][1] * p[1] + H[0][2]) / w, (H[1][0] * p[0] + H[1][1] * p[1] + H[1][2]) / w];
  };
  // src(바닥 cm) → dst(이미지 px). 최소제곱 DLT(h33 = 1), 4점 이상
  function homography(src, dst) {
    if (src.length < 4) return null;
    const Ts = normT(src), Td = normT(dst);
    const s = src.map((p) => apply(Ts, p)), d = dst.map((p) => apply(Td, p));
    const AtA = Array.from({ length: 8 }, () => Array(8).fill(0)), Atb = Array(8).fill(0);
    const add = (row, v) => { for (let i = 0; i < 8; i++) { Atb[i] += row[i] * v; for (let j = 0; j < 8; j++) AtA[i][j] += row[i] * row[j]; } };
    s.forEach(([x, y], i) => {
      const [u, v] = d[i];
      add([x, y, 1, 0, 0, 0, -u * x, -u * y], u); add([0, 0, 0, x, y, 1, -v * x, -v * y], v);
    });
    const h = solve(AtA, Atb); if (!h) return null;
    const Hn = [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1]];
    const H = mul(mul(inv3(Td), Hn), Ts);
    return H.map((r) => r.map((v) => v / H[2][2]));
  }
  const DIRS = { E: 0, N: 90, W: 180, S: -90 };
  function heading(a, b) { return Math.atan2(b.y - a.y, b.x - a.x); }
  const wrapDeg = (d) => ((((d + 180) % 360) + 360) % 360) - 180;
  // 상차 노드의 지금 코드 도킹 자세: 가로선 가까운 가장자리에 IR 이 닿아 정지 → 축 기준 반시계 90°
  function predictedDock(L, nodeId, p) {
    const n = L.nodes[nodeId], prev = L.nodes[n.links[0]];
    const th = heading(prev, n), u = [Math.cos(th), Math.sin(th)];
    const back = p.halfLine + p.irAhead - p.overshoot;           // 노드 중심 → 축(접근 방향 뒤로)
    return { x: n.x - u[0] * back, y: n.y - u[1] * back, th: th + Math.PI / 2, approach: th, from: n.links[0] };
  }
  // 바퀴 축 자세 → 로봇 사각형 네 꼭짓점(바닥 cm). 로봇 앞 = +x
  function box(pose, p) {
    const c = Math.cos(pose.th), s = Math.sin(pose.th), f = p.frontAxle, r = f - p.L, w = p.W / 2;
    return [[f, w], [f, -w], [r, -w], [r, w]].map(([a, b]) => [pose.x + a * c - b * s, pose.y + a * s + b * c]);
  }
  // 예상 → 원하는 자세: 선 따라 a cm 더 가고(접근 방향 u), 회전 rot, 새 방향으로 b cm(+ 앞)
  // 0.1° 로 반올림한 ②가 0° 이면 회전 뒤 방향이 접근선과 같음 → ③ 없이 ①만(옆으로는 못 맞춤, side_cm).
  // 180° 쪽 평행(det ≈ 0)은 예전처럼 ①만 계산하지만 ② 가 범위 밖이라 적용 안 됨.
  function plan(pred, want) {
    const u = [Math.cos(pred.approach), Math.sin(pred.approach)], h = [Math.cos(want.th), Math.sin(want.th)];
    const dx = want.x - pred.x, dy = want.y - pred.y, det = u[0] * h[1] - u[1] * h[0];
    let rot = wrapDeg((want.th - pred.approach) * 180 / Math.PI);
    const zero = Math.round(rot * 10) / 10 === 0, parallel = zero || Math.abs(det) < 1e-6;
    let a, b;
    if (zero) rot = 0;
    if (parallel) { a = dx * u[0] + dy * u[1]; b = 0; }
    else { a = (dx * h[1] - dy * h[0]) / det; b = (u[0] * dy - u[1] * dx) / det; }
    const dth = wrapDeg((want.th - pred.th) * 180 / Math.PI);
    const lat = -dx * u[1] + dy * u[0];                                 // 접근선 왼쪽 + (cm)
    return { advance_cm: a, rotate_deg: rot, move_cm: b, dx, dy, dtheta_deg: dth, along_cm: dx * u[0] + dy * u[1], left_cm: lat,
      parallel, side_cm: parallel ? lat : 0 };
  }
  // 지금 코드 예상 자세 + ①②③ → 바퀴 축 자세(로봇에 적용된 값 표시용)
  function poseFromPlan(pred, pl) {
    const u = [Math.cos(pred.approach), Math.sin(pred.approach)], th = pred.approach + pl.rotate_deg * Math.PI / 180;
    return { x: pred.x + u[0] * pl.advance_cm + Math.cos(th) * pl.move_cm, y: pred.y + u[1] * pl.advance_cm + Math.sin(th) * pl.move_cm, th };
  }
  // 실제 − 목표 (접근선 기준): 선 따라(앞 +), 옆(왼쪽 +), 각(반시계 +)
  function resultError(target, measured, approach) {
    const u = [Math.cos(approach), Math.sin(approach)], dx = measured.x - target.x, dy = measured.y - target.y;
    return { along_cm: dx * u[0] + dy * u[1], left_cm: -dx * u[1] + dy * u[0], dtheta_deg: wrapDeg((measured.th - target.th) * 180 / Math.PI) };
  }
  // 계약 1절 허용 범위. ② 는 노드별(roport_line_nav dock.ROTATE_BANDS 와 같게, 양 끝 포함)
  const LIMITS = { advance_cm: 6, move_cm: 6, rotate_min: 60, rotate_max: 120 };
  const ROTATE_BANDS_DEFAULT = [[60, 120]];
  const ROTATE_BANDS = { N21: [[-10, 10], [60, 120]], N22: [[-10, 10], [60, 120]] };
  const rotateBands = (node) => ROTATE_BANDS[node] || ROTATE_BANDS_DEFAULT;
  const bandsText = (node) => rotateBands(node).map(([lo, hi]) => `${lo}~${hi}°`).join(" · ");
  // 로봇 상태의 dock 에 없는 노드가 쓰는 값(roport_line_nav dock.NODE_INITIAL 과 같게): N21·N22 는 0°·3 cm 후진
  const INITIAL_DEFAULT = { advance_cm: 0, rotate_deg: 90, move_cm: 0 };
  const NODE_INITIAL = { N21: { advance_cm: 0, rotate_deg: 0, move_cm: -3 }, N22: { advance_cm: 0, rotate_deg: 0, move_cm: -3 } };
  const initialPlan = (node) => NODE_INITIAL[node] || INITIAL_DEFAULT;
  const initialText = (node) => { const p = initialPlan(node); return `${p.advance_cm} cm · ${p.rotate_deg}° · ${p.move_cm} cm`; };
  function inLimits(pl, node) {
    if (![pl.advance_cm, pl.rotate_deg, pl.move_cm].every(Number.isFinite)) return false;
    const r = pl.rotate_deg;
    return Math.abs(pl.advance_cm) <= LIMITS.advance_cm && Math.abs(pl.move_cm) <= LIMITS.move_cm
      && rotateBands(node).some(([lo, hi]) => r >= lo && r <= hi);
  }
  // /api/route/graph(route_view.graph_view) → 이 계산이 쓰는 배치 L
  function layoutFromGraph(g) {
    const nodes = {};
    for (const n of g.nodes || []) nodes[n.id] = { x: n.x, y: n.y, bar: n.bar || "", links: n.links || [] };
    return { graph_version: g.version || "", line_width: g.line_width || 2.4, bar_length: g.bar_length || 12,
      nodes, walls: g.walls || [], objects: g.objects || [], dock_nodes: g.dock_nodes || [] };
  }
  function refPoints(L) {
    const pts = [], hb = L.bar_length / 2;
    for (const [k, n] of Object.entries(L.nodes)) {
      pts.push({ id: k, label: `${k} 테이프 중심`, x: n.x, y: n.y });
      if (n.bar === "x") { pts.push({ id: k + "_barW", label: `${k} 가로선 서쪽 끝`, x: n.x - hb, y: n.y }); pts.push({ id: k + "_barE", label: `${k} 가로선 동쪽 끝`, x: n.x + hb, y: n.y }); }
      if (n.bar === "y") { pts.push({ id: k + "_barS", label: `${k} 가로선 남쪽 끝`, x: n.x, y: n.y - hb }); pts.push({ id: k + "_barN", label: `${k} 가로선 북쪽 끝`, x: n.x, y: n.y + hb }); }
    }
    for (const o of L.objects) {
      const [x0, y0] = o.min, [x1, y1] = o.max;
      pts.push({ id: o.name + "_SW", label: `${o.name} 남서 모서리`, x: x0, y: y0 }, { id: o.name + "_SE", label: `${o.name} 남동 모서리`, x: x1, y: y0 },
        { id: o.name + "_NE", label: `${o.name} 북동 모서리`, x: x1, y: y1 }, { id: o.name + "_NW", label: `${o.name} 북서 모서리`, x: x0, y: y1 });
    }
    return pts;
  }
  return { solve, homography, inv3, apply, predictedDock, box, plan, poseFromPlan, resultError, inLimits, LIMITS,
    ROTATE_BANDS, rotateBands, bandsText, initialPlan, initialText,
    layoutFromGraph, refPoints, heading, wrapDeg, DIRS };
})();
if (typeof module !== "undefined") module.exports = DockCore;
