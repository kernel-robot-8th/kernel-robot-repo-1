// 정밀 도킹 탭(노트북 mode:=fleet). 화면 구성은 data/dock_tool/dock_console_mock.html 시안과 같음.
// 카메라: /api/dock/*(노트북이 SSH 로 카메라 PC ffmpeg 중계), 로봇 상태: /robot/<이름>/api/route(노트북 중계),
// 명령: /api/fleet/route(dock·start·load_done·stop), 복귀: /robot/<이름>/api/route/return.
"use strict";

(() => {
  const Core = DockCore;
  const $ = (id) => document.getElementById(id);
  const cv = $("cv"), ctx = cv.getContext("2d"), liveImg = $("liveImg");
  // 로봇 치수(시안·dock_zone 과 같음): footprint 12×12, 앞 끝↔축 3.8, IR↔축 3.0, 테이프 반폭 1.2, 정지 밀림 0.5
  const P = { L: 12, W: 12, frontAxle: 3.8, irAhead: 3.0, halfLine: 1.2, overshoot: 0.5 };
  const ROUTES = { N11: ["N07", "N23", "N19", "N11"], N01: ["N07", "N18", "N01"] };
  const DEFAULT_PLAN = { advance_cm: 0, rotate_deg: 90, move_cm: 0 };
  const LIVE_MAX_MS = 10 * 60 * 1000;
  const KEY = "pinkyDock.v1";
  const JSON_HDR = { "Content-Type": "application/json" };

  let L = null, cams = [], cam = null, H = null, Hinv = null, calibSource = "";
  let bg = null, bgKind = "none", liveTimer = null;     // bg = ImageBitmap | liveImg
  let view = { s: 1, ox: 0, oy: 0 }, drag = null;
  let robotState = null, robotOk = false, seenEv = new Set();
  let store = { want: {}, meas: {} };
  try { const s = JSON.parse(localStorage.getItem(KEY) || "null"); if (s && s.want) store.want = s.want; } catch (_) { /* 저장 안 돼도 됨 */ }
  const save = () => { try { localStorage.setItem(KEY, JSON.stringify({ want: store.want })); } catch (_) { /* 무시 */ } };

  const f1 = (v) => (v >= 0 ? "+" : "") + v.toFixed(1);
  const r1 = (v) => Math.round(v * 10) / 10;
  const deg = (rad) => rad * 180 / Math.PI;
  const msg = (id, text, kind) => { const e = $(id); e.textContent = text; e.className = "status-msg" + (kind ? " " + kind : ""); };

  async function getJson(url) {
    const res = await fetch(url, { cache: "no-store" });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.msg || `HTTP ${res.status}`);
    return data;
  }
  async function postJson(url, body) {
    const res = await fetch(url, { method: "POST", headers: JSON_HDR, body: JSON.stringify(body) });
    const data = await res.json().catch(() => ({}));
    if (!res.ok || data.success === false) throw new Error(data.msg || `HTTP ${res.status}`);
    return data;
  }

  // ---------------------------------------------------------------- 지도·도킹 자세
  const node = () => (cam ? cam.node : null);
  const pred = () => Core.predictedDock(L, node(), P);
  const wantPose = () => store.want[node()] || (({ x, y, th }) => ({ x, y, th }))(pred());
  const robotDock = () => ((robotState && robotState.dock) || {})[node()] || null;
  const appliedPose = () => { const d = robotDock(); return d ? Core.poseFromPlan(pred(), d) : null; };
  const estPose = () => {
    const p = robotState && robotState.pose_est;
    return p && Number.isFinite(p.x_cm) && Number.isFinite(p.y_cm) ? { x: p.x_cm, y: p.y_cm, th: (p.heading_deg || 0) * Math.PI / 180 } : null;
  };
  // 오차 기준 목표: 로봇에 적용된 값이 있으면 그 자세, 없으면 지금 코드(기본 0·90·0) 자세
  const targetPose = () => appliedPose() || Core.poseFromPlan(pred(), DEFAULT_PLAN);

  const w2i = (p) => Core.apply(H, p), i2w = (p) => Core.apply(Hinv, p);
  const toScr = (e) => { const r = cv.getBoundingClientRect(); return [(e.clientX - r.left) * cv.width / r.width, (e.clientY - r.top) * cv.height / r.height]; };
  const toImg = (e) => { const q = toScr(e); return [q[0] / view.s + view.ox, q[1] / view.s + view.oy]; };
  const lw = () => Math.max(1.5, cv.width / 640) / view.s;

  function poly(cs, stroke, fill, width, dash) {
    const c = cs.map(w2i);
    ctx.beginPath(); c.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath();
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.setLineDash(dash ? [lw() * 4, lw() * 3] : []); ctx.strokeStyle = stroke; ctx.lineWidth = width || lw(); ctx.stroke(); ctx.setLineDash([]); }
  }
  function drawMap() {
    const h = L.line_width / 2, hb = L.bar_length / 2, done = new Set();
    for (const [k, n] of Object.entries(L.nodes)) {
      for (const o of n.links) {
        const key = [k, o].sort().join("-"), m = L.nodes[o];
        if (done.has(key) || !m) continue;
        done.add(key);
        poly([[Math.min(n.x, m.x) - h, Math.min(n.y, m.y) - h], [Math.max(n.x, m.x) + h, Math.min(n.y, m.y) - h],
          [Math.max(n.x, m.x) + h, Math.max(n.y, m.y) + h], [Math.min(n.x, m.x) - h, Math.max(n.y, m.y) + h]], null, "rgba(59,130,246,.30)");
      }
      if (n.bar === "x") poly([[n.x - hb, n.y - h], [n.x + hb, n.y - h], [n.x + hb, n.y + h], [n.x - hb, n.y + h]], null, "rgba(59,130,246,.5)");
      if (n.bar === "y") poly([[n.x - h, n.y - hb], [n.x + h, n.y - hb], [n.x + h, n.y + hb], [n.x - h, n.y + hb]], null, "rgba(59,130,246,.5)");
    }
    for (const o of L.objects) { const [x0, y0] = o.min, [x1, y1] = o.max; poly([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], "rgba(59,130,246,.8)", null, lw() * 0.8); }
  }
  function robotBox(pose, color, fill, label, dash) {
    poly(Core.box(pose, P), color, fill, lw() * 1.4, dash);
    const ax = w2i([pose.x, pose.y]), fr = w2i([pose.x + Math.cos(pose.th) * P.frontAxle, pose.y + Math.sin(pose.th) * P.frontAxle]);
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = lw() * 1.4;
    ctx.beginPath(); ctx.moveTo(ax[0], ax[1]); ctx.lineTo(fr[0], fr[1]); ctx.stroke();
    ctx.beginPath(); ctx.arc(ax[0], ax[1], lw() * 2, 0, 7); ctx.fill();
    ctx.beginPath(); ctx.arc(fr[0], fr[1], lw() * 3.2, 0, 7); ctx.stroke();
    if (label) { ctx.font = `${Math.round(lw() * 8)}px system-ui`; ctx.fillText(label, fr[0] + lw() * 5, fr[1]); }
  }

  function draw() {
    const w = bg ? (bg.naturalWidth || bg.width) : 0, h = bg ? (bg.naturalHeight || bg.height) : 0;
    if (w && h && (cv.width !== w || cv.height !== h)) { cv.width = w; cv.height = h; view = { s: 1, ox: 0, oy: 0 }; }
    ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.fillStyle = "#000"; ctx.fillRect(0, 0, cv.width, cv.height);
    ctx.setTransform(view.s, 0, 0, view.s, -view.ox * view.s, -view.oy * view.s);
    if (bg && w) { try { ctx.drawImage(bg, 0, 0); } catch (_) { /* 아직 첫 장 전 */ } }
    if (!L || !cam || !H) return;
    if ($("showMap").checked) drawMap();
    const p = pred();
    robotBox({ x: p.x, y: p.y, th: p.th }, "#d97706", "rgba(217,119,6,.12)", "지금 코드");
    const ap = appliedPose();
    if (ap) robotBox(ap, "#38bdf8", null, "적용됨", true);
    robotBox(wantPose(), "#16a34a", "rgba(22,163,74,.18)", "목표");
    const m = store.meas[node()];
    if (m) robotBox(m, "#ef4444", "rgba(239,68,68,.15)", "실제");
    const est = estPose();
    if (est) robotBox(est, "#c084fc", "rgba(192,132,252,.25)", "Pinky(추정)");
  }

  function updatePanel() {
    if (!L || !cam) return;
    const p = pred(), pl = Core.plan(p, wantPose()), rounded = { advance_cm: r1(pl.advance_cm), rotate_deg: r1(pl.rotate_deg), move_cm: r1(pl.move_cm) };
    $("nodeTxt").textContent = cam.node;
    $("appTxt").textContent = `${p.from} → ${cam.node}`;
    $("pA").textContent = f1(pl.advance_cm) + " cm";
    $("pR").textContent = pl.rotate_deg.toFixed(1) + "°";
    $("pB").textContent = f1(pl.move_cm) + " cm";
    const ok = Core.inLimits(rounded) && !pl.parallel;
    $("pWarn").textContent = pl.parallel ? "목표 방향이 접근선과 평행이라 ③으로 옆 이동을 할 수 없음"
      : ok ? "" : "허용 범위를 넘어서 적용할 수 없음";
    $("pWarn").className = "tiny-note" + (ok ? "" : " dk-err");
    $("apply").disabled = !ok || !H || !robotName();
    const d = robotDock();
    $("appliedTxt").textContent = !robotOk ? "로봇 상태 모름" : d
      ? `① ${f1(d.advance_cm)} cm · ② ${Number(d.rotate_deg).toFixed(1)}° · ③ ${f1(d.move_cm)} cm`
      : (robotState && "dock" in robotState ? "없음(지금 코드 0 · 90° · 0)" : "로봇이 dock 값을 알리지 않음(line_nav 업데이트 필요?)");
    const lc = robotState && robotState.last_command;
    $("lastCmd").textContent = lc ? `${lc.cmd} ${lc.ok ? "ok" : "거부"} ${lc.detail || ""}` : "-";
    const m = store.meas[node()];
    if (m) {
      const e = Core.resultError(targetPose(), m, p.approach);
      $("rA").textContent = f1(e.along_cm) + " cm"; $("rL").textContent = f1(e.left_cm) + " cm"; $("rT").textContent = f1(e.dtheta_deg) + "°";
    } else { $("rA").textContent = "-"; $("rL").textContent = "-"; $("rT").textContent = "-"; }
  }
  function update() { draw(); updatePanel(); }

  // ---------------------------------------------------------------- 카메라
  function setChip(text, kind) { const c = $("camChip"); c.textContent = "● " + text; c.className = "chip" + (kind ? " " + kind : ""); }
  function stopLive() {
    if (liveTimer) { clearTimeout(liveTimer); liveTimer = null; }
    if (bgKind === "live") { bg = null; bgKind = "none"; }
    liveImg.removeAttribute("src");                // 연결을 닫으면 노트북이 ssh ffmpeg 를 정리
    $("live").textContent = "▶ 실시간";
  }
  async function cameraError() {
    try {
      const d = await getJson("/api/dock/cameras");
      const c = (d.cameras || []).find((x) => x.id === cam.id);
      return (c && c.status && c.status.error) || "실시간 연결 실패";
    } catch (e) { return e.message; }
  }
  function startLive() {
    stopLive();
    setChip("실시간 연결 중…");
    liveImg.onload = () => { bg = liveImg; bgKind = "live"; setChip("실시간", "batt-ok"); msg("camMsg", ""); update(); };
    liveImg.onerror = async () => {
      if (!liveImg.getAttribute("src")) return;
      stopLive();
      const m = await cameraError();
      setChip("실시간 실패", "batt-low"); msg("camMsg", m, "err");
    };
    liveImg.src = `/api/dock/camera/${encodeURIComponent(cam.id)}/stream?t=${Date.now()}`;
    $("live").textContent = "■ 실시간 끄기";
    liveTimer = setTimeout(() => { stopLive(); setChip("실시간 자동 종료(10분)"); }, LIVE_MAX_MS);
  }
  async function fetchPhoto(label) {
    setChip("사진 받는 중…");
    const res = await fetch(`/api/dock/camera/${encodeURIComponent(cam.id)}/snapshot.jpg?t=${Date.now()}`, { cache: "no-store" });
    if (!res.ok) { const d = await res.json().catch(() => ({})); throw new Error(d.msg || `HTTP ${res.status}`); }
    bg = await createImageBitmap(await res.blob());   // blob URL 없이(CSP img-src 'self')
    bgKind = "photo";
    setChip(`${label} ${new Date().toLocaleTimeString()}`);
  }
  async function photo(label) {
    try { await fetchPhoto(label); msg("camMsg", ""); } catch (e) { setChip("사진 실패", "batt-low"); msg("camMsg", e.message, "err"); }
    update();
  }
  async function freeze() {                         // 도착 사진: 실시간이면 지금 장을 고정
    if (bgKind === "live" && liveImg.naturalWidth) {
      try {
        const bm = await createImageBitmap(liveImg);
        stopLive(); bg = bm; bgKind = "photo"; setChip(`도착 사진 ${new Date().toLocaleTimeString()}`);
        update(); return;
      } catch (_) { /* 아래에서 사진 1장 */ }
    }
    stopLive();
    await photo("도착 사진");
  }
  async function loadCalib() {
    H = null; Hinv = null; calibSource = "";
    try {
      const c = await getJson(`/api/dock/calib/${encodeURIComponent(cam.id)}`);
      setCalib(c, c.file);
    } catch (e) { $("calTxt").textContent = `없음 — ${e.message}`; }
    update();
  }
  function setCalib(c, name) {
    const m = c.floor_to_img;
    if (!Array.isArray(m) || m.length !== 3 || !m.every((r) => Array.isArray(r) && r.length === 3 && r.every(Number.isFinite))) throw new Error("floor_to_img 3x3 필요");
    if (c.node && c.node !== cam.node) throw new Error(`보정 노드 ${c.node} ≠ 카메라 노드 ${cam.node}`);
    const inv = Core.inv3(m);
    if (!inv) throw new Error("역행렬 없음");
    H = m; Hinv = inv; calibSource = String(c.source || name || "");
    $("calTxt").textContent = `${name || "-"} (${calibSource})`;
  }
  function selectCam(i) {
    stopLive(); bg = null; bgKind = "none"; setChip("영상 없음"); msg("camMsg", "");
    cam = cams[i]; view = { s: 1, ox: 0, oy: 0 };
    const [w, h] = (cam.size || "1280x720").split("x").map(Number);
    cv.width = w || 1280; cv.height = h || 720;
    const g = document.querySelector(`input[name=goal][value=${cam.node}]`);
    if (g) { g.checked = true; showRoute(); }
    loadCalib();
  }

  // ---------------------------------------------------------------- 로봇
  const robotName = () => $("robot").value;
  async function refreshRobots() {
    try {
      const snap = await getJson("/api/fleet");
      const sel = $("robot"), keep = sel.value, names = (snap.robots || []).map((r) => r.name);
      if ([...sel.options].map((o) => o.value).join() !== names.join()) {
        sel.replaceChildren(...names.map((n) => { const o = document.createElement("option"); o.value = n; o.textContent = n; return o; }));
        let pick = keep;
        try { pick = keep || window.parent.document.getElementById("robotSel").value; } catch (_) { /* 단독 화면 */ }
        if (names.includes(pick)) sel.value = pick;
      }
    } catch (_) { /* 다음 주기 */ }
  }
  async function pollRobot() {
    const name = robotName();
    if (!name) { robotOk = false; robotState = null; $("stateChip").textContent = "로봇 없음"; return; }
    try {
      const d = await getJson(`/robot/${encodeURIComponent(name)}/api/route`);
      robotState = d.state || null;
      robotOk = !!robotState && !d.stale;
      const st = robotState ? robotState.state : "-";
      $("stateChip").textContent = d.stale ? `state ${st} (끊김)` : `state ${st}`;
      $("stateChip").className = "chip" + (d.stale ? " batt-stale" : "");
      $("loadDone").disabled = st !== "loading";
      for (const e of (robotState && robotState.events) || []) {
        const k = `${e.t}|${e.event}|${e.detail}`;
        if (seenEv.has(k)) continue;
        seenEv.add(k);
        addEvent(`${e.event} ${e.detail || ""}`);
      }
    } catch (e) {
      robotOk = false; robotState = null;
      $("stateChip").textContent = "콘솔 연결 안 됨"; $("stateChip").className = "chip batt-low";
    }
    update();
  }
  function addEvent(text) {
    const d = document.createElement("div");
    d.textContent = `${new Date().toLocaleTimeString()} ${text}`;
    $("ev").prepend(d);
    while ($("ev").childElementCount > 60) $("ev").lastChild.remove();
  }
  async function command(body, label) {
    const name = robotName();
    if (!name) { msg("runMsg", "로봇을 먼저 고르세요(Pinky 설정에서 등록)", "err"); return null; }
    try {
      const d = await postJson("/api/fleet/route", Object.assign({ name }, body));
      const r = d.result || {};
      if (r.success === false) throw new Error(r.msg || "로봇 콘솔이 거부");
      addEvent(`→ ${label}`);
      return d;
    } catch (e) { addEvent(`✕ ${label}: ${e.message}`); throw e; }
  }
  const goal = () => document.querySelector("input[name=goal]:checked").value;
  function showRoute() { $("routeTxt").textContent = ROUTES[goal()].join(" → ") + ` (상차 노드 ${goal()} 정밀 도킹)`; }

  // ---------------------------------------------------------------- 끌기·키
  const dragWhich = () => document.querySelector("input[name=drag]:checked").value;
  function measStart() { return store.meas[node()] || Object.assign({}, estPose() || appliedPose() || wantPose()); }
  cv.addEventListener("mousedown", (e) => {
    if (!H || !L) return;
    const w = i2w(toImg(e)), which = dragWhich(), cur = which === "want" ? wantPose() : measStart();
    const fr = [cur.x + Math.cos(cur.th) * P.frontAxle, cur.y + Math.sin(cur.th) * P.frontAxle];
    drag = { which, kind: Math.hypot(w[0] - fr[0], w[1] - fr[1]) < 2 ? "rot" : "move", ox: w[0] - cur.x, oy: w[1] - cur.y };
    if (which === "meas") store.meas[node()] = cur;
  });
  window.addEventListener("mousemove", (e) => {
    if (!drag) return;
    const w = i2w(toImg(e)), tgt = drag.which === "want" ? store.want : store.meas, cur = drag.which === "want" ? wantPose() : store.meas[node()];
    tgt[node()] = drag.kind === "move" ? { x: w[0] - drag.ox, y: w[1] - drag.oy, th: cur.th } : { x: cur.x, y: cur.y, th: Math.atan2(w[1] - cur.y, w[0] - cur.x) };
    update();
  });
  window.addEventListener("mouseup", () => { if (drag) { drag = null; save(); } });
  async function stopAll() {
    try { await postJson("/api/fleet/stop_all", {}); msg("runMsg", "전체 정지 보냄", "ok"); } catch (e) { msg("runMsg", `전체 정지 실패: ${e.message}`, "err"); }
  }
  window.addEventListener("keydown", (e) => {
    if (["INPUT", "SELECT", "TEXTAREA"].includes(e.target.tagName) && e.target.type !== "radio" && e.target.type !== "checkbox") return;
    if (e.code === "Space" || e.code === "Escape") { e.preventDefault(); stopAll(); return; }
    if (!H || !L) return;
    const which = dragWhich(), tgt = which === "want" ? store.want : store.meas;
    const cur = Object.assign({}, which === "want" ? wantPose() : measStart()), d = e.shiftKey ? 0.1 : 0.5, r = (e.shiftKey ? 0.2 : 1) * Math.PI / 180;
    const k = { ArrowLeft: [-d, 0], ArrowRight: [d, 0], ArrowUp: [0, d], ArrowDown: [0, -d] }[e.key];
    if (k) { cur.x += k[0]; cur.y += k[1]; } else if (e.key === "[" || e.key === "{") cur.th += r; else if (e.key === "]" || e.key === "}") cur.th -= r; else return;
    e.preventDefault(); tgt[node()] = cur; if (which === "want") save(); update();
  });
  cv.addEventListener("wheel", (e) => {
    e.preventDefault();
    const q = toScr(e), b = toImg(e);
    view.s = Math.min(20, Math.max(1, view.s * (e.deltaY < 0 ? 1.2 : 1 / 1.2)));
    view.ox = b[0] - q[0] / view.s; view.oy = b[1] - q[1] / view.s;
    if (view.s <= 1) view = { s: 1, ox: 0, oy: 0 };
    update();
  }, { passive: false });

  // ---------------------------------------------------------------- 버튼
  $("cam").addEventListener("change", (e) => selectCam(+e.target.value));
  $("robot").addEventListener("change", () => { seenEv = new Set(); $("ev").replaceChildren(); pollRobot(); });
  $("live").addEventListener("click", () => (liveImg.getAttribute("src") ? stopLive() : startLive()));
  $("shot").addEventListener("click", () => { stopLive(); photo("사진"); });
  $("snap").addEventListener("click", freeze);
  $("showMap").addEventListener("change", update);
  $("zoomNode").addEventListener("click", () => {
    if (!H || !L) return;
    const n = L.nodes[cam.node], c = w2i([n.x, n.y]), a = w2i([n.x - 20, n.y]), b = w2i([n.x + 20, n.y]);
    view.s = Math.min(20, Math.max(1, cv.width * 0.5 / Math.max(1, Math.hypot(b[0] - a[0], b[1] - a[1]))));
    view.ox = Math.max(0, c[0] - cv.width / view.s / 2); view.oy = Math.max(0, c[1] - cv.height / view.s / 2);
    update();
  });
  $("zoomAll").addEventListener("click", () => { view = { s: 1, ox: 0, oy: 0 }; update(); });
  $("resetWant").addEventListener("click", () => { delete store.want[node()]; save(); update(); });
  $("calibFile").addEventListener("change", (e) => {
    const f = e.target.files[0];
    if (!f) return;
    const rd = new FileReader();
    rd.onload = () => {
      try { setCalib(JSON.parse(rd.result), f.name); msg("camMsg", ""); } catch (err) { msg("camMsg", `보정 JSON 오류: ${err.message}`, "err"); }
      update();
    };
    rd.readAsText(f);
  });
  $("apply").addEventListener("click", async () => {
    const pl = Core.plan(pred(), wantPose());
    const body = { cmd: "dock", node: cam.node, advance_cm: r1(pl.advance_cm), rotate_deg: r1(pl.rotate_deg), move_cm: r1(pl.move_cm) };
    if (!Core.inLimits(body)) { msg("applyMsg", "허용 범위 밖", "err"); return; }
    try {
      await command(body, `dock ${cam.node} ① ${f1(body.advance_cm)} ② ${body.rotate_deg.toFixed(1)}° ③ ${f1(body.move_cm)}`);
      msg("applyMsg", `${robotName()} 에 ${cam.node} 도킹 값을 보냄 — 「마지막 명령 결과」·파란 점선으로 적용 확인`, "ok");
    } catch (e) { msg("applyMsg", e.message, "err"); }
  });
  $("resetRobot").addEventListener("click", async () => {
    if (!confirm(`${robotName()} 의 ${cam.node} 도킹 값을 지우고 기본(0 · 90° · 0)으로 돌립니다.`)) return;
    try { await command({ cmd: "dock", node: cam.node, reset: true }, `dock ${cam.node} reset`); msg("applyMsg", "지우기 보냄", "ok"); } catch (e) { msg("applyMsg", e.message, "err"); }
  });
  document.querySelectorAll("input[name=goal]").forEach((r) => r.addEventListener("change", () => {
    showRoute();
    const i = cams.findIndex((c) => c.node === goal());
    if (i >= 0 && cams[i] !== cam) { $("cam").value = String(i); selectCam(i); }
  }));
  $("start").addEventListener("click", async () => {
    const route = ROUTES[goal()];
    if (!confirm(`${robotName()}: ${route.join(" → ")}\n로봇이 N07 에서 남쪽(OMX 쪽)을 보고 IR 3개가 동서 선 위에 있는지, 주변에 사람·물건이 없는지 확인했나요?`)) return;
    try {
      await command({ cmd: "start", route, start_cross: true, start_facing: "S" }, `start ${route.join("→")} (교차선·남쪽)`);
      delete store.meas[goal()];
      msg("runMsg", "시작 보냄", "ok");
    } catch (e) { msg("runMsg", e.message, "err"); }
  });
  $("loadDone").addEventListener("click", async () => { try { await command({ cmd: "load_done" }, "load_done"); msg("runMsg", "상차 완료 보냄", "ok"); } catch (e) { msg("runMsg", e.message, "err"); } });
  $("stop").addEventListener("click", async () => { try { await command({ cmd: "stop" }, "stop"); msg("runMsg", "정지 보냄", "ok"); } catch (e) { msg("runMsg", e.message, "err"); } });
  $("home").addEventListener("click", async () => {
    const name = robotName();
    if (!name || !confirm(`${name} 을(를) P5 로 복귀시킵니다(상차 대기면 상차 완료 뒤 출발).`)) return;
    try { const d = await postJson(`/robot/${encodeURIComponent(name)}/api/route/return`, {}); msg("runMsg", d.msg || "복귀 보냄", "ok"); addEvent(`→ 복귀 ${(d.route || []).join("→")}`); } catch (e) { msg("runMsg", `복귀 실패: ${e.message}`, "err"); }
  });
  $("btnEstop").addEventListener("click", stopAll);
  const poseOut = (p) => ({ x_cm: r1(p.x), y_cm: r1(p.y), heading_deg: r1(deg(p.th)) });
  $("save").addEventListener("click", async () => {
    const m = store.meas[node()];
    if (!m) { msg("saveMsg", "「실제 위치 표시」로 빨간 박스를 먼저 맞추세요", "err"); return; }
    const p = pred(), tg = targetPose(), e = Core.resultError(tg, m, p.approach), d = robotDock() || DEFAULT_PLAN;
    const body = { node: cam.node, camera: cam.id, route: ROUTES[cam.node] || [],
      applied: { advance_cm: Number(d.advance_cm), rotate_deg: Number(d.rotate_deg), move_cm: Number(d.move_cm) },
      target: poseOut(tg), predicted: poseOut({ x: p.x, y: p.y, th: p.th }), measured: poseOut(m),
      error: { along_cm: r1(e.along_cm), left_cm: r1(e.left_cm), dtheta_deg: r1(e.dtheta_deg) },
      note: $("note").value.slice(0, 200), calib_source: calibSource.slice(0, 300) };
    if (robotName()) body.robot = robotName();
    if (robotState && typeof robotState.state === "string") body.state = robotState.state.slice(0, 32);
    try { const r = await postJson("/api/dock/result", body); msg("saveMsg", `기록함: ${r.file}`, "ok"); loadResults(); } catch (err) { msg("saveMsg", err.message, "err"); }
  });
  async function loadResults() {
    try {
      const d = await getJson("/api/dock/results");
      $("resDir").textContent = d.error || d.dir;
      const tb = $("hist").querySelector("tbody");
      tb.replaceChildren(...(d.results || []).slice(0, 15).map((r) => {
        const tr = document.createElement("tr"), e = r.error || {};
        for (const v of [(r.saved_at || "").slice(5, 16).replace("T", " "), r.node,
          Number.isFinite(e.along_cm) ? f1(e.along_cm) : "-", Number.isFinite(e.left_cm) ? f1(e.left_cm) : "-",
          Number.isFinite(e.dtheta_deg) ? f1(e.dtheta_deg) + "°" : "-"]) {
          const td = document.createElement("td"); td.textContent = v; tr.appendChild(td);
        }
        return tr;
      }));
    } catch (e) { $("resDir").textContent = e.message; }
  }

  // ---------------------------------------------------------------- 시작
  async function init() {
    showRoute();
    try { L = Core.layoutFromGraph(await getJson("/api/route/graph")); $("verTxt").textContent = L.graph_version; } catch (e) { msg("camMsg", `지도 없음: ${e.message}`, "err"); }
    await refreshRobots();
    try {                       // 통합 화면 위쪽 로봇 선택을 따라감(같은 출처)
      const ps = window.parent !== window && window.parent.document.getElementById("robotSel");
      if (ps) ps.addEventListener("change", () => { if ([...$("robot").options].some((o) => o.value === ps.value)) { $("robot").value = ps.value; $("robot").dispatchEvent(new Event("change")); } });
    } catch (_) { /* 단독 화면 */ }
    try {
      const d = await getJson("/api/dock/cameras");
      cams = (d.cameras || []).filter((c) => !L || L.nodes[c.node]);
      if (d.config_error) msg("camMsg", d.config_error, "err");
      $("cam").replaceChildren(...cams.map((c, i) => { const o = document.createElement("option"); o.value = String(i); o.textContent = `${c.label} (${c.host} ${c.device}) → ${c.node}`; return o; }));
      if (cams.length) selectCam(0);
    } catch (e) { msg("camMsg", `카메라 목록 실패: ${e.message}`, "err"); }
    loadResults();
    setInterval(refreshRobots, 3000);
    setInterval(pollRobot, 400);
    setInterval(() => { if (bgKind === "live") draw(); }, 150);
    update();
  }
  window.addEventListener("pagehide", stopLive);
  init();
})();
