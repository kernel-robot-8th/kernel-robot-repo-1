// pinky_test_console 테스트 관제(노트북). /api/fleet 를 1 s 마다 읽어 지도·표를 그리고,
// 목적지 클릭 → /api/route/plan 으로 최단 경로 → /api/fleet/route 로 선택한 로봇에 명령. 로그는 /api/fleet/logs.
"use strict";

const $ = (id) => document.getElementById(id);
const POLL_MS = 1000;
const LOG_KEEP = 1500;
const BUSY = ["follow", "pass", "advance", "rotate", "loading", "reverse", "paused"];
let selected = null;
let robots = [];
let graph = null;
const pick = {};        // 로봇별 {start, goal, route}
let logs = [];
let logLast = 0;

function colorOf(name) {
  const i = robots.findIndex((r) => r.name === name);
  return RM.ROBOT_COLORS[(i < 0 ? 0 : i) % RM.ROBOT_COLORS.length];
}

function consoleMode(r) {
  return r && r.status && r.status.ok && r.status.route ? (r.status.route.mode || "-") : "-";
}

function navState(r) {
  return r && r.status && r.status.ok && r.status.route ? r.status.route.state : null;
}

function currentNode(st) {
  const e = RM.endMemory(st);   // 끝 기억 노드(state.end 우선, 없으면 arrived·loading·handoff 의 route 마지막)
  return e ? e.node : null;
}

const map = new RM.RouteMap($("routeCanvas"), () => {}, { mode: "goal", onGoal: (id) => setGoal(id) });

// ---------------- 출발·목적지·경로 ----------------
function selPick() {
  if (!selected) return null;
  if (!pick[selected]) pick[selected] = { start: null, goal: null, route: [] };
  return pick[selected];
}

let lastEndKey = null;
function fillStarts() {
  const sel = $("startSel");
  sel.replaceChildren();
  const r = robots.find((x) => x.name === selected);
  if (!r || !graph) return;
  const p = selPick();
  const cur = currentNode(navState(r));
  const opts = [];
  if (cur) opts.push([cur, `현재 위치 (${cur})`]);
  for (const s of graph.starts || []) opts.push([s.node, `${s.place} (${s.node})`]);
  for (const [v, label] of opts) {
    const o = document.createElement("option");
    o.value = v;
    o.textContent = label;
    sel.appendChild(o);
  }
  // 사용자가 직접 고르지 않았으면 기본값은 「현재」(끝 기억 노드)
  if (cur && !p.userStart) p.start = cur;
  if (!p.start || !opts.some(([v]) => v === p.start)) {
    p.start = cur || (r.role === "smartcart" ? "H1" : "N16");
  }
  sel.value = p.start;
}

async function plan() {
  const p = selPick();
  if (!p) return;
  p.route = [];
  if (p.start && p.goal) {
    const via = p.via ? `&via=${encodeURIComponent(p.via)}` : "";
    const r = await RM.getJson(`/api/route/plan?from=${encodeURIComponent(p.start)}&to=${encodeURIComponent(p.goal)}${via}`);
    if (r.httpStatus === 200) p.route = r.route;
    else RM.setMsg($("cmdMsg"), `경로 없음: ${r.msg || r.httpStatus}`, "err");
  }
  showPick();
}

function showPick() {
  const p = selPick();
  $("goalText").textContent = p && p.goal ? p.goal : "지도에서 정차 노드를 누름";
  $("viaSel").value = p && p.via ? p.via : "";
  $("routeText").textContent = p && p.route.length ? p.route.join(" → ") : "-";
  map.setStartGoal(p ? p.start : null, p ? p.goal : null);
  map.setRoute(p ? p.route : []);
  updateButtons();
}

function setGoal(id) {
  if (!selected) { RM.setMsg($("cmdMsg"), "먼저 오른쪽 표에서 로봇을 선택", "err"); return; }
  selPick().goal = id;
  RM.setMsg($("cmdMsg"), "", null);
  plan();
}

$("startSel").addEventListener("change", (ev) => { const p = selPick(); p.start = ev.target.value; p.userStart = true; plan(); });
$("viaSel").addEventListener("change", (ev) => { selPick().via = ev.target.value; plan(); });

// 속도: 슬라이더를 움직이면 표시만 바꾸고 「속도 적용」 때 보냄. 그 전에는 로봇 값을 따라감.
let speedDirty = false;
function showSpeedLabels() {
  $("spLinV").textContent = `${Number($("spLin").value).toFixed(1)} cm/s`;
  $("spRotV").textContent = `${Number($("spRot").value).toFixed(2)} rad/s`;
}
function syncSpeed(sp) {
  if (!sp) { $("spNow").textContent = "현재 -"; return; }
  $("spLin").min = String(sp.min_speed * 100);
  $("spLin").max = String(sp.max_linear * 100);
  $("spRot").min = String(sp.rotate_min_speed);
  $("spRot").max = String(sp.max_angular);
  $("spNow").textContent = `현재 ${(sp.base_speed * 100).toFixed(1)} cm/s · ${sp.rotate_speed.toFixed(2)} rad/s`;
  if (!speedDirty) {
    $("spLin").value = String(sp.base_speed * 100);
    $("spRot").value = String(sp.rotate_speed);
  }
  showSpeedLabels();
}
for (const id of ["spLin", "spRot"]) {
  $(id).addEventListener("input", () => { speedDirty = true; showSpeedLabels(); });
}
function speedBody() {
  return { cmd: "speed", base_speed: Number($("spLin").value) / 100, rotate_speed: Number($("spRot").value) };
}
$("btnSpeed").addEventListener("click", () => { speedDirty = false; command(speedBody()); });

// 후진 속도(0.01~0.03 m/s): 직선·회전 속도와 따로 적용 (FFT1-172)
let revDirty = false;
function showRevLabel() { $("spRevV").textContent = `${Number($("spRev").value).toFixed(1)} cm/s`; }
$("spRev").addEventListener("input", () => { revDirty = true; showRevLabel(); });
$("btnRevSpeed").addEventListener("click", () => {
  revDirty = false;
  command({ cmd: "speed", reverse_speed: Number($("spRev").value) / 100 });
});
function syncReverseSpeed(sp) {
  const v = sp && typeof sp.reverse_follow_speed === "number" ? sp.reverse_follow_speed : null;
  $("spRevNow").textContent = v === null ? "후진 기본 2.0 cm/s (로봇 값 보고 없음)" : `현재 후진 ${(v * 100).toFixed(1)} cm/s`;
  if (v !== null && !revDirty) $("spRev").value = String(v * 100);
  showRevLabel();
}
showRevLabel();
$("btnPause").addEventListener("click", () => command({ cmd: "pause" }));
$("btnResume").addEventListener("click", () => command({ cmd: "resume" }));
$("btnBack").addEventListener("click", () => {
  const to = $("backTo").value;
  if (to) command({ cmd: "back", to });
});

function updateButtons() {
  const r = robots.find((x) => x.name === selected);
  const st = navState(r);
  const busy = st && BUSY.includes(st.state);
  const p = selPick();
  $("btnStart").disabled = !r || !p || p.route.length < 2 || !!busy || !(r.status && r.status.ok);
  $("btnLoad").disabled = !r || !st || st.state !== "loading";
  $("btnStop").disabled = !r;
  $("btnSpeed").disabled = !r || !st;
  $("btnRevSpeed").disabled = !r || !st;
  $("btnPause").disabled = !r || !st || !["follow", "pass", "advance", "rotate"].includes(st.state);
  $("btnResume").disabled = !r || !st || st.state !== "paused";
  $("btnBack").disabled = !r || !st || !$("backTo").value;   // 후보가 없으면 value 가 비어 꺼짐
  $("btnHome").disabled = !r || !r.home || !st || !["arrived", "handoff"].includes(st.state);
}

async function command(body) {
  if (!selected) return;
  const r = await RM.postJson("/api/fleet/route", Object.assign({ name: selected }, body));
  const ok = r.success && r.result && r.result.success;
  RM.setMsg($("cmdMsg"), ok ? `${selected}: ${body.cmd} 보냄` : `실패: ${r.msg || (r.result && r.result.msg) || r.httpStatus}`,
    ok ? "ok" : "err");
  pollLogs();
  return ok;
}

$("btnStart").addEventListener("click", async () => {
  const p = selPick();
  // 끝 기억 노드와 다른 출발이면 확인 → 먼저 stop(기억 지움) 뒤 start. 취소하면 출발 안 함.
  const warn = RM.startMismatchMsg(navState(robots.find((x) => x.name === selected)), p.route[0]);
  if (warn) {
    if (!confirm(warn)) return;
    if (!(await command({ cmd: "stop" }))) return;
  }
  command({ cmd: "start", route: p.route }).then((ok) => {
    if (ok) { p.goal = null; p.route = []; p.start = null; p.userStart = false; showPick(); }
  });
});
$("btnLoad").addEventListener("click", () => command({ cmd: "load_done" }));
$("btnStop").addEventListener("click", () => command({ cmd: "stop" }));

function homeMsg(result) {
  return Object.entries(result || {}).map(([n, x]) => `${n}: ${x.msg}`).join(" / ");
}

async function returnHome(name) {
  const body = name ? { name } : {};
  const r = await RM.postJson("/api/fleet/return", body);
  const res = r.result || {};
  const ok = r.success && Object.values(res).every((x) => x.ok);
  const text = r.success ? homeMsg(res) : `복귀 실패: ${r.msg || r.httpStatus}`;
  RM.setMsg(name ? $("cmdMsg") : $("estopStatus"), text, ok ? "ok" : "err");
  pollLogs();
}
$("btnHome").addEventListener("click", () => returnHome(selected));
$("btnHomeAll").addEventListener("click", () => {
  if (confirm("노드에서 끝난 모든 로봇을 각자 복귀 칸으로 보냅니다. 로봇끼리 충돌 회피는 없으니 경로가 겹치지 않는지 확인하세요. 진행할까요?")) {
    returnHome(null);
  }
});

async function stopAll() {
  const r = await RM.postJson("/api/fleet/stop_all", {});
  RM.setMsg($("estopStatus"), r.success ? "전체 정지 보냄" : "전체 정지 실패", r.success ? "ok" : "err");
  pollLogs();
}
$("btnEstop").addEventListener("click", stopAll);
document.addEventListener("keydown", (ev) => {
  if ((ev.code === "Space" || ev.code === "Escape") && !["INPUT", "SELECT"].includes(ev.target.tagName)) {
    ev.preventDefault();
    stopAll();
  }
});

// ---------------- 표·지도 ----------------
function cell(tr, text) {
  const td = document.createElement("td");
  td.textContent = text;
  tr.appendChild(td);
  return td;
}

function render(snap) {
  robots = snap.robots || [];
  $("logFile").textContent = snap.log_error || snap.log_file || "-";
  const tb = $("robots").querySelector("tbody");
  tb.replaceChildren();
  const legend = $("legend");
  legend.replaceChildren();
  for (const r of robots) {
    const st = navState(r);
    const tr = document.createElement("tr");
    tr.classList.add("fl-row");
    if (r.name === selected) tr.classList.add("mk-seen");
    const sw = cell(tr, "■");
    sw.classList.add("fl-swatch");
    sw.dataset.color = String(robots.indexOf(r) % RM.ROBOT_COLORS.length);
    cell(tr, r.name);
    cell(tr, r.home || "-");
    cell(tr, r.status ? (r.status.ok ? "OK" : "꺼짐") : "-");
    const b = r.status && r.status.ok && r.status.route ? r.status.route.battery : null;
    const bc = cell(tr, b && b.percent !== null && b.percent !== undefined
      ? `${Math.round(b.percent)}%${b.stale ? " (오래됨)" : ""}` : "-");
    if (b && b.percent !== null && b.percent !== undefined) {
      bc.classList.add(b.stale ? "batt-stale" : b.percent < 20 ? "batt-low" : b.percent < 40 ? "batt-mid" : "batt-ok");
    }
    cell(tr, consoleMode(r));
    cell(tr, st ? st.state : "-");
    cell(tr, st && st.leg ? `${st.leg.from}→${st.leg.to}` : "-");
    tr.addEventListener("click", () => {
      selected = r.name;
      speedDirty = false;
      revDirty = false;
      $("selName").textContent = r.name;
      fillStarts();
      plan();
      render(snap);
    });
    tb.appendChild(tr);
    const li = document.createElement("span");
    li.className = "fl-key";
    li.dataset.color = sw.dataset.color;
    li.textContent = `● ${r.name}`;
    legend.appendChild(li);
  }
  if (!selected && robots.length) {
    selected = robots[0].name;
    $("selName").textContent = selected;
    fillStarts();
    showPick();
  }
  map.setRobots(robots.map((r) => {
    const st = navState(r);
    return { name: r.name, color: colorOf(r.name), pose: st ? st.pose_est : null, traversed: RM.traversed(st) };
  }));
  const r = robots.find((x) => x.name === selected);
  const st = navState(r);
  // 끝 기억 노드가 바뀌면 출발 목록·기본값(「현재」)을 새로 채움(직접 고른 출발은 유지)
  const endKey = `${selected}|${currentNode(st) || ""}`;
  if (endKey !== lastEndKey) {
    lastEndKey = endKey;
    const before = selPick() ? selPick().start : null;
    fillStarts();
    if (selPick() && selPick().start !== before) plan();
  }
  $("sState").textContent = st ? st.state : "-";
  syncSpeed(st && st.speed);
  syncReverseSpeed(st && st.speed);
  RM.fillBackSelect($("backTo"), RM.backCandidates(st));
  { const rt = RM.reverseText(st);
  $("sReverse").textContent = rt === "-" && st && st.back_why ? `후진 불가: ${st.back_why}` : rt; }
  $("sReason").textContent = st && st.reason ? st.reason : "-";
  $("sLeg").textContent = st && st.leg ? `${st.leg.from} → ${st.leg.to} (${st.leg.action})` : "-";
  $("sTravel").textContent = st ? `${st.traveled_cm}` + (st.window_cm ? ` / ${st.window_cm[0]}~${st.window_cm[1]}` : "") : "-";
  $("sOn").textContent = st && st.ir ? st.ir.on_line.map((x) => (x ? "■" : "□")).join(" ") : "-";
  $("sOdom").textContent = st ? (st.odom_ok ? "OK" : "없음") : "-";
  $("sLast").textContent = st && st.last_command ? `${st.last_command.cmd}: ${st.last_command.ok ? "ok" : "거부"} (${st.last_command.detail})` : "-";
  $("sAge").textContent = r && r.status ? `${r.status.age_sec} s${r.status.ok ? "" : " — " + (r.status.error || "")}` : "-";
  $("consoleMode").textContent = r ? (r.status && r.status.ok ? `mode:=${consoleMode(r)}` : "꺼짐 또는 연결 안 됨") : "-";
  $("localDir").textContent = snap.log_file ? snap.log_file.replace(/\/[^/]*$/, "/") + (selected || "<로봇>") + "/" : "-";
  for (const id of ["btnConsoleTest", "btnConsoleIr", "btnConsoleStop", "btnFiles"]) $(id).disabled = !r || !!r.busy;
  // 로봇 목록 거르기 선택지
  const lr = $("logRobot");
  const have = new Set([...lr.options].map((o) => o.value));
  for (const x of robots) {
    if (!have.has(x.name)) {
      const o = document.createElement("option");
      o.value = x.name;
      o.textContent = x.name;
      lr.appendChild(o);
    }
  }
  updateButtons();
}

// ---------------- 로그 ----------------
function renderLogs() {
  const who = $("logRobot").value;
  const src = $("logSource").value;
  $("btnLogCsv").href = "/api/fleet/logs.csv" + (who ? `?robot=${encodeURIComponent(who)}` : "");
  const tb = $("logs").querySelector("tbody");
  tb.replaceChildren();
  for (const e of logs) {
    if (who && e.robot !== who) continue;
    if (src && e.source !== src) continue;
    const tr = document.createElement("tr");
    tr.className = "fl-log-" + e.source;
    cell(tr, e.time.slice(11, 23));
    cell(tr, e.robot);
    cell(tr, e.source);
    cell(tr, e.event);
    cell(tr, e.detail);
    tb.appendChild(tr);
  }
  if ($("logFollow").checked) $("logWrap").scrollTop = $("logWrap").scrollHeight;
}

async function pollLogs() {
  const r = await RM.getJson(`/api/fleet/logs?after=${logLast}`);
  if (r.httpStatus !== 200) return;
  if (r.last < logLast) logs = [];           // 서버가 다시 시작됨
  logLast = r.last;
  if (r.entries.length) {
    logs = logs.concat(r.entries).slice(-LOG_KEEP);
    renderLogs();
  }
}
$("logRobot").addEventListener("change", renderLogs);
$("logSource").addEventListener("change", renderLogs);

// ---------------- 로봇 콘솔(SSH) · 로그 파일 ----------------
async function job(body, confirmText) {
  if (!selected) return null;
  if (confirmText && !confirm(confirmText)) return null;
  const r = await RM.postJson("/api/fleet/job", Object.assign({ name: selected }, body));
  RM.setMsg($("cmdMsg"), r.success ? `${selected}: ${body.action} 시작(로그에서 결과 확인)` : `실패: ${r.msg || r.httpStatus}`,
    r.success ? "ok" : "err");
  pollLogs();
  return r;
}
$("btnConsoleTest").addEventListener("click", () => job({ action: "console_start", mode: "test" },
  `${selected}에서 테스트 콘솔을 mode:=test(경로 주행 + IR)로 켭니다. 이미 켜져 있으면 먼저 끄세요. bringup은 따로 실행해야 합니다.`));
$("btnConsoleIr").addEventListener("click", () => job({ action: "console_start", mode: "ir" },
  `${selected}에서 테스트 콘솔을 mode:=ir(IR 측정)로 켭니다. 이미 켜져 있으면 먼저 끄세요.`));
$("btnConsoleStop").addEventListener("click", () => job({ action: "console_stop" },
  `${selected}의 테스트 콘솔을 끕니다(경로 주행 중이면 정지). bringup은 그대로 둡니다.`));

function fileRow(tb, label, size, onFetch) {
  const tr = document.createElement("tr");
  cell(tr, label);
  cell(tr, size === null || size === undefined ? "폴더" : `${Math.round(size / 1024)} KB`);
  const td = cell(tr, "");
  const b = document.createElement("button");
  b.type = "button";
  b.className = "btn btn-outline btn-small";
  b.textContent = "가져오기";
  b.addEventListener("click", onFetch);
  td.appendChild(b);
  tb.appendChild(tr);
}

async function loadFiles() {
  if (!selected) return;
  const r = await RM.getJson(`/api/fleet/files?name=${encodeURIComponent(selected)}`);
  const tb = $("files").querySelector("tbody");
  tb.replaceChildren();
  if (!r.success) { RM.setMsg($("cmdMsg"), `목록 실패: ${r.msg || r.httpStatus}`, "err"); return; }
  const robot = robots.find((x) => x.name === selected);
  const res = r.result || {};
  for (const f of res.route_logs || []) {
    fileRow(tb, f.name, f.size, () => {
      $("scpCmd").value = `scp -i ~/.ssh/pinky_ed25519 pinky@${robot.ip}:roport_logs/${f.name} ${$("localDir").textContent}`;
      job({ action: "fetch_log", file: f.name });
    });
  }
  for (const d of res.ros_logs || []) {
    fileRow(tb, `ROS 로그 ${d.name}`, null, () => {
      $("scpCmd").value = `ssh -i ~/.ssh/pinky_ed25519 pinky@${robot.ip} 'tar czf - -C ~/.ros/log ${d.name}' > ${$("localDir").textContent}roslog_${d.name}.tar.gz`;
      job({ action: "fetch_roslog", dir: d.name });
    });
  }
  if (!tb.children.length) {
    const tr = document.createElement("tr");
    cell(tr, "로그 파일 없음");
    tb.appendChild(tr);
  }
}
$("btnFiles").addEventListener("click", loadFiles);
$("btnLogClear").addEventListener("click", () => { logs = []; renderLogs(); });

async function poll() {
  try {
    const snap = await RM.getJson("/api/fleet");
    RM.markConnection(snap.httpStatus === 200);
    if (snap.httpStatus === 200) render(snap);
    await pollLogs();
  } catch (_) {
    RM.markConnection(false);
  }
  setTimeout(poll, POLL_MS);
}

async function init() {
  const g = await RM.getJson("/api/route/graph");
  if (g.httpStatus === 200) {
    graph = g;
    map.setGraph(g);
  } else {
    RM.setMsg($("cmdMsg"), `노드 그래프 없음: ${g.msg || g.httpStatus}`, "err");
  }
  poll();
}

init();
