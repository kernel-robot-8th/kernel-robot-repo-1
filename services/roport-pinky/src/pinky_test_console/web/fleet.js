// pinky_test_console 테스트 관제(노트북). /api/fleet 를 1 s 마다 읽어 지도·표를 그리고,
// 목적지 클릭 → /api/route/plan 으로 최단 경로 → /api/fleet/route 로 선택한 로봇에 명령. 로그는 /api/fleet/logs.
"use strict";

const $ = (id) => document.getElementById(id);
const POLL_MS = 1000;
const LOG_KEEP = 1500;
const BUSY = ["follow", "pass", "advance", "rotate", "loading", "reverse"];
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
  return st && Array.isArray(st.route) && st.route.length && ["arrived", "handoff"].includes(st.state)
    ? st.route[st.route.length - 1] : null;
}

const map = new RM.RouteMap($("routeCanvas"), () => {}, { mode: "goal", onGoal: (id) => setGoal(id) });

// ---------------- 출발·목적지·경로 ----------------
function selPick() {
  if (!selected) return null;
  if (!pick[selected]) pick[selected] = { start: null, goal: null, route: [] };
  return pick[selected];
}

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
    const r = await RM.getJson(`/api/route/plan?from=${encodeURIComponent(p.start)}&to=${encodeURIComponent(p.goal)}`);
    if (r.httpStatus === 200) p.route = r.route;
    else RM.setMsg($("cmdMsg"), `경로 없음: ${r.msg || r.httpStatus}`, "err");
  }
  showPick();
}

function showPick() {
  const p = selPick();
  $("goalText").textContent = p && p.goal ? p.goal : "지도에서 정차 노드를 누름";
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

$("startSel").addEventListener("change", (ev) => { selPick().start = ev.target.value; plan(); });

function updateButtons() {
  const r = robots.find((x) => x.name === selected);
  const st = navState(r);
  const busy = st && BUSY.includes(st.state);
  const p = selPick();
  $("btnStart").disabled = !r || !p || p.route.length < 2 || !!busy || !(r.status && r.status.ok);
  $("btnLoad").disabled = !r || !st || st.state !== "loading";
  $("btnStop").disabled = !r;
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

$("btnStart").addEventListener("click", () => {
  const p = selPick();
  command({ cmd: "start", route: p.route }).then((ok) => {
    if (ok) { p.goal = null; p.route = []; p.start = null; showPick(); }
  });
});
$("btnLoad").addEventListener("click", () => command({ cmd: "load_done" }));
$("btnStop").addEventListener("click", () => command({ cmd: "stop" }));

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
    cell(tr, r.status ? (r.status.ok ? "OK" : "꺼짐") : "-");
    cell(tr, consoleMode(r));
    cell(tr, st ? st.state : "-");
    cell(tr, st && st.leg ? `${st.leg.from}→${st.leg.to}` : "-");
    tr.addEventListener("click", () => {
      selected = r.name;
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
  $("sState").textContent = st ? st.state : "-";
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
