// pinky_test_console 노드 경로 주행 화면. /api/route 를 0.25 s 마다 읽고 /api/route/command 로 명령.
"use strict";

const $ = (id) => document.getElementById(id);
const POLL_MS = 250;
let routeIds = [];
let lastState = null;

const map = new RM.RouteMap($("routeCanvas"), (ids) => {
  routeIds = ids;
  $("routeText").textContent = ids.length ? ids.join(" → ") : "-";
  updateButtons();
});

function updateButtons() {
  const st = lastState ? lastState.state : null;
  const busy = st && !["idle", "arrived", "handoff", "stopped", "error"].includes(st);
  $("btnStart").disabled = routeIds.length < 2 || !!busy;
  $("btnLoad").disabled = st !== "loading";
  map.editable = !busy;
}

async function command(body) {
  const r = await RM.postJson("/api/route/command", body);
  RM.setMsg($("cmdMsg"), r.success ? `보냄: ${body.cmd}` : `실패: ${r.msg || r.httpStatus}`,
    r.success ? "ok" : "err");
}

$("btnStart").addEventListener("click", () => {
  const body = { cmd: "start", route: routeIds };
  const off = $("offset").value.trim();
  if (off !== "") body.start_offset_cm = Number(off);
  command(body);
});
$("btnLoad").addEventListener("click", () => command({ cmd: "load_done" }));
$("btnStop").addEventListener("click", () => command({ cmd: "stop" }));
$("btnClear").addEventListener("click", () => map.setRoute([]));
$("presets").addEventListener("change", (ev) => {
  const v = ev.target.value;
  if (v) map.setRoute(v.split(","));
  ev.target.value = "";
});

async function estop() {
  const r = await RM.postJson("/api/estop", {});
  RM.setMsg($("estopStatus"), r.success ? "정지 보냄" : "정지 실패", r.success ? "ok" : "err");
}
$("btnEstop").addEventListener("click", estop);
document.addEventListener("keydown", (ev) => {
  if (ev.code === "Space" || ev.code === "Escape") {
    if (ev.target.tagName === "INPUT") return;
    ev.preventDefault();
    estop();
  }
});

function fmt(v) { return v === null || v === undefined ? "-" : String(v); }

function render(snap) {
  const s = snap.state;
  lastState = s;
  $("robotChip").textContent = "robot " + (snap.robot || "-");
  $("stateChip").textContent = "state " + (s ? s.state : "-");
  $("sStale").hidden = !snap.stale;
  if (!s) { updateButtons(); return; }
  $("sState").textContent = s.state;
  $("sReason").textContent = s.reason || "-";
  $("sLeg").textContent = s.leg ? `${s.leg.from} → ${s.leg.to} (${s.leg.action}, ${s.leg.detect})` : "-";
  $("sTravel").textContent = `${fmt(s.traveled_cm)}` + (s.window_cm ? ` / ${s.window_cm[0]}~${s.window_cm[1]}` : "");
  $("sRot").textContent = fmt(s.rotated_deg);
  $("sIr").textContent = s.ir ? s.ir.raw.join(", ") : "-";
  $("sOn").textContent = s.ir ? s.ir.on_line.map((b) => (b ? "■" : "□")).join(" ") : "-";
  $("sOdom").textContent = s.odom_ok ? "OK" : "없음";
  $("sCmd").textContent = s.cmd ? `${s.cmd.linear}, ${s.cmd.angular}` : "-";
  $("sLast").textContent = s.last_command ? `${s.last_command.cmd}: ${s.last_command.ok ? "ok" : "거부"} (${s.last_command.detail})` : "-";
  const tb = $("events").querySelector("tbody");
  tb.replaceChildren();
  for (const e of (s.events || []).slice().reverse().slice(0, 30)) {
    const tr = document.createElement("tr");
    for (const v of [e.t, e.event, e.detail]) {
      const td = document.createElement("td");
      td.textContent = fmt(v);
      tr.appendChild(td);
    }
    tb.appendChild(tr);
  }
  map.setProgress(s);
  updateButtons();
}

async function poll() {
  try {
    const snap = await RM.getJson("/api/route");
    RM.markConnection(snap.httpStatus === 200);
    if (snap.httpStatus === 200) render(snap);
  } catch (_) {
    RM.markConnection(false);
  }
  setTimeout(poll, POLL_MS);
}

async function init() {
  const g = await RM.getJson("/api/route/graph");
  if (g.httpStatus !== 200) {
    RM.setMsg($("cmdMsg"), `노드 그래프 없음: ${g.msg || g.httpStatus}`, "err");
  } else {
    map.setGraph(g);
    for (const p of g.presets) {
      const o = document.createElement("option");
      o.value = p.route.join(",");
      o.textContent = p.name;
      $("presets").appendChild(o);
    }
  }
  poll();
}

init();
