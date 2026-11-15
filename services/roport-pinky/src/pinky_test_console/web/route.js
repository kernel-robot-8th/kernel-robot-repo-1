// pinky_test_console 노드 경로 주행 화면. /api/route 를 0.25 s 마다 읽고 /api/route/command 로 명령.
"use strict";

const $ = (id) => document.getElementById(id);
const POLL_MS = 250;
let routeIds = [];
let lastState = null;
let lastStale = true;

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
  $("btnHome").disabled = !!busy && st !== "loading";
  map.editable = !busy;
}

async function command(body) {
  const r = await RM.postJson("api/route/command", body);
  RM.setMsg($("cmdMsg"), r.success ? `보냄: ${body.cmd}` : `실패: ${r.msg || r.httpStatus}`,
    r.success ? "ok" : "err");
}


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
  const near = typeof sp.approach_speed === "number" ? ` · 노드 앞 ${(sp.approach_speed * 100).toFixed(1)} cm/s` : "";
  $("spNow").textContent = `현재 ${(sp.base_speed * 100).toFixed(1)} cm/s · ${sp.rotate_speed.toFixed(2)} rad/s${near}`;
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

$("btnStart").addEventListener("click", () => {
  const body = { cmd: "start", route: routeIds };
  const off = $("offset").value.trim();
  if (off !== "") body.start_offset_cm = Number(off);
  command(body);
});
$("btnLoad").addEventListener("click", () => command({ cmd: "load_done" }));
$("btnStop").addEventListener("click", () => command({ cmd: "stop" }));
$("btnHome").addEventListener("click", async () => {
  // 경로 끝(도착·인계·상차 대기)이면 로봇이 아는 위치에서 복귀. 그 밖(대기·정지·오류·상태 오래됨)은
  // 지금 노드를 「지금 위치」에서 새로 골라야 함. 지난 경로의 출발점은 쓰지 않음(PR #43 리뷰 P2)
  const st = lastState ? lastState.state : null;
  const known = !lastStale && ["arrived", "handoff", "loading"].includes(st);
  const here = $("homeFrom").value;
  if (!known && !here) {
    RM.setMsg($("cmdMsg"), "로봇 위치를 모릅니다. 「지금 위치」에서 로봇이 서 있는 노드를 고르세요.", "err");
    $("homeFrom").focus();
    return;
  }
  const body = known ? {} : { start: here };
  $("homeFrom").value = "";
  const r = await RM.postJson("api/route/return", body);
  RM.setMsg($("cmdMsg"), r.success ? r.msg : `복귀 실패: ${r.msg || r.httpStatus}`, r.success ? "ok" : "err");
});
$("btnClear").addEventListener("click", () => map.setRoute([]));
$("presets").addEventListener("change", (ev) => {
  const v = ev.target.value;
  if (v) map.setRoute(v.split(","));
  ev.target.value = "";
});

async function estop() {
  const r = await RM.postJson("api/estop", {});
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

function showBattery(b) {
  const chip = $("battChip");
  if (!b || b.percent === null || b.percent === undefined) {
    chip.textContent = "배터리 -";
    chip.className = "chip";
    return;
  }
  const v = b.voltage === null || b.voltage === undefined ? "" : ` ${b.voltage.toFixed(1)} V`;
  chip.textContent = `배터리 ${Math.round(b.percent)}%${v}${b.stale ? " (오래됨)" : ""}`;
  chip.className = "chip" + (b.stale ? " batt-stale" : b.percent < 20 ? " batt-low" : b.percent < 40 ? " batt-mid" : "");
}

function render(snap) {
  showBattery(snap.battery);
  const s = snap.state;
  syncSpeed(s && s.speed);
  lastState = s;
  lastStale = !!snap.stale;
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
  map.setProgress(s, snap.robot);
  updateButtons();
}

async function poll() {
  try {
    const snap = await RM.getJson("api/route");
    RM.markConnection(snap.httpStatus === 200);
    if (snap.httpStatus === 200) render(snap);
  } catch (_) {
    RM.markConnection(false);
  }
  setTimeout(poll, POLL_MS);
}

async function init() {
  const g = await RM.getJson("api/route/graph");
  if (g.httpStatus !== 200) {
    RM.setMsg($("cmdMsg"), `노드 그래프 없음: ${g.msg || g.httpStatus}`, "err");
  } else {
    map.setGraph(g);
    for (const n of g.nodes) {
      const o = document.createElement("option");
      o.value = n.id;
      o.textContent = n.place ? `${n.id} (${n.place})` : n.id;
      $("homeFrom").appendChild(o);
    }
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
