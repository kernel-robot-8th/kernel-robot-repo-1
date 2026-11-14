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

// 출발 기본값 = 「현재」: 경로가 비어 있고 로봇이 끝 기억 노드를 보고하면 그 노드를 첫 노드로 미리 찍어 둠.
// 같은 노드로는 한 번만(사용자가 지우면 다시 안 채움).
let seededEnd = null;
function seedStartFromEnd(s) {
  const e = RM.endMemory(s);
  if (!e) { seededEnd = null; return; }
  if (seededEnd === e.node || routeIds.length) return;
  seededEnd = e.node;
  map.setRoute([e.node]);
}

function updateButtons() {
  const st = lastState ? lastState.state : null;
  const busy = st && !["idle", "arrived", "handoff", "stopped", "error"].includes(st);
  $("btnStart").disabled = routeIds.length < 2 || !!busy;
  $("btnLoad").disabled = st !== "loading";
  $("btnHome").disabled = !!busy && st !== "loading";
  // 일시정지·후진 (FFT1-172). 주행 중 = 전진 추종·통과·전진·회전. 거절은 로봇의 line_nav 가 한다.
  $("btnPause").disabled = !["follow", "pass", "advance", "rotate"].includes(st);
  $("btnResume").disabled = st !== "paused";
  $("btnBack").disabled = lastStale || !$("backTo").value;   // 후보가 없으면 value 가 비어 꺼짐
  map.editable = !busy;
}

async function command(body) {
  const r = await RM.postJson("api/route/command", body);
  RM.setMsg($("cmdMsg"), r.success ? `보냄: ${body.cmd}` : `실패: ${r.msg || r.httpStatus}`,
    r.success ? "ok" : "err");
  return !!r.success;
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

// 후진 속도(0.01~0.03 m/s): 직선·회전 속도와 따로 적용. 로봇이 reverse_follow_speed 를 보고하면 그 값을 따라감.
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

$("btnStart").addEventListener("click", async () => {
  // 끝 기억 노드와 다른 곳에서 출발하면 확인 → 먼저 stop(기억 지움) 뒤 start. 취소하면 출발 안 함.
  const warn = RM.startMismatchMsg(lastState, routeIds[0]);
  if (warn) {
    // 1) 기본 제안: 로봇이 기억하는 노드에서 같은 목적지로 다시 계획해 출발(기억 유지)
    const here = RM.endMemory(lastState).node;
    const goal = routeIds[routeIds.length - 1];
    if (goal !== here && confirm(`로봇이 ${here} 에 있다고 기억 중입니다.\n[확인] ${here} → ${goal} 경로로 다시 계획해 출발\n[취소] 다른 선택`)) {
      const r = await RM.getJson(`api/route/plan?from=${encodeURIComponent(here)}&to=${encodeURIComponent(goal)}`);
      if (!Array.isArray(r.route) || r.route.length < 2) {
        RM.setMsg($("cmdMsg"), `경로 계획 실패: ${here} → ${goal}`, "err");
        return;
      }
      map.setRoute(r.route);
      routeIds = r.route;
    } else {
      // 2) 로봇을 지도 경로의 첫 노드로 옮겨 놓았으면: 기억을 지우고(stop) 그 경로로 출발
      if (!confirm(warn)) return;
      if (!(await command({ cmd: "stop" }))) return;   // stop 실패면 start 하지 않음(실패 문구 유지)
    }
  }
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
$("btnClear").addEventListener("click", () => { map.setRoute([]); seededEnd = null; seedStartFromEnd(lastState); });
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
  syncReverseSpeed(s && s.speed);
  lastState = s;
  seedStartFromEnd(s);
  RM.fillBackSelect($("backTo"), RM.backCandidates(s));
  lastStale = !!snap.stale;
  $("robotChip").textContent = "robot " + (snap.robot || "-");
  $("stateChip").textContent = "state " + (s ? s.state : "-");
  $("sStale").hidden = !snap.stale;
  if (!s) { updateButtons(); return; }
  $("sState").textContent = s.state;
  $("sReason").textContent = s.reason || "-";
  $("sLeg").textContent = s.leg ? `${s.leg.from} → ${s.leg.to} (${s.leg.action}, ${s.leg.detect})` : "-";
  { const rt = RM.reverseText(s);
  $("sReverse").textContent = rt === "-" && s && s.back_why ? `후진 불가: ${s.back_why}` : rt; }
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
