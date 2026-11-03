// pinky_test_console 테스트 관제(노트북). /api/fleet 를 1 s 마다 읽고, /api/fleet/route 로 선택한 로봇에 명령.
"use strict";

const $ = (id) => document.getElementById(id);
const POLL_MS = 1000;
const ROLE = { logistics: "물류", smartcart: "스마트카트" };
let selected = null;
let routeIds = [];
let robots = [];

const map = new RM.RouteMap($("routeCanvas"), (ids) => {
  routeIds = ids;
  $("routeText").textContent = ids.length ? ids.join(" → ") : "-";
  updateButtons();
});

function navState(r) {
  return r && r.status && r.status.ok && r.status.route ? r.status.route.state : null;
}

function updateButtons() {
  const r = robots.find((x) => x.name === selected);
  const st = navState(r);
  const busy = st && !["idle", "arrived", "handoff", "stopped", "error"].includes(st.state);
  $("btnStart").disabled = !r || routeIds.length < 2 || !!busy;
  $("btnLoad").disabled = !r || !st || st.state !== "loading";
  $("btnStop").disabled = !r;
}

async function command(body) {
  if (!selected) return;
  const r = await RM.postJson("/api/fleet/route", Object.assign({ name: selected }, body));
  const ok = r.success && r.result && r.result.success;
  RM.setMsg($("cmdMsg"), ok ? `${selected}: ${body.cmd} 보냄` : `실패: ${r.msg || (r.result && r.result.msg) || r.httpStatus}`,
    ok ? "ok" : "err");
}

$("btnStart").addEventListener("click", () => command({ cmd: "start", route: routeIds }));
$("btnLoad").addEventListener("click", () => command({ cmd: "load_done" }));
$("btnStop").addEventListener("click", () => command({ cmd: "stop" }));
$("btnClear").addEventListener("click", () => map.setRoute([]));
$("presets").addEventListener("change", (ev) => {
  if (ev.target.value) map.setRoute(ev.target.value.split(","));
  ev.target.value = "";
});

async function stopAll() {
  const r = await RM.postJson("/api/fleet/stop_all", {});
  RM.setMsg($("estopStatus"), r.success ? "전체 정지 보냄" : "전체 정지 실패", r.success ? "ok" : "err");
}
$("btnEstop").addEventListener("click", stopAll);
document.addEventListener("keydown", (ev) => {
  if ((ev.code === "Space" || ev.code === "Escape") && ev.target.tagName !== "INPUT") {
    ev.preventDefault();
    stopAll();
  }
});

function cell(tr, text) {
  const td = document.createElement("td");
  td.textContent = text;
  tr.appendChild(td);
  return td;
}

function render(snap) {
  robots = snap.robots || [];
  const tb = $("robots").querySelector("tbody");
  tb.replaceChildren();
  for (const r of robots) {
    const st = navState(r);
    const tr = document.createElement("tr");
    if (r.name === selected) tr.classList.add("mk-seen");
    cell(tr, r.name);
    cell(tr, ROLE[r.role] || r.role);
    cell(tr, r.ip);
    cell(tr, String(r.domain));
    cell(tr, r.status ? (r.status.ok ? "OK" : "꺼짐") : "-");
    cell(tr, st ? st.state : "-");
    cell(tr, st && st.leg ? `${st.leg.from}→${st.leg.to}` : "-");
    const td = cell(tr, "");
    const b = document.createElement("button");
    b.type = "button";
    b.className = "btn btn-outline btn-small";
    b.textContent = r.name === selected ? "선택됨" : "선택";
    b.addEventListener("click", () => { selected = r.name; $("selName").textContent = r.name; render(snap); });
    td.appendChild(b);
    tb.appendChild(tr);
  }
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
  map.setProgress(st);
  updateButtons();
}

async function poll() {
  try {
    const snap = await RM.getJson("/api/fleet");
    RM.markConnection(snap.httpStatus === 200);
    if (snap.httpStatus === 200) render(snap);
  } catch (_) {
    RM.markConnection(false);
  }
  setTimeout(poll, POLL_MS);
}

async function init() {
  const g = await RM.getJson("/api/route/graph");
  if (g.httpStatus === 200) {
    map.setGraph(g);
    for (const p of g.presets) {
      const o = document.createElement("option");
      o.value = p.route.join(",");
      o.textContent = p.name;
      $("presets").appendChild(o);
    }
  } else {
    RM.setMsg($("cmdMsg"), `노드 그래프 없음: ${g.msg || g.httpStatus}`, "err");
  }
  poll();
}

init();
