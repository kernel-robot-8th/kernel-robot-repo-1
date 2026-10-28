/*
 * Derived from Pinky Pro (https://github.com/pinklab-art/pinky_pro,
 *   commit 014a09f289e6988894fbffde2624fdb1e257815b)
 *   original file: pinky_navigation/scripts/index.html (inline <script>)
 * Original work licensed under the Apache License, Version 2.0.
 * Modified for pinky_test_console (2026): teleop (hold-to-drive, WASD/arrows),
 *   stop bar, status panel, map save/list, versioned grid transfer drawn with
 *   ImageData, pointer coordinates corrected for CSS-scaled canvas.
 *
 * 외부 라이브러리·CDN 없음 (강의실 오프라인 환경).
 */
"use strict";

const $ = (id) => document.getElementById(id);

const canvas = $("mapCanvas");
const ctx = canvas.getContext("2d");

let info = { mode: "-", max_linear: 0.10, max_angular: 0.6, deadman_sec: 0.3, teleop_hz: 10 };
let latestState = null;

// 레이어별 캐시: { version, meta, image(offscreen canvas) }
const grids = { map: null, global: null, local: null };

let showGlobalCostmap = true;
let showLocalCostmap = true;

// goal 드래그 상태
let isDraggingGoal = false;
let goalStartWorld = null;
let goalCurrentCanvas = null;
let clickMode = "goal";  // "goal" | "initialpose"

// ------------------------------------------------------------
// 공통 HTTP
// ------------------------------------------------------------
async function postJson(url, body, timeoutMs = 5000) {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), timeoutMs);
  try {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
      signal: ctrl.signal,
    });
    let data = {};
    try { data = await res.json(); } catch (_) { /* 빈 응답 */ }
    data.httpStatus = res.status;
    return data;
  } finally {
    clearTimeout(timer);
  }
}

function setMsg(el, text, kind) {
  el.classList.remove("ok", "err");
  if (kind) el.classList.add(kind);
  el.textContent = text;
}

// ------------------------------------------------------------
// 연결 상태
// ------------------------------------------------------------
let lastOkTime = 0;
function markConnection(ok) {
  if (ok) lastOkTime = Date.now();
  const alive = Date.now() - lastOkTime < 2000;
  $("connDot").className = "chip-dot " + (alive ? "live" : "dead");
  $("connText").textContent = alive ? "LIVE" : "연결 끊김";
}

// ------------------------------------------------------------
// 정지 버튼 (상단)
// ------------------------------------------------------------
async function emergencyStop() {
  releaseAll(false);
  setMsg($("estopStatus"), "정지 요청 중...");
  try {
    const d = await postJson("/api/estop", {}, 3000);
    if (d.success) {
      setMsg($("estopStatus"), "정지 완료" + (d.nav_cancelled ? " · Nav2 취소" : ""), "ok");
    } else {
      setMsg($("estopStatus"), "정지 실패", "err");
    }
  } catch (e) {
    setMsg($("estopStatus"), "서버 응답 없음! 로봇을 직접 확인하세요", "err");
  }
}
$("btnEstop").addEventListener("click", emergencyStop);

// ------------------------------------------------------------
// Teleop: 누르고 있는 동안만. 서버가 0.3 s deadman·속도 상한을 다시 강제한다.
// ------------------------------------------------------------
const held = new Set();          // "fwd" | "back" | "left" | "right"
let teleopTimer = null;
let teleopInFlight = false;
let speedScale = 1.0;

const KEY_DIR = {
  w: "fwd", W: "fwd", ArrowUp: "fwd",
  s: "back", S: "back", ArrowDown: "back",
  a: "left", A: "left", ArrowLeft: "left",
  d: "right", D: "right", ArrowRight: "right",
};

function currentCmd() {
  let lin = 0, ang = 0;
  if (held.has("fwd")) lin += 1;
  if (held.has("back")) lin -= 1;
  if (held.has("left")) ang += 1;
  if (held.has("right")) ang -= 1;
  return {
    linear: lin * info.max_linear * speedScale,
    angular: ang * info.max_angular * speedScale,
  };
}

function renderPad() {
  document.querySelectorAll(".pad-btn").forEach((b) => {
    b.classList.toggle("held", held.has(b.dataset.dir));
  });
}

async function sendTeleop() {
  if (held.size === 0 || teleopInFlight) return;
  teleopInFlight = true;
  try {
    // deadman(0.3 s)보다 짧은 타임아웃: 늦은 응답을 기다리지 않는다
    const d = await postJson("/api/teleop", currentCmd(), 250);
    if (d.nav_cancelled) setMsg($("teleopStatus"), "조종 입력 → Nav2 목표를 먼저 취소했습니다.", "ok");
  } catch (_) {
    setMsg($("teleopStatus"), "teleop 요청 실패 (서버가 0.3 s 후 자동 정지)", "err");
  } finally {
    teleopInFlight = false;
  }
}

function press(dir) {
  if (held.has(dir)) return;
  held.add(dir);
  renderPad();
  sendTeleop();
  if (!teleopTimer) {
    teleopTimer = setInterval(sendTeleop, Math.round(1000 / info.teleop_hz));
  }
}

function release(dir) {
  if (!held.delete(dir)) return;
  renderPad();
  if (held.size === 0) releaseAll(true);
}

function releaseAll(sendStop) {
  const wasHeld = held.size > 0 || teleopTimer !== null;
  held.clear();
  renderPad();
  if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; }
  if (sendStop && wasHeld) {
    postJson("/api/teleop/stop", {}, 1000).catch(() => {});
  }
}

document.querySelectorAll(".pad-btn").forEach((btn) => {
  const dir = btn.dataset.dir;
  btn.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    btn.setPointerCapture(e.pointerId);
    press(dir);
  });
  ["pointerup", "pointercancel", "lostpointercapture"].forEach((ev) =>
    btn.addEventListener(ev, () => release(dir)));
  btn.addEventListener("contextmenu", (e) => e.preventDefault());  // 모바일 길게 누르기
});

function isTyping(e) {
  const t = e.target;
  return t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA");
}

window.addEventListener("keydown", (e) => {
  if (isTyping(e)) return;
  if (e.key === " " || e.key === "Escape") {
    e.preventDefault();
    emergencyStop();
    return;
  }
  const dir = KEY_DIR[e.key];
  if (!dir) return;
  e.preventDefault();  // 방향키 스크롤 방지
  press(dir);
});

window.addEventListener("keyup", (e) => {
  const dir = KEY_DIR[e.key];
  if (dir) release(dir);
});

// 창 포커스를 잃으면 keyup 이 오지 않을 수 있다 → 바로 정지
window.addEventListener("blur", () => releaseAll(true));
document.addEventListener("visibilitychange", () => {
  if (document.hidden) releaseAll(true);
});
window.addEventListener("pagehide", () => releaseAll(true));

$("speedScale").addEventListener("input", (e) => {
  speedScale = Number(e.target.value) / 100;
  $("speedText").textContent = e.target.value + "%";
});

function updateTeleopView(t) {
  if (!t) return;
  $("teleopState").textContent = t.active ? "DRIVING" : "idle";
  $("cmdText").innerHTML = "";
  $("cmdText").append(
    `${t.linear.toFixed(2)} m/s`, document.createElement("br"), `${t.angular.toFixed(2)} rad/s`);
}

// ------------------------------------------------------------
// Nav2 클릭 모드 / 상태 / 취소 (원본 기능)
// ------------------------------------------------------------
function setClickMode(mode) {
  clickMode = mode;
  const isGoal = (mode === "goal");
  $("btnModeGoal").classList.toggle("active", isGoal);
  $("btnModeInit").classList.toggle("active", !isGoal);
  $("modeHint").textContent = isGoal
    ? "Click & drag on the map to set the goal position and heading."
    : "Click & drag on the map to set the robot's current pose.";
}
$("btnModeGoal").addEventListener("click", () => setClickMode("goal"));
$("btnModeInit").addEventListener("click", () => setClickMode("initialpose"));

function updateNavStatus(navigating) {
  $("navStatusDot").classList.toggle("active", navigating);
  $("navStatusText").textContent = navigating ? "주행 중 (Navigating)" : "대기 중 (Idle)";
  $("btnNavStop").disabled = !navigating;
}

$("btnNavStop").addEventListener("click", async () => {
  setMsg($("navStopStatus"), "취소 요청 중...");
  try {
    const d = await postJson("/api/nav/stop", {});
    if (d.success) setMsg($("navStopStatus"), "취소 요청을 보냈습니다.", "ok");
    else setMsg($("navStopStatus"), "취소 실패 (Nav2 실행 중인가요?)", "err");
  } catch (e) {
    setMsg($("navStopStatus"), "취소 요청 중 오류가 발생했습니다.", "err");
  }
});

// ------------------------------------------------------------
// SLAM: 초기화 / 저장
// ------------------------------------------------------------
$("btnSlamReset").addEventListener("click", async () => {
  if (!confirm("현재 SLAM 지도를 지우고 새로 시작할까요?")) return;
  setMsg($("slamStatus"), "Resetting SLAM map...");
  try {
    const d = await postJson("/api/slam/reset", {});
    if (d.success) setMsg($("slamStatus"), "SLAM reset requested. 천천히 주행하며 새 지도를 만드세요.", "ok");
    else setMsg($("slamStatus"), "/slam_toolbox/reset 호출 실패 (mode:=slam 인가요?)", "err");
  } catch (e) {
    setMsg($("slamStatus"), "Error while calling SLAM reset.", "err");
  }
});

const NAME_RE = /^[A-Za-z0-9_-]{1,64}$/;

async function saveMap(overwrite) {
  const name = ($("mapNameInput").value || "").trim();
  if (!NAME_RE.test(name)) {
    setMsg($("slamStatus"), "이름은 영문·숫자·_·- 만, 1~64자", "err");
    return;
  }
  setMsg($("slamStatus"), `Saving '${name}'... (몇 초 걸립니다)`);
  $("btnSlamSave").disabled = true;
  try {
    const d = await postJson("/api/slam/save_map", { name, overwrite }, 30000);
    if (d.httpStatus === 409 && !overwrite) {
      if (confirm(`'${name}' 지도가 이미 있습니다. 덮어쓸까요?`)) {
        await saveMap(true);
      } else {
        setMsg($("slamStatus"), "저장 취소", "err");
      }
      return;
    }
    if (d.success) {
      setMsg($("slamStatus"), `저장됨: ${d.yaml}`, "ok");
      refreshMaps(d.name);
    } else {
      setMsg($("slamStatus"), "저장 실패: " + (d.msg || ""), "err");
    }
  } catch (e) {
    setMsg($("slamStatus"), "SaveMap 요청 오류/시간 초과", "err");
  } finally {
    $("btnSlamSave").disabled = false;
  }
}
$("btnSlamSave").addEventListener("click", () => saveMap(false));

// ------------------------------------------------------------
// 저장된 지도 목록
// ------------------------------------------------------------
function fmtBytes(n) {
  if (n >= 1048576) return (n / 1048576).toFixed(1) + " MB";
  return Math.round(n / 1024) + " KB";
}

function showNavCmd(m) {
  $("navCmd").value =
    `ros2 launch pinky_test_console console.launch.py mode:=nav map:=${m.yaml}`;
}

async function refreshMaps(selectName) {
  try {
    const res = await fetch("/api/maps");
    const d = await res.json();
    $("mapDirText").textContent = d.map_dir;
    const ul = $("mapList");
    ul.replaceChildren();
    if (d.maps.length === 0) {
      const li = document.createElement("li");
      li.textContent = "(저장된 지도 없음)";
      ul.append(li);
      return;
    }
    d.maps.forEach((m) => {
      const li = document.createElement("li");
      const name = document.createElement("span");
      name.textContent = m.name + (m.has_pgm ? "" : " (pgm 없음)");
      const meta = document.createElement("span");
      meta.className = "meta";
      meta.textContent = `${new Date(m.mtime * 1000).toLocaleString()} · ${fmtBytes(m.pgm_bytes)}`;
      li.append(name, meta);
      li.addEventListener("click", () => {
        ul.querySelectorAll("li").forEach((x) => x.classList.remove("selected"));
        li.classList.add("selected");
        showNavCmd(m);
        $("navCmd").select();
      });
      if (m.name === selectName) { li.classList.add("selected"); showNavCmd(m); }
      ul.append(li);
    });
  } catch (e) {
    $("mapDirText").textContent = "지도 목록을 불러오지 못했습니다.";
  }
}
$("btnMapsRefresh").addEventListener("click", () => refreshMaps());
$("navCmd").addEventListener("focus", (e) => e.target.select());

// ------------------------------------------------------------
// 상태 패널 (1 s)
// ------------------------------------------------------------
let baselineUsedKb = null;
let lastUsedKb = null;

function mb(kb) { return (kb / 1024).toFixed(0) + " MB"; }

function setValue(id, text, level) {
  const el = $(id);
  el.textContent = text;
  el.classList.remove("warn", "err");
  if (level) el.classList.add(level);
}

function renderStatus(s) {
  const m = s.memory;
  if (m) {
    lastUsedKb = m.used_kb;
    const pct = m.used_pct;
    const level = pct >= 90 ? "err" : pct >= 75 ? "warn" : null;
    setValue("memUsed", `${mb(m.used_kb)} / ${mb(m.total_kb)} (${pct}%)`, level);
    const bar = $("memBar");
    bar.className = "bar" + (level ? " " + level : "");
    bar.firstElementChild.style.width = Math.min(100, pct) + "%";
    setValue("memAvail", mb(m.available_kb), level);
    setValue("swapUsed", m.swap_total_kb ? `${mb(m.swap_used_kb)} / ${mb(m.swap_total_kb)}` : "swap 없음",
      m.swap_used_kb > 0 ? "warn" : null);
    if (baselineUsedKb !== null) {
      const delta = m.used_kb - baselineUsedKb;
      setValue("memDelta", (delta >= 0 ? "+" : "−") + mb(Math.abs(delta)));
    }
  }
  setValue("loadAvg", s.loadavg ? s.loadavg.map((v) => v.toFixed(2)).join(" / ") : "-");
  if (s.cpu_temp_c !== null && s.cpu_temp_c !== undefined) {
    setValue("cpuTemp", s.cpu_temp_c.toFixed(1) + " °C",
      s.cpu_temp_c >= 80 ? "err" : s.cpu_temp_c >= 70 ? "warn" : null);
  } else {
    setValue("cpuTemp", "-");
  }
  const bp = s.battery_percent, bv = s.battery_voltage;
  setValue("battPct", bp ? `${bp.value.toFixed(0)} %${bp.stale ? " (오래됨)" : ""}` : "수신 없음",
    bp && (bp.stale || bp.value < 20) ? "warn" : null);
  setValue("battVolt", bv ? `${bv.value.toFixed(2)} V${bv.stale ? " (오래됨)" : ""}` : "수신 없음",
    bv && bv.value <= 6.8 ? "err" : (bv && bv.stale ? "warn" : null));
  setValue("consoleRss", s.console_rss_kb
    ? `${mb(s.console_rss_kb)} (${mb(s.console_peak_rss_kb || 0)}) · pid ${s.pid}` : "-");
}

$("btnBaseline").addEventListener("click", () => {
  if (lastUsedKb === null) return;
  baselineUsedKb = lastUsedKb;
  setValue("memDelta", "+0 MB");
});

async function fetchStatus() {
  try {
    const res = await fetch("/api/status");
    if (res.ok) renderStatus(await res.json());
  } catch (_) { /* 연결 표시는 fetchState 가 담당 */ }
}

// ------------------------------------------------------------
// 지도 상태 폴링 (변경된 grid 만 전송받음)
// ------------------------------------------------------------
function stateUrl() {
  const layers = ["map"];
  if (showGlobalCostmap) layers.push("global");
  if (showLocalCostmap) layers.push("local");
  const v = (k) => (grids[k] ? grids[k].version : -1);
  return `/api/state?layers=${layers.join(",")}&map_v=${v("map")}&global_v=${v("global")}&local_v=${v("local")}`;
}

// grid → offscreen canvas (1 셀 = 1 픽셀, y 뒤집기)
function buildImage(g, colorFn) {
  const off = document.createElement("canvas");
  off.width = g.width;
  off.height = g.height;
  const octx = off.getContext("2d");
  const img = octx.createImageData(g.width, g.height);
  const px = img.data;
  for (let iy = 0; iy < g.height; iy++) {
    const row = g.height - 1 - iy;
    for (let ix = 0; ix < g.width; ix++) {
      const c = colorFn(g.data[iy * g.width + ix]);
      if (!c) continue;
      const o = (row * g.width + ix) * 4;
      px[o] = c[0]; px[o + 1] = c[1]; px[o + 2] = c[2]; px[o + 3] = c[3];
    }
  }
  octx.putImageData(img, 0, 0);
  return off;
}

const MAP_COLOR = (v) => {
  if (v < 0) return [75, 85, 99, 255];        // unknown
  if (v === 0) return [255, 255, 255, 255];   // free
  if (v >= 100) return [0, 0, 0, 255];        // obstacle
  return [209, 213, 219, 255];
};
const GLOBAL_COLOR = (v) => (v > 0 ? [56, 189, 248, 128] : null);
const LOCAL_COLOR = (v) => (v > 0 ? [124, 58, 237, 178] : null);
const LAYER_COLOR = { map: MAP_COLOR, global: GLOBAL_COLOR, local: LOCAL_COLOR };

function mergeGrid(key, g) {
  if (!g) { grids[key] = null; return; }
  if (g.data) {
    grids[key] = { version: g.version, meta: g, image: buildImage(g, LAYER_COLOR[key]) };
    delete g.data;
  } else if (grids[key]) {
    grids[key].meta = g;
  }
}

async function fetchState() {
  try {
    const res = await fetch(stateUrl());
    if (!res.ok) { markConnection(false); return; }
    const data = await res.json();
    markConnection(true);
    ["map", "global", "local"].forEach((k) => mergeGrid(k, data[k]));
    latestState = data;
    drawState();
    $("updateText").textContent = new Date().toLocaleTimeString();
    const p = data.pose;
    $("poseText").textContent = p
      ? `x=${p.x.toFixed(2)}, y=${p.y.toFixed(2)}, yaw=${p.yaw.toFixed(2)}`
      : "pose -";
    updateNavStatus(!!data.navigating);
    updateTeleopView(data.teleop);
  } catch (e) {
    markConnection(false);
  }
}

// ------------------------------------------------------------
// 좌표 변환 (map frame ↔ canvas 픽셀)
// ------------------------------------------------------------
function view() {
  const g = grids.map && grids.map.meta;
  if (!g) return null;
  const scale = Math.min(canvas.width / g.width, canvas.height / g.height);
  return { g, scale };
}

function worldToCanvas(v, wx, wy) {
  const mx = (wx - v.g.origin.x) / v.g.resolution;
  const my = (wy - v.g.origin.y) / v.g.resolution;
  return [mx * v.scale, (v.g.height - my) * v.scale];
}

function canvasToWorld(cx, cy) {
  const v = view();
  if (!v) return null;
  const mx = cx / v.scale;
  const my = v.g.height - cy / v.scale;
  return { x: v.g.origin.x + mx * v.g.resolution, y: v.g.origin.y + my * v.g.resolution };
}

function eventToCanvas(evt) {
  // CSS 로 축소된 canvas 도 정확히 (원본은 1:1 크기만 가정)
  const rect = canvas.getBoundingClientRect();
  return {
    x: (evt.clientX - rect.left) * (canvas.width / rect.width),
    y: (evt.clientY - rect.top) * (canvas.height / rect.height),
  };
}

function drawOverlay(v, layer) {
  const m = layer.meta;
  // grid 왼쪽 위 모서리 = (origin.x, origin.y + height*res)
  const [x0, y0] = worldToCanvas(v, m.origin.x, m.origin.y + m.height * m.resolution);
  const w = (m.width * m.resolution / v.g.resolution) * v.scale;
  const h = (m.height * m.resolution / v.g.resolution) * v.scale;
  ctx.drawImage(layer.image, x0, y0, w, h);
}

function drawState() {
  ctx.fillStyle = "black";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const v = view();
  if (!v || !grids.map.image) {
    ctx.fillStyle = "#9ca3af";
    ctx.font = "16px system-ui";
    ctx.fillText("지도 대기 중 (/map)...", 24, 40);
    return;
  }
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(grids.map.image, 0, 0, v.g.width * v.scale, v.g.height * v.scale);

  if (showGlobalCostmap && grids.global) drawOverlay(v, grids.global);
  // local costmap 은 odom frame 이지만 원본과 같이 map 좌표로 그린다(근사)
  if (showLocalCostmap && grids.local) drawOverlay(v, grids.local);

  const path = (latestState && latestState.path) || [];
  if (path.length > 0) {
    ctx.strokeStyle = "#22c55e";
    ctx.lineWidth = 2;
    ctx.beginPath();
    path.forEach((pt, i) => {
      const [px, py] = worldToCanvas(v, pt.x, pt.y);
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }

  const pose = latestState && latestState.pose;
  if (pose) {
    const [rx, ry] = worldToCanvas(v, pose.x, pose.y);
    ctx.fillStyle = "#f97316";
    ctx.beginPath();
    ctx.arc(rx, ry, 6, 0, 2 * Math.PI);
    ctx.fill();
    ctx.strokeStyle = "#fde68a";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(rx + 20 * Math.cos(-pose.yaw), ry + 20 * Math.sin(-pose.yaw));
    ctx.stroke();
  }

  if (isDraggingGoal && goalStartWorld && goalCurrentCanvas) {
    const [sx, sy] = worldToCanvas(v, goalStartWorld.x, goalStartWorld.y);
    const color = (clickMode === "initialpose") ? "#22c55e" : "#fb7185";
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(goalCurrentCanvas.x, goalCurrentCanvas.y);
    ctx.stroke();
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(goalCurrentCanvas.x, goalCurrentCanvas.y, 4, 0, 2 * Math.PI);
    ctx.fill();
  }
}

// ------------------------------------------------------------
// goal / initial pose: 클릭 + 드래그 (원본 기능)
// ------------------------------------------------------------
function resetDrag() {
  isDraggingGoal = false;
  goalStartWorld = null;
  goalCurrentCanvas = null;
}

canvas.addEventListener("pointerdown", (evt) => {
  const c = eventToCanvas(evt);
  const w = canvasToWorld(c.x, c.y);
  if (!w) return;
  canvas.setPointerCapture(evt.pointerId);
  isDraggingGoal = true;
  goalStartWorld = w;
  goalCurrentCanvas = c;
  drawState();
});

canvas.addEventListener("pointermove", (evt) => {
  if (!isDraggingGoal) return;
  goalCurrentCanvas = eventToCanvas(evt);
  drawState();
});

canvas.addEventListener("pointerup", async (evt) => {
  if (canvas.hasPointerCapture(evt.pointerId)) canvas.releasePointerCapture(evt.pointerId);
  if (!isDraggingGoal || !goalStartWorld) { resetDrag(); return; }
  const c = eventToCanvas(evt);
  const endWorld = canvasToWorld(c.x, c.y);
  const start = goalStartWorld;
  resetDrag();
  drawState();
  if (!endWorld) return;

  const yaw = Math.atan2(endWorld.y - start.y, endWorld.x - start.x);
  const payload = { x: start.x, y: start.y, yaw };
  const isInit = clickMode === "initialpose";
  try {
    const d = await postJson(isInit ? "/api/initialpose" : "/api/goal", payload);
    if (!d.success && !isInit) {
      setMsg($("navStopStatus"), "목표 전송 실패: Nav2(navigate_to_pose)가 실행 중인가요? (mode:=nav)", "err");
    } else {
      setMsg($("navStopStatus"), isInit ? "초기 위치를 설정했습니다." : "목표를 전송했습니다.", "ok");
    }
    // 초기 위치는 한 번 설정 후 실수 방지를 위해 목적지 모드로 자동 복귀
    if (isInit) setClickMode("goal");
  } catch (e) {
    setMsg($("navStopStatus"), "요청 오류", "err");
  }
});

canvas.addEventListener("pointercancel", () => { resetDrag(); drawState(); });

$("toggleGlobal").addEventListener("change", (e) => {
  showGlobalCostmap = e.target.checked;
  if (!showGlobalCostmap) grids.global = null;
  drawState();
});
$("toggleLocal").addEventListener("change", (e) => {
  showLocalCostmap = e.target.checked;
  if (!showLocalCostmap) grids.local = null;
  drawState();
});

// ------------------------------------------------------------
// 시작
// ------------------------------------------------------------
async function init() {
  try {
    const res = await fetch("/api/info");
    info = await res.json();
  } catch (_) { /* 기본값 사용 */ }
  $("modeChip").textContent = "mode: " + info.mode;
  $("teleopHint").textContent =
    `누르고 있는 동안 / WASD·방향키 (최대 ${info.max_linear} m/s, ${info.max_angular} rad/s). ` +
    `손을 떼면 정지, 입력이 ${info.deadman_sec} s 끊기면 서버가 자동 정지.`;
  // 현재 모드에서 쓰지 않는 카드는 흐리게
  $("navCard").classList.toggle("dim", info.mode === "slam");
  $("slamCard").classList.toggle("dim", info.mode === "nav");
  if (info.mode === "slam") {
    $("mapHint").textContent = "SLAM 모드: 조종 패드/키보드로 천천히 돌며 지도를 만든 뒤 Save Map.";
  }
  refreshMaps();
}

init();
drawState();
setInterval(fetchState, 500);
setInterval(fetchStatus, 1000);
setInterval(() => markConnection(false), 1000);
fetchState();
fetchStatus();
