// pinky_test_console AMCL 정합 확인 화면. /api/amcl 을 0.5 s 로 갱신한다.
"use strict";

const POLL_MS = 500;
const PASS_WITHIN_10 = 0.8;   // 제안 합격 기준: 스캔 점 80% 이상이 벽 10 cm 안
const $ = (id) => document.getElementById(id);
const canvas = $("amclCanvas");
const ctx = canvas.getContext("2d");

let mapGrid = null;    // { version, width, height, resolution, origin, image }
let last = null;

function markConnection(ok) {
  $("connDot").className = "chip-dot " + (ok ? "live" : "dead");
  $("connText").textContent = ok ? "연결됨" : "연결 끊김";
}

function setMsg(el, text, kind) {
  el.textContent = text || "";
  el.className = "status-msg" + (kind ? " " + kind : "");
}

async function postJson(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok || data.success === false) throw new Error(data.msg || res.status);
  return data;
}

// ---------------- 지도 그리기 ----------------
function buildMapImage(g) {
  const off = document.createElement("canvas");
  off.width = g.width;
  off.height = g.height;
  const octx = off.getContext("2d");
  const img = octx.createImageData(g.width, g.height);
  for (let iy = 0; iy < g.height; iy++) {
    const row = g.height - 1 - iy;
    for (let ix = 0; ix < g.width; ix++) {
      const v = g.data[iy * g.width + ix];
      const c = v < 0 ? [75, 85, 99] : v >= 65 ? [0, 0, 0] : [255, 255, 255];
      const o = (row * g.width + ix) * 4;
      img.data[o] = c[0]; img.data[o + 1] = c[1]; img.data[o + 2] = c[2]; img.data[o + 3] = 255;
    }
  }
  octx.putImageData(img, 0, 0);
  return off;
}

function view() {
  if (!mapGrid) return null;
  const scale = Math.min(canvas.width / mapGrid.width, canvas.height / mapGrid.height);
  return { scale, res: mapGrid.resolution, ox: mapGrid.origin.x, oy: mapGrid.origin.y, h: mapGrid.height };
}

function toCanvas(v, x, y) {
  return [(x - v.ox) / v.res * v.scale, (v.h - (y - v.oy) / v.res) * v.scale];
}

function draw(s) {
  ctx.fillStyle = "#111827";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const v = view();
  if (!v) {
    ctx.fillStyle = "#9ca3af";
    ctx.font = "16px sans-serif";
    ctx.fillText("지도 대기 중 (/map, map_server)...", 24, 40);
    return;
  }
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapGrid.image, 0, 0, mapGrid.width * v.scale, mapGrid.height * v.scale);

  // 노드
  for (const n of s.nodes || []) {
    const [cx, cy] = toCanvas(v, n.map_x, n.map_y);
    ctx.fillStyle = n.nav2_switch ? "#2563eb" : "#f97316";
    ctx.beginPath();
    ctx.arc(cx, cy, n.nav2_switch ? 6 : 3.5, 0, 2 * Math.PI);
    ctx.fill();
    if (n.nav2_switch || n.place) {
      ctx.font = "11px sans-serif";
      ctx.fillText(n.id, cx + 6, cy - 6);
    }
  }

  // 스캔 점: 벽까지 거리로 색 (0 = 5 cm 안, 1 = 10 cm 안, 2 = 밖)
  const COLORS = ["#22c55e", "#eab308", "#ef4444"];
  if (s.scan && s.scan.points) {
    s.scan.points.forEach(([x, y], i) => {
      const [cx, cy] = toCanvas(v, x, y);
      const k = s.scan.classes[i];
      ctx.fillStyle = k === undefined ? "#9ca3af" : COLORS[k];
      ctx.fillRect(cx - 2, cy - 2, 4, 4);
    });
  }

  // AMCL 위치
  if (s.amcl) {
    const [cx, cy] = toCanvas(v, s.amcl.x, s.amcl.y);
    const r = Math.max(4, Math.hypot(s.amcl.std[0], s.amcl.std[1]) / v.res * v.scale);
    ctx.strokeStyle = "rgba(37, 99, 235, 0.9)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + 18 * Math.cos(s.amcl.yaw), cy - 18 * Math.sin(s.amcl.yaw));
    ctx.stroke();
  }
}

// ---------------- 표시 ----------------
const pct = (x) => (x === null || x === undefined ? "-" : Math.round(x * 100) + "%");

function render(s) {
  if (s.map) {
    if (s.map.data) {
      mapGrid = Object.assign({}, s.map);
      mapGrid.image = buildMapImage(s.map);
    }
  }
  setMsg($("amclError"), s.error || (s.frame ? "" : "배치 yaml 없음(layout:=): 좌표는 map m × 100, 노드 버튼 없음"), s.error ? "err" : "");

  const sc = s.score;
  $("within5").textContent = sc ? pct(sc.within[0]) : "-";
  $("within10").textContent = sc ? pct(sc.within[1]) : "-";
  $("median").textContent = sc && sc.median_m !== null ? (sc.median_m * 100).toFixed(1) + " cm" : "-";
  $("points").textContent = sc ? sc.points : "-";
  const verdict = $("verdict");
  if (!sc || sc.within[1] === null) {
    verdict.textContent = s.scan ? "스캔 오래됨" : "스캔 없음";
  } else {
    verdict.textContent = sc.within[1] >= PASS_WITHIN_10 ? "합격 (≥80%)" : "확인 필요 (<80%)";
  }
  $("scanChip").textContent = s.scan ? `스캔 ${s.scan.age_sec}s 전` : "스캔 없음";

  const a = s.amcl;
  if (a) {
    const hasCm = a.x_cm !== undefined;
    $("amclXY").textContent = hasCm ? `${a.x_cm}, ${a.y_cm}` : `${(a.x * 100).toFixed(1)}, ${(a.y * 100).toFixed(1)} (map)`;
    $("amclYaw").textContent = hasCm ? `${a.yaw_deg}°` : `${(a.yaw * 180 / Math.PI).toFixed(1)}° (map)`;
    $("amclStd").textContent = `${(a.std[0] * 100).toFixed(1)}, ${(a.std[1] * 100).toFixed(1)} cm, ${(a.std[2] * 180 / Math.PI).toFixed(1)}°`;
    $("amclAge").textContent = `${a.age_sec} s 전`;
  } else {
    ["amclXY", "amclYaw", "amclStd", "amclAge"].forEach((id) => { $(id).textContent = "-"; });
  }
  renderNodes(s.nodes || []);
  renderRecords(s.records || []);
  draw(s);
}

let nodeKey = "";
function renderNodes(nodes) {
  const key = nodes.map((n) => n.id).join(",");
  if (key === nodeKey) return;
  nodeKey = key;
  const box = $("nodeButtons");
  box.textContent = "";
  // Nav2 전환 노드와 운영 위치(P) 노드만 버튼으로
  for (const n of nodes.filter((k) => k.nav2_switch || k.place)) {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "btn btn-outline btn-small" + (n.nav2_switch ? " am-switch" : "");
    b.textContent = n.id + (n.place ? ` ${n.place}` : "") + (n.nav2_switch ? " (Nav2)" : "");
    b.title = `${n.role} · x ${n.x_cm}, y ${n.y_cm} cm, ${n.yaw_deg}°`;
    b.addEventListener("click", () => setNodePose(n));
    box.appendChild(b);
  }
}

function cell(text) {
  const td = document.createElement("td");
  td.textContent = text === null || text === undefined || text === "" ? "-" : text;
  return td;
}

function renderRecords(rows) {
  const tbody = $("records").tBodies[0];
  tbody.textContent = "";
  for (const r of rows.slice().reverse()) {
    const tr = tbody.insertRow();
    tr.appendChild(cell(r.label));
    tr.appendChild(cell(r.err_xy_cm ? r.err_xy_cm + " cm" : ""));
    tr.appendChild(cell(r.err_yaw_deg ? r.err_yaw_deg + "°" : ""));
    tr.appendChild(cell(r.within_10cm !== "" ? pct(parseFloat(r.within_10cm)) : ""));
    tr.appendChild(cell(r.std_x_cm ? `${r.std_x_cm}/${r.std_y_cm}` : ""));
  }
}

// ---------------- 동작 ----------------
async function setNodePose(n) {
  $("truthX").value = n.x_cm;
  $("truthY").value = n.y_cm;
  $("truthYaw").value = n.yaw_deg;
  $("label").value = n.id;
  try {
    await postJson("/api/amcl/initialpose", { node: n.id });
    setMsg($("poseMsg"), `${n.id} 초기 위치 보냄 → 「제자리 갱신」을 몇 번 누르세요`, "ok");
  } catch (e) {
    setMsg($("poseMsg"), "초기 위치 실패: " + e.message, "err");
  }
}

function truthFromInputs(needYaw) {
  const x = parseFloat($("truthX").value);
  const y = parseFloat($("truthY").value);
  const yaw = parseFloat($("truthYaw").value);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  const out = { x_cm: x, y_cm: y };
  if (Number.isFinite(yaw)) out.yaw_deg = yaw;
  else if (needYaw) out.yaw_deg = 0;
  return out;
}

$("btnInit").addEventListener("click", async () => {
  const t = truthFromInputs(true);
  if (!t) { setMsg($("poseMsg"), "x, y (cm)를 넣으세요", "err"); return; }
  try {
    await postJson("/api/amcl/initialpose", t);
    setMsg($("poseMsg"), "초기 위치 보냄", "ok");
  } catch (e) {
    setMsg($("poseMsg"), "초기 위치 실패: " + e.message, "err");
  }
});

$("btnNomotion").addEventListener("click", async () => {
  try {
    await postJson("/api/amcl/nomotion");
    setMsg($("poseMsg"), "제자리 갱신 요청", "ok");
  } catch (e) {
    setMsg($("poseMsg"), "제자리 갱신 실패(amcl 실행 중?): " + e.message, "err");
  }
});

$("btnRecord").addEventListener("click", async () => {
  const body = { label: $("label").value.trim() };
  const t = truthFromInputs(false);
  if (t) body.truth = t;
  try {
    const d = await postJson("/api/amcl/record", body);
    const r = d.row;
    setMsg($("recMsg"), r.err_xy_cm !== undefined
      ? `기록: 오차 ${r.err_xy_cm} cm, 10 cm 안 ${pct(r.within_10cm)}`
      : `기록: AMCL ${r.amcl_x_cm}, ${r.amcl_y_cm} cm`, "ok");
  } catch (e) {
    setMsg($("recMsg"), "기록 실패: " + e.message, "err");
  }
});

async function estop() {
  try {
    await postJson("/api/estop");
    setMsg($("estopStatus"), "정지 보냄", "ok");
  } catch (e) {
    setMsg($("estopStatus"), "정지 실패: " + e.message, "err");
  }
}
$("btnEstop").addEventListener("click", estop);
document.addEventListener("keydown", (e) => {
  if ((e.key === " " || e.key === "Escape") && !["INPUT", "TEXTAREA"].includes(document.activeElement.tagName)) {
    e.preventDefault();
    estop();
  }
});

async function poll() {
  try {
    const v = mapGrid ? mapGrid.version : -1;
    const res = await fetch(`/api/amcl?map_v=${v}`);
    if (!res.ok) throw new Error(res.status);
    last = await res.json();
    markConnection(true);
    render(last);
  } catch (e) {
    markConnection(false);
  }
  setTimeout(poll, POLL_MS);
}
poll();
