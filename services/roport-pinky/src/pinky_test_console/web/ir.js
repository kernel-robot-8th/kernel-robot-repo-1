// pinky_test_console IR 측정 화면. /api/ir 를 0.25 s 마다 읽어 값·그래프를 그리고,
// /api/ir/record 로 시간 지정 기록을 시작한다.
"use strict";

const COLORS = ["#ff9ad4", "#7a9be8", "#4ade80"];
const POLL_MS = 250;
const ADC_MAX = 4095;

const $ = (id) => document.getElementById(id);
let wasRecording = false;

async function postJson(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  let data = {};
  try { data = await res.json(); } catch (_) { /* 빈 응답 */ }
  data.httpStatus = res.status;
  return data;
}

function setMsg(el, text, kind) {
  el.classList.remove("ok", "err");
  if (kind) el.classList.add(kind);
  el.textContent = text;
}

function markConnection(ok) {
  $("connDot").className = "chip-dot " + (ok ? "live" : "dead");
  $("connText").textContent = ok ? "연결됨" : "연결 끊김";
}

// ---------------- 실시간 값·그래프 ----------------
function renderLive(s) {
  const stale = s.age_sec === null || s.age_sec > 1.0;
  for (let c = 0; c < 3; c++) {
    const v = s.latest ? s.latest[c] : null;
    $("v" + c).textContent = v === null ? "-" : v;
    $("b" + c).style.width = v === null ? "0%" : (100 * v / ADC_MAX).toFixed(1) + "%";
  }
  $("rateChip").textContent = s.hz === null ? "- Hz" : s.hz + " Hz";
  $("ageChip").textContent = s.age_sec === null ? "값 없음" : (stale ? "멈춤 " : "") + s.age_sec + " s 전";
  $("ageChip").classList.toggle("ir-stale", stale);
  $("noData").hidden = !stale;
  drawChart(s.history || []);
}

function drawChart(hist) {
  const cv = $("chart");
  const ctx = cv.getContext("2d");
  const w = cv.width, h = cv.height, pad = 28;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = "rgba(255,255,255,0.12)";
  ctx.fillStyle = "#9ca3af";
  ctx.font = "11px system-ui";
  for (const v of [0, 1024, 2048, 3072, 4095]) {
    const y = h - pad - (h - 2 * pad) * v / ADC_MAX;
    ctx.beginPath(); ctx.moveTo(pad + 20, y); ctx.lineTo(w - 4, y); ctx.stroke();
    ctx.fillText(String(v), 2, y + 4);
  }
  if (hist.length < 2) return;
  const x = (t) => pad + 20 + (w - pad - 24) * (1 + t / 10.0);   // t: -10 ~ 0 s
  for (let c = 0; c < 3; c++) {
    ctx.strokeStyle = COLORS[c];
    ctx.lineWidth = 2;
    ctx.beginPath();
    hist.forEach((r, i) => {
      const px = x(r[0]), py = h - pad - (h - 2 * pad) * r[c + 1] / ADC_MAX;
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }
}

// ---------------- 기록 ----------------
function renderRecording(s) {
  const rec = s.recording;
  $("btnRecord").disabled = !!rec;
  $("btnCancel").disabled = !rec;
  const bar = $("recBar").firstElementChild;
  if (rec) {
    bar.style.width = Math.min(100, 100 * rec.elapsed / rec.duration).toFixed(0) + "%";
    setMsg($("recMsg"), `기록 중: ${rec.label} ${rec.elapsed}/${rec.duration} s, ${rec.samples}개`);
  } else {
    bar.style.width = "0%";
    if (wasRecording && s.last_result) {
      const r = s.last_result;
      if (r.success) {
        setMsg($("recMsg"), `저장: ${r.file} (${r.stats.n}개)`, "ok");
      } else {
        setMsg($("recMsg"), `실패: ${r.msg}`, "err");
      }
      refreshRecords();
    }
  }
  wasRecording = !!rec;
}

async function startRecord() {
  const label = $("label").value.trim();
  const duration = Number($("duration").value);
  try {
    const r = await postJson("/api/ir/record", { label, duration });
    if (!r.success) setMsg($("recMsg"), r.msg || `오류 (${r.httpStatus})`, "err");
  } catch (e) {
    setMsg($("recMsg"), "요청 실패: " + e, "err");
  }
}

async function cancelRecord() {
  await postJson("/api/ir/cancel", {});
  setMsg($("recMsg"), "취소했습니다.");
}

// ---------------- 기록 목록·임계값 ----------------
function cell(text) {
  const td = document.createElement("td");
  td.textContent = text;
  return td;
}

function fmtCh(ch) {
  return ch ? `${ch.mean} ± ${ch.std}` : "-";
}

function renderSuggestion(sg) {
  const box = $("suggest");
  if (!sg) return;
  box.textContent = "";
  const t = document.createElement("table");
  t.className = "ir-table";
  const head = t.createTHead().insertRow();
  for (const k of ["", "carpet", "tape", "임계값", "분리도"]) {
    const th = document.createElement("th"); th.textContent = k; head.appendChild(th);
  }
  const body = t.createTBody();
  sg.channels.forEach((c, i) => {
    const tr = body.insertRow();
    tr.appendChild(cell(`ir[${i}]`));
    tr.appendChild(cell(c.carpet_mean));
    tr.appendChild(cell(c.tape_mean));
    tr.appendChild(cell(`${c.threshold} (테이프 ${c.tape_is_higher ? "↑" : "↓"})`));
    tr.appendChild(cell(c.separation === null ? "-" : c.separation));
  });
  box.appendChild(t);
  const note = document.createElement("div");
  note.className = "tiny-note";
  note.textContent = `carpet ${sg.carpet_records}건, tape ${sg.tape_records}건 평균. ` +
    "분리도 = |차이| / (표준편차 합), 3 이상이면 구분이 쉽습니다. 참고값이며 확정값이 아닙니다.";
  box.appendChild(note);
}

async function refreshRecords() {
  try {
    const res = await fetch("/api/ir/records");
    const data = await res.json();
    $("irDir").textContent = "저장 위치(로봇): " + data.ir_dir;
    const tbody = $("records").tBodies[0];
    tbody.textContent = "";
    for (const r of data.records) {
      const tr = tbody.insertRow();
      tr.appendChild(cell(r.label));
      tr.appendChild(cell(r.stats.n));
      for (let c = 0; c < 3; c++) tr.appendChild(cell(fmtCh(r.stats.channels[c])));
      const td = document.createElement("td");
      const a = document.createElement("a");
      a.href = "/api/ir/records/" + encodeURIComponent(r.file);
      a.textContent = "↓";
      a.title = r.file;
      td.appendChild(a);
      tr.appendChild(td);
    }
    renderSuggestion(data.suggestion);
  } catch (e) {
    $("irDir").textContent = "목록을 읽지 못했습니다: " + e;
  }
}

// ---------------- 주기 갱신 ----------------
async function poll() {
  try {
    const res = await fetch("/api/ir");
    const s = await res.json();
    markConnection(true);
    renderLive(s);
    renderRecording(s);
  } catch (_) {
    markConnection(false);
  } finally {
    setTimeout(poll, POLL_MS);
  }
}

$("presets").addEventListener("click", (e) => {
  const label = e.target.dataset && e.target.dataset.label;
  if (label) $("label").value = label;
});
$("btnRecord").addEventListener("click", startRecord);
$("btnCancel").addEventListener("click", cancelRecord);
$("btnRefresh").addEventListener("click", refreshRecords);
refreshRecords();
poll();
