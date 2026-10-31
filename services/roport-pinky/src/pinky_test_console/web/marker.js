// pinky_test_console 마커 테스트 화면. /api/marker 를 0.3 s, 영상을 약 5 fps 로 갱신한다.
"use strict";

const POLL_MS = 300;
const FRAME_MS = 200;
const $ = (id) => document.getElementById(id);

function markConnection(ok) {
  $("connDot").className = "chip-dot " + (ok ? "live" : "dead");
  $("connText").textContent = ok ? "연결됨" : "연결 끊김";
}

function cell(text) {
  const td = document.createElement("td");
  td.textContent = text === null || text === undefined ? "-" : text;
  return td;
}

function renderDetections(s) {
  const tbody = $("dets").tBodies[0];
  tbody.textContent = "";
  for (const d of s.detections) {
    const tr = tbody.insertRow();
    tr.appendChild(cell(d.id));
    tr.appendChild(cell(d.name || "표에 없음"));
    tr.appendChild(cell(d.size_mm ? d.size_mm + " mm" : null));
    tr.appendChild(cell(d.side_px + " px"));
    tr.appendChild(cell(d.distance_m === null ? null : d.distance_m + " m"));
  }
  if (!s.detections.length) {
    const tr = tbody.insertRow();
    const td = cell("보이는 마커 없음");
    td.colSpan = 5;
    tr.appendChild(td);
  }
  $("distNote").textContent = s.hfov_deg > 0
    ? `추정 거리는 화각 ${s.hfov_deg}° 가정의 핀홀 근사입니다(카메라 보정 전 참고값). 줄자 측정값을 기준으로 하세요.`
    : "추정 거리는 camera_hfov_deg 를 주면 표시합니다(보정 전 참고값).";
}

function renderChecklist(s) {
  const tbody = $("checklist").tBodies[0];
  tbody.textContent = "";
  let seen = 0, total = 0;
  for (const c of s.checklist) {
    if (c.in_table) { total++; if (c.seen) seen++; }
    const tr = tbody.insertRow();
    tr.className = c.seen ? "mk-seen" : "";
    tr.appendChild(cell(c.id));
    tr.appendChild(cell(c.name || "표에 없음"));
    tr.appendChild(cell(c.seen ? "✓" : ""));
    tr.appendChild(cell(Math.round(c.rate * 100) + "%"));
    tr.appendChild(cell(c.ago_sec === null ? null : c.ago_sec + " s 전"));
  }
  $("summary").textContent = total ? `표의 마커 ${total}개 중 ${seen}개 확인` : "마커 표 없음 (본 ID만 표시)";
}

async function poll() {
  try {
    const res = await fetch("/api/marker");
    const s = await res.json();
    markConnection(true);
    $("fpsChip").textContent = s.fps === null ? "- fps" : s.fps + " fps";
    $("camError").textContent = s.error ? "카메라: " + s.error : "";
    renderDetections(s);
    renderChecklist(s);
    renderRecording(s);
  } catch (_) {
    markConnection(false);
  } finally {
    setTimeout(poll, POLL_MS);
  }
}

// 영상: 이전 그림이 다 받아진 뒤에 다음 그림을 요청한다(느린 Wi-Fi 에서 요청이 쌓이지 않게).
// 첫 요청도 여기서 시작한다. HTML 의 src 로 시작하면 스크립트가 붙기 전에 load/error 가 끝나
// 다음 요청이 영영 시작되지 않을 수 있다. 응답이 2 s 넘게 없으면 다시 요청한다.
const FRAME_STALL_MS = 2000;
let frameTimer = null;

function requestFrame() {
  clearTimeout(frameTimer);
  $("frame").src = "/api/marker/frame.jpg?t=" + Date.now();
  frameTimer = setTimeout(requestFrame, FRAME_STALL_MS);
}

function nextFrame() {
  clearTimeout(frameTimer);
  frameTimer = setTimeout(requestFrame, FRAME_MS);
}
$("frame").addEventListener("load", nextFrame);
$("frame").addEventListener("error", nextFrame);
requestFrame();

// ---------------- 거리별 기록 ----------------
let wasRecording = false;

function renderRecording(s) {
  const rec = s.recording;
  $("btnRecord").disabled = !!rec;
  const bar = $("recBar").firstElementChild;
  if (rec) {
    bar.style.width = Math.min(100, 100 * rec.elapsed / rec.duration).toFixed(0) + "%";
    $("recMsg").textContent = `기록 중: ${rec.distance_cm} cm, ${rec.elapsed}/${rec.duration} s, ${rec.frames}프레임`;
  } else {
    bar.style.width = "0%";
    if (wasRecording && s.last_record) {
      const r = s.last_record;
      $("recMsg").textContent = r.success
        ? `저장: ${r.distance_cm} cm · ID ${r.marker_id ?? "-"} · 인식률 ${Math.round(r.rate * 100)}%`
        : `실패: ${r.msg}`;
      refreshRanges();
    }
  }
  wasRecording = !!rec;
}

async function startRecord() {
  const body = { distance_cm: Number($("distance").value), duration: Number($("recDuration").value),
    note: $("note").value.trim() };
  const id = $("markerId").value.trim();
  if (id !== "") body.marker_id = Number(id);
  try {
    const res = await fetch("/api/marker/record", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
    });
    const r = await res.json();
    if (!r.success) $("recMsg").textContent = "오류: " + (r.msg || res.status);
  } catch (e) {
    $("recMsg").textContent = "요청 실패: " + e;
  }
}

async function refreshRanges() {
  try {
    const data = await (await fetch("/api/marker/records")).json();
    const tbody = $("ranges").tBodies[0];
    tbody.textContent = "";
    for (const r of data.records.slice().reverse()) {
      const tr = tbody.insertRow();
      tr.appendChild(cell(r.distance_cm + " cm"));
      tr.appendChild(cell(r.marker_id === "" ? "-" : `${r.marker_id} ${r.name || ""}`));
      tr.appendChild(cell(Math.round(Number(r.rate) * 100) + "%"));
      tr.appendChild(cell(r.side_px_med === "" ? "-" : `${r.side_px_med} (${r.side_px_min}~${r.side_px_max})`));
      tr.appendChild(cell(r.top_px === "" ? "-" : `${r.top_px} · ${r.bottom_px} / ${r.frame_h}`));
      tr.appendChild(cell(r.pred_50mm_px === "" ? "-" : r.pred_50mm_px + " px"));
      tr.title = r.note ? `${r.time} · ${r.note}` : r.time;
    }
  } catch (_) { /* 다음 갱신 때 다시 */ }
}

$("btnRecord").addEventListener("click", startRecord);
refreshRanges();

$("btnReset").addEventListener("click", async () => {
  await fetch("/api/marker/reset", {
    method: "POST", headers: { "Content-Type": "application/json" }, body: "{}",
  });
});
poll();
