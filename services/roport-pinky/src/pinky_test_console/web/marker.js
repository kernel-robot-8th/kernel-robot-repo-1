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
  } catch (_) {
    markConnection(false);
  } finally {
    setTimeout(poll, POLL_MS);
  }
}

// 영상: 이전 그림이 다 받아진 뒤에 다음 그림을 요청한다(느린 Wi-Fi 에서 요청이 쌓이지 않게)
function nextFrame() {
  setTimeout(() => { $("frame").src = "/api/marker/frame.jpg?t=" + Date.now(); }, FRAME_MS);
}
$("frame").addEventListener("load", nextFrame);
$("frame").addEventListener("error", nextFrame);

$("btnReset").addEventListener("click", async () => {
  await fetch("/api/marker/reset", {
    method: "POST", headers: { "Content-Type": "application/json" }, body: "{}",
  });
});
poll();
