"use strict";
const el = (id) => document.getElementById(id);
let latest = null;
let pending = false;
function message(text, error = false) {
  el("message").textContent = text;
  el("message").className = "status-msg" + (error ? " err" : "");
}
function canApply() {
  const yaw = el("heading").value;
  return latest && latest.mode === "amcl" && latest.map_loaded && !latest.navigating &&
    latest.sensors.scan.fresh && latest.sensors.stopped &&
    latest.nodes.some((n) => n.id === el("initialNode").value) &&
    yaw !== "" && Number.isFinite(Number(yaw)) && Math.abs(Number(yaw)) <= 180 &&
    el("placement").checked && !pending;
}
function updateApply() { el("apply").disabled = !canApply(); }
async function post(url, body) {
  const response = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  const data = await response.json();
  if (!response.ok || data.success === false) throw new Error(data.msg || "요청이 거부되었습니다");
  return data;
}
async function poll() {
  try {
    const response = await fetch("/api/smartcart", { cache: "no-store" });
    if (!response.ok) throw new Error("상태 수신 실패");
    latest = await response.json();
    el("connection").textContent = "실시간 연결 · 0.5초 갱신";
    el("map").textContent = latest.map_loaded ? "지도 수신됨" : "지도 수신 대기";
    for (const topic of ["scan", "odom", "tf"]) {
      const status = latest.sensors[topic];
      el(topic).textContent = status.fresh ? `수신 중 · ${status.age_sec}s 전` : (status.age_sec === null ? "수신 없음" : `지연 · ${status.age_sec}s 전`);
    }
    el("stationary").textContent = latest.sensors.stopped ? "odom 연속 정지 확인" : "정지 미확인 / 이동 중";
    const pose = latest.amcl;
    el("amcl").textContent = pose ? `${latest.sensors.amcl.fresh ? "수신됨" : "갱신 지연"} · ${pose.age_sec}s 전` : "AMCL 위치 대기";
    el("position").textContent = pose ? `x ${pose.x.toFixed(3)} m · y ${pose.y.toFixed(3)} m · ${(pose.yaw * 180 / Math.PI).toFixed(1)}°` : "위치 추정 대기";
    el("navigation").textContent = latest.navigating ? "주행 goal 실행 중" : "활성 goal 관측 없음";
    el("deliveryReason").textContent = latest.delivery_reason;
    el("initialHint").textContent = latest.mode === "amcl" ? "방향 입력 · 실제 배치 확인 · 최신 센서 · 연속 정지가 필요합니다." : "AMCL 모드의 콘솔이 필요합니다. 이 페이지는 자동 실행하지 않습니다.";
  } catch (error) {
    latest = null;
    el("connection").textContent = "연결 끊김 · 상태를 확인하세요";
    for (const id of ["map", "scan", "odom", "tf", "amcl", "stationary", "position", "navigation"]) el(id).textContent = "연결 끊김 · 현재 상태 모름";
  }
  updateApply();
  setTimeout(poll, 500);
}
for (const id of ["heading", "placement", "initialNode"]) el(id).addEventListener("input", updateApply);
el("apply").addEventListener("click", async () => {
  if (!canApply()) return;
  pending = true; updateApply();
  try {
    await post("/api/smartcart/initialpose", { node: el("initialNode").value, yaw_deg: Number(el("heading").value), placement_confirmed: el("placement").checked });
    message("초기 위치를 전달했습니다. 지도와 라이다 정합을 확인하세요. 주행은 시작하지 않았습니다.");
    el("placement").checked = false;
  } catch (error) { message(error.message, true); }
  finally { pending = false; updateApply(); }
});
el("nomotion").addEventListener("click", async () => {
  try { await post("/api/amcl/nomotion", {}); message("제자리 관측 갱신을 요청했습니다."); }
  catch (error) { message(error.message, true); }
});
el("stop").addEventListener("click", async () => {
  try { await post("/api/estop", {}); message("정지를 요청했습니다. 실제 로봇이 멈췄는지 확인하세요."); }
  catch (error) { message(error.message, true); }
});
poll();
