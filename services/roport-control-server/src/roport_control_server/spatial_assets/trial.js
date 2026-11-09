import { renderComposer, hasUnappliedChanges } from "./scenario.js";
import { renderService, hasServiceDraft } from "./service.js";
// Trial actions use the server-owned state machine; this page never publishes MQTT.
const panel = document.getElementById("trial-panel");
const key = document.getElementById("trial-key");
const feedback = document.getElementById("trial-feedback");
const buttons = [...panel.querySelectorAll("button[data-action]")];
let state, connected = false, pending = false;

function transportBlocked() {
  return state?.mode === 'hardware' && location.protocol !== 'https:' &&
    !['localhost', '127.0.0.1', '[::1]'].includes(location.hostname);
}
function enable() {
  const blocked = transportBlocked();
  key.disabled = blocked;
  if (blocked) key.value = '';
  for (const button of buttons) {
    button.disabled = blocked || !connected || pending || state?.reset_pending || !key.value ||
      (button.dataset.action === 'start' && (hasUnappliedChanges() || hasServiceDraft())) ||
      !state?.[`can_${button.dataset.action}`];
  }
  const reasons = {
    "robot status unavailable/stale": "핑키 상태가 없거나 오래되었습니다.",
    "robot is not safely waiting": "정지 상태·라인 제어 해제·로봇 오류를 확인하세요.",
    "unexpected route start": "출발 노드와 line 모드를 확인하세요.",
    "MQTT subscriptions not ready": "MQTT 연결 대기 중입니다.",
    "logistics is not at warehouse": "물류 핑키의 창고 복귀 위치를 확인하세요.",
  };
  const error = state?.start_error ?? state?.request_error;
  document.getElementById("trial-readiness").textContent = blocked ? "실물 조작은 로컬 주소 또는 HTTPS에서만 가능합니다."
    : !connected ? "실행부 연결 대기 · 조작 비활성"
    : state?.phase === 'READY' && (hasUnappliedChanges() || hasServiceDraft()) ? "편집한 시나리오를 먼저 실행 계획에 적용하세요."
    : error ? reasons[error] ?? error
    : !key.value ? "조작 키를 입력하면 현재 단계에서 가능한 버튼이 활성화됩니다."
    : state?.can_start ? (state.service_request ? "출발 가능 · 작업 실행 버튼을 누르세요." : "출발 가능 · 시나리오 출발 버튼을 누르세요.")
    : "현재 단계에서 가능한 버튼만 활성화됩니다.";
}
key.addEventListener("input", () => { enable(); renderComposer(state, connected && !state?.reset_pending, key.value); renderService(state, connected && !state?.reset_pending, key.value); });
document.addEventListener('scenario-draft-change', enable);
document.addEventListener('service-draft-change', enable);

export function trialDisconnected() {
  connected = false;
  renderComposer(state, false, key.value); renderService(state, false, key.value);
  enable();
}

export function renderTrial(data) {
  state = data.trial;
  connected = !!state?.available;
  document.getElementById("trial-reset").hidden = !state?.reset_available;
  document.getElementById("trial-phase").textContent = state?.label ?? "실행부 미연결";
  document.getElementById("trial-connection").textContent =
    !state ? "실행부 미연결" : `${state?.mode === "hardware" ? "실물 시험" : "가상 시험"} · MQTT ${state?.mqtt_ready ? "연결" : "미연결"}`;
  const diagnostics = state?.motion_diagnostics;
  const motionStatus = document.getElementById("trial-motion-diagnostics");
  const rejectionLabels = {
    clock_skew_or_expired: "관측 시각 지연/시계 차이 · PC와 로봇 시계 동기화 확인",
    out_of_order: "중복·역순", identity_mismatch: "기체 식별 불일치", invalid_sample: "잘못된 위치 메시지",
  };
  motionStatus.hidden = !diagnostics?.rejected;
  motionStatus.textContent = diagnostics?.rejected
    ? `위치 수신 거부 누적 ${diagnostics.rejected}건 · ${Object.entries(diagnostics.reasons).map(([reason, count]) => `${rejectionLabels[reason] ?? reason} ${count}건`).join(" / ")}` : "";
  document.getElementById("trial-dummy").textContent = state?.phase === "WAIT_CART_LOAD"
    ? "OMX 카트 적재 완료 수동 확인" : state?.phase === "WAIT_LOAD"
    ? "OMX 창고 상차 완료 수동 확인" : "OMX 입고 완료 수동 확인";
  document.getElementById("trial-scenario").textContent = !state ? "시나리오 응답 대기"
    : state.custom_plan ? state.plan.name
    : state.smartcart_only ? "스마트카트 단독 시험 · 창고 작업·입고 완료 가정"
    : state.scenario === "single_pinky" ? "1대 시험 · 동일 핑키가 물류·카트 역할 수행"
    : "2대 시험 · 물류 핑키 복귀 후 스마트카트 진입";
  document.getElementById("trial-request").hidden = !state?.plan?.steps.some((s) => s.kind === "request");
  document.getElementById("trial-package").textContent = !state ? "" :
    `시험 물품 ${state.package_id} · 1개${state.request ? " · 요청 접수됨" : ""}`;
  document.getElementById("trial-scope").textContent = !state ? "시험 범위 응답 대기" : state.scope === "line-only"
    ? "시험 범위: N12 정차까지 · Nav2 전환/골 전송 제외" : "시험 범위: N12 → Nav2 전환 → 공용공간 목표";
  if (state?.custom_plan) {
    document.getElementById("trial-scope").textContent = `선택 계획 · ${state.plan.steps.length}단계 · ${Math.min(state.plan_index + 1, state.plan.steps.length)}번째 단계`;
    document.getElementById("trial-dummy").textContent = "현재 작업 완료 수동 확인";
  }
  const selected = data.robots.find((row) => row.robot_id === state?.active_robot)
    ?? data.robots.find((row) => row.status?.mode);
  const robot = selected?.status;
  document.getElementById("trial-status").textContent = robot
    ? `${selected.robot_id} · 노드 ${robot.node_id ?? "미확인"} · ${robot.mode} · 정지 ${robot.stopped ? "확인" : "아니오"} · 위치 추정 ${robot.localization_ready ? "준비" : "미준비"} · Nav2 ${robot.nav2_ready ? "준비" : "미준비"}`
    : "핑키 상태 응답 대기";
  document.getElementById("trial-route").textContent = state?.route?.length
    ? state.route.join(" → ") : "이동 명령 대기 · OMX 작업 완료 입력 대기";
  document.getElementById("trial-events").replaceChildren(
    ...(state?.events ?? []).slice(-5).reverse().map((event) => {
      const row = document.createElement("li");
      row.textContent = `${event.type} · ${event.source} · ${event.sent_at}`;
      return row;
    }),
  );
  document.getElementById("trial-slots").replaceChildren(
    ...data.robots.filter((row) => row.robot_id.startsWith("pinky")).map((row) => {
      const card = document.createElement("article");
      card.dataset.robotId = row.robot_id;
      const title = document.createElement("h3");
      title.textContent = `${row.robot_id}${row.role ? ` · ${row.role === "logistics" ? "물류" : "스마트카트"}` : ""}`;
      const status = document.createElement("p");
      status.textContent = `${row.mode}${row.status?.mode && row.stale ? " · 갱신 지연 / 마지막 수신" : ""}`;
      card.append(title, status);
      return card;
    }),
  );
  renderComposer(state, connected && !state?.reset_pending && !transportBlocked(), key.value);
  renderService(state, connected && !state?.reset_pending && !transportBlocked(), key.value);
  panel.querySelector('[data-action="start"]').textContent = state?.service_request ? "작업 실행" : "시나리오 출발";
  enable();
}

for (const button of buttons) button.addEventListener("click", async () => {
  if (button.disabled || !state) return;
  if (button.dataset.action === "reset" && !confirm("가상 로봇 위치·진행 상태·편집 계획을 실행 당시 초기값으로 되돌립니다. 이전 시험 기록은 보존합니다. 초기화할까요?")) return;
  const request = {
    action: button.dataset.action, session_id: state.session_id,
    task_id: state.task_id, phase: state.phase,
  };
  if (request.action === "dummy") request.stop_session_id = state.gate?.stop_session_id;
  if (request.action === "request") request.items = [{package_id: state.package_id, quantity: 1}];
  pending = true;
  enable();
  feedback.textContent = "요청 처리 중…";
  try {
    const response = await fetch("/api/trial/commands", {
      method: "POST", headers: {"Content-Type": "application/json", "X-Roport-Trial-Token": key.value},
      body: JSON.stringify(request), signal: AbortSignal.timeout(3000),
    });
    const result = await response.json();
    feedback.textContent = response.ok
      ? request.action === "reset" ? "초기화 요청 접수 · 잠시 후 같은 화면에서 새 시험이 연결됩니다." : "요청 접수 · 수신 상태에서 진행을 확인하세요."
      : result.detail;
  } catch {
    feedback.textContent = "응답 미확인 · 자동 재전송하지 않습니다. 현재 상태를 확인하세요.";
  } finally {
    pending = false;
    enable();
  }
});
