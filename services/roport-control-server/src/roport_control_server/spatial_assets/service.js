import { send, acceptServicePlan, hasUnappliedChanges } from './scenario.js';
const $ = id => document.getElementById(id);
let state, connected, token, preview, busy = false, dirty = false, requestId = crypto.randomUUID();
export function hasServiceDraft() { return dirty; }
function refresh() {
  $('service-fields').disabled = !connected || !token || busy || !state?.can_configure || !!(state?.service_request && state.phase !== 'COMPLETE');
  $('service-apply').disabled = !preview || busy;
}
function changed() {
  preview = null; dirty = true; requestId = crypto.randomUUID();
  $('service-preview-steps').replaceChildren(); $('service-field-confirm').checked = false;
  refresh(); document.dispatchEvent(new Event('service-draft-change'));
}
for (const id of ['service-operation', 'service-source-confirm']) $(id).addEventListener('change', changed);
document.addEventListener('scenario-draft-change', () => {
  if (hasUnappliedChanges()) { preview = null; dirty = false; refresh(); }
});
function request() {
  const operation = $('service-operation').value;
  return {request_id: requestId, operation, package_id: state.package_id, quantity: 1,
    destination: operation === 'inbound' ? 'shelf' : 'handover', source_confirmed: $('service-source-confirm').checked};
}
async function perform(fn) {
  busy = true; refresh();
  try { await fn(); }
  catch (e) { $('service-feedback').textContent = e.name === 'TimeoutError' ? '응답 미확인 · 현재 작업을 확인하세요. 같은 요청 재시도는 새 작업을 만들지 않습니다.' : e.message; }
  finally { busy = false; refresh(); }
}
$('service-preview-button').addEventListener('click', () => perform(async () => {
  preview = null; dirty = true; document.dispatchEvent(new Event('service-draft-change'));
  const intent = request();
  for (let i = 0; i < 5; i++) {
    const result = await send('service_preview', {request: intent});
    if (!result.pending) { preview = result; break; }
    $('service-feedback').textContent = '배정할 기체의 현재 상태 확인 중…';
    await new Promise(resolve => setTimeout(resolve, 700));
  }
  if (!preview) throw new Error('기체 상태를 아직 받지 못했습니다. 연결을 확인하고 계획 확인을 다시 누르세요.');
  $('service-preview-steps').replaceChildren(...preview.plan.steps.map(s => {
    const li = document.createElement('li'); li.textContent = `${s.label} · ${state.catalogue.robots[s.role].robot_id}`; return li;
  }));
  $('service-feedback').textContent = '계획 확인 완료 · 적용 후 작업 실행 버튼을 누르세요.';
}));
$('service-apply').addEventListener('click', () => perform(async () => {
  const result = await send('service_submit', {request: preview.request, preview_id: preview.preview_id, field_confirmed: $('service-field-confirm').checked});
  if (result.plan) acceptServicePlan(result.plan, result.session_id);
  dirty = false; document.dispatchEvent(new Event('service-draft-change'));
  $('service-feedback').textContent = '서비스 요청 접수 · 상태 갱신 후 작업 실행을 누르세요.';
}));
export function renderService(next, online, key) {
  const phaseChanged = state?.phase !== next?.phase || state?.task_id !== next?.task_id;
  if (state && next?.session_id !== state.session_id) {
    preview = null; dirty = false; requestId = crypto.randomUUID();
    $('service-source-confirm').checked = false; $('service-field-confirm').checked = false;
    $('service-preview-steps').replaceChildren();
    document.dispatchEvent(new Event('service-draft-change'));
  }
  state = next; connected = online; token = key;
  if (phaseChanged && state?.service_request && !busy) {
    $('service-feedback').textContent = state.phase === 'COMPLETE' ? '작업 시험 완료 · 다음 서비스 요청을 선택할 수 있습니다.'
      : state.phase === 'READY' ? '서비스 요청 접수 · 작업 실행을 누르세요.'
      : state.phase === 'HOLD' ? '작업 보류 · 원인을 확인하고 현장 상태를 정리하세요.' : '서비스 작업 진행 중';
  }
  const operations = state?.catalogue?.service?.operations ?? {};
  if ($('service-operation').dataset.options !== JSON.stringify(operations)) {
    $('service-operation').replaceChildren(...Object.entries(operations).map(([value, name]) => new Option(name, value)));
    $('service-operation').dataset.options = JSON.stringify(operations);
  }
  const operation = $('service-operation').value;
  $('service-input-summary').textContent = state ? `물품: ${state.package_id} · 1개 / ${operation === 'inbound' ? '창고 → 인도장 선반' : '인도장 선반 → ' + (state.catalogue?.service?.handover_scope ?? '인도 위치')}` : '실행부 연결 대기';
  $('service-progress').replaceChildren(...(state?.service_request ? state.plan.steps : []).map((s, i) => {
    const li = document.createElement('li');
    li.textContent = `${i < state.plan_index ? '완료' : i === state.plan_index && state.phase !== 'READY' ? '현재' : '대기'} · ${s.label}`;
    return li;
  }));
  const completed = state?.transfers?.at(-1);
  const omx = state?.omx;
  $('service-completion').textContent = [
    state?.service_request && state.phase === 'COMPLETE' ? (state.service_request.operation === 'outbound' ? '인도 준비 시험 완료 · 고객 수령 확인은 별도입니다.' : '입고·복귀 시험 완료 · 재고 확정은 별도입니다.') : '',
    omx?.enabled ? `OMX ${omx.execution_mode === 'mock' ? '모의 실행' : '실물 실행 설정'} · ${omx.phase}` : 'OMX MQTT 미설정 · 작업 완료를 수동으로 확인하세요.',
    omx?.manual_error ?? '',
    completed ? `최근 완료 입력: ${completed.completion_source === 'mqtt' ? 'MQTT' : 'UI 수동'} · ${completed.evidence === 'arm_return_only' ? '팔 복귀 확인 / 물품 위치 미확정' : '운영자 확인'}` : '',
  ].filter(Boolean).join(' / ');
  refresh();
}
