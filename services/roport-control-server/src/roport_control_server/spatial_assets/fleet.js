// Requests go to the single owner; roles, shelf stock and tasks share its saved state.
const stages = {queued:'배차 대기', warehouse_route:'창고로 이동', warehouse_load:'창고 상차', choose_slot:'선반 배정 대기', shelf_route:'인도장으로 이동', transfer:'선반 상하차', return_route:'대기 장소로 복귀', switch_route:'N12로 이동', switch_nav:'Nav2 전환', navigate:'고객에게 이동', await_receipt:'고객 수령 대기', return_nav:'N12로 복귀', switch_line:'라인 주행 전환', done:'완료'};
const shelfStates = {empty:'빈 칸', occupied:'보관 중', reserved:'작업 예약'};
const $ = id => document.getElementById(id);
let state, online = false, busy = false, request;
function refresh() {
  const blocked = state?.mode === 'hardware' && location.protocol !== 'https:' && !['localhost', '127.0.0.1', '[::1]'].includes(location.hostname);
  $('fleet-fields').disabled = !online || busy || blocked || !$('fleet-key').value;
  $('fleet-enable').disabled = !online || busy || blocked || !$('fleet-key').value || !!state?.fault || state?.enabled;
  $('fleet-stop').disabled = !online || busy || blocked || !$('fleet-key').value;
}
async function send(action, extra = {}) {
  busy = true; refresh();
  try {
    const response = await fetch('/api/fleet/commands', {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Roport-Trial-Token': $('fleet-key').value}, body: JSON.stringify({action, session_id: state.session_id, ...extra}), signal: AbortSignal.timeout(3000)});
    const result = await response.json();
    if (!response.ok) throw new Error(result.detail);
    $('fleet-feedback').textContent = '접수 완료 · 실행부 상태에서 진행을 확인하세요.';
    return result;
  } catch (error) {
    $('fleet-feedback').textContent = error.name === 'TimeoutError' ? '응답 미확인 · 같은 요청을 다시 보내도 중복 실행하지 않습니다.' : error.message;
    return null;
  } finally { busy = false; refresh(); }
}
$('fleet-key').addEventListener('input', refresh);
$('fleet-enable').onclick = () => send('enable');
$('fleet-stop').onclick = () => send('stop');
$('fleet-assign').onclick = () => send('assign', {robot: $('fleet-robot').value, role: $('fleet-role').value || null});
for (const id of ['fleet-operation', 'fleet-package', 'fleet-goal']) $(id).addEventListener('change', () => {request = null;});
$('fleet-submit').onclick = async () => {
  request ??= {request_id: crypto.randomUUID(), operation: $('fleet-operation').value, package_id: $('fleet-package').value.trim(), goal: $('fleet-operation').value === 'inbound' ? null : $('fleet-goal').value};
  if (await send('submit', {request})) request = null;
};
export function fleetDisconnected() { online = false; refresh(); }
export function renderFleet(data) {
  state = data.fleet; online = !!state?.available;
  $('fleet-state').textContent = !online ? '실행부 연결 끊김' : state.fault ? `보류: ${state.fault}` : state.enabled ? '요청 처리 중' : '요청 처리 시작 대기';
  $('fleet-roles').textContent = Object.entries(state.roles ?? {}).map(([robot, role]) => `${robot}: ${role === 'logistics' ? '물류' : role === 'smartcart' ? '스마트카트' : '배정 제외'}`).join(' / ');
  const goals = JSON.stringify(state.goals ?? []);
  if ($('fleet-goal').dataset.values !== goals) {
    $('fleet-goal').replaceChildren(...(state.goals ?? []).map(g => new Option(g, g)));
    $('fleet-goal').dataset.values = goals;
  }
  $('fleet-shelf').replaceChildren(...Object.entries(state.slots ?? {}).map(([slot, value]) => {
    const cell = document.createElement('div');
    cell.textContent = `${slot} · ${state.slot_definitions[slot].node} · ${shelfStates[value.state] ?? value.state} · ${value.package_id ?? '—'}`;
    return cell;
  }));
  $('fleet-work').replaceChildren(...Object.entries(state.tasks ?? {}).map(([id, task]) => {
    const item = document.createElement('li');
    item.textContent = `${task.request.package_id} · ${task.robot ?? '배정 대기'} · ${stages[task.stage] ?? task.stage}${task.reason ? ` / ${task.reason}` : ''}`;
    if (task.stage === 'await_receipt' && state.mode === 'simulation') {
      const button = document.createElement('button'); button.textContent = '가상 고객 수령 확인';
      button.disabled = !online || busy || !$('fleet-key').value;
      button.onclick = () => send('receipt', {request_id: id}); item.append(button);
    }
    return item;
  }));
  $('fleet-resources').textContent = `점유: ${Object.keys(state.resources ?? {}).join(', ') || '이동 예약 없음'}`;
  refresh();
}
