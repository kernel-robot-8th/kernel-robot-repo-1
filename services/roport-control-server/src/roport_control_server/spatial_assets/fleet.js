import { newRequestId } from './scenario.js';

// Requests go to the single owner; roles, shelf stock and tasks share its saved state.
const stages = {queued:'배차 대기', warehouse_route:'창고로 이동', warehouse_load:'창고 상차', choose_slot:'선반 배정 대기', shelf_route:'인도장으로 이동', transfer:'선반 상하차', return_route:'대기 장소로 복귀', await_shelf_alignment:'선반 정렬 완료 대기', switch_route:'N12로 이동', switch_nav:'Nav2 전환', navigate:'고객에게 이동', await_receipt:'고객 수령 대기', follow_pending:'추종 승인 대기', follow_start:'추종 시작 확인 중', following:'추종 중', follow_stopping:'추종 정지 확인 중', follow_paused:'추종 정지', return_nav:'N12로 복귀', switch_line:'라인 주행 전환', done:'완료'};
const shelfStates = {empty:'빈 칸', occupied:'보관 중', reserved:'작업 예약'};
const $ = id => document.getElementById(id);
let state, online = false, busy = false, request;
function refresh() {
  const blocked = state?.mode === 'hardware' && location.protocol !== 'https:' && !['localhost', '127.0.0.1', '[::1]'].includes(location.hostname);
  for (const button of $('fleet-work').querySelectorAll('button')) {
    const robot = button.dataset.robot;
    const report = state?.robots?.[robot];
    const idle = report?.state.stopped && !report.active_command_id && !report.state.fault && state.ages?.[robot] <= 3 && !state.awaiting_state?.includes(robot);
    button.disabled = !online || busy || blocked || !$('fleet-key').value ||
      (button.dataset.operation && (!state.enabled || !!state.fault || (button.dataset.operation !== 'pause' && !idle)));
  }
  $('fleet-fields').disabled = !online || busy || blocked || !$('fleet-key').value;
  $('fleet-enable').disabled = !online || busy || blocked || !$('fleet-key').value || !!state?.fault || state?.enabled;
  $('fleet-stop').disabled = !online || busy || blocked || !$('fleet-key').value;
  $('fleet-recovery').disabled = !online || busy || blocked || !$('fleet-key').value || (!state?.fault && !Object.values(state?.pending_arrivals ?? {}).some(a => a.recovery_boot));
  const n16 = $('fleet-recovery-outcome').value === 'repositioned_n16';
  const id = $('fleet-recovery-command').value;
  const command = state?.commands?.[id];
  const report = state?.robots?.[command?.message.target];
  const arrivalRobot = $('fleet-arrival').value;
  const arrival = state?.pending_arrivals?.[arrivalRobot];
  const check = $('fleet-recovery-confirm');
  const context = JSON.stringify([n16, state?.session_id, id, $('fleet-recovery-outcome').value, command?.stop?.command_id, report?.boot_version, report?.state?.node, report?.state?.stopped, arrivalRobot, arrival]);
  if (!online || check.dataset.context !== context) check.checked = false;
  check.dataset.context = context;
  $('fleet-confirm-label').textContent = n16 && id
    ? command.message.target + '의 N16 재배치, N14 방향, 기체 정지와 통로 확보를 현장에서 확인했습니다.'
    : '정지·물품 위치를 현장에서 확인했습니다.';

  const confirmed = $('fleet-recovery-confirm').checked && $('fleet-recovery-note').value.trim();
  $('fleet-reconcile').disabled = !state?.fault || !confirmed || !$('fleet-recovery-command').value;
  $('fleet-recover-stop').disabled = !state?.fault || !confirmed || !command?.stop || !report?.boot_version;
  $('fleet-confirm-arrival').disabled = !confirmed || !arrival?.recovery_boot;
  $('fleet-release').disabled = !state?.fault || !confirmed || Object.keys(state?.commands ?? {}).length > 0 || Object.keys(state?.pending_arrivals ?? {}).length > 0;
  document.querySelectorAll('[data-fleet-receipt]').forEach(button => { button.disabled = $('fleet-fields').disabled; });
}
async function send(action, extra = {}) {
  busy = true; refresh();
  try {
    const response = await fetch('/api/fleet/commands', {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Roport-Trial-Token': $('fleet-key').value}, body: JSON.stringify({action, session_id: state.session_id, ...extra}), signal: AbortSignal.timeout(3000)});
    const result = await response.json().catch(() => null);
    if (!response.ok || result?.accepted !== true) throw new Error(typeof result?.detail === 'string' ? result.detail : `HTTP ${response.status} · 실행 결과 미확인. 상태를 확인한 뒤 다시 요청하세요.`);
    $('fleet-feedback').textContent = action === 'stop' ? '관제 보류 접수 · 기체 정지는 상태에서 확인하세요.' : '접수 완료 · 실행부 상태에서 진행을 확인하세요.';
    return result;
  } catch (error) {
    const detail = error.name === 'TimeoutError' ? '응답 미확인 · 같은 요청을 다시 보내도 중복 실행하지 않습니다.' : error.message;
    $('fleet-feedback').textContent = action === 'stop' ? `정지 여부 미확인 · ${detail} · 현장 정지 수단으로 기체 정지를 확인하세요.` : detail;
    return null;
  } finally { busy = false; refresh(); }
}
$('fleet-key').addEventListener('input', refresh);
$('fleet-enable').onclick = () => send('enable');
$('fleet-stop').onclick = () => send('stop');
for (const id of ['fleet-recovery-note', 'fleet-recovery-confirm', 'fleet-recovery-command', 'fleet-recovery-outcome', 'fleet-arrival']) $(id).addEventListener('input', refresh);
$('fleet-reconcile').onclick = () => {
  const command_id = $('fleet-recovery-command').value, outcome = $('fleet-recovery-outcome').value;
  const extra = outcome === 'repositioned_n16' ? {
    confirmed: $('fleet-recovery-confirm').checked,
    boot_version: state.robots[state.commands[command_id].message.target]?.boot_version,
  } : {};
  return send('reconcile', {command_id, outcome, note: $('fleet-recovery-note').value.trim(), ...extra});
};
$('fleet-recover-stop').onclick = () => {
  const command_id = $('fleet-recovery-command').value, command = state.commands[command_id];
  return send('recover_stop', {command_id, stop_id: command.stop.command_id, boot_version: state.robots[command.message.target].boot_version, note: $('fleet-recovery-note').value.trim()});
};
$('fleet-confirm-arrival').onclick = () => {
  const robot = $('fleet-arrival').value, arrival = state.pending_arrivals[robot];
  return send('confirm_arrival', {robot, command_id: arrival.command_id, boot_version: arrival.recovery_boot, note: $('fleet-recovery-note').value.trim()});
};
$('fleet-release').onclick = () => send('release_hold', {note: $('fleet-recovery-note').value.trim()});
$('fleet-assign').onclick = () => send('assign', {robot: $('fleet-robot').value, role: $('fleet-role').value || null});
for (const id of ['fleet-operation', 'fleet-package', 'fleet-goal']) $(id).addEventListener('change', () => {request = null;});
$('fleet-submit').onclick = async () => {
  // Keep the ID on rejection/unknown outcomes; only accepted or edited input starts a new request.
  request ??= {request_id: newRequestId(), operation: $('fleet-operation').value, package_id: $('fleet-package').value.trim(), goal: $('fleet-operation').value === 'inbound' ? null : $('fleet-goal').value};
  if (await send('submit', {request})) request = null;
};
export function fleetDisconnected() { online = false; renderDiagnostics(); refresh(); }
function list(id, rows, empty) {
  $(id).replaceChildren(...(rows.length ? rows : [empty]).map(text => {
    const item = document.createElement('li'); item.textContent = text; return item;
  }));
}
function renderDiagnostics() {
  const data = state ?? {};
  const commands = Object.keys(data.commands ?? {}), selection = $('fleet-recovery-command').value;
  if ($('fleet-recovery-command').dataset.keys !== JSON.stringify(commands)) {
    $('fleet-recovery-command').replaceChildren(...commands.map(id => new Option(`${data.commands[id].message.target} · ${id}`, id)));
    if (commands.includes(selection)) $('fleet-recovery-command').value = selection;
    $('fleet-recovery-command').dataset.keys = JSON.stringify(commands);
  }
  const arrivals = Object.entries(data.pending_arrivals ?? {}).filter(([,a]) => a.recovery_boot), selected = $('fleet-arrival').value;
  if ($('fleet-arrival').dataset.values !== JSON.stringify(arrivals)) {
    $('fleet-arrival').replaceChildren(...arrivals.map(([robot, a]) => new Option(`${robot} · ${a.node} · ${a.command_id} · ${a.recovery_boot}`, robot)));
    if (arrivals.some(([robot]) => robot === selected)) $('fleet-arrival').value = selected;
    $('fleet-arrival').dataset.values = JSON.stringify(arrivals);
  }
  $('fleet-diagnostic-status').textContent = !online ? '연결 끊김 · 아래 내용은 마지막 수신 상태입니다.' : data.fault ? `보류 사유: ${data.fault}` : '보류 없음 · 최신 수신 상태';
  const section = $('fleet-diagnostics');
  if (data.fault && section.dataset.fault !== data.fault) section.open = true;
  section.dataset.fault = data.fault ?? '';
  list('fleet-robot-observations', Object.entries(data.robots ?? {}).map(([robot, observation]) => {
    const seen = observation.state ?? {}, age = data.ages?.[robot];
    const fresh = online && Number.isFinite(age) && age <= 3;
    const time = Number.isFinite(age) ? `${Math.floor(age)}초 전` : '수신 시각 미확인';
    const stopped = seen.stopped === true ? '정지 관측' : seen.stopped === false ? '정지 미확인' : '정지 여부 미확인';
    const location = seen.node ?? (seen.mode === 'nav2' ? '공용 공간' : '노드 미확인');
    const routeCommand = Object.values(data.commands ?? {}).find(c =>
      c.message.target === robot && c.message.type === 'line_route_command' && data.tasks?.[c.task]?.status === 'HOLD');
    const route = routeCommand?.message.payload.route ?? [];
    const index = routeCommand?.route_index ?? 0;
    const reason = (routeCommand && data.tasks[routeCommand.task].reason) || seen.fault;
    const leg = route[index + 1] ? ` · 마지막 확인 ${route[index]} → ${route[index + 1]} 도착 미확인` : '';
    const incident = routeCommand ? `${leg} · 정지 사유: ${reason ?? data.fault} · ${fresh ? 'MQTT 상태 수신 중' : 'MQTT 상태 지연'}` : '';
    const holding = seen.holding_package_id ?? seen.holding_state ?? '미확인';
    return `${robot} · ${fresh ? '최신' : '오래된 관측'} (${time}) · ${location} · ${seen.mode ?? '팔'} · ${stopped}${incident} · 적재: ${holding} · 기체 명령: ${observation.active_command_id ?? '없음'}`;
  }), '수신한 기체 상태 없음');
  list('fleet-pending-commands', Object.entries(data.commands ?? {}).map(([id, command]) => {
    const task = data.tasks?.[command.task];
    return `${command.message.target} · ${command.message.type} · ${task?.request.package_id ?? command.task} · ${stages[task?.stage] ?? task?.stage ?? '단계 미확인'} · 명령 ${id}`;
  }), '실행부의 미완료 명령 없음');
  list('fleet-reservation-owners', Object.entries(data.resources ?? {}).map(([resource, owner]) => {
    const task = data.tasks?.[owner];
    return `${resource} → ${task?.robot ?? '기체 미배정'} / ${task?.request.package_id ?? owner}`;
  }), '유지 중인 예약 없음');
}
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
    const alignment = value.alignment === 'unconfirmed' ? ' · 정렬 미확인' : '';
    cell.textContent = `${slot} · ${state.slot_definitions[slot].node} · ${shelfStates[value.state] ?? value.state} · ${value.package_id ?? '—'}${alignment}`;
    return cell;
  }));
  // Keep clickable controls mounted between robot pose updates.
  const work = $('fleet-work');
  const workKey = JSON.stringify([state.session_id, state.mode, Object.entries(state.tasks ?? {}).map(([id, t]) =>
    [id, t.robot, t.request.package_id, t.stage, t.status, t.reason, t.follow_card])]);
  if (work.dataset.key !== workKey) {
    work.dataset.key = workKey;
    work.replaceChildren(...Object.entries(state.tasks ?? {}).map(([id, task]) => {
      const item = document.createElement('li');
      item.textContent = [task.request.package_id, task.robot ?? '배정 대기', task.status === 'CANCELED' ? '운영자 처리로 종료' : (stages[task.stage] ?? task.stage), task.reason].filter(Boolean).join(' · ');
      const operations = {follow_pending: [['approve','추종 승인'],['return','복귀']], follow_start: [['pause','추종 정지']], following: [['pause','추종 정지']], follow_paused: [['resume','추종 재개'],['return','추종 종료 및 복귀']]}[task.stage] ?? [];
      if (task.follow_card) item.append(document.createTextNode(` · 카드 ${task.follow_card}`));
      for (const [operation, label] of operations) {
        const button = document.createElement('button');
        button.textContent = label;
        button.dataset.robot = task.robot;
        button.dataset.operation = operation;
        button.onclick = () => send('follow', {request_id: id, operation});
        item.append(button);
      }
      if (task.stage === 'await_receipt' && state.mode === 'simulation') {
        const button = document.createElement('button'); button.textContent = '가상 고객 수령 확인';
        button.onclick = () => send('receipt', {request_id: id}); item.append(button);
      }
      return item;
    }));
  }
  $('fleet-resources').textContent = `점유: ${Object.keys(state.resources ?? {}).join(', ') || '이동 예약 없음'}`;
  renderDiagnostics();
  refresh();
}
