// Drafts never dispatch robots. The owner validates and freezes the applied plan.
const $ = (id) => document.getElementById(id);
const copy = (value) => structuredClone(value);
let current, connected, token, draft, draftSession, templates = [], generatedRequest = null, pending = false, revision;
let dirty = false;
export function hasUnappliedChanges() { return dirty; }
const names = {route: '라인 이동', gate: '작업 완료 승인 (더미)', request: '물품 요청 대기', mode_switch: 'Nav2 전환', navigate: 'Nav2 목표 이동'};

async function send(action, extra = {}) {
  if (!connected || !token || pending) throw new Error('실행부 연결과 조작 키를 확인하세요.');
  pending = true;
  refreshEnabled();
  try {
    const response = await fetch('/api/trial/commands', {
      method: 'POST', headers: {'Content-Type': 'application/json', 'X-Roport-Trial-Token': token},
      body: JSON.stringify({action, session_id: current.session_id, task_id: current.task_id, phase: current.phase, ...extra}),
      signal: AbortSignal.timeout(5000),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.detail);
    return data.result;
  } finally { pending = false; refreshEnabled(); }
}
function refreshEnabled() {
  $('scenario-fields').disabled = !connected || !token || pending || !current?.can_configure;
}
function changed() {
  dirty = true;
  document.dispatchEvent(new Event('scenario-draft-change'));
  generatedRequest = null;
  $('scenario-confirm').checked = false;
  $('scenario-feedback').textContent = '편집 중 · 실행 중인 계획에는 반영되지 않습니다.';
  preview();
}
function field(parent, label, value, update, type = 'text') {
  const row = document.createElement('label'), input = document.createElement('input');
  row.append(document.createTextNode(label + ' '), input);
  input.type = type;
  if (type === 'number') input.step = 'any';
  input.value = value;
  if (label === '노드 목록') { input.size = 52; input.placeholder = 'N16 N14 N02 … N11'; }
  input.addEventListener('input', () => { update(type === 'number' ? (input.value === '' ? null : Number(input.value)) : input.value); changed(); });
  parent.append(row);
}
function button(parent, label, fn) {
  const b = document.createElement('button'); b.type = 'button'; b.textContent = label;
  b.addEventListener('click', fn); parent.append(b); return b;
}
function renderSteps() {
  $('scenario-name').value = draft.name;
  $('scenario-assumption').value = draft.assumptions.join(' / ');
  $('scenario-steps').replaceChildren(...draft.steps.map((s, index) => {
    const li = document.createElement('li'), header = document.createElement('strong');
    header.textContent = `${index + 1}. ${names[s.kind]}`; li.append(header);
    const select = document.createElement('select'); select.setAttribute('aria-label', `단계 ${index + 1} 기체`);
    for (const [role, robot] of Object.entries(current.catalogue.robots)) select.add(new Option(`${robot.robot_id} · ${role}`, role));
    select.value = s.role; select.addEventListener('change', () => { s.role = select.value; changed(); }); li.append(select);
    field(li, '작업 이름', s.label, (v) => { s.label = v; });
    if (s.kind === 'route') {
      field(li, '노드 목록', s.payload.route.join(' '), (v) => {
        s.payload.route = v.trim().split(/[\s,→]+/).filter(Boolean);
        s.payload.start_node = s.payload.route[0]; s.payload.goal.node_id = s.payload.route.at(-1);
      });
      field(li, '출발 방향(rad)', s.payload.start_heading_rad, (v) => { s.payload.start_heading_rad = v; }, 'number');
      field(li, '속도 상한(m/s)', s.payload.speed_limit_mps, (v) => { s.payload.speed_limit_mps = v; }, 'number');
      button(li, '첫·끝 노드로 경로 계산', () => perform(async () => {
        const result = await send('route_plan', {start_node: s.payload.start_node, goal_node: s.payload.goal.node_id});
        s.payload.route = result.route; changed(); renderSteps();
        return '최소 간선 경로 생성 · 출발 방향과 통로를 확인하세요.';
      }));
    } else if (s.kind === 'navigate') {
      for (const k of ['x', 'y', 'yaw']) field(li, `목표 ${k}${k === 'yaw' ? '(rad)' : '(m)'}`, s.payload.pose[k], (v) => { s.payload.pose[k] = v; }, 'number');
    } else if (s.kind === 'mode_switch') {
      const text = document.createElement('p'); text.textContent = 'N12 정차 · line → Nav2'; li.append(text);
    } else field(li, '대기 노드', s.payload.node_id, (v) => { s.payload.node_id = v.trim(); });
    button(li, '위로', () => { if (index) { [draft.steps[index - 1], draft.steps[index]] = [s, draft.steps[index - 1]]; changed(); renderSteps(); } });
    button(li, '삭제', () => { draft.steps.splice(index, 1); changed(); renderSteps(); });
    return li;
  }));
  preview();
}
function preview() {
  const svg = $('scenario-preview'), ns = 'http://www.w3.org/2000/svg';
  const nodes = current?.catalogue?.node_positions ?? {};
  const values = Object.values(nodes); if (!values.length) return;
  const minX = Math.min(...values.map(p => p[0])), maxX = Math.max(...values.map(p => p[0]));
  const minY = Math.min(...values.map(p => p[1])), maxY = Math.max(...values.map(p => p[1]));
  const xy = p => [20 + (p[0] - minX) / (maxX - minX || 1) * 360, 200 - (p[1] - minY) / (maxY - minY || 1) * 180];
  const items = [];
  for (const s of draft?.steps ?? []) if (s.kind === 'route') {
    const line = document.createElementNS(ns, 'polyline');
    line.setAttribute('points', s.payload.route.filter(n => nodes[n]).map(n => xy(nodes[n]).join(',')).join(' '));
    line.setAttribute('fill', 'none'); line.setAttribute('stroke', '#257164'); line.setAttribute('stroke-width', '2'); items.push(line);
  }
  for (const [id, p] of Object.entries(nodes)) {
    const text = document.createElementNS(ns, 'text'), [x,y] = xy(p);
    text.setAttribute('x', x); text.setAttribute('y', y); text.setAttribute('font-size', '9'); text.textContent = id; items.push(text);
  }
  svg.replaceChildren(...items);
}
async function perform(fn) {
  try { $('scenario-feedback').textContent = await fn(); }
  catch (e) { $('scenario-feedback').textContent = e.name === 'TimeoutError' ? '응답 미확인 · 자동 재전송 없이 현재 계획을 확인하세요.' : e.message; }
}
function draftValue() {
  draft.name = $('scenario-name').value;
  draft.assumptions = $('scenario-assumption').value.split(' / ').map(s => s.trim()).filter(Boolean);
  return copy(draft);
}
$('scenario-name').addEventListener('input', changed);
$('scenario-assumption').addEventListener('input', changed);
$('scenario-preset-load').addEventListener('click', () => {
  const preset = current?.catalogue?.presets?.[$('scenario-preset').value];
  if (!preset) return;
  draft = copy(preset); draftSession = current.session_id;
  if (!$('scenario-nav2').checked) draft.steps = draft.steps.filter(s => !['mode_switch', 'navigate'].includes(s.kind));
  changed(); renderSteps();
  $('scenario-feedback').textContent = '기본안을 편집창에 불러왔습니다. 노드·출발 방향을 확인하고 실행 계획 적용을 누르세요.';
});
$('scenario-copy').addEventListener('click', () => { draft = copy(current.plan); draftSession = current.session_id; draft.name += ' 복제'; changed(); renderSteps(); });
$('scenario-add').addEventListener('click', () => {
  const step = copy(templates[Number($('scenario-template').value)]); if (!step) return;
  step.id = 'step_' + crypto.randomUUID(); draft.steps.push(step); changed(); renderSteps();
});
$('scenario-load').addEventListener('click', () => perform(async () => {
  const r = await send('load_plan', {name: $('scenario-saved').value}); draft = r.plan; draftSession = current.session_id; changed(); renderSteps(); return '저장된 계획을 편집창에 불러왔습니다.';
}));
$('scenario-validate').addEventListener('click', () => perform(async () => {
  await send('validate_plan', {plan: draftValue()}); return '설정 검증 통과 · 실제 출발 가능 여부는 적용 후 최신 기체 상태로 확인합니다.';
}));
$('scenario-save').addEventListener('click', () => perform(async () => {
  await send('save_plan', {plan: draftValue()}); return '서버 scenario-plans.json에 저장했습니다. 실행 계획은 바뀌지 않았습니다.';
}));
$('scenario-apply').addEventListener('click', () => perform(async () => {
  const plan = draftValue(), extra = {plan, session_id: draftSession, field_confirmed: $('scenario-confirm').checked};
  if (generatedRequest) extra.request = generatedRequest;
  const result = await send(generatedRequest ? 'submit_delivery' : 'configure', extra);
  draftSession = result.session_id;
  dirty = false;
  document.dispatchEvent(new Event('scenario-draft-change'));
  return '계획 적용 완료 · 상태 확인 후 시나리오 출발을 누르세요.';
}));
$('delivery-generate').addEventListener('click', () => perform(async () => {
  const request = {request_id: $('delivery-id').value, package_id: current.package_id, quantity: 1,
    role: $('delivery-role').value, pickup: $('delivery-pickup').value, destination: $('delivery-destination').value,
    pickup_heading_rad: Number($('delivery-heading').value), nav2: $('delivery-nav2').checked};
  const result = await send('delivery_plan', {request}); draft = result.plan; draftSession = current.session_id; changed(); generatedRequest = request; renderSteps();
  return '요청 계획 생성 · 배정 기체와 경로를 검토한 뒤 적용하세요.';
}));
export function renderComposer(state, online, operationToken) {
  current = state; connected = online; token = operationToken;
  refreshEnabled();
  if (!state?.catalogue || !state.plan) return;
  const presets = state.catalogue.presets ?? {};
  const presetKeys = JSON.stringify(Object.keys(presets));
  if ($('scenario-preset').dataset.keys !== presetKeys) {
    $('scenario-preset').replaceChildren(...Object.entries(presets).map(([id,p]) => new Option(p.name,id)));
    $('scenario-preset').dataset.keys = presetKeys;
  }
  $('scenario-nav2').disabled = state.scope === 'line-only';
  if (state.scope === 'line-only') $('scenario-nav2').checked = false;
  $("scenario-stops").textContent = `종료 가능한 정차 노드: ${(state.catalogue.stop_nodes ?? []).join(", ")}. 교차로 노드는 경유 지점으로 지정하세요.`;
  if (!draft) {
    draft = copy(state.plan); draftSession = state.session_id;
    const role = Object.keys(state.catalogue.robots)[0];
    templates = Object.entries(state.catalogue.routes).map(([id, payload]) => ({id, kind: 'route', role, label: id, payload}));
    templates.push(...state.plan.steps.filter(s => s.kind !== 'route'));
    if (!templates.some(s => s.kind === 'mode_switch')) templates.push({id: 'mode_switch', kind: 'mode_switch', role, label: 'Nav2 전환', payload: {node_id: 'N12', from_mode: 'line', to_mode: 'nav2', map_id: state.catalogue.nav_goal.map_id, map_version: state.catalogue.nav_goal.map_version}});
    if (!templates.some(s => s.kind === 'navigate')) templates.push({id: 'navigate', kind: 'navigate', role, label: 'Nav2 목표 이동', payload: state.catalogue.nav_goal});
    $('scenario-template').replaceChildren(...templates.map((s,i) => new Option(`${names[s.kind]} · ${s.label}`, i)));
    $('delivery-role').replaceChildren(new Option('자동 배정', 'auto'), ...Object.entries(state.catalogue.robots).map(([r,v]) => new Option(`${v.robot_id} · ${r}`, r)));
    $('delivery-id').value = 'request_' + crypto.randomUUID();
    renderSteps();
  }
  const saved = state.catalogue.saved ?? [];
  if (JSON.stringify(saved) !== $('scenario-saved').dataset.names) {
    $('scenario-saved').replaceChildren(...saved.map(n => new Option(n,n))); $('scenario-saved').dataset.names = JSON.stringify(saved);
  }
  if (revision && revision !== state.plan_revision) $('scenario-confirm').checked = false;
  revision = state.plan_revision;
  $('scenario-progress').replaceChildren(...state.plan.steps.map((s,i) => {
    const li = document.createElement('li'); li.textContent = `${i < state.plan_index ? '완료' : i === state.plan_index ? '현재' : '대기'} · ${s.label} · ${state.catalogue.robots[s.role]?.robot_id}`; return li;
  }));
}
