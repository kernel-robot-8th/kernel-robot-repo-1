// OFFLINE/DEMO는 기존 시안, opt-in Fleet 등록은 실행부의 공유 저장소를 사용한다.
// 로봇 설치·ROS 도메인 적용은 이 화면에서 수행하지 않는다.
import { newRequestId } from './scenario.js';

const root = document.getElementById("pinky-detail");
const demo = new URLSearchParams(location.search).get("mode") === "demo";
const ROLE = { logistics: "물류", smartcart: "스마트카트" };
const ACTION_LABEL = {
  check: "연결 확인", read_domain: "도메인 읽기", set_domain: "도메인 적용",
  deploy: "업데이트 설치", release: "릴리즈 설치", build: "빌드",
};
let view = null;
let robots = [];
const jobs = [];
const SLOTS = ['pinky1', 'pinky2', 'pinky3'];
let registryState, registryAccess, registryDraft, registryPending, registryDirty = false;

function el(tag, attrs = {}, text = "") {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "disabled") e.disabled = !!v;
    else e.setAttribute(k, v);
  }
  if (text) e.textContent = text;
  return e;
}

async function post(path, body) {
  const res = await fetch(path, {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
  });
  let data = {};
  try { data = await res.json(); } catch { /* 빈 응답 */ }
  return { status: res.status, data };
}

function setMsg(text, kind) {
  const m = document.getElementById("ps-msg");
  m.textContent = text;
  m.dataset.kind = kind || "";
}

async function validate(kind, value) {
  const r = await post("/api/pinky/settings/validate", { kind, value });
  if (r.status !== 200) throw new Error(r.data.detail || `HTTP ${r.status}`);
  return r.data.value;
}

async function save() {
  const value = {
    name: document.getElementById("ps-name").value.trim(),
    ip: document.getElementById("ps-ip").value.trim(),
    role: document.getElementById("ps-role").value,
    ros_domain_id: Number(document.getElementById("ps-domain").value),
    calibration: document.getElementById("ps-calib").value,
  };
  try {
    const robot = await validate("robot", value);
    if (demo) {
      robots = robots.filter((r) => r.name !== robot.name).concat([robot]);
      robots.sort((a, b) => a.name.localeCompare(b.name));
      setMsg(`DEMO 저장: ${robot.name} (이 브라우저에서만)`, "ok");
      render();
      return;
    }
    const r = await post("/api/pinky/settings/robots", value);
    setMsg(r.status === 503 ? "형식 확인됨. 실행부·운영 DB 미연결이라 저장하지 않음" : `실패: ${r.data.detail}`,
      r.status === 503 ? "warn" : "err");
  } catch (e) {
    setMsg(`입력 확인: ${e.message}`, "err");
  }
}

async function runJob(robot, action) {
  const body = { name: robot.name, action: action === "release" ? "deploy" : action };
  if (action === "release") body.release = true;
  if (action === "deploy") body.ref = document.getElementById("ps-ref").value.trim();
  if (action === "set_domain") body.ros_domain_id = robot.ros_domain_id;
  try {
    const job = await validate("job", body);
    const detail = job.ref ? ` ${job.ref}` : job.ros_domain_id !== undefined ? ` ROS_DOMAIN_ID=${job.ros_domain_id}` : "";
    jobs.unshift({ t: new Date().toLocaleTimeString(), text: `${robot.name} ${ACTION_LABEL[action]}${detail} — DEMO(실행 안 함)` });
    if (jobs.length > 20) jobs.pop();
    render();
  } catch (e) {
    setMsg(`작업 확인: ${e.message}`, "err");
  }
}

function render() {
  if (registryState) return;
  root.replaceChildren();
  const box = el("div", { class: "ps" });
  const head = el("div", { class: "section-head" });
  head.append(el("h2", {}, "Pinky Settings"),
    el("small", {}, demo ? "DEMO · 이 브라우저에서만 바뀜 · 로봇 접속 없음" : "OFFLINE · 실행부·운영 DB 미연결"));
  box.append(head);
  box.append(el("p", { class: "note" }, demo
    ? "예시 Pinky 3대입니다. 저장·작업은 형식만 확인하고 실제로 실행하지 않습니다."
    : "등록·설치·도메인 적용은 실행부가 연결되면 사용할 수 있습니다. 지금은 Pinky 테스트 콘솔(mode:=fleet)에서 시험합니다."));

  const table = el("table", { class: "ps-table" });
  const thead = el("tr");
  for (const h of ["이름", "역할", "IP", "ROS_DOMAIN_ID", "IR 기준값", "작업"]) thead.append(el("th", {}, h));
  table.append(thead);
  if (!robots.length) {
    const tr = el("tr");
    const td = el("td", { colspan: "6", class: "note" }, "등록된 Pinky가 없습니다.");
    tr.append(td);
    table.append(tr);
  }
  for (const r of robots) {
    const tr = el("tr");
    for (const v of [r.name, ROLE[r.role] || r.role, r.ip, String(r.ros_domain_id), r.calibration]) tr.append(el("td", {}, v));
    const td = el("td", { class: "ps-actions" });
    for (const a of ["check", "read_domain", "set_domain", "deploy", "release", "build"]) {
      const b = el("button", { type: "button", disabled: !demo, title: demo ? "" : "실행부 미구현" }, ACTION_LABEL[a]);
      b.onclick = () => runJob(r, a);
      td.append(b);
    }
    tr.append(td);
    table.append(tr);
  }
  box.append(table);

  const ref = el("label", { class: "ps-field" }, "업데이트 브랜치 ");
  ref.append(el("input", { id: "ps-ref", value: "origin/main", maxlength: "100" }));
  box.append(ref);
  box.append(el("p", { class: "note" },
    `설치 = 브랜치(업데이트) 또는 ${view.release_ref}(릴리즈)의 ${view.packages.join(", ")}를 로봇 ~/pinky_test_ws/src에 보내고 빌드. ` +
    "도메인 적용 = 로봇 ~/.bashrc에 export ROS_DOMAIN_ID=N  # roport-domain 한 줄."));

  const form = el("fieldset", { class: "ps-form" });
  form.append(el("legend", {}, "등록 / 수정"));
  const fields = [
    ["ps-name", "이름", el("input", { id: "ps-name", maxlength: "16", placeholder: "pinky1" })],
    ["ps-ip", "IP", el("input", { id: "ps-ip", maxlength: "15", placeholder: "192.168.0.21" })],
    ["ps-role", "역할", (() => {
      const s = el("select", { id: "ps-role" });
      for (const r of view.roles) s.append(el("option", { value: r }, ROLE[r] || r));
      return s;
    })()],
    ["ps-domain", "ROS_DOMAIN_ID", el("input", { id: "ps-domain", type: "number", min: "0", max: String(view.domain_range[1]), value: "14" })],
    ["ps-calib", "IR 기준값", (() => {
      const s = el("select", { id: "ps-calib" });
      for (const c of ["pinky_d660", "pinky_6422", "pinky_764e"]) s.append(el("option", { value: c }, c));
      return s;
    })()],
  ];
  for (const [id, label, input] of fields) {
    const l = el("label", { class: "ps-field", for: id }, label + " ");
    l.append(input);
    form.append(l);
  }
  const btn = el("button", { type: "button" }, demo ? "저장 (DEMO)" : "형식 확인");
  btn.onclick = save;
  form.append(btn, el("p", { id: "ps-msg", role: "status", class: "note" }));
  box.append(form);

  if (jobs.length) {
    box.append(el("h2", {}, "작업 기록 (DEMO)"));
    const ul = el("ul", { class: "ps-jobs" });
    for (const j of jobs) ul.append(el("li", {}, `${j.t} ${j.text}`));
    box.append(ul);
  }
  root.append(box);
}

// Opt-in Fleet registration shares the owner, database and operator key with the overview.
export function renderPinkyRegistry(state, access) {
  if (!state?.registry) return;
  registryState = state;
  registryAccess = access;
  if (!document.getElementById('ps-registry')) buildRegistry();
  refreshRegistry();
}

function registryRow() {
  return registryState.registry.robots?.find(r => r.robot_id === document.getElementById('ps-slot').value);
}

function loadRegistryRow() {
  const row = registryRow();
  registryDraft = row;
  registryDirty = false;
  registryPending = null;
  const config = row?.desired_config ?? {};
  for (const [id, value] of Object.entries({
    'ps-device': row?.device_id ?? '', 'ps-ip': config.ip ?? '',
    'ps-role': config.role ?? 'logistics', 'ps-domain': config.ros_domain_id ?? 14,
    'ps-calib': config.calibration ?? '',
  })) document.getElementById(id).value = value;
  document.getElementById('ps-empty-load').checked = false;
  document.getElementById('ps-off-lane').checked = false;
  setMsg('등록값은 관제 설정입니다. 로봇의 설치·실제 적용 상태는 별도로 확인하세요.');
}

function refreshRegistry() {
  const registry = registryState.registry;
  const row = registryRow();
  if (!registryDirty && !registryPending && row?.revision !== registryDraft?.revision) loadRegistryRow();
  document.getElementById('ps-registry').disabled = registryAccess.disabled || registry.available === false || !row;
  document.getElementById('ps-registry-status').textContent = !registryAccess.online || registry.available === false
    ? '등록 저장소 연결 끊김 · 마지막 수신 정보'
    : `${registry.development ? '가상 시험 등록' : '현장 등록'} · 고정 3자리 · 통합관제와 같은 저장소`;
  const table = document.getElementById('ps-registry-rows');
  const key = JSON.stringify(registry.robots);
  if (table.dataset.key !== key) {
    table.dataset.key = key;
    table.replaceChildren(...SLOTS.map(slot => {
      const r = registry.robots?.find(item => item.robot_id === slot);
      const tr = el('tr', {'data-slot': slot}), config = r?.desired_config;
      const status = !r?.registered ? '미등록' : r.excluded ? '배정 제외' : '등록됨';
      for (const text of [slot, r?.device_id ?? '—', ROLE[config?.role] ?? '—',
        config?.ip ?? '—', ROLE[r?.fleet_role] ?? '배정 없음', status]) tr.append(el('td', {}, text));
      return tr;
    }));
  }
  document.getElementById('ps-device').disabled = !!registryDraft?.registered;
  document.getElementById('ps-save').textContent = registryDraft?.registered ? '설정 저장' : '기체 등록';
  document.getElementById('ps-exclude').disabled = !row?.registered;
  document.getElementById('ps-exclude').textContent = row?.excluded ? '배정에 다시 포함' : '배정에서 제외';
  document.getElementById('ps-unregister').disabled = !row?.registered || !row.excluded ||
    !document.getElementById('ps-empty-load').checked || !document.getElementById('ps-off-lane').checked;
  document.getElementById('ps-retry').hidden = !registryPending;
  document.getElementById('ps-revision').textContent = registryDirty && row?.revision !== registryDraft?.revision
    ? '다른 조작에서 등록 정보가 바뀌었습니다. 다시 불러온 후 수정하세요.'
    : `슬롯 변경 번호 ${row?.revision ?? '—'} · 로봇 설정 적용 여부 미확인`;
  const report = registryState.robots?.[row?.robot_id];
  const context = JSON.stringify([registryState.session_id, row?.binding_id, report?.boot_version, report?.active_command_id, report?.state?.stopped]);
  const confirmation = document.getElementById('ps-confirmations');
  if (registryAccess.disabled || confirmation.dataset.context !== context) {
    document.getElementById('ps-empty-load').checked = false;
    document.getElementById('ps-off-lane').checked = false;
    document.getElementById('ps-unregister').disabled = true;
  }
  confirmation.dataset.context = context;
}

async function sendRegistry(operation) {
  const row = registryDraft;
  if (!row || registryAccess.disabled) return;
  const request = {expected_revision: row.revision};
  if (operation === 'registration' || operation === 'config') {
    const fields = ['ps-ip', 'ps-domain', 'ps-calib', ...(operation === 'registration' ? ['ps-device'] : [])];
    if (fields.some(id => !document.getElementById(id).reportValidity())) return;
    request.config = {
      ip: document.getElementById('ps-ip').value.trim(), role: document.getElementById('ps-role').value,
      ros_domain_id: Number(document.getElementById('ps-domain').value), calibration: document.getElementById('ps-calib').value.trim(),
    };
  }
  if (operation === 'registration') request.device_id = document.getElementById('ps-device').value.trim();
  else request.binding_id = row.binding_id;
  if (operation === 'exclude') request.excluded = !row.excluded;
  const message = {robot: row.robot_id, operation, request};
  if (operation === 'unregister') message.confirmed = {
    empty_load: document.getElementById('ps-empty-load').checked,
    removed_from_lane: document.getElementById('ps-off-lane').checked,
  };
  const key = JSON.stringify(message);
  if (registryPending?.key !== key) registryPending = {key, message: {...message, request: {...request, request_id: newRequestId()}}};
  await retryRegistry();
}

async function retryRegistry() {
  if (!registryPending || registryAccess.disabled) return;
  const pending = registryPending;
  if (pending.message.operation === 'unregister' &&
    (!document.getElementById('ps-empty-load').checked || !document.getElementById('ps-off-lane').checked)) {
    setMsg('빈 적재함과 통로 밖 배치를 다시 확인하세요.', 'warn'); return;
  }
  setMsg('저장 결과 확인 중…');
  const result = await registryAccess.send('registry', pending.message);
  if (result) {
    registryPending = null;
    registryState = result.result;
    loadRegistryRow();
    setMsg('저장 완료 · 통합관제 배정에 반영했습니다. 로봇 설정 파일은 변경하지 않습니다.', 'ok');
  } else setMsg(document.getElementById('fleet-feedback').textContent, 'err');
  refreshRegistry();
}

function buildRegistry() {
  root.replaceChildren();
  const box = el('div', {class: 'ps'});
  box.append(el('h2', {}, 'Pinky Settings'), el('p', {id: 'ps-registry-status', class: 'note'}));
  const table = el('table', {class: 'ps-table'}), head = el('tr');
  for (const label of ['슬롯', '기체 식별자', '설정 역할', 'IP', '현재 배차 역할', '등록 상태']) head.append(el('th', {}, label));
  table.append(head, el('tbody', {id: 'ps-registry-rows'}));
  const scroll = el('div', {class: 'ps-scroll'}); scroll.append(table); box.append(scroll);
  box.append(el('p', {class: 'note'}, '통합관제의 조작 키를 먼저 입력하세요. 역할·IP·ROS 도메인·보정값은 등록 정보이며 로봇에 자동 설치하거나 적용하지 않습니다.'));
  const form = el('fieldset', {id: 'ps-registry', class: 'ps-form'});
  form.append(el('legend', {}, '기체 등록 / 설정'));
  const slot = el('select', {id: 'ps-slot'}), role = el('select', {id: 'ps-role'});
  for (const value of SLOTS) slot.append(el('option', {value}, value));
  for (const [value, label] of Object.entries(ROLE)) role.append(el('option', {value}, label));
  const fields = [
    ['슬롯', slot], ['기체 식별자', el('input', {id: 'ps-device', required: '', maxlength: '64', pattern: '[a-zA-Z0-9][a-zA-Z0-9_.\\-]{0,63}', placeholder: 'sim-unit-1 또는 실제 기체 식별자'})],
    ['IP', el('input', {id: 'ps-ip', required: '', maxlength: '15', placeholder: '192.0.2.11'})],
    ['역할', role], ['ROS_DOMAIN_ID', el('input', {id: 'ps-domain', required: '', type: 'number', min: '0', max: '101'})],
    ['보정 식별자', el('input', {id: 'ps-calib', required: '', maxlength: '14', pattern: 'pinky_[0-9a-z]{2,8}', placeholder: 'pinky_d660'})],
  ];
  for (const [label, input] of fields) {
    const field = el('label', {class: 'ps-field', for: input.id}, label);
    field.append(input); form.append(field);
    if (input !== slot) input.addEventListener('input', () => {registryDirty = true; registryPending = null; refreshRegistry();});
  }
  slot.onchange = () => {loadRegistryRow(); refreshRegistry();};
  const actions = el('div', {class: 'ps-actions'});
  for (const [id, text, handler] of [
    ['ps-save', '기체 등록', () => sendRegistry(registryDraft?.registered ? 'config' : 'registration')],
    ['ps-reload', '다시 불러오기', () => {loadRegistryRow(); refreshRegistry();}],
    ['ps-exclude', '배정에서 제외', () => sendRegistry('exclude')],
    ['ps-retry', '같은 요청 다시 확인', retryRegistry],
  ]) {const button = el('button', {id, type: 'button'}, text); button.onclick = handler; actions.append(button);}
  form.append(actions, el('p', {id: 'ps-revision', class: 'note'}));
  const confirmation = el('div', {id: 'ps-confirmations'});
  for (const [id, label] of [['ps-empty-load', '물품이 실려 있지 않음을 확인했습니다.'], ['ps-off-lane', '기체를 주행 통로 밖으로 옮겼습니다.']]) {
    const field = el('label', {class: 'ps-confirm'}), check = el('input', {id, type: 'checkbox'});
    check.onchange = refreshRegistry; field.append(check, document.createTextNode(label)); confirmation.append(field);
  }
  const unregister = el('button', {id: 'ps-unregister', type: 'button'}, '등록 해제');
  unregister.onclick = () => sendRegistry('unregister');
  confirmation.append(unregister, el('p', {class: 'note'}, '먼저 배정에서 제외하세요. 실행부가 최신 정차·진행 작업·점유 상태를 확인한 뒤 해제합니다.'));
  form.append(confirmation);
  box.append(form, el('p', {id: 'ps-msg', role: 'status', class: 'note'}));
  root.append(box); loadRegistryRow();
}

async function boot() {
  const res = await fetch(`/api/pinky/settings?mode=${demo ? "demo" : "offline"}`, { cache: "no-store" });
  if (!res.ok) throw new Error(`pinky settings: ${res.status}`);
  view = await res.json();
  robots = view.robots;
  render();
}

boot().catch((e) => {
  if (registryState) return;
  root.textContent = `Pinky Settings 초기화 실패: ${e.message}`;
});
