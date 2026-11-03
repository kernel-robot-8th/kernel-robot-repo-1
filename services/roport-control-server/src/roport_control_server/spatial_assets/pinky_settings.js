// Pinky 탭 > Pinky Settings (시안 v0.2). OFFLINE = 실행부 미연결이라 등록·작업은 검사만 하고 503,
// DEMO = 이 브라우저 안에서만 바뀌는 예시 목록(새로고침하면 처음으로). 로봇·SSH·MQTT에 연결하지 않음.
// 실제 설치·도메인 적용은 실행부가 생기기 전까지 Pinky 테스트 콘솔(mode:=fleet)에서 한다.
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

async function boot() {
  const res = await fetch(`/api/pinky/settings?mode=${demo ? "demo" : "offline"}`, { cache: "no-store" });
  if (!res.ok) throw new Error(`pinky settings: ${res.status}`);
  view = await res.json();
  robots = view.robots;
  render();
}

boot().catch((e) => {
  root.textContent = `Pinky Settings 초기화 실패: ${e.message}`;
});
