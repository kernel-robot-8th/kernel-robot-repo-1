// pinky_test_console Pinky 설정(노트북). 등록·삭제 /api/fleet/robots, 작업 /api/fleet/job, 기록 /api/fleet 의 jobs.
"use strict";

const $ = (id) => document.getElementById(id);
const POLL_MS = 1500;
const ROLE = { logistics: "물류", smartcart: "스마트카트" };
const ACTIONS = [
  ["check", "연결 확인"], ["read_domain", "도메인 읽기"], ["set_domain", "도메인 적용"],
  ["deploy", "업데이트 설치"], ["release", "릴리즈 설치"], ["build", "빌드"],
];

async function job(name, action) {
  const body = { name, action: action === "release" ? "deploy" : action };
  if (action === "release") body.release = true;
  if (action === "deploy") {
    const ref = $("ref").value;
    if (!ref) { alert("업데이트 설치 브랜치를 고르세요"); return; }
    body.ref = ref;
  }
  if (action === "set_domain" && !confirm(`${name}의 ~/.bashrc에 ROS_DOMAIN_ID 한 줄을 넣습니다. 진행할까요?`)) return;
  if ((action === "deploy" || action === "release") && !confirm(`${name}에 ${body.ref || "origin/main"}을 설치합니다. 로봇 ~/pinky_test_ws/src의 같은 패키지 파일을 덮어씁니다. 진행할까요?`)) return;
  const r = await RM.postJson("/api/fleet/job", body);
  if (!r.success) alert(`실패: ${r.msg || r.httpStatus}`);
  refresh();
}

async function remove(name) {
  if (!confirm(`${name} 등록을 지울까요? (로봇 자체는 바뀌지 않음)`)) return;
  const r = await RM.postJson("/api/fleet/robots/delete", { name });
  if (!r.success) alert(`실패: ${r.msg || r.httpStatus}`);
  refresh();
}

function edit(r) {
  $("fName").value = r.name;
  $("fIp").value = r.ip;
  $("fRole").value = r.role;
  $("fDomain").value = r.domain;
  $("fCalib").value = r.calib;
  $("fRmw").value = r.rmw || "";
}

$("btnSave").addEventListener("click", async () => {
  const body = {
    name: $("fName").value.trim(), ip: $("fIp").value.trim(), role: $("fRole").value,
    domain: Number($("fDomain").value), calib: $("fCalib").value, rmw: $("fRmw").value,
  };
  const r = await RM.postJson("/api/fleet/robots", body);
  RM.setMsg($("formMsg"), r.success ? `저장: ${body.name}` : `실패: ${r.msg || r.httpStatus}`, r.success ? "ok" : "err");
  refresh();
});

async function loadBranches() {
  const r = await RM.getJson("/api/fleet/branches");
  const sel = $("ref");
  sel.replaceChildren();
  const first = document.createElement("option");
  first.value = "";
  first.textContent = r.httpStatus === 200 ? "브랜치 선택…" : `불러오기 실패: ${r.msg || r.httpStatus}`;
  sel.appendChild(first);
  for (const b of r.branches || []) {
    const o = document.createElement("option");
    o.value = b;
    o.textContent = b;
    sel.appendChild(o);
  }
}
$("btnBranches").addEventListener("click", loadBranches);

function cell(tr, text) {
  const td = document.createElement("td");
  td.textContent = text;
  tr.appendChild(td);
  return td;
}

function button(td, text, fn, disabled) {
  const b = document.createElement("button");
  b.type = "button";
  b.className = "btn btn-outline btn-small";
  b.textContent = text;
  b.disabled = !!disabled;
  b.addEventListener("click", fn);
  td.appendChild(b);
}

function render(snap) {
  $("fleetFile").textContent = snap.fleet_file || "-";
  $("pkgs").textContent = (snap.packages || []).join(", ");
  $("releaseRef").textContent = snap.release_ref || "origin/main";
  const tb = $("robots").querySelector("tbody");
  tb.replaceChildren();
  for (const r of snap.robots || []) {
    const tr = document.createElement("tr");
    cell(tr, r.name);
    cell(tr, ROLE[r.role] || r.role);
    cell(tr, r.ip);
    cell(tr, String(r.domain));
    cell(tr, r.rmw ? `${r.calib} · ${r.rmw}` : r.calib);
    cell(tr, r.status ? (r.status.ok ? "OK" : "꺼짐") : "-");
    const td = cell(tr, "");
    for (const [a, label] of ACTIONS) button(td, label, () => job(r.name, a), r.busy);
    button(td, "수정", () => edit(r));
    button(td, "삭제", () => remove(r.name), r.busy);
    tb.appendChild(tr);
  }
  const box = $("jobs");
  box.replaceChildren();
  for (const j of (snap.jobs || []).slice().reverse()) {
    const sec = document.createElement("details");
    if (j.state === "running") sec.open = true;
    const sum = document.createElement("summary");
    const when = new Date(j.started * 1000).toLocaleTimeString();
    sum.textContent = `${when} ${j.robot} ${j.action}${j.params.ref ? " " + j.params.ref : ""} — ${j.state}${j.code !== null ? " (" + j.code + ")" : ""}`;
    sec.appendChild(sum);
    const pre = document.createElement("pre");
    pre.className = "cmd-box";
    pre.textContent = j.log.join("\n");
    sec.appendChild(pre);
    box.appendChild(sec);
  }
}

async function refresh() {
  try {
    const snap = await RM.getJson("/api/fleet");
    RM.markConnection(snap.httpStatus === 200);
    if (snap.httpStatus === 200) render(snap);
  } catch (_) {
    RM.markConnection(false);
  }
}

async function poll() {
  await refresh();
  setTimeout(poll, POLL_MS);
}

loadBranches();
poll();
