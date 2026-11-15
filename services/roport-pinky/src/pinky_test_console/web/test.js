// pinky_test_console 통합 테스트 화면(노트북). 탭마다 iframe 을 처음 열 때 만들고, 다른 탭으로 가도 그대로 둔다.
// 로봇 화면 탭은 /robot/<이름>/<화면> (노트북 중계). 로봇을 바꾸면 로봇 화면 탭만 다시 연다.
"use strict";

const $ = (id) => document.getElementById(id);
const NEEDS = { route: ["route", "test"], ir: ["ir", "test", "route"], console: ["slam", "nav"], amcl: ["amcl"] };
const frames = {};
let current = null;
let robots = [];

function frameFor(tab, src) {
  let f = frames[tab];
  if (!f || f.dataset.src !== src) {
    if (f) f.remove();
    f = document.createElement("iframe");
    f.className = "tt-frame";
    f.title = tab;
    f.dataset.src = src;
    f.src = src;
    if (!tab.startsWith("robot")) {
      f.addEventListener("load", () => {   // 같은 출처: 관제·설정 화면의 헤더만 숨김
        try { f.contentDocument.body.classList.add("tt-embedded"); } catch (_) { /* 그대로 둠 */ }
      });
    }
    $("frames").appendChild(f);
    frames[tab] = f;
  }
  return f;
}

function robotMode(name) {
  const r = robots.find((x) => x.name === name);
  return r && r.status && r.status.ok && r.status.route ? (r.status.route.mode || "?") : null;
}

function show(tab) {
  const btn = document.querySelector(`.tt-tab[data-tab="${tab}"]`);
  if (!btn) return;
  const robot = $("robotSel").value;
  let src = `${tab}.html`;
  if (btn.dataset.robot) {
    if (!robot) { $("modeHint").textContent = "등록된 로봇이 없음(Pinky 설정에서 등록)"; return; }
    src = `robot/${encodeURIComponent(robot)}/${btn.dataset.robot === "index.html" ? "" : btn.dataset.robot}`;
    const mode = robotMode(robot);
    $("modeHint").textContent = mode === null ? `${robot}: 콘솔 연결 안 됨`
      : NEEDS[tab].includes(mode) ? `${robot}: mode:=${mode}`
        : `${robot}: 지금 mode:=${mode} — 이 화면은 mode:=${NEEDS[tab].join("|")}에서 동작`;
  } else {
    $("modeHint").textContent = "";
  }
  for (const f of Object.values(frames)) f.hidden = true;
  frameFor(btn.dataset.robot ? `robot-${tab}` : tab, src).hidden = false;
  for (const b of document.querySelectorAll(".tt-tab")) b.setAttribute("aria-pressed", String(b === btn));
  current = tab;
  try { localStorage.setItem("pinky_test_tab", tab); } catch (_) { /* 저장 안 돼도 됨 */ }
}

for (const b of document.querySelectorAll(".tt-tab")) b.addEventListener("click", () => show(b.dataset.tab));
$("robotSel").addEventListener("change", () => {
  for (const b of document.querySelectorAll(".tt-tab[data-robot]")) {
    const f = frames[`robot-${b.dataset.tab}`];
    if (f) { f.remove(); delete frames[`robot-${b.dataset.tab}`]; }
  }
  if (current) show(current);
});

async function stopAll() {
  const res = await fetch("/api/fleet/stop_all", {
    method: "POST", headers: { "Content-Type": "application/json" }, body: "{}",
  });
  $("modeHint").textContent = res.ok ? "전체 정지 보냄" : "전체 정지 실패";
}
$("btnEstop").addEventListener("click", stopAll);
$("btnHomeAll").addEventListener("click", async () => {
  if (!confirm("노드에서 끝난 모든 로봇을 각자 복귀 칸으로 보냅니다. 로봇끼리 충돌 회피는 없으니 경로가 겹치지 않는지 확인하세요. 진행할까요?")) return;
  const res = await fetch("/api/fleet/return", {
    method: "POST", headers: { "Content-Type": "application/json" }, body: "{}",
  });
  const data = await res.json().catch(() => ({}));
  $("modeHint").textContent = res.ok
    ? Object.entries(data.result || {}).map(([n, x]) => `${n}: ${x.msg}`).join(" / ")
    : `전체 복귀 실패: ${data.msg || res.status}`;
});
document.addEventListener("keydown", (ev) => {
  if ((ev.code === "Space" || ev.code === "Escape") && !["INPUT", "SELECT"].includes(ev.target.tagName)) {
    ev.preventDefault();
    stopAll();
  }
});

async function refreshRobots() {
  try {
    const res = await fetch("/api/fleet", { cache: "no-store" });
    const snap = await res.json();
    robots = snap.robots || [];
    const sel = $("robotSel");
    const keep = sel.value;
    const names = robots.map((r) => r.name);
    if ([...sel.options].map((o) => o.value).join() !== names.join()) {
      sel.replaceChildren();
      for (const n of names) {
        const o = document.createElement("option");
        o.value = n;
        o.textContent = n;
        sel.appendChild(o);
      }
      if (names.includes(keep)) sel.value = keep;
    }
  } catch (_) { /* 다음 주기에 다시 */ }
}

async function init() {
  await refreshRobots();
  setInterval(refreshRobots, 3000);
  let tab = "fleet";
  try { tab = localStorage.getItem("pinky_test_tab") || "fleet"; } catch (_) { /* 기본 탭 */ }
  show(tab);
}

init();
