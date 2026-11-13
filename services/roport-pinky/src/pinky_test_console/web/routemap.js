// 노드 그래프를 캔버스에 그린다(route.html·fleet.html 공용). 인라인 코드·CDN 없음.
// 좌표는 실측 cm(배치 yaml), 위 = 창가. 경로 연결 검사는 로봇의 line_nav 가 최종으로 한다.
//   mode "build": 노드를 차례로 눌러 경로를 만든다(route.html)
//   mode "goal" : 정차 노드(목적지)만 누를 수 있다. 누르면 onGoal(id) (fleet.html, 경로는 서버가 계산)
// 로봇 표시: setRobots([{name, color, pose:{x_cm,y_cm,heading_deg}, traversed:[[from,to,frac],...],
//                       nav_pose:{x_cm,y_cm,heading_deg} (Nav2 위치, 나중에 스마트카트 게이트 시험)}])
// 배치(벽·물체·03 구역·마커)는 /api/route/graph 의 walls·objects·zones·markers 로 그린다(전체 지도).
"use strict";

const RM = (() => {
  async function getJson(url) {
    const res = await fetch(url, { cache: "no-store" });
    let data = {};
    try { data = await res.json(); } catch (_) { /* 빈 응답 */ }
    data.httpStatus = res.status;
    return data;
  }

  async function postJson(url, body) {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    let data = {};
    try { data = await res.json(); } catch (_) { /* 빈 응답 */ }
    data.httpStatus = res.status;
    return data;
  }

  function setMsg(el, text, kind) {
    el.classList.remove("ok", "err");
    if (kind) el.classList.add(kind);
    el.textContent = text;
  }

  function markConnection(ok) {
    const dot = document.getElementById("connDot");
    const txt = document.getElementById("connText");
    if (dot) dot.className = "chip-dot " + (ok ? "live" : "dead");
    if (txt) txt.textContent = ok ? "연결됨" : "연결 끊김";
  }

  const TERMINAL_DONE = ["arrived", "handoff"];

  // line_nav/state → 지나온 구간 [[from, to, 비율 0~1], ...]. 새 경로가 시작되면 route 가 바뀌어 처음부터.
  function traversed(st) {
    if (!st || !Array.isArray(st.route) || st.route.length < 2) return [];
    const legs = [];
    for (let i = 0; i + 1 < st.route.length; i++) legs.push([st.route[i], st.route[i + 1]]);
    if (TERMINAL_DONE.includes(st.state)) return legs.map(([a, b]) => [a, b, 1]);
    const out = [];
    const idx = Math.min(st.index || 0, legs.length - 1);
    for (let i = 0; i < idx; i++) out.push([legs[i][0], legs[i][1], 1]);
    if (st.state !== "idle") out.push([legs[idx][0], legs[idx][1], Math.max(0, Math.min(1, st.leg_progress || 0))]);
    return out;
  }

  // 로봇이 끝났다고 기억하는 노드(FFT1-172 후속). 필드 이름이 바뀌면 이 함수만 고친다.
  // state.end {node, facing} 를 우선 읽고, 없으면(옛 line_nav) arrived·loading 일 때 route 마지막 노드로 대신한다.
  // handoff 는 옛 동작 호환으로 포함. 반환: {node, facing|null} 또는 null.
  function endMemory(st) {
    if (!st) return null;
    if ("end" in st) {   // 새 line_nav: end 키가 있으면 그 값만 쓴다(null = 기억 없음)
      const e = st.end;
      if (e && typeof e === "object" && typeof e.node === "string" && e.node) {
        return { node: e.node, facing: e.facing === undefined ? null : e.facing };
      }
      return null;
    }
    if (Array.isArray(st.route) && st.route.length && ["arrived", "loading", "handoff"].includes(st.state)) {
      return { node: st.route[st.route.length - 1], facing: null };
    }
    return null;
  }

  // 출발 노드가 끝 기억과 다르면 확인창에 쓸 문구, 같거나 기억이 없으면 null.
  function startMismatchMsg(st, startNode) {
    const e = endMemory(st);
    if (!e || !startNode || e.node === startNode) return null;
    return `로봇이 ${e.node} 에서 끝났다고 기억 중입니다. 로봇을 ${startNode} 에 옮겨 놓았나요?`;
  }

  // 후진 양보(FFT1-172): 「후진」으로 돌아갈 수 있는 지나온 노드(가장 최근 먼저, 현재 노드 제외).
  // paused = route[0..index] (index 구간의 출발 노드 포함), arrived·loading = 마지막(현재) 노드만 뺀 앞쪽.
  // 그 밖의 상태는 후진을 시작할 수 없어 빈 목록. 최종 판단(지나온 경로 여부·자세)은 로봇의 line_nav 가 한다.
  // state.back_targets(line_nav 가 지금 back 으로 받는 노드, 최근 순)가 배열이면 그것만 쓴다(빈 배열 = 후진 불가).
  // 필드가 없을 때(옛 line_nav)만 아래 경로 기반 계산으로 대신한다.
  function backCandidates(st) {
    if (st && Array.isArray(st.back_targets)) {
      const out = [];
      for (const id of st.back_targets) if (typeof id === "string" && id && !out.includes(id)) out.push(id);
      return out;
    }
    if (!st || !Array.isArray(st.route) || st.route.length < 2) return [];
    const r = st.route;
    let passed;
    if (st.state === "paused") passed = r.slice(0, Math.max(0, Math.min(st.index || 0, r.length - 1)) + 1);
    else if (st.state === "arrived" || st.state === "loading") passed = r.slice(0, -1);
    else return [];
    const here = st.state === "paused" ? null : r[r.length - 1];
    const out = [];
    for (let i = passed.length - 1; i >= 0; i--) {
      if (passed[i] !== here && !out.includes(passed[i])) out.push(passed[i]);
    }
    return out;
  }

  // state.reverse {to, leg:[A,B], expected_cm, done_cm, y_cm, theta_deg} → 표시 문자열(없으면 "-")
  function reverseText(st) {
    const rv = st && st.reverse;
    if (!rv || typeof rv !== "object") return "-";
    const n = (v, d) => (typeof v === "number" && isFinite(v) ? v.toFixed(d) : "-");
    const leg = Array.isArray(rv.leg) ? `${rv.leg[1]}→${rv.leg[0]}` : "-";
    const sign = (v) => (typeof v === "number" && v > 0 ? "+" : "");
    return `목표 ${rv.to || "-"} · 구간 ${leg} · ${n(rv.done_cm, 1)} / ${n(rv.expected_cm, 1)} cm`
      + ` · 옆 ${typeof rv.y_cm === "number" ? sign(rv.y_cm) : ""}${n(rv.y_cm, 1)} cm`
      + ` · 방향 ${typeof rv.theta_deg === "number" ? sign(rv.theta_deg) : ""}${n(rv.theta_deg, 1)}°`;
  }

  // 후진 노드 선택 상자를 후보로 다시 채운다(같은 목록이면 그대로, 고른 값이 남아 있으면 유지, 아니면 가장 최근).
  function fillBackSelect(sel, cands) {
    const key = cands.join(",");
    if (sel.dataset.key === key) return;
    const keep = cands.includes(sel.value) ? sel.value : "";
    sel.replaceChildren();
    for (const id of cands) {
      const o = document.createElement("option");
      o.value = id;
      o.textContent = id;
      sel.appendChild(o);
    }
    sel.dataset.key = key;
    sel.value = keep || (cands.length ? cands[0] : "");
  }

  const COLORS = {
    bg: "#ffffff", grid: "#eef0f3", tape: "#9cc7b4", node: "#ffffff", text: "#222222",
    route: "#e1702b", sw: "#2e4fd8", goal: "#e1702b", wall: "#2f3640", zone: "#eef0fb",
    object: "#ddd8c8", objectEdge: "#8a8577", marker: "#b4441c",
  };

  // map(m) → 실측(cm): 배치 yaml frame 의 역변환(map = R(yaw)·실측/100 + (x_m, y_m))
  function mapToCm(frame, x, y) {
    const yaw = ((frame && frame.yaw_deg) || 0) * Math.PI / 180;
    const dx = x - ((frame && frame.x_m) || 0), dy = y - ((frame && frame.y_m) || 0);
    return [(Math.cos(yaw) * dx + Math.sin(yaw) * dy) * 100, (-Math.sin(yaw) * dx + Math.cos(yaw) * dy) * 100];
  }
  const ROBOT_COLORS = ["#d9480f", "#2e4fd8", "#9c36b5", "#0b7285"];

  class RouteMap {
    constructor(canvas, onChange, opts) {
      this.canvas = canvas;
      this.ctx = canvas.getContext("2d");
      this.onChange = onChange || (() => {});
      this.mode = (opts && opts.mode) || "build";
      this.onGoal = (opts && opts.onGoal) || (() => {});
      this.graph = null;
      this.byId = {};
      this.route = [];
      this.goal = null;
      this.start = null;
      this.robots = [];
      this.editable = true;
      canvas.addEventListener("click", (ev) => this._click(ev));
    }

    setGraph(g) {
      this.graph = g;
      this.byId = {};
      for (const n of g.nodes) this.byId[n.id] = n;
      this.goals = new Set(g.goals || g.nodes.filter((n) => n.bar).map((n) => n.id));
      this.draw();
    }

    setRoute(ids) {
      this.route = ids.filter((id) => this.byId[id]);
      this.draw();
      this.onChange(this.route.slice());
    }

    setStartGoal(start, goal) {
      this.start = start;
      this.goal = goal;
      this.draw();
    }

    setRobots(robots) {
      this.robots = robots || [];
      this.draw();
    }

    // 예전 화면 호환: 한 대의 진행 상태를 로봇 표시로 바꾼다
    setProgress(state, name) {
      this.setRobots(state ? [{ name: name || "", color: ROBOT_COLORS[0], pose: state.pose_est, traversed: traversed(state) }] : []);
    }

    linked(a, b) {
      const na = this.byId[a];
      return !!na && na.links.includes(b);
    }

    _scale() {
      const m = 30, W = this.canvas.width, H = this.canvas.height;
      const r = (this.graph && this.graph.room) || [0, 0, 444.5, 221];
      const w = r[2] - r[0], h = r[3] - r[1];
      const s = Math.min((W - 2 * m) / w, (H - 2 * m) / h);
      return { s, ox: (W - w * s) / 2 - r[0] * s, oy: (H + h * s) / 2 + r[1] * s, room: r };
    }

    _pt(x, y) {
      const { s, ox, oy } = this._scale();
      return [ox + x * s, oy - y * s];
    }

    _xy(n) { return this._pt(n.x, n.y); }

    _click(ev) {
      if (!this.graph || !this.editable) return;
      const r = this.canvas.getBoundingClientRect();
      const px = (ev.clientX - r.left) * (this.canvas.width / r.width);
      const py = (ev.clientY - r.top) * (this.canvas.height / r.height);
      let best = null, bestD = 16;
      for (const n of this.graph.nodes) {
        if (this.mode === "goal" && !this.goals.has(n.id)) continue;
        const [x, y] = this._xy(n);
        const d = Math.hypot(px - x, py - y);
        if (d < bestD) { best = n; bestD = d; }
      }
      if (!best) return;
      if (this.mode === "goal") { this.onGoal(best.id); return; }
      const last = this.route[this.route.length - 1];
      if (best.id === last) this.route.pop();                 // 마지막 노드를 다시 누르면 취소
      else if (!last || this.linked(last, best.id)) this.route.push(best.id);
      else return;                                            // 이웃이 아니면 무시
      this.draw();
      this.onChange(this.route.slice());
    }

    _line(a, b, frac, color, width) {
      const c = this.ctx;
      const [x1, y1] = this._xy(a), [x2, y2] = this._xy(b);
      c.strokeStyle = color;
      c.lineWidth = width;
      c.beginPath(); c.moveTo(x1, y1); c.lineTo(x1 + (x2 - x1) * frac, y1 + (y2 - y1) * frac); c.stroke();
    }

    draw() {
      const c = this.ctx, W = this.canvas.width, H = this.canvas.height;
      c.fillStyle = COLORS.bg;
      c.fillRect(0, 0, W, H);
      if (!this.graph) return;
      const { s, room } = this._scale();
      const rect = (x0, y0, x1, y1) => {
        const [ax, ay] = this._pt(x0, y1), [bx, by] = this._pt(x1, y0);
        return [ax, ay, bx - ax, by - ay];
      };
      // 03 공용 내부공간(Nav2 구역)
      c.font = "bold 12px sans-serif";
      c.textAlign = "center";
      for (const z of this.graph.zones || []) {
        c.fillStyle = COLORS.zone;
        c.fillRect(...rect(z.x0, z.y0, z.x1, z.y1));
        const [tx, ty] = this._pt((z.x0 + z.x1) / 2, z.y1 - 12);
        c.fillStyle = COLORS.sw;
        c.fillText(z.name, tx, ty);
      }
      // 물체(상자·삼각대·선반·OMX 받침·창고박스)
      c.font = "10px sans-serif";
      for (const o of this.graph.objects || []) {
        const r = rect(o.min[0], o.min[1], o.max[0], o.max[1]);
        c.fillStyle = COLORS.object;
        c.fillRect(...r);
        c.strokeStyle = COLORS.objectEdge;
        c.lineWidth = 1;
        c.setLineDash(o.provisional ? [4, 3] : []);
        c.strokeRect(...r);
        c.setLineDash([]);
      }
      // 벽·칸막이
      c.strokeStyle = COLORS.wall;
      c.lineWidth = 3;
      c.lineCap = "square";
      for (const w of this.graph.walls || []) {
        const [x1, y1] = this._pt(w.from[0], w.from[1]), [x2, y2] = this._pt(w.to[0], w.to[1]);
        c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke();
      }
      if (!(this.graph.walls || []).length) {
        c.strokeStyle = COLORS.grid;
        c.lineWidth = 1;
        c.strokeRect(...rect(room[0], room[1], room[2], room[3]));
      }
      // 마커(게이트 G1~G3·ARR·P1~P7)
      for (const m of this.graph.markers || []) {
        const [x, y] = this._pt(m.center[0], m.center[1]);
        c.fillStyle = COLORS.marker;
        c.fillRect(x - 3, y - 3, 6, 6);
        if (!m.floor && /^(G\d|ARR)$/.test(m.name)) {
          c.fillText(m.name, x + (x > this.canvas.width / 2 ? -16 : 16), y + 4);
        }
      }
      const tapeW = Math.max(4, 2.4 * s);
      c.lineCap = "round";
      // 기본 라인(테이프)
      const seen = new Set();
      for (const n of this.graph.nodes) {
        for (const o of n.links) {
          const key = [n.id, o].sort().join("-");
          if (seen.has(key) || !this.byId[o]) continue;
          seen.add(key);
          this._line(n, this.byId[o], 1, COLORS.tape, tapeW);
        }
        if (n.bar) {
          const [x, y] = this._xy(n), h = 6 * s;
          c.strokeStyle = COLORS.tape;
          c.lineWidth = tapeW;
          c.beginPath();
          if (n.bar === "x") { c.moveTo(x - h, y); c.lineTo(x + h, y); } else { c.moveTo(x, y - h); c.lineTo(x, y + h); }
          c.stroke();
        }
      }
      // 로봇별 지나온 구간(로봇 색). 새 경로가 시작되면 traversed 가 비워져 기본 색으로 돌아감
      this.robots.forEach((rb, i) => {
        const color = rb.color || ROBOT_COLORS[i % ROBOT_COLORS.length];
        for (const [a, b, f] of rb.traversed || []) {
          if (this.byId[a] && this.byId[b]) this._line(this.byId[a], this.byId[b], f, color, tapeW + 3);
        }
      });
      // 계획한 경로(점선)
      if (this.route.length > 1) {
        c.strokeStyle = COLORS.route;
        c.lineWidth = 2.5;
        c.setLineDash([7, 5]);
        c.beginPath();
        this.route.forEach((id, i) => {
          const [x, y] = this._xy(this.byId[id]);
          if (i === 0) c.moveTo(x, y); else c.lineTo(x, y);
        });
        c.stroke();
        c.setLineDash([]);
      }
      // 노드
      c.font = "11px sans-serif";
      c.textAlign = "center";
      for (const n of this.graph.nodes) {
        const [x, y] = this._xy(n);
        const selectable = this.mode === "goal" && this.goals.has(n.id);
        c.beginPath();
        c.arc(x, y, selectable ? 8 : 6, 0, 2 * Math.PI);
        c.fillStyle = n.id === this.goal ? COLORS.goal : n.id === this.start ? "#ffd8a8"
          : (n.nav2_switch ? COLORS.sw : COLORS.node);
        c.fill();
        c.lineWidth = selectable ? 2.5 : 1.5;
        c.strokeStyle = selectable ? COLORS.goal : (n.nav2_switch ? COLORS.sw : "#5c7f70");
        c.stroke();
        c.fillStyle = COLORS.text;
        c.fillText(n.id + (n.place ? ` (${n.place})` : ""), x, y - 12);
        const idx = this.mode === "build" ? this.route.indexOf(n.id) : -1;
        if (idx >= 0) { c.fillStyle = COLORS.route; c.fillText(String(idx + 1), x, y + 20); }
      }
      // 로봇(라인 주행 추정 위치 = 채운 화살표, Nav2 위치 = 빈 화살표)
      const arrow = (pose, color, filled) => {
        const [x, y] = this._pt(pose.x_cm, pose.y_cm);
        c.save();
        c.translate(x, y);
        c.rotate(-(pose.heading_deg || 0) * Math.PI / 180);   // 화면 y 는 아래가 +
        c.fillStyle = filled ? color : "#ffffff";
        c.strokeStyle = filled ? "#ffffff" : color;
        c.lineWidth = filled ? 2 : 2.5;
        c.beginPath();
        c.moveTo(13, 0); c.lineTo(-8, 8); c.lineTo(-4, 0); c.lineTo(-8, -8); c.closePath();
        c.fill(); c.stroke();
        c.restore();
        return [x, y];
      };
      this.robots.forEach((rb, i) => {
        const color = rb.color || ROBOT_COLORS[i % ROBOT_COLORS.length];
        const p = rb.nav_pose || rb.pose;
        if (rb.nav_pose) arrow(rb.nav_pose, color, false);
        if (rb.pose) arrow(rb.pose, color, true);
        if (!p) return;
        const [x, y] = this._pt(p.x_cm, p.y_cm);
        c.font = "bold 12px sans-serif";
        c.fillStyle = color;
        c.fillText(rb.name || "", x, y + 24);
        c.font = "11px sans-serif";
      });
    }
  }

  return { getJson, postJson, setMsg, markConnection, RouteMap, traversed, mapToCm, ROBOT_COLORS,
    backCandidates, reverseText, fillBackSelect, endMemory, startMismatchMsg };
})();
if (typeof module !== "undefined") module.exports = RM;
