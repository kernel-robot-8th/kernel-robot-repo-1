// 노드 그래프를 캔버스에 그린다(route.html·fleet.html 공용). 인라인 코드·CDN 없음.
// 좌표는 실측 cm(배치 yaml), 위 = 창가. 경로 연결 검사는 로봇의 line_nav 가 최종으로 한다.
//   mode "build": 노드를 차례로 눌러 경로를 만든다(route.html)
//   mode "goal" : 정차 노드(목적지)만 누를 수 있다. 누르면 onGoal(id) (fleet.html, 경로는 서버가 계산)
// 로봇 표시: setRobots([{name, color, pose:{x_cm,y_cm,heading_deg}, traversed:[[from,to,frac],...]}])
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

  const COLORS = {
    bg: "#ffffff", grid: "#eef0f3", tape: "#9cc7b4", node: "#ffffff", text: "#222222",
    route: "#e1702b", sw: "#2e4fd8", goal: "#e1702b",
  };
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
      const s = Math.min((W - 2 * m) / 444.5, (H - 2 * m) / 221);
      return { s, ox: (W - 444.5 * s) / 2, oy: (H + 221 * s) / 2 };
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
      const { s, ox, oy } = this._scale();
      c.strokeStyle = COLORS.grid;
      c.lineWidth = 1;
      c.strokeRect(ox, oy - 221 * s, 444.5 * s, 221 * s);
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
      // 로봇(추정 위치 + 방향)
      this.robots.forEach((rb, i) => {
        if (!rb.pose) return;
        const color = rb.color || ROBOT_COLORS[i % ROBOT_COLORS.length];
        const [x, y] = this._pt(rb.pose.x_cm, rb.pose.y_cm);
        const a = -(rb.pose.heading_deg || 0) * Math.PI / 180;   // 화면 y 는 아래가 +
        c.save();
        c.translate(x, y);
        c.rotate(a);
        c.fillStyle = color;
        c.strokeStyle = "#ffffff";
        c.lineWidth = 2;
        c.beginPath();
        c.moveTo(13, 0); c.lineTo(-8, 8); c.lineTo(-4, 0); c.lineTo(-8, -8); c.closePath();
        c.fill(); c.stroke();
        c.restore();
        c.font = "bold 12px sans-serif";
        c.fillStyle = color;
        c.fillText(rb.name || "", x, y + 24);
        c.font = "11px sans-serif";
      });
    }
  }

  return { getJson, postJson, setMsg, markConnection, RouteMap, traversed, ROBOT_COLORS };
})();
