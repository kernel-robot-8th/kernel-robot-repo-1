// 노드 그래프를 캔버스에 그리고 클릭으로 경로를 만든다(route.html·fleet.html 공용). 인라인 코드·CDN 없음.
// 좌표는 실측 cm(배치 yaml), 위 = 창가. 경로 연결 검사는 로봇의 line_nav 가 최종으로 한다.
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

  const COLORS = {
    bg: "#ffffff", grid: "#eef0f3", tape: "#1e6b52", node: "#ffffff", text: "#222222",
    route: "#e1702b", active: "#2e4fd8", sw: "#2e4fd8", place: "#8a5a00",
  };

  class RouteMap {
    constructor(canvas, onChange) {
      this.canvas = canvas;
      this.ctx = canvas.getContext("2d");
      this.onChange = onChange || (() => {});
      this.graph = null;
      this.byId = {};
      this.route = [];
      this.active = null;      // {from, to}
      this.done = new Set();   // 지나온 노드
      this.editable = true;
      canvas.addEventListener("click", (ev) => this._click(ev));
    }

    setGraph(g) {
      this.graph = g;
      this.byId = {};
      for (const n of g.nodes) this.byId[n.id] = n;
      this.draw();
    }

    setRoute(ids) {
      this.route = ids.filter((id) => this.byId[id]);
      this.draw();
      this.onChange(this.route.slice());
    }

    setProgress(state) {
      // state = line_nav/state JSON (없으면 null)
      this.active = null;
      this.done = new Set();
      if (state && state.leg && state.route && state.state !== "idle") {
        this.active = { from: state.leg.from, to: state.leg.to };
        for (let i = 0; i <= state.index && i < state.route.length; i++) this.done.add(state.route[i]);
      }
      this.draw();
    }

    linked(a, b) {
      const na = this.byId[a];
      return !!na && na.links.includes(b);
    }

    _scale() {
      const m = 28, W = this.canvas.width, H = this.canvas.height;
      const s = Math.min((W - 2 * m) / 444.5, (H - 2 * m) / 221);
      return { s, ox: (W - 444.5 * s) / 2, oy: (H + 221 * s) / 2 };
    }

    _xy(n) {
      const { s, ox, oy } = this._scale();
      return [ox + n.x * s, oy - n.y * s];
    }

    _click(ev) {
      if (!this.graph || !this.editable) return;
      const r = this.canvas.getBoundingClientRect();
      const px = (ev.clientX - r.left) * (this.canvas.width / r.width);
      const py = (ev.clientY - r.top) * (this.canvas.height / r.height);
      let best = null, bestD = 16;
      for (const n of this.graph.nodes) {
        const [x, y] = this._xy(n);
        const d = Math.hypot(px - x, py - y);
        if (d < bestD) { best = n; bestD = d; }
      }
      if (!best) return;
      const last = this.route[this.route.length - 1];
      if (best.id === last) this.route.pop();                 // 마지막 노드를 다시 누르면 취소
      else if (!last || this.linked(last, best.id)) this.route.push(best.id);
      else return;                                            // 이웃이 아니면 무시
      this.draw();
      this.onChange(this.route.slice());
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
      // 테이프(연결)
      const seen = new Set();
      c.lineCap = "square";
      for (const n of this.graph.nodes) {
        for (const o of n.links) {
          const key = [n.id, o].sort().join("-");
          if (seen.has(key) || !this.byId[o]) continue;
          seen.add(key);
          const [x1, y1] = this._xy(n), [x2, y2] = this._xy(this.byId[o]);
          const onActive = this.active && ((this.active.from === n.id && this.active.to === o)
            || (this.active.from === o && this.active.to === n.id));
          c.strokeStyle = onActive ? COLORS.active : COLORS.tape;
          c.lineWidth = Math.max(3, 2.4 * s);
          c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke();
        }
        if (n.bar) {
          const [x, y] = this._xy(n), h = 6 * s;
          c.strokeStyle = COLORS.tape;
          c.lineWidth = Math.max(3, 2.4 * s);
          c.beginPath();
          if (n.bar === "x") { c.moveTo(x - h, y); c.lineTo(x + h, y); } else { c.moveTo(x, y - h); c.lineTo(x, y + h); }
          c.stroke();
        }
      }
      // 계획한 경로
      if (this.route.length > 1) {
        c.strokeStyle = COLORS.route;
        c.lineWidth = 3;
        c.setLineDash([6, 4]);
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
        const idx = this.route.indexOf(n.id);
        c.beginPath();
        c.arc(x, y, 7, 0, 2 * Math.PI);
        c.fillStyle = idx >= 0 ? COLORS.route : (n.nav2_switch ? COLORS.sw : COLORS.node);
        if (this.done.has(n.id)) c.fillStyle = COLORS.active;
        c.fill();
        c.lineWidth = 2;
        c.strokeStyle = n.nav2_switch ? COLORS.sw : COLORS.tape;
        c.stroke();
        c.fillStyle = COLORS.text;
        const label = n.id + (n.place ? ` (${n.place})` : "");
        c.fillText(label, x, y - 11);
        if (idx >= 0) {
          c.fillStyle = "#ffffff";
          c.fillText(String(idx + 1), x, y + 4);
        }
      }
    }
  }

  return { getJson, postJson, setMsg, markConnection, RouteMap };
})();
