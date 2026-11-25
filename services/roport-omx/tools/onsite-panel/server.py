#!/usr/bin/env python3
"""OMX On-site Control Panel: Host PC에서 관제 통신 기반 미션을 시험·운영하는 작은 웹 화면 (127.0.0.1 전용).

- Host(cooperative_host.py) 시작·정지: 로컬 브로커(127.0.0.1:1883) 또는 관제 PC 브로커
- 제어판(127.0.0.1:8865) armed 켜기/끄기, 홈 위치로 이동, 중지
- HOLD 해제(제어판 [HOLD 해제]와 같은 경로) · 강제 해제(Host를 --clear-hold로 다시 시작)
- v2(기본, roport-control-v2): 패널 안의 가상 관제(v2control.py, 미니 Fleet)가 상태 조회·입고·하차·
  정지·HOLD 해제 명령과 1초 작업 유지 확인·ACK를 보내고, 로컬 브로커에서는 Pinky 상태도 흉내 낸다.
  관제 PC 브로커에서는 보기만 한다(실제 Fleet이 명령).
- v1(--protocol v1): 시험용 관제 송신기(cooperative_control_sim.py)로 status / 입고 / 하차

설정은 제어판 Host 실행기와 같은 ~/.config/roport/omx-host.json을 읽는다(site/control/binding은
link_config). 브로커 인증값은 읽지 않는다(로컬·무인증 시험 브로커만).

실행:  ./run.sh   (또는 uv run --no-sync --project <repo> python server.py)  →  http://127.0.0.1:8870
"""

from __future__ import annotations

import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
PORT = int(os.environ.get("TESTPANEL_PORT", "8870"))
PANEL = os.environ.get("TESTPANEL_PANEL_URL", "http://127.0.0.1:8865")
CONFIG = Path.home() / ".config/roport/omx-host.json"
LOCAL_BROKER = {"host": "127.0.0.1", "port": 1883}
HOLD_CHECKS = ("arms_safe", "package_checked", "pinky_checked")
# latest front_2 photo the shelf aligner saved (shows where boxes are now). ONSITE_ALIGN_RUNS changes the folder;
# with none the shelf picker shows the slot buttons on a dark background (no real photo is kept in the repository)
RUNS = Path(os.path.expanduser(os.environ.get("ONSITE_ALIGN_RUNS", "~/il_ws/src/omx_sim_align/runs")))
ARMS_CACHE_S = 4.0



def _local_hms(stamp) -> str:
    """Message times are UTC ("...Z"); show them in this PC's time (KST) like the rest of the page."""
    if not stamp:
        return ""
    try:
        return datetime.fromisoformat(str(stamp).replace("Z", "+00:00")).astimezone().strftime("%H:%M:%S")
    except ValueError:
        return str(stamp)[11:19]

def _path(p) -> Path:
    return Path(os.path.expanduser(str(p)))


def load_settings() -> dict:
    raw = json.loads(CONFIG.read_text())
    # link_config (site / control / binding) is only for v1; a v2-only setup (config/omx-host.example.json) has none
    link = json.loads(_path(raw["link_config"]).read_text()) if raw.get("link_config") else {}
    broker = {}
    env_file = _path(raw["broker_env"])
    if env_file.exists():
        for line in env_file.read_text().splitlines():
            m = re.match(r"\s*(?:export\s+)?ROPORT_MQTT_(HOST|PORT|TLS|CA_FILE|USERNAME|PASSWORD)=(.*)", line)
            if m:
                broker[m.group(1)] = m.group(2).strip().strip("\"'")
    journal = _path(raw["journal"])
    # v2 keeps its own journal (the Host refuses the other protocol's), same rule as the panel's
    # Host launcher (omx_host.py); the simulated executor gets a separate one so a mock HOLD never
    # reaches the hardware Host
    journal_v2 = _path(raw["journal_v2"]) if raw.get("journal_v2") else journal.with_name(journal.stem + "-v2" + journal.suffix)
    return {
        "repo": _path(raw["repo"]),
        "routes": _path(raw["routes"]),
        "journal": journal,
        "journal_v2": journal_v2,
        "journal_v2_mock": journal_v2.with_name(journal_v2.stem + "-mock" + journal_v2.suffix),
        "control_v2": str(raw.get("control") or "control-01"),
        "run_dir": journal.parent,
        "client_id": raw.get("client_id", "roport-shelf-01-host"),
        "site": link.get("site_id") or "shelf-01",
        "control": link.get("control_id"),
        "binding": (link.get("omx") or {}).get("binding_id"),
        "control_broker": {
            "host": broker.get("HOST"),
            "port": int(broker.get("PORT") or 1883),
            "tls": broker.get("TLS") == "1",
            "ca_file": broker.get("CA_FILE") or None,
            "username": broker.get("USERNAME") or None,
        },
        # kept out of the screen's state; handed to the Host only through its environment
        "control_password": broker.get("PASSWORD") or None,
    }


S = load_settings()
sys.path.insert(0, str(S["repo"] / "src"))
try:  # the Host's own route rules (episode_s default, markers, modes)
    try:  # main (#180): the v2 executor is shelf_panel; panel_executor there is the old v1 one
        from roport_omx.shelf_panel import PanelExecutor, load_routes
    except ImportError:  # task/field checkouts: panel_executor is the v2 executor
        from roport_omx.panel_executor import PanelExecutor, load_routes
except Exception as exc:  # noqa: BLE001 - shown on screen; the panel still works without it
    PanelExecutor = load_routes = None
    IMPORT_ERROR = f"{type(exc).__name__}: {exc}"
else:
    IMPORT_ERROR = None

try:
    from v2control import V2Control
except Exception as exc:  # noqa: BLE001
    V2Control, V2_ERROR = None, f"{type(exc).__name__}: {exc}"
else:
    V2_ERROR = None
HOST_KNOWS_V2 = '"--protocol"' in (S["repo"] / "scripts/cooperative_host.py").read_text()

UV = shutil.which("uv") or str(Path.home() / ".local/bin/uv")
LOCK = threading.Lock()
STATE = {
    "host_log": None,  # log file of the Host this panel started
    "mission": {"running": False, "proc": None, "lines": [], "op": None, "rc": None, "log": None},
    "arms": {"at": 0.0, "value": None},
    # camera watch, as the control panel's own page does it (omx_panel.html watchCams), but here in the
    # server so it also runs with no browser open: a USB change that left saved cameras missing gets
    # the panel's auto-detect once it settled (3 s) and nothing holds the cameras
    "cams": {"missing": [], "unconfirmed": False, "sig": "", "since": 0.0, "done": "", "last": None, "error": None,
             "roles": [], "roles_at": 0.0},
}


# ---------------------------------------------------------------- helpers


def say(text: str) -> None:
    """A line in the mission event box."""
    with LOCK:
        m = STATE["mission"]
        m["lines"].append(f"[{now()}] {text}")
        m["lines"] = m["lines"][-300:]


V2 = V2Control(S["control_v2"], S["run_dir"], say) if V2Control else None


def versions_for(op: str, slot: str) -> tuple[str, str]:
    """model/calibration version the v2 Host accepts for this route (route value, else file default)."""
    raw = json.loads(S["routes"].read_text())
    for r in raw.get("routes", []):
        if r.get("operation") == op and r.get("slot_id") == slot:
            model = r.get("model_version") or raw.get("model_version")
            cal = r.get("calibration_version") or raw.get("calibration_version")
            if model and cal:
                return model, cal
    raise RuntimeError(f"경로표에 {op} {slot}의 model_version/calibration_version이 없습니다.")


def now() -> str:
    return datetime.now().strftime("%H:%M:%S")


def panel(method: str, path: str, body: dict | None = None, timeout: float = 5.0) -> dict:
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(
        PANEL + path, data=data, method=method, headers={"Content-Type": "application/json"}
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as exc:
        try:
            reply = json.loads(exc.read() or b"{}")
        except ValueError:
            reply = {}
        raise RuntimeError(reply.get("error") or f"제어판 {path}: HTTP {exc.code}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"제어판에 연결하지 못했습니다 ({PANEL}): {exc.reason}") from exc


def host_procs() -> list[dict]:
    """Running Host processes (the Python process itself, not the uv wrapper)."""
    found = []
    for d in Path("/proc").iterdir():
        if not d.name.isdigit():
            continue
        try:
            args = [a.decode(errors="replace") for a in (d / "cmdline").read_bytes().split(b"\0") if a]
        except OSError:
            continue
        if len(args) < 2 or not re.search(r"/python[0-9.]*$", args[0]):
            continue
        if not any(a.endswith("scripts/cooperative_host.py") for a in args[1:]):
            continue

        def opt(name, default=None, args=args):
            return args[args.index(name) + 1] if name in args and args.index(name) + 1 < len(args) else default

        host = opt("--host")
        protocol = opt("--protocol")
        if protocol is None:
            # no flag: v2 only if the process started after the Host code learned v2 (#135)
            script = S["repo"] / "scripts/cooperative_host.py"
            protocol = "v2" if HOST_KNOWS_V2 and d.stat().st_mtime >= script.stat().st_mtime else "v1"
        found.append(
            {
                "pid": int(d.name),
                "started": datetime.fromtimestamp(d.stat().st_mtime).strftime("%m-%d %H:%M:%S"),
                "broker": f"{host}:{opt('--port', '1883')}" if host else "환경변수(관제 PC 브로커)",
                "local": host in ("127.0.0.1", "localhost"),
                "end_on_place": "--no-end-on-place" not in args,
                "report_placement": "--report-placement" in args,  # off by default since #134
                "protocol": protocol,
                "executor": opt("--executor", "mock"),
                "journal": opt("--journal"),
                "args": args[1:],
            }
        )
    return found


def current_journal() -> Path:
    """The running Host's journal; with no Host, the v2 hardware journal (the default)."""
    procs = host_procs()
    if procs and procs[0]["journal"]:
        return _path(procs[0]["journal"])
    return S["journal_v2"] if HOST_KNOWS_V2 else S["journal"]


def channel_file(kind: str) -> Path:
    stem = current_journal().with_suffix("")
    return stem.with_name(f"{stem.name}.{kind}.json")


def host_state() -> dict:
    stem = current_journal().with_suffix("")
    path = stem.with_name(stem.name + ".host-state.json")
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def _model_name(path: str) -> str:
    """The model folder of a route's model_path (…/<model>/pretrained_model → <model>); a short placeholder as it is."""
    parts = [p for p in str(path).rstrip("/").split("/") if p]
    return parts[-2] if len(parts) >= 2 else (parts[-1] if parts else "")


def routes() -> list[dict]:
    out = []
    try:
        if load_routes is not None:
            for (op, slot, mode), r in load_routes(S["routes"]).items():
                out.append(
                    {"op": op, "slot": slot, "mode": mode, "markers": sorted(r.markers) if r.markers else None,
                     "episode_s": r.episode_s, "model": _model_name(r.model_path)}
                )
        else:
            for r in json.loads(S["routes"].read_text())["routes"]:
                out.append({"op": r["operation"], "slot": r["slot_id"], "mode": r.get("manipulation_mode") or "",
                            "markers": r.get("markers"), "episode_s": r.get("episode_s"), "model": r["model_path"]})
    except Exception as exc:  # noqa: BLE001
        return [{"error": f"경로표를 읽지 못했습니다: {exc}"}]
    return out


def arms(force: bool = False) -> dict:
    """Live arm flags as the Host sees them (panel /api/arms against every route's home)."""
    cache = STATE["arms"]
    if not force and time.monotonic() - cache["at"] < ARMS_CACHE_S and cache["value"] is not None:
        return cache["value"]
    if PanelExecutor is None:
        value = {"error": IMPORT_ERROR}
    else:
        try:
            ready = PanelExecutor(load_routes(S["routes"])).readiness()
            value = {"arms": ready["arms"], "busy": ready["busy"], "error": ready["error"]}
        except Exception as exc:  # noqa: BLE001
            value = {"error": str(exc)}
    cache.update(at=time.monotonic(), value=value)
    return value


def arms_home(value: dict) -> bool:
    a = value.get("arms") or {}
    return set(a) == {"omx1", "omx2"} and all(
        v.get("connected") is True and v.get("returned") is True and v.get("grasping") is False
        for v in a.values()
    )


def camera_roles(force: bool = False) -> list[dict]:
    c = STATE["cams"]
    if force or time.monotonic() - c["roles_at"] > 5:
        try:
            setup = panel("GET", "/api/robot_setup", timeout=5)
            c["roles"] = [{"name": x["name"], "port": x["path"].split("usb-")[-1].replace(":1.0-video-index0", ""),
                           "label": x.get("label", ""), "present": bool(x.get("present"))}
                          for x in setup.get("active_cameras") or []]
            c["roles_at"] = time.monotonic()
        except RuntimeError as exc:
            c["error"] = str(exc)
    return c["roles"]


def run_detect(apply: bool, why: str) -> dict:
    res = panel("POST", "/api/camera_detect", {"apply": apply}, timeout=90)
    picks = {r: {"port": (p.get("path") or "").split("usb-")[-1].replace(":1.0-video-index0", "") or None,
                 "sure": p.get("sure"), "why": p.get("why")} for r, p in (res.get("picks") or {}).items()}
    last = {"at": now(), "why": why, "apply": apply, "applied": res.get("applied"), "sure": res.get("sure"),
            "changed": res.get("changed"), "picks": picks}
    STATE["cams"]["last"] = last
    camera_roles(force=True)
    if apply:
        if res.get("applied"):
            say("카메라: 포트가 바뀌어 자동으로 찾아 저장했습니다. 평가 전에 카메라 화면을 확인하고 [확인 후 저장]을 누르세요.")
        elif not res.get("sure"):
            say("카메라: 포트가 바뀌었는데 일부를 판별하지 못했습니다: "
                + ", ".join(f"{r} {p['why']}" for r, p in picks.items() if not p["sure"]))
    return last


def camera_watch_loop() -> None:
    """Every 2 s: the panel's camera_watch; auto-detect once per USB change (same rule as the panel page)."""
    c = STATE["cams"]
    while True:
        try:
            st = panel("GET", "/api/status", timeout=4)
            w = st.get("camera_watch") or {}
            c.update(missing=w.get("missing") or [], unconfirmed=bool(w.get("unconfirmed")), error=None)
            sig = w.get("sig") or ""
            if sig != c["sig"]:
                c["sig"], c["since"], c["done"] = sig, time.monotonic(), ""
                camera_roles(force=True)
            busy = bool(st.get("preview")) or bool((st.get("job") or {}).get("running"))
            if c["missing"] and sig != c["done"] and time.monotonic() - c["since"] >= 3 and not busy:
                c["done"] = sig
                run_detect(True, "USB 변경 자동")
        except Exception as exc:  # noqa: BLE001 - shown on screen; the watch goes on
            c["error"] = str(exc)[:200]
        time.sleep(2)


def confirm_cameras() -> dict:
    """The person looked at every camera's picture: save the current roles as they are (the panel's [저장]),
    which clears "화면 확인 필요" and takes the views now seen as the new reference."""
    setup = panel("GET", "/api/robot_setup", timeout=5).get("setup") or {}
    mode = setup.get("mode")
    body = {"mode": mode, "cameras": [{"path": x["path"], "role": x["name"], "label": x.get("label", "")}
                                      for x in (setup.get("cameras") or {}).get(mode) or []],
            "wrist_offset_deg": setup.get("wrist_offset_deg") or {}}
    if mode == "bimanual":
        body.update(left=(setup.get("bimanual") or {}).get("left"), right=(setup.get("bimanual") or {}).get("right"))
    else:
        body["set"] = (setup.get("single") or {}).get("set")
    res = panel("POST", "/api/robot_setup", body, timeout=30)
    camera_roles(force=True)
    say("카메라: 화면 확인 후 현재 배정을 저장했습니다(기준 화면 갱신).")
    return {"ok": True, "message": "카메라 배정을 저장했습니다.", "panel": {k: res.get(k) for k in ("ok",) if k in res}}


def tail(path: Path | None, n: int = 60) -> list[str]:
    if not path or not path.exists():
        return []
    lines = path.read_text(errors="replace").splitlines()[-400:]
    return [line for line in lines if "VIRTUAL_ENV" not in line][-n:]


def broker_for(choice: str) -> dict:
    """Broker address plus its TLS/login settings (control PC: from broker_env)."""
    if choice == "control":
        b = S["control_broker"]
        if not b["host"]:
            raise RuntimeError("broker_env에 ROPORT_MQTT_HOST가 없습니다.")
        if b["tls"] and not b["ca_file"]:
            raise RuntimeError("broker_env에 TLS=1인데 ROPORT_MQTT_CA_FILE이 없습니다.")
        return {"host": b["host"], "port": b["port"], "tls": b["tls"], "ca_file": b["ca_file"],
                "username": b["username"], "password": S["control_password"]}
    return dict(LOCAL_BROKER) | {"tls": False, "ca_file": None, "username": None, "password": None}


def broker_env(b: dict) -> dict:
    """ROPORT_MQTT_* for the Host process (the password never goes on its command line)."""
    env = {"ROPORT_MQTT_TLS": "1" if b["tls"] else "0"}
    for key, name in (("ca_file", "CA_FILE"), ("username", "USERNAME"), ("password", "PASSWORD")):
        if b.get(key):
            env["ROPORT_MQTT_" + name] = str(b[key])
    return env


# ---------------------------------------------------------------- Host


def start_host(broker: str, end_on_place: bool, clear_hold: bool, report_placement: bool = False,
               protocol: str = "v2", executor: str = "panel") -> dict:
    if host_procs():
        raise RuntimeError("이미 Host가 실행 중입니다. 먼저 정지하세요.")
    if protocol == "v2":
        return start_host_v2(broker, end_on_place, clear_hold, executor, report_placement)
    if not (S["control"] and S["binding"]):
        raise RuntimeError("v1에는 omx-host.json의 link_config(site·control·binding)가 필요합니다.")
    if report_placement and broker == "control":
        raise RuntimeError("관제 v1은 놓였음 조기 COMPLETE(shelf_placed)를 아직 받지 않고 HOLD합니다. "
                           "관제 PC 브로커에서는 이 옵션을 끄세요.")
    if report_placement and not end_on_place:
        raise RuntimeError("놓였음 즉시 COMPLETE는 넘겨받기(놓음 판정)와 함께 써야 합니다.")
    b = broker_for(broker)
    cmd = [UV, "run", "python", "scripts/cooperative_host.py"]
    if HOST_KNOWS_V2:
        cmd += ["--protocol", "v1"]  # the Host speaks v2 unless told otherwise (#135)
    cmd += ["--executor", "panel",
           "--host", b["host"], "--port", str(b["port"]),
           "--routes", str(S["routes"]), "--site", S["site"], "--control", S["control"],
           "--binding", S["binding"], "--client-id", S["client_id"], "--journal", str(S["journal"])]
    if not end_on_place:
        cmd.append("--no-end-on-place")
    # the Host turns both on by default (2026-11-22): say "off" explicitly
    cmd.append("--report-placement" if report_placement else "--no-report-placement")
    if clear_hold:
        cmd.append("--clear-hold")
    return launch(cmd, broker_env(b))


def start_host_v2(broker: str, end_on_place: bool, clear_hold: bool, executor: str,
                  report_placement: bool = False) -> dict:
    if not HOST_KNOWS_V2:
        raise RuntimeError(f"{S['repo']}의 Host가 v2(--protocol)를 모릅니다. Host 코드를 갱신하세요.")
    if V2 is None:
        raise RuntimeError(f"가상 관제 모듈을 불러오지 못했습니다: {V2_ERROR}")
    if executor not in ("panel", "mock"):
        raise RuntimeError("실행부는 panel 또는 mock입니다.")
    b = broker_for(broker)
    journal = S["journal_v2"] if executor == "panel" else S["journal_v2_mock"]
    cmd = [UV, "run", "python", "scripts/cooperative_host.py", "--protocol", "v2", "--executor", executor,
           "--host", b["host"], "--port", str(b["port"]), "--control", S["control_v2"],
           "--client-id", S["client_id"], "--journal", str(journal)]
    if executor == "panel":
        cmd += ["--routes", str(S["routes"])]
        if not end_on_place:
            cmd.append("--no-end-on-place")
        if report_placement and not end_on_place:
            raise RuntimeError("놓였음 즉시 COMPLETE는 넘겨받기(놓음 판정)와 함께 써야 합니다.")
        # the Host default changed (#156 review: off): always say which one
        cmd.append("--report-placement" if report_placement else "--no-report-placement")
    else:
        model, cal = versions_for("INBOUND", "2b")
        # simulated arms; the Pinky check stays on (this panel plays the Pinky). The docking sensor
        # check was dropped (2026-11-23); an older Host still needs it switched off for the mock.
        if "--require-docking" in (S["repo"] / "scripts/cooperative_host.py").read_text():
            cmd.append("--no-require-docking")
        cmd += ["--mock-versions", model, cal, "--mock-run-s", "6"]
    if clear_hold:
        cmd.append("--clear-hold")
    res = launch(cmd, broker_env(b))
    if res["ok"]:
        try:
            V2.connect(b["host"], b["port"], local=broker == "local", tls=b["tls"], ca_file=b["ca_file"],
                       username=b["username"], password=b["password"])
        except Exception as exc:  # noqa: BLE001
            res["message"] = f"Host는 떴지만 가상 관제가 브로커에 붙지 못했습니다: {exc}"
    return res


def launch(cmd: list[str], mqtt_env: dict) -> dict:
    env = {k: v for k, v in os.environ.items() if k != "VIRTUAL_ENV" and not k.startswith("ROPORT_MQTT_")}
    env.update(mqtt_env, PYTHONUNBUFFERED="1")
    log = S["run_dir"] / f"testpanel-host-{datetime.now():%Y%m%d-%H%M%S}.log"
    with log.open("a") as out:
        out.write(f"===== {datetime.now():%Y-%m-%d %H:%M:%S} 시험 패널 → Host 시작: {' '.join(cmd[2:])}\n")
        out.flush()
        subprocess.Popen(cmd, cwd=S["repo"], env=env, stdout=out, stderr=subprocess.STDOUT,
                         start_new_session=True)  # keeps running if this panel restarts
    STATE["host_log"] = log
    for i in range(40):  # wait for "subscribed" (or an early exit)
        time.sleep(0.25)
        text = log.read_text(errors="replace")
        if "subscribed" in text or "Traceback" in text or (i > 8 and not host_procs()):
            break
    return {"ok": bool(host_procs()), "log": tail(log, 12)}


def stop_host(force: bool = False) -> dict:
    procs = host_procs()
    if not procs:
        return {"ok": True, "message": "실행 중인 Host가 없습니다."}
    active = host_state().get("active")
    if active and not force:
        raise RuntimeError(f"Host가 작업 중입니다({active[:8]}). 끝난 뒤 정지하세요.")
    if V2 is not None:
        V2.close()  # also stops the simulated Pinky
    for p in procs:
        os.kill(p["pid"], signal.SIGINT)  # same as Ctrl+C: the Host closes its journal
    for _ in range(60):
        if not host_procs():
            return {"ok": True, "message": "Host를 정지했습니다."}
        time.sleep(0.25)
    raise RuntimeError("Host가 15초 안에 끝나지 않았습니다.")


# ---------------------------------------------------------------- mission (test control sender)


def _read_mission(proc: subprocess.Popen) -> None:
    m = STATE["mission"]
    for line in proc.stdout:
        with LOCK:
            m["lines"].append(f"[{now()}] {line.rstrip()}")
            m["lines"] = m["lines"][-300:]
    rc = proc.wait()
    with LOCK:
        m.update(running=False, rc=rc, proc=None)
        m["lines"].append(f"[{now()}] ── 송신기 종료 (코드 {rc})")


def send_mission(op: str, slot: str, package: str, timeout: float) -> dict:
    m = STATE["mission"]
    if m["running"]:
        raise RuntimeError("이전 명령이 아직 결과를 기다리는 중입니다.")
    procs = host_procs()
    if not procs:
        raise RuntimeError("Host가 실행 중이 아닙니다.")
    if op not in ("status", "inbound", "outbound"):
        raise RuntimeError("알 수 없는 명령입니다.")
    host = procs[0]
    if not host["local"]:
        b = broker_for("control")
    else:
        b = dict(LOCAL_BROKER)
    log = S["run_dir"] / f"testpanel-mission-{datetime.now():%Y%m%d-%H%M%S}-{op}.jsonl"
    cmd = [UV, "run", "python", "scripts/cooperative_control_sim.py", op,
           "--host", b["host"], "--port", str(b["port"]), "--site", S["site"], "--control", S["control"],
           "--binding", S["binding"], "--slot", slot, "--package", str(package),
           "--timeout", str(timeout), "--log", str(log)]
    env = {k: v for k, v in os.environ.items() if k != "VIRTUAL_ENV"}
    env["PYTHONUNBUFFERED"] = "1"
    proc = subprocess.Popen(cmd, cwd=S["repo"], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)
    label = {"status": "상태 조회", "inbound": "입고", "outbound": "하차"}[op]
    with LOCK:
        m.update(running=True, proc=proc, rc=None, op=op, log=str(log))
        m["lines"].append(f"[{now()}] ▶ {label} 보냄: 칸 {slot}" + ("" if op == "status" else f", 상품 {package}"))
    threading.Thread(target=_read_mission, args=(proc,), daemon=True).start()
    return {"ok": True}


def stop_waiting() -> dict:
    proc = STATE["mission"]["proc"]
    if proc is None:
        return {"ok": True}
    proc.send_signal(signal.SIGINT)
    return {"ok": True, "message": "결과 기다리기를 멈췄습니다. Host의 작업은 계속되고 결과는 Host가 보관합니다."}


# ---------------------------------------------------------------- state


def state() -> dict:
    try:
        st = panel("GET", "/api/status", timeout=3)
        r = st.get("last_result") or {}
        job = st.get("job") or {}
        link = ((st.get("gpu_link") or {}).get("infer") or {}).get("udp") or {}
        panel_view = {
            "ok": True, "version": st.get("version"), "armed": st.get("armed"),
            "armed_off_reason": st.get("armed_off_reason"),
            "job": {"running": job.get("running"), "kind": job.get("kind"),
                    "phase": (job.get("info") or {}).get("phase")},
            "ai_link": {"ok": link.get("ok"), "stage": link.get("stage"),
                        "age_s": round(time.time() - link["at"], 1) if link.get("at") else None},
            "last": {"source": r.get("source"), "stage": r.get("stage"), "final": r.get("final"),
                     "exit_ok": r.get("exit_ok"), "stopped_by": r.get("stopped_by"),
                     "end_reason": (r.get("last_episode") or {}).get("end_reason"),
                     "elapsed_s": (r.get("last_episode") or {}).get("elapsed_s"),
                     "align_ok": (r.get("align") or {}).get("ok"),
                     "handover": (r.get("handover") or {}).get("state")},
        }
    except RuntimeError as exc:
        panel_view = {"ok": False, "error": str(exc)}
    a = arms() if not (panel_view.get("job") or {}).get("running") else {"busy": True}
    with LOCK:
        m = STATE["mission"]
        mission = {"running": m["running"], "op": m["op"], "rc": m["rc"], "lines": m["lines"][-80:], "log": m["log"]}
    procs = host_procs()
    return {
        "time": now(),
        "settings": {"site": S["site"], "control": S["control"], "binding": S["binding"],
                     "control_v2": S["control_v2"], "host_knows_v2": HOST_KNOWS_V2, "v2_error": V2_ERROR,
                     "routes_file": str(S["routes"]), "journal": str(current_journal()),
                     "control_broker": S["control_broker"], "import_error": IMPORT_ERROR},
        "hosts": [{k: v for k, v in p.items() if k != "args"} for p in procs],
        "host_state": host_state(),
        "host_log": tail(STATE["host_log"]) if STATE["host_log"] else [],
        "panel": panel_view,
        "arms": a,
        "routes": routes(),
        "mission": mission,
        "v2": V2.view() if V2 is not None else None,
        "journal": journal_view() if (procs and procs[0]["protocol"] == "v2") else None,
        "real_shot": _real_shot(),
        "cams": {k: v for k, v in STATE["cams"].items() if k not in ("since", "done", "roles_at")}
                | {"roles": camera_roles()},
    }


def _short(x) -> str:
    return str(x)[:8] if x else "-"


def journal_view() -> dict:
    """Commands and their results straight from the Host's journal (read only; the Host replaces it
    atomically). Works on the control PC's broker too, where this panel may not read event topics."""
    path = current_journal()
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        return {"error": f"journal을 읽지 못했습니다: {exc}", "path": str(path)}
    if (data.get("identity") or {}).get("profile") != "roport-control-v2":
        return {"error": "v2 journal이 아닙니다.", "path": str(path)}
    rows = []
    for cid, rec in list((data.get("commands") or {}).items())[-12:]:
        c = rec.get("command") or {}
        p = c.get("payload") or {}
        kind = c.get("type", "")
        if kind == "shelf_transfer_command":
            what = f"{'입고' if p.get('operation') == 'store' else '하차'} {str(p.get('slot_id', '')).upper()} · 상자 {p.get('package_id')} · {p.get('pinky_id')}"
        elif kind == "robot_stop_command":
            what = f"정지 → {_short(p.get('target_command_id'))} ({p.get('reason')})"
        elif kind == "hold_release_command":
            what = f"HOLD 해제 → {_short(p.get('hold_command_id'))} ({p.get('operator')})"
        else:
            what = kind
        events = []
        for e in rec.get("events") or []:
            q = e.get("payload") or {}
            r = q.get("result") if isinstance(q.get("result"), dict) else None
            summary = None
            if r and "placement_confirmed" in r:
                summary = f"놓음 {r.get('placement_confirmed')} · 손 뗌 {r.get('release_confirmed')}"
            elif r:
                summary = json.dumps(r, ensure_ascii=False)[:120]
            events.append({"status": q.get("status"), "reason": q.get("reason"), "at": _local_hms(e.get("sent_at")),
                           "seq": q.get("sequence"), "result": summary})
        t = rec.get("timing") or {}
        base = t.get("received")
        segs = []
        if base is not None:
            names = [("sent", "관제 전송"), ("accepted", "접수"), ("start_sent", "시작 요청"), ("start_done", "제어판 시작 응답"),
                     ("step:연결 중", "연결·모델 준비"), ("step:안전 시작 중", "안전 시작(팔 움직임)"), ("step:실행 중", "모델 제어"),
                     ("placed", "놓였음 완료 보고"), ("step:정렬 중", "정렬"), ("ended", "종료")]
            for key, label in names:
                if key in t:
                    segs.append({"label": label, "at_s": round(t[key] - base, 2)})
            for key in t:
                if key.startswith("step:") and key not in dict(names):
                    segs.append({"label": key[5:], "at_s": round(t[key] - base, 2)})
            segs.sort(key=lambda x: x["at_s"])
        rows.append({"id": _short(cid), "type": kind, "what": what, "timing": segs, "sent": _local_hms(c.get("sent_at")),
                     "status": rec.get("status"), "detail": rec.get("detail"), "step": rec.get("last_step"),
                     "evidence": rec.get("evidence"), "align_error": rec.get("align_error"), "events": events})
    return {"path": str(path), "commands": rows[::-1], "active": _short(data.get("active")),
            "hold": _short(data.get("hold")), "outbox": len(data.get("outbox") or [])}


def _real_shot() -> dict:
    shots = sorted(RUNS.glob("*_front2.jpg"), key=lambda p: p.stat().st_mtime) if RUNS.exists() else []
    if not shots:
        return {"name": None, "taken": None}
    return {"name": shots[-1].name, "taken": datetime.fromtimestamp(shots[-1].stat().st_mtime).strftime("%m-%d %H:%M:%S")}


# ---------------------------------------------------------------- actions


def release_by_file(hold: str, checks: dict) -> dict:
    """The Host's local HOLD channel (same files the control panel's [HOLD 해제] writes)."""
    request_id = f"testpanel-{int(time.time() * 1000)}"
    request = {"request_id": request_id, "hold": hold, "checks": checks, "requested_at": time.time(), "by": "testpanel"}
    path = channel_file("hold-request")
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(request))
    tmp.replace(path)
    deadline = time.monotonic() + 13
    while time.monotonic() < deadline:
        try:
            result = json.loads(channel_file("hold-result").read_text()).get("results", {}).get(request_id)
        except (OSError, ValueError):
            result = None
        if result:
            STATE["arms"]["at"] = 0.0
            if result.get("cleared"):
                return {"ok": True, "message": "HOLD를 풀었습니다."}
            raise RuntimeError(f"Host가 해제를 거절했습니다: {result.get('reason')}")
        time.sleep(0.25)
    raise RuntimeError("Host가 13초 안에 답하지 않았습니다(요청 파일은 Host가 다음에 읽습니다).")


def need_checks(body: dict) -> dict:
    checks = body.get("checks") or {}
    if not all(checks.get(k) is True for k in HOLD_CHECKS):
        raise RuntimeError("확인 항목 3개(팔·상자·Pinky)를 모두 체크하세요.")
    return {k: True for k in HOLD_CHECKS}


def v2_running() -> bool:
    procs = host_procs()
    return bool(procs) and procs[0]["protocol"] == "v2" and V2 is not None


def act(path: str, body: dict) -> dict:
    if path == "/api/host/start":
        return start_host(body.get("broker", "local"), bool(body.get("end_on_place", True)), False,
                          bool(body.get("report_placement")), body.get("protocol", "v2"),
                          body.get("executor", "panel"))
    if path == "/api/host/stop":
        return stop_host(bool(body.get("force")))
    if path == "/api/armed":
        on = bool(body.get("on"))
        if on and not body.get("confirm"):
            raise RuntimeError("로봇 옆에 사람이 있는지 확인하고 체크하세요.")
        if on and not host_procs():
            # armed with no Host makes the panel start its own Host on the control PC broker
            raise RuntimeError("Host를 먼저 시작하세요(없으면 제어판이 관제 PC 브로커용 Host를 띄웁니다).")
        return panel("POST", "/api/armed", {"on": on, "confirm": bool(body.get("confirm"))})
    if path == "/api/home":
        if not body.get("confirm"):
            raise RuntimeError("로봇 옆에 사람이 있는지 확인하고 체크하세요. 팔이 움직입니다.")
        res = panel("POST", "/api/home/return", {}, timeout=10)
        STATE["arms"]["at"] = 0.0
        return res
    if path == "/api/stop":
        return panel("POST", "/api/stop", {})
    if path == "/api/hold/release":
        hold = host_state().get("hold")
        if not hold:
            raise RuntimeError("Host에 HOLD가 없습니다.")
        if not host_procs():
            raise RuntimeError("Host가 실행 중이어야 버튼 해제를 처리합니다. 강제 해제를 쓰거나 Host를 시작하세요.")
        return release_by_file(hold.get("command_id"), need_checks(body))
    if path == "/api/hold/force":
        checks = need_checks(body)  # noqa: F841 - the person's confirmation, as for the button
        hold = host_state().get("hold")
        if not hold:
            raise RuntimeError("Host에 HOLD가 없습니다.")
        a = arms(force=True)
        if not arms_home(a) and not body.get("allow_not_home"):
            raise RuntimeError("양팔이 홈·그리퍼 놓음 상태가 아닙니다. 먼저 [홈 위치로 이동]을 하세요. "
                               "(Host는 명령 접수 때 팔 위치를 보지 않습니다)")
        procs = host_procs()
        broker = "local" if not procs or procs[0]["local"] else "control"
        end_on_place = procs[0]["end_on_place"] if procs else bool(body.get("end_on_place", True))
        report = procs[0]["report_placement"] if procs else bool(body.get("report_placement"))
        protocol = procs[0]["protocol"] if procs else body.get("protocol", "v2")
        executor = procs[0]["executor"] if procs else body.get("executor", "panel")
        stop_host(force=True)
        res = start_host(broker, end_on_place, clear_hold=True, report_placement=report,
                         protocol=protocol, executor=executor)
        res["message"] = "Host를 --clear-hold로 다시 시작해 HOLD를 풀었습니다."
        return res
    if path == "/api/mission" and v2_running():
        op = body.get("op", "status")
        if op == "status":
            V2.status()
            return {"ok": True, "message": "상태 조회를 보냈습니다."}
        if op not in ("inbound", "outbound"):
            raise RuntimeError("알 수 없는 명령입니다.")
        slot, operation = str(body.get("slot", "2b")), ("store" if op == "inbound" else "retrieve")
        V2.transfer(operation, slot, str(body.get("package", "16")), versions_for(op.upper(), slot))
        return {"ok": True, "message": "v2 명령을 보냈습니다. 결과는 미션 이벤트에 나옵니다."}
    if path == "/api/v2/stop":
        if not v2_running():
            raise RuntimeError("v2 Host가 실행 중이 아닙니다.")
        target = V2.active or host_state().get("active")
        if not target:
            raise RuntimeError("정지할 작업이 없습니다.")
        V2.stop(target)
        return {"ok": True, "message": "정지 명령을 보냈습니다."}
    if path == "/api/v2/release":
        if not v2_running():
            raise RuntimeError("v2 Host가 실행 중이 아닙니다.")
        hold = host_state().get("hold")
        if not hold:
            raise RuntimeError("Host에 HOLD가 없습니다.")
        V2.release(hold["command_id"], need_checks(body))
        return {"ok": True, "message": "관제 HOLD 해제 명령을 보냈습니다. 결과는 미션 이벤트에 나옵니다."}
    if path == "/api/mission":
        return send_mission(body.get("op", "status"), str(body.get("slot", "2b")),
                            str(body.get("package", "16")), float(body.get("timeout", 900)))
    if path == "/api/cams/detect":
        return run_detect(bool(body.get("apply")), "수동") | {"ok": True, "message": "카메라를 다시 찾았습니다."}
    if path == "/api/cams/confirm":
        if not body.get("looked"):
            raise RuntimeError("모든 카메라 화면을 확인했다고 체크하세요.")
        return confirm_cameras()
    if path == "/api/mission/stop_wait":
        return stop_waiting()
    raise RuntimeError("알 수 없는 요청입니다.")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args) -> None:
        pass

    def _json(self, obj: dict, code: int = 200) -> None:
        data = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            data = (HERE / "index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif self.path == "/api/state":
            self._json(state())
        elif self.path.startswith("/img/cam?"):
            self._cam_image()
        elif self.path.startswith("/img/"):
            self._image(self.path.split("?")[0][5:])
        else:
            self._json({"error": "not found"}, 404)

    def _cam_image(self) -> None:
        """One picture of a saved camera role through the panel's probe (not while a run holds the cameras)."""
        q = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
        role = (q.get("role") or [""])[0]
        try:
            st = panel("GET", "/api/status", timeout=4)
            if (st.get("job") or {}).get("running") or st.get("preview"):
                return self._json({"error": "작업이나 카메라 보기 중에는 사진을 찍지 않습니다."}, 409)
            setup = panel("GET", "/api/robot_setup", timeout=5)
            path = next((c["path"] for c in setup.get("active_cameras") or [] if c["name"] == role), None)
            if not path:
                return self._json({"error": "그 역할의 카메라가 없습니다."}, 404)
            req = urllib.request.Request(PANEL + "/api/camera_probe?path=" + urllib.parse.quote(path))
            with urllib.request.urlopen(req, timeout=20) as r:
                data = r.read()
        except Exception as exc:  # noqa: BLE001
            return self._json({"error": str(exc)}, 502)
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _image(self, name: str) -> None:
        if name == "real":
            shots = sorted(RUNS.glob("*_front2.jpg"), key=lambda p: p.stat().st_mtime) if RUNS.exists() else []
            path = shots[-1] if shots else HERE / "static/shelf_real_ref.jpg"
        elif name == "sim":
            path = HERE / "static/shelf_sim.jpg"
        else:
            return self._json({"error": "not found"}, 404)
        if not path.exists():  # no aligner photo yet and no local reference picture
            return self._json({"error": "no picture"}, 404)
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Taken", datetime.fromtimestamp(path.stat().st_mtime).strftime("%m-%d %H:%M"))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self) -> None:
        raw = self.rfile.read(int(self.headers.get("Content-Length", 0)) or 0)
        try:
            body = json.loads(raw or b"{}")
            self._json(act(self.path, body) or {"ok": True})
        except Exception as exc:  # noqa: BLE001 - every refusal goes back to the screen
            self._json({"error": str(exc)}, 400)


def main() -> None:
    # started with nohup ... & (open.sh, desktop icon) bash leaves SIGINT ignored, and the Host we
    # launch would inherit that: [Host 정지] sends SIGINT, so put Ctrl+C back for us and our children
    signal.signal(signal.SIGINT, signal.default_int_handler)
    threading.Thread(target=camera_watch_loop, name="camera-watch", daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)  # this PC only
    print(f"OMX Host 시험 패널: http://127.0.0.1:{PORT}  (site {S['site']}, 경로표 {S['routes']})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
