"""Fake panel HTTP server and fixtures for the shelf v2 panel executor (shelf_panel.py).

Copied from 5dd0f9e's ci/test_panel_executor.py (task/omx-cooperative), helpers only, so the
#169/#171 tests run against the shelf_panel copy; consolidate when task/omx-cooperative is integrated.
"""

import json
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx import shelf_panel as panel_executor  # noqa: F401
from roport_omx.shelf_panel import PanelExecutor, Route, load_routes  # noqa: F401

MODEL = "/models/real_act_2b/pretrained_model"
ROUTES = {
    ("INBOUND", "2b", "COOPERATIVE"): Route(
        MODEL, "INBOUND tray->shelf 2B | marker {box}", 40, 15
    )
}
OUT_MODEL = "/models/outbound_act_re1112x37_60k_1113/pretrained_model"
OUT_ROUTE = {
    ("OUTBOUND", "2b", "COOPERATIVE"): Route(
        OUT_MODEL, "OUTBOUND shelf 2B->pinky | marker {box}", 40, 15
    )
}


def pinky(on_pinky=True, on_shelf=False, **change):
    """last_result.pinky of the panel's camera-only check after an unloading evaluation.

    Shaped like panel 3.20 in the field (11/21 logs/pinky_result.json): the commanded slot's own
    slot_empty, and the tray compared with the photo taken right before this mission.
    """
    return {
        "ran": True,
        "on_pinky": on_pinky,
        "on_shelf": on_shelf,
        "slot": "2B",
        "slot_empty": None if not isinstance(on_shelf, bool) else not on_shelf,
        "pinky": "ok" if on_pinky else "tray_empty",
        "shelf": "ok" if on_shelf else "marker_not_found",
        "box": 16,
        "tray_reference": "pre_mission",
        "tray_reference_state": "ok",
        "pre_mission_match": True,
        "returncode": 0,
        "skipped": None,
    } | change


def arm(side, *, home=True, gripper_open=True):
    """last_result arm; gripper_open=True means released (closed on nothing, 50.0), False = holding (59.6)."""
    grip = 50.0 if gripper_open else 59.6
    return {
        "side": side,
        "joints": {"shoulder_pan": 0.0, "gripper": grip},
        "gripper": grip,
        "gripper_open": False,
        "at_home": home,
    }


def live(side, *, home=True, grip=49.0):
    """Panel 3.11 /api/arms entry for one arm, read from the bus while idle."""
    return {
        "side": side,
        "joints": {"shoulder_pan": 0.0, "gripper": grip},
        "gripper": grip,
        "at_home": home,
    }


def align(*, far=(), check="ok", **change):
    """last_result.align of panel 3.9 after a successful shelf alignment."""
    rest_arm = {"joints": {"shoulder_pan": 1.0}, "gripper_state": "closed"}
    return {
        "ran": True,
        "ok": True,
        "box": 16,
        "slot": "2B",
        "returncode": 0,
        "stopped_by": None,
        "shelf": "ok",
        "on_shelf": True,
        "align": "aligned",
        "released": True,
        "skipped": None,
        "rest": {
            "check": check,
            "tolerance": 5.0,
            "arms": {
                "OMX 1": rest_arm | {"side": "left", "far_joints": list(far)},
                "OMX 2": rest_arm | {"side": "right", "far_joints": []},
            },
        },
    } | change


class FakePanel:
    def __init__(self):
        self.status = {
            "version": "3.20.0",
            "armed": True,
            "job": {"running": False},
            "last_result": None,
            "gpu_link": {
                "ok": False,  # SSH tunnel; not needed for a mission since 3.13
                "infer": {
                    "transport": "udp",
                    "udp": {"ok": True, "stage": "ok", "error": "", "at": None},
                },
            },
        }
        # seconds since the panel's last UDP check; None keeps "at" as set
        self.udp_age = 1.0
        self.home_returns = 0  # POST /api/home/return calls
        self.home_return_bodies = []
        self.home_return_error = None
        self.models = [MODEL]
        self.starts = []
        self.start_error = None
        self.start_refusal = None  # JSON body of a 400 refusal (panel 3.20 tray check)
        self.arms = {
            "OMX 1": live("left"),
            "OMX 2": live("right"),
        }  # None = panel before 3.11
        self.arms_at = "2026-11-22T21:30:00"  # panel /api/arms "at" (bus read time)
        self.arms_models = []
        server = ThreadingHTTPServer(("127.0.0.1", 0), self._handler())
        self.server = server
        self.url = f"http://127.0.0.1:{server.server_address[1]}"
        threading.Thread(target=server.serve_forever, daemon=True).start()

    def _handler(self):
        panel = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _json(self, body, code=200):
                raw = json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_GET(self):
                if self.path == "/api/status":
                    body = json.loads(json.dumps(panel.status))
                    infer = (body.get("gpu_link") or {}).get("infer") or {}
                    if panel.udp_age is not None and isinstance(infer.get("udp"), dict):
                        infer["udp"]["at"] = time.time() - panel.udp_age
                    self._json(body)
                elif self.path.startswith("/api/arms?"):
                    query = urllib.parse.urlsplit(self.path).query
                    panel.arms_models.append(urllib.parse.parse_qs(query)["model"][0])
                    if panel.arms is None:
                        self._json({"error": "not found"}, 404)
                    else:
                        self._json(
                            {
                                "source": "bus",
                                "at": panel.arms_at,
                                "arms": panel.arms,
                                "error": None,
                            }
                        )
                elif self.path == "/api/train/runs":
                    self._json({"runs": [{"model_path": m} for m in panel.models]})
                else:
                    self._json({"error": "unknown"}, 404)

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                if self.path == "/api/home/return":
                    if panel.home_return_error:
                        self._json({"error": panel.home_return_error}, 400)
                        return
                    panel.home_returns += 1
                    panel.home_return_bodies.append(body)
                    panel.status["job"] = {
                        "running": True,
                        "kind": "home",
                        "info": {"phase": "홈 위치로 이동 중"},
                    }
                    self._json({"ok": True, "mode": "return_home"})
                elif self.path != "/api/eval/start":
                    self._json({"error": "unknown"}, 404)
                elif panel.start_refusal:
                    self._json(panel.start_refusal, 400)
                elif panel.start_error:
                    self._json({"error": panel.start_error}, 400)
                else:
                    panel.starts.append(body)
                    repo = f"user/eval_test_{len(panel.starts)}"
                    panel.status["job"] = {
                        "running": True,
                        "kind": "eval",
                        "info": {"repo_id": repo, "phase": "모델 불러오는 중"},
                    }
                    self._json({"ok": True, "repo_id": repo})

        return Handler

    def place(self, *, on_shelf=True, slot="2B", **evidence):
        """Panel 3.17: the evaluation ended, the aligner read the shelf and now aligns."""
        info = self.status["job"]["info"]
        repo = info.get("repo_id") or info.get("eval_repo_id")
        self.status["job"] = {
            "running": True,
            "kind": "align",
            "info": {"phase": "정렬 중", "eval_repo_id": repo},
        }
        self.status["last_result"] = {
            "kind": "eval",
            "source": "mqtt",
            "repo_id": repo,
            "exit_ok": True,
            "stopped_by": None,
            "err": None,
            "arms": {"OMX 1": arm("left"), "OMX 2": arm("right")},
            "stage": "align",
            "final": False,
            "placement": {
                "on_shelf": on_shelf,
                "slot": slot,
                "box": 16,
                "source": "align",
                "observed_at": "2026-11-18T01:00:00.000+09:00",
                "observed_ts": time.time() - 0.5,
                "observed_clock": "capture",
                "reported_at": "2026-11-18T01:00:00.200+09:00",
                # panel 3.20: the arms measured when the evaluation ended on its return
                "eval_end": {
                    "end_reason": "returned",
                    "all_at_home": True,
                    "arms": {
                        "left": {"at_home": True, "gripper": 50.0},
                        "right": {"at_home": True, "gripper": 49.0},
                    },
                },
            }
            | evidence,
        }

    def finish(self, **result):
        info = self.status["job"]["info"]
        repo = info.get("repo_id") or info.get("eval_repo_id")
        self.status["job"] = {
            "running": False,
            "kind": "eval",
            "info": {"repo_id": repo},
        }
        self.status["last_result"] = {
            "kind": "eval",
            "source": "mqtt",
            "repo_id": repo,
            "exit_ok": True,
            "stopped_by": None,
            "err": None,
            "arms": {"OMX 1": arm("left"), "OMX 2": arm("right")},
            "home_check": "ok",
        } | result
