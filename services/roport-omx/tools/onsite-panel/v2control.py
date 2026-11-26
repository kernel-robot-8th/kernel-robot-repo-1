"""Mini Fleet for the test panel: speaks roport-control-v2 to the shelf-01 Host like control does.

- every second: robot_state_request to shelf-01/omx1/omx2 (keep_running for the active command, as
  Fleet does), so the Host's watchdog keeps a running task alive only while this panel is up;
- commands: shelf_transfer_command (store/retrieve), robot_stop_command, hold_release_command;
- every command_status is written to a JSONL log, then acknowledged with event_stored;
- on the control PC's broker it only WATCHES (no request, no ack, no command): the real Fleet
  owns that link, and an ack from here would clear Host results Fleet never stored;
- on the LOCAL broker only, a simulated pinky1 robot_state (stopped at N11 with a new arrival per
  transfer). Never on the control PC's broker: a fake Pinky there would mislead the real Fleet.

Uses the Host repo's own wire module (field_wire) so every message is checked by the contract.
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from datetime import UTC, datetime, timedelta
from pathlib import Path
from queue import Empty

from roport_omx.mqtt_link import field_wire as wire
from roport_omx.mqtt_link.mqtt import MqttConfig, MqttTransport

ENDPOINTS = ("shelf-01", "omx1", "omx2")
PINKY = "pinky1"
TERMINAL = {"completed", "rejected", "failed", "interrupted"}


def _iso(delta_s: float = 0.0) -> str:
    return (datetime.now(UTC) + timedelta(seconds=delta_s)).isoformat()


class V2Control:
    def __init__(self, control_id: str, log_dir: Path, say):
        self.control_id, self.log_dir, self.say = control_id, log_dir, say
        self.lock = threading.RLock()
        self.transport: MqttTransport | None = None
        self.broker: dict | None = None
        self.simulate_pinky = False
        self.error: str | None = None
        self.states: dict[str, dict] = {}  # endpoint -> {"payload", "received"}
        self.commands: dict[str, dict] = {}  # command_id -> {"type", "label", "status", ...}
        self.active: str | None = None  # the transfer this panel sent and keeps running
        self.log: Path | None = None
        self.pinky = {"boot": f"boot-testpanel-{PINKY}-{uuid.uuid4().hex[:6]}", "seq": 0, "arrival": None}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------ link
    def connect(self, host: str, port: int, *, local: bool, tls: bool = False, ca_file: str | None = None,
                username: str | None = None, password: str | None = None) -> None:
        self.close()
        transport = MqttTransport(
            MqttConfig(host=host, client_id=f"omx-testpanel-{self.control_id}", port=port, tls=tls,
                       ca_file=ca_file, username=username, password=password),
            [wire.topic(e, "event") for e in ENDPOINTS],
        )
        transport.start()
        with self.lock:
            self.transport, self.broker = transport, {"host": host, "port": port, "local": local}
            self.simulate_pinky, self.error = local, None
            self.states.clear()
            self.log = self.log_dir / f"testpanel-v2-{datetime.now():%Y%m%d-%H%M%S}.jsonl"
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="v2-control", daemon=True)
        self._thread.start()
        where = "로컬 브로커 · Pinky 상태 흉내 켬" if local else "관제 PC 브로커 · 보기 전용(송신·ACK 안 함)"
        self.say(f"가상 관제 연결: {host}:{port} ({where}), 관제 ID {self.control_id}")

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)
        with self.lock:
            if self.transport:
                try:
                    self.transport.stop()
                except Exception:  # noqa: BLE001
                    pass
            self.transport = None
            self.active = None

    @property
    def connected(self) -> bool:
        with self.lock:
            return bool(self.transport and self.transport.ready)

    def _publish(self, topic: str, message: dict) -> None:
        with self.lock:
            transport = self.transport
        if transport is None:
            raise RuntimeError("가상 관제가 브로커에 연결되어 있지 않습니다.")
        transport.publish(topic, message, timeout=3)

    def _send(self, kind: str, target: str, payload: dict, *, command: bool = False, expires_s: float | None = None) -> dict:
        extra = {}
        if command:
            extra["command_id"] = str(uuid.uuid4())
        if expires_s is not None:
            extra["expires_at"] = _iso(expires_s)
        message = wire.build_message(kind, self.control_id, target, payload, **extra)
        self._publish(wire.topic(target, "cmd"), message)
        return message

    # ------------------------------------------------------------------ loop
    def _run(self) -> None:
        next_tick = 0.0
        while not self._stop.is_set():
            transport = self.transport
            if transport is None:
                return
            try:
                delivery = transport.receive(timeout=0.1)
            except Empty:
                delivery = None
            except ConnectionError as exc:
                self.error = f"브로커 연결 끊김: {exc}"
                self.say(self.error)
                return
            if delivery is not None:
                try:
                    self._handle(wire.decode(delivery.payload))
                except ValueError as exc:
                    self.say(f"받은 메시지 무시: {exc}")
                try:
                    transport.ack(delivery)
                except ConnectionError:
                    pass
            now = time.monotonic()
            if now >= next_tick:
                next_tick = now + 1.0
                try:
                    self._tick()
                except Exception as exc:  # noqa: BLE001 - shown on screen, the loop goes on
                    self.error = f"주기 송신 실패: {exc}"

    def _handle(self, message: dict) -> None:
        source, p = message["source"], message["payload"]
        if message["target"] != self.control_id:
            return
        if message["type"] == "robot_state":
            with self.lock:
                self.states[source] = {"payload": p, "received": time.time()}
            return
        if message["type"] != "command_status":
            return
        cid = message["command_id"]
        with self.lock:
            if self.log:
                with self.log.open("a") as f:
                    f.write(json.dumps(message, ensure_ascii=False) + "\n")
            entry = self.commands.get(cid)
            seen = entry is not None and p["sequence"] <= entry.get("sequence", 0)
            if entry is not None and not seen:
                entry.update(status=p["status"], sequence=p["sequence"], reason=p.get("reason"), result=p.get("result"))
            if cid == self.active and p["status"] in TERMINAL:
                self.active = None
        # stored (JSONL) → acknowledge, like control's journal-then-event_stored (local broker only)
        if not self.broker["local"]:
            ack = None
        else:
            ack = wire.build_message("event_stored", self.control_id, source, {"acked_message_id": message["message_id"]})
        if ack is not None:
            self._publish(wire.topic(source, "ack"), ack)
        if seen:
            return
        label = entry["label"] if entry else "(이 패널이 보내지 않은 명령)"
        text = f"{source} · {label} · {cid[:8]} → {p['status']}"
        if p.get("reason"):
            text += f" · reason {p['reason']}"
        result = p.get("result")
        if isinstance(result, dict):
            if "placement_confirmed" in result:
                text += (f" · 놓음 {result.get('placement_confirmed')} · 손 뗌 {result.get('release_confirmed')}"
                         f" · 상자 {result.get('package_id')} · 칸 {result.get('slot_id')}")
            elif result:
                text += " · " + json.dumps(result, ensure_ascii=False)[:160]
        self.say(text)

    def _tick(self) -> None:
        if not self.broker["local"]:
            return  # watch only: the real Fleet sends requests on the control PC's broker
        if self.simulate_pinky:
            self._pinky_state()
        boot = self.boot("shelf-01")
        with self.lock:
            active = self.active
        for endpoint in ENDPOINTS:
            if endpoint == "shelf-01" and active and boot:
                # Fleet's renewal for the active task: a new message every second, valid 3 s
                self._send(
                    "robot_state_request",
                    endpoint,
                    {"expected_boot_version": boot, "keep_running_command_id": active},
                    expires_s=3,
                )
            else:
                self._send("robot_state_request", endpoint, {})

    def _pinky_state(self) -> None:
        pk = self.pinky
        pk["seq"] += 1
        state = {
            "node": "N11",
            "arrival_command_id": pk["arrival"],
            "mode": "line",
            "stopped": True,
            "line_enabled": False,
            "nav2_ready": False,
            "localization_ready": False,
            "drive_owner": "none",
            "graph_version": "line-testpanel",
            "map_version": "map-testpanel",
            "heading_rad": 0.0,
            "fault": None,
        }
        message = wire.build_message(
            "robot_state",
            PINKY,
            self.control_id,
            {"boot_version": pk["boot"], "observed_at": _iso(), "sequence": pk["seq"], "active_command_id": None, "state": state},
        )
        self._publish(wire.topic(PINKY, "event"), message)

    # ------------------------------------------------------------------ views
    def boot(self, endpoint: str) -> str | None:
        with self.lock:
            seen = self.states.get(endpoint)
        if not seen or time.time() - seen["received"] > 3:
            return None
        return seen["payload"]["boot_version"]

    def view(self) -> dict:
        with self.lock:
            states = {}
            for e in ENDPOINTS:
                seen = self.states.get(e)
                if not seen:
                    continue
                p, s = seen["payload"], seen["payload"]["state"]
                states[e] = {
                    "age_s": round(time.time() - seen["received"], 1),
                    "boot": p["boot_version"],
                    "active": p["active_command_id"],
                    "ready": s.get("ready"),
                    "stopped": s.get("stopped"),
                    "fault": s.get("fault"),
                    "home": s.get("home_reached"),
                    "holding": s.get("holding_state"),
                    "docking": s.get("docking"),
                }
            commands = [
                {"id": cid, **{k: v for k, v in c.items() if k != "message"}}
                for cid, c in list(self.commands.items())[-8:]
            ]
            return {
                "connected": self.connected,
                "broker": self.broker,
                "control_id": self.control_id,
                "error": self.error,
                "simulate_pinky": self.simulate_pinky,
                "pinky": {"boot": self.pinky["boot"], "arrival": self.pinky["arrival"]} if self.simulate_pinky else None,
                "states": states,
                "active": self.active,
                "commands": commands,
                "log": str(self.log) if self.log else None,
            }

    # ------------------------------------------------------------------ commands
    def _remember(self, message: dict, label: str) -> None:
        with self.lock:
            self.commands[message["command_id"]] = {"type": message["type"], "label": label, "status": "sent", "sequence": 0}
            while len(self.commands) > 40:
                self.commands.pop(next(iter(self.commands)))

    def _local_only(self) -> None:
        if not self.connected:
            raise RuntimeError("가상 관제가 브로커에 연결되어 있지 않습니다.")
        if not self.broker["local"]:
            raise RuntimeError("관제 PC 브로커에서는 실제 관제(Fleet)가 명령합니다. 이 패널은 보기만 합니다.")

    def status(self) -> None:
        self._local_only()
        for endpoint in ENDPOINTS:
            self._send("robot_state_request", endpoint, {})
        self.say("▶ 상태 조회 보냄 (shelf-01, omx1, omx2)")

    def transfer(self, operation: str, slot: str, package: str, versions: tuple[str, str]) -> str:
        self._local_only()
        with self.lock:
            if self.active:
                raise RuntimeError(f"이전 명령({self.active[:8]})이 아직 끝나지 않았습니다.")
        boots = {e: self.boot(e) for e in ENDPOINTS}
        missing = [e for e, b in boots.items() if not b]
        if missing:
            raise RuntimeError(f"Host 상태(robot_state)를 3초 안에 받지 못했습니다: {', '.join(missing)}. Host가 v2로 떠 있나요?")
        # a new Pinky visit for each transfer (the Host takes one transfer per arrival)
        self.pinky["arrival"] = f"cmd-arrival-{uuid.uuid4().hex[:12]}"
        self._pinky_state()
        time.sleep(0.3)
        payload = {
            "expected_boot_version": boots["shelf-01"],
            "operation": operation,
            "package_id": str(package),
            "slot_id": slot,
            "pinky_id": PINKY,
            "arrival_command_id": self.pinky["arrival"],
            "arms": [
                {"robot_id": "omx1", "expected_boot_version": boots["omx1"]},
                {"robot_id": "omx2", "expected_boot_version": boots["omx2"]},
            ],
            "model_version": versions[0],
            "calibration_version": versions[1],
            "expected_pinky_boot_version": self.pinky["boot"],
        }
        label = f"{'입고' if operation == 'store' else '하차'} {slot.upper()} 상자 {package}"
        message = wire.build_message(
            "shelf_transfer_command", self.control_id, "shelf-01", payload,
            command_id=str(uuid.uuid4()), expires_at=_iso(5),
        )
        self._remember(message, label)
        with self.lock:
            self.active = message["command_id"]
        try:
            self._publish(wire.topic("shelf-01", "cmd"), message)
        except Exception:
            with self.lock:
                self.active = None
            raise
        self.say(f"▶ {label} 보냄 · {message['command_id'][:8]} · 모델 {versions[0]} · 보정 {versions[1]}")
        return message["command_id"]

    def stop(self, target: str) -> str:
        self._local_only()
        boot = self.boot("shelf-01")
        if not boot:
            raise RuntimeError("shelf-01 상태를 받지 못해 boot를 모릅니다.")
        message = self._send(
            "robot_stop_command", "shelf-01",
            {"expected_boot_version": boot, "target_command_id": target, "reason": "operator_request"},
            command=True, expires_s=3,
        )
        self._remember(message, f"정지 → {target[:8]}")
        self.say(f"▶ 정지 명령 보냄 · 대상 {target[:8]}")
        return message["command_id"]

    def release(self, hold: str, checks: dict) -> str:
        self._local_only()
        boot = self.boot("shelf-01")
        if not boot:
            raise RuntimeError("shelf-01 상태를 받지 못해 boot를 모릅니다.")
        message = self._send(
            "hold_release_command", "shelf-01",
            {
                "expected_boot_version": boot,
                "hold_command_id": hold,
                "operator": "testpanel",
                "confirmed": {"arms": bool(checks.get("arms_safe")), "box": bool(checks.get("package_checked")),
                              "pinky": bool(checks.get("pinky_checked"))},
            },
            command=True, expires_s=5,
        )
        self._remember(message, f"HOLD 해제 → {hold[:8]}")
        self.say(f"▶ 관제 HOLD 해제 명령 보냄 · HOLD {hold[:8]}")
        return message["command_id"]
