"""PR45 COOPERATIVE bridge, called only by the Pinky trial's single owner."""

import re
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from time import monotonic
from uuid import uuid4

from roport_control_server.adapters.mqtt.messages import timestamp, topic

PROFILE = "roport-omx-task-v1"
WORKCELL = "shelf-01"
IDS = ("task_id", "step_id", "command_id", "execution_id")
ECHO = (
    "operation",
    "manipulation_mode",
    "assigned_arm_id",
    "robot_ids",
    "package_id",
    "slot_id",
    "pinky_id",
    "dock_session_id",
    "model_version",
    "calibration_version",
)


def validate_settings(settings):
    if not isinstance(settings, dict) or set(settings) != {
        "binding_id",
        "slot_id",
        "model_version",
        "calibration_version",
        "execution_mode",
    }:
        raise ValueError("invalid OMX trial settings")
    if any(
        not isinstance(v, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", v)
        for v in settings.values()
    ):
        raise ValueError("invalid OMX setting identifier")
    if settings["slot_id"] not in {"1a", "1b", "1c", "2a", "2b", "2c"} or settings[
        "execution_mode"
    ] not in {"mock", "hardware"}:
        raise ValueError("unsupported OMX slot/execution mode")


def returned(payload):
    arms = payload.get("arms")
    return (
        isinstance(arms, dict)
        and set(arms) == {"omx1", "omx2"}
        and all(
            isinstance(a, dict)
            and all(
                a.get(k) is v
                for k, v in {
                    "connected": True,
                    "returned": True,
                    "safe": True,
                    "grasping": False,
                }.items()
            )
            for a in arms.values()
        )
    )


def available(payload):
    return (
        payload.get("ready") is True
        and payload.get("armed") is True
        and all(
            k in payload and payload[k] is None
            for k in ("active", "hold", "executor_error")
        )
        and returned(payload)
    )


class OmxTrial:
    def __init__(self, control):
        self.control = control
        self.settings = control.config["omx"]
        validate_settings(self.settings)
        self.status = None
        self.heard = -float("inf")
        self.pending = {}
        self.next_poll = 0
        self.deadline = None
        if "omx" not in control.state:
            state = deepcopy(control.state)
            state["omx"] = {"commands": {}, "events": {}}
            self.save(state)

    def save(self, state):
        try:
            self.control.journal.save(state)
        except ValueError as exc:
            # A full journal is a storage failure, never a malformed MQTT delivery.
            self.control.journal.failed = True
            raise OSError("OMX journal unavailable; reconcile before restart") from exc

    def envelope(self, kind, payload, ids=None):
        c = self.control
        now = datetime.now(UTC)
        return {
            "schema_version": "1.0",
            "profile": PROFILE,
            "message_id": str(uuid4()),
            "type": kind,
            "site_id": c.config["site_id"],
            "source": c.config["control_id"],
            "target": WORKCELL,
            "sent_at": now.isoformat(),
            **(
                {"expires_at": (now + timedelta(seconds=5)).isoformat()}
                if kind.endswith(".requested")
                else {}
            ),
            **(
                ids
                or {
                    "task_id": c.state["task_id"],
                    "step_id": "omx-status",
                    "command_id": str(uuid4()),
                    "execution_id": str(uuid4()),
                }
            ),
            "payload": payload,
        }

    def publish(self, message, channel="cmd"):
        self.control.transport.publish(
            topic(self.control.config["site_id"], "workcell", WORKCELL, channel),
            message,
        )

    def ack(self, event):
        self.publish(
            self.envelope(
                "event.ack",
                {"acked_message_id": event["message_id"], "status": "stored"},
                {k: event[k] for k in IDS},
            ),
            "ack",
        )

    def command_for(self, gate):
        return next(
            (
                cmd
                for cmd in self.control.state["omx"]["commands"].values()
                if cmd["payload"]["dock_session_id"] == gate["stop_session_id"]
            ),
            None,
        )

    def manual_error(self):
        gate = self.control.state["gate"]
        if not gate or not gate.get("omx_operation"):
            return None
        cmd = self.command_for(gate)
        if cmd:
            accepted = next(
                (
                    e
                    for e in self.control.state["omx"]["events"].values()
                    if e["command_id"] == cmd["command_id"]
                    and e["type"].endswith(".accepted")
                ),
                None,
            )
            if (
                accepted
                and self.status
                and monotonic() - self.heard <= 3
                and self.status["boot_id"] == cmd["payload"]["expected_boot_id"]
                and available(self.status)
                and timestamp(self.status["observed_at"])
                > timestamp(accepted["payload"]["observed_at"])
            ):
                return None
            return "OMX 작업 전송됨 · 작업 접수 후 최신 대기·팔 복귀 확인 또는 완료 MQTT를 기다리세요."
        if self.status and (monotonic() - self.heard > 3 or not available(self.status)):
            return "OMX 상태를 확인하세요. 실행 중·오류·상태 지연에서는 출발할 수 없습니다."
        return None

    def snapshot(self):
        gate = self.control.state["gate"]
        command = self.command_for(gate) if gate else None
        return {
            "enabled": True,
            "execution_mode": self.settings["execution_mode"],
            "phase": "작업 완료 대기"
            if command
            else "상태 확인 대기"
            if gate and gate.get("omx_operation")
            else "대기",
            "manual_error": self.manual_error(),
        }

    def poll(self):
        c = self.control
        gate = c.state["gate"]
        if (
            c.state["phase"] in {"READY", "HOLD", "COMPLETE"}
            or not gate
            or not gate.get("omx_operation")
        ):
            return
        cmd = self.command_for(gate)
        now = monotonic()
        if cmd:
            if self.deadline is not None and now > self.deadline:
                c.hold("omx_result_unknown")
                self.deadline = None
            if c.state["phase"] == "HOLD":
                return
        if not cmd and (
            self.status and now - self.heard <= 3 and available(self.status)
        ):
            c._gate_error("gate")
            payload = {
                k: self.settings[k]
                for k in (
                    "binding_id",
                    "slot_id",
                    "model_version",
                    "calibration_version",
                )
            }
            payload.update(
                expected_boot_id=self.status["boot_id"],
                operation=gate["omx_operation"],
                workcell_id=WORKCELL,
                manipulation_mode="COOPERATIVE",
                assigned_arm_id=None,
                robot_ids=["omx1", "omx2"],
                package_id=gate["package_id"],
                quantity=1,
                pinky_id=gate["robot_id"],
                dock_session_id=gate["stop_session_id"],
                resource_token=str(uuid4()),
            )
            ids = {
                "task_id": c.state["task_id"],
                "step_id": gate["step_id"],
                "command_id": str(uuid4()),
                "execution_id": str(uuid4()),
            }
            cmd = self.envelope(
                "shelf_" + gate["omx_operation"].lower() + ".requested", payload, ids
            )
            state = deepcopy(c.state)
            state["omx"]["commands"][cmd["command_id"]] = cmd
            self.save(state)
            self.deadline = now + c.config["command_timeout_s"]
            self.status = None
            self.publish(cmd)
        elif now >= self.next_poll:
            self.pending = {
                k: v
                for k, v in self.pending.items()
                if timestamp(v["expires_at"]) > datetime.now(UTC)
            }
            cmd = self.envelope(
                "status.requested", {"binding_id": self.settings["binding_id"]}
            )
            self.pending[cmd["command_id"]] = cmd
            self.publish(cmd)
            self.next_poll = now + 2

    def handle(self, delivery, event):
        c = self.control
        p = event["payload"]
        if (
            delivery.retained
            or delivery.qos != 1
            or delivery.topic
            != topic(c.config["site_id"], "workcell", WORKCELL, "event")
            or (
                event.get("profile"),
                event["site_id"],
                event["source"],
                event["target"],
                p.get("binding_id"),
            )
            != (
                PROFILE,
                c.config["site_id"],
                WORKCELL,
                c.config["control_id"],
                self.settings["binding_id"],
            )
        ):
            raise ValueError("untrusted OMX event")
        stored = c.state["omx"]["events"].get(event["message_id"])
        if stored:
            if stored != event:
                raise ValueError("conflicting OMX event ID")
            self.ack(event)
            return
        status = event["command_id"] in self.pending
        cmd = (
            self.pending.get(event["command_id"])
            if status
            else c.state["omx"]["commands"].get(event["command_id"])
        )
        if not cmd:
            return
        gate = c.state["gate"]
        current = bool(
            not status
            and gate
            and cmd["task_id"] == c.state["task_id"]
            and cmd["payload"]["dock_session_id"] == gate["stop_session_id"]
        )
        if any(event[k] != cmd[k] for k in IDS):
            raise ValueError("OMX execution mismatch")
        kind, _, phase = event["type"].rpartition(".")
        if kind != cmd["type"].removesuffix(".requested") or phase not in {
            "accepted",
            "progress",
            "completed",
            "failed",
            "rejected",
        }:
            raise ValueError("invalid OMX result type")
        if (
            type(p.get("phase_revision")) is not int
            or p["phase_revision"] < 1
            or not isinstance(p.get("boot_id"), str)
        ):
            raise ValueError("invalid OMX result identity")
        if not status and p["boot_id"] != cmd["payload"]["expected_boot_id"]:
            if current:
                c.hold("omx_restarted")
            return
        if (
            not status
            and phase != "rejected"
            and any(p.get(k) != cmd["payload"][k] for k in ECHO)
        ):
            raise ValueError("OMX transfer mismatch")
        age = (datetime.now(UTC) - timestamp(p.get("observed_at"))).total_seconds()
        if status and (
            timestamp(cmd["expires_at"]) <= datetime.now(UTC) or not -2 <= age <= 3
        ):
            return
        previous = [
            e
            for e in c.state["omx"]["events"].values()
            if e["command_id"] == cmd["command_id"]
        ]
        if any(e["payload"]["phase_revision"] == p["phase_revision"] for e in previous):
            raise ValueError("conflicting OMX phase revision")
        state = deepcopy(c.state)
        state["omx"]["events"][event["message_id"]] = event
        self.save(state)
        # A closed stop session cannot change a later task, even on a late failure.
        if not status and not current:
            self.ack(event)
            return
        if previous and p["phase_revision"] < max(
            e["payload"]["phase_revision"] for e in previous
        ):
            self.ack(event)
            return
        if status:
            self.pending.pop(cmd["command_id"], None)
            if phase == "completed":
                self.status, self.heard = p, monotonic()
                gate = c.state["gate"]
                active = self.command_for(gate) if gate else None
                if active and p["boot_id"] != active["payload"]["expected_boot_id"]:
                    c.hold("omx_restarted")
                elif active and (
                    p.get("hold") is not None or p.get("executor_error") is not None
                ):
                    c.hold("omx_host_hold")
            else:
                c.hold("omx_status_rejected")
        elif not any(
            e["type"].endswith((".completed", ".failed", ".rejected")) for e in previous
        ):
            if phase in {"failed", "rejected"}:
                c.hold("omx_" + str(p.get("error_code", "failed")))
            elif phase == "completed":
                self.deadline = None
                if (
                    not -2 <= age <= 5
                    or p.get("departure_ready") is not True
                    or not returned(p)
                    or p.get("evidence") not in {"arm_return_only", "shelf_align"}
                ):
                    c.hold("omx_completion_not_safe")
                elif c.state["phase"] != "HOLD":
                    try:
                        c.complete_transfer(
                            cmd["payload"]["dock_session_id"], "mqtt", event
                        )
                    except ValueError:
                        c.hold("omx_next_step_not_ready")
        self.ack(event)
