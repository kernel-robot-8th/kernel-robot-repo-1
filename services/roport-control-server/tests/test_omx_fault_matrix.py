"""Adversarial OMX observations. Keep failing expectations; do not patch product code here."""

import json
import os
import re
import sys
import threading
import time
from contextlib import ExitStack
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from pathlib import Path
from queue import Empty
from uuid import uuid4

import pytest
from test_omx_device_mqtt import VERSIONS, panel_device_executor, runner_executor

SERVICE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVICE / "tools"))
from field_host import serve
from roport_omx.field_host import FieldOmxHost
from roport_omx.mqtt_link import field_wire as wire
from roport_omx.mqtt_link.journal import Journal
from roport_omx.mqtt_link.mqtt import MqttConfig, MqttTransport
from run_virtual_fleet import profiles
from test_panel_executor import arm

from roport_control_server.executor.fleet import FleetControl

NODES = {"omx3": "N01", "omx1": "N21", "omx2": "N22", "shelf-01": "N11"}
SLOTS = {"omx1": "1a", "omx2": "1c", "shelf-01": "1b"}


def now(offset=0):
    return (datetime.now(UTC) + timedelta(seconds=offset)).isoformat()


class Rig:
    def __init__(self, stack, tmp_path, name, target="shelf-01"):
        self.name, self.target, self.events = name, target, []
        self.transport_kind = "direct-contract"
        self.devices = {
            t: runner_executor(
                t,
                routes=[("INBOUND", SLOTS.get(t))],
                package_sizes={"package-17": "large"},
            )
            for t in ("omx1", "omx2", "omx3")
        }
        self.panel = panel_device_executor(
            stack, package_id="package-17", arrival_observation=lambda _: "arrival-1"
        )
        self.devices["shelf-01"] = self.panel
        self.journal = Journal(tmp_path / (name + "-host.json"))
        stack.callback(self.journal.close)
        self.sensor = {
            "pinky_id": "pinky1",
            "pinky_boot_version": "pinky-boot",
            "arrival_command_id": "arrival-1",
            "observed_at": now(),
            "alignment": "confirmed",
        }
        self.host = FieldOmxHost(
            self.journal,
            {k: d.executor for k, d in self.devices.items()},
            alignment=lambda _: dict(self.sensor),
        )
        self.pinky = wire.build_message(
            "robot_state",
            "pinky1",
            "control-01",
            {
                "boot_version": "pinky-boot",
                "observed_at": now(),
                "sequence": 1,
                "active_command_id": None,
                "state": {
                    "node": NODES[target],
                    "arrival_command_id": "arrival-1",
                    "mode": "line",
                    "stopped": True,
                    "line_enabled": False,
                    "nav2_ready": False,
                    "localization_ready": False,
                    "drive_owner": "none",
                    "graph_version": "test-graph",
                    "map_version": "test-map",
                    "heading_rad": 0,
                    "fault": None,
                },
            },
        )
        self.command = self.make_command(target)
        self.host.observe_pinky(self.pinky)
        config, _ = profiles()
        self.control_store = Journal(tmp_path / (name + "-control.json"))
        stack.callback(self.control_store.close)
        self.control = FleetControl(config, self.control_store, clock=lambda: 1)
        self.control.observed["pinky1"] = deepcopy(self.pinky["payload"])
        for t in self.devices:
            self.control.observed[t] = self.host.state(t)["payload"]
        self.control.heard = {k: 1 for k in self.control.observed}
        data = deepcopy(self.control.data)
        self.key = "inbound-case"
        self.stage = "warehouse_load" if target == "omx3" else "transfer"
        data["enabled"] = True
        data["tasks"][self.key] = {
            "request": {
                "request_id": self.key,
                "operation": "inbound",
                "package_id": "package-17",
                "goal": None,
            },
            "status": "RUNNING",
            "stage": self.stage,
            "robot": "pinky1",
            "arrival": "arrival-1",
            "parking": "N16",
            "reason": None,
        }
        if target in SLOTS:
            data["tasks"][self.key]["slot"] = SLOTS[target]
            data["slots"][SLOTS[target]] = {
                "state": "reserved",
                "package_id": "package-17",
            }
        self.control.save(data)
        self.control.dispatched(self.key, self.command)

    def make_command(self, target):
        p = {
            "expected_boot_version": self.host.boot_version,
            "expected_pinky_boot_version": "pinky-boot",
            "arrival_command_id": "arrival-1",
            "package_id": "package-17",
            **VERSIONS,
        }
        if target == "shelf-01":
            kind = "shelf_transfer_command"
            p.update(
                operation="store",
                slot_id="1b",
                pinky_id="pinky1",
                arms=[
                    {"robot_id": a, "expected_boot_version": self.host.boot_version}
                    for a in ("omx1", "omx2")
                ],
            )
        elif target == "omx3":
            kind = "warehouse_load_command"
            p.update(
                pinky_id="pinky1", pickup_location_id="pickup-01", load_position="deck"
            )
        else:
            kind = "arm_action_command"
            p.update(
                operation="transfer",
                workcell_id="shelf-01",
                source_location={
                    "kind": "pinky",
                    "robot_id": "pinky1",
                    "position": "deck",
                },
                destination_location={"kind": "shelf", "slot_id": SLOTS[target]},
            )
        return wire.build_message(
            kind, "control-01", target, p, command_id=str(uuid4()), expires_at=now(10)
        )

    def refresh_pinky(self):
        self.pinky["payload"]["sequence"] += 1
        self.host.observe_pinky(self.pinky)

    def consume(self, events):
        for event in events:
            self.events.append(deepcopy(event))
            try:
                self.control.result(event)
            except ValueError as exc:
                self.control.hold(str(exc))

    def finish(self, timeout=2):
        deadline = time.monotonic() + timeout
        while self.host.data["active"] and time.monotonic() < deadline:
            self.panel.tick()
            self.consume(self.host.poll())
            time.sleep(0.01)

    def calls(self):
        return {
            k: d.robot.sent if k != "shelf-01" else len(d.panel.starts)
            for k, d in self.devices.items()
        }

    def evidence(self, expectation):
        task = self.control.data["tasks"][self.key]
        safe = task["stage"] == self.stage and not task.get("departure")
        if self.target in SLOTS:
            safe &= (
                self.control.data["slots"][SLOTS[self.target]]["state"] != "occupied"
            )
        data = {
            "case": self.name,
            "transport": self.transport_kind,
            "expectation": expectation,
            "safe_no_advance": bool(safe),
            "command": self.command,
            "pinky": self.pinky,
            "alignment": self.sensor,
            "events": self.events,
            "device_calls": self.calls(),
            "panel_observations": {
                "status": deepcopy(self.panel.panel.status),
                "arms": deepcopy(self.panel.panel.arms),
            },
            "robot_observations": {
                k: {
                    "joints": list(d.robot.state),
                    "policy_actions": d.policy.n,
                    "holds": d.robot.held,
                }
                for k, d in self.devices.items()
                if k != "shelf-01"
            },
            "host": self.host.data,
            "control": self.control.data,
        }
        output = os.environ.get("ROPORT_OMX_MATRIX_EVIDENCE_DIR")
        if output:
            root = Path(output)
            root.mkdir(parents=True, exist_ok=True)
            (root / (re.sub(r"[^a-zA-Z0-9_.-]", "_", self.name) + ".json")).write_text(
                json.dumps(data, indent=2)
            )
        return safe


@pytest.fixture
def rig_factory(tmp_path, request):
    with ExitStack() as stack:
        rigs = []

        def make(target="shelf-01"):
            rig = Rig(stack, tmp_path, request.node.name, target)
            rigs.append(rig)
            return rig

        yield make
        for rig in rigs:
            for target, device in rig.devices.items():
                if target != "shelf-01":
                    host = (
                        device.executor.host
                        if target == "omx3"
                        else device.executor.current.host
                        if device.executor.current
                        else None
                    )
                    if host:
                        host.abort()
                        host.wait(2)


PREFLIGHT = [
    ("omx3", "wrong_node"),
    ("omx1", "wrong_node"),
    ("omx2", "wrong_node"),
    ("shelf-01", "wrong_node"),
    ("shelf-01", "wrong_robot"),
    ("shelf-01", "wrong_arrival"),
    ("omx3", "wrong_pinky_boot"),
    ("shelf-01", "wrong_host_boot"),
    ("omx1", "stale_state"),
    ("omx2", "unconfirmed_alignment"),
    ("shelf-01", "stale_alignment"),
    ("shelf-01", "drive_owner"),
    ("omx1", "line_enabled"),
    ("omx3", "active_pinky_command"),
    ("shelf-01", "arm_busy"),
    ("shelf-01", "arm_fault"),
    ("shelf-01", "arm_not_home"),
    ("shelf-01", "arm_holding"),
    ("omx1", "wrong_slot"),
    ("omx3", "unknown_package"),
]


def mutate(r, fault):
    p, state = r.pinky["payload"], r.pinky["payload"]["state"]
    if fault == "wrong_node":
        state["node"] = "N16"
    elif fault == "wrong_robot":
        r.host.pinkies.clear()
        r.pinky["source"] = "pinky2"
    elif fault == "wrong_arrival":
        state["arrival_command_id"] = "previous-arrival"
    elif fault == "wrong_pinky_boot":
        p["boot_version"] = "rebooted-pinky"
    elif fault == "wrong_host_boot":
        r.command["payload"]["expected_boot_version"] = "previous-host"
    elif fault == "stale_state":
        p["observed_at"] = now(-10)
    elif fault == "unconfirmed_alignment":
        r.sensor["alignment"] = "unconfirmed"
    elif fault == "stale_alignment":
        r.sensor["observed_at"] = now(-10)
    elif fault == "drive_owner":
        state["drive_owner"] = "line"
    elif fault == "line_enabled":
        state["line_enabled"] = True
    elif fault == "active_pinky_command":
        p["active_command_id"] = "unfinished-route"
    elif fault == "arm_busy":
        r.panel.panel.status["job"]["running"] = True
    elif fault == "arm_fault":
        r.panel.panel.arms["OMX 2"]["joints"] = {}
    elif fault == "arm_not_home":
        r.panel.panel.arms["OMX 2"]["at_home"] = False
    elif fault == "arm_holding":
        r.panel.panel.arms["OMX 2"]["gripper"] = 59.6
    elif fault == "wrong_slot":
        r.command["payload"]["destination_location"]["slot_id"] = "1c"
    elif fault == "unknown_package":
        r.command["payload"]["package_id"] = "package-not-mapped"
    else:
        raise AssertionError(fault)
    r.refresh_pinky()


@pytest.mark.parametrize("target,fault", PREFLIGHT)
def test_preflight_must_block_motion_and_inventory(rig_factory, target, fault):
    r = rig_factory(target)
    mutate(r, fault)
    r.consume(r.host.handle(r.command))
    r.finish()
    safe = r.evidence(
        "Reject before device action; no inventory or departure authorization"
    )
    assert all(e["payload"]["status"] == "rejected" for e in r.events), r.events
    assert not any(r.calls().values()), r.calls()
    assert safe, r.control.data


@pytest.mark.parametrize(
    "fault", ["drive_owner", "line_enabled", "active_pinky_command"]
)
def test_drive_reactivation_during_transfer_holds_inventory(rig_factory, fault):
    r = rig_factory()
    r.consume(r.host.handle(r.command))
    assert r.events[0]["payload"]["status"] == "accepted"
    assert len(r.panel.panel.starts) == 1
    mutate(r, fault)
    r.finish()
    assert r.evidence(
        "Pinky drive reactivation during transfer must hold inventory and departure"
    ), r.control.data
    assert r.events[-1]["payload"]["status"] == "failed"
    assert r.events[-1]["payload"]["reason"] == "pinky_visit_not_confirmed"
    assert r.host.data["hold"] and r.control.data["fault"]


@pytest.mark.parametrize(
    "fault", ["wrong_package", "wrong_slot", "partial_arm", "late_after_watchdog"]
)
def test_device_result_fault_cannot_release_pinky(rig_factory, fault):
    r = rig_factory()
    r.consume(r.host.handle(r.command))
    if fault in {"wrong_package", "wrong_slot"}:
        original = r.panel.executor.completion

        def bad(task):
            proof = original(task)
            if proof:
                proof["package_id" if fault == "wrong_package" else "slot_id"] = (
                    "other-package" if fault == "wrong_package" else "2b"
                )
            return proof

        r.panel.executor.completion = bad
    elif fault == "partial_arm":
        r.panel.panel.finish(
            arms={"OMX 1": arm("left"), "OMX 2": arm("right", home=False)}
        )
        r.panel.sensor.update(run=1, placed=True)
    else:
        r.host.deadline = 0
    r.finish()
    if fault == "late_after_watchdog":
        # A device result can arrive after the Host has already declared uncertainty.
        r.panel.panel.finish()
        r.panel.sensor.update(run=1, placed=True)
        r.consume(r.host.poll())
    assert r.evidence(
        "Partial/mismatched/late device result holds inventory and Pinky"
    ), r.control.data
    assert r.control.data["fault"], r.events


@pytest.mark.parametrize("first,second", [("shelf-01", "omx1"), ("omx1", "shelf-01")])
def test_cooperative_and_independent_requests_cannot_overlap(
    rig_factory, first, second
):
    r = rig_factory(first)
    r.consume(r.host.handle(r.command))
    other = r.make_command(second)
    reply = r.host.handle(other)
    r.events.extend(reply)
    r.evidence("Second request sharing arm resources must not execute")
    assert reply[0]["payload"]["status"] == "rejected"
    assert reply[0]["payload"]["reason"] == "workcell_held_or_busy"
    assert r.calls()[second] == 0


def test_exact_duplicate_replays_without_another_device_action(rig_factory):
    r = rig_factory("omx3")
    r.consume(r.host.handle(r.command))
    r.finish()
    before = r.calls()
    r.consume(r.host.handle(r.command))
    r.evidence("Same command is replayed; no second action or extra stage advance")
    assert r.calls() == before
    assert r.control.data["tasks"][r.key]["stage"] == "choose_slot"
    assert r.host.data["commands"][r.command["command_id"]]["status"] == "completed"


def test_restart_then_old_completion_cannot_release_pinky(rig_factory):
    r = rig_factory()
    r.consume(r.host.handle(r.command))
    replacement = FieldOmxHost(
        r.journal, r.host.executors, alignment=lambda _: dict(r.sensor)
    )
    r.control.observe(replacement.state("shelf-01"))
    r.finish()
    assert r.evidence(
        "Host reboot keeps ownership unknown even if old owner completes later"
    ), r.control.data
    assert replacement.data["hold"] == "host_restarted_outcome_unknown"
    assert r.control.data["fault"]


@pytest.mark.parametrize("fault", ["wrong_node", "active_pinky_command"])
def test_mqtt_host_rejects_unsafe_visit_without_inventory_advance(
    rig_factory, broker, fault
):
    r = rig_factory()
    r.transport_kind = "actual-loopback-MQTT"
    mutate(r, fault)
    r.host.pinkies.clear()  # The endpoint must receive the bad observation through MQTT.
    with ExitStack() as stack:
        controller = MqttTransport(
            MqttConfig("127.0.0.1", "matrix-control", port=broker[0]),
            [wire.topic("shelf-01", "event")],
        )
        endpoint = MqttTransport(
            MqttConfig("127.0.0.1", "matrix-host", port=broker[0]),
            [wire.topic(t, c) for t in r.devices for c in ("cmd", "ack")]
            + [wire.topic("pinky1", "event")],
        )
        for transport in (controller, endpoint):
            transport.start()
            stack.callback(transport.stop)
        stopped, errors = threading.Event(), []

        def serve_host():
            try:
                serve(r.host, endpoint, stop=stopped)
            except Exception as exc:  # noqa: BLE001 - forward worker failure
                errors.append(repr(exc))

        worker = threading.Thread(target=serve_host, daemon=True)
        worker.start()
        stack.callback(worker.join, 5)
        stack.callback(stopped.set)
        controller.publish(wire.topic("pinky1", "event"), r.pinky)
        deadline = time.monotonic() + 2
        while not r.host.pinkies and time.monotonic() < deadline:
            time.sleep(0.01)
        assert r.host.pinkies
        controller.publish(wire.topic("shelf-01", "cmd"), r.command)
        deadline = time.monotonic() + 3
        terminal = False
        while not terminal and time.monotonic() < deadline:
            r.panel.tick()
            try:
                delivery = controller.receive(0.02)
            except Empty:
                continue
            event = wire.decode(delivery.payload)
            if event["type"] == "command_status":
                r.consume([event])
                terminal = event["payload"]["status"] in {
                    "completed",
                    "failed",
                    "rejected",
                    "interrupted",
                }
                controller.publish(
                    wire.topic("shelf-01", "ack"),
                    wire.build_message(
                        "event_stored",
                        "control-01",
                        "shelf-01",
                        {"acked_message_id": event["message_id"]},
                    ),
                )
            controller.ack(delivery)
        safe = r.evidence(
            "Actual MQTT unsafe visit cannot actuate, commit stock or authorize departure"
        )
        assert not errors, errors
        assert terminal and safe, r.control.data
        assert not r.panel.panel.starts, r.panel.panel.starts
