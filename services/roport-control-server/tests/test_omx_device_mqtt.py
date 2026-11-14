"""MQTT/Host/Runner/PanelExecutor are real; only device, policy and vision I/O is fake."""

import json
import os
import sys
import threading
import time
from contextlib import ExitStack
from datetime import UTC, datetime, timedelta
from pathlib import Path
from queue import Empty
from types import SimpleNamespace
from uuid import uuid4

import pytest

SERVICE = Path(__file__).resolve().parents[1]
OMX = SERVICE.parent / "roport-omx"
for path in (OMX / "src", OMX / "scripts", OMX / "ci"):
    sys.path.insert(0, str(path))

from field_host import serve
from roport_omx.field_executors import (
    IndependentArmExecutor,
    PanelFieldExecutor,
    WarehouseExecutor,
)
from roport_omx.field_host import FieldOmxHost
from roport_omx.mqtt_link import field_wire as wire
from roport_omx.mqtt_link.journal import Journal
from roport_omx.mqtt_link.mqtt import MqttConfig, MqttTransport
from roport_omx.panel_executor import PanelExecutor, Route
from test_omx3_runner import FakeRobot, ScriptPolicy, make
from test_panel_executor import MODEL, FakePanel

VERSIONS = {"model_version": "test-model", "calibration_version": "test-cal"}


def now():
    return datetime.now(UTC).isoformat()


def test_v2_transport_does_not_import_legacy_pinky_motion_dependencies():
    transport = MqttTransport(
        MqttConfig("127.0.0.1", "test-omx"), [wire.topic("omx3", "cmd")]
    )
    with pytest.raises(ValueError):
        transport.publish_motion("invalid", {"type": "robot_pose"})


class WallClock:
    """Keep the actual Runner's timing/verification loop, without a motor."""

    def __call__(self):
        return time.monotonic()

    @staticmethod
    def sleep(seconds):
        time.sleep(seconds)


def runner_executor(
    target, *, routes=(), marker_ids=None, package_sizes=None, versions=None
):
    """Reusable actual Runner executor; fixtures replace only Robot/Policy/Marker ports.

    All routes use test calibration, never a deployable physical slot calibration.
    Return executor and I/O counters so an outer service test can prove action calls.
    """
    robot, policy = FakeRobot(), ScriptPolicy(away=4)
    # Reset each episode through the actual policy.reset(), even on repeated commands.
    runner, markers = make(
        robot,
        policy,
        lambda frame: (300.0, 300.0) if policy.n < 4 else (480.0, 70.0),
        WallClock(),
    )
    kwargs = {"marker_ids": marker_ids or {"package-17": 17}, **(versions or VERSIONS)}
    if target == "omx3":
        executor = WarehouseExecutor(runner, package_sizes=package_sizes, **kwargs)
    else:
        executor = IndependentArmExecutor(
            target, {key: runner for key in routes}, **kwargs
        )
    return SimpleNamespace(
        executor=executor, robot=robot, policy=policy, runner=runner, markers=markers
    )


def panel_device_executor(
    stack, *, package_id, arrival_observation, versions=None, marker_id=17
):
    """Actual PanelExecutor over simulated HTTP/device/shelf sensor observations.

    arrival_observation(task) reads the fixture's currently observed Pinky visit.
    The fixture's fixed marker/package association is not a production inventory lookup.
    """
    panel = FakePanel()
    stack.callback(panel.server.server_close)
    stack.callback(panel.server.shutdown)
    routes = {
        (op, slot): Route(MODEL, op + " " + slot + " marker {box}", 40, 15)
        for op in ("INBOUND", "OUTBOUND")
        for slot in ("1b", "2b")
    }
    actual = PanelExecutor(routes, box_marker=marker_id, base_url=panel.url)
    sensor = {"run": 0, "started": None, "placed": False}

    def tick():
        if len(panel.starts) != sensor["run"]:
            sensor.update(run=len(panel.starts), started=time.monotonic(), placed=False)
        if (
            sensor["started"] is not None
            and not sensor["placed"]
            and time.monotonic() - sensor["started"] >= 0.15
        ):
            panel.finish()
            sensor["placed"] = True

    def placement(task):
        if not sensor["placed"]:
            return None
        return {
            "command_id": task["command_id"],
            "package_id": package_id,
            "slot_id": panel.starts[-1]["task"].split()[1],
            "arrival_command_id": arrival_observation(task),
            "observed_at": now(),
            "placement_confirmed": True,
            "release_confirmed": True,
        }

    executor = PanelFieldExecutor(
        actual,
        **(versions or VERSIONS),
        completion=placement,
        stop=lambda: panel.finish(exit_ok=False, stopped_by="중지"),
    )
    return SimpleNamespace(executor=executor, panel=panel, tick=tick, sensor=sensor)


@pytest.mark.parametrize(
    "target,direction,node,slot",
    [
        ("omx3", "LOAD", "N01", None),
        ("omx1", "INBOUND", "N21", "1a"),
        ("omx1", "OUTBOUND", "N21", "2a"),
        ("omx2", "INBOUND", "N22", "1c"),
        ("omx2", "OUTBOUND", "N22", "2c"),
        ("shelf-01", "INBOUND", "N11", "2b"),
        ("shelf-01", "OUTBOUND", "N11", "2b"),
    ],
)
def test_device_executor_command_to_saved_result_over_mqtt(
    broker, tmp_path, target, direction, node, slot
):
    port, package = broker[0], "package-17"
    command_id, arrival = str(uuid4()), "arrival-before-transfer"
    panel, robot, policy, markers = None, None, None, []
    failures, events, acks = [], [], []
    with ExitStack() as stack:
        if target == "shelf-01":
            device = panel_device_executor(
                stack, package_id=package, arrival_observation=lambda task: arrival
            )
            panel, executor = device.panel, device.executor
        else:
            device = runner_executor(
                target,
                routes=[(direction, slot)],
                marker_ids={package: 17},
                package_sizes={package: "large"},
            )
            executor, robot, policy = device.executor, device.robot, device.policy
            runner, markers = device.runner, device.markers
        host_journal = Journal(tmp_path / "host.json")
        stack.callback(host_journal.close)
        received = Journal(tmp_path / "received.json")
        stack.callback(received.close)
        received.save({"events": []})
        host = FieldOmxHost(
            host_journal,
            {target: executor},
            alignment=lambda _: {
                "pinky_id": "pinky1",
                "pinky_boot_version": "device-test-pinky-boot",
                "arrival_command_id": arrival,
                "observed_at": now(),
                "alignment": "confirmed",
            },
        )
        command_payload = {
            "expected_boot_version": host.boot_version,
            "package_id": package,
            "expected_pinky_boot_version": "device-test-pinky-boot",
            "arrival_command_id": arrival,
            **VERSIONS,
        }
        if target == "omx3":
            kind = "warehouse_load_command"
            command_payload.update(
                pinky_id="pinky1", pickup_location_id="pickup-01", load_position="deck"
            )
        elif target == "shelf-01":
            kind = "shelf_transfer_command"
            command_payload.update(
                operation="store" if direction == "INBOUND" else "retrieve",
                slot_id=slot,
                pinky_id="pinky1",
                arms=[
                    {"robot_id": a, "expected_boot_version": host.boot_version}
                    for a in ("omx1", "omx2")
                ],
            )
        else:
            kind = "arm_action_command"
            pinky = {"kind": "pinky", "robot_id": "pinky1", "position": "deck"}
            shelf = {"kind": "shelf", "slot_id": slot}
            command_payload.update(
                operation="transfer",
                workcell_id="shelf-01",
                source_location=pinky if direction == "INBOUND" else shelf,
                destination_location=shelf if direction == "INBOUND" else pinky,
            )
        command = wire.build_message(
            kind,
            "control-01",
            target,
            command_payload,
            command_id=command_id,
            expires_at=(datetime.now(UTC) + timedelta(seconds=15)).isoformat(),
        )
        controller = MqttTransport(
            MqttConfig("127.0.0.1", "device-test-control", port=port),
            [wire.topic(target, "event")],
        )
        endpoint = MqttTransport(
            MqttConfig("127.0.0.1", "device-test-host", port=port),
            [wire.topic(target, c) for c in ("cmd", "ack")]
            + [wire.topic("pinky1", "event")],
        )
        controller.start()
        stack.callback(controller.stop)
        endpoint.start()
        stack.callback(endpoint.stop)
        stop = threading.Event()

        def run():
            try:
                serve(host, endpoint, stop=stop)
            except Exception as exc:  # noqa: BLE001 - forward thread exceptions to the test
                failures.append(exc)

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        stack.callback(thread.join, 5)
        stack.callback(stop.set)
        seen, last_state = 0, 0.0

        def pump(predicate, timeout=10):
            nonlocal seen, last_state
            deadline = time.monotonic() + timeout
            while not predicate():
                assert not failures, failures
                if time.monotonic() - last_state >= 0.5:
                    seen += 1
                    state = wire.build_message(
                        "robot_state",
                        "pinky1",
                        "control-01",
                        {
                            "boot_version": "device-test-pinky-boot",
                            "observed_at": now(),
                            "sequence": seen,
                            "active_command_id": None,
                            "state": {
                                "node": node,
                                "arrival_command_id": arrival,
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
                    controller.publish(wire.topic("pinky1", "event"), state)
                    last_state = time.monotonic()
                if panel:
                    device.tick()
                try:
                    delivery = controller.receive(0.02)
                except Empty:
                    delivery = None
                if delivery:
                    event = wire.decode(delivery.payload)
                    if event["type"] == "command_status":
                        events.append(event)
                        received.save({"events": list(events)})
                        assert event in received.data["events"]
                        ack = wire.build_message(
                            "event_stored",
                            "control-01",
                            target,
                            {"acked_message_id": event["message_id"]},
                        )
                        controller.publish(wire.topic(target, "ack"), ack)
                        acks.append(ack)
                    controller.ack(delivery)
                assert time.monotonic() < deadline, {
                    "events": events,
                    "host": host.data,
                }

        pump(lambda: bool(host.pinkies))
        controller.publish(wire.topic(target, "cmd"), command)
        pump(lambda: any(e["payload"]["status"] == "completed" for e in events))
        pump(lambda: not host.pending())
        statuses = [e["payload"]["status"] for e in events]
        assert {"accepted", "in_progress", "completed"} <= set(statuses)
        assert not {"failed", "rejected", "interrupted"} & set(statuses)
        result = next(
            e["payload"]["result"]
            for e in events
            if e["payload"]["status"] == "completed"
        )
        assert (
            result["package_id"] == package and result["arrival_command_id"] == arrival
        )
        assert result["placement_confirmed"] and result["release_confirmed"]
        assert all(
            a["home_reached"] and a["holding_state"] == "empty" for a in result["arms"]
        )
        if slot:
            assert result["slot_id"] == slot
        if target == "omx3":
            assert result["package_size"] == "large"
        if robot:
            assert robot.sent >= 4 and robot.held >= 1
            assert policy.frames and all("marked" in f for f in policy.frames)
            assert any(m.marked >= runner.settings.done_hold_frames for m in markers)
            calls = {
                "robot_send": robot.sent,
                "robot_hold": robot.held,
                "policy_act": policy.n,
                "vision_frames": sum(m.marked for m in markers),
            }
        else:
            assert len(panel.starts) == 1 and panel.starts[0]["source"] == "mqtt"
            assert panel.arms_models and panel.status["last_result"]["exit_ok"]
            calls = {
                "panel_http_start": len(panel.starts),
                "panel_motor_reads": len(panel.arms_models),
            }
        # Exact replay produces the durable terminal event, not a second device execution.
        original_calls = dict(calls)
        before = len(events)
        controller.publish(wire.topic(target, "cmd"), command)
        pump(lambda: len(events) > before)
        pump(lambda: not host.pending())
        assert (robot.sent if robot else len(panel.starts)) == original_calls.get(
            "robot_send", 1
        )
        output = os.environ.get("ROPORT_DEVICE_EVIDENCE_DIR")
        if output:
            path = Path(output)
            path.mkdir(parents=True, exist_ok=True)
            (path / (target + "-" + direction.lower() + ".json")).write_text(
                json.dumps(
                    {
                        "simulated_boundaries": ["robot_io", "policy_io", "vision_io"]
                        if robot
                        else ["panel_http_device", "placement_sensor"],
                        "real_components": [
                            "MQTT",
                            "schema",
                            "FieldOmxHost",
                            type(executor).__name__,
                            "Runner" if robot else "PanelExecutor",
                        ],
                        "command": command,
                        "events": events,
                        "saved_acks": acks,
                        "device_calls": calls,
                        "result": "passed",
                    },
                    indent=2,
                )
            )
