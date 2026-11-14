"""Real MQTT + real endpoint/driver/line logic; only device I/O and Nav2 are simulated."""

import sys
import threading
import time
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace

import pytest

SERVICE = Path(__file__).resolve().parents[1]
OMX = SERVICE.parent / "roport-omx"
for path in (SERVICE / "tools", OMX / "src", OMX / "scripts"):
    sys.path.insert(0, str(path))

from field_host import serve
from field_mock import virtual_alignment, virtual_executors
from roport_omx.field_host import FieldOmxHost
from run_virtual_fleet import profiles
from virtual_pinky import VirtualPinky

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.field_pinky import FieldPinkyClient
from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.mqtt.mqtt import MqttConfig, MqttTransport
from roport_control_server.executor.fleet import ROLES, TARGETS, FleetControl
from roport_control_server.executor.fleet_runtime import FleetRuntime


@pytest.fixture
def fleet_mqtt(broker, tmp_path):
    port = broker[0]
    config, robots = profiles()
    errors = []
    with ExitStack() as stack:
        store = Journal(tmp_path / "control.json")
        stack.callback(store.close)
        control = FleetControl(config, store)
        transport = MqttTransport(
            MqttConfig("127.0.0.1", "fleet-e2e-control", port=port),
            [wire.topic(r, "event") for r in TARGETS]
            + [wire.topic(r, "motion") for r in ROLES],
        )
        transport.start()
        stack.callback(transport.stop)
        runtime = FleetRuntime(control, transport)
        clients = []
        for robot, (role, profile) in robots.items():
            driver = VirtualPinky(profile, role, threaded=False)
            stack.callback(driver.close)
            client = FieldPinkyClient(
                MqttConfig("127.0.0.1", "fleet-e2e-" + robot, port=port),
                profile,
                role,
                tmp_path / (robot + ".json"),
                driver,
            )
            client.start()
            stack.callback(client.close)
            clients.append(client)
        omx_store = Journal(tmp_path / "omx.json")
        stack.callback(omx_store.close)
        executors = virtual_executors(0.01, package_sizes={"large": "large"})
        host = FieldOmxHost(omx_store, executors, control=config["control_id"])
        host.alignment = lambda cmd: virtual_alignment(host, cmd)
        omx_transport = MqttTransport(
            MqttConfig("127.0.0.1", "fleet-e2e-omx", port=port),
            [wire.topic(r, c) for r in executors for c in ("cmd", "ack")]
            + [wire.topic(r, "event") for r in ROLES],
        )
        omx_transport.start()
        stack.callback(omx_transport.stop)
        stop = threading.Event()

        def run_omx():
            try:
                serve(host, omx_transport, stop=stop)
            except Exception as exc:  # noqa: BLE001 - propagate worker failure to test owner
                errors.append(exc)

        worker = threading.Thread(target=run_omx, daemon=True)
        worker.start()
        stack.callback(worker.join, 5)
        stack.callback(stop.set)

        def pump_until(predicate, timeout=60):
            deadline = time.monotonic() + timeout
            while not predicate():
                assert not errors, errors
                assert not control.data["fault"], control.data
                for client in clients:
                    # Real sensor callbacks run independently of MQTT handling;
                    # refresh the manual simulator before answering a queued poll.
                    client.driver.tick()
                    client.poll(0)
                    if client.transport.ready and not client.halted:
                        client.driver.heartbeat()
                    # Faster virtual sensor time; the real driver and its wall-clock
                    # stopping/mode-transition checks remain unchanged.
                    for _ in range(100):
                        client.driver.tick()
                    for command, phase, details in client.driver.drain_results():
                        client.report(command, phase, **details)
                    client.publish_motion()
                for _ in range(12):
                    runtime.poll(0)
                if time.monotonic() >= deadline:
                    raise AssertionError(
                        {
                            "control": control.data,
                            "robots": {
                                c.robot: {
                                    "state": c.driver.observe(),
                                    "halted": c.halted,
                                    "active": c.journal.data["active"],
                                }
                                for c in clients
                            },
                        }
                    )
                time.sleep(0.002)

        pump_until(lambda: len(control.observed) == len(TARGETS))
        control.enable()

        def submit(package, operation):
            key = operation + "-" + package
            control.submit(
                {
                    "request_id": key,
                    "operation": operation,
                    "package_id": package,
                    "goal": "PUBLIC_SPACE_TRIAL" if operation == "outbound" else None,
                }
            )
            return key

        yield SimpleNamespace(
            control=control,
            clients=clients,
            host=host,
            executors=executors,
            transport=transport,
            omx_transport=omx_transport,
            pump=pump_until,
            submit=submit,
            port=port,
            robots=robots,
            tmp_path=tmp_path,
            stack=stack,
        )


def test_three_pinkies_all_shelf_columns_and_nav2_return_over_mqtt(fleet_mqtt):
    env = fleet_mqtt
    control, clients, executors = env.control, env.clients, env.executors
    pump_until, submit = env.pump, env.submit
    # Three small items exceed either column capacity, exercising both independent arms.
    for package in ("left-first", "left-second", "right", "large"):
        key = submit(package, "inbound")
        pump_until(lambda key=key: control.data["tasks"][key]["status"] == "COMPLETE")
        if package == "left-first":
            control.assign("pinky1", None)
        elif package == "left-second":
            control.assign("pinky1", "logistics")
    small = {
        control.data["tasks"]["inbound-" + p]["slot"][-1]: p
        for p in ("left-first", "left-second", "right")
    }
    assert set(small) == {"a", "c"}
    assert control.data["tasks"]["inbound-large"]["slot"].endswith("b")
    assert {
        control.data["tasks"]["inbound-" + p]["robot"]
        for p in ("left-first", "left-second", "right", "large")
    } == {"pinky1", "pinky2"}
    for package in (small["a"], small["c"], "large"):
        key = submit(package, "outbound")
        pump_until(
            lambda key=key: control.data["tasks"][key]["stage"] == "await_receipt"
        )
        # Arrival is not customer receipt; an explicit receipt releases return.
        assert control.data["tasks"][key]["status"] != "COMPLETE"
        control.receipt(key)
        pump_until(lambda key=key: control.data["tasks"][key]["status"] == "COMPLETE")
        assert control.observed["pinky3"]["state"]["node"] in {"H1", "H2"}
    cart = next(c for c in clients if c.robot == "pinky3")
    commands = [r["command"] for r in cart.journal.data["records"].values()]
    assert sum(c["type"] == "nav_goal_command" for c in commands) == 6
    assert sum(c["type"] == "set_drive_mode_command" for c in commands) == 6
    assert {c["payload"].get("to_mode") for c in commands} >= {"line", "nav2"}
    assert len(executors["omx3"].tasks) == 4
    assert sorted(len(executors[a].tasks) for a in ("omx1", "omx2")) == [2, 3]
    assert len(executors["shelf-01"].tasks) == 2
    assert not env.host.data["hold"]


def test_one_arm_failure_holds_before_departure_over_mqtt(fleet_mqtt):
    env = fleet_mqtt
    env.executors["shelf-01"].run_s = 0.4
    key = env.submit("large", "inbound")
    env.pump(
        lambda: (
            bool(env.host.data["active"])
            and env.host.data["commands"][env.host.data["active"]]["command"]["target"]
            == "shelf-01"
        )
    )
    env.executors["shelf-01"].unreturned.add("omx2")
    env.pump(lambda: bool(env.control.data["fault"]))
    task = env.control.data["tasks"][key]
    assert task["status"] == "HOLD"
    assert task["stage"] == "transfer"
    assert env.control.data["slots"][task["slot"]]["state"] != "occupied"
    assert env.executors["shelf-01"].aborted
    assert env.control.next_action() is None


def test_duplicate_and_late_real_host_result_do_not_advance_over_mqtt(fleet_mqtt):
    env = fleet_mqtt
    env.executors["omx3"].run_s = 0.4
    key = env.submit("first", "inbound")
    env.pump(lambda: bool(env.host.data["active"]))
    original = next(iter(env.control.data["commands"].values()))["message"]
    for _ in range(2):
        env.transport.publish(wire.topic(original["target"], "cmd"), original)
    env.pump(lambda: env.control.data["tasks"][key]["status"] == "COMPLETE")
    assert len(env.executors["omx3"].tasks) == 1
    old = env.host.data["commands"][original["command_id"]]["last_event"]
    next_key = env.submit("second", "inbound")
    env.pump(lambda: bool(env.control.data["commands"]))
    active = next(iter(env.control.data["commands"].values()))["message"]["command_id"]
    env.omx_transport.publish(wire.topic(old["source"], "event"), old)
    env.pump(lambda: env.control.data["tasks"][next_key]["status"] == "COMPLETE")
    assert active != old["command_id"]
    assert len(env.executors["omx3"].tasks) == 2
    assert sum(len(env.executors[a].tasks) for a in ("omx1", "omx2")) == 2
    assert not env.control.data["fault"]


def test_robot_restart_holds_active_command_over_mqtt(fleet_mqtt):
    env = fleet_mqtt
    key = env.submit("restart", "inbound")
    env.pump(lambda: any(c.journal.data["active"] for c in env.clients))
    index = next(i for i, c in enumerate(env.clients) if c.journal.data["active"])
    old = env.clients[index]
    old.close()
    role, profile = env.robots[old.robot]
    replacement = FieldPinkyClient(
        MqttConfig("127.0.0.1", "fleet-restarted-" + old.robot, port=env.port),
        profile,
        role,
        env.tmp_path / (old.robot + ".json"),
        old.driver,
    )
    replacement.start()
    env.stack.callback(replacement.close)
    env.clients[index] = replacement
    env.pump(lambda: bool(env.control.data["fault"]))
    assert replacement.boot != old.boot
    assert env.control.data["tasks"][key]["status"] == "HOLD"
    assert not env.executors["omx3"].tasks
    assert env.control.next_action() is None
