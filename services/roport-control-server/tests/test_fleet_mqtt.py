"""Real MQTT + real endpoint/driver/line logic; only device I/O and Nav2 are simulated."""

import sys
import threading
import time
from contextlib import ExitStack
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

import pytest

SERVICE = Path(__file__).resolve().parents[1]
OMX = SERVICE.parent / "roport-omx"
for path in (SERVICE / "tools", OMX / "src", OMX / "scripts"):
    sys.path.insert(0, str(path))

from field_host import serve
from field_mock import virtual_alignment, virtual_executors
from fleet_profiles import profiles
from roport_omx.field_host import FieldOmxHost
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
        host = FieldOmxHost(
            omx_store,
            executors,
            control=config["control_id"],
            alignment=virtual_alignment,
        )
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

        def pump_until(predicate, timeout=60, *, expected_fault=None):
            deadline = time.monotonic() + timeout
            while not predicate():
                assert not errors, errors
                assert control.data["fault"] in (None, expected_fault), control.data
                for client in clients:
                    # Real sensor callbacks run independently of MQTT handling;
                    # refresh the manual simulator before answering a queued poll.
                    client.driver.tick()
                    client.poll(0)
                    if client.transport.ready and not client.halted:
                        client.driver.heartbeat()
                    # Bound accelerated CPU work so the next MQTT-owner heartbeat
                    # runs before the real one-second drive lease expires on CI.
                    for _ in range(10):
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


def test_accelerated_sensors_yield_before_drive_lease_expires(fleet_mqtt, monkeypatch):
    env = fleet_mqtt
    env.submit("slow-cpu", "inbound")
    env.pump(lambda: any(c.journal.data["active"] for c in env.clients))
    client = next(c for c in env.clients if c.journal.data["active"])
    tick = client.driver.tick
    samples = 0

    def slow_tick():
        nonlocal samples
        if samples < 20:
            time.sleep(0.055)
        samples += 1
        tick()

    monkeypatch.setattr(client.driver, "tick", slow_tick)
    env.pump(lambda: samples >= 20)
    assert client.driver.observe()["fault"] is None
    assert not env.control.data["fault"]


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

    from roport_control_server.executor.fleet_recovery import request_host_release

    fault = env.control.data["fault"]
    cmd_id = next(
        cid
        for cid, cmd in env.control.data["commands"].items()
        if cmd["message"]["target"] == "shelf-01"
    )
    env.executors["shelf-01"].unreturned.clear()
    env.pump(
        lambda: (
            not env.host.data.get("recovery_command_id")
            and env.control.observed["shelf-01"]["state"]["fault"]
        ),
        expected_fault=fault,
    )
    request_host_release(
        env.control,
        "release-test",
        cmd_id,
        "operator-test",
        {"arms": True, "box": True, "pinky": True},
    )
    env.pump(
        lambda: (
            env.control.data["host_releases"]["release-test"]["status"] != "pending"
        ),
        expected_fault=fault,
    )
    assert env.control.data["host_releases"]["release-test"]["status"] == "completed"
    assert env.host.data["hold"] is None
    assert not env.control.data["enabled"] and env.control.data["fault"] == fault
    assert env.control.data["slots"][task["slot"]]["state"] != "occupied"


@pytest.mark.parametrize("verify_failed", [False, True])
def test_warehouse_runner_result_controls_pinky_departure(
    fleet_mqtt, monkeypatch, verify_failed
):
    from test_omx_device_mqtt import runner_executor

    env = fleet_mqtt
    device = runner_executor(
        "omx3",
        versions=env.executors["omx3"].versions,
        package_sizes={"package-17": "small"},
    )
    monkeypatch.setitem(env.executors, "omx3", device.executor)
    original_alignment = env.host.alignment

    def shelf_alignment(command, observation):
        assert command["type"] != "warehouse_load_command"
        return original_alignment(command, observation)

    monkeypatch.setattr(env.host, "alignment", shelf_alignment)
    if verify_failed:
        original_factory = device.runner.marker_factory

        def marker_factory(marker_id):
            marker = original_factory(marker_id)
            find = marker.find
            # The box leaves the tray at the final camera check, after the policy finishes.
            marker.find = lambda frame: None if marker.marked else find(frame)
            return marker

        monkeypatch.setattr(device.runner, "marker_factory", marker_factory)

    key = env.submit("package-17", "inbound")
    env.pump(
        lambda: (
            bool(env.control.data["fault"])
            if verify_failed
            else env.control.data["tasks"][key]["status"] == "COMPLETE"
        )
    )
    task = env.control.data["tasks"][key]
    load = next(
        r
        for r in env.host.data["commands"].values()
        if r["command"]["type"] == "warehouse_load_command"
    )
    event = load["last_event"]
    assert event in env.control.data["events"].values()
    assert device.robot.sent > 0 and device.policy.marker_id == 17
    if verify_failed:
        assert event["payload"]["status"] == "failed"
        assert event["payload"]["reason"] == "VERIFY_FAILED"
        assert task["status"] == "HOLD" and task["stage"] == "warehouse_load"
        assert env.control.data["fault"] == "VERIFY_FAILED"
        assert env.host.data["hold"]  # Fleet may already have requested a stop.
        assert env.control.next_action() is None
        client = next(c for c in env.clients if c.robot == task["robot"])
        assert env.control.observed[task["robot"]]["state"]["node"] == "N01"
        assert all(
            r["command"]["payload"]["route"][0] != "N01"
            for r in client.journal.data["records"].values()
            if r["command"]["type"] == "line_route_command"
        )
        assert not any(env.executors[a].tasks for a in ("omx1", "omx2", "shelf-01"))
    else:
        assert event["payload"]["status"] == "completed"
        assert event["payload"]["result"]["placement_confirmed"] is True
        assert task["status"] == "COMPLETE"


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


@pytest.mark.parametrize("restart_during_stop", [False, True])
def test_robot_restart_recovers_inherited_command_without_automatic_motion(
    fleet_mqtt, monkeypatch, restart_during_stop
):
    from test_fleet_socket import request

    from roport_control_server.executor.fleet_socket import FleetSocket

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
    runtime = FleetRuntime(env.control, env.transport)

    def held_until(predicate):
        deadline = time.monotonic() + 10
        while not predicate():
            for client in env.clients:
                client.driver.tick()
                client.poll(0)
            for _ in range(12):
                runtime.poll(0)
            assert not env.control.data["enabled"]
            assert env.control.next_action() is None
            assert time.monotonic() < deadline, env.control.snapshot()
            time.sleep(0.002)

    cid = replacement.journal.data["active"]
    held_until(lambda: bool(env.control.data["commands"][cid].get("stop")))
    assert env.control.observed[old.robot]["active_command_id"] == cid
    assert env.control.observed[old.robot]["state"]["fault"] == "recovery_required"
    resources = dict(env.control.data["resources"])
    old_record = replacement.journal.data["records"][cid]

    with TemporaryDirectory(prefix="rp-reboot-") as directory:
        owner = FleetSocket(Path(directory) / "s", env.control)
        env.stack.callback(owner.close)

        def operate(action, **fields):
            return request(
                owner,
                action=action,
                session_id=owner.session_id,
                expires_at=time.time() + 1,
                **fields,
            )

        assert not operate("release_hold", note="still unresolved")["ok"]
        assert operate(
            "recover_stop",
            command_id=cid,
            stop_id=env.control.data["commands"][cid]["stop"]["command_id"],
            boot_version=replacement.boot,
            note="checked stopped robot; inherited work needs removal",
        )["ok"]
        stop = env.control.data["commands"][cid]["stop"]
        assert stop["payload"]["expected_boot_version"] == replacement.boot
        if restart_during_stop:
            rejected = set()
            received = set()
            handle = runtime.handle

            def record_rejection(delivery):
                try:
                    handle(delivery)
                except ValueError:
                    rejected.add(wire.decode(delivery.payload)["message_id"])
                    raise
                finally:
                    received.add(wire.decode(delivery.payload)["message_id"])

            monkeypatch.setattr(runtime, "handle", record_rejection)
            # Crash after durable STOP acceptance, before it can produce completion.
            monkeypatch.setattr(replacement, "_finish_stop", lambda: None)
            held_until(lambda: replacement.journal.data["active"] == stop["command_id"])
            replacement.close()
            replacement = FieldPinkyClient(
                MqttConfig(
                    "127.0.0.1", "fleet-stop-restarted-" + old.robot, port=env.port
                ),
                profile,
                role,
                env.tmp_path / (old.robot + ".json"),
                old.driver,
            )
            replacement.start()
            env.stack.callback(replacement.close)
            env.clients[index] = replacement
            assert replacement.boot != stop["payload"]["expected_boot_version"]
        held_until(
            lambda: (
                replacement.journal.data["active"] is None
                and (
                    restart_during_stop
                    or any(
                        e["command_id"] == stop["command_id"]
                        and e["payload"]["status"] == "completed"
                        for e in env.control.data["events"].values()
                    )
                )
            )
        )
        interrupted = replacement.journal.data["records"][cid]["event"]
        assert interrupted["payload"]["status"] == "interrupted"
        assert (
            replacement.journal.data["records"][cid]["command"] == old_record["command"]
        )
        if restart_during_stop:
            completed = replacement.journal.data["records"][stop["command_id"]]["event"]
            unconfirmed = {e["message_id"]: e for e in (interrupted, completed)}
            held_until(lambda: unconfirmed.keys() <= rejected)
            assert unconfirmed.keys() <= replacement.journal.data["outbox"].keys()
            assert not unconfirmed.keys() & env.control.data["events"].keys()
        else:
            held_until(
                lambda: (
                    interrupted["message_id"] not in replacement.journal.data["outbox"]
                )
            )
            assert env.control.data["events"][interrupted["message_id"]] == interrupted
        assert env.control.data["resources"] == resources
        assert not operate(
            "reconcile", command_id=cid, outcome="completed", note="not completion"
        )["ok"]
        assert not operate(
            "reconcile", command_id=cid, outcome="removed", note="endpoint still held"
        )["ok"]

        # Simulate the operator removing goods, parking, and restarting the endpoint/driver.
        # Reopen the same journal; do not clear the inherited command history.
        replacement.close()
        driver = VirtualPinky(profile, role, threaded=False)
        env.stack.callback(driver.close)
        parked = FieldPinkyClient(
            MqttConfig("127.0.0.1", "fleet-parked-" + old.robot, port=env.port),
            profile,
            role,
            env.tmp_path / (old.robot + ".json"),
            driver,
        )
        parked.start()
        env.stack.callback(parked.close)
        env.clients[index] = parked
        held_until(
            lambda: env.control.observed[old.robot]["boot_version"] == parked.boot
        )
        assert parked.halted is None and parked.journal.data["active"] is None
        assert parked.journal.data["records"][cid]["event"] == interrupted
        if restart_during_stop:
            assert not operate(
                "reconcile",
                command_id=cid,
                outcome="removed",
                note="stop not confirmed",
            )["ok"]
            assert operate(
                "recover_stop",
                command_id=cid,
                stop_id=stop["command_id"],
                boot_version=parked.boot,
                note="checked current idle robot and removed goods",
            )["ok"]
        assert operate(
            "reconcile",
            command_id=cid,
            outcome="removed",
            note="goods removed; parked and checked",
        )["ok"]
        assert env.control.data["tasks"][key]["status"] == "CANCELED"
        assert not env.control.data["commands"] and not env.control.data["resources"]
        assert operate("release_hold", note="checked fleet")["ok"]
        assert env.control.next_action() is None
        assert not env.control.data["enabled"]
        if restart_during_stop:
            received.clear()
            for event in unconfirmed.values():
                parked._send(event)
            held_until(lambda: unconfirmed.keys() <= received)
            assert unconfirmed.keys() <= parked.journal.data["outbox"].keys()
            assert not unconfirmed.keys() & env.control.data["events"].keys()
            assert env.control.data["fault"] is None
            assert env.control.data["tasks"][key]["status"] == "CANCELED"
        assert operate("enable")["ok"]
        assert not env.executors["omx3"].tasks
