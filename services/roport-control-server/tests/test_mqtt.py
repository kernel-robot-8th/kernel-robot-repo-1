import os
import select
import shutil
import socket
import ssl
import subprocess
import sys
import time
from dataclasses import replace
from uuid import uuid4

import pytest

from roport_control_server.communication.messages import decode, encode, topic
from roport_control_server.communication.mqtt import MqttConfig, MqttTransport
from roport_control_server.examples.mqtt_demo import DEVICES, MockExecutor, make_command


def wait_until(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("condition timed out")


@pytest.fixture
def broker(tmp_path, request):
    binary = os.environ.get("MOSQUITTO_BIN") or shutil.which("mosquitto")
    if not binary:
        if os.environ.get("REQUIRE_MQTT_TESTS") == "1":
            pytest.fail("Mosquitto is required for MQTT integration tests")
        pytest.skip("install Mosquitto or set MOSQUITTO_BIN for integration tests")
    port = 0
    config = tmp_path / "mosquitto.conf"
    config.write_text("allow_anonymous true\npersistence false\n")
    if getattr(request, "param", False):
        acl = tmp_path / "acl"
        acl.write_text("topic read #\ntopic write roport/v1/test/robot/pinky1/event\n")
        config.write_text(config.read_text() + f"acl_file {acl}\n")
    processes = []

    def start():
        process = subprocess.Popen(
            [binary, "-c", str(config)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        processes.append(process)

        def listening():
            if process.poll() is not None:
                raise RuntimeError(process.stderr.read())
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                    return True
            except OSError:
                return False

        wait_until(listening)
        return process

    try:
        settings = config.read_text()
        for attempt in range(3):
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                port = reservation.getsockname()[1]
            config.write_text(f"listener {port} 127.0.0.1\n" + settings)
            try:
                process = start()
                break
            except RuntimeError as exc:
                if "Address already in use" not in str(exc) or attempt == 2:
                    raise
        yield port, process, start
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
            process.stderr.close()


@pytest.fixture
def clients(broker):
    created = []

    def connect(subscriptions, **kwargs):
        client = MqttTransport(
            MqttConfig("127.0.0.1", str(uuid4()), port=broker[0], **kwargs),
            subscriptions,
        )
        created.append(client)
        client.start()
        return client

    yield connect
    for client in created:
        client.stop()


@pytest.mark.parametrize("device", DEVICES)
def test_broker_roundtrip_and_duplicate_command(clients, device):
    cmd_topic = topic("test", DEVICES[device], device, "cmd")
    event_topic = topic("test", DEVICES[device], device, "event")
    control = clients([event_topic])
    robot = clients([cmd_topic])
    executor = MockExecutor(device)
    command = make_command(device)
    received_ids = []
    for _ in range(2):
        control.publish(cmd_topic, command)
        delivery = robot.receive()
        assert delivery.qos == 1 and not delivery.retained
        for result in executor.handle(decode(delivery.payload)):
            robot.publish(event_topic, result)
        robot.ack(delivery)
        events = []
        for _ in range(2):
            event_delivery = control.receive()
            events.append(decode(event_delivery.payload))
            control.ack(event_delivery)
        assert [event["phase"] for event in events] == ["accepted", "completed"]
        assert all(event["execution_id"] == command["execution_id"] for event in events)
        received_ids.append([event["message_id"] for event in events])
    assert received_ids[0] == received_ids[1]
    assert executor.execution_count == 1


def test_retained_command_is_visible_even_to_existing_subscription(clients):
    destination = topic("test", "robot", "pinky1", "cmd")
    sender = clients([topic("test", "robot", "pinky1", "event")])
    receiver = clients([destination])
    # Deliberately bypass the safe publisher to simulate a misconfigured sender.
    info = sender._client.publish(
        destination, encode(make_command("pinky1")), qos=1, retain=True
    )
    info.wait_for_publish(2)
    for subscriber in [receiver, clients([destination])]:
        delivery = subscriber.receive()
        executor = MockExecutor("pinky1")
        events = executor.handle(decode(delivery.payload), retained=delivery.retained)
        assert events[0]["payload"]["error_code"] == "retained_command"
        assert executor.execution_count == 0
        subscriber.ack(delivery)


def test_reconnect_resubscribes_and_old_ack_is_rejected(broker, clients):
    destination = topic("test", "robot", "pinky1", "cmd")
    sender = clients([topic("test", "robot", "pinky1", "event")])
    receiver = clients([destination])
    command = make_command("pinky1")
    sender.publish(destination, command)
    old = receiver.receive()
    executor = MockExecutor("pinky1")
    original_events = executor.handle(decode(old.payload))
    broker[1].terminate()
    broker[1].wait(timeout=5)
    wait_until(lambda: not receiver.ready and not sender.ready)
    with pytest.raises(ConnectionError):
        sender.publish(destination, command)
    broker[2]()
    wait_until(lambda: sender.ready and receiver.ready)
    with pytest.raises(ConnectionError, match="stale delivery"):
        receiver.ack(old)
    sender.publish(destination, command)
    delivery = receiver.receive()
    assert decode(delivery.payload)["command_id"] == command["command_id"]
    assert executor.handle(decode(delivery.payload)) == original_events
    assert executor.execution_count == 1
    receiver.ack(delivery)


def test_queue_overflow_stops_transport_and_reports_fault(clients):
    destination = topic("test", "robot", "pinky1", "cmd")
    sender = clients([topic("test", "robot", "pinky1", "event")])
    receiver = clients([destination], queue_size=1)
    # QoS0 bypasses ReceiveMaximum so the bounded application queue fills.
    for _ in range(2):
        sender._client.publish(
            destination, encode(make_command("pinky1")), qos=0
        ).wait_for_publish(2)
    wait_until(lambda: receiver.fault is not None)
    assert not receiver.ready
    with pytest.raises(ConnectionError, match="queue full"):
        receiver.receive()


def test_cloud_config_keeps_certificate_checks_and_hides_password():
    config = MqttConfig(
        "broker.example.com",
        "test-client",
        port=8883,
        tls=True,
        username="test",
        password="not-a-real-secret",
    )
    transport = MqttTransport(config, [topic("test", "robot", "pinky1", "event")])
    context = transport._client._ssl_context
    assert context.check_hostname and context.verify_mode == ssl.CERT_REQUIRED
    assert context.minimum_version >= ssl.TLSVersion.TLSv1_2
    assert config.password not in repr(config)
    with pytest.raises(ValueError, match="requires username and TLS"):
        replace(config, tls=False)


@pytest.mark.parametrize("broker", [True], indirect=True)
def test_broker_publish_rejection_is_not_reported_as_success(clients):
    sender = clients([topic("test", "robot", "pinky1", "event")])
    with pytest.raises(ConnectionError, match="broker rejected publish"):
        sender.publish(topic("test", "robot", "pinky1", "cmd"), make_command("pinky1"))
    assert sender.ready and sender.fault is None
    event = MockExecutor("pinky1").handle(make_command("pinky1"))[0]
    sender.publish(topic("test", "robot", "pinky1", "event"), event)
    delivery = sender.receive()
    assert decode(delivery.payload) == event
    sender.ack(delivery)


def test_publish_rejects_wrong_site_or_device(clients):
    sender = clients([topic("test", "robot", "pinky1", "event")])
    for destination in [
        topic("other", "robot", "pinky1", "cmd"),
        topic("test", "robot", "pinky2", "cmd"),
    ]:
        with pytest.raises(ValueError, match="mismatch"):
            sender.publish(destination, make_command("pinky1"))


def test_communication_folder_is_portable(tmp_path):
    from pathlib import Path

    from roport_control_server import communication

    shutil.copytree(Path(communication.__file__).parent, tmp_path / "communication")
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    subprocess.run(
        [
            sys.executable,
            "-c",
            """
import sys
from communication.mqtt import MqttConfig, MqttTransport
from communication.messages import topic
MqttTransport(MqttConfig('localhost', 'portable-test'), [topic('test', 'robot', 'pinky1', 'cmd')])
assert not any(name in sys.modules for name in ['fastapi', 'rclpy', 'PyQt5', 'PyQt6', 'psycopg'])
assert 'roport_control_server' not in sys.modules
""",
        ],
        cwd=tmp_path,
        env=environment,
        check=True,
        timeout=5,
    )


@pytest.mark.parametrize("device", DEVICES)
def test_demo_runs_in_separate_processes(broker, device):
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    command = [sys.executable, "-m", "roport_control_server.examples.mqtt_demo"]
    with subprocess.Popen(
        [*command, device, "--port", str(broker[0])],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=environment,
    ) as receiver:
        try:
            assert select.select([receiver.stdout], [], [], 5)[0], (
                "device startup timed out"
            )
            assert "MQTT subscribed" in receiver.stdout.readline()
            result = subprocess.run(
                [*command, "control", "--device", device, "--port", str(broker[0])],
                env=environment,
                capture_output=True,
                text=True,
                timeout=10,
                check=True,
            )
            assert '"phase": "accepted"' in result.stdout
            assert '"phase": "completed"' in result.stdout
        finally:
            receiver.terminate()
            receiver.wait(timeout=5)


def test_oversized_message_is_dropped_without_poisoning_connection(clients):
    from roport_control_server.communication.messages import MAX_MESSAGE_BYTES

    destination = topic("test", "robot", "pinky1", "cmd")
    sender = clients([topic("test", "robot", "pinky1", "event")])
    receiver = clients([destination])
    sender._client.publish(
        destination, b"x" * (MAX_MESSAGE_BYTES + 1), qos=1
    ).wait_for_publish(2)
    wait_until(lambda: receiver.rejected_messages == 1)
    assert receiver.ready and receiver.fault is None
    command = make_command("pinky1")
    sender.publish(destination, command)
    delivery = receiver.receive()
    assert decode(delivery.payload) == command
    receiver.ack(delivery)
