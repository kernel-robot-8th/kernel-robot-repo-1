"""Local Mosquitto fixture shared by task and state integration tests."""

import os
import shutil
import socket
import subprocess
import time
from contextlib import contextmanager

import pytest


def wait_until(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("condition timed out")


@contextmanager
def local_broker(tmp_path, *, restricted=False):
    binary = os.environ.get("MOSQUITTO_BIN") or shutil.which("mosquitto")
    if not binary:
        if os.environ.get("REQUIRE_MQTT_TESTS") == "1":
            pytest.fail("Mosquitto is required for MQTT integration tests")
        pytest.skip("install Mosquitto or set MOSQUITTO_BIN for integration tests")
    port = 0
    config = tmp_path / "mosquitto.conf"
    config.write_text("allow_anonymous true\npersistence false\n")
    if restricted:
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
def broker(tmp_path, request):
    with local_broker(
        tmp_path, restricted=getattr(request, "param", False)
    ) as instance:
        yield instance
