"""Synthetic registration and both follow modes through the production MQTT endpoint."""

import io
import time
import urllib.error
import urllib.request
from datetime import UTC, datetime, timedelta
from uuid import uuid4

import pytest
from PIL import Image
from test_field_pinky import RecordingTransport, deliver
from test_virtual_pinky import simulation_profile
from virtual_follow_frames import frame_bytes, start_preview
from virtual_pinky import VirtualPinky

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.field_pinky import FieldPinkyClient
from roport_control_server.adapters.mqtt.mqtt import MqttConfig


@pytest.fixture
def virtual_follow(tmp_path):
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", threaded=False)
    driver.mode = "nav2"
    client = FieldPinkyClient(
        MqttConfig("127.0.0.1", "virtual-follow"),
        profile,
        "logistics",
        tmp_path / "pinky.json",
        driver,
    )
    client.transport = RecordingTransport()
    try:
        yield client, driver
    finally:
        client.close()
        driver.close()


def send(client, kind="follow_registration_command", **payload):
    message = wire.build_message(
        kind,
        client.control,
        client.robot,
        {
            "expected_boot_version": client.boot,
            "map_version": client.profile["map_version"],
            "follow_session_id": "customer-session-1",
            **payload,
        },
        command_id=uuid4().hex,
        expires_at=(datetime.now(UTC) + timedelta(seconds=20)).isoformat(),
    )
    deliver(client, message)
    return message


def status(client, command):
    return client.journal.data["records"][command["command_id"]]["event"]["payload"]


def tick(client, driver):
    driver.heartbeat()
    driver.tick()
    for command, phase, result in driver.drain_results():
        client.report(command, phase, **result)
    client.poll(0)


def select_target(client, driver, *, box=False):
    capture = send(client, operation="capture")
    tick(client, driver)
    result = status(client, capture)["result"]
    selection = (
        {"box": [0.1, 0.2, 0.3, 0.8]}
        if box
        else {"candidate_id": result["candidates"][0]["candidate_id"]}
    )
    selected = send(
        client, operation="select", frame_ref=result["frame_ref"], **selection
    )
    tick(client, driver)
    assert status(client, selected)["status"] == "completed"
    return result, status(client, selected)["result"]


def start_target(client, target, **changed):
    return send(
        client,
        "follow_target_command",
        follow_mode="appearance",
        **{
            "target_ref": target["target_ref"],
            "perception_boot_id": target["perception_boot_id"],
            **changed,
        },
    )


def test_capture_is_async_non_motion_and_retransmit_retains_same_frame(virtual_follow):
    client, driver = virtual_follow
    pose = driver.observe_motion()["pose"]
    command = send(client, operation="capture")
    assert status(client, command)["status"] == "accepted"
    assert driver.registration is None
    assert driver.observe()["stopped"] is True
    assert driver.drain_results() == []
    tick(client, driver)
    result = status(client, command)["result"]
    wire.validate_follow_registration_result(command["payload"], result)
    assert driver.observe_motion()["pose"] == pose
    assert driver.follow is None
    deliver(client, command)
    assert status(client, command)["result"]["frame_ref"] == result["frame_ref"]
    assert driver.registration_pending is None


@pytest.mark.parametrize("box", [False, True])
def test_selected_appearance_starts_only_on_tick_and_release_invalidates_target(
    virtual_follow, box
):
    client, driver = virtual_follow
    pose = driver.observe_motion()["pose"]
    capture, target = select_target(client, driver, box=box)
    assert driver.observe_motion()["pose"] == pose
    command = start_target(client, target)
    assert status(client, command)["status"] == "accepted"
    assert driver.observe()["follow"]["active"] is False
    client.poll(0)
    assert status(client, command)["status"] == "accepted"
    tick(client, driver)
    assert status(client, command)["result"]["follow_active"] is True
    assert driver.observe()["follow"]["target_ref"] == target["target_ref"]
    stop = wire.build_message(
        "robot_stop_command",
        client.control,
        client.robot,
        {
            "expected_boot_version": client.boot,
            "target_command_id": command["command_id"],
            "reason": "follow_pause",
        },
        command_id=uuid4().hex,
        expires_at=(datetime.now(UTC) + timedelta(seconds=20)).isoformat(),
    )
    deliver(client, stop)
    tick(client, driver)
    assert status(client, stop)["status"] == "completed"
    assert driver.observe()["stopped"] is True
    release = send(client, operation="release")
    tick(client, driver)
    assert status(client, release)["result"]["cleared"] is True
    assert frame_bytes(driver, capture["image_ref"]) is None
    rejected = start_target(client, target)
    assert status(client, rejected)["status"] == "rejected"
    assert driver.follow is None


@pytest.mark.parametrize(
    "changed",
    [
        {"follow_session_id": "other-customer"},
        {"target_ref": "other-person"},
        {"perception_boot_id": "other-process"},
    ],
)
def test_target_bound_to_session_target_and_perception(virtual_follow, changed):
    client, driver = virtual_follow
    _, target = select_target(client, driver)
    command = start_target(client, target, **changed)
    assert status(client, command)["status"] == "rejected"
    assert driver.follow is None


@pytest.mark.parametrize(
    "mutation", ["expired", "endpoint_restart", "perception_restart", "binding", "map"]
)
def test_target_cannot_survive_expiry_or_binding_changes(virtual_follow, mutation):
    client, driver = virtual_follow
    _, target = select_target(client, driver)
    if mutation == "expired":
        driver.registration["expires_mono"] = time.monotonic() - 1
    elif mutation == "perception_restart":
        driver.perception_boot_id = "new-perception-process"
    else:
        key = {
            "endpoint_restart": "expected_boot_id",
            "binding": "binding_id",
            "map": "map_version",
        }[mutation]
        driver.registration[key] = "previous-value"
    command = start_target(client, target)
    assert status(client, command)["status"] == "rejected"
    assert driver.follow is None


def test_recapture_invalidates_previous_frame_and_target(virtual_follow):
    client, driver = virtual_follow
    capture, target = select_target(client, driver)
    send(client, operation="capture")
    tick(client, driver)
    assert frame_bytes(driver, capture["image_ref"]) is None
    old_frame = send(
        client,
        operation="select",
        frame_ref=capture["frame_ref"],
        candidate_id="virtual-person-1",
    )
    assert status(client, old_frame)["reason"] == "frame_mismatch"
    old_target = start_target(client, target)
    assert status(client, old_target)["status"] == "rejected"


def test_appearance_expiry_during_follow_stops_and_fails(virtual_follow):
    client, driver = virtual_follow
    _, target = select_target(client, driver)
    command = start_target(client, target)
    tick(client, driver)
    driver.registration["expires_mono"] = time.monotonic() - 1
    tick(client, driver)
    assert status(client, command)["status"] == "failed"
    assert status(client, command)["reason"] == "follow_registration_expired"
    assert driver.observe()["stopped"] is True


def test_virtual_marker_still_works_without_registration(virtual_follow):
    client, driver = virtual_follow
    command = send(client, "follow_target_command", follow_mode="marker", card_id=35)
    tick(client, driver)
    assert status(client, command)["result"]["follow_active"] is True
    assert driver.observe()["follow"]["follow_mode"] == "marker"
    assert driver.registration is None


def test_preview_is_loopback_authenticated_and_invalid_after_release(virtual_follow):
    client, driver = virtual_follow
    capture, _ = select_target(client, driver)
    token = "virtual-test-token-" + uuid4().hex
    server = start_preview(driver, 0, token)
    assert server.server_address[0] == "127.0.0.1"
    url = f"http://127.0.0.1:{server.server_port}/frames/{capture['image_ref']}"
    try:
        with pytest.raises(urllib.error.HTTPError) as denied:
            urllib.request.urlopen(url, timeout=2)
        assert denied.value.code == 401
        request = urllib.request.Request(
            url, headers={"Authorization": f"Bearer {token}"}
        )
        with urllib.request.urlopen(request, timeout=2) as response:
            assert response.headers["Cache-Control"] == "no-store"
            data = response.read()
        assert Image.open(io.BytesIO(data)).size == (640, 480)
        release = send(client, operation="release")
        tick(client, driver)
        assert status(client, release)["status"] == "completed"
        with pytest.raises(urllib.error.HTTPError) as expired:
            urllib.request.urlopen(request, timeout=2)
        assert expired.value.code == 404
    finally:
        server.shutdown()
        server.server_close()
