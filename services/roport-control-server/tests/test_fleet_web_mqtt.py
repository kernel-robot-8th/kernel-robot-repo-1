"""HTTP -> owner socket -> real MQTT endpoints; only robot I/O is simulated."""

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from tempfile import TemporaryDirectory

from fastapi.testclient import TestClient
from fleet_web_fixture import LAYOUT, MAP, TOKEN
from test_fleet_mqtt import fleet_mqtt as fleet_mqtt  # noqa: PLC0414

from roport_control_server.adapters.api.app import create_app
from roport_control_server.executor.fleet_socket import FleetSocket


def test_web_requests_run_inbound_outbound_and_receipt_over_mqtt(fleet_mqtt):
    env = fleet_mqtt
    data = dict(env.control.data, enabled=False)
    env.control.save(data)
    with TemporaryDirectory(prefix="rp-web-mqtt-") as directory:
        channel = FleetSocket(Path(directory) / "s", env.control)
        try:
            app = create_app(
                MAP, trial_layout=LAYOUT, fleet_socket=channel.path, trial_token=TOKEN
            )
            with (
                TestClient(
                    app, base_url="http://127.0.0.1", client=("127.0.0.1", 50000)
                ) as client,
                ThreadPoolExecutor(max_workers=1) as pool,
            ):

                def request(method, path, **kwargs):
                    future = pool.submit(client.request, method, path, **kwargs)

                    def ready():
                        channel.poll()
                        return future.done()

                    env.pump(ready, timeout=5)
                    response = future.result()
                    assert response.status_code == 200, response.text
                    return response.json()

                def command(action, **fields):
                    return request(
                        "POST",
                        "/api/fleet/commands",
                        headers={
                            "origin": "http://127.0.0.1",
                            "x-roport-trial-token": TOKEN,
                        },
                        json={
                            "action": action,
                            "session_id": channel.session_id,
                            **fields,
                        },
                    )

                bootstrap = request("GET", "/api/bootstrap")
                assert bootstrap["mode"] == "fleet"
                inbound = {
                    "request_id": "web-in",
                    "operation": "inbound",
                    "package_id": "web-box",
                    "goal": None,
                }
                command("submit", request=inbound)
                command("submit", request=inbound)
                assert len(env.control.data["tasks"]) == 1
                assert not env.control.data["commands"]
                command("enable")
                env.pump(
                    lambda: env.control.data["tasks"]["web-in"]["status"] == "COMPLETE"
                )
                snapshot = request("GET", "/api/snapshot?mode=fleet")["fleet"]
                assert any(
                    s["package_id"] == "web-box" and s["state"] == "occupied"
                    for s in snapshot["slots"].values()
                )
                command(
                    "submit",
                    request={
                        "request_id": "web-out",
                        "operation": "outbound",
                        "package_id": "web-box",
                        "goal": "PUBLIC_SPACE_TRIAL",
                    },
                )
                env.pump(
                    lambda: (
                        env.control.data["tasks"]["web-out"]["stage"] == "await_receipt"
                    )
                )
                snapshot = request("GET", "/api/snapshot?mode=fleet")["fleet"]
                assert snapshot["tasks"]["web-out"]["status"] == "AWAITING_RECEIPT"
                command("receipt", request_id="web-out")
                env.pump(
                    lambda: env.control.data["tasks"]["web-out"]["status"] == "COMPLETE"
                )
                snapshot = request("GET", "/api/snapshot?mode=fleet")["fleet"]
                assert not snapshot["commands"] and not snapshot["resources"]
                assert all(s["state"] == "empty" for s in snapshot["slots"].values())
                assert not snapshot["fault"]
        finally:
            channel.close()
