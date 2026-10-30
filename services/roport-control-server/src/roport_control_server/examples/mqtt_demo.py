"""Run a loopback-only control client or a simulated device in separate terminals."""

import argparse
import json
import sys
from datetime import UTC, datetime, timedelta
from queue import Empty
from time import monotonic
from uuid import uuid4

from ..communication.messages import (
    CommandHistory,
    check_command,
    check_expiry,
    decode,
    topic,
)
from ..communication.mqtt import MqttConfig, MqttTransport

SITE = "test"
CONTROL = "control-test"
DEVICES = {"pinky1": "robot", "omx3": "robot", "shelf-01": "workcell"}
COMMANDS = {"pinky1": "route", "omx3": "load", "shelf-01": "shelf_outbound"}


def make_command(device: str) -> dict:
    now = datetime.now(UTC)
    payload = {"package_id": "package-test", "resource_token": "simulation-only"}
    if device == "pinky1":
        payload = {
            "graph_version": "test-v1",
            "route": ["test-start", "test-goal"],
            "goal": {"node_id": "test-goal"},
        }
    elif device == "shelf-01":
        payload["robot_ids"] = ["omx1", "omx2"]
    return {
        "schema_version": "1.0",
        "message_id": str(uuid4()),
        "type": f"{COMMANDS[device]}.requested",
        "site_id": SITE,
        "source": CONTROL,
        "target": device,
        "sent_at": now.isoformat(),
        "expires_at": (now + timedelta(seconds=30)).isoformat(),
        "task_id": "task-test",
        "step_id": "step-test",
        "command_id": str(uuid4()),
        "execution_id": str(uuid4()),
        "payload": payload,
    }


class MockExecutor:
    """Test-only state machine; never use its simulated acceptance for hardware."""

    def __init__(self, device: str, *, returned_arms: set[str] | None = None):
        self.device = device
        self.kind = DEVICES[device]
        self.history = CommandHistory()
        self.boot_id = str(uuid4())
        self.execution_count = 0
        self.returned_arms = (
            {"omx1", "omx2"} if returned_arms is None else returned_arms
        )

    def _event(self, command: dict, phase: str, revision: int, payload: dict) -> dict:
        now = datetime.now(UTC).isoformat()
        return {
            **{
                k: command[k]
                for k in (
                    "schema_version",
                    "site_id",
                    "task_id",
                    "step_id",
                    "command_id",
                    "execution_id",
                )
            },
            "message_id": str(uuid4()),
            "type": f"{COMMANDS[self.device]}.{phase}",
            "source": self.device,
            "target": CONTROL,
            "sent_at": now,
            "phase": phase,
            "phase_revision": revision,
            "boot_id": self.boot_id,
            "observed_at": now,
            "payload": {"simulated": True, **payload},
        }

    def handle(self, command: dict, *, retained: bool = False) -> list[dict]:
        # Routing failures are not answered to an untrusted target/source.
        if (command["site_id"], command["target"], command["source"]) != (
            SITE,
            self.device,
            CONTROL,
        ):
            raise ValueError("routing_mismatch")
        try:
            check_command(
                command,
                site=SITE,
                target=self.device,
                source=CONTROL,
                retained=retained,
            )
            previous = self.history.lookup(command)
            if previous is not None:
                return previous
            check_expiry(command)
            if command["type"] != f"{COMMANDS[self.device]}.requested":
                raise ValueError("unsupported_mock_command")
            payload = command["payload"]
            if self.device == "pinky1" and payload != {
                "graph_version": "test-v1",
                "route": ["test-start", "test-goal"],
                "goal": {"node_id": "test-goal"},
            }:
                raise ValueError("unsupported_mock_route")
            if self.device != "pinky1" and (
                payload.get("resource_token") != "simulation-only"
                or payload.get("package_id") != "package-test"
            ):
                raise ValueError("invalid_mock_grant")
            if self.device == "shelf-01" and payload.get("robot_ids") != [
                "omx1",
                "omx2",
            ]:
                raise ValueError("joint_workcell_required")
        except ValueError as exc:
            return [self._event(command, "rejected", 1, {"error_code": str(exc)})]

        events = [self._event(command, "accepted", 1, {})]
        if self.device == "shelf-01":
            complete = self.returned_arms == {"omx1", "omx2"}
            events.append(
                self._event(
                    command,
                    "completed" if complete else "progress",
                    2,
                    {
                        "arms_returned": sorted(self.returned_arms),
                        "departure_ready": complete,
                    },
                )
            )
        else:
            events.append(self._event(command, "completed", 2, {"holding": True}))
        self.history.remember(command, events)
        self.execution_count += 1
        return events


def run_device(transport: MqttTransport, device: str) -> None:
    executor = MockExecutor(device)
    while True:
        try:
            delivery = transport.receive()
        except Empty:
            continue
        try:
            command = decode(delivery.payload)
            for event in executor.handle(command, retained=delivery.retained):
                transport.publish(topic(SITE, executor.kind, device, "event"), event)
        except ValueError as exc:
            print(f"Rejected malformed/unroutable test message: {exc}", flush=True)
        # Memory-handled only. No application /ack or DB durability is claimed.
        transport.ack(delivery)


def run_control(transport: MqttTransport, device: str, timeout: float = 5) -> None:
    command = make_command(device)
    transport.publish(topic(SITE, DEVICES[device], device, "cmd"), command)
    deadline = monotonic() + timeout
    last_revision = 0
    boot_id = None
    while True:
        remaining = deadline - monotonic()
        if remaining <= 0:
            raise TimeoutError("completion timeout; execution result unknown")
        try:
            delivery = transport.receive(timeout=remaining)
        except Empty as exc:
            raise TimeoutError("completion timeout; execution result unknown") from exc
        event = decode(delivery.payload)
        if any(event[key] != command[key] for key in ("site_id", "task_id", "step_id")):
            raise ValueError("event does not match current test task")
        if (
            event["source"],
            event["target"],
            event["command_id"],
            event["execution_id"],
        ) != (device, CONTROL, command["command_id"], command["execution_id"]):
            raise ValueError("event does not match current test execution")
        phase = event.get("phase")
        revision = event.get("phase_revision")
        if (
            delivery.retained
            or phase not in {"accepted", "progress", "completed", "failed", "rejected"}
            or type(revision) is not int
            or revision < 1
            or not isinstance(event.get("boot_id"), str)
            or not event["boot_id"]
        ):
            raise ValueError("invalid test event state")
        if boot_id is not None and boot_id != event["boot_id"]:
            raise ConnectionError("device restarted; reconcile execution")
        boot_id = event["boot_id"]
        transport.ack(delivery)
        if revision <= last_revision:
            continue
        last_revision = revision
        print(json.dumps(event, ensure_ascii=False), flush=True)
        if phase in {"failed", "rejected"}:
            raise RuntimeError(f"test command {phase}")
        if phase == "completed":
            return


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=["control", *DEVICES])
    parser.add_argument("--device", choices=DEVICES, default="pinky1")
    parser.add_argument("--port", type=int, default=1883)
    args = parser.parse_args()
    device = args.device if args.role == "control" else args.role
    channel = "event" if args.role == "control" else "cmd"
    transport = MqttTransport(
        MqttConfig("127.0.0.1", f"roport-test-{args.role}", port=args.port),
        [topic(SITE, DEVICES[device], device, channel)],
    )
    try:
        transport.start()
        print(f"MQTT subscribed: test/{args.role} (simulation only)", flush=True)
        if args.role == "control":
            run_control(transport, device)
        else:
            run_device(transport, device)
    except KeyboardInterrupt:
        pass
    except (ConnectionError, TimeoutError) as exc:
        print(
            f"Demo stopped: {exc}. Result may be unknown; reconcile before retry. "
            "Restarting the mock loses its command history.",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc
    finally:
        transport.stop()


if __name__ == "__main__":
    main()
