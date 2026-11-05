"""Build a credential-free, standalone MQTT handoff zip from current local sources."""

import argparse
import hashlib
import json
from copy import deepcopy
from datetime import UTC, datetime
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

from roport_control_server.adapters.mqtt.trial_wire import (
    ID,
    PROFILE,
    SCHEMAS,
    check_result,
    command,
    enum,
    obj,
    reply,
    validate_trial,
)

SERVICE = Path(__file__).resolve().parents[1]


def wire_schema():
    variants = []
    for kind, payload in SCHEMAS.items():
        properties = {
            key: ID
            for key in (
                "message_id",
                "site_id",
                "source",
                "target",
                "task_id",
                "step_id",
                "command_id",
                "execution_id",
            )
        }
        properties.update(
            schema_version=enum("1.0"),
            profile=enum(PROFILE),
            type=enum(kind),
            sent_at={"type": "string", "format": "date-time"},
            payload=payload,
        )
        if kind.endswith(".requested"):
            properties["expires_at"] = {"type": "string", "format": "date-time"}
        variants.append(obj(**properties))
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "title": PROFILE,
        "oneOf": variants,
    }


def examples():
    common = {"binding_id": "example-binding", "expected_boot_id": "example-boot"}
    payloads = {
        "status": {"binding_id": "example-binding"},
        "route": {
            **common,
            "graph_version": "example-graph-v1",
            "start_node": "TEST-A",
            "start_heading_rad": 0.0,
            "start_maneuver": "none",
            "route": ["TEST-A", "TEST-B"],
            "goal": {"node_id": "TEST-B", "location_id": None, "on_arrival": "hold"},
            "speed_limit_mps": 0.1,
        },
        "mode_switch": {
            **common,
            "node_id": "N12",
            "from_mode": "line",
            "to_mode": "nav2",
            "map_id": "example-map",
            "map_version": "example-map-v1",
        },
        "navigate": {
            **common,
            "goal_id": "TEST_NAV_GOAL",
            "goal_version": "example-goal-v1",
            "map_id": "example-map",
            "map_version": "example-map-v1",
            "frame_id": "map",
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "position_tolerance_m": 0.1,
            "yaw_tolerance_rad": 0.15,
        },
    }
    commands = {
        kind: command(
            site="example-trial",
            control="control-example",
            robot="pinky1",
            task="example-task",
            step=kind,
            kind=kind,
            payload=payload,
        )
        for kind, payload in payloads.items()
    }
    state = {
        "binding_id": "example-binding",
        "boot_id": "example-boot",
        "phase_revision": 1,
        "observed_at": datetime.now(UTC).isoformat(),
        "node_id": "TEST-A",
        "mode": "line",
        "stopped": True,
        "line_enabled": False,
        "nav2_ready": False,
        "localization_ready": False,
        "drive_owner": "none",
        "graph_version": "example-graph-v1",
        "map_id": "example-map",
        "map_version": "example-map-v1",
        "heading_rad": 0.0,
        "fault": None,
    }
    messages = list(commands.values())
    route_completed = None
    for kind in SCHEMAS:
        if kind.endswith(".requested") or kind == "event.ack":
            continue
        operation, phase = kind.split(".")
        payload = deepcopy(state)
        payload["phase_revision"] = {
            "accepted": 1,
            "progress": 2,
            "completed": 3,
            "failed": 3,
            "rejected": 1,
        }[phase]
        if phase in {"failed", "rejected"}:
            payload["error_code"] = "example_error"
        if operation == "route" and phase in {"progress", "completed"}:
            complete = phase == "completed"
            payload.update(
                node_id="TEST-B" if complete else "TEST-A",
                seq=1 if complete else 0,
                total=2,
                prev_node="TEST-A" if complete else None,
                next_node=None if complete else "TEST-B",
            )
        if operation in {"mode_switch", "navigate"} and phase == "completed":
            payload.update(
                node_id="N12",
                mode="nav2",
                nav2_ready=True,
                localization_ready=True,
                drive_owner="nav2",
            )
        if kind == "navigate.completed":
            p = payloads["navigate"]
            payload.update(
                goal_id=p["goal_id"],
                goal_version=p["goal_version"],
                frame_id="map",
                pose=p["pose"],
                nav2_result="SUCCEEDED",
            )
        event = reply(commands[operation], kind, payload)
        check_result(commands[operation], event)
        messages.append(event)
        if kind == "route.completed":
            route_completed = event
    messages.append(
        reply(
            route_completed,
            "event.ack",
            {"acked_message_id": route_completed["message_id"], "status": "stored"},
        )
    )
    for message in messages:
        validate_trial(message)
    return messages


def build(output, *, scenarios_dir=None):
    files = {
        f"pinky_mqtt/{name}": (
            SERVICE / "src/roport_control_server/adapters/mqtt" / name
        ).read_bytes()
        for name in (
            "__init__.py",
            "messages.py",
            "mqtt.py",
            "journal.py",
            "pinky_trial.py",
            "trial_wire.py",
            "trial_runner.py",
        )
    }
    files["README.md"] = (SERVICE / "docs/pinky-trial.md").read_bytes()
    files["requirements.txt"] = b"paho-mqtt==2.1.0\n"
    files["message.schema.json"] = json.dumps(
        wire_schema(), ensure_ascii=False, indent=2
    ).encode()
    files["examples.json"] = json.dumps(
        examples(), ensure_ascii=False, indent=2
    ).encode()
    if scenarios_dir is not None:
        for scenario in ("single", "two"):
            for mode in ("simulation", "hardware"):
                folder = Path(scenarios_dir) / scenario / mode
                for name in ("trial.json", "broker.env.example", "CONNECT.md"):
                    files[f"scenarios/{scenario}/{mode}/{name}"] = (
                        folder / name
                    ).read_bytes()
    manifest = {
        "profile": PROFILE,
        "kit_revision": "2026-11-05-reviewable-handoff",
        "source_status": "local implementation; not a merged release",
        "files": {
            name: hashlib.sha256(raw).hexdigest() for name, raw in sorted(files.items())
        },
    }
    files["manifest.json"] = json.dumps(manifest, indent=2).encode()
    with ZipFile(output, "x", compression=ZIP_DEFLATED) as archive:
        for name, raw in sorted(files.items()):
            archive.writestr(name, raw)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--scenarios-dir")
    args = parser.parse_args()
    build(args.output, scenarios_dir=args.scenarios_dir)
    print(args.output)


if __name__ == "__main__":
    main()
