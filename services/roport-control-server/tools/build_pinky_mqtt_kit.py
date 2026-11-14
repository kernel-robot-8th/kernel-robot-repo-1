"""Build a credential-free, standalone MQTT handoff zip from current local sources."""

import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

from roport_control_server.adapters.mqtt.field_wire import (
    PROFILE,
    json_schema,
    validate,
)

SERVICE = Path(__file__).resolve().parents[1]


def wire_schema():
    return json_schema()


def examples():
    rows = json.loads((SERVICE / "interfaces/mqtt/examples.json").read_text())
    messages = [row["message"] for row in rows]
    for message in messages:
        validate(message)
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
            "field_pinky.py",
            "field_wire.py",
            "trial_wire.py",
            "trial_runner.py",
            "motion.py",
        )
    }
    files["README.md"] = (SERVICE / "docs/mqtt-field.md").read_bytes()
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
        "kit_revision": "2026-11-13-field-v2",
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
