"""Prepare a local, credential-free field handoff. Never connects to devices."""

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import yaml
from build_pinky_mqtt_kit import MQTT_FILES
from run_virtual_fleet import profiles
from run_virtual_trial import LAYOUT, MAP, SERVICE

ROOT = SERVICE.parents[1]


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def prepare_field(output, *, map_path=MAP, layout_path=LAYOUT):
    output = Path(output).resolve()
    map_path, layout_path = Path(map_path).resolve(), Path(layout_path).resolve()
    if map_path.name == "layout.yaml":
        raise ValueError("map YAML must not be named layout.yaml in the handoff")
    fleet, robots = profiles(
        map_path=map_path, layout_path=layout_path, simulation=False
    )
    # Keep the exact YAML bytes: map_version hashes YAML + image together.
    image = Path(yaml.safe_load(map_path.read_text())["image"])
    if image.is_absolute() or ".." in image.parts:
        raise ValueError("map image must be a relative path inside the map folder")
    source_image = map_path.parent / image
    if not source_image.is_file():
        raise ValueError("map image missing")
    output.mkdir(parents=True, exist_ok=False)
    fleet["control_id"] = "control-01"
    write_json(output / "fleet.json", fleet)
    for robot, (role, profile) in robots.items():
        profile["control_id"] = fleet["control_id"]
        write_json(output / robot / "trial.json", profile)
    for name, source in (
        ("maps/" + map_path.name, map_path),
        ("maps/" + str(image), source_image),
        ("maps/layout.yaml", layout_path),
    ):
        dest = output / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, dest)
    for name in MQTT_FILES:
        dest = output / "pinky_mqtt" / name
        dest.parent.mkdir(exist_ok=True)
        shutil.copyfile(
            SERVICE / "src/roport_control_server/adapters/mqtt" / name, dest
        )
    (output / "requirements.txt").write_text("paho-mqtt==2.1.0\n")
    # Copy only the deployable team packages; no venv/build/logs or vendored LeRobot.
    paths = (
        subprocess.check_output(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
            cwd=ROOT,
        )
        .decode()
        .split("\0")
    )
    for name in sorted(set(paths)):
        mappings = (
            ("services/roport-pinky/src/roport_pinky/", "pinky-src/roport_pinky/"),
            (
                "services/roport-pinky/src/roport_line_nav/",
                "pinky-src/roport_line_nav/",
            ),
            ("services/roport-omx/src/", "omx/src/"),
            ("docs/field/", "manuals/"),
        )
        for prefix, dest_prefix in mappings:
            if name.startswith(prefix):
                source = ROOT / name
                if source.is_file() and not source.is_symlink():
                    dest = output / (dest_prefix + name[len(prefix) :])
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source, dest)
    for name in (
        "pyproject.toml",
        "uv.lock",
        "README.md",
        "scripts/field_host.py",
        "scripts/field_mock.py",
        "scripts/check_hardware.py",
    ):
        dest = output / "omx" / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / "services/roport-omx" / name, dest)
    (output / "broker.env.example").write_text(
        "# Copy to broker.env and enter the broker PC address (not the robot IP).\n"
        "export ROPORT_MQTT_HOST=REPLACE_WITH_BROKER_LAN_IP\n"
        "export ROPORT_MQTT_PORT=18883\nexport ROPORT_MQTT_TLS=0\n"
        "# Credentials remain local; password authentication requires TLS.\n"
    )
    control_sources = sorted(
        {
            name
            for name in paths
            if name.startswith(
                (
                    "services/roport-control-server/src/",
                    "services/roport-control-server/tools/",
                )
            )
            or name
            in {
                "services/roport-control-server/pyproject.toml",
                "services/roport-control-server/uv.lock",
            }
        }
    )
    (output / "CONTROL_SHA256SUMS").write_text(
        "".join(
            f"{hashlib.sha256((ROOT / name).read_bytes()).hexdigest()}  {name}\n"
            for name in control_sources
            if (ROOT / name).is_file() and not (ROOT / name).is_symlink()
        )
    )
    hashes = {
        str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(output.rglob("*"))
        if p.is_file()
    }
    write_json(
        output / "manifest.json",
        {
            "source_head": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=ROOT
            )
            .decode()
            .strip(),
            "source_dirty": bool(
                subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT)
            ),
            "source_status": "local snapshot; not a published release",
            "profile": "roport-control-v2",
            "schema_version": "2.0",
            "map_version": fleet["map_version"],
            "graph_version": fleet["graph_version"],
            "files": [
                {"path": name, "sha256": digest} for name, digest in hashes.items()
            ],
        },
    )
    (output / "SHA256SUMS").write_text(
        "".join(f"{h}  {p}\n" for p, h in hashes.items())
    )
    return fleet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--map", type=Path, default=MAP)
    parser.add_argument("--layout", type=Path, default=LAYOUT)
    args = parser.parse_args()
    prepare_field(args.output, map_path=args.map, layout_path=args.layout)
    print(
        f"Prepared {args.output.resolve()}; field_verified=false, OMX versions unset."
    )
    print(
        "Copy once, verify SHA256SUMS, then configure working copies on site. No device started."
    )


if __name__ == "__main__":
    main()
