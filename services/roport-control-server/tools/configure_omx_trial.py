"""Add shelf-01 MQTT settings to an existing Pinky profile without regenerating identities."""

import argparse
import json
import shlex
from copy import deepcopy
from pathlib import Path
from uuid import uuid4

from roport_control_server.executor.pinky_trial import validate_config

# The OMX command's package_id is the box's top ArUco marker number as a string (Host and OMX3
# rule; 18 is the Pinky tray marker, docs/aruco/markers.yaml v1.6). Anything else is refused by
# the Host with INVALID_TARGET once the Pinky has docked (2026-11-21 review).
PACKAGE_MARKERS = frozenset({15, 16, 17, 19, 20, 21, 22})


def configure(profile, *, mode, slot, model, calibration, package_marker):
    if not isinstance(profile, dict) or not isinstance(profile.get("omx", {}), dict):
        raise TypeError("trial profile and OMX settings must be objects")
    if type(package_marker) is not int or package_marker not in PACKAGE_MARKERS:
        raise ValueError(
            f"package marker must be one of {sorted(PACKAGE_MARKERS)}: {package_marker!r}"
        )
    result = deepcopy(profile)
    result["package_id"] = str(package_marker)
    result["omx"] = {
        "binding_id": profile.get("omx", {}).get("binding_id", str(uuid4())),
        "slot_id": slot,
        "model_version": model,
        "calibration_version": calibration,
        "execution_mode": mode,
    }
    validate_config(result, require_field_verification=False)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execution-mode", choices=("mock", "hardware"), required=True)
    parser.add_argument(
        "--slot", choices=("1a", "1b", "1c", "2a", "2b", "2c"), required=True
    )
    parser.add_argument("--model-version", required=True)
    parser.add_argument("--calibration-version", required=True)
    parser.add_argument(
        "--package-marker",
        type=int,
        required=True,
        help="ArUco marker number on the real box (15-17, 19-22); becomes package_id",
    )
    args = parser.parse_args()
    try:
        profile = configure(
            json.loads(args.config.read_text()),
            mode=args.execution_mode,
            slot=args.slot,
            model=args.model_version,
            calibration=args.calibration_version,
            package_marker=args.package_marker,
        )
        with args.output.open("x") as output:
            output.write(json.dumps(profile, ensure_ascii=False, indent=2) + "\n")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    print(f"Created {args.output}; original profile and Pinky identities preserved.")
    print(
        f'package_id is now "{profile["package_id"]}" (box marker); check it on the real box. '
        f"Request it with: request {profile['package_id']}"
    )
    command = [
        "uv",
        "run",
        "python",
        "scripts/cooperative_host.py",
        "--protocol",
        "v1",  # OmxTrial (roport-omx-task-v1); the Host's default is v2 (#135)
        "--site",
        profile["site_id"],
        "--control",
        profile["control_id"],
        "--binding",
        profile["omx"]["binding_id"],
        "--client-id",
        "roport-shelf-01-host",
        "--journal",
        "REPLACE_WITH_PERSISTENT_OMX_JOURNAL_PATH",
    ]
    if args.execution_mode == "hardware":
        command += [
            "--executor",
            "panel",
            "--routes",
            "REPLACE_WITH_ROUTES_PATH",
        ]  # the box comes from each command's package_id (no --box-marker)
        print(
            "Hardware needs the panel Host branch and live arm readiness; see docs/pinky-trial.md."
        )
    else:
        print(
            "MOCK ONLY: timed completion does not prove physical transfer; keep OMX arms disconnected."
        )
    print("On OMX PC, source the same ROPORT_MQTT_* broker settings, then review:")
    print(shlex.join(command))
    print(
        "Read-only status check from a separate terminal (stop the control owner first):"
    )
    print(
        "The existing status sender supports unauthenticated, non-TLS test brokers only."
    )
    print(
        shlex.join(
            [
                "uv",
                "run",
                "python",
                "scripts/cooperative_control_sim.py",
                "status",
                "--site",
                profile["site_id"],
                "--control",
                profile["control_id"],
                "--binding",
                profile["omx"]["binding_id"],
                "--log",
                "REPLACE_WITH_STATUS_LOG_PATH",
            ]
        )
    )


if __name__ == "__main__":
    main()
