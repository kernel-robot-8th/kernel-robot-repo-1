"""Add shelf-01 MQTT settings to an existing Pinky profile without regenerating identities."""

import argparse
import json
import shlex
from copy import deepcopy
from pathlib import Path
from uuid import uuid4

from roport_control_server.executor.pinky_trial import validate_config


def configure(profile, *, mode, slot, model, calibration):
    if not isinstance(profile, dict) or not isinstance(profile.get("omx", {}), dict):
        raise TypeError("trial profile and OMX settings must be objects")
    result = deepcopy(profile)
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
    args = parser.parse_args()
    try:
        profile = configure(
            json.loads(args.config.read_text()),
            mode=args.execution_mode,
            slot=args.slot,
            model=args.model_version,
            calibration=args.calibration_version,
        )
        with args.output.open("x") as output:
            output.write(json.dumps(profile, ensure_ascii=False, indent=2) + "\n")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    print(f"Created {args.output}; original profile and Pinky identities preserved.")
    command = [
        "uv",
        "run",
        "python",
        "scripts/cooperative_host.py",
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
            "--box-marker",
            "REPLACE_WITH_BOX_MARKER",
        ]
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
