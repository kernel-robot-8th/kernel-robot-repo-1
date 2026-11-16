"""Read-only source/kit comparison; never imports drivers or connects to devices."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

BASELINE = "111984aa4197cfc20afb5959342e3e94014131d3"
MQTT = "services/roport-control-server/src/roport_control_server/adapters/mqtt"
SCOPES = {
    "control": ["services/roport-control-server"],
    "omx": ["services/roport-omx"],
    "pinky": ["services/roport-pinky", MQTT],
}
KIT_FILES = (
    "__init__.py",
    "messages.py",
    "mqtt.py",
    "journal.py",
    "pinky_trial.py",
    "trial_wire.py",
    "trial_runner.py",
    "motion.py",
)


def git(repo, *args):
    return subprocess.check_output(
        ["git", "-C", str(repo), *args], stderr=subprocess.PIPE
    )


def inspect(repo, role, config, kit=None, baseline=BASELINE):
    paths = SCOPES[role]
    changed = (
        git(repo, "diff", "--name-only", baseline, "--", *paths).decode().splitlines()
    )
    untracked = (
        git(repo, "ls-files", "--others", "--exclude-standard", "--", *paths)
        .decode()
        .splitlines()
    )
    kit_errors = []
    if role == "pinky":
        if kit is None:
            raise ValueError("Pinky requires --kit for the deployed MQTT code")
        for name in KIT_FILES:
            target = kit / "pinky_mqtt" / name
            expected = git(repo, "show", f"{baseline}:{MQTT}/{name}")
            if not target.is_file() or target.read_bytes() != expected:
                kit_errors.append(f"pinky_mqtt/{name}")
        actual = {
            p.relative_to(kit / "pinky_mqtt").as_posix()
            for p in (kit / "pinky_mqtt").rglob("*.py")
        }
        kit_errors.extend(sorted(actual - set(KIT_FILES)))
        requirements = kit / "requirements.txt"
        if (
            not requirements.is_file()
            or requirements.read_bytes() != b"paho-mqtt==2.1.0\n"
        ):
            kit_errors.append("requirements.txt")
    raw = config.read_bytes()
    profile = json.loads(raw)
    if not isinstance(profile, dict):
        raise ValueError("trial config must be a JSON object")
    return {
        "result": "PASS" if not (changed or untracked or kit_errors) else "MISMATCH",
        "role": role,
        "baseline": baseline,
        "checkout_head": git(repo, "rev-parse", "HEAD").decode().strip(),
        "compared_paths": paths,
        "changed_files": changed,
        "untracked_files": untracked,
        "kit_mismatches": kit_errors,
        "config_sha256": hashlib.sha256(raw).hexdigest(),
        "hardware_verified": False,
        "limits": "Source files only. Runtime overlays, installed dependencies, ignored files, broker and hardware are not verified.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--role", choices=SCOPES, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--kit", type=Path)
    args = parser.parse_args()
    try:
        report = inspect(args.repo, args.role, args.config, args.kit)
    except (OSError, ValueError, subprocess.CalledProcessError):
        parser.exit(
            2,
            "Cannot verify: check repository/baseline object, config JSON and kit paths. No device was contacted.\n",
        )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(0 if report["result"] == "PASS" else 1)


if __name__ == "__main__":
    main()
