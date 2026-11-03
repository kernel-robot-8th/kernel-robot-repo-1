"""Explicit local learning environment entrypoint. Never connects to a robot."""

import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from roport_omx.learning import inspect_dataset, prepare_phase_dataset, run_training


def main():
    os.environ.update(
        HF_HUB_OFFLINE="1", HF_DATASETS_OFFLINE="1", WANDB_MODE="disabled"
    )
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("inspect", "prepare"):
        command = commands.add_parser(name)
        command.add_argument("--root", required=True, type=Path)
        command.add_argument("--expected-profile", required=True, type=Path)
        if name == "prepare":
            command.add_argument("--phase", required=True)
            command.add_argument("--output-dir", required=True, type=Path)
    train = commands.add_parser("train")
    train.add_argument("--request", required=True, type=Path)
    train.add_argument("--runtime-profile", required=True, type=Path)
    train.add_argument("--job-dir", required=True, type=Path)
    args = parser.parse_args()
    if args.command == "train":
        request = json.loads(args.request.read_text())
        profile = json.loads(args.runtime_profile.read_text())
        root = Path(profile["dataset_root"]) / request["parameters"]["dataset_id"]
        manifest = inspect_dataset(root, profile["expected_profile"])
        result = run_training(request, manifest, profile, args.job_dir)
    else:
        expected = json.loads(args.expected_profile.read_text())
        if args.command == "inspect":
            result = inspect_dataset(args.root, expected)
        else:
            result = prepare_phase_dataset(
                args.root, expected, args.phase, args.output_dir
            )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
