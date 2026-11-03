"""Copy pinned npm files for offline packaging, or verify the prepared assets."""

import argparse
import hashlib
import json
from pathlib import Path

ASSET_PATH = Path("src/roport_control_server/spatial_assets")


def verify(root):
    assets = root / ASSET_PATH
    for name, expected in json.loads((assets / "checksums.json").read_text()).items():
        path = assets / name
        if (
            not path.is_file()
            or hashlib.sha256(path.read_bytes()).hexdigest() != expected
        ):
            raise ValueError(
                f"Missing or changed asset: {name}. "
                "Run npm ci --ignore-scripts && npm run prepare:assets."
            )


def prepare(root):
    assets = root / ASSET_PATH
    upstream = root / "node_modules/three"
    expected_version = json.loads((root / "package.json").read_text())["dependencies"][
        "three"
    ]
    if (
        json.loads((upstream / "package.json").read_text())["version"]
        != expected_version
    ):
        raise ValueError("Three.js version mismatch; run npm ci --ignore-scripts")
    staged = {}
    for name, expected in json.loads((assets / "checksums.json").read_text()).items():
        if not name.startswith("vendor/three/"):
            continue
        relative = name.removeprefix("vendor/three/")
        source = upstream / relative.replace("esm/", "build/", 1)
        target = assets / name
        if not source.resolve().is_relative_to(upstream.resolve()) or not (
            target.resolve().is_relative_to((assets / "vendor/three").resolve())
        ):
            raise ValueError("Asset path must stay inside the Three.js bundle")
        data = source.read_bytes()
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f"Upstream checksum mismatch: {name}")
        staged[target] = data
    # Validate every upstream file before replacing any prepared file.
    for target, data in staged.items():
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    verify(root)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    (verify if args.check else prepare)(root)
    print("Offline display asset checksums verified.")
