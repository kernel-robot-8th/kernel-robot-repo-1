"""Verify offline asset bytes in the built wheel without importing the UI."""

import hashlib
import json
import sys
import zipfile

PREFIX = "roport_control_server/spatial_assets/"


def check(path):
    with zipfile.ZipFile(path) as wheel:
        manifest = json.loads(wheel.read(PREFIX + "checksums.json"))
        for name, expected in manifest.items():
            actual = hashlib.sha256(wheel.read(PREFIX + name)).hexdigest()
            if actual != expected:
                raise ValueError(f"Packaged asset checksum mismatch: {name}")
        if any("node_modules/" in name for name in wheel.namelist()):
            raise ValueError("wheel must contain only selected npm assets")
    print(f"Verified {len(manifest)} packaged asset hashes: {path}")


if __name__ == "__main__":
    check(sys.argv[1])
