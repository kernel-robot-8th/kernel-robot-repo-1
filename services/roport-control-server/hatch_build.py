"""Reject incomplete release packages without downloading during a build."""

import subprocess
import sys
from pathlib import Path

from hatchling.builders.hooks.plugin.interface import BuildHookInterface


class CustomBuildHook(BuildHookInterface):
    def initialize(self, version, build_data):
        if version == "editable":
            return
        subprocess.run(
            [
                sys.executable,
                str(Path(self.root) / "tools/prepare_web_assets.py"),
                "--check",
            ],
            check=True,
        )
