import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from check_trial_baseline import KIT_FILES, MQTT, inspect


class BaselineTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo = self.root / "repo"
        self.repo.mkdir()
        self.git("init", "-q")
        self.git("config", "user.name", "test")
        self.git("config", "user.email", "test@example.com")
        for name in KIT_FILES:
            path = self.repo / MQTT / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("# source\n")
        self.git("add", ".")
        self.git("commit", "-qm", "baseline")
        self.base = self.git("rev-parse", "HEAD").strip()
        self.config = self.root / "trial.json"
        self.config.write_text(json.dumps({"profile": "test"}))

    def git(self, *args):
        return subprocess.check_output(
            ["git", "-C", str(self.repo), *args], stderr=subprocess.PIPE, text=True
        )

    def test_changed_and_untracked_sources_fail(self):
        self.assertEqual(
            inspect(self.repo, "control", self.config, baseline=self.base)["result"],
            "PASS",
        )
        (self.repo / MQTT / "mqtt.py").write_text("# changed\n")
        (self.repo / MQTT / "extra.py").write_text("# extra\n")
        result = inspect(self.repo, "control", self.config, baseline=self.base)
        self.assertEqual(result["result"], "MISMATCH")
        self.assertIn(f"{MQTT}/mqtt.py", result["changed_files"])
        self.assertIn(f"{MQTT}/extra.py", result["untracked_files"])

    def test_kit_is_compared_with_source_not_its_own_manifest(self):
        kit = self.root / "kit"
        (kit / "pinky_mqtt").mkdir(parents=True)
        for name in KIT_FILES:
            (kit / "pinky_mqtt" / name).write_text("# source\n")
        (kit / "requirements.txt").write_text("paho-mqtt==2.1.0\n")
        result = inspect(self.repo, "pinky", self.config, kit, self.base)
        self.assertEqual(result["result"], "PASS")
        (kit / "pinky_mqtt/mqtt.py").write_text("# old version\n")
        self.assertEqual(
            inspect(self.repo, "pinky", self.config, kit, self.base)["result"],
            "MISMATCH",
        )

    def test_document_only_commit_and_config_hash(self):
        before = inspect(self.repo, "omx", self.config, baseline=self.base)
        (self.repo / "README.md").write_text("documentation\n")
        self.git("add", ".")
        self.git("commit", "-qm", "docs")
        after = inspect(self.repo, "omx", self.config, baseline=self.base)
        self.assertNotEqual(before["checkout_head"], after["checkout_head"])
        self.assertEqual(after["result"], "PASS")
        self.config.write_text("{}\n")
        self.assertNotEqual(
            after["config_sha256"],
            inspect(self.repo, "omx", self.config, baseline=self.base)["config_sha256"],
        )

    def test_explicit_integrated_baseline_is_checked_by_cli(self):
        script = Path(__file__).with_name("check_trial_baseline.py")
        result = subprocess.run(
            [
                sys.executable,
                str(script),
                "--repo",
                str(self.repo),
                "--role",
                "control",
                "--config",
                str(self.config),
                "--baseline",
                self.base,
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["baseline"], self.base)
        result = subprocess.run(
            [
                sys.executable,
                str(script),
                "--repo",
                str(self.repo),
                "--role",
                "control",
                "--config",
                str(self.config),
                "--baseline",
                "HEAD",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 2)
