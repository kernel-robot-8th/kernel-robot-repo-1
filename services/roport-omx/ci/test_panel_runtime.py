"""Panel job lifecycle with temporary data, no devices or heavy dependencies."""

import json
import sqlite3
import sys
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from roport_omx.panel_runtime import PanelRuntime, panel_request, workflow_handlers


def request(operation="record", request_id="synthetic-request", **params):
    return {
        "schema_version": "omx.workflow/1",
        "request_id": request_id,
        "operation": operation,
        "profile_id": "synthetic",
        "parameters": params,
    }


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.profiles = {"synthetic": {"operations": ["record", "train", "evaluate"]}}

    def runtime(self, handlers):
        runtime = PanelRuntime(
            self.root / "jobs.db",
            handlers,
            self.profiles,
            self.root / "datasets",
            self.root / "models",
        )
        self.addCleanup(runtime.close)
        return runtime

    def test_concurrent_duplicate_creates_one_job_and_payload_conflict(self):
        calls, hold = [], threading.Event()
        self.addCleanup(hold.set)

        def record(req, profile, job_dir, stop):
            calls.append(req)
            hold.wait(5)
            return {"state": "SUCCEEDED", "frames": 1}

        runtime = self.runtime({"record": record})
        with ThreadPoolExecutor(max_workers=4) as executor:
            jobs = list(executor.map(lambda _: runtime.submit(request()), range(8)))
        self.assertEqual(len({job["job_id"] for job in jobs}), 1)
        with self.assertRaisesRegex(ValueError, "IDEMPOTENCY_CONFLICT"):
            other = request()
            other["parameters"] = {}
            other["profile_id"] = "other"
            self.profiles["other"] = self.profiles["synthetic"]
            runtime.profiles["other"] = self.profiles["other"]
            runtime.submit(other)
        hold.set()
        runtime.close()
        self.assertEqual(len(calls), 1)
        self.assertEqual(runtime.status(jobs[0]["job_id"])["state"], "SUCCEEDED")

    def test_validation_and_symlink_escape(self):
        runtime = self.runtime({"train": lambda *args: {}})
        train = request(
            "train",
            dataset_id="demo",
            output_id="model",
            phase="GRASPING",
            steps=1,
            batch_size=1,
            seed=0,
            chunk_size=1,
            n_action_steps=1,
        )
        for key, value in (
            ("dataset_id", "../secret"),
            ("output_id", "/tmp/model"),
            ("steps", True),
            ("phase", "ALL"),
            ("shell", "echo"),
        ):
            invalid = json.loads(json.dumps(train))
            invalid["parameters"][key] = value
            with self.assertRaises(ValueError):
                runtime.submit(invalid)
        (self.root / "datasets").mkdir()
        (self.root / "outside").mkdir()
        (self.root / "datasets/demo").symlink_to(self.root / "outside")
        with self.assertRaisesRegex(ValueError, "escapes"):
            runtime.submit(train)
        with self.assertRaises(ValueError):
            runtime.submit(request("evaluate"))
        with self.assertRaises(ValueError):
            runtime.submit(request() | {"argv": []})

    def test_stop_is_cooperative_and_physical_unknown_blocks_new_job(self):
        started = threading.Event()

        def record(req, profile, job_dir, stop):
            started.set()
            stop.wait(5)
            return {"state": "RECONCILING", "error": "support not confirmed"}

        runtime = self.runtime({"record": record})
        job = runtime.submit(request())
        self.assertTrue(started.wait(2))
        self.assertEqual(
            runtime.control(job["job_id"], "stop")["state"], "STOP_REQUESTED"
        )
        runtime.close()
        self.assertEqual(runtime.status(job["job_id"])["state"], "RECONCILING")
        with self.assertRaises(ValueError):
            runtime.control("unknown", "stop")
        with self.assertRaises(ValueError):
            runtime.control(job["job_id"], "resume")
        restored = self.runtime({"record": record})
        with self.assertRaisesRegex(ValueError, "reconciliation"):
            restored.submit(request(request_id="new"))
        with self.assertRaises(TypeError):
            restored.reconcile(job["job_id"])
        restored.profiles["synthetic"]["reconcile_job"] = lambda _: {
            "confirmed": True,
            "evidence": "synthetic actual-state check",
        }
        self.assertEqual(restored.reconcile(job["job_id"])["state"], "FAILED")

    def test_restart_recovers_queued_running_and_stop_without_execute(self):
        runtime = self.runtime({"record": lambda *args: {}})
        runtime.close()
        with sqlite3.connect(runtime.db_path) as db:
            for index, state in enumerate(("QUEUED", "RUNNING", "STOP_REQUESTED")):
                req = request(request_id=f"synthetic-{index}")
                db.execute(
                    "INSERT INTO jobs VALUES (?,?,?,?,NULL,NULL)",
                    (
                        str(index),
                        req["request_id"],
                        json.dumps(req, sort_keys=True, separators=(",", ":")),
                        state,
                    ),
                )
        calls = []
        restored = self.runtime({"record": lambda *args: calls.append(args)})
        for index in range(3):
            self.assertEqual(restored.status(str(index))["state"], "RECONCILING")
        self.assertEqual(calls, [])

    def test_database_has_one_process_owner(self):
        self.runtime({})
        with self.assertRaisesRegex(ValueError, "owns"):
            self.runtime({})

    def test_queued_device_job_rechecks_previous_unresolved_result(self):
        started, release, calls = threading.Event(), threading.Event(), []

        def first(req, profile, job_dir, stop):
            started.set()
            release.wait(5)
            return {"state": "RECONCILING"}

        runtime = self.runtime(
            {"record": first, "evaluate": lambda *args: calls.append(args)}
        )
        self.addCleanup(release.set)
        a = runtime.submit(request())
        self.assertTrue(started.wait(2))
        b = runtime.submit(request("evaluate", request_id="queued"))
        release.set()
        runtime.close()
        self.assertEqual(runtime.status(a["job_id"])["state"], "RECONCILING")
        self.assertEqual(runtime.status(b["job_id"])["state"], "FAILED")
        self.assertIn("reconciliation", runtime.status(b["job_id"])["error"])
        self.assertEqual(calls, [])

    def test_profile_training_roots_must_equal_runtime_allowlist(self):
        runtime = self.runtime({"train": lambda *args: {"state": "SUCCEEDED"}})
        profile = runtime.profiles["synthetic"]
        profile.update(
            dataset_root=self.root / "outside", checkpoint_root=self.root / "models"
        )
        train = request(
            "train",
            dataset_id="demo",
            output_id="model",
            phase="GRASPING",
            steps=1,
            batch_size=1,
            seed=0,
            chunk_size=1,
            n_action_steps=1,
        )
        with self.assertRaisesRegex(ValueError, "roots differ"):
            runtime.submit(train)

    def test_panel_bridge_rejects_unapproved_device_form_and_policy(self):
        profile = {
            "panel_parameters": {
                "record": {"task": "synthetic-inbound", "episodes": 1},
                "evaluate": {"preset_id": "synthetic"},
            }
        }
        result = panel_request(
            "/api/record",
            {"task": "synthetic-inbound", "episodes": 1},
            request_id="r",
            profile_id="synthetic",
            profile=profile,
        )
        self.assertEqual(result, request(request_id="r"))
        for route, body in (
            ("/api/record", {"argv": []}),
            ("/api/eval/start", {"model_path": "/arbitrary"}),
            ("/api/train/start", {"policy": "diffusion"}),
        ):
            with self.assertRaises(ValueError):
                panel_request(
                    route, body, request_id="r", profile_id="synthetic", profile=profile
                )
        self.assertEqual(set(workflow_handlers()), {"train"})


if __name__ == "__main__":
    unittest.main()
