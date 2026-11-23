"""A recovery confirmation only releases the lock of the evaluation it was recorded for."""

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from act_test_support import MockClock, synthetic_profile
from test_lerobot_runtime import FakeDevices, FakeWriter

from roport_omx import lerobot_runtime
from roport_omx.act_support import Rejected
from roport_omx.lerobot_runtime import DeviceLock, reconcile_evaluation, record

CONFIRM = {"stopped_and_supported": True}


class RecoveryIdentityTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.addCleanup(lerobot_runtime._RECOVERY_LOCKS.clear)
        self.root = Path(temporary.name)
        self.clock, self.writer = MockClock(), FakeWriter()
        self.profile = {
            "device_mode": "fake",
            "authorization": {"supervisor": True},
            "host_profile": synthetic_profile(),
            "operation": "INBOUND",
            "arm_id": "omx1",
            "fps": 10000,
            "max_frames": 2,
            "shared_lock_path": str(self.root / "devices.lock"),
            "writer_factory": lambda request, profile: self.writer,
            "wall_clock": self.clock.wall,
        }
        self.devices = FakeDevices(self.clock, self.profile)
        self.lock_path = self.profile["shared_lock_path"]

    def unreleased_run(self, request_id):
        """An evaluation that ends RECONCILING; returns its job with onsite release evidence."""
        self.devices.confirm_stop = False
        request = {
            "schema_version": "omx.workflow/1",
            "request_id": request_id,
            "profile_id": "fake",
            "operation": "record",
            "parameters": {},
        }
        result = record(request, self.profile, self.devices)
        self.assertEqual(result["state"], "RECONCILING")
        # Real LeRobotDevices.stop() returns a dict here; the fake returns a string.
        evidence = result["physical_stop"]["evidence"]
        evidence["evidence"] = {}
        evidence["ownership_released"] = True
        return {"result": result}

    def lock_is_held(self):
        try:
            DeviceLock(self.lock_path).acquire().release()
        except Rejected:
            return True
        return False

    def test_the_result_names_the_evaluation_it_belongs_to(self):
        job = self.unreleased_run("job-A")
        self.assertEqual(job["result"]["request_id"], "job-A")

    def test_resending_an_old_recovery_does_not_release_a_later_runs_lock(self):
        job_a = self.unreleased_run("job-A")
        self.assertTrue(reconcile_evaluation(self.profile, job_a, CONFIRM)["confirmed"])
        job_b = self.unreleased_run("job-B")  # same lock path, now unresolved again
        with self.assertRaises(Rejected):
            reconcile_evaluation(self.profile, job_a, CONFIRM)
        self.assertTrue(self.lock_is_held())
        self.assertTrue(reconcile_evaluation(self.profile, job_b, CONFIRM)["confirmed"])
        self.assertFalse(self.lock_is_held())

    def test_the_same_holds_after_a_restart_with_only_the_marker_file(self):
        job_a = self.unreleased_run("job-A")
        self.assertTrue(reconcile_evaluation(self.profile, job_a, CONFIRM)["confirmed"])
        job_b = self.unreleased_run("job-B")
        # Simulate a restart: the in-memory lock is gone, only the marker file remains.
        lerobot_runtime._RECOVERY_LOCKS.pop(
            str(Path(self.lock_path).resolve())
        ).release()
        with self.assertRaises(Rejected):
            reconcile_evaluation(self.profile, job_a, CONFIRM)
        self.assertTrue(self.lock_is_held())
        self.assertTrue(reconcile_evaluation(self.profile, job_b, CONFIRM)["confirmed"])
        self.assertFalse(self.lock_is_held())


if __name__ == "__main__":
    unittest.main()
