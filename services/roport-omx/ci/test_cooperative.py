"""shelf-01 COOPERATIVE Host link with the simulated executor; no broker, panel or device."""

import importlib.util
import sys
import tempfile
import unittest
from datetime import UTC, datetime, timedelta
from pathlib import Path
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.cooperative import PROFILE, CooperativeHost
from roport_omx.mqtt_link.journal import Journal

spec = importlib.util.spec_from_file_location(
    "cooperative_mock", ROOT / "scripts/cooperative_mock.py"
)
mock = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mock)

SITE, CONTROL, BINDING = "test", "control-test", "test-binding"


def request(kind: str, payload: dict, *, expires_s: float = 10, **ids) -> dict:
    now = datetime.now(UTC)
    return {
        "schema_version": "1.0",
        "profile": PROFILE,
        "message_id": str(uuid4()),
        "type": kind,
        "site_id": SITE,
        "source": CONTROL,
        "target": "shelf-01",
        "sent_at": now.isoformat(),
        "expires_at": (now + timedelta(seconds=expires_s)).isoformat(),
        "task_id": "t1",
        "step_id": "s1",
        "command_id": ids.get("command_id", str(uuid4())),
        "execution_id": ids.get("execution_id", str(uuid4())),
        "payload": payload,
    }


def task_payload(boot_id: str, **changes) -> dict:
    payload = {
        "binding_id": BINDING,
        "expected_boot_id": boot_id,
        "operation": "INBOUND",
        "workcell_id": "shelf-01",
        "manipulation_mode": "COOPERATIVE",
        "assigned_arm_id": None,
        "robot_ids": ["omx1", "omx2"],
        "package_id": "pkg-1",
        "quantity": 1,
        "slot_id": "1a",
        "pinky_id": "pinky1",
        "dock_session_id": "dock-1",
        "resource_token": "grant-1",
        "model_version": "m1",
        "calibration_version": "c1",
    }
    return payload | changes


class CooperativeHostTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "journal.json"
        self.executor = mock.MockExecutor(run_s=0)
        self.journal = Journal(self.path)
        self.host = self.make_host()

    def tearDown(self):
        self.journal.close()
        self.dir.cleanup()

    def make_host(self, **kwargs):
        return CooperativeHost(
            self.journal,
            self.executor,
            site=SITE,
            control=CONTROL,
            binding=BINDING,
            **kwargs,
        )

    def inbound(self, **changes):
        return request(
            "shelf_inbound.requested", task_payload(self.host.boot_id, **changes)
        )

    def phases(self, events):
        return [(e["type"], e["payload"]["phase"]) for e in events]

    def test_status_reports_boot_and_armed(self):
        (event,) = self.host.handle(
            request("status.requested", {"binding_id": BINDING})
        )
        self.assertEqual(event["type"], "status.completed")
        self.assertEqual(event["payload"]["boot_id"], self.host.boot_id)
        self.assertIs(event["payload"]["armed"], True)
        self.assertIs(event["payload"]["ready"], True)
        self.assertEqual(
            event["payload"]["slots"], ["1a", "1b", "1c", "2a", "2b", "2c"]
        )

    def test_inbound_completes_when_both_arms_return(self):
        accepted = self.host.handle(self.inbound())
        self.assertEqual(
            self.phases(accepted), [("shelf_inbound.accepted", "ACCEPTED")]
        )
        (done,) = self.host.poll()
        p = done["payload"]
        self.assertEqual(
            (done["type"], p["phase"], p["phase_revision"]),
            ("shelf_inbound.completed", "COMPLETE", 2),
        )
        self.assertIs(p["departure_ready"], True)
        self.assertIsNone(p["placement_confirmed"])
        self.assertIsNone(p["quantity_confirmed"])
        self.assertEqual(p["evidence"], "arm_return_only")
        self.assertEqual(p["slot_id"], "1a")
        self.assertEqual(self.host.poll(), [])
        self.assertIsNone(self.host.hold)

    def test_progress_reports_each_step_once(self):
        self.executor.run_s = 60
        self.host.handle(self.inbound())
        (progress,) = self.host.poll()
        self.assertEqual(
            (progress["payload"]["phase"], progress["payload"]["step"]),
            ("RUNNING", "pick"),
        )
        self.assertEqual(self.host.poll(), [])

    def test_duplicate_returns_same_events_without_new_start(self):
        command = self.inbound()
        first = self.host.handle(command)
        again = self.host.handle({**command, "message_id": str(uuid4())})
        self.assertEqual(
            [e["message_id"] for e in first], [e["message_id"] for e in again]
        )
        self.assertEqual(len(self.executor.tasks), 1)

    def test_changed_content_with_same_command_id_conflicts(self):
        command = self.inbound()
        self.host.handle(command)
        changed = {**command, "payload": {**command["payload"], "slot_id": "2b"}}
        (event,) = self.host.handle(changed)
        self.assertEqual(event["payload"]["error_code"], "IDEMPOTENCY_CONFLICT")
        self.assertEqual(len(self.executor.tasks), 1)

    def test_not_armed_is_rejected_without_start(self):
        self.executor.armed = False
        (event,) = self.host.handle(self.inbound())
        self.assertEqual(
            (event["type"], event["payload"]["error_code"]),
            ("shelf_inbound.rejected", "NOT_ARMED"),
        )
        self.assertEqual(self.executor.tasks, [])

    def test_armed_check_can_be_disabled(self):
        self.executor.armed = False
        host = self.make_host(require_armed=False)
        self.host = host
        self.assertEqual(host.handle(self.inbound())[0]["payload"]["phase"], "ACCEPTED")

    def test_unreturned_arm_fails_and_holds(self):
        self.executor.unreturned = {"omx2"}
        self.host.handle(self.inbound())
        (failed,) = self.host.poll()
        p = failed["payload"]
        self.assertEqual(
            (failed["type"], p["error_code"]),
            ("shelf_inbound.failed", "ARM_NOT_RETURNED"),
        )
        self.assertIs(p["departure_ready"], False)
        self.assertIs(p["arms"]["omx2"]["returned"], False)
        (event,) = self.host.handle(self.inbound())
        self.assertEqual(event["payload"]["error_code"], "HOST_HOLD")
        self.host.clear_hold()
        self.assertEqual(
            self.host.handle(self.inbound())[0]["payload"]["phase"], "ACCEPTED"
        )

    def test_invalid_requests_are_rejected(self):
        cases = {
            "BOOT_MISMATCH": self.inbound(expected_boot_id="old-boot"),
            "BINDING_MISMATCH": self.inbound(binding_id="other"),
            "COOPERATIVE_SHELF_REQUIRED": self.inbound(manipulation_mode="DEDICATED"),
            "QUANTITY_UNSUPPORTED": self.inbound(quantity=True),
            "UNSUPPORTED_FIELDS": self.inbound(extra="x"),
            "UNKNOWN_SLOT": self.inbound(slot_id="1A"),
            "UNSUPPORTED_PROFILE": {
                **self.inbound(),
                "profile": "roport-pinky-driving-trial-v1",
            },
        }
        for code, command in cases.items():
            with self.subTest(code):
                (event,) = self.host.handle(command)
                self.assertEqual(event["payload"]["error_code"], code)
        expired = request(
            "shelf_inbound.requested", task_payload(self.host.boot_id), expires_s=-1
        )
        expired["sent_at"] = (datetime.now(UTC) - timedelta(seconds=5)).isoformat()
        self.assertEqual(
            self.host.handle(expired)[0]["payload"]["error_code"], "INVALID_REQUEST"
        )
        self.assertEqual(
            self.host.handle(self.inbound(), retained=True)[0]["payload"]["error_code"],
            "INVALID_REQUEST",
        )
        self.assertEqual(self.executor.tasks, [])

    def test_each_shelf_slot_is_accepted(self):
        for slot in ("1a", "1b", "1c", "2a", "2b", "2c"):
            with self.subTest(slot):
                (event,) = self.host.handle(self.inbound(slot_id=slot))
                self.assertEqual(event["payload"]["phase"], "ACCEPTED")
                self.assertEqual(event["payload"]["slot_id"], slot)
                self.host.poll()
        for slot in ("3a", "1d", "A1"):
            with self.subTest(slot):
                (event,) = self.host.handle(self.inbound(slot_id=slot))
                self.assertEqual(event["payload"]["error_code"], "UNKNOWN_SLOT")

    def test_unrouted_sender_gets_no_reply(self):
        with self.assertRaises(ValueError):
            self.host.handle({**self.inbound(), "source": "intruder"})

    def test_ack_clears_outbox(self):
        (event,) = self.host.handle(
            request("status.requested", {"binding_id": BINDING})
        )
        self.assertEqual(self.host.pending(), [event])
        ack = request(
            "event.ack", {"acked_message_id": event["message_id"], "status": "stored"}
        )
        del ack["expires_at"]
        self.assertEqual(self.host.handle(ack), [])
        self.assertEqual(self.host.pending(), [])

    def test_restart_during_task_holds_and_keeps_unacked_events(self):
        self.executor.run_s = 60
        accepted = self.host.handle(self.inbound())
        self.journal.close()
        self.journal = Journal(self.path)
        host = self.make_host()
        self.assertEqual(host.hold, accepted[0]["command_id"])
        self.assertEqual(
            [e["message_id"] for e in host.pending()], [accepted[0]["message_id"]]
        )
        self.assertNotEqual(host.boot_id, self.host.boot_id)


if __name__ == "__main__":
    unittest.main()
