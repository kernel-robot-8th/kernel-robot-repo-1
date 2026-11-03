"""Synthetic panel → learning artifact → ACT provider → real Host validation."""

import json
import sys
import unittest
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from uuid import UUID

SERVICE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVICE / "src"))
sys.path.insert(0, str(SERVICE / "scripts"))
sys.path.insert(0, str(SERVICE.parent / "roport-ai-server/src"))
from dedicated_demo import MockClock, mock_envelopes, synthetic_profile
import test_learning as learning_fixture

import act_provider
from act_provider import ActProvider
from dedicated_policy import PolicyRouter
from observations import ObservationImageResolver
from roport_omx.dedicated import DedicatedHost, Profile
from roport_omx.observations import ObservationImageProducer
from roport_omx.panel_runtime import PanelRuntime, panel_request, workflow_handlers
from roport_omx.udp_frame_sender import fragment_frame


class Tensor:
    def __init__(self, values=None, shape=None):
        self.values, self.shape = values, shape

    def to(self, **kwargs):
        return self

    def __truediv__(self, value):
        return self

    def __getitem__(self, index):
        return Tensor([[0, 0, 0, 0, 0, 50]], [1, 6])

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return self.values


class Processor:
    def reset(self):
        pass

    def __call__(self, value):
        return value

    def eval(self):
        pass

    def predict_action_chunk(self, batch):
        return Tensor(shape=[1, 2, 6])


class IntegrationTests(unittest.TestCase):
    def test_panel_train_artifact_image_bound_act_candidate_and_host(self):
        fixture = learning_fixture.LearningTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        profile = fixture.runtime
        profile.update(
            operations=["train"],
            runner=fixture.fake_runner,
            dataset_aliases={"local/synthetic": "demo"},
            training_parameters={
                "phase": "GRASPING",
                "seed": 7,
                "chunk_size": 2,
                "n_action_steps": 1,
            },
        )
        profile["model_metadata"]["policy_bundle_version"] = "mock-bundle"
        payload = {
            "policy": "ACT",
            "datasets": [{"repo": "local/synthetic", "exclude": []}],
            "name": "model-1",
            "steps": 2,
            "batch_size": 1,
            "num_workers": 0,
            "augmentation": False,
            "note": "",
        }
        request = panel_request(
            "/api/train/start",
            payload,
            request_id="synthetic-train",
            profile_id="synthetic",
            profile=profile,
        )
        runtime = PanelRuntime(
            fixture.root / "jobs.db",
            workflow_handlers(),
            {"synthetic": profile},
            profile["dataset_root"],
            profile["checkpoint_root"],
        )
        self.addCleanup(runtime.close)
        job = runtime.submit(request)
        runtime.close()
        result = runtime.status(job["job_id"])
        self.assertEqual(result["state"], "SUCCEEDED", result)
        checkpoint = Path(result["result"]["checkpoint"])
        metadata = json.loads((checkpoint / "metadata.json").read_text())
        clock, hp = MockClock(), synthetic_profile()
        for key in (
            "feature_names",
            "feature_units",
            "use_degrees",
            "feature_schema_version",
            "calibration_version",
        ):
            hp[key] = metadata[key]
        for phases in hp["policies"].values():
            for entry in phases.values():
                entry["model_version"] = metadata["model_version"]
        host = DedicatedHost(
            Profile(hp),
            fixture.root / "host.db",
            host_boot_id="mock-host-boot",
            wall_clock=clock.wall,
            monotonic_clock=clock.monotonic,
        )
        self.addCleanup(host.close)
        command, permit = mock_envelopes(clock)
        command["payload"].update(
            {key: hp[key] for key in ("feature_schema_version", "calibration_version")}
        )
        session, peer = UUID(int=1), ("127.0.0.1", 12345)
        resolver = ObservationImageResolver(
            {("host1", "side", "rgb"): (peer, session)},
            timeout=0.1,
            max_inflight=2,
            max_bytes=100_000,
            max_frames=2,
            max_age_ns=1_000_000_000,
            max_timestamp_skew_ns=1_000_000,
            clock_ns=lambda: int(clock.wall() * 1e9),
        )

        class Sender:
            def send(self, frame, info):
                for packet in fragment_frame(frame, info):
                    resolver.feed(packet, peer)

        producer = ObservationImageProducer(
            {"side": Sender()}, source="host1", stream="rgb", session=session
        )
        images = producer.send(
            {"side": b"synthetic-jpeg"},
            frame_id=1,
            captured_ns=int(clock.wall() * 1e9),
            observation_group="synthetic-group",
        )
        for arm in ("omx1", "omx2"):
            host.observe(
                arm,
                [0, 0, 0, 0, 0, 50],
                observation_seq=1,
                captured_at=clock.stamp(),
                images=images if arm == "omx1" else None,
            )
        host.update_permit(permit)
        host.accept_command(command)
        torch = SimpleNamespace(
            float32="float32",
            tensor=lambda values, **kw: Tensor(values),
            inference_mode=nullcontext,
        )
        provider = ActProvider(checkpoint, metadata, resolver)
        router = PolicyRouter(
            {hp["policies"]["INBOUND"]["GRASPING"]["policy_id"]: provider},
            {hp["policies"]["INBOUND"]["GRASPING"]["policy_id"]: metadata},
        )
        with (
            patch.object(
                act_provider,
                "_load_runtime",
                return_value=(torch, Processor(), Processor(), Processor()),
            ),
            patch.object(
                act_provider, "_decode_jpeg", return_value=Tensor(shape=[3, 2, 2])
            ),
        ):
            host.acknowledge_reset(
                router.handle(host.reset_request(), now=clock.wall())
            )
            candidate = router.handle(host.infer_request(), now=clock.wall())
            self.assertEqual(candidate["type"], "action.candidate", candidate)
            host.validate_candidate(candidate)
            self.assertEqual(host.next_action(), (0, 0, 0, 0, 0, 50))
        self.assertEqual(metadata["evaluation"]["status"], "NOT_RUN")


if __name__ == "__main__":
    unittest.main()
