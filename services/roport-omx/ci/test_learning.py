"""Synthetic dataset contracts and fake learning processes; no torch or devices."""

import json
import subprocess
import sys
import tempfile
import threading
import unittest
from copy import deepcopy
from pathlib import Path
from typing import ClassVar
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx import learning


def example_profile():
    return {
        "operation": "INBOUND",
        "arm_id": "omx1",
        "feature_names": [f"joint{i}.pos" for i in range(5)] + ["gripper.pos"],
        "feature_units": ["degrees"] * 5 + ["range_0_100"],
        "use_degrees": True,
        "feature_schema_version": "synthetic-six-v1",
        "calibration_version": "synthetic-cal-v1",
        "fps": 10,
        "timestamp_tolerance_s": 0.001,
        "cameras": {"observation.images.side": {"shape": [2, 2, 3], "fps": 10}},
    }


def write_dataset(root):
    profile = example_profile()
    features = {
        key: {"dtype": "float32", "shape": [6], "names": profile["feature_names"]}
        for key in ("observation.state", "action")
    }
    features["observation.images.side"] = {
        "dtype": "video",
        "shape": [2, 2, 3],
        "names": ["height", "width", "channels"],
    }
    (root / "meta").mkdir(parents=True)
    (root / "meta/info.json").write_text(
        json.dumps(
            {
                "codebase_version": "v3.0",
                "fps": 10,
                "features": features,
                "total_episodes": 3,
                "total_frames": 24,
            }
        )
    )
    (root / "videos").mkdir()
    (root / "videos/side.mp4").write_bytes(b"synthetic-video")
    (root / "data/chunk-000").mkdir(parents=True)
    (root / "data/chunk-000/file-000.parquet").write_bytes(b"synthetic-not-parquet")
    (root / "meta/episodes/chunk-000").mkdir(parents=True)
    (root / "meta/episodes/chunk-000/file-000.parquet").write_bytes(
        b"synthetic-not-parquet"
    )
    (root / "meta/stats.json").write_text("{}")
    episodes = []
    for index, split in enumerate(("train", "validation", "evaluation")):
        episodes.append(
            {
                "episode_index": index,
                "source_episode_id": f"demo-{index}",
                "session_id": f"session-{index}",
                "condition_group_id": f"condition-{index}",
                "operation": "INBOUND",
                "arm_id": "omx1",
                "slot_id": "synthetic-slot",
                "package_id": "synthetic-box",
                "stopping_deviation": {"x_mm": 0, "y_mm": 1, "yaw_deg": 0},
                "split": split,
                "num_frames": 8,
                "complete": True,
                "failure_label": "NONE",
                "camera_files": {"observation.images.side": ["videos/side.mp4"]},
                "frame_timestamps": [
                    {
                        "observation_ns": (i + 1) * 100_000_000,
                        "action_ns": (i + 1) * 100_000_000 + 100,
                        "cameras_ns": {
                            "observation.images.side": (i + 1) * 100_000_000 + 200
                        },
                    }
                    for i in range(8)
                ],
                "phases": [
                    {
                        "phase": phase,
                        "start_frame": i * 2,
                        "end_frame": i * 2 + 2,
                        "evidence": "synthetic supervisor annotation",
                    }
                    for i, phase in enumerate(learning.PHASES)
                ],
            }
        )
    manifest = {
        "schema_version": "omx.dataset/1",
        "dataset_version": "synthetic-v1",
        "repo_id": "local/synthetic",
        "complete": True,
        "profile": profile,
        "episodes": episodes,
    }
    (root / "manifest.json").write_text(json.dumps(manifest))
    return manifest


class FakeValue:
    def __init__(self, value, shape=None):
        self.value = value
        self.shape = shape

    def item(self):
        return self.value

    def tolist(self):
        return self.value

    def permute(self, *axes):
        if axes != (1, 2, 0):
            raise AssertionError("expected CHW to HWC conversion")
        return FakeValue(self.value, (2, 2, 3))

    def numpy(self):
        return self


class FakeDataset:
    outputs: ClassVar[list] = []
    bad_timestamp = False

    def __init__(self, repo_id, root, episodes, **kwargs):
        self.episode = episodes[0]
        self.features = json.loads((Path(root) / "meta/info.json").read_text())[
            "features"
        ]

    def __len__(self):
        return 8

    def __getitem__(self, index):
        return {
            "timestamp": FakeValue(index / 10 + (0.1 if self.bad_timestamp else 0)),
            "episode_index": FakeValue(self.episode),
            "frame_index": FakeValue(index),
            "observation.state": FakeValue([float(self.episode)] * 6),
            "action": FakeValue([float(self.episode)] * 6),
            "observation.images.side": FakeValue(index, (3, 2, 2)),
        }

    @classmethod
    def create(cls, **kwargs):
        result = object.__new__(cls)
        result.frames = []
        result.saved = []
        result.root = Path(kwargs["root"])
        result.root.mkdir(parents=True)
        cls.outputs.append(result)
        return result

    def add_frame(self, frame):
        self.frames.append(frame)

    def save_episode(self):
        self.saved.append(self.frames)
        self.frames = []

    def finalize(self):
        (self.root / "meta").mkdir(exist_ok=True)
        (self.root / "meta/stats.json").write_text(
            json.dumps(
                {
                    "action": {
                        "mean": self.saved[0][0]["action"].tolist()
                        if self.saved
                        else []
                    }
                }
            )
        )


class LearningTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.dataset = self.root / "data/demo"
        self.manifest = write_dataset(self.dataset)
        self.expected = example_profile()
        FakeDataset.outputs = []
        FakeDataset.bad_timestamp = False
        self.request = {
            "schema_version": "omx.workflow/1",
            "request_id": "train-1",
            "operation": "train",
            "profile_id": "synthetic",
            "parameters": {
                "dataset_id": "demo",
                "output_id": "model-1",
                "phase": "GRASPING",
                "steps": 2,
                "batch_size": 1,
                "seed": 7,
                "chunk_size": 2,
                "n_action_steps": 1,
            },
        }
        self.runtime = {
            "python_executable": sys.executable,
            "learning_cli": str(ROOT / "scripts/learning_cli.py"),
            "dataset_root": self.root / "data",
            "checkpoint_root": self.root / "models",
            "expected_profile": self.expected,
            "device": "cpu",
            "runtime_version": "synthetic-v1",
            "environment": {"python": "3.12-synthetic", "lerobot": "v0.6.1"},
            "model_metadata": {"policy_bundle_version": "synthetic-bundle"},
            "prepared_dataset_root": self.root / "prepared",
        }

    def inspect(self):
        return learning.inspect_dataset(self.dataset, self.expected)

    def save_manifest(self):
        (self.dataset / "manifest.json").write_text(json.dumps(self.manifest))

    def test_inspect_accepts_complete_synthetic_contract(self):
        result = self.inspect()
        self.assertEqual(result["dataset_version"], "synthetic-v1")
        self.assertEqual(len(result["episodes"]), 3)

    def test_inspect_cli_uses_core_environment(self):
        profile = self.root / "expected.json"
        profile.write_text(json.dumps(self.expected))
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts/learning_cli.py"),
                "inspect",
                "--root",
                str(self.dataset),
                "--expected-profile",
                str(profile),
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertEqual(json.loads(result.stdout)["dataset_version"], "synthetic-v1")

    def test_rejects_incomplete_feature_units_and_camera_files(self):
        for change in ("complete", "feature", "units", "camera", "fps", "calibration"):
            with self.subTest(change=change):
                self.manifest = write_dataset_fresh(self.dataset)
                if change == "complete":
                    self.manifest["episodes"][0]["complete"] = False
                elif change == "feature":
                    self.manifest["profile"]["feature_names"].reverse()
                elif change == "units":
                    self.manifest["profile"]["feature_units"][0] = "rad"
                elif change == "camera":
                    self.manifest["episodes"][0]["camera_files"][
                        "observation.images.side"
                    ] = ["absent.mp4"]
                elif change == "fps":
                    self.manifest["profile"]["fps"] = 20
                else:
                    self.manifest["profile"]["calibration_version"] = "other"
                self.save_manifest()
                with self.assertRaises(ValueError):
                    self.inspect()

    def test_rejects_source_session_condition_leakage(self):
        for key in ("source_episode_id", "session_id", "condition_group_id"):
            with self.subTest(key=key):
                self.manifest = write_dataset_fresh(self.dataset)
                self.manifest["episodes"][1][key] = self.manifest["episodes"][0][key]
                self.save_manifest()
                with self.assertRaisesRegex(ValueError, "split leakage"):
                    self.inspect()

    def test_rejects_phase_overlap_boundary_and_reversed_outbound(self):
        for change in ("overlap", "boundary", "reverse"):
            with self.subTest(change=change):
                self.manifest = write_dataset_fresh(self.dataset)
                episode = self.manifest["episodes"][0]
                if change == "overlap":
                    episode["phases"][1]["start_frame"] = 1
                elif change == "boundary":
                    episode["phases"][-1]["end_frame"] = 9
                else:
                    episode["reversed_from_inbound"] = True
                self.save_manifest()
                with self.assertRaises(ValueError):
                    self.inspect()

    def test_rejects_unsynchronized_timestamps(self):
        self.manifest["episodes"][0]["frame_timestamps"][2]["action_ns"] += 10_000_000
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "synchron"):
            self.inspect()

    def test_rejects_incomplete_data_and_metadata_files(self):
        (self.dataset / "data/chunk-000/file-000.parquet").unlink()
        with self.assertRaisesRegex(ValueError, "data"):
            self.inspect()

    def test_camera_timestamp_order_does_not_depend_on_json_object_order(self):
        self.expected["cameras"]["observation.images.wrist"] = {
            "shape": [2, 2, 3],
            "fps": 10,
        }
        self.manifest["profile"] = deepcopy(self.expected)
        info = json.loads((self.dataset / "meta/info.json").read_text())
        info["features"]["observation.images.wrist"] = deepcopy(
            info["features"]["observation.images.side"]
        )
        (self.dataset / "meta/info.json").write_text(json.dumps(info))
        for episode in self.manifest["episodes"]:
            episode["camera_files"]["observation.images.wrist"] = ["videos/side.mp4"]
            for i, record in enumerate(episode["frame_timestamps"]):
                t = record["observation_ns"] + i * 900_000
                record["observation_ns"] = t
                record["action_ns"] = t + 100
                pairs = [
                    ("observation.images.side", t - 900_000),
                    ("observation.images.wrist", t),
                ]
                record["cameras_ns"] = dict(pairs if i % 2 else reversed(pairs))
        self.save_manifest()
        self.assertEqual(self.inspect()["profile"]["cameras"], self.expected["cameras"])

    def test_extracts_phase_per_split_with_train_only_statistics(self):
        prepared = learning.prepare_phase_dataset(
            self.dataset,
            self.expected,
            "TRANSFERRING",
            self.root / "prepared",
            dataset_cls=FakeDataset,
        )
        self.assertEqual(set(prepared["splits"]), {"train", "validation", "evaluation"})
        for output in FakeDataset.outputs:
            self.assertEqual(
                [f["observation.images.side"].value for f in output.saved[0]], [2, 3]
            )
            self.assertNotIn("timestamp", output.saved[0][0])
        stats = json.loads((self.root / "prepared/train/meta/stats.json").read_text())
        self.assertEqual(stats["action"]["mean"], [0.0] * 6)
        self.assertEqual(prepared["statistics_split"], "train")

    def test_extraction_rejects_actual_timestamp_drift(self):
        FakeDataset.bad_timestamp = True
        with self.assertRaisesRegex(ValueError, "timestamp"):
            learning.prepare_phase_dataset(
                self.dataset,
                self.expected,
                "GRASPING",
                self.root / "prepared",
                dataset_cls=FakeDataset,
            )

    def test_held_out_failure_labels_are_retained_without_training_on_them(self):
        self.manifest["episodes"][2]["failure_label"] = "DROP"
        self.save_manifest()
        prepared = learning.prepare_phase_dataset(
            self.dataset,
            self.expected,
            "GRASPING",
            self.root / "prepared",
            dataset_cls=FakeDataset,
        )
        self.assertEqual(prepared["episodes"][-1]["failure_label"], "DROP")

    def test_build_is_act_offline_and_rejects_arbitrary_parameters(self):
        argv = learning.build_act_training(self.request, self.inspect(), self.runtime)
        self.assertIn("--policy.type=act", argv)
        self.assertIn("--policy.push_to_hub=false", argv)
        self.assertIn("--wandb.enable=false", argv)
        self.assertIn("--dataset.eval_split=0", argv)
        self.assertIn("--policy.pretrained_backbone_weights=null", argv)
        self.assertTrue(any(arg.endswith("prepared/train") for arg in argv))
        for key, value in (
            ("argv", ["sh"]),
            ("output_id", "../escape"),
            ("chunk_size", True),
        ):
            request = deepcopy(self.request)
            request["parameters"][key] = value
            with self.assertRaises(ValueError):
                learning.build_act_training(request, self.inspect(), self.runtime)

    def fake_runner(self, argv, **kwargs):
        self.assertIs(kwargs["shell"], False)
        self.assertEqual(kwargs["env"]["HF_HUB_OFFLINE"], "1")
        if "prepare" in argv:
            dest = Path(argv[argv.index("--output-dir") + 1])
            learning.prepare_phase_dataset(
                self.dataset, self.expected, "GRASPING", dest, dataset_cls=FakeDataset
            )
        else:
            output = Path(
                next(x.split("=", 1)[1] for x in argv if x.startswith("--output_dir="))
            )
            checkpoint = output / "checkpoints/000002/pretrained_model"
            checkpoint.mkdir(parents=True)
            config = {
                "type": "act",
                "chunk_size": 2,
                "n_action_steps": 1,
                "input_features": {
                    "observation.state": {"shape": [6]},
                    "observation.images.side": {"shape": [3, 2, 2]},
                },
                "output_features": {"action": {"shape": [6]}},
            }
            (checkpoint / "config.json").write_text(json.dumps(config))
            (checkpoint / "model.safetensors").write_bytes(
                b"synthetic-not-real-weights"
            )
            for name in ("preprocessor", "postprocessor"):
                (checkpoint / f"policy_{name}.json").write_text(
                    json.dumps(
                        {
                            "steps": [
                                {
                                    "state_file": f"{name}.safetensors",
                                    "registry_name": "normalizer_processor"
                                    if name == "preprocessor"
                                    else "unnormalizer_processor",
                                }
                            ]
                        }
                    )
                )
                (checkpoint / f"{name}.safetensors").write_bytes(b"synthetic-state")
            (output / "checkpoints/last").symlink_to("000002", target_is_directory=True)
        return 0

    def test_training_requires_artifacts_and_writes_handoff(self):
        self.runtime["runner"] = self.fake_runner
        result = learning.run_training(
            self.request, self.inspect(), self.runtime, self.root / "job"
        )
        self.assertEqual(result["state"], "SUCCEEDED")
        metadata = json.loads(
            (Path(result["checkpoint"]) / "metadata.json").read_text()
        )
        self.assertEqual(metadata["assigned_arm_id"], "omx1")
        self.assertEqual(metadata["cameras"]["side"]["shape"], [3, 2, 2])
        self.assertEqual(metadata["evaluation"]["physical_success_rate"], None)
        self.assertEqual(metadata["training"]["seed"], 7)
        with self.assertRaises(FileExistsError):
            learning.run_training(
                self.request, self.inspect(), self.runtime, self.root / "job2"
            )

    def test_exit_zero_without_checkpoint_is_failed(self):
        def runner(argv, **kwargs):
            if "prepare" in argv:
                return self.fake_runner(argv, **kwargs)
            return 0

        self.runtime["runner"] = runner
        with self.assertRaisesRegex(ValueError, "checkpoint"):
            learning.run_training(
                self.request, self.inspect(), self.runtime, self.root / "job"
            )

    def test_processor_state_missing_is_failed(self):
        def runner(argv, **kwargs):
            result = self.fake_runner(argv, **kwargs)
            if "prepare" not in argv:
                (
                    self.root
                    / "models/model-1/checkpoints/000002/pretrained_model/preprocessor.safetensors"
                ).unlink()
            return result

        self.runtime["runner"] = runner
        with self.assertRaisesRegex(ValueError, "processor"):
            learning.run_training(
                self.request, self.inspect(), self.runtime, self.root / "job"
            )

    def test_stateless_normalizer_cannot_be_reported_as_training_success(self):
        def runner(argv, **kwargs):
            result = self.fake_runner(argv, **kwargs)
            if "prepare" not in argv:
                checkpoint = (
                    self.root / "models/model-1/checkpoints/000002/pretrained_model"
                )
                (checkpoint / "policy_preprocessor.json").write_text(
                    json.dumps({"steps": [{"registry_name": "normalizer_processor"}]})
                )
            return result

        self.runtime["runner"] = runner
        with self.assertRaisesRegex(ValueError, "normaliz"):
            learning.run_training(
                self.request, self.inspect(), self.runtime, self.root / "job"
            )

    def test_training_failure_and_stop_between_processes(self):
        self.runtime["runner"] = lambda *args, **kwargs: 3
        with self.assertRaisesRegex(RuntimeError, "code 3"):
            learning.run_training(
                self.request, self.inspect(), self.runtime, self.root / "failed"
            )
        stop = threading.Event()

        def runner(argv, **kwargs):
            result = self.fake_runner(argv, **kwargs)
            stop.set()
            return result

        self.runtime.update(runner=runner, stop_event=stop)
        with self.assertRaises(InterruptedError):
            learning.run_training(
                self.request, self.inspect(), self.runtime, self.root / "stopped"
            )
        self.assertFalse((self.root / "models/model-1").exists())

    def test_subprocess_cancel_terminates_learning_process(self):
        stop = threading.Event()

        class Process:
            stopped = False

            def wait(self, timeout=None):
                stop.set()
                return 0

            def poll(self):
                return None

            def terminate(self):
                self.stopped = True

        process = Process()
        with (
            patch.object(learning.subprocess, "Popen", return_value=process),
            self.assertRaises(InterruptedError),
        ):
            learning._run_process(
                ["fake"],
                cwd=self.root,
                env={},
                log_path=self.root / "log",
                stop_event=stop,
                shell=False,
            )
        self.assertTrue(process.stopped)

    def test_cancel_before_launch_does_not_run_runner(self):
        stop = threading.Event()
        stop.set()
        self.runtime["stop_event"] = stop
        self.runtime["runner"] = lambda *a, **k: self.fail("cancelled process ran")
        with self.assertRaises(InterruptedError):
            learning.run_training(
                self.request, self.inspect(), self.runtime, self.root / "job"
            )

    def test_output_reservation_excludes_competing_job_and_preserves_other_lock(self):
        lock = self.root / "models/.model-1.lock"

        def runner(argv, **kwargs):
            self.assertTrue(lock.is_file())
            with self.assertRaises(FileExistsError):
                learning.run_training(
                    self.request, self.inspect(), self.runtime, self.root / "competitor"
                )
            return self.fake_runner(argv, **kwargs)

        self.runtime["runner"] = runner
        learning.run_training(
            self.request, self.inspect(), self.runtime, self.root / "job"
        )
        self.assertFalse(lock.exists())
        request = deepcopy(self.request)
        request["parameters"]["output_id"] = "model-2"
        foreign = self.root / "models/.model-2.lock"

        def replaced_lock_runner(argv, **kwargs):
            foreign.write_text("another-owner")
            return 1

        self.runtime["runner"] = replaced_lock_runner
        with self.assertRaises(RuntimeError):
            learning.run_training(
                request, self.inspect(), self.runtime, self.root / "other-job"
            )
        self.assertEqual(foreign.read_text(), "another-owner")

    def test_dangling_output_and_lock_symlinks_fail_closed(self):
        models = self.root / "models"
        models.mkdir()
        outside = self.root / "outside"
        (models / "model-1").symlink_to(outside, target_is_directory=True)
        self.runtime["runner"] = lambda *a, **kw: self.fail("symlink path executed")
        with self.assertRaises(ValueError):
            learning.run_training(
                self.request, self.inspect(), self.runtime, self.root / "job"
            )
        (models / "model-1").unlink()
        (models / ".model-1.lock").symlink_to(models / "absent-lock-target")
        with self.assertRaises(FileExistsError):
            learning.run_training(
                self.request, self.inspect(), self.runtime, self.root / "job"
            )
        self.assertFalse((models / "absent-lock-target").exists())


def write_dataset_fresh(root):
    # Files are shared synthetic fixtures; reset JSON without changing recorded bytes.
    manifest = json.loads((root / "manifest.json").read_text())
    profile = example_profile()
    manifest["complete"] = True
    manifest["profile"] = profile
    for i, episode in enumerate(manifest["episodes"]):
        episode.update(
            complete=True,
            source_episode_id=f"demo-{i}",
            session_id=f"session-{i}",
            condition_group_id=f"condition-{i}",
            reversed_from_inbound=False,
        )
        episode["camera_files"] = {"observation.images.side": ["videos/side.mp4"]}
        for p, phase in enumerate(episode["phases"]):
            phase.update(start_frame=p * 2, end_frame=p * 2 + 2)
    return manifest


if __name__ == "__main__":
    unittest.main()
