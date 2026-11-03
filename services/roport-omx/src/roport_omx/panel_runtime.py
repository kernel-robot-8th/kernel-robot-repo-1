"""Local panel job boundary; no web server, device connection or model import."""

import json
import re
import sqlite3
import threading
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

SCHEMA = "omx.workflow/1"
STATES = {"QUEUED", "RUNNING", "SUCCEEDED", "FAILED", "STOP_REQUESTED", "RECONCILING"}
PARAMETERS = {
    "record": set(),
    "evaluate": set(),
    "train": {
        "dataset_id",
        "output_id",
        "phase",
        "steps",
        "batch_size",
        "seed",
        "chunk_size",
        "n_action_steps",
    },
}


def _identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,128}", value):
        raise ValueError("expected a bounded identifier")
    if value in {".", ".."}:
        raise ValueError("invalid identifier")
    return value


def _inside(root, value):
    _identifier(value)
    path = (root / value).resolve()
    if path == root or not path.is_relative_to(root):
        raise ValueError("reference escapes configured root")
    return path


class PanelRuntime:
    """One local job owner per DB; handlers run serially outside submit().

    Trusted handlers receive (request, profile, job_dir, stop_event). A returned
    state may be SUCCEEDED, FAILED or RECONCILING; cancellation never proves a
    physical stop. All paths and physical authorization come from deployment.
    """

    def __init__(
        self, db_path, handlers, profile_registry, dataset_root, checkpoint_root
    ):
        import fcntl

        self.db_path = Path(db_path).resolve()
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self.dataset_root = Path(dataset_root).resolve()
        self.checkpoint_root = Path(checkpoint_root).resolve()
        self.handlers = dict(handlers)
        if set(self.handlers) - PARAMETERS.keys() or any(
            not callable(handler) for handler in self.handlers.values()
        ):
            raise ValueError("only explicit workflow handlers can be registered")
        self.profiles = dict(profile_registry)
        self._owner = self.db_path.with_suffix(self.db_path.suffix + ".lock").open("a+")
        try:
            fcntl.flock(self._owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self._owner.close()
            raise ValueError("another panel runtime owns this job database") from None
        self._guard = threading.RLock()
        self._events = {}
        self._closed = False
        self._pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="omx-workflow"
        )
        with self._db() as db:
            db.execute("""CREATE TABLE IF NOT EXISTS jobs (
                job_id TEXT PRIMARY KEY, request_id TEXT UNIQUE NOT NULL,
                payload TEXT NOT NULL, state TEXT NOT NULL,
                result TEXT, error TEXT)""")
            db.execute("""UPDATE jobs SET state='RECONCILING',
                error='runtime restarted; inspect actual state before a new request'
                WHERE state IN ('QUEUED','RUNNING','STOP_REQUESTED')""")

    def _db(self):
        return sqlite3.connect(self.db_path, timeout=10)

    def _validate(self, request):
        if not isinstance(request, dict) or set(request) != {
            "schema_version",
            "request_id",
            "operation",
            "profile_id",
            "parameters",
        }:
            raise ValueError("exact workflow request fields required")
        if request["schema_version"] != SCHEMA:
            raise ValueError("unsupported workflow schema")
        _identifier(request["request_id"])
        _identifier(request["profile_id"])
        operation = request["operation"]
        if not isinstance(operation, str) or operation not in self.handlers:
            raise ValueError("workflow operation is not registered")
        profile = self.profiles.get(request["profile_id"])
        if not isinstance(profile, dict) or operation not in profile.get(
            "operations", []
        ):
            raise ValueError("profile does not allow this operation")
        parameters = request["parameters"]
        if not isinstance(parameters, dict) or set(parameters) - PARAMETERS[operation]:
            raise ValueError("unsupported workflow parameters")
        for key, root in (
            ("dataset_id", self.dataset_root),
            ("output_id", self.checkpoint_root),
        ):
            if key in parameters:
                _inside(root, parameters[key])
        if operation == "train":
            for key, root in (
                ("dataset_root", self.dataset_root),
                ("checkpoint_root", self.checkpoint_root),
            ):
                configured = profile.get(key)
                if configured is None or Path(configured).resolve() != root:
                    raise ValueError("profile roots differ from runtime allowed roots")
            if set(parameters) != PARAMETERS["train"]:
                raise ValueError("explicit ACT training parameters required")
            if parameters["phase"] not in {
                "GRASPING",
                "TRANSFERRING",
                "RELEASING",
                "RETURNING",
            }:
                raise ValueError("unknown dedicated phase")
            for key in ("steps", "batch_size", "chunk_size", "n_action_steps", "seed"):
                value = parameters[key]
                if type(value) is not int or value < (0 if key == "seed" else 1):
                    raise ValueError(
                        "positive explicit ACT counts and nonnegative seed required"
                    )
            if parameters["n_action_steps"] > parameters["chunk_size"]:
                raise ValueError("n_action_steps exceeds chunk_size")
        return json.dumps(
            request, sort_keys=True, separators=(",", ":"), allow_nan=False
        )

    def submit(self, request):
        with self._guard:
            if self._closed:
                raise ValueError("runtime is closed")
            payload = self._validate(request)
            with self._db() as db:
                existing = db.execute(
                    "SELECT job_id,payload FROM jobs WHERE request_id=?",
                    (request["request_id"],),
                ).fetchone()
                if existing:
                    if existing[1] != payload:
                        raise ValueError("IDEMPOTENCY_CONFLICT")
                    return self.status(existing[0])
                # Unknown physical results must be reconciled before another device job.
                if (
                    request["operation"] in {"record", "evaluate"}
                    and db.execute(
                        "SELECT 1 FROM jobs WHERE state='RECONCILING' "
                        "AND json_extract(payload,'$.operation') IN ('record','evaluate')"
                    ).fetchone()
                ):
                    raise ValueError("device reconciliation required")
                job_id = str(uuid.uuid4())
                db.execute(
                    "INSERT INTO jobs VALUES (?,?,?,?,NULL,NULL)",
                    (job_id, request["request_id"], payload, "QUEUED"),
                )
            event = threading.Event()
            self._events[job_id] = event
            self._pool.submit(self._execute, job_id, json.loads(payload), event)
            return self.status(job_id)

    def status(self, job_id):
        _identifier(job_id)
        with self._db() as db:
            row = db.execute(
                "SELECT request_id,state,result,error FROM jobs WHERE job_id=?",
                (job_id,),
            ).fetchone()
        if row is None:
            raise ValueError("unknown job_id")
        return {
            "job_id": job_id,
            "request_id": row[0],
            "state": row[1],
            "result": json.loads(row[2]) if row[2] is not None else None,
            "error": row[3],
        }

    def _save(self, job_id, state, result=None, error=None):
        if state not in STATES:
            raise ValueError("unsupported job state")
        encoded = json.dumps(result, allow_nan=False) if result is not None else None
        with self._db() as db:
            db.execute(
                "UPDATE jobs SET state=?,result=?,error=? WHERE job_id=?",
                (state, encoded, error, job_id),
            )

    def _execute(self, job_id, request, event):
        with self._guard:
            if event.is_set():
                self._save(job_id, "FAILED", error="stopped before execution")
                self._events.pop(job_id, None)
                return
            try:
                self._validate(request)
                with self._db() as db:
                    unresolved = (
                        request["operation"] != "train"
                        and db.execute(
                            "SELECT 1 FROM jobs WHERE state='RECONCILING' "
                            "AND json_extract(payload,'$.operation') IN ('record','evaluate')"
                        ).fetchone()
                    )
                if unresolved:
                    raise ValueError("previous device job requires reconciliation")
            except (TypeError, ValueError) as exc:
                self._save(job_id, "FAILED", error=str(exc))
                self._events.pop(job_id, None)
                return
            self._save(job_id, "RUNNING")
        try:
            job_dir = self.db_path.parent / "jobs" / job_id
            job_dir.mkdir(parents=True, exist_ok=False)
            profile = self.profiles[request["profile_id"]]
            result = self.handlers[request["operation"]](
                request, profile, job_dir, event
            )
            if not isinstance(result, dict):
                raise TypeError("handler must return an explicit result object")
            state = result.get("state")
            if state not in {"SUCCEEDED", "FAILED", "RECONCILING"}:
                raise ValueError(
                    "handler did not return a terminal or reconciliation state"
                )
            with self._guard:
                if event.is_set() and state == "SUCCEEDED":
                    state = (
                        "RECONCILING" if request["operation"] != "train" else "FAILED"
                    )
                self._save(job_id, state, result, result.get("error"))
        except Exception as exc:  # noqa: BLE001 -- Persist handler errors for the panel.
            state = "FAILED" if request["operation"] == "train" else "RECONCILING"
            with self._guard:
                self._save(job_id, state, error=str(exc))
        finally:
            with self._guard:
                self._events.pop(job_id, None)

    def control(self, job_id, command):
        if command != "stop":
            raise ValueError("only cooperative stop is supported; no automatic resume")
        with self._guard:
            current = self.status(job_id)
            if current["state"] in {"QUEUED", "RUNNING", "STOP_REQUESTED"}:
                event = self._events.get(job_id)
                if event is None:
                    raise ValueError("job needs reconciliation")
                self._save(job_id, "STOP_REQUESTED")
                event.set()
            return self.status(job_id)

    def reconcile(self, job_id):
        """Explicit trusted recovery check; a recovered job is never re-executed."""
        with self._guard:
            current = self.status(job_id)
            if current["state"] != "RECONCILING":
                raise ValueError("only unresolved jobs can be reconciled")
            with self._db() as db:
                request = json.loads(
                    db.execute(
                        "SELECT payload FROM jobs WHERE job_id=?", (job_id,)
                    ).fetchone()[0]
                )
            callback = self.profiles[request["profile_id"]].get("reconcile_job")
            if not callable(callback):
                raise TypeError("trusted reconciliation callback required")
            evidence = callback(current)
            if (
                not isinstance(evidence, dict)
                or evidence.get("confirmed") is not True
                or not evidence.get("evidence")
            ):
                raise ValueError("actual-state reconciliation has not been confirmed")
            self._save(
                job_id,
                "FAILED",
                {"reconciliation": evidence},
                "closed after explicit recovery; not re-executed",
            )
            return self.status(job_id)

    def close(self):
        """Drain jobs, then release database ownership; never kill device work."""
        with self._guard:
            self._closed = True
        self._pool.shutdown(wait=True)
        self._owner.close()


def workflow_handlers(*, devices_factory=None, host_factory=None):
    """Trusted composition for the panel server, independent of its HTTP/UI code."""

    def train(request, profile, job_dir, stop_event):
        from .learning import inspect_dataset, run_training

        runtime = dict(profile, stop_event=stop_event)
        root = Path(runtime["dataset_root"]).resolve()
        manifest = inspect_dataset(
            _inside(root, request["parameters"]["dataset_id"]),
            runtime["expected_profile"],
        )
        return run_training(request, manifest, runtime, job_dir)

    handlers = {"train": train}
    if devices_factory is not None:

        def record(request, profile, job_dir, stop_event):
            from .lerobot_runtime import record

            # Preserve recovery lock state on the trusted profile across requests.
            profile["stop_event"] = stop_event
            return record(request, profile, devices_factory(profile))

        handlers["record"] = record
        if host_factory is not None:

            def evaluate(request, profile, job_dir, stop_event):
                from .lerobot_runtime import evaluate

                profile["stop_event"] = stop_event
                return evaluate(
                    request, profile, host_factory(profile), devices_factory(profile)
                )

            handlers["evaluate"] = evaluate
    return handlers


def panel_request(route, payload, *, request_id, profile_id, profile):
    """Translate the supported external panel subset, rejecting silent downgrades.

    Device-job form values must match trusted profile.panel_parameters exactly.
    The panel must expose phase and training counts explicitly for train jobs.
    """
    routes = {
        "/api/record": "record",
        "/api/train/start": "train",
        "/api/eval/start": "evaluate",
    }
    operation = routes.get(route)
    if operation is None or not isinstance(payload, dict):
        raise ValueError("unsupported panel route or payload")
    if operation == "train":
        if payload.get("policy") not in {"act", "ACT"}:
            raise ValueError("only ACT is supported")
        if set(payload) == PARAMETERS["train"] | {"policy"}:
            parameters = {
                key: value for key, value in payload.items() if key != "policy"
            }
        else:
            allowed = {
                "policy",
                "steps",
                "batch_size",
                "num_workers",
                "augmentation",
                "note",
                "datasets",
                "name",
            }
            datasets = payload.get("datasets")
            if (
                set(payload) - allowed
                or not isinstance(datasets, list)
                or len(datasets) != 1
                or not isinstance(datasets[0], dict)
                or set(datasets[0]) != {"repo", "exclude"}
                or datasets[0]["exclude"] != []
                or not isinstance(datasets[0]["repo"], str)
                or payload.get("num_workers", 0) != 0
                or payload.get("augmentation", False) is not False
                or payload.get("note", "") != ""
            ):
                raise ValueError(
                    "only one complete dataset without panel transforms is supported"
                )
            dataset_id = profile.get("dataset_aliases", {}).get(datasets[0]["repo"])
            settings = profile.get("training_parameters")
            if not isinstance(settings, dict) or not isinstance(dataset_id, str):
                raise ValueError(
                    "trusted dataset alias and phase training profile required"
                )
            parameters = dict(
                settings,
                dataset_id=dataset_id,
                output_id=payload.get("name"),
                steps=payload.get("steps"),
                batch_size=payload.get("batch_size"),
            )
    else:
        allowed = (
            {"repo_id", "name", "task", "episodes", "episode_s", "reset_s"}
            if operation == "record"
            else {"preset_id", "task", "episodes", "episode_s", "reset_s"}
        )
        if set(payload) - allowed:
            raise ValueError("unsupported panel device fields; select a trusted preset")
        expected = profile.get("panel_parameters", {}).get(operation)
        if not isinstance(expected, dict) or payload != expected:
            raise ValueError("panel form differs from the approved device profile")
        parameters = {}
    return {
        "schema_version": SCHEMA,
        "request_id": request_id,
        "operation": operation,
        "profile_id": profile_id,
        "parameters": parameters,
    }
