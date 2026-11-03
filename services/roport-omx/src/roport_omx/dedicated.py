"""Dedicated shelf source/mock execution boundary. No motor or callback adapter."""

import json
import math
import sqlite3
import time
from collections import deque
from copy import deepcopy
from datetime import datetime
from uuid import uuid4

PROFILE_VERSION = "omx.dedicated/1"
PHASES = ("GRASPING", "TRANSFERRING", "RELEASING", "RETURNING")
WAITS = ("WAIT_GRASP", "WAIT_SEATED", "WAIT_RELEASE", "WAIT_SAFE")
BINDINGS = (
    "execution_id",
    "operation",
    "assigned_arm_id",
    "phase",
    "phase_revision",
    "policy_id",
    "model_version",
    "policy_bundle_version",
    "feature_schema_version",
    "calibration_version",
    "request_id",
    "observation_seq",
    "captured_at",
    "expires_at",
)


class Rejected(ValueError):
    """Input was rejected; an active execution is held on unsafe input."""


def finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def vector(value):
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 6
        or not all(map(finite, value))
    ):
        raise Rejected("expected six finite positions")
    return tuple(value)


def timestamp(value):
    if not isinstance(value, str):
        raise Rejected("timestamp must be offset ISO8601")
    try:
        result = datetime.fromisoformat(value)
    except ValueError as exc:
        raise Rejected("invalid timestamp") from exc
    if result.tzinfo is None:
        raise Rejected("timestamp needs UTC offset")
    return result.timestamp()


def encoded(value):
    try:
        return json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":"))
    except (ValueError, TypeError) as exc:
        raise Rejected("input must be finite JSON") from exc


def nonempty(value):
    return isinstance(value, str) and bool(value.strip())


def bounds(value):
    if not isinstance(value, list) or len(value) != 6:
        raise Rejected("six joint bounds required")
    for pair in value:
        if not isinstance(pair, list) or len(pair) != 2 or not all(map(finite, pair)):
            raise Rejected("invalid joint bounds")
        if pair[0] > pair[1]:
            raise Rejected("reversed joint bounds")
    return value


def inside(positions, limits):
    return all(lo <= pos <= hi for pos, (lo, hi) in zip(positions, limits, strict=True))


class Profile:
    """Explicit profile: missing physical and timing values have no defaults."""

    @classmethod
    def from_dict(cls, data):
        return cls(data)

    def __init__(self, data):
        if not isinstance(data, dict):
            raise Rejected("profile must be an object")
        self.data = deepcopy(data)
        d = self.data
        encoded(d)
        for name in (
            "workcell_id",
            "site_id",
            "control_id",
            "profile_version",
            "policy_bundle_version",
            "feature_schema_version",
            "calibration_version",
            "workcell_profile_version",
        ):
            if not nonempty(d.get(name)):
                raise Rejected(f"profile requires {name}")
        if d["profile_version"] != PROFILE_VERSION or d["workcell_id"] != "shelf-01":
            raise Rejected("unsupported workcell/profile")
        roles = d.get("roles")
        if not isinstance(roles, dict) or set(roles) != {"INBOUND", "OUTBOUND"}:
            raise Rejected("two operation roles required")
        if not all(map(nonempty, roles.values())) or len(set(roles.values())) != 2:
            raise Rejected("distinct arms required")
        for name in ("slot_ids", "package_ids"):
            if (
                not isinstance(d.get(name), list)
                or not d[name]
                or not all(map(nonempty, d[name]))
            ):
                raise Rejected(f"profile requires {name}")
        names = d.get("feature_names")
        if (
            not isinstance(names, list)
            or len(names) != 6
            or not all(map(nonempty, names))
            or len(set(names)) != 6
        ):
            raise Rejected("six ordered unique feature names required")
        if type(d.get("use_degrees")) is not bool:
            raise Rejected("use_degrees must be boolean")
        expected_units = ["degrees" if d["use_degrees"] else "range_m100_100"] * 5 + [
            "range_0_100"
        ]
        if d.get("feature_units") != expected_units:
            raise Rejected(
                "feature units must preserve OMX joint/gripper normalization"
            )
        directions = d.get("joint_directions")
        if (
            not isinstance(directions, list)
            or len(directions) != 6
            or any(type(x) is not int or x not in (-1, 1) for x in directions)
        ):
            raise Rejected("six explicit joint directions required")
        bounds(d.get("joint_bounds"))
        if any(lo < 0 or hi > 100 for lo, hi in [d["joint_bounds"][-1]]):
            raise Rejected("gripper bounds must use range_0_100")
        step = vector(d.get("max_step"))
        if any(x <= 0 for x in step):
            raise Rejected("positive max_step required")
        tolerance = vector(d.get("safe_tolerance"))
        if any(x < 0 for x in tolerance):
            raise Rejected("nonnegative safe tolerance required")
        safe = d.get("safe_positions")
        if not isinstance(safe, dict) or set(safe) != set(roles.values()):
            raise Rejected("safe positions required for both arms")
        for pos in safe.values():
            if not inside(vector(pos), d["joint_bounds"]):
                raise Rejected("safe positions out of bounds")
        timing = d.get("timing")
        required = {
            "clock_skew",
            "permit_max_age",
            "permit_lease",
            "observation_max_age",
            "wait_timeout",
            "inference_timeout",
            "phase_timeout",
        }
        if not isinstance(timing, dict) or set(timing) != required:
            raise Rejected("explicit timing fields required")
        if any(not finite(x) or x <= 0 for x in timing.values()):
            raise Rejected("positive finite timing values required")
        policies = d.get("policies")
        if not isinstance(policies, dict) or set(policies) != set(roles):
            raise Rejected("policies required for both operations")
        for operation in roles:
            if set(policies[operation]) != set(PHASES):
                raise Rejected("four phase policies required")
            if len({p.get("policy_id") for p in policies[operation].values()}) != 4:
                raise Rejected("four separate phase policy registrations required")
            for policy in policies[operation].values():
                if not all(
                    nonempty(policy.get(k)) for k in ("policy_id", "model_version")
                ):
                    raise Rejected("policy/model identity required")
                for name in ("entry_bounds", "exit_bounds"):
                    for pair, physical in zip(
                        bounds(policy.get(name)), d["joint_bounds"], strict=True
                    ):
                        if pair[0] < physical[0] or pair[1] > physical[1]:
                            raise Rejected(
                                "phase envelope exceeds physical joint bounds"
                            )


class DedicatedHost:
    """Durable source/mock supervisor. Returned targets are data, never motor writes."""

    def __init__(
        self,
        profile,
        journal_path,
        *,
        host_boot_id,
        wall_clock=time.time,
        monotonic_clock=time.monotonic,
    ):
        if not nonempty(host_boot_id):
            raise Rejected("host boot identity required")
        self.profile = deepcopy(profile.data)
        self.host_boot_id = host_boot_id
        self.wall_clock, self.monotonic_clock = wall_clock, monotonic_clock
        self.db = sqlite3.connect(journal_path)
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.execute(
            "CREATE TABLE IF NOT EXISTS executions (command_id TEXT PRIMARY KEY, execution_id TEXT UNIQUE NOT NULL, content TEXT NOT NULL, state TEXT NOT NULL, reason TEXT NOT NULL, result TEXT)"
        )
        if "result" not in {
            row[1] for row in self.db.execute("PRAGMA table_info(executions)")
        }:
            self.db.execute("ALTER TABLE executions ADD COLUMN result TEXT")
        self.db.execute(
            "UPDATE executions SET state='RECONCILING', reason='host restarted with uncertain execution', result=NULL WHERE state NOT IN ('SAFE_RETURNED', 'RECONCILING', 'HOLD')"
        )
        self.db.commit()
        self.state = "READY"
        self.command = None
        self.reason = ""
        self.phase_revision = 0
        self.permit = None
        self.permit_deadline = 0
        self.observations = {}
        self.queue = deque()
        self.pending = None
        self.reset_acknowledged = False
        self.last_action_seq = -1
        self.last_request_observation = -1
        self.last_confirmation_seq = -1
        self.requested_target = None
        self.phase_started = self.monotonic_clock()
        self.wait_started = None
        if self.db.execute(
            "SELECT 1 FROM executions WHERE state != 'SAFE_RETURNED'"
        ).fetchone():
            self.state = "RECONCILING"
            self.reason = (
                "journal requires explicit external reconciliation; no automatic resume"
            )

    def close(self):
        self.db.close()

    @property
    def phase(self):
        if self.state in PHASES:
            return self.state
        if self.state in WAITS:
            return PHASES[WAITS.index(self.state)]
        return None

    def _persist(self):
        if self.command:
            self.db.execute(
                "UPDATE executions SET state=?, reason=?, result=? WHERE command_id=?",
                (
                    self.state,
                    self.reason,
                    encoded(self._event_snapshot()),
                    self.command["command_id"],
                ),
            )
            self.db.commit()

    def hold(self, reason):
        self.state, self.reason = "HOLD", str(reason)
        self.queue.clear()
        self.pending = None
        self.reset_acknowledged = False
        self._persist()

    def _reject(self, reason):
        if self.command and self.state != "SAFE_RETURNED":
            self.hold(reason)
        raise Rejected(reason)

    def _fresh(self, stamp, max_age):
        age = self.wall_clock() - timestamp(stamp)
        if age < -self.profile["timing"]["clock_skew"] or age > max_age:
            raise Rejected("stale or future observation")

    def _envelope(self, envelope, kind, retained):
        if retained or not isinstance(envelope, dict):
            raise Rejected("retained/non-object message")
        encoded(envelope)
        for field in (
            "message_id",
            "type",
            "site_id",
            "source",
            "target",
            "task_id",
            "step_id",
            "command_id",
            "execution_id",
            "sent_at",
            "expires_at",
        ):
            if not nonempty(envelope.get(field)):
                raise Rejected(f"envelope requires {field}")
        if envelope.get("schema_version") != "1.0" or envelope["type"] != kind:
            raise Rejected("wrong envelope type/schema")
        sent, expiry = timestamp(envelope["sent_at"]), timestamp(envelope["expires_at"])
        if (
            sent > self.wall_clock() + self.profile["timing"]["clock_skew"]
            or expiry <= self.wall_clock()
            or expiry <= sent
        ):
            raise Rejected("invalid envelope deadline")
        if (
            envelope["site_id"] != self.profile["site_id"]
            or envelope["source"] != self.profile["control_id"]
            or envelope["target"] != self.profile["workcell_id"]
        ):
            raise Rejected("untrusted site/controller/target routing")
        p = envelope.get("payload")
        if (
            not isinstance(p, dict)
            or p.get("profile_version") != PROFILE_VERSION
            or p.get("workcell_id") != self.profile["workcell_id"]
        ):
            raise Rejected("wrong payload profile/workcell")
        return p

    def _permit_binding(self, p, command):
        c = command["payload"]
        for field in ("permit_id", "resource_token", "pinky_id", "workcell_id"):
            if p[field] != c[field]:
                raise Rejected(f"permit {field} mismatch")
        if p["execution_id"] != command["execution_id"]:
            raise Rejected("permit execution mismatch")
        if self.permit and self.command and self.state != "SAFE_RETURNED":
            old = self.permit["payload"]
            for field in (
                "control_boot_id",
                "host_boot_id",
                "pinky_boot_id",
                "docking_execution_id",
            ):
                if p[field] != old[field]:
                    raise Rejected("permit boot/docking identity changed")

    def update_permit(self, envelope, *, retained=False):
        try:
            p = self._envelope(envelope, "workcell.permit", retained)
            for field in (
                "permit_id",
                "resource_token",
                "execution_id",
                "control_boot_id",
                "host_boot_id",
                "pinky_id",
                "pinky_boot_id",
                "docking_execution_id",
                "source_observed_at",
                "expires_at",
            ):
                if not nonempty(p.get(field)):
                    raise Rejected(f"permit requires {field}")
            if (
                p["host_boot_id"] != self.host_boot_id
                or p["execution_id"] != envelope["execution_id"]
            ):
                raise Rejected("permit host/execution identity mismatch")
            if any(
                type(p.get(field)) is not bool or not p[field]
                for field in ("docked", "stopped", "allowed")
            ):
                raise Rejected("permit revoked or unproven")
            if type(p.get("evidence_seq")) is not int or p["evidence_seq"] < 0:
                raise Rejected("invalid evidence sequence")
            self._fresh(
                p["source_observed_at"], self.profile["timing"]["permit_max_age"]
            )
            expiry = min(timestamp(p["expires_at"]), timestamp(envelope["expires_at"]))
            if (
                expiry <= self.wall_clock()
                or timestamp(p["source_observed_at"])
                > timestamp(envelope["sent_at"]) + self.profile["timing"]["clock_skew"]
            ):
                raise Rejected("invalid permit expiry/source time")
            if self.command and self.state != "SAFE_RETURNED":
                self._permit_binding(p, self.command)
                for field in (
                    "site_id",
                    "source",
                    "target",
                    "task_id",
                    "step_id",
                    "command_id",
                    "execution_id",
                ):
                    if envelope[field] != self.command[field]:
                        raise Rejected("permit envelope binding mismatch")
            if self.permit:
                old = self.permit["payload"]
                same_source = all(
                    p[k] == old[k]
                    for k in (
                        "control_boot_id",
                        "pinky_boot_id",
                        "docking_execution_id",
                        "execution_id",
                    )
                )
                if same_source and p["evidence_seq"] <= old["evidence_seq"]:
                    if p["evidence_seq"] == old["evidence_seq"] and p == old:
                        return False
                    raise Rejected("out-of-order or altered duplicate permit")
                if same_source and timestamp(p["source_observed_at"]) <= timestamp(
                    old["source_observed_at"]
                ):
                    raise Rejected("source evidence did not advance")
            self.permit = deepcopy(envelope)
            self.permit_deadline = self.monotonic_clock() + min(
                self.profile["timing"]["permit_lease"],
                expiry - self.wall_clock(),
                self.profile["timing"]["permit_max_age"]
                - (self.wall_clock() - timestamp(p["source_observed_at"])),
            )
            return True
        except (Rejected, KeyError, TypeError) as exc:
            self._reject(str(exc))

    def observe(self, arm_id, positions, *, observation_seq, captured_at):
        try:
            if (
                arm_id not in self.profile["safe_positions"]
                or type(observation_seq) is not int
                or observation_seq < 0
            ):
                raise Rejected("invalid arm/observation sequence")
            pos = vector(positions)
            if not inside(pos, self.profile["joint_bounds"]):
                raise Rejected("measured arm out of joint bounds")
            self._fresh(captured_at, self.profile["timing"]["observation_max_age"])
            previous = self.observations.get(arm_id)
            if previous and (
                observation_seq <= previous["seq"]
                or timestamp(captured_at) < timestamp(previous["captured_at"])
            ):
                raise Rejected("observation did not advance")
            self.observations[arm_id] = {
                "positions": pos,
                "seq": observation_seq,
                "captured_at": captured_at,
                "received": self.monotonic_clock(),
            }
            if self.command and self.state != "SAFE_RETURNED":
                self._arms()
        except (Rejected, KeyError, TypeError) as exc:
            self._reject(str(exc))

    def _safe(self, arm_id):
        positions = self.observations[arm_id]["positions"]
        return all(
            abs(x - target) <= tol
            for x, target, tol in zip(
                positions,
                self.profile["safe_positions"][arm_id],
                self.profile["safe_tolerance"],
                strict=True,
            )
        )

    def _arms(self):
        for arm in self.profile["safe_positions"]:
            if arm not in self.observations:
                raise Rejected("both measured arm observations required")
            o = self.observations[arm]
            self._fresh(o["captured_at"], self.profile["timing"]["observation_max_age"])
            if (
                self.monotonic_clock() - o["received"]
                > self.profile["timing"]["observation_max_age"]
            ):
                raise Rejected("local arm observation expired")
        if self.command and not self._safe(self.command["payload"]["inactive_arm_id"]):
            raise Rejected("inactive arm left safe position")

    def _policy(self, phase=None):
        return self.profile["policies"][self.command["payload"]["operation"]][
            phase or self.phase
        ]

    def _entry(self, phase):
        arm = self.command["payload"]["assigned_arm_id"]
        if not inside(
            self.observations[arm]["positions"], self._policy(phase)["entry_bounds"]
        ):
            raise Rejected("measured position outside next policy entry envelope")

    def accept_command(self, envelope, *, retained=False):
        if retained or not isinstance(envelope, dict):
            self._reject("retained/non-object command")
        self._envelope(envelope, "shelf.execute.requested", retained)
        content = encoded(
            {k: v for k, v in envelope.items() if k not in ("message_id", "sent_at")}
        )
        row = self.db.execute(
            "SELECT content, state, reason, result FROM executions WHERE command_id=?",
            (envelope.get("command_id"),),
        ).fetchone()
        if row:
            if row[0] != content:
                self._reject("same command ID with different content")
            return {
                "duplicate": True,
                "state": row[1],
                "reason": row[2],
                "result": json.loads(row[3]) if row[3] else None,
            }
        if self.state not in ("READY", "SAFE_RETURNED"):
            self._reject("host not ready; explicit external reconciliation required")
        try:
            p = self._envelope(envelope, "shelf.execute.requested", retained)
            for field in (
                "resource_token",
                "permit_id",
                "assigned_arm_id",
                "inactive_arm_id",
                "order_id",
                "package_id",
                "slot_id",
                "pinky_id",
                "policy_bundle_version",
                "feature_schema_version",
                "calibration_version",
                "workcell_profile_version",
            ):
                if not nonempty(p.get(field)):
                    raise Rejected(f"command requires {field}")
            operation = p.get("operation")
            if (
                p.get("manipulation_mode") != "DEDICATED"
                or operation not in self.profile["roles"]
            ):
                raise Rejected("dedicated operation/mode required")
            if (
                p["assigned_arm_id"] != self.profile["roles"][operation]
                or {p["assigned_arm_id"], p["inactive_arm_id"]}
                != set(self.profile["roles"].values())
                or p.get("robot_ids") != [p["assigned_arm_id"]]
            ):
                raise Rejected("arm role/robot assignment mismatch")
            if (
                p.get("expected_package_ids") != [p["package_id"]]
                or p["package_id"] not in self.profile["package_ids"]
                or p["slot_id"] not in self.profile["slot_ids"]
            ):
                raise Rejected("unsupported package/slot or multiple packages")
            if p["pinky_id"] not in (
                ("pinky1", "pinky2") if operation == "INBOUND" else ("pinky3",)
            ):
                raise Rejected("wrong Pinky operation assignment")
            for field in (
                "policy_bundle_version",
                "feature_schema_version",
                "calibration_version",
                "workcell_profile_version",
            ):
                if p[field] != self.profile[field]:
                    raise Rejected(f"{field} mismatch")
            if not self.permit or self.monotonic_clock() >= self.permit_deadline:
                raise Rejected("fresh initial permit required")
            self._fresh(
                self.permit["payload"]["source_observed_at"],
                self.profile["timing"]["permit_max_age"],
            )
            if (
                min(
                    timestamp(self.permit["payload"]["expires_at"]),
                    timestamp(self.permit["expires_at"]),
                )
                <= self.wall_clock()
            ):
                raise Rejected("initial permit expired")
            self._permit_binding(self.permit["payload"], envelope)
            for field in (
                "site_id",
                "source",
                "target",
                "task_id",
                "step_id",
                "command_id",
                "execution_id",
            ):
                if self.permit[field] != envelope[field]:
                    raise Rejected("initial permit envelope binding mismatch")
            self._arms()
            if not all(self._safe(arm) for arm in self.profile["safe_positions"]):
                raise Rejected("both arms must begin safe")
            arm = p["assigned_arm_id"]
            if not inside(
                self.observations[arm]["positions"],
                self.profile["policies"][operation]["GRASPING"]["entry_bounds"],
            ):
                raise Rejected("initial policy entry envelope mismatch")
            self.db.execute(
                "INSERT INTO executions (command_id, execution_id, content, state, reason) VALUES (?, ?, ?, ?, ?)",
                (
                    envelope["command_id"],
                    envelope["execution_id"],
                    content,
                    "GRASPING",
                    "",
                ),
            )
            self.db.commit()
        except sqlite3.IntegrityError as exc:
            raise Rejected("execution ID already exists") from exc
        self.command = deepcopy(envelope)
        self.command_deadline = (
            self.monotonic_clock()
            + timestamp(envelope["expires_at"])
            - self.wall_clock()
        )
        self.state, self.reason, self.phase_revision = "GRASPING", "", 1
        self.phase_started = self.monotonic_clock()
        self.queue.clear()
        self.pending = None
        self.reset_acknowledged = False
        self.last_action_seq = self.last_request_observation = (
            self.last_confirmation_seq
        ) = -1
        self.requested_target = None
        self._persist()
        return {"duplicate": False, "state": self.state}

    def tick(self):
        if not self.command or self.state in ("SAFE_RETURNED", "HOLD", "RECONCILING"):
            return self.state
        try:
            if self.monotonic_clock() >= self.command_deadline:
                raise Rejected("execution local deadline expired")
            if not self.permit or self.monotonic_clock() >= self.permit_deadline:
                raise Rejected("permit local lease expired")
            p = self.permit["payload"]
            self._fresh(
                p["source_observed_at"], self.profile["timing"]["permit_max_age"]
            )
            if (
                min(
                    timestamp(p["expires_at"]),
                    timestamp(self.permit["expires_at"]),
                    timestamp(self.command["expires_at"]),
                )
                <= self.wall_clock()
            ):
                raise Rejected("execution/permit expired")
            self._arms()
            if (
                self.wait_started is not None
                and self.monotonic_clock() - self.wait_started
                >= self.profile["timing"]["wait_timeout"]
            ):
                raise Rejected("boundary confirmation timeout")
            if (
                self.monotonic_clock() - self.phase_started
                >= self.profile["timing"]["phase_timeout"]
            ):
                raise Rejected("phase timeout")
            if self.pending and self.monotonic_clock() >= self.pending["deadline"]:
                raise Rejected("policy reset/inference timeout")
        except Rejected as exc:
            self.hold(str(exc))
        return self.state

    def _active(self, *, allow_wait=False):
        self.tick()
        allowed = (*PHASES, *WAITS) if allow_wait else PHASES
        if self.state not in allowed:
            self._reject("motion phase is not active")

    def _request(self, kind):
        self._active(allow_wait=kind == "policy.reset")
        if self.pending or self.queue:
            self._reject("previous request/actions still pending")
        if kind == "policy.infer" and not self.reset_acknowledged:
            self._reject("current phase reset acknowledgment required")
        arm = self.command["payload"]["assigned_arm_id"]
        observation = self.observations[arm]
        if (
            kind == "policy.infer"
            and observation["seq"] <= self.last_request_observation
        ):
            self._reject("fresh observation required for inference")
        c, policy = self.command, self._policy()
        request = {
            "schema_version": PROFILE_VERSION,
            "type": kind,
            "execution_id": c["execution_id"],
            "operation": c["payload"]["operation"],
            "assigned_arm_id": arm,
            "phase": self.phase,
            "phase_revision": self.phase_revision,
            "policy_id": policy["policy_id"],
            "model_version": policy["model_version"],
            **{
                k: self.profile[k]
                for k in (
                    "policy_bundle_version",
                    "feature_schema_version",
                    "calibration_version",
                )
            },
            "request_id": uuid4().hex,
            "observation_seq": observation["seq"],
            "captured_at": observation["captured_at"],
            "expires_at": datetime.fromtimestamp(
                min(
                    timestamp(c["expires_at"]),
                    self.wall_clock() + self.profile["timing"]["inference_timeout"],
                    timestamp(observation["captured_at"])
                    + self.profile["timing"]["observation_max_age"],
                ),
                tz=datetime.fromisoformat(observation["captured_at"]).tzinfo,
            ).isoformat(),
        }
        if kind == "policy.infer":
            request["observation"] = {
                "positions": list(observation["positions"]),
                "feature_names": self.profile["feature_names"],
                "use_degrees": self.profile["use_degrees"],
            }
            self.last_request_observation = observation["seq"]
        self.pending = {
            "request": deepcopy(request),
            "deadline": self.monotonic_clock()
            + min(
                self.profile["timing"]["inference_timeout"],
                timestamp(request["expires_at"]) - self.wall_clock(),
            ),
        }
        return request

    def reset_request(self):
        self.reset_acknowledged = False
        return self._request("policy.reset")

    def infer_request(self):
        return self._request("policy.infer")

    def _reply(self, reply, kind, request_kind):
        if not isinstance(reply, dict):
            self._reject("policy reply must be an object")
        self._active(allow_wait=request_kind == "policy.reset")
        if not self.pending or self.pending["request"]["type"] != request_kind:
            self._reject("unsolicited policy reply")
        encoded(reply)
        request = self.pending["request"]
        if (
            reply.get("schema_version") != PROFILE_VERSION
            or reply.get("type") != kind
            or any(
                reply.get(k) != request[k] or type(reply.get(k)) is not type(request[k])
                for k in BINDINGS
            )
        ):
            self._reject("policy reply binding/type mismatch")
        if timestamp(reply["expires_at"]) <= self.wall_clock():
            self._reject("policy reply expired")

    def acknowledge_reset(self, reply):
        try:
            self._reply(reply, "policy.reset.ack", "policy.reset")
            self.pending = None
            self.queue.clear()
            self.last_action_seq = -1
            self.reset_acknowledged = True
        except (Rejected, KeyError, TypeError) as exc:
            self._reject(str(exc))

    def validate_candidate(self, reply):
        try:
            self._reply(reply, "action.candidate", "policy.infer")
            if (
                not self.reset_acknowledged
                or type(reply.get("action_seq")) is not int
                or reply["action_seq"] <= self.last_action_seq
            ):
                raise Rejected("candidate ordering/reset mismatch")
            actions = reply.get("actions")
            if not isinstance(actions, list) or not actions:
                raise Rejected("empty action chunk")
            encoded_reply = encoded(reply)
            if len(encoded_reply.encode()) > 60000:
                raise Rejected("candidate exceeds datagram limit")
            previous = self.observations[self.command["payload"]["assigned_arm_id"]][
                "positions"
            ]
            validated = []
            for action in actions:
                action = vector(action)
                if not inside(action, self.profile["joint_bounds"]) or any(
                    abs(a - b) > limit
                    for a, b, limit in zip(
                        action, previous, self.profile["max_step"], strict=True
                    )
                ):
                    raise Rejected("candidate exceeds physical/step limits")
                validated.append(action)
                previous = action
            self.queue.extend(
                (action, deepcopy(reply), self.pending["deadline"])
                for action in validated
            )
            self.last_action_seq = reply["action_seq"]
            self.pending = None
            return tuple(validated)
        except (Rejected, KeyError, TypeError) as exc:
            self._reject(str(exc))

    def next_action(self):
        self._active()
        if not self.queue:
            return None
        action, reply, local_deadline = self.queue[0]
        arm = self.command["payload"]["assigned_arm_id"]
        measured = self.observations[arm]["positions"]
        if (
            timestamp(reply["expires_at"]) <= self.wall_clock()
            or self.monotonic_clock() >= local_deadline
            or reply["phase_revision"] != self.phase_revision
            or any(
                abs(a - b) > limit
                for a, b, limit in zip(
                    action, measured, self.profile["max_step"], strict=True
                )
            )
        ):
            self._reject(
                "queued action expired or no longer within measured step limit"
            )
        self.queue.popleft()
        self.requested_target = action
        return action

    def phase_boundary(self):
        """Explicit supervisor stop; chunk exhaustion never calls this method."""
        self._active()
        phase = self.phase
        arm = self.command["payload"]["assigned_arm_id"]
        if not inside(
            self.observations[arm]["positions"], self._policy()["exit_bounds"]
        ):
            self._reject("phase boundary outside measured exit envelope")
        self.state = WAITS[PHASES.index(phase)]
        self.phase_revision += 1
        self.wait_started = self.monotonic_clock()
        self.boundary_observation_seq = self.observations[arm]["seq"]
        self.queue.clear()
        self.pending = None
        self.reset_acknowledged = False
        self._persist()
        return self.state

    def confirm(self, evidence):
        try:
            if not isinstance(evidence, dict):
                raise Rejected("confirmation must be an object")
            self.tick()
            if self.state not in WAITS:
                raise Rejected("not waiting for boundary confirmation")
            if not self.reset_acknowledged or self.pending:
                raise Rejected("WAIT policy reset acknowledgment required")
            for name, expected in (
                ("execution_id", self.command["execution_id"]),
                ("phase", self.phase),
                ("phase_revision", self.phase_revision),
            ):
                if evidence.get(name) != expected or type(
                    evidence.get(name)
                ) is not type(expected):
                    raise Rejected("boundary evidence binding mismatch")
            if type(evidence.get("positive")) is not bool or not evidence["positive"]:
                raise Rejected("negative/unproven boundary confirmation")
            self._fresh(
                evidence.get("captured_at"),
                self.profile["timing"]["observation_max_age"],
            )
            seq = evidence.get("observation_seq")
            arm = self.command["payload"]["assigned_arm_id"]
            observed = self.observations[arm]
            if (
                type(seq) is not int
                or seq <= max(self.last_confirmation_seq, self.boundary_observation_seq)
                or seq != observed["seq"]
                or evidence["captured_at"] != observed["captured_at"]
            ):
                raise Rejected("fresh bound one-use confirmation observation required")
            if not inside(observed["positions"], self._policy()["exit_bounds"]):
                raise Rejected("confirmed boundary outside exit envelope")
            index = WAITS.index(self.state)
            if index == 3:
                if not all(self._safe(a) for a in self.profile["safe_positions"]):
                    raise Rejected(
                        "safe return needs measured safe positions of both arms"
                    )
                self.state = "SAFE_RETURNED"
            else:
                next_phase = PHASES[index + 1]
                if (
                    evidence.get("next_policy_id")
                    != self._policy(next_phase)["policy_id"]
                ):
                    raise Rejected("next policy identity required")
                self._entry(next_phase)
                self.state = next_phase
                self.phase_revision += 1
            self.reset_acknowledged = False
            self.last_confirmation_seq = seq
            self.wait_started = None
            self.phase_started = self.monotonic_clock()
            self._persist()
            return self.state
        except (Rejected, KeyError, TypeError) as exc:
            self._reject(str(exc))

    def _event_snapshot(self):
        arms = {}
        for arm, observation in self.observations.items():
            safe = False
            try:
                self._fresh(
                    observation["captured_at"],
                    self.profile["timing"]["observation_max_age"],
                )
                safe = (
                    self._safe(arm)
                    and self.monotonic_clock() - observation["received"]
                    <= self.profile["timing"]["observation_max_age"]
                )
            except Rejected:
                pass
            arms[arm] = {
                "positions": list(observation["positions"]),
                "observation_seq": observation["seq"],
                "captured_at": observation["captured_at"],
                "safe": safe,
            }
        command = self.command
        payload = deepcopy(command["payload"])
        terminal = self.state == "SAFE_RETURNED"
        kind = (
            "completed"
            if terminal
            else "held"
            if self.state in ("HOLD", "RECONCILING")
            else "progress"
        )
        payload.update(
            state=self.state,
            reason=self.reason,
            phase=self.phase,
            phase_revision=self.phase_revision,
            host_boot_id=self.host_boot_id,
            arms=arms,
            active_arm_safe=arms.get(payload["assigned_arm_id"], {}).get("safe", False),
            inactive_arm_safe=arms.get(payload["inactive_arm_id"], {}).get(
                "safe", False
            ),
            source_observed_at=max(
                (o["captured_at"] for o in arms.values()), key=timestamp, default=None
            ),
            location=("slot" if payload["operation"] == "INBOUND" else "pinky")
            if terminal
            else "UNKNOWN",
            source_mock_only=True,
            departure_allowed=False,
        )
        event = {k: deepcopy(v) for k, v in command.items() if k != "payload"}
        event.update(
            message_id=uuid4().hex,
            boot_id=self.host_boot_id,
            observed_at=payload["source_observed_at"],
            type=f"shelf.execute.{kind}",
            source=self.profile["workcell_id"],
            target=self.profile["control_id"],
            sent_at=datetime.fromtimestamp(
                self.wall_clock(), tz=datetime.fromisoformat(command["sent_at"]).tzinfo
            ).isoformat(),
            payload=payload,
        )
        return event

    def result_event(self):
        """Return the durable terminal snapshot, or current source/mock progress."""
        if not self.command:
            raise Rejected("no active command for result event")
        if self.state in ("SAFE_RETURNED", "HOLD"):
            row = self.db.execute(
                "SELECT result FROM executions WHERE command_id=?",
                (self.command["command_id"],),
            ).fetchone()
            if row and row[0]:
                return json.loads(row[0])
        return self._event_snapshot()

    def status(self):
        return {
            "state": self.state,
            "phase": self.phase,
            "phase_revision": self.phase_revision,
            "execution_id": self.command["execution_id"] if self.command else None,
            "reason": self.reason,
            "arms_held": self.state in (*WAITS, "HOLD", "RECONCILING"),
            "departure_allowed": False,
            "source_mock_only": True,
        }
