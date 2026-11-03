"""Source/mock contract checks only: no trained model or physical robot."""

import json
from copy import deepcopy
from datetime import UTC, datetime, timedelta

import pytest

from dedicated_policy import (
    BINDINGS,
    MAX_DATAGRAM_BYTES,
    DatagramError,
    PolicyRouter,
    decode_datagram,
    encode_datagram,
)

NOW = datetime(2026, 11, 3, tzinfo=UTC).timestamp()
FEATURES = [f"synthetic_joint_{index}" for index in range(6)]
METADATA = {
    "operation": "INBOUND",
    "assigned_arm_id": "omx1",
    "phase": "GRASPING",
    "model_version": "synthetic-model-v1",
    "policy_bundle_version": "synthetic-bundle-v1",
    "feature_schema_version": "synthetic-features-v1",
    "calibration_version": "synthetic-calibration-v1",
    "feature_names": FEATURES,
    "use_degrees": True,
}


def request(kind="policy.reset", **overrides):
    result = {
        "schema_version": "omx.dedicated/1",
        "type": kind,
        **{
            key: value
            for key, value in METADATA.items()
            if key not in {"feature_names", "use_degrees"}
        },
        "execution_id": "synthetic-execution-1",
        "phase_revision": 0,
        "policy_id": "synthetic-grasp",
        "request_id": "synthetic-request-1",
        "observation_seq": 1,
        "captured_at": "2026-11-03T00:00:00+00:00",
        "expires_at": "2026-11-03T00:00:05+00:00",
    }
    if kind == "policy.infer":
        result["observation"] = {
            "positions": [0.0] * 6,
            "feature_names": FEATURES.copy(),
            "use_degrees": True,
        }
    return result | overrides


class SyntheticPolicy:
    def __init__(self):
        self.resets = 0
        self.observations = []
        self.actions = [[0.25] * 6, [0.5] * 6]
        self.fail_reset = False
        self.fail_infer = False

    def reset(self):
        self.resets += 1
        if self.fail_reset:
            raise RuntimeError("synthetic reset error")

    def infer(self, observation):
        self.observations.append(observation)
        if self.fail_infer:
            raise RuntimeError("synthetic inference error")
        return self.actions


@pytest.fixture
def configured():
    provider = SyntheticPolicy()
    return PolicyRouter(
        {"synthetic-grasp": provider}, {"synthetic-grasp": METADATA}
    ), provider


def test_image_input_errors_are_distinct_from_model_failure():
    from observations import ImageNotReady

    class WaitingPolicy(SyntheticPolicy):
        def infer(self, observation):
            raise ImageNotReady("synthetic missing current image")

    provider = WaitingPolicy()
    router = PolicyRouter(
        {"synthetic-grasp": provider},
        {
            "synthetic-grasp": METADATA
            | {"cameras": {"side": {"shape": [3, 2, 2], "fps": 30}}}
        },
    )
    assert router.handle(request(), now=NOW)["type"] == "policy.reset.ack"
    infer = request("policy.infer")
    assert router.handle(infer, now=NOW)["code"] == "invalid_observation"
    infer["observation"].update(
        captured_at=infer["captured_at"],
        images={
            "side": {
                "source": "host1",
                "camera": "side",
                "stream": "rgb",
                "session": "00000000-0000-0000-0000-000000000001",
                "frame_id": 1,
                "captured_ns": int(NOW * 1e9),
                "observation_group": "synthetic",
            }
        },
    )
    infer["observation"]["captured_at"] = "2026-11-03T00:00:01+00:00"
    assert router.handle(infer, now=NOW)["code"] == "invalid_observation"
    infer["observation"]["captured_at"] = infer["captured_at"]
    assert router.handle(infer, now=NOW)["code"] == "input_not_ready"
    assert router.handle(infer, now=NOW)["code"] == "reset_required"


def test_requires_configured_model_and_reset(configured):
    router, provider = configured
    assert (
        PolicyRouter({}, {}).handle(request(), now=NOW)["code"]
        == "model_not_configured"
    )
    assert router.handle(request("policy.infer"), now=NOW)["code"] == "reset_required"
    assert provider.observations == []


def test_reset_echo_and_idempotence_keep_sequence(configured):
    router, provider = configured
    reset = request()
    ack = router.handle(reset, now=NOW)
    assert ack["type"] == "policy.reset.ack"
    assert all(ack[key] == reset[key] for key in BINDINGS)
    infer = request("policy.infer")
    candidate = router.handle(infer, now=NOW)
    assert candidate["type"] == "action.candidate"
    assert candidate["action_seq"] == 1
    assert candidate["actions"] == provider.actions
    assert all(candidate[key] == infer[key] for key in BINDINGS)
    assert router.handle(reset, now=NOW) == ack
    assert provider.resets == 1
    assert router.handle(infer, now=NOW)["code"] == "stale_observation"
    assert (
        router.handle(request("policy.infer", observation_seq=2), now=NOW)["action_seq"]
        == 2
    )


def test_reset_revision_cannot_roll_back_or_rebind(configured):
    router, provider = configured
    assert (
        router.handle(request(phase_revision=2), now=NOW)["type"] == "policy.reset.ack"
    )
    assert router.handle(request(phase_revision=1), now=NOW)["code"] == "stale_reset"
    metadata = deepcopy(METADATA)
    metadata["phase"] = "TRANSFERRING"
    router.metadata["synthetic-transfer"] = metadata
    router.policies["synthetic-transfer"] = SyntheticPolicy()
    changed = request(
        phase_revision=2, policy_id="synthetic-transfer", phase="TRANSFERRING"
    )
    assert router.handle(changed, now=NOW)["code"] == "reset_conflict"
    changed["phase_revision"] = 3
    assert router.handle(changed, now=NOW)["type"] == "policy.reset.ack"
    assert (
        router.handle(request("policy.infer", phase_revision=2), now=NOW)["code"]
        == "reset_required"
    )
    assert provider.observations == []


def test_wait_boundary_higher_revision_resets_current_policy(configured):
    router, provider = configured
    router.handle(request(phase_revision=1), now=NOW)
    router.handle(request("policy.infer", phase_revision=1), now=NOW)
    wait_reset = request(phase_revision=2, request_id="wait-reset")
    assert router.handle(wait_reset, now=NOW)["type"] == "policy.reset.ack"
    assert provider.resets == 2
    assert router.handle(wait_reset, now=NOW)["type"] == "policy.reset.ack"
    assert provider.resets == 2
    assert (
        router.handle(request("policy.infer", phase_revision=1), now=NOW)["code"]
        == "reset_required"
    )
    next_metadata = METADATA | {"phase": "TRANSFERRING"}
    next_provider = SyntheticPolicy()
    router.metadata["synthetic-transfer"] = next_metadata
    router.policies["synthetic-transfer"] = next_provider
    next_reset = request(
        phase_revision=3, phase="TRANSFERRING", policy_id="synthetic-transfer"
    )
    assert router.handle(next_reset, now=NOW)["type"] == "policy.reset.ack"
    assert next_provider.resets == 1


@pytest.mark.parametrize("operation", ["INBOUND", "OUTBOUND"])
@pytest.mark.parametrize(
    "phase", ["GRASPING", "TRANSFERRING", "RELEASING", "RETURNING"]
)
def test_all_operation_phase_registrations_return_candidates(operation, phase):
    arm = "omx1" if operation == "INBOUND" else "omx2"
    metadata = METADATA | {
        "operation": operation,
        "assigned_arm_id": arm,
        "phase": phase,
    }
    router = PolicyRouter(
        {"synthetic-grasp": SyntheticPolicy()}, {"synthetic-grasp": metadata}
    )
    bindings = {"operation": operation, "assigned_arm_id": arm, "phase": phase}
    assert router.handle(request(**bindings), now=NOW)["type"] == "policy.reset.ack"
    assert (
        router.handle(request("policy.infer", **bindings), now=NOW)["type"]
        == "action.candidate"
    )


def test_old_execution_cannot_reactivate(configured):
    router, provider = configured
    router.handle(request(), now=NOW)
    router.handle(request(execution_id="synthetic-execution-2"), now=NOW)
    assert router.handle(request(), now=NOW)["code"] == "stale_execution"
    assert router.handle(request("policy.infer"), now=NOW)["code"] == "stale_execution"
    assert provider.resets == 2


def test_session_capacity_preserves_active_and_retired_executions():
    provider = SyntheticPolicy()
    router = PolicyRouter(
        {"synthetic-grasp": provider}, {"synthetic-grasp": METADATA}, max_sessions=2
    )
    router.handle(request(), now=NOW)
    router.handle(request(execution_id="synthetic-execution-2"), now=NOW)
    assert (
        router.handle(request(execution_id="synthetic-execution-3"), now=NOW)["code"]
        == "session_capacity_exceeded"
    )
    assert router.handle(request(), now=NOW)["code"] == "stale_execution"
    candidate = router.handle(
        request("policy.infer", execution_id="synthetic-execution-2"), now=NOW
    )
    assert candidate["type"] == "action.candidate"
    assert provider.resets == 2


def test_timestamp_regression_rejected_despite_increasing_sequence(configured):
    router, _ = configured
    router.handle(request(), now=NOW)
    router.handle(
        request("policy.infer", captured_at="2026-11-03T00:00:01+00:00"), now=NOW + 1
    )
    assert (
        router.handle(request("policy.infer", observation_seq=2), now=NOW + 1)["code"]
        == "stale_observation"
    )


def test_same_clock_fraction_rounded_to_wire_precision_is_accepted(configured):
    router, _ = configured
    now = 1800000000.1299999
    captured = datetime.fromtimestamp(now, UTC).isoformat()
    assert datetime.fromisoformat(captured).timestamp() > now
    timestamps = {
        "captured_at": captured,
        "expires_at": datetime.fromtimestamp(now + 5, UTC).isoformat(),
    }
    assert router.handle(request(**timestamps), now=now)["type"] == "policy.reset.ack"
    candidate = router.handle(request("policy.infer", **timestamps), now=now)
    assert candidate["type"] == "action.candidate"
    assert candidate["captured_at"] == captured


def test_one_wire_microsecond_in_future_is_still_rejected(configured):
    router, provider = configured
    now = 1800000000.1299999
    captured = datetime.fromtimestamp(now, UTC) + timedelta(microseconds=1)
    timestamps = {
        "captured_at": captured.isoformat(),
        "expires_at": datetime.fromtimestamp(now + 5, UTC).isoformat(),
    }
    assert router.handle(request(**timestamps), now=now)["code"] == "future_observation"
    assert provider.resets == 0


def test_wire_rounding_does_not_extend_execution_deadline(configured, monkeypatch):
    router, _ = configured
    now = 1800000000.1300004
    expires = datetime.fromtimestamp(now + 1, UTC).isoformat()
    timestamps = {
        "captured_at": datetime.fromtimestamp(now, UTC).isoformat(),
        "expires_at": expires,
    }
    router.handle(request(**timestamps), now=now)
    ticks = iter([0.0, 0.9999998])
    monkeypatch.setattr("dedicated_policy.time.monotonic", lambda: next(ticks))
    assert (
        router.handle(request("policy.infer", **timestamps), now=now)["code"]
        == "deadline_exceeded"
    )


@pytest.mark.parametrize("missing", BINDINGS)
def test_all_binding_fields_required(configured, missing):
    router, provider = configured
    reset = request()
    del reset[missing]
    assert router.handle(reset, now=NOW)["code"] == "invalid_request"
    assert provider.resets == 0


@pytest.mark.parametrize(
    "field,value",
    [
        ("model_version", "wrong"),
        ("operation", "OUTBOUND"),
        ("assigned_arm_id", "omx2"),
        ("calibration_version", "wrong"),
        ("feature_schema_version", "wrong"),
        ("policy_bundle_version", "wrong"),
    ],
)
def test_metadata_bindings_are_enforced(configured, field, value):
    router, provider = configured
    reply = router.handle(request(**{field: value}), now=NOW)
    assert reply["code"] == "metadata_mismatch"
    assert reply[field] == value
    assert provider.resets == 0


@pytest.mark.parametrize(
    "override,code",
    [
        ({"expires_at": "2026-11-03T00:00:00+00:00"}, "expired_observation"),
        ({"captured_at": "2026-11-03T00:00:01+00:00"}, "future_observation"),
        ({"captured_at": "2026-11-03T00:00:00"}, "invalid_timestamp"),
        ({"phase_revision": True}, "invalid_request"),
        ({"observation_seq": -1}, "invalid_request"),
        ({"schema_version": "other"}, "invalid_request"),
        ({"type": []}, "invalid_request"),
        ({"type": {}}, "invalid_request"),
        ({"phase": "WAIT_GRASP"}, "invalid_request"),
    ],
)
def test_invalid_or_expired_requests_never_call_provider(configured, override, code):
    router, provider = configured
    assert router.handle(request(**override), now=NOW)["code"] == code
    assert provider.resets == 0


@pytest.mark.parametrize(
    "field,value",
    [
        ("positions", [0.0] * 5),
        ("positions", [True] * 6),
        ("positions", [float("nan")] * 6),
        ("feature_names", list(reversed(FEATURES))),
        ("feature_names", ["duplicate"] * 6),
        ("use_degrees", 1),
        ("use_degrees", False),
    ],
)
def test_bad_observation_rejected(configured, field, value):
    router, provider = configured
    router.handle(request(), now=NOW)
    infer = request("policy.infer")
    infer["observation"][field] = value
    reply = router.handle(infer, now=NOW)
    assert reply["type"] == "inference.error"
    assert reply["code"] in {"invalid_request", "invalid_observation"}
    assert reply["execution_id"] == infer["execution_id"]
    assert reply["request_id"] == infer["request_id"]
    assert provider.observations == []


@pytest.mark.parametrize(
    "actions", [[], [[1.0] * 5], [[True] * 6], [[float("inf")] * 6]]
)
def test_invalid_candidates_require_new_reset(configured, actions):
    router, provider = configured
    router.handle(request(), now=NOW)
    provider.actions = actions
    assert router.handle(request("policy.infer"), now=NOW)["code"] == "invalid_actions"
    assert (
        router.handle(request("policy.infer", observation_seq=2), now=NOW)["code"]
        == "reset_required"
    )


@pytest.mark.parametrize(
    "actions",
    [
        [[0.123456789] * 6] * 1000,
        [[0] * 6] * MAX_DATAGRAM_BYTES,
    ],
)
def test_candidate_chunk_is_bounded_by_wire_budget(configured, actions):
    router, provider = configured
    router.handle(request(), now=NOW)
    provider.actions = actions
    reply = router.handle(request("policy.infer"), now=NOW)
    assert reply["code"] == "response_too_large"
    assert (
        router.handle(request("policy.infer", observation_seq=2), now=NOW)["code"]
        == "reset_required"
    )


def test_provider_failures_are_explicit(configured):
    router, provider = configured
    provider.fail_reset = True
    assert router.handle(request(), now=NOW)["code"] == "reset_failed"
    assert router.handle(request("policy.infer"), now=NOW)["code"] == "reset_required"
    provider.fail_reset = False
    router.handle(request(phase_revision=1), now=NOW)
    provider.fail_infer = True
    assert (
        router.handle(request("policy.infer", phase_revision=1), now=NOW)["code"]
        == "inference_failed"
    )


def test_deadline_after_provider_discards_candidate(configured, monkeypatch):
    router, provider = configured
    router.handle(request(), now=NOW)
    ticks = iter([0.0, 6.0])
    monkeypatch.setattr("dedicated_policy.time.monotonic", lambda: next(ticks))
    assert (
        router.handle(request("policy.infer"), now=NOW)["code"] == "deadline_exceeded"
    )
    assert len(provider.observations) == 1


def test_serialization_elapsed_time_also_counts_toward_deadline(
    configured, monkeypatch
):
    router, _ = configured
    router.handle(request(), now=NOW)
    ticks = iter([0.0, 0.0, 6.0])
    monkeypatch.setattr("dedicated_policy.time.monotonic", lambda: next(ticks))
    assert (
        router.handle(request("policy.infer"), now=NOW)["code"] == "deadline_exceeded"
    )


def test_reset_deadline_does_not_unlock_inference(configured, monkeypatch):
    router, provider = configured
    ticks = iter([0.0, 6.0])
    with monkeypatch.context() as patched:
        patched.setattr("dedicated_policy.time.monotonic", lambda: next(ticks))
        assert router.handle(request(), now=NOW)["code"] == "deadline_exceeded"
    assert provider.resets == 1
    assert router.handle(request("policy.infer"), now=NOW)["code"] == "reset_required"


def test_reset_ack_must_fit_datagram_before_mutating_provider(configured):
    router, provider = configured
    reset = request()
    reset["request_id"] += "x" * (MAX_DATAGRAM_BYTES - len(encode_datagram(reset)))
    assert len(encode_datagram(reset)) == MAX_DATAGRAM_BYTES
    reply = decode_datagram(router.handle_datagram(encode_datagram(reset), now=NOW))
    assert reply["code"] == "response_too_large"
    assert provider.resets == 0


@pytest.mark.parametrize(
    "data",
    [
        b'{"x":1,"x":2}',
        b'{"nested":{"x":1,"x":2}}',
        b'{"x":NaN}',
        b'{"x":Infinity}',
        b'{"x":1e999}',
        b"[]",
        b"\xff",
        b"{",
        b" " * (MAX_DATAGRAM_BYTES + 1),
    ],
)
def test_strict_datagram_rejects_malformed_json(data):
    with pytest.raises(DatagramError):
        decode_datagram(data)


def test_datagram_round_trip_and_error(configured):
    router, _ = configured
    reply = decode_datagram(router.handle_datagram(encode_datagram(request()), now=NOW))
    assert reply["type"] == "policy.reset.ack"
    assert reply["captured_at"] == request()["captured_at"]
    assert (
        decode_datagram(router.handle_datagram(b"{", now=NOW))["code"]
        == "invalid_datagram"
    )
    with pytest.raises(DatagramError):
        encode_datagram({"value": float("nan")})
    with pytest.raises(DatagramError):
        encode_datagram({"value": "x" * MAX_DATAGRAM_BYTES})
    assert json.loads(encode_datagram(reply)) == reply


@pytest.mark.parametrize(
    "field,value",
    [
        ("feature_names", FEATURES[:5]),
        ("feature_names", ["duplicate"] * 6),
        ("use_degrees", "degrees"),
        ("model_version", ""),
    ],
)
def test_registry_metadata_requires_explicit_configuration(field, value):
    with pytest.raises(ValueError):
        PolicyRouter({}, {"synthetic-grasp": METADATA | {field: value}})
