"""Burst requests over real MQTT; concurrent movement with disjoint reservations."""

import json
import time
from collections import Counter
from copy import deepcopy
from itertools import pairwise

import pytest
from test_fleet_mqtt import fleet_mqtt  # noqa: F401


class Trace:
    def __init__(self, env, monkeypatch):
        self.env = env
        self.started = time.monotonic()
        self.commands = []
        self.completed = []
        self.reasons = set()
        self.max_active = 0
        self.max_moving = 0
        self.max_assigned = 0
        self.max_queued = 0
        dispatch = env.control.dispatched

        def measured_dispatch(request_id, message):
            # This assertion concerns emitted physical commands, not merely tasks.
            previous = set(env.control.data["resources"])
            dispatch(request_id, message)
            data = env.control.data
            owned = {k for k, v in data["resources"].items() if v == request_id}
            assert not previous & owned
            route = message["payload"].get("route", [])
            assert all(
                data["resources"]["edge:" + "-".join(sorted(edge))] == request_id
                for edge in pairwise(route)
            )
            if route:
                assert not set(route) & env.control.occupied(message["target"])
            self.commands.append(
                {
                    "elapsed_s": round(time.monotonic() - self.started, 3),
                    "request_id": request_id,
                    "target": message["target"],
                    "type": message["type"],
                    "command_id": message["command_id"],
                    "route": route,
                    "observed_nodes": {
                        r: p["state"].get("node")
                        for r, p in env.control.observed.items()
                    },
                    "resources": deepcopy(data["resources"]),
                }
            )

        monkeypatch.setattr(env.control, "dispatched", measured_dispatch)

    def sample(self):
        data = self.env.control.data
        active = [c.robot for c in self.env.clients if c.journal.data["active"]]
        moving = [
            c.robot for c in self.env.clients if not c.driver.observe()["stopped"]
        ]
        assigned = [
            t["robot"]
            for t in data["tasks"].values()
            if t["status"] != "COMPLETE" and "robot" in t
        ]
        parking = [
            t["parking"]
            for t in data["tasks"].values()
            if t["status"] != "COMPLETE" and "parking" in t
        ]
        assert len(assigned) == len(set(assigned)), "robot assigned twice"
        assert len(parking) == len(set(parking)), "return bay reserved twice"
        assert len(active) <= len(self.env.clients)
        assert len(moving) <= len(self.env.clients)
        self.max_active = max(self.max_active, len(active))
        self.max_moving = max(self.max_moving, len(moving))
        self.max_assigned = max(self.max_assigned, len(assigned))
        self.max_queued = max(
            self.max_queued,
            sum(t["stage"] == "queued" for t in data["tasks"].values()),
        )
        for key, task in data["tasks"].items():
            if task["reason"]:
                self.reasons.add((key, task["reason"]))
            if task["status"] == "COMPLETE" and key not in self.completed:
                self.completed.append(key)
        return data

    def until(self, predicate, timeout=90):
        self.env.pump(lambda: predicate(self.sample()), timeout=timeout)

    def save(self):
        data = self.sample()
        output = {
            "result": "passed",
            "scope": "real loopback MQTT; actual Pinky driver/line logic; virtual device I/O, Nav2, OMX",
            "timing": "accelerated sensor ticks; elapsed values are test wall time",
            "elapsed_s": round(time.monotonic() - self.started, 3),
            "registered_robots": sorted(c.robot for c in self.env.clients),
            "max_simultaneous_active_pinkies": self.max_active,
            "max_simultaneous_moving_pinkies": self.max_moving,
            "max_assigned_unfinished_tasks": self.max_assigned,
            "max_queued_requests": self.max_queued,
            "parallel_motion_supported": True,
            "completion_order": self.completed,
            "waiting_reasons": sorted(self.reasons),
            "commands_by_robot": dict(Counter(c["target"] for c in self.commands)),
            "commands": self.commands,
            "tasks": data["tasks"],
            "field_versions": data["field_versions"],
            "limits": [
                "Passed segments release after the following node observation; OMX host operations remain serial.",
                "Finite workload only; unbounded arrival starvation is not proven absent.",
                "No physical collision, radio, braking or Nav2 obstacle validation.",
            ],
        }
        (self.env.tmp_path / "concurrent-summary.json").write_text(
            json.dumps(output, indent=2, ensure_ascii=False) + "\n"
        )


def test_four_simultaneous_inbound_requests_share_routes_without_double_assignment(
    fleet_mqtt,  # noqa: F811 - shared pytest fixture
    monkeypatch,
):
    env = fleet_mqtt
    trace = Trace(env, monkeypatch)
    keys = [
        env.submit(p, "inbound") for p in ("small-1", "small-2", "small-3", "large")
    ]
    trace.sample()
    assert trace.max_queued == 4
    # Repeated customer delivery is idempotent; conflicting contents are rejected.
    env.submit("small-1", "inbound")
    assert len(env.control.data["tasks"]) == 4
    with pytest.raises(ValueError, match="request ID reused"):
        env.control.submit(
            {
                "request_id": keys[0],
                "operation": "inbound",
                "package_id": "different-box",
                "goal": None,
            }
        )
    trace.until(
        lambda data: all(data["tasks"][k]["status"] == "COMPLETE" for k in keys)
    )
    assert trace.max_active >= 2 and trace.max_moving >= 2
    assert set(trace.completed) == set(keys)
    assert (
        sum(s["state"] == "occupied" for s in env.control.data["slots"].values()) == 4
    )
    assert env.control.data["commands"] == {}
    assert env.control.data["resources"] == {}
    trace.save()


@pytest.mark.parametrize("first_package", ["small-first", "large"])
def test_two_inbounds_follow_short_route_after_clearance(
    fleet_mqtt,  # noqa: F811 - shared pytest fixture
    monkeypatch,
    first_package,
):
    env = fleet_mqtt
    trace = Trace(env, monkeypatch)
    first = env.submit(first_package, "inbound")
    second = env.submit("second-package", "inbound")
    trace.until(
        lambda data: all(
            data["tasks"][key]["status"] == "COMPLETE" for key in (first, second)
        )
    )
    approach = []
    for command in trace.commands:
        if command["request_id"] != second or not command["route"]:
            continue
        route = command["route"]
        if route[0] == "N05":
            assert command["observed_nodes"]["pinky1"] == "N23"
            assert command["resources"]["edge:N07-N23"] == first
        approach.extend(route if not approach else route[1:])
        if route[-1] == "N01":
            break
    assert approach == ["N17", "N15", "N05", "N07", "N18", "N01"]
    assert trace.max_moving >= 2
    assert env.control.data["commands"] == {}
    assert env.control.data["resources"] == {}
    trace.save()


def test_customer_receipt_wait_does_not_block_logistics_but_cart_is_not_reassigned(
    fleet_mqtt,  # noqa: F811 - shared pytest fixture
    monkeypatch,
):
    env = fleet_mqtt
    trace = Trace(env, monkeypatch)
    for package in ("stock-1", "stock-2"):
        key = env.submit(package, "inbound")
        trace.until(lambda data, key=key: data["tasks"][key]["status"] == "COMPLETE")
    first = env.submit("stock-1", "outbound")
    inbound = env.submit("new-stock", "inbound")
    second = env.submit("stock-2", "outbound")
    trace.sample()
    trace.until(
        lambda data: (
            data["tasks"][first]["stage"] == "await_receipt"
            and data["tasks"][inbound]["status"] == "COMPLETE"
            and data["tasks"][second]["reason"]
        )
    )
    second_task = env.control.data["tasks"][second]
    assert second_task["stage"] == "queued"
    assert "robot" not in second_task
    assert "no ready smartcart" in second_task["reason"]
    assert env.control.data["tasks"][first]["robot"] == "pinky3"
    # The stopped customer-side cart retains its task, while logistics can finish.
    assert trace.max_assigned >= 2
    env.control.receipt(first)
    trace.until(lambda data: data["tasks"][first]["status"] == "COMPLETE")
    trace.until(lambda data: data["tasks"][second]["stage"] == "await_receipt")
    env.control.receipt(second)
    trace.until(lambda data: data["tasks"][second]["status"] == "COMPLETE")
    assert trace.max_active >= 2 and trace.max_moving >= 2
    trace.save()


def test_unavailable_first_request_does_not_starve_feasible_requests(
    fleet_mqtt,  # noqa: F811 - shared pytest fixture
    monkeypatch,
):
    env = fleet_mqtt
    trace = Trace(env, monkeypatch)
    with pytest.raises(ValueError, match="선반에 등록"):
        env.submit("not-in-stock", "outbound")
    keys = [env.submit(p, "inbound") for p in ("available-1", "available-2")]
    trace.sample()
    trace.until(
        lambda data: all(data["tasks"][k]["status"] == "COMPLETE" for k in keys)
    )
    assert "outbound-not-in-stock" not in env.control.data["tasks"]
    assert env.control.data["commands"] == {}
    assert env.control.data["resources"] == {}
    assert trace.max_active >= 2 and trace.max_moving >= 2
    trace.save()


def test_two_logistics_and_cart_move_concurrently_then_finish(
    fleet_mqtt,  # noqa: F811 - shared pytest fixture
    monkeypatch,
):
    env = fleet_mqtt
    stocked = env.submit("ready-for-customer", "inbound")
    env.pump(lambda: env.control.data["tasks"][stocked]["status"] == "COMPLETE")
    trace = Trace(env, monkeypatch)
    inbound = [env.submit(p, "inbound") for p in ("parallel-a", "parallel-b")]
    outbound = env.submit("ready-for-customer", "outbound")
    trace.until(
        lambda data: (
            data["tasks"][outbound]["stage"] == "await_receipt"
            and all(data["tasks"][key]["status"] == "COMPLETE" for key in inbound)
        )
    )
    assert trace.max_active == 3 and trace.max_moving == 3
    env.control.receipt(outbound)
    trace.until(lambda data: data["tasks"][outbound]["status"] == "COMPLETE")
    assert env.control.data["commands"] == {} and env.control.data["resources"] == {}
    trace.save()
