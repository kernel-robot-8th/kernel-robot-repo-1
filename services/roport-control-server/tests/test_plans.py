"""Shared execution: legacy presets, individual stages and request-generated routes."""

# fixture imports are intentionally registered in this test module.
# ruff: noqa: F811
import time
from copy import deepcopy

import pytest
from mock_pinky_trial import FakePinky
from test_pinky_trial import (  # noqa: F401
    MemoryTransport,
    control_memory,
    delivery,
    profile,
    single_profile,
)
from test_trial_line_driver import rig_module  # noqa: F401
from test_trial_web import HEADERS, fence, local_owner  # noqa: F401

from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.mqtt.messages import topic
from roport_control_server.adapters.mqtt.mqtt import MqttConfig, MqttTransport
from roport_control_server.adapters.mqtt.pinky_trial import PinkyTrialClient
from roport_control_server.executor.pinky_trial import TrialControl
from roport_control_server.executor.plans import (
    delivery_plan,
    legacy_plan,
    route_payload,
    shortest_route,
    stop_nodes,
    switch_payload,
    trial_presets,
    validate_plan,
)


def plan(*steps):
    return {
        "version": 1,
        "name": "구간 시험",
        "assumptions": ["선행 입고는 시험 제외"],
        "steps": list(steps),
    }


def step(kind, role, payload, sid="segment"):
    return {
        "id": sid,
        "kind": kind,
        "role": role,
        "label": sid,
        "payload": deepcopy(payload),
    }


def request(profile, role="auto"):
    return {
        "request_id": "order-1",
        "package_id": profile["package_id"],
        "quantity": 1,
        "role": role,
        "pickup": "N11",
        "destination": "N12",
        "pickup_heading_rad": 0.0017453292519943296,
        "nav2": True,
    }


@pytest.mark.parametrize(
    "fault",
    [
        "gap",
        "mode",
        "duplicate",
        "unknown",
        "bay",
        "maneuver",
        "speed",
        "nan",
        "extra",
        "role",
        "empty",
    ],
)
def test_invalid_plan_is_rejected_before_mutation(control_memory, fault):
    c = control_memory
    p = legacy_plan(c.config)
    if fault == "gap":
        p["steps"] = [p["steps"][0], {**p["steps"][-3], "role": "logistics"}]
    elif fault == "mode":
        p["steps"] = [
            p["steps"][0],
            step("navigate", "logistics", c.config["nav_goal"], "nav"),
        ]
    elif fault == "duplicate":
        p["steps"].append(p["steps"][0])
    elif fault == "unknown":
        p["steps"][0]["payload"]["route"][1] = "N99"
    elif fault == "bay":
        p["steps"] = [
            step(
                "route", "logistics", route_payload(c.config, ["N14", "N16", "N14"], 0)
            )
        ]
    elif fault == "maneuver":
        p["steps"][0]["payload"]["start_maneuver"] = "turn_back"
    elif fault == "speed":
        p["steps"][0]["payload"]["speed_limit_mps"] = 0.2
    elif fault == "nan":
        p["steps"][0]["payload"]["start_heading_rad"] = float("nan")
    elif fault == "extra":
        p["override_stop"] = True
    elif fault == "role":
        p["steps"][0]["role"] = "unknown"
    else:
        p["steps"] = []
    before = deepcopy(c.state)
    with pytest.raises(ValueError):
        c.configure(p)
    assert c.state == before and c.transport.sent == []


def test_shortest_path_and_occupied_nodes(profile):
    route = shortest_route(profile["nodes"], "H1", "N11")
    assert route == ["H1", "N09", "N24", "N19", "N11"]
    with pytest.raises(ValueError):
        shortest_route(profile["nodes"], "H1", "N11", {"N19"})
    with pytest.raises(ValueError):
        shortest_route(profile["nodes"], "unknown", "N11")


@pytest.mark.parametrize("speed", [0.01, 0.014, 0.0801, 0.1])
def test_unsupported_speed_rejected_before_state_change(control_memory, speed):
    c = control_memory
    p = legacy_plan(c.config)
    p["steps"][0]["payload"]["speed_limit_mps"] = speed
    before = deepcopy(c.state)
    with pytest.raises(ValueError, match="local trial speed must be 0.015..0.08"):
        c.configure(p)
    assert c.state == before and c.transport.sent == []


@pytest.mark.parametrize("speed", [0.015, 0.06, 0.08])
def test_route_speed_reaches_unchanged_pinky_driver(control_memory, rig_module, speed):
    c = control_memory
    p = legacy_plan(c.config)
    p["steps"][0]["payload"]["speed_limit_mps"] = speed
    c.configure(p)
    c.start()
    assert c.state["active"]["payload"]["speed_limit_mps"] == speed
    rig_module.Rig().logic.validate_action(c.state["active"])


def test_request_assignment_and_replanning(control_memory):
    c = control_memory
    p = c.plan_delivery(request(c.config))
    assert p["steps"][0]["role"] == "smartcart"
    assert p["steps"][0]["payload"]["route"] == ["H1", "N09", "N24", "N19", "N11"]
    assert all(
        s["payload"]["speed_limit_mps"] == 0.06
        for s in p["steps"]
        if s["kind"] == "route"
    )
    statuses = deepcopy(c.state["statuses"])
    statuses["pinky3"]["node_id"] = "N11"
    p = delivery_plan(c.config, request(c.config), statuses)
    assert p["steps"][0]["kind"] == "gate"
    assert p["steps"][0]["role"] == "smartcart"
    p = delivery_plan(c.config, request(c.config, "logistics"), c.state["statuses"])
    assert p["steps"][0]["payload"]["route"][0] == "N16"


def test_hardware_confirmation_is_per_plan(control_memory):
    c = control_memory
    c.config["mode"] = "hardware"
    p = plan(step("route", "logistics", c.config["routes"]["to_loading"]))
    with pytest.raises(ValueError, match="confirm this plan"):
        c.configure(p)
    c.configure(p, field_confirmed=True)
    assert c.state["field_confirmation"]["confirmed"]
    c.start()
    with pytest.raises(ValueError, match="frozen"):
        c.configure(p, field_confirmed=True)


@pytest.mark.parametrize(
    "problem", ["node", "heading", "occupied", "unknown_other", "stale", "fault"]
)
def test_partial_start_fails_closed(control_memory, problem):
    c = control_memory
    p = legacy_plan(c.config)
    c.configure(p)
    s = deepcopy(c.state)
    if problem == "node":
        s["statuses"]["pinky1"]["node_id"] = "N19"
    if problem == "heading":
        s["statuses"]["pinky1"]["heading_rad"] = 1.0
    if problem == "occupied":
        s["statuses"]["pinky3"]["node_id"] = "N14"
    if problem == "unknown_other":
        s["statuses"]["pinky3"]["node_id"] = None
    if problem == "stale":
        c.heard["pinky1"] -= 4
    if problem == "fault":
        s["statuses"]["pinky1"]["fault"] = "line_not_found"
    c.journal.save(s)
    with pytest.raises(ValueError):
        c.start()
    assert c.transport.sent == []


def test_api_plan_save_apply_fences_old_browser_and_preserves_config(local_owner):
    client, c, _channel, transport = local_owner
    original = deepcopy(c.config)
    snap = client.get("/api/snapshot?mode=trial").json()
    before = fence(snap)
    p = plan(step("route", "logistics", c.config["routes"]["to_delivery"]))

    def post(action, **extra):
        return client.post(
            "/api/trial/commands",
            headers=HEADERS,
            json={
                **fence(client.get("/api/snapshot?mode=trial").json(), action),
                **extra,
            },
        )

    assert post("validate_plan", plan=p).status_code == 200
    assert c.state["custom_plan"] is False
    assert post("save_plan", plan=p).status_code == 200
    assert post("load_plan", name=p["name"]).json()["result"]["plan"] == p
    assert post("configure", plan=p, field_confirmed=False).status_code == 200
    assert c.config == original and transport.sent == []
    assert (
        client.post("/api/trial/commands", headers=HEADERS, json=before).status_code
        == 409
    )
    assert post("start").status_code == 200
    assert post("configure", plan=p, field_confirmed=False).status_code == 409
    assert post("save_plan", plan=p).status_code == 409
    assert len(transport.sent) == 1


@pytest.mark.parametrize(
    "kind", ["route", "gate", "request", "mode_switch", "navigate", "delivery"]
)
def test_individual_and_request_plans_real_mqtt(profile, broker, tmp_path, kind):
    config = profile
    role = "smartcart"
    fake = FakePinky(config, role)
    if kind in {"mode_switch", "navigate"}:
        fake.state.update(node_id="N12", nav2_ready=True, localization_ready=True)
    if kind == "navigate":
        fake.state.update(mode="nav2", drive_owner="nav2")
    payloads = {
        "route": config["routes"]["cart_to_delivery"],
        "gate": {"node_id": "H1", "package_id": config["package_id"], "quantity": 1},
        "request": {"node_id": "H1", "package_id": config["package_id"], "quantity": 1},
        "mode_switch": switch_payload(config),
        "navigate": config["nav_goal"],
    }
    p = (
        delivery_plan(config, request(config, role), {"pinky3": fake.observe()})
        if kind == "delivery"
        else plan(step(kind, role, payloads[kind]))
    )
    config_mqtt = MqttConfig("127.0.0.1", "plan-owner", port=broker[0])
    transport = MqttTransport(
        config_mqtt, [topic(config["site_id"], "robot", "pinky3", "event")]
    )
    journal = Journal(tmp_path / "control.json")
    c = TrialControl(config, transport, journal, smartcart_only=True)
    c.configure(p)
    client = PinkyTrialClient(
        MqttConfig("127.0.0.1", "plan-robot", port=broker[0]),
        site=config["site_id"],
        control=config["control_id"],
        robot="pinky3",
        binding_id=config["robots"][role]["binding_id"],
        journal_path=tmp_path / "robot.json",
        observe=fake.observe,
        validate_action=fake.validate_action,
        dispatch=fake.dispatch,
        safe_stop=fake.stop,
    )
    try:
        transport.start()
        client.start()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            client.poll(0.005)
            fake.finish(client)
            c.poll(0.005)
            assert c.state["phase"] != "HOLD", c.state["fault"]
            if c.state["phase"] == "READY" and not c.start_error():
                c.start()
            elif c.state["gate"]:
                c.dummy(c.state["gate"]["stop_session_id"])
            elif (
                c.state["phase"] != "COMPLETE"
                and c._step()["kind"] == "request"
                and c.state["phase"] == c._step()["id"]
            ):
                c.request([{"package_id": config["package_id"], "quantity": 1}])
            if c.state["phase"] == "COMPLETE":
                break
        assert c.state["phase"] == "COMPLETE"
        assert fake.execution_count == sum(
            s["kind"] in {"route", "mode_switch", "navigate"} for s in p["steps"]
        )
        assert all(e["source"] == "pinky3" for e in c.state["events"].values())
        old = deepcopy(c.state)
        # New run archives the immutable old run; a late old reply cannot advance it.
        c.heard["pinky3"] = time.monotonic()
        c.configure(
            plan(
                step(
                    "gate",
                    role,
                    {
                        "node_id": "N11",
                        "package_id": config["package_id"],
                        "quantity": 1,
                    },
                )
            )
        )
        assert (tmp_path / f"run-{old['task_id']}.json").exists()
        assert c.state["task_id"] != old["task_id"]
        for e in old["events"].values():
            c.handle(delivery(e, "event"))
        assert c.state["phase"] == "READY" and not c.state["completed"]
    finally:
        client.close()
        transport.stop()
        journal.close()


def test_request_route_with_unmodified_team_driver(profile, rig_module):
    fake = FakePinky(profile, "smartcart")
    p = delivery_plan(
        profile, request(profile, "smartcart"), {"pinky3": fake.observe()}
    )
    rig = rig_module.Rig("H1", "N")
    for s in p["steps"]:
        if s["kind"] != "route":
            continue
        cmd = {"type": "route.requested", "payload": s["payload"]}
        rig.logic.validate_action(cmd)
        rig.results.clear()
        rig.logic.dispatch(cmd)
        assert rig.run(until=rig_module.finished)
        assert rig.results[-1][1] == "completed"


def test_line_only_scope_cannot_be_bypassed_by_custom_plan(profile, tmp_path):
    journal = Journal(tmp_path / "line.json")
    try:
        c = TrialControl(profile, MemoryTransport(), journal, scope="line-only")
        with pytest.raises(ValueError, match="line-only owner"):
            c.configure(plan(step("mode_switch", "smartcart", switch_payload(profile))))
        assert not c.state["custom_plan"]
    finally:
        journal.close()


def test_request_id_is_idempotent_and_conflicts_are_rejected(control_memory):
    c = control_memory
    r = request(c.config)
    p = c.plan_delivery(r)
    c.configure(p, request=r)
    before = deepcopy(c.state)
    c.configure(p, request=r)
    assert c.state == before
    with pytest.raises(ValueError, match="conflicting request_id"):
        c.configure(p, request={**r, "pickup": "N01"})
    assert not c.transport.sent


def test_custom_plan_restart_does_not_dispatch(control_memory):
    c = control_memory
    c.configure(plan(step("route", "logistics", c.config["routes"]["to_loading"])))
    c.start()
    resumed = TrialControl(c.config, c.transport, c.journal)
    assert resumed.state["phase"] == "HOLD"
    assert len(c.transport.sent) == 1


def test_nav_goal_bounds_rejected_by_api(local_owner):
    client, c, _, transport = local_owner
    p = plan(step("navigate", "logistics", c.config["nav_goal"]))
    p["steps"][0]["payload"]["pose"]["x"] = 9999
    f = fence(client.get("/api/snapshot?mode=trial").json(), "validate_plan")
    response = client.post(
        "/api/trial/commands", headers=HEADERS, json={**f, "plan": p}
    )
    assert response.status_code == 409
    assert "outside" in response.json()["detail"]
    assert transport.sent == []


def test_store_failure_cannot_apply_or_publish(control_memory, monkeypatch):
    c = control_memory
    before = deepcopy(c.plan)

    def fail(_):
        raise OSError("disk full")

    monkeypatch.setattr(c.journal, "save", fail)
    with pytest.raises(OSError):
        c.configure(plan(step("route", "logistics", c.config["routes"]["to_loading"])))
    assert c.plan == before and c.transport.sent == []


def test_all_graph_pair_routes_match_team_planner(profile, rig_module):
    from roport_line_nav.graph import plan_route

    graph = rig_module.Graph.load(rig_module.LAYOUT)
    count = 0
    stops = stop_nodes(profile)
    assert stops == {n.id for n in graph.nodes.values() if n.bar}
    for start in profile["nodes"]:
        for goal in stops:
            if start == goal:
                continue
            path = shortest_route(profile["nodes"], start, goal)
            p = plan(step("route", "logistics", route_payload(profile, path, 0)))
            validate_plan(profile, p)
            assert len(plan_route(graph, path)) == len(path) - 1
            count += 1
    assert count == len(stops) * (len(profile["nodes"]) - 1)


def test_uturn_is_rejected_before_robot_dispatch(profile):
    p = plan(
        step(
            "route",
            "logistics",
            route_payload(profile, ["N14", "N15", "N14", "N16"], 0),
        )
    )
    with pytest.raises(ValueError, match="U-turn"):
        validate_plan(profile, p)


def test_common_presets_preserve_hardware_paths_and_identity(profile, single_profile):
    before = deepcopy(profile)
    presets = trial_presets(profile)
    assert set(presets) == {"single", "two", "smartcart"}
    for preset in presets.values():
        validate_plan(profile, preset)
    assert presets["single"]["steps"] == legacy_plan(single_profile)["steps"]
    assert presets["two"]["steps"] == legacy_plan(profile)["steps"]
    assert (
        presets["smartcart"]["steps"]
        == legacy_plan(profile, smartcart_only=True)["steps"]
    )
    assert profile == before
    assert all(
        s["kind"] not in {"navigate", "mode_switch"}
        for p in trial_presets(profile, "line-only").values()
        for s in p["steps"]
    )


def test_switch_presets_in_one_owner_without_waiting_for_unused_robot(control_memory):
    c = control_memory
    before = deepcopy(c.config)
    presets = trial_presets(c.config)
    for name, roles in (
        ("single", {"logistics"}),
        ("smartcart", {"smartcart"}),
        ("two", {"logistics", "smartcart"}),
    ):
        c.configure(presets[name])
        assert set(c.robots) == roles
        assert c.start_error() is None
    c.heard.pop("pinky1")
    c.configure(presets["smartcart"])
    assert c.start_error() is None
    assert c.plan_delivery(request(c.config))["steps"][0]["role"] == "smartcart"
    c.configure(presets["two"])
    assert c.start_error() == "robot status unavailable/stale"
    assert c.config == before and not c.transport.sent


def test_unused_robot_late_status_does_not_hold_selected_plan(control_memory):
    c = control_memory
    c.poll(0)
    cmd = next(m for _, m in c.transport.sent if m["target"] == "pinky1")
    from roport_control_server.adapters.mqtt.trial_wire import reply

    event = reply(
        cmd, "status.completed", {**c.state["statuses"]["pinky1"], "phase_revision": 1}
    )
    c.configure(trial_presets(c.config)["smartcart"])
    before = deepcopy(c.state)
    c.handle(delivery(event, "event"))
    assert c.state == before
    resumed = TrialControl(c.config, c.transport, c.journal)
    assert set(resumed.robots) == {"smartcart"}
    assert set(resumed.allowed_robots) == {"logistics", "smartcart"}


def test_preset_changes_are_fenced_after_start_and_hold(control_memory):
    c = control_memory
    presets = trial_presets(c.config)
    c.configure(presets["single"])
    c.start()
    with pytest.raises(ValueError, match="frozen"):
        c.configure(presets["two"])
    c.hold("operator_stopped")
    with pytest.raises(ValueError, match="frozen"):
        c.configure(presets["smartcart"])
