"""Stop sessions and reservation leases fail closed before driving."""

import pytest
from roport_smartcart.sim_control import ReturnPermit
from roport_smartcart.sim_return_h1 import line_return_route


def grant(**changes):
    return {
        "cmd": "return",
        "session": "current",
        "receipt_confirmed": True,
        "bay": "H2",
        "reservation_id": "reservation-1",
        "expires_at": 300.0,
        **changes,
    }


@pytest.mark.parametrize(
    "changes",
    [
        {"session": "old"},
        {"receipt_confirmed": False},
        {"receipt_confirmed": 1},
        {"bay": "H3"},
        {"reservation_id": ""},
        {"expires_at": 99.0},
        {"expires_at": float("nan")},
        {"expires_at": float("inf")},
        {"expires_at": True},
        {"expires_at": 501.0},
    ],
)
def test_invalid_grants_cannot_release_wait(changes):
    permit = ReturnPermit("current")
    permit.waiting = True
    permit.observe(grant(**changes), 100.0)
    assert permit.grant is None


def test_grant_only_during_current_delivery_stop():
    permit = ReturnPermit("current")
    permit.observe(grant(), 100.0)
    assert permit.grant is None
    permit.waiting = True
    permit.observe(grant(), 100.0)
    assert permit.grant["bay"] == "H2"
    permit.check(101.0)
    with pytest.raises(RuntimeError, match="heartbeat"):
        permit.check(102.01)


def test_duplicate_grant_does_not_change_reserved_bay_or_extend_expiry():
    permit = ReturnPermit("current")
    permit.waiting = True
    permit.observe(grant(), 100.0)
    permit.observe(grant(bay="H1", expires_at=301.0), 101.0)
    assert permit.grant == grant() and permit.last_seen == 100.0
    permit.observe(grant(), 299.0)
    with pytest.raises(RuntimeError, match="expired"):
        permit.check(300.0)


def test_stop_invalidates_even_a_later_valid_grant():
    permit = ReturnPermit("current")
    permit.waiting = True
    permit.observe({"cmd": "stop", "session": "old"}, 100.0)
    permit.check(100.0)
    permit.observe({"cmd": "stop", "session": "current"}, 100.0)
    permit.observe(grant(), 100.1)
    assert permit.grant is None
    with pytest.raises(RuntimeError, match="stop"):
        permit.check(100.1)


@pytest.mark.parametrize(
    "bay,route", [("H1", ["N12", "N08", "N09", "H1"]), ("H2", ["N12", "N08", "H2"])]
)
def test_selected_bay_has_a_park_route(bay, route):
    from pathlib import Path
    from roport_line_nav.graph import Graph, plan_route

    path = (
        Path(__file__).parents[1].parent / "roport_gz_world/layout/roport_airport.yaml"
    )
    assert line_return_route(bay) == route
    assert plan_route(Graph.load(path), route)[-1].action == "park"
    with pytest.raises(ValueError):
        line_return_route("home")


def test_control_fixture_rejects_physical_domain_before_ros(monkeypatch):
    from roport_smartcart.sim_control import main

    monkeypatch.setenv("ROS_DOMAIN_ID", "13")
    monkeypatch.setattr("sys.argv", ["sim_control"])
    with pytest.raises(SystemExit) as exc:
        main()
    assert exc.value.code == 2
