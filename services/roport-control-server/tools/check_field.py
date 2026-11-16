"""Check field settings and optionally RECEIVE MQTT states; never publishes commands."""

import argparse
import json
import math
import re
from datetime import UTC, datetime, timedelta
from pathlib import Path
from queue import Empty
from time import monotonic
from uuid import uuid4

from prepare_pinky_trial import trial_inputs

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.mqtt import MqttTransport
from roport_control_server.adapters.mqtt.trial_runner import select_robot
from roport_control_server.adapters.mqtt.trial_wire import mqtt_config
from roport_control_server.executor.fleet import ROLES, TARGETS


def settings(root, targets):
    fleet = json.loads((root / "fleet.json").read_text())
    issues = []
    if fleet.get("mode") != "hardware" or fleet.get("field_verified") is not True:
        issues.append(
            "fleet: hardware / field_verified=true requires onsite confirmation"
        )
    maps = [p for p in (root / "maps").glob("*.yaml") if p.name != "layout.yaml"]
    if len(maps) != 1:
        raise ValueError("handoff requires one map YAML and layout.yaml")
    map_path = maps[0]
    grid, layout, nodes = trial_inputs(map_path, root / "maps/layout.yaml")
    if fleet.get("map_version") != grid["version"]:
        issues.append("fleet: map hash mismatch")
    if fleet.get("graph_version") != layout["nodes"]["graph_version"]:
        issues.append("fleet: graph version mismatch")
    if fleet.get("nodes") != {n: d["links"] for n, d in nodes.items()}:
        issues.append("fleet: node graph differs from layout")
    speed = fleet.get("speed_limit_mps")
    if (
        type(speed) not in (int, float)
        or not math.isfinite(speed)
        or not 0 < speed <= 0.06
    ):
        issues.append("fleet: field line speed must be in (0, 0.06] m/s")
    for name, goal in {**fleet["goals"], "N12_RETURN": fleet["n12_goal"]}.items():
        if goal.get("map_version") != fleet["map_version"]:
            issues.append(f"fleet: goal {name} map_version mismatch")
        try:
            wire.build_message(
                "nav_goal_command",
                fleet["control_id"],
                "pinky3",
                {
                    **goal,
                    "expected_boot_version": "settings-check",
                    "map_version": fleet["map_version"],
                },
                command_id="settings-check",
                expires_at=(datetime.now(UTC) + timedelta(seconds=5)).isoformat(),
            )
        except ValueError:
            issues.append(f"fleet: invalid goal {name}")
    for target in targets:
        if target in ROLES:
            profile = json.loads((root / target / "trial.json").read_text())
            role = ROLES[target]
            try:
                robot = select_robot(profile, role, hardware_authorized=True)
                if robot["robot_id"] != target:
                    issues.append(f"{target}: robot slot mismatch")
            except (ValueError, KeyError):
                issues.append(f"{target}: profile/role or onsite confirmation missing")
            for key in (
                "control_id",
                "map_id",
                "map_version",
                "graph_version",
                "nodes",
            ):
                if profile.get(key) != fleet.get(key):
                    issues.append(f"{target}: {key} differs from fleet")
        else:
            versions = fleet.get("omx", {}).get(target, {})
            for key in ("model_version", "calibration_version"):
                value = versions.get(key)
                if (
                    not isinstance(value, str)
                    or not re.fullmatch(wire.ID["pattern"], value)
                    or value.startswith(("sim-", "REPLACE"))
                ):
                    issues.append(f"{target}: actual {key} required")
    return fleet, issues


class StateCheck:
    def __init__(self, config, targets, nav2=()):
        self.config, self.targets, self.nav2 = config, targets, nav2
        self.samples = {}
        self.boots = {t: set() for t in targets}
        self.counts = {t: 0 for t in targets}
        self.rejected = 0

    def receive(self, delivery):
        message = wire.decode(delivery.payload)
        if message["type"] != "robot_state":
            return
        target = message["source"]
        if (
            target not in self.targets
            or message["target"] != self.config["control_id"]
            or delivery.topic != wire.topic(target, "event")
            or delivery.qos != 1
            or delivery.retained
        ):
            raise ValueError("state topic/identity/QoS/retained mismatch")
        p = message["payload"]
        self.boots[target].add(p["boot_version"])
        previous = self.samples.get(target)
        if (
            previous
            and previous["boot_version"] == p["boot_version"]
            and p["sequence"] <= previous["sequence"]
        ):
            return
        self.samples[target] = p
        self.counts[target] += 1

    def report(self, now=None):
        now = now or datetime.now(UTC)
        rows = {}
        for target in self.targets:
            problems = []
            p = self.samples.get(target)
            if p is None:
                rows[target] = {"ok": False, "issues": ["no robot_state received"]}
                continue
            s = p["state"]
            age = (now - wire.timestamp(p["observed_at"])).total_seconds()
            if not -1 <= age <= 3:
                problems.append("state stale or clocks differ")
            if self.counts[target] < 2:
                problems.append("need two increasing state sequences")
            if len(self.boots[target]) != 1:
                problems.append(
                    "boot changed or duplicate endpoint: inspect owners and rerun"
                )
            if s["stopped"] is not True or s["fault"] or p["active_command_id"]:
                problems.append("not stopped/idle or fault present")
            if target in ROLES:
                for key in ("map_version", "graph_version"):
                    if s[key] != self.config[key]:
                        problems.append(key + " mismatch")
                if (
                    s["mode"] != "line"
                    or s["node"] not in self.config["nodes"]
                    or s["heading_rad"] is None
                ):
                    problems.append("initial line position/heading unknown")
                if s["line_enabled"] or s["drive_owner"] != "none":
                    problems.append("drive owner not released")
                if target in self.nav2 and not (
                    s["nav2_ready"] and s["localization_ready"]
                ):
                    problems.append("Nav2/localization not ready")
            else:
                if not s["ready"]:
                    problems.append("OMX not ready")
                arms = s["arms"] if target == "shelf-01" else [s]
                if any(
                    not a["ready"]
                    or not a["stopped"]
                    or not a["home_reached"]
                    or a["holding_state"] != "empty"
                    or a["holding_package_id"]
                    or a["fault"]
                    for a in arms
                ):
                    problems.append("arm stop/home/empty not confirmed")
            rows[target] = {
                "ok": not problems,
                "issues": problems,
                "samples": self.counts[target],
                "state_age_s": round(age, 3),
                "boot_version": p["boot_version"],
                "state": s,
            }
        return {
            "ok": not self.rejected and all(r["ok"] for r in rows.values()),
            "rejected_states": self.rejected,
            "targets": rows,
        }


def listen(config, targets, connection, duration, *, nav2=()):
    check = StateCheck(config, targets, nav2)
    transport = MqttTransport(connection, [wire.topic(t, "event") for t in targets])
    try:
        transport.start()
        deadline = monotonic() + duration
        while monotonic() < deadline:
            try:
                delivery = transport.receive(
                    timeout=min(0.1, max(0, deadline - monotonic()))
                )
            except Empty:
                continue
            try:
                check.receive(delivery)
            except ValueError:
                check.rejected += 1
            finally:
                # MQTT PUBACK only; never publish application event_stored or a lease.
                transport.ack(delivery)
        return check.report()
    finally:
        transport.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--targets", nargs="+", choices=TARGETS, default=list(TARGETS))
    parser.add_argument("--nav2", nargs="*", choices=ROLES, default=[])
    parser.add_argument("--listen", action="store_true")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
        parser.error("seconds must be in (0, 60]")
    if not set(args.nav2) <= set(args.targets):
        parser.error("--nav2 robots must be included in --targets")
    report = {
        "scope": "settings_and_passive_mqtt",
        "motion_commands_sent": 0,
        "motion_verified": False,
        "ok": False,
    }
    try:
        config, issues = settings(args.bundle, args.targets)
        report.update(settings_ok=not issues, settings_issues=issues)
        if args.listen:
            connection = mqtt_config("field-check-" + uuid4().hex[:12])
            report["mqtt"] = listen(
                config, args.targets, connection, args.seconds, nav2=args.nav2
            )
        report["ok"] = not issues and (not args.listen or report["mqtt"]["ok"])
    except (
        ValueError,
        KeyError,
        TypeError,
        StopIteration,
        OSError,
        ConnectionError,
    ) as exc:
        report["error"] = (
            type(exc).__name__
            + ": configuration or connection failed; check local settings"
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
