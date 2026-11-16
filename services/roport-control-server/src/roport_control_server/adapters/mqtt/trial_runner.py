"""Portable trial entry point; one shared profile, one role and an injected driver."""

import argparse
import importlib
import json
from copy import deepcopy
from pathlib import Path

from .field_pinky import FieldPinkyClient
from .trial_wire import PROFILE, command, mqtt_config


def select_robot(profile, role, *, hardware_authorized=False):
    scenario = profile.get("scenario", "two_pinky")
    roles = {"logistics"} if scenario == "single_pinky" else {"logistics", "smartcart"}
    if (
        profile.get("profile") != PROFILE
        or scenario not in {"single_pinky", "two_pinky"}
        or profile.get("mode") not in {"simulation", "hardware"}
        or set(profile.get("robots", {})) != roles
        or role not in roles
    ):
        raise ValueError("trial profile/scenario/role mismatch")
    if profile["mode"] == "hardware" and (
        profile.get("field_verified") is not True or not hardware_authorized
    ):
        raise ValueError("hardware requires field_verified and --hardware-authorized")
    robot = profile["robots"][role]
    if robot["robot_id"] not in {"pinky1", "pinky2", "pinky3"}:
        raise ValueError("Pinky slot required")
    command(
        site=profile["site_id"],
        control=profile["control_id"],
        robot=robot["robot_id"],
        task="validation",
        step="status",
        kind="status",
        payload={"binding_id": robot["binding_id"]},
    )
    return robot


def run(profile, role, connection, journal_path, driver):
    for name in (
        "observe",
        "validate_action",
        "dispatch",
        "safe_stop",
        "drain_results",
    ):
        if not callable(getattr(driver, name, None)):
            raise TypeError(f"driver requires callable {name}")
    robot = profile["robots"][role]
    client = FieldPinkyClient(connection, profile, role, journal_path, driver)
    try:
        client.start()
        print(
            f"MQTT endpoint running: {role}/{robot['robot_id']} (not motion readiness)",
            flush=True,
        )
        while True:
            client.poll(0.1)
            # Only the MQTT owner loop renews the independent robot drive lease.
            heartbeat = getattr(driver, "heartbeat", None)
            if heartbeat is not None and client.transport.ready and not client.halted:
                heartbeat()
            for original, phase, details in driver.drain_results():
                client.report(original, phase, **details)
            client.publish_motion()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            client.close()
        finally:
            close = getattr(driver, "close", None)
            if close is not None:
                close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--role", required=True, choices=("logistics", "smartcart"))
    parser.add_argument("--journal", required=True)
    parser.add_argument("--driver", help="trusted Python module:factory(profile, role)")
    parser.add_argument("--hardware-authorized", action="store_true")
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate identity/mode; no driver import or network",
    )
    args = parser.parse_args()
    try:
        profile = json.loads(Path(args.config).read_text())
        robot = select_robot(
            profile, args.role, hardware_authorized=args.hardware_authorized
        )
        connection = mqtt_config(f"{profile['site_id']}-{robot['robot_id']}")
        if profile["mode"] == "simulation" and connection.host not in {
            "127.0.0.1",
            "localhost",
            "::1",
        }:
            raise ValueError("simulation profile is loopback-only")
        if args.check:
            print(
                f"Identity/mode valid: {profile.get('scenario', 'two_pinky')} / {args.role}; driver, broker and motion not checked"
            )
            return
        if not args.driver or args.driver.count(":") != 1:
            raise ValueError("--driver requires a trusted module:factory")
        module, factory = args.driver.split(":")
        make_driver = getattr(importlib.import_module(module), factory)
        driver = make_driver(deepcopy(profile), args.role)
        run(profile, args.role, connection, args.journal, driver)
    except (ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
