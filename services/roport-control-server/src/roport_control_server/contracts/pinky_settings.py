"""Pinky Settings request/display contract (standard library only).

Mirrors the Pinky test console fleet mode (services/roport-pinky/src/pinky_test_console,
mode:=fleet) so the control UI and the future executor validate the same fields.
Registration: name, IPv4, role, ROS_DOMAIN_ID (per robot, 0..101), IR calibration name.
Jobs: check, read_domain, set_domain, deploy (branch ref or release = origin/main), build.
The API only validates here; executing jobs belongs to the executor (not implemented).
"""

import ipaddress
import re

NAME = re.compile(r"^[a-z][a-z0-9_-]{0,15}$")
CALIBRATION = re.compile(r"^pinky_[0-9a-z]{2,8}$")
REF = re.compile(r"^origin/[A-Za-z0-9][A-Za-z0-9._/-]{0,92}$")
ROLES = ("logistics", "smartcart")
ACTIONS = ("check", "read_domain", "set_domain", "deploy", "build")
DOMAIN_MAX = 101
RELEASE_REF = "origin/main"
PACKAGES = (
    "pinky_test_console",
    "roport_pinky_line",
    "roport_line_nav",
    "roport_gz_world",
)


def _int_in(value, low, high, field):
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not low <= value <= high
    ):
        raise ValueError(f"{field} must be an integer {low}..{high}")
    return value


def validate_registration(data: dict) -> dict:
    """Return a normalized registration or raise ValueError/TypeError."""
    if not isinstance(data, dict):
        raise TypeError("registration must be an object")
    name = data.get("name")
    if not isinstance(name, str) or not NAME.match(name):
        raise ValueError(
            "name must be lowercase letters/digits/-/_ (max 16), e.g. pinky1"
        )
    try:
        ip = str(ipaddress.IPv4Address(str(data.get("ip", "")).strip()))
    except ValueError:
        raise ValueError("ip must be an IPv4 address") from None
    role = data.get("role")
    if role not in ROLES:
        raise ValueError(f"role must be one of {', '.join(ROLES)}")
    calibration = data.get("calibration")
    if not isinstance(calibration, str) or not CALIBRATION.match(calibration):
        raise ValueError("calibration must look like pinky_d660")
    return {
        "name": name,
        "ip": ip,
        "role": role,
        "ros_domain_id": _int_in(
            data.get("ros_domain_id"), 0, DOMAIN_MAX, "ros_domain_id"
        ),
        "calibration": calibration,
    }


def validate_job(data: dict) -> dict:
    """Return a normalized job request or raise ValueError/TypeError."""
    if not isinstance(data, dict):
        raise TypeError("job must be an object")
    name = data.get("name")
    if not isinstance(name, str) or not NAME.match(name):
        raise ValueError("name must be a registered robot name")
    action = data.get("action")
    if action not in ACTIONS:
        raise ValueError(f"action must be one of {', '.join(ACTIONS)}")
    job = {"name": name, "action": action}
    if action == "set_domain":
        job["ros_domain_id"] = _int_in(
            data.get("ros_domain_id"), 0, DOMAIN_MAX, "ros_domain_id"
        )
    if action == "deploy":
        if data.get("release") is True:
            job["ref"] = RELEASE_REF
        else:
            ref = data.get("ref")
            if (
                not isinstance(ref, str)
                or not REF.match(ref)
                or ".." in ref
                or ref.endswith("/")
            ):
                raise ValueError("ref must be a remote branch such as origin/main")
            job["ref"] = ref
    return job
