"""Pinky Settings view for the web UI. Read-only until the operating DB and executor exist.

OFFLINE returns an empty registry and rejects writes with 503 after validation.
DEMO returns sample robots; the browser keeps DEMO edits in its own session only.
Nothing here connects to robots, SSH, MQTT or a database.
"""

from ...contracts.pinky_settings import (
    ACTIONS,
    DOMAIN_MAX,
    PACKAGES,
    RELEASE_REF,
    ROLES,
)

# Example values only (2026-11-03 per-robot domain decision); not live devices.
DEMO_ROBOTS = [
    {
        "name": "pinky1",
        "ip": "192.0.2.11",
        "role": "logistics",
        "ros_domain_id": 14,
        "calibration": "pinky_d660",
    },
    {
        "name": "pinky2",
        "ip": "192.0.2.12",
        "role": "logistics",
        "ros_domain_id": 15,
        "calibration": "pinky_6422",
    },
    {
        "name": "pinky3",
        "ip": "192.0.2.13",
        "role": "smartcart",
        "ros_domain_id": 16,
        "calibration": "pinky_764e",
    },
]

NOT_CONNECTED = "executor_not_connected"


def settings_view(demo: bool) -> dict:
    return {
        "demo": demo,
        "executor_connected": False,
        "robots": [dict(r) for r in DEMO_ROBOTS] if demo else [],
        "roles": list(ROLES),
        "actions": list(ACTIONS),
        "domain_range": [0, DOMAIN_MAX],
        "release_ref": RELEASE_REF,
        "packages": list(PACKAGES),
    }
