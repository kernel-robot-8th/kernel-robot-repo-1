"""Shared registry contract. No transport, database or robot dependencies."""

import ipaddress
import re
from dataclasses import asdict, dataclass
from uuid import UUID

PINKY_SLOTS = ("pinky1", "pinky2", "pinky3")
ROLES = ("logistics", "smartcart")


class RegistryError(ValueError):
    """Stable error code shared by callers; never contains database details."""

    def __init__(self, code: str, status: int = 400):
        super().__init__(code)
        self.code, self.status = code, status


def integer(value, low: int, high: int, field: str) -> int:
    if type(value) is not int or not low <= value <= high:
        raise RegistryError(f"invalid_{field}")
    return value


def identifier(value, field: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(
        r"[a-zA-Z0-9][a-zA-Z0-9_.-]{0,63}", value
    ):
        raise RegistryError(f"invalid_{field}")
    return value


def uuid(value, field: str) -> str:
    if not isinstance(value, str):
        raise RegistryError(f"invalid_{field}")
    try:
        return str(UUID(value))
    except ValueError:
        raise RegistryError(f"invalid_{field}") from None


@dataclass(frozen=True)
class DesiredConfig:
    ip: str
    role: str
    ros_domain_id: int
    calibration: str

    @classmethod
    def parse(cls, data):
        if not isinstance(data, dict) or set(data) != {
            "ip",
            "role",
            "ros_domain_id",
            "calibration",
        }:
            raise RegistryError("invalid_config_fields")
        if not isinstance(data["ip"], str):
            raise RegistryError("invalid_ip")
        try:
            ip = str(ipaddress.IPv4Address(data["ip"].strip()))
        except ValueError:
            raise RegistryError("invalid_ip") from None
        if data["role"] not in ROLES:
            raise RegistryError("invalid_role")
        calibration = data["calibration"]
        if not isinstance(calibration, str) or not re.fullmatch(
            r"pinky_[0-9a-z]{2,8}", calibration
        ):
            raise RegistryError("invalid_calibration")
        return cls(
            ip,
            data["role"],
            integer(data["ros_domain_id"], 0, 101, "ros_domain_id"),
            calibration,
        )


@dataclass(frozen=True)
class RegistryRequest:
    request_id: str
    expected_revision: int
    binding_id: str | None = None
    device_id: str | None = None
    config: DesiredConfig | None = None
    excluded: bool | None = None

    @classmethod
    def parse(cls, action: str, data):
        required = {"request_id", "expected_revision"}
        if action == "registration":
            required |= {"device_id", "config"}
        elif action in {"config", "exclude", "unregister"}:
            required.add("binding_id")
            if action != "unregister":
                required.add("excluded" if action == "exclude" else "config")
        else:
            raise RegistryError("unknown_action", 404)
        if not isinstance(data, dict) or set(data) != required:
            raise RegistryError("invalid_request_fields")
        if action == "exclude" and type(data["excluded"]) is not bool:
            raise RegistryError("invalid_excluded")
        return cls(
            uuid(data["request_id"], "request_id"),
            integer(data["expected_revision"], 0, 2**53 - 1, "expected_revision"),
            uuid(data["binding_id"], "binding_id") if "binding_id" in data else None,
            identifier(data["device_id"], "device_id") if "device_id" in data else None,
            DesiredConfig.parse(data["config"]) if "config" in data else None,
            data.get("excluded"),
        )

    def payload(self):
        return asdict(self)


@dataclass(frozen=True)
class ReleaseEvidence:
    """Future executor/maintenance input, never accepted from browser JSON."""

    online: bool | None = None
    stopped: bool | None = None
    active_task: bool | None = None
    pending_command: bool | None = None
    occupied_resource: bool | None = None
    loaded: bool | None = None
    maintenance: bool | None = None

    def reasons(self, excluded: bool) -> list[str]:
        reasons = [] if excluded else ["exclude_first"]
        for field, value in asdict(self).items():
            expected = field in {"online", "stopped"}
            if value is not expected:
                reasons.append(f"{field}_unknown" if value is None else field)
        return reasons


def unknown_observation() -> dict:
    # Missing evidence is not zero battery, an empty load or a stopped robot.
    return {
        key: None
        for key in (
            "boot_id",
            "sequence",
            "observed_at",
            "received_at",
            "online",
            "battery_percent",
            "loaded",
            "stopped",
            "pose",
            "runtime_config",
        )
    }
