"""Check one paired panel/wire capture from a single-arm 2A run, entirely offline."""

import json
import sys
from pathlib import Path

from roport_control_server.adapters.mqtt import field_wire

PANEL_JOINTS = {
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
}


def check(sample):
    if sample["sample_origin"] not in {"synthetic", "measured"}:
        raise ValueError("sample_origin must be synthetic or measured")
    evidence = sample["evidence"]
    fields = {**evidence, "mapping_revision": sample["mapping_revision"]}
    for name in ("mapping_revision", "dock", "view_direction", "phase", "media_ref"):
        value = fields[name]
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"supply {name} for this observation")
        if sample["sample_origin"] == "measured" and value.strip().lower().startswith(
            "synthetic-"
        ):
            raise ValueError(f"replace the synthetic {name} with the field value")
    offset = evidence.get("clip_offset_ms")
    if offset is not None and (type(offset) is not int or offset < 0):
        raise ValueError("clip_offset_ms must be nonnegative milliseconds")
    panel, wire = sample["panel"], sample["wire"]
    field_wire.validate(wire)
    if (sample["slot"], panel["arm"], wire["source"], wire["type"]) != (
        "2A",
        "left",
        "omx1",
        "omx_joint_state",
    ) or sample["topic"] != field_wire.topic("omx1", "motion"):
        raise ValueError("2A requires left, omx1, its motion topic and omx_joint_state")
    payload = wire["payload"]
    if payload["sample_kind"] != "mission":
        raise ValueError("capture the participating arm during its 2A mission")
    if field_wire.timestamp(panel["observed_at"]) != field_wire.timestamp(
        payload["observed_at"]
    ):
        raise ValueError("the panel and wire must identify the same observation")
    joints = panel["joints"]
    if not isinstance(joints, dict) or joints.keys() != PANEL_JOINTS:
        raise ValueError("supply all six named panel joints")
    for name, value in joints.items():
        lower = 0 if name == "gripper" else -100
        if type(value) not in (float, int) or not lower <= value <= 100:
            raise ValueError(f"{name} must be a normalized number in [{lower}, 100]")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("Usage: python tools/check_omx_2a_sample.py capture.json [...]")
    for filename in sys.argv[1:]:
        try:
            capture = json.loads(Path(filename).read_bytes())
            check(capture)
        except (KeyError, OSError, TypeError, ValueError) as exc:
            sys.exit(f"{filename}: {exc}")
        print(f"OK {filename} ({capture['sample_origin']}, 2A / left / omx1)")
