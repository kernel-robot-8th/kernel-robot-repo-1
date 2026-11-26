import json
import sys
from pathlib import Path

import pytest

SERVICE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVICE / "tools"))

from check_omx_2a_sample import check


def example():
    return json.loads((SERVICE / "docs/examples/omx-2a-sample.json").read_text())


def test_synthetic_pair_matches_v2_contract():
    check(example())


def test_idle_read_cannot_substitute_for_2a_mission():
    sample = example()
    sample["wire"]["payload"]["sample_kind"] = "idle"
    with pytest.raises(ValueError, match="2A mission"):
        check(sample)


def test_measured_capture_needs_its_own_calibration_and_evidence():
    sample = example()
    sample["sample_origin"] = "measured"
    with pytest.raises(ValueError, match="mapping_revision"):
        check(sample)
    sample["mapping_revision"] = "fixture-table-r2"
    with pytest.raises(ValueError, match="dock"):
        check(sample)
    sample["evidence"].update(
        dock="fixture-stop", media_ref="frame.jpg", clip_offset_ms=None
    )
    check(sample)


@pytest.mark.parametrize(
    ("section", "key", "value"),
    [
        ("panel", "arm", "right"),
        ("panel", "observed_at", "2030-01-01T00:00:00.200Z"),
        ("panel", "joints", {"gripper": 12}),
        ("wire", "source", "omx2"),
    ],
)
def test_different_arm_time_or_incomplete_read_is_not_paired(section, key, value):
    sample = example()
    sample[section][key] = value
    with pytest.raises(ValueError):
        check(sample)
