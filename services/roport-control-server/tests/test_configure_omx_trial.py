import importlib.util
import json
import subprocess
import sys
from copy import deepcopy

import pytest
from test_pinky_trial import SERVICE, profile  # noqa: F401

spec = importlib.util.spec_from_file_location(
    "configure_omx", SERVICE / "tools/configure_omx_trial.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def configure(config, **changes):
    options = {
        "mode": "mock",
        "slot": "2b",
        "model": "test-model",
        "calibration": "test-cal",
        "package_marker": 16,
    }
    return module.configure(config, **(options | changes))


def test_preserves_original_and_existing_binding(profile):  # noqa: F811
    before = deepcopy(profile)
    result = configure(profile)
    assert profile == before
    # everything but the OMX settings and the package_id (now the box marker) is kept
    kept = {k: v for k, v in result.items() if k not in {"omx", "package_id"}}
    assert kept == {k: v for k, v in before.items() if k != "package_id"}
    assert result["package_id"] == "16"
    again = configure(result, slot="1a")
    assert again["omx"]["binding_id"] == result["omx"]["binding_id"]


def test_package_id_is_the_box_marker(profile):  # noqa: F811
    # 2026-11-21 review: "trial-package-1" was kept and the Host refused it (INVALID_TARGET)
    assert profile["package_id"] != "16"
    assert configure(profile)["package_id"] == "16"
    assert configure(profile, package_marker=22)["package_id"] == "22"


@pytest.mark.parametrize(
    "changes",
    [
        {"mode": "hardware"},
        {"slot": "3a"},
        {"model": ""},
        {"package_marker": 18},  # the Pinky tray marker
        {"package_marker": 14},
        {"package_marker": "16"},
        {"package_marker": True},
    ],
)
def test_rejects_invalid_or_real_executor_for_simulation(profile, changes):  # noqa: F811
    with pytest.raises(ValueError):
        configure(profile, **changes)


def test_hardware_does_not_grant_field_approval(profile):  # noqa: F811
    profile["mode"] = "hardware"
    result = configure(profile, mode="hardware")
    assert result["field_verified"] is False
    assert result["mode"] == "hardware"


@pytest.mark.parametrize("config", [[], {"omx": None}])
def test_rejects_non_object_settings(config):
    with pytest.raises(TypeError):
        configure(config)


def test_cli_never_overwrites_source_or_output(profile, tmp_path):  # noqa: F811
    source, output = tmp_path / "trial.json", tmp_path / "omx.json"
    source.write_text(json.dumps(profile))
    original = source.read_bytes()
    command = [
        sys.executable,
        str(SERVICE / "tools/configure_omx_trial.py"),
        "--config",
        str(source),
        "--output",
        str(output),
        "--execution-mode",
        "mock",
        "--slot",
        "2b",
        "--model-version",
        "test-model",
        "--calibration-version",
        "test-cal",
        "--package-marker",
        "16",
    ]
    first = subprocess.run(command, capture_output=True, text=True, check=False)
    assert first.returncode == 0, first.stderr
    config = json.loads(output.read_text())
    assert config["omx"]["binding_id"] in first.stdout
    assert config["package_id"] == "16"
    assert "--box-marker" not in first.stdout
    assert profile["site_id"] in first.stdout and "MOCK ONLY" in first.stdout
    saved = output.read_bytes()
    assert subprocess.run(command, capture_output=True, check=False).returncode == 2
    assert output.read_bytes() == saved and source.read_bytes() == original
    command[command.index("--output") + 1] = str(source)
    assert subprocess.run(command, capture_output=True, check=False).returncode == 2
    assert source.read_bytes() == original
