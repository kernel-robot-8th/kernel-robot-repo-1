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
    }
    return module.configure(config, **(options | changes))


def test_preserves_original_and_existing_binding(profile):  # noqa: F811
    before = deepcopy(profile)
    result = configure(profile)
    assert profile == before
    assert {k: v for k, v in result.items() if k != "omx"} == before
    again = configure(result, slot="1a")
    assert again["omx"]["binding_id"] == result["omx"]["binding_id"]


@pytest.mark.parametrize(
    "changes", [{"mode": "hardware"}, {"slot": "3a"}, {"model": ""}]
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
    ]
    first = subprocess.run(command, capture_output=True, text=True, check=False)
    assert first.returncode == 0, first.stderr
    config = json.loads(output.read_text())
    assert config["omx"]["binding_id"] in first.stdout
    assert profile["site_id"] in first.stdout and "MOCK ONLY" in first.stdout
    saved = output.read_bytes()
    assert subprocess.run(command, capture_output=True, check=False).returncode == 2
    assert output.read_bytes() == saved and source.read_bytes() == original
    command[command.index("--output") + 1] = str(source)
    assert subprocess.run(command, capture_output=True, check=False).returncode == 2
    assert source.read_bytes() == original
