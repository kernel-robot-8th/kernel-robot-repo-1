"""The registry trial must not silently enable registration on field robots."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import run_service_fleet
import run_virtual_fleet


@pytest.mark.parametrize(
    "module, args, message",
    [
        (run_virtual_fleet, ["--registry"], "requires --service-env"),
        (
            run_virtual_fleet,
            ["--registry", "--service-env", "unused.json", "--verify"],
            "manual robot registration",
        ),
        (
            run_service_fleet,
            ["--registry", "--field-bundle", "unused", "--hardware-authorized"],
            "isolated virtual trial only",
        ),
    ],
)
def test_registration_trial_rejects_unsafe_combinations_before_starting(
    module, args, message, monkeypatch, capsys
):
    monkeypatch.setattr(sys, "argv", [module.__file__, *args])
    with pytest.raises(SystemExit) as error:
        module.main()
    assert error.value.code == 2
    assert message in capsys.readouterr().err
