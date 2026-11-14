"""PR66 completion evidence stays trial-only; it never becomes inventory proof."""

# ruff: noqa: F811
import pytest
from test_pinky_trial import control_memory, profile  # noqa: F401
from test_service_tasks import deliver, dispatch_omx, omx_pair  # noqa: F401


@pytest.mark.parametrize("evidence", ["arm_return_only", "shelf_align"])
def test_panel_completion_keeps_original_evidence_and_unknown_placement(
    omx_pair, evidence
):
    control, host, executor = omx_pair
    observe = executor.observe
    executor.observe = lambda: {**observe(), "evidence": evidence}
    dispatch_omx(control, host)
    deliver(control, host.poll()[0])
    assert control.state["phase"] == "return"
    assert control.state["transfers"][0]["evidence"] == evidence
    assert control.state["transfers"][0]["placement_confirmed"] is None


def test_unknown_panel_evidence_never_releases_pinky(omx_pair):
    control, host, executor = omx_pair
    observe = executor.observe
    executor.observe = lambda: {**observe(), "evidence": "unknown"}
    dispatch_omx(control, host)
    deliver(control, host.poll()[0])
    assert control.state["phase"] == "HOLD"
    assert not control.state["transfers"]
