from collections import deque
from queue import Empty
from types import SimpleNamespace

import pytest

from roport_control_server.communication.messages import encode
from roport_control_server.examples import mqtt_demo as demo


class ScriptedTransport:
    def __init__(self, phases):
        self.phases = phases
        self.acked = 0

    def publish(self, destination, command):
        executor = demo.MockExecutor("pinky1")
        self.events = deque(
            executor._event(command, phase, revision, {})
            for phase, revision in self.phases
        )

    def receive(self, timeout):
        if not self.events:
            raise Empty
        return SimpleNamespace(payload=encode(self.events.popleft()), retained=False)

    def ack(self, delivery):
        self.acked += 1


@pytest.mark.parametrize(
    "phases",
    [
        [("accepted", 1), ("accepted", 1)],
        [("accepted", 1), ("progress", 2)],
    ],
)
def test_control_never_equates_two_events_with_completion(phases):
    transport = ScriptedTransport(phases)
    with pytest.raises(TimeoutError, match="result unknown"):
        demo.run_control(transport, "pinky1")
    assert transport.acked == 2


def test_control_waits_through_duplicates_and_old_events():
    transport = ScriptedTransport(
        [("accepted", 1), ("progress", 2), ("accepted", 1), ("completed", 3)]
    )
    demo.run_control(transport, "pinky1")
    assert transport.acked == 4


@pytest.mark.parametrize("phase", ["failed", "rejected"])
def test_control_terminal_failure_is_not_success(phase):
    with pytest.raises(RuntimeError, match=phase):
        demo.run_control(ScriptedTransport([("accepted", 1), (phase, 2)]), "pinky1")


@pytest.mark.parametrize(
    "failure", [TimeoutError("PUBACK timeout"), ConnectionError("offline")]
)
def test_cli_reports_unknown_outcome_and_stops(monkeypatch, capsys, failure):
    stopped = []

    def start():
        raise failure

    transport = SimpleNamespace(start=start, stop=lambda: stopped.append(True))
    monkeypatch.setattr(demo, "MqttTransport", lambda *args: transport)
    monkeypatch.setattr("sys.argv", ["demo", "pinky1"])
    with pytest.raises(SystemExit) as exc:
        demo.main()
    assert exc.value.code == 1 and stopped == [True]
    assert "reconcile before retry" in capsys.readouterr().err
