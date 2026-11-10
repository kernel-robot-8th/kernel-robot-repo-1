"""Preview selection failures must never silently switch to another target."""

import math
import threading
from types import SimpleNamespace

import pytest

from roport_smartcart.target_tracking import Detection, TargetSelection
from roport_smartcart.target_preview import VideoWorker, main


def detection(track_id=1, box=(0, 0, 100, 100), class_id=0):
    return Detection(track_id, box, .9, class_id)


def selected():
    selection = TargetSelection()
    selection.select([detection()], 50, 50, 10, 10.1)
    assert selection.target.track_id == 1
    return selection


def test_follow_selected_id_even_when_other_detection_is_first():
    selection = selected()
    moved = detection(box=(10, 5, 110, 105))
    selection.update([detection(2, (200, 0, 300, 100)), moved], 10.2, 10.3)
    assert selection.target == moved


@pytest.mark.parametrize('detections', [
    [], [detection(2)], [detection(), detection()],
    [detection(class_id=1)], [detection(box=(200, 0, 300, 100))],
    [detection(), detection(2, (50, 50, 150, 150))],
])
def test_loss_ambiguity_and_id_jump_latch_until_reselection(detections):
    selection = selected()
    selection.update(detections, 10.2, 10.3)
    assert selection.target is None
    selection.update([detection()], 10.4, 10.45)
    assert selection.target is None
    selection.select([detection()], 50, 50, 10.4, 10.45)
    assert selection.target is not None


@pytest.mark.parametrize('stamp,now', [
    (10, 10.51), (11, 10), (math.nan, 10), (10, math.inf), (9.9, 10),
])
def test_bad_or_old_frame_invalidates_selection(stamp, now):
    selection = selected()
    selection.update([detection()], stamp, now)
    assert selection.target is None


def test_frozen_frame_expires_even_if_tracker_keeps_same_id():
    selection = selected()
    selection.update([detection()], 10, 10.4)
    assert selection.target is not None
    selection.update([detection()], 10, 10.6)
    assert selection.target is None


@pytest.mark.parametrize('detections,x,y,stamp,now', [
    ([detection()], 150, 50, 10, 10),
    ([detection(), detection(2)], 50, 50, 10, 10),
    ([detection()], 50, 50, 10, 11),
])
def test_empty_ambiguous_or_stale_click_does_not_select(detections, x, y, stamp, now):
    selection = selected()
    selection.select(detections, x, y, stamp, now)
    assert selection.target is None


@pytest.mark.parametrize('box', [(0, 0, 0, 1), (2, 0, 1, 1), (0, 0, math.nan, 1)])
def test_invalid_detection_rejected(box):
    with pytest.raises(ValueError):
        detection(box=box)


def test_capture_error_is_redacted_and_released():
    released = []
    capture = SimpleNamespace(
        isOpened=lambda: True, get=lambda _: 30,
        read=lambda: (_ for _ in ()).throw(RuntimeError('private stream credentials')),
        release=lambda: released.append(True))
    cv = SimpleNamespace(VideoCapture=lambda _: capture, CAP_PROP_FPS=5)
    worker = VideoWorker(cv, None, 0, [0], .35)
    worker.run()
    latest, error = worker.snapshot()
    assert latest is None
    assert error == 'SOURCE / DETECTOR ERROR (RuntimeError)'
    assert released == [True]


def test_slow_capture_does_not_block_status_reader():
    entered, unblock = threading.Event(), threading.Event()

    def read():
        entered.set()
        unblock.wait(2)
        return False, None

    capture = SimpleNamespace(isOpened=lambda: True, get=lambda _: 30,
                              read=read, release=lambda: None)
    cv = SimpleNamespace(VideoCapture=lambda _: capture, CAP_PROP_FPS=5)
    worker = VideoWorker(cv, None, 0, [0], .35)
    worker.thread.start()
    try:
        assert entered.wait(1)
        assert worker.snapshot() == (None, None)
    finally:
        unblock.set()
        worker.thread.join(2)
    assert not worker.thread.is_alive()
    assert worker.snapshot()[1] is not None


def test_missing_model_rejected_before_optional_dependencies(tmp_path):
    with pytest.raises(SystemExit) as error:
        main(['--source', '0', '--model', str(tmp_path / 'missing.pt')])
    assert error.value.code == 2
