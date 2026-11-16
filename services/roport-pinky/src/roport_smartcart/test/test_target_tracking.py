"""Preview selection failures must never silently switch to another target."""

import math
import sys
import threading
from types import SimpleNamespace

import pytest

from roport_smartcart.target_tracking import Detection, TargetSelection
from roport_smartcart.target_preview import VideoWorker, main, parse_args


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


@pytest.mark.parametrize('value,expected', [('0', 0), ('2', 2),
    ('/outside-repository/test.mp4', '/outside-repository/test.mp4'),
    ('rtsp://example.com/stream', 'rtsp://example.com/stream')])
def test_environment_source_resolves_without_printing_value(tmp_path, monkeypatch, capsys, value, expected):
    model = tmp_path / 'test.pt'
    model.touch()
    monkeypatch.setenv('PINKY_VIDEO_SOURCE', value)
    _, args = parse_args(['--source-env', 'PINKY_VIDEO_SOURCE', '--model', str(model)])
    assert args.source == expected
    captured = capsys.readouterr()
    assert captured.out == captured.err == ''


@pytest.mark.parametrize('value', [None, '', '   '])
def test_missing_or_blank_environment_source_is_rejected(tmp_path, monkeypatch, capsys, value):
    model = tmp_path / 'test.pt'
    model.touch()
    monkeypatch.delenv('PINKY_VIDEO_SOURCE', raising=False)
    if value is not None:
        monkeypatch.setenv('PINKY_VIDEO_SOURCE', value)
    with pytest.raises(SystemExit) as error:
        parse_args(['--source-env', 'PINKY_VIDEO_SOURCE', '--model', str(model)])
    assert error.value.code == 2
    assert 'populated environment variable' in capsys.readouterr().err


def test_source_options_are_mutually_exclusive(tmp_path, monkeypatch, capsys):
    model = tmp_path / 'test.pt'
    model.touch()
    monkeypatch.setenv('PINKY_VIDEO_SOURCE', 'rtsp://example.com/private-stream')
    with pytest.raises(SystemExit) as error:
        parse_args(['--source', '0', '--source-env', 'PINKY_VIDEO_SOURCE', '--model', str(model)])
    assert error.value.code == 2
    assert 'private-stream' not in capsys.readouterr().err


def test_source_value_is_not_echoed_when_other_validation_fails(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv('PINKY_VIDEO_SOURCE', 'rtsp://example.com/private-stream')
    with pytest.raises(SystemExit):
        parse_args(['--source-env', 'PINKY_VIDEO_SOURCE', '--model', str(tmp_path / 'missing.pt')])
    assert 'private-stream' not in capsys.readouterr().err


def test_missing_optional_dependency_exits_cleanly(tmp_path, monkeypatch, capsys):
    model = tmp_path / 'test.pt'
    model.touch()
    monkeypatch.setitem(sys.modules, 'ultralytics', None)
    with pytest.raises(SystemExit) as error:
        main(['--source', '0', '--model', str(model)])
    assert error.value.code == 2
    assert 'optional preview dependencies' in capsys.readouterr().err


def test_unknown_model_class_exits_before_starting_video(tmp_path, monkeypatch, capsys):
    model = tmp_path / 'test.pt'
    model.touch()
    monkeypatch.setitem(sys.modules, 'cv2', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'numpy', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(
        YOLO=lambda *a, **k: SimpleNamespace(names={1: 'doll'})))
    with pytest.raises(SystemExit) as error:
        main(['--source', '0', '--model', str(model)])
    assert error.value.code == 2
    assert 'class ID is absent' in capsys.readouterr().err
