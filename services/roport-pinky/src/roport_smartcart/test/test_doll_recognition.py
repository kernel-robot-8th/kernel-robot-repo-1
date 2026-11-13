"""Synthetic geometry only: real artwork stays outside the repository."""
import json

import pytest


@pytest.fixture
def recognition():
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    from roport_smartcart.doll_recognition import DollRecognizer
    return cv2, np, DollRecognizer


def test_transformed_artwork_and_empty_frame(tmp_path, recognition):
    cv2, np, DollRecognizer = recognition
    rng = np.random.default_rng(7)
    reference = np.full((240, 180), 220, np.uint8)
    for _ in range(150):
        x, y = rng.integers([10, 10], [170, 230])
        cv2.circle(reference, (int(x), int(y)), int(rng.integers(2, 7)), int(rng.integers(0, 200)), -1)
    cv2.imwrite(str(tmp_path / 'ref.png'), reference)
    cv2.imwrite(str(tmp_path / 'mask.png'), np.full(reference.shape, 255, np.uint8))
    manifest = tmp_path / 'refs.json'
    manifest.write_text(json.dumps([{'label': 'A', 'view': 'front', 'image': 'ref.png', 'mask': 'mask.png'}]))
    detector = DollRecognizer(manifest)
    source = np.float32([[0, 0], [180, 0], [180, 240], [0, 240]])
    target = np.float32([[180, 60], [350, 80], [325, 300], [165, 290]])
    matrix = cv2.getPerspectiveTransform(source, target)
    scene = cv2.warpPerspective(reference, matrix, (500, 400))
    found = detector.detect(cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR))
    assert len(found) == 1 and found[0]['label'] == 'A'
    assert np.max(np.abs(np.array(found[0]['quad']) - target)) < 4
    assert detector.detect(np.zeros((400, 500, 3), np.uint8)) == []
    noise = rng.integers(0, 256, (400, 500, 3), dtype=np.uint8)
    assert detector.detect(noise) == []


def test_empty_manifest_rejected(tmp_path, recognition):
    _, _, DollRecognizer = recognition
    path = tmp_path / 'empty.json'
    path.write_text('[]')
    with pytest.raises(ValueError, match='empty'):
        DollRecognizer(path)


def test_missing_mask_rejected(tmp_path, recognition):
    _, _, DollRecognizer = recognition
    path = tmp_path / 'refs.json'
    path.write_text(json.dumps([{'label': 'A', 'view': 'front', 'image': 'absent.png', 'mask': 'absent.png'}]))
    with pytest.raises(ValueError, match='equal dimensions'):
        DollRecognizer(path)


@pytest.mark.parametrize('rows', [
    [{}],
    [{'label': 'A', 'image': 'ref.png', 'mask': 'mask.png'}],
    [{'label': 'A', 'view': 'front', 'mask': 'mask.png'}],
    [{'label': 'A', 'view': 'front', 'image': 'ref.png'}],
    [{'label': 'A', 'view': 'front', 'image': '', 'mask': 'mask.png'}],
])
def test_incomplete_reference_reports_input_error(tmp_path, recognition, rows):
    _, _, DollRecognizer = recognition
    path = tmp_path / 'refs.json'
    path.write_text(json.dumps(rows))
    with pytest.raises(ValueError, match='reference needs label, view, image and mask'):
        DollRecognizer(path)


def test_video_results_are_valid_json(tmp_path, recognition, monkeypatch):
    cv2, np, _ = recognition
    from roport_smartcart import doll_recognition

    source = tmp_path / 'source.avi'
    writer = cv2.VideoWriter(str(source), cv2.VideoWriter_fourcc(*'MJPG'), 10, (100, 80))
    assert writer.isOpened()
    for _ in range(3):
        writer.write(np.zeros((80, 100, 3), np.uint8))
    writer.release()

    class EmptyDetector:
        def __init__(self, manifest):
            pass

        def detect(self, frame):
            return []

    monkeypatch.setattr(doll_recognition, 'DollRecognizer', EmptyDetector)
    output = tmp_path / 'result'
    doll_recognition.main(['--references', str(tmp_path / 'unused.json'),
                           '--video', str(source), '--output', str(output)])
    records = json.loads((output / 'detections.json').read_text())
    summary = json.loads((output / 'summary.json').read_text())
    assert [row['frame'] for row in records] == [0, 1, 2]
    assert summary['frames'] == 3
    assert summary['detected_frames'] == {'A': 0, 'B': 0, 'C': 0}
