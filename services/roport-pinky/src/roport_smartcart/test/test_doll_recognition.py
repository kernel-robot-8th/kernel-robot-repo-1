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
