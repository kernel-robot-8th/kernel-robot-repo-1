"""원본 녹화(v): 파일에는 겹쳐 그리기 전 영상만 들어간다(카메라·로봇 없음)."""
from types import SimpleNamespace

import cv2
import numpy as np

from roport_follow.floor_geom import CameraModel
from roport_follow.session import ViewSession


class FakeSource:
    def __init__(self, img):
        self.img, self.seq = img, 0

    def latest(self):
        self.seq += 1
        return SimpleNamespace(seq=self.seq, image=self.img)


def _args(tmp_path):
    return SimpleNamespace(out=str(tmp_path), name='t', save_dir=None, card_mm=30.0, yolo=False,
                           prefix='cust_card', burst_sec=0.5, fps=10.0, lens_height_cm=0.0,
                           camera=str(tmp_path / 'camera.json'))


def test_raw_recording_has_no_overlay(tmp_path):
    rng = np.random.default_rng(0)
    raw = cv2.GaussianBlur(rng.integers(0, 255, (480, 640, 3), dtype=np.uint8), (31, 31), 0)
    ses = ViewSession(_args(tmp_path), CameraModel(), FakeSource(raw))
    shown = ses.step()
    assert np.abs(shown.astype(int) - raw).max() > 100          # 화면에는 격자·글자가 그려짐
    assert 'raw recording ON' in ses.key('v')
    for _ in range(5):
        ses.step()
    st = ses.state()
    assert st['rec'] and st['rec_frames'] == 5
    ses.key('v')                                                 # 다시 누르면 닫힘
    assert not ses.state()['rec']
    path = tmp_path / 'video'
    files = list(path.glob('cust_card_*_raw.mp4'))
    assert len(files) == 1
    cap = cv2.VideoCapture(str(files[0]))
    n, first = 0, None
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        first = fr if first is None else first
        n += 1
    assert n == 5
    assert np.abs(first.astype(int) - raw).mean() < 4             # 압축 오차만(격자선 없음)


def test_close_without_recording_is_noop(tmp_path):
    ses = ViewSession(_args(tmp_path), CameraModel(), FakeSource(np.zeros((480, 640, 3), np.uint8)))
    ses.close()
    assert not (tmp_path / 'video').exists()
