import json
from types import SimpleNamespace as NS
import sys

import numpy as np
import pytest

from roport_smartcart.appearance_reid import AppearanceGallery, load_gallery, normalized


def test_views_of_one_identity_do_not_compete_and_absence_is_not_a_match():
    gallery = AppearanceGallery(['santa', 'santa', 'barbie'],
                                [[1., 0., 0.], [.99, .01, 0.], [0., 1., 0.]])
    matches = gallery.classify([[10., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    assert [m['label'] for m in matches] == ['santa', 'barbie', None]
    assert gallery.select('santa', matches) == {'state': 'MATCH', 'index': 0}
    assert gallery.select('santa', matches[1:]) == {'state': 'LOST', 'index': None}
    assert gallery.select('santa', [matches[0], matches[0]]) == {'state': 'AMBIGUOUS', 'index': None}
    with pytest.raises(ValueError, match='not registered'):
        gallery.select('unknown', matches)


def test_similar_candidates_are_rejected_instead_of_picking_a_name():
    gallery = AppearanceGallery(['santa', 'barbie'], [[1.,0.], [0.,1.]])
    assert gallery.classify([[1.,1.]])[0]['label'] is None
    assert gallery.classify(np.empty((0,2))) == []
    with pytest.raises(ValueError, match='dimensions'):
        gallery.classify([[1.,0.,0.]])


@pytest.mark.parametrize('features', [[[0.,0.]], [[float('nan'), 1.]],
                                     [[float('inf'), 1.]], [1.,2.]])
def test_invalid_features_never_produce_a_match(features):
    with pytest.raises(ValueError):
        normalized(features)


@pytest.mark.parametrize('labels,threshold,margin', [([], .7, .05),
    ([''], .7, .05), (['a','b'], .7, .05), (['a'], 2., .05), (['a'], .7, 0.),
    (['a'], float('nan'), .05)])
def test_invalid_gallery_rejected(labels, threshold, margin):
    with pytest.raises(ValueError):
        AppearanceGallery(labels, [[1.,0.]], threshold, margin)


def test_missing_reference_does_not_create_a_gallery(tmp_path):
    path = tmp_path/'references.json'
    path.write_text(json.dumps([{'label':'santa','image':'missing.png'}]))
    with pytest.raises(ValueError, match='unavailable'):
        load_gallery(path, object())


def test_analysis_does_not_encode_tiny_background_drawings():
    from roport_smartcart.reid_video import analyze
    calls = []
    def predict(frame, **kwargs):
        assert kwargs['agnostic_nms'] is True
        return [NS(boxes=NS(xyxy=NS(cpu=lambda: NS(
            tolist=lambda: [[1,1,20,40],[5,5,70,75]]))))]
    def encode(crops):
        calls.append(len(crops))
        return np.array([[1.,0.] for _ in crops])
    gallery = AppearanceGallery(['santa','barbie'], [[1.,0.],[0.,1.]])
    boxes, matches, selected = analyze(np.zeros((80,80,3),np.uint8),
        NS(predict=predict), NS(encode=encode), gallery, 'santa')
    assert calls == [1] and boxes == [[5,5,70,75]]
    assert selected['state'] == 'MATCH'


def test_offline_pipeline_reports_loss_and_reacquisition_without_ai_download(tmp_path, monkeypatch):
    import cv2
    from roport_smartcart import reid_video

    class Encoder:
        def __init__(self, *args):
            pass

        def encode(self, crops):
            return np.array([[1.,0.] if c[:,:,2].mean() > c[:,:,0].mean()
                             else [0.,1.] for c in crops])

    class Detector:
        def __init__(self, *args, **kwargs):
            pass

        def predict(self, frame, **kwargs):
            boxes = [[5.,5.,70.,70.]] if frame.max() > 30 else []
            return [NS(boxes=NS(xyxy=NS(cpu=lambda: NS(tolist=lambda: boxes))))]

    monkeypatch.setattr(reid_video, 'DinoEncoder', Encoder)
    monkeypatch.setitem(sys.modules, 'ultralytics', NS(YOLO=Detector))
    red = np.full((80,80,3), (0,0,200), np.uint8)
    blue = np.full((80,80,3), (200,0,0), np.uint8)
    blank = np.zeros_like(red)
    cv2.imwrite(str(tmp_path/'santa.png'), red)
    cv2.imwrite(str(tmp_path/'barbie.png'), blue)
    references = tmp_path/'refs.json'
    references.write_text(json.dumps([{'label':name, 'image':name+'.png'}
                                     for name in ('santa','barbie')]))
    video = tmp_path/'source.avi'
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'MJPG'), 10., (80,80))
    assert writer.isOpened()
    for frame in (red, blank, red, blue, red, red):
        writer.write(frame)
    writer.release()
    model, weights = tmp_path/'local.pt', tmp_path/'local.pth'
    model.write_bytes(b'synthetic')
    weights.write_bytes(b'synthetic')
    output = tmp_path/'new-result'
    args = ['--references',str(references),'--encoder-source',str(tmp_path),
            '--encoder-weights',str(weights),'--model',str(model),'--video',str(video),
            '--target','santa','--output',str(output),'--stride','1']
    assert reid_video.main(args) == 0
    rows = [json.loads(line) for line in (output/'frames.jsonl').read_text().splitlines()]
    assert [r['state'] for r in rows] == ['MATCH','LOST','REACQUIRED','LOST','REACQUIRED','MATCH']
    summary = json.loads((output/'summary.json').read_text())
    assert summary['processed'] == 6 and summary['reacquisitions'] == 2
    assert 'NOT accuracy' in summary['note']
    capture = cv2.VideoCapture(str(output/'preview.mp4'))
    assert capture.isOpened() and capture.get(cv2.CAP_PROP_FRAME_COUNT) == 6
    capture.release()
    original = (output/'frames.jsonl').read_bytes()
    with pytest.raises(SystemExit):
        reid_video.main(args)
    assert (output/'frames.jsonl').read_bytes() == original
