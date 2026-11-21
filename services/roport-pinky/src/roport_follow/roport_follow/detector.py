"""노트북 YOLO 추론(선택). ultralytics 가 없으면 Detector 를 만들 때 알려 준다."""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

DEFAULT_MODEL = '~/dev_ws/roport/models/yolo11n.pt'   # 저장소 밖(가중치는 커밋 금지)
COCO_PERSON, COCO_TEDDY = 0, 77


@dataclass(frozen=True)
class Detection:
    cls: int
    name: str
    conf: float
    box: tuple          # x1, y1, x2, y2 화소


class Detector:
    def __init__(self, model=DEFAULT_MODEL, classes=(COCO_PERSON, COCO_TEDDY), conf=0.25, imgsz=640, threads=0):
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise RuntimeError('ultralytics 없음: ~/dev_ws/roport/venvs/follow/bin/python 으로 실행') from e
        path = Path(model).expanduser()
        if not path.exists():
            # 처음 한 번: 공식 가중치를 모델 폴더로 내려받는다(ultralytics 는 현재 폴더에 받음)
            path.parent.mkdir(parents=True, exist_ok=True)
            cwd = os.getcwd()
            try:
                os.chdir(path.parent)
                YOLO(path.name)
            finally:
                os.chdir(cwd)
        if threads:
            # CPU 추론 스레드 수. 노트북(8코어)에서 기본값(전부)은 RViz·브라우저와 다투어 오히려 느림:
            # 11-19 측정 4개 41 ms vs 8개 129 ms(느린 10 % 330 ms) → 실로봇에서 4 로 확인할 것
            import torch
            torch.set_num_threads(int(threads))
        self.threads = int(threads) if threads else None
        self.model = YOLO(str(path))
        # 직접 학습한 인형 모델(adult·child·human)은 COCO 번호 필터를 쓰지 않음
        if classes is not None and len(self.model.names) != 80:
            classes = None
        self.classes, self.conf, self.imgsz = list(classes) if classes else None, float(conf), int(imgsz)

    def __call__(self, image):
        r = self.model.predict(image, imgsz=self.imgsz, conf=self.conf, classes=self.classes,
                               verbose=False)[0]
        out = []
        for b in r.boxes:
            c = int(b.cls[0])
            out.append(Detection(c, r.names.get(c, str(c)), float(b.conf[0]),
                                 tuple(float(v) for v in b.xyxy[0].tolist())))
        return out
