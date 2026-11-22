"""inference.InferClient 구현: 팀 infer_link(UDP, RPI1)로 GPU 추론 서버에 관측 1개를 보낸다.

형식은 README 「INFER 요청·응답 형식」을 따른다: state 6개 + 원 그린 정면 영상 JPEG 1장
(RGB → BGR, 품질 90, LOADED의 image_size로 INTER_AREA 축소), 응답 actions(N x 6)를 그대로 돌려준다.
InferenceClient는 스레드에 안전하지 않으므로 호출을 잠금으로 하나씩 보낸다.
모델은 주행 전에 load()로 올린다(다른 Host의 추론을 막지 않게, README).
"""

from __future__ import annotations

import logging
import stat
import threading
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import cv2

from ...infer_link.client import InferenceClient, InferenceTimeout
from ...infer_link.wire import check_key

log = logging.getLogger(__name__)
FRONT = "observation.images.front"  # 마커 ACT 모델의 유일한 카메라 입력


def read_key(path: str | Path) -> bytes:
    """공유 키 파일(16진수 한 줄, chmod 600). 서버 read_key와 같은 규칙. 저장소에 두지 않는다."""
    path = Path(path).expanduser()
    if path.stat().st_mode & (stat.S_IRWXG | stat.S_IRWXO):
        raise ValueError(f"{path} must be readable by its owner only (chmod 600)")
    return check_key(bytes.fromhex(path.read_text().strip()))


class UdpInferClient:
    def __init__(
        self,
        server: tuple[str, int],
        key: bytes,
        model: str,
        *,
        chunk: int = 100,  # 모델 chunk_size. 빼면 서버가 50으로 자른다
        timeout_s: float = 1.0,  # 요청 1개의 응답 대기. 넘으면 ChunkPolicy가 새 관측으로 재요청
        jpeg_quality: int = 90,  # 제어판 기본값(학습·평가와 같게)
        # 테스트용. 넘기면 server·key는 쓰지 않는다
        client: InferenceClient | None = None,
    ):
        self.client = client or InferenceClient(server, key)
        self.model, self.chunk = model, chunk
        self.timeout_s, self.jpeg_quality = timeout_s, jpeg_quality
        self._lock = threading.Lock()
        self._image_size: tuple[int, int] | None = None
        self._loaded = False

    def load(self, timeout: float = 180.0) -> dict:
        # 초기화·응답 검증·적용을 한 잠금 안에서: 재로드가 실패하면 그 뒤 추론은 보내지 않는다
        with self._lock:
            self._loaded, self._image_size = False, None
            info = self.client.load(self.model, timeout=timeout)
            if info.get("n_obs", 1) != 1:  # 관측 1개만 보낸다(ACT). 0·None도 거절
                raise ValueError(f"model needs {info['n_obs']} observations")
            size = info.get("image_size")
            self._image_size = (int(size[0]), int(size[1])) if size else None
            self._loaded = True
        return info

    def infer(
        self, state: Sequence[float], marked_front: Any, task: str, robot_type: str
    ) -> Sequence[Sequence[float]]:
        # 잠금 대기와 UDP 요청이 같은 마감 시각을 쓴다: 전체가 timeout_s 안에 끝난다(InferClient 약속)
        deadline = time.monotonic() + self.timeout_s
        if not self._lock.acquire(timeout=self.timeout_s):
            raise InferenceTimeout("another request holds the link")
        try:
            # 준비 상태와 영상 크기는 잠금을 얻은 뒤의 값으로 본다(load()와 겹쳐도 섞이지 않게)
            if not self._loaded:
                raise RuntimeError("call load() before infer()")
            jpeg = self._encode(marked_front)
            header = {
                "model": self.model,
                "overrides": {},  # load()와 같게
                "task": task,
                "robot_type": robot_type,
                "obs": [
                    {
                        "state": [float(v) for v in state],
                        "images": {FRONT: [0, len(jpeg)]},
                    }
                ],
                "chunk": self.chunk,
                "rtc": None,
            }
            left = deadline - time.monotonic()
            if left <= 0:
                raise InferenceTimeout("no time left after waiting for the link")
            reply = self.client.infer(header, jpeg, timeout=left)
        finally:
            self._lock.release()
        if (reply.get("load_ms") or 0) > 0:
            # 서버 캐시(2개, LRU)에서 밀려나 이 요청 때 다시 올림. 1초 넘게 걸리면 시간 초과로 이 응답은 못 받는다
            log.warning(
                "GPU server reloaded %s during a run (load_ms=%s); load models before driving",
                self.model,
                reply["load_ms"],
            )
        return reply["actions"]

    def close(self) -> None:
        self.client.close()

    def _encode(self, rgb) -> bytes:
        if self._image_size and tuple(rgb.shape[:2]) != self._image_size:
            h, w = self._image_size
            rgb = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_AREA)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(
            ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            raise ValueError("JPEG encoding failed")
        return buf.tobytes()
