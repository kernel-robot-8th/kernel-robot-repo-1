"""양팔 UDP 추론 클라이언트: 팀 infer_link(RPI1)로 GPU 추론 서버에 관측 1개를 보낸다.

PR #115 UdpInferClient(OMX3)와 같은 방식이고 형식만 양팔이다:
state 12개 + 카메라마다 JPEG 1장(RGB → BGR, 품질 90, LOADED의 image_size가 있으면 INTER_AREA 축소),
robot_type "bi_omx_follower", 응답 actions(N x 12)를 돌려준다.
응답이 제때 오지 않으면 같은 관측을 새 요청 번호로 다시 보내고(attempts번), 그래도 없으면 PolicyUnavailable.
Panel(omx_infer.UdpRemoteClient)의 현장 값: 요청마다 1.5초, 3번.
"""

from __future__ import annotations

import logging
import math
import stat
import threading
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

from ..infer_link.client import InferenceClient, InferenceError, InferenceTimeout
from ..infer_link.wire import check_key
from .contract import CAMERAS, ROBOT_TYPE, WIDTH
from .ports import PolicyUnavailable

log = logging.getLogger(__name__)
Encoder = Callable[[Any, tuple[int, int] | None, int], bytes]


def read_key(path: str | Path) -> bytes:
    """공유 키 파일(16진수 한 줄, chmod 600). 저장소에 두지 않는다."""
    path = Path(path).expanduser()
    if path.stat().st_mode & (stat.S_IRWXG | stat.S_IRWXO):
        raise ValueError(f"{path} must be readable by its owner only (chmod 600)")
    return check_key(bytes.fromhex(path.read_text().strip()))


def encode_jpeg(rgb, size: tuple[int, int] | None, quality: int) -> bytes:
    import cv2

    if size and tuple(rgb.shape[:2]) != tuple(size):
        rgb = cv2.resize(rgb, (size[1], size[0]), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(
        ".jpg",
        cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
        [cv2.IMWRITE_JPEG_QUALITY, quality],
    )
    if not ok:
        raise ValueError("JPEG encoding failed")
    return buf.tobytes()


class BiUdpInferClient:
    def __init__(
        self,
        server: tuple[str, int],
        key: bytes,
        model: str,
        *,
        cameras: Sequence[str] = CAMERAS,
        chunk: int = 100,  # 모델 chunk_size (현장 ACT 100)
        timeout_s: float = 1.5,
        attempts: int = 3,
        jpeg_quality: int = 90,
        client: InferenceClient
        | None = None,  # 테스트용. 넘기면 server·key는 쓰지 않는다
        encoder: Encoder = encode_jpeg,
    ):
        if attempts < 1:
            raise ValueError("attempts must be >= 1")
        self.client = client or InferenceClient(server, key)
        self.model, self.cameras, self.chunk = model, tuple(cameras), chunk
        self.timeout_s, self.attempts = timeout_s, attempts
        self.jpeg_quality, self.encoder = jpeg_quality, encoder
        self.context: dict | None = (
            None  # 실행 중인 MQTT 명령의 task/command/execution id
        )
        self.timeouts = 0
        self._lock = threading.Lock()
        self._image_size: tuple[int, int] | None = None
        self._loaded = False

    def load(self, timeout: float = 180.0) -> dict:
        """주행 전에 모델을 GPU에 올린다. 재로드가 실패하면 그 뒤 추론은 보내지 않는다."""
        with self._lock:
            self._loaded, self._image_size = False, None
            info = self.client.load(self.model, timeout=timeout)
            if info.get("n_obs", 1) != 1:  # 관측 1개만 보낸다(ACT). 0·None도 거절
                raise ValueError(f"model needs {info.get('n_obs')} observations")
            size = info.get("image_size")
            self._image_size = (int(size[0]), int(size[1])) if size else None
            self._loaded = True
        return info

    def infer(
        self, state: Sequence[float], images: Mapping[str, Any], task: str
    ) -> list[list[float]]:
        if len(state) != WIDTH:
            raise ValueError(f"state needs {WIDTH} values")
        missing = [c for c in self.cameras if c not in images]
        if missing:
            raise ValueError(f"missing camera frames: {missing}")
        with self._lock:
            if not self._loaded:
                raise RuntimeError("call load() before infer()")
            blobs, refs, offset = [], {}, 0
            for cam in self.cameras:
                jpeg = self.encoder(images[cam], self._image_size, self.jpeg_quality)
                refs[f"observation.images.{cam}"] = [offset, len(jpeg)]
                blobs.append(jpeg)
                offset += len(jpeg)
            header = {
                "model": self.model,
                "overrides": {},  # load()와 같게
                "task": task,
                "robot_type": ROBOT_TYPE,
                "obs": [{"state": [float(v) for v in state], "images": refs}],
                "chunk": self.chunk,
                "rtc": {"delay": 0, "left": None},  # Panel과 같음(ACT는 RTC 안 씀)
            }
            data = b"".join(blobs)
            for attempt in range(self.attempts):
                try:
                    reply = self.client.infer(
                        header, data, timeout=self.timeout_s, context=self.context
                    )
                    break
                except InferenceTimeout:
                    self.timeouts += 1
                    if attempt == self.attempts - 1:
                        raise PolicyUnavailable(
                            f"no inference reply ({self.attempts} x {self.timeout_s:g}s)"
                        ) from None
                except InferenceError as exc:
                    raise PolicyUnavailable(f"inference error {exc.code}") from exc
        if (reply.get("load_ms") or 0) > 0:
            log.warning(
                "GPU server reloaded %s during a run (load_ms=%s)",
                self.model,
                reply["load_ms"],
            )
        actions = reply.get("actions")
        if not isinstance(actions, list) or not actions:
            raise PolicyUnavailable("reply has no actions")
        out = []
        for row in actions:
            if not isinstance(row, list) or len(row) != WIDTH:
                raise PolicyUnavailable(f"action rows must have {WIDTH} values")
            values = [float(v) for v in row]
            if not all(math.isfinite(v) for v in values):
                raise PolicyUnavailable("non-finite action value")
            out.append(values)
        return out

    def close(self) -> None:
        self.client.close()
