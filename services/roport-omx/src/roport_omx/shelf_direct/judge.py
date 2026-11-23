"""놓음 판정(end_on_place): Panel omx_run.PlaceWatch를 옮겼다. 판정 자체는 panel_vendor/handover_judge.py 그대로.

위 카메라(front) 원본 프레임과 양팔 관절로 상자가 선반 칸에 안착하고 손끝이 빠졌는지를 본다.
제어 루프가 판정을 기다리지 않게 별도 스레드에서 rate_hz(6 Hz)로 돌리고, 루프는 결과만 확인한다.
shadow=True면 기록만 하고 에피소드를 끝내지 않는다. 시각은 Panel처럼 벽시계(time.time)를 쓴다.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Sequence
from datetime import UTC, datetime

from .contract import STATE_KEYS
from .ports import Observation


def _iso(ts: float | None) -> str | None:
    if ts is None:
        return None
    return (
        datetime.fromtimestamp(ts, UTC)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def panel_judge(box_ids: Sequence[int]):
    from .panel_vendor import handover_judge

    return handover_judge.Judge(
        None, box_ids=tuple(int(b) for b in box_ids)
    ), handover_judge.VERSION


def rgb_to_gray(img):
    import cv2

    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) if img.ndim == 3 else img


class PlaceWatch:
    def __init__(
        self,
        box_ids: Sequence[int],
        *,
        camera: str = "front",
        rate_hz: float = 6.0,
        shadow: bool = False,
        judge_factory: Callable = panel_judge,
        to_gray: Callable = rgb_to_gray,
        copy: Callable = lambda img: img.copy(),
        wall: Callable[[], float] = time.time,
    ):
        self.camera, self.period, self.shadow = camera, 1.0 / rate_hz, shadow
        self.judge, self.version = judge_factory(box_ids)
        self.to_gray, self.copy, self.wall = to_gray, copy, wall
        self.lock, self.new = threading.Lock(), threading.Event()
        self.frame = self.joints = self.t = self.frame_ts = None
        self.taken_at = 0.0
        self.hit: dict | None = None
        self.last: dict | None = None
        self.updates = self.errors = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def reset(self, t0: float | None = None) -> None:
        with self.lock:
            self.judge.reset(self.wall() if t0 is None else t0)
            self.hit, self.last, self.updates, self.errors, self.frame = (
                None,
                None,
                0,
                0,
                None,
            )
            self.taken_at = 0.0

    def feed(self, obs: Observation, t: float | None = None) -> None:
        """매 제어 주기에 부른다. 판정 스레드가 쉴 때만 프레임을 복사해 넘긴다."""
        t = self.wall() if t is None else t
        if t - self.taken_at < self.period or self.new.is_set():
            return
        img = obs.images.get(self.camera)
        if img is None:
            return
        joints = {
            k.removesuffix(".pos"): float(v)
            for k, v in zip(STATE_KEYS, obs.state, strict=True)
        }
        with self.lock:
            self.frame, self.joints, self.t, self.frame_ts = (
                self.copy(img),
                joints,
                t,
                obs.at or None,
            )
        self.taken_at = t
        self.new.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            if not self.new.wait(0.2):
                continue
            with self.lock:
                img, joints, t, frame_ts = (
                    self.frame,
                    self.joints,
                    self.t,
                    self.frame_ts,
                )
            try:
                r = self.judge.update(self.to_gray(img), joints, t)
            except Exception as exc:  # noqa: BLE001 - 판정기는 오류를 키로 돌려준다. 마지막 방어
                r = {"placed": False, "error": f"{type(exc).__name__}: {exc}"[:160]}
            with self.lock:
                self.updates += 1
                self.errors += bool(r.get("error"))
                self.last = r
                if r.get("placed") and self.hit is None:
                    self.hit = dict(
                        r,
                        judged_ts=t,
                        judged_at=_iso(t),
                        joints=joints,
                        version=self.version,
                        frame_ts=frame_ts,
                        frame_at=_iso(frame_ts),
                    )
            self.new.clear()

    def placed(self) -> dict | None:
        with self.lock:
            return self.hit

    def end_check(self, obs: Observation, _now: float) -> str | None:
        """ShelfRunner end_checks에 넣는다: 판정되면 "placed"(shadow면 계속)."""
        self.feed(obs)
        return "placed" if self.placed() is not None and not self.shadow else None

    def summary(self) -> dict:
        with self.lock:
            return {
                "version": self.version,
                "shadow": self.shadow,
                "updates": self.updates,
                "errors": self.errors,
                "hit": self.hit,
                "last": self.last,
            }

    def close(self) -> None:
        self._stop.set()
        self._thread.join(1.0)
