"""Per-frame diagnostics of a model run: pose, command, inference time, occasional camera frames.

Diagnostics only. Every method swallows its own errors and switches tracing off, so a full disk
or an odd value can never stop or delay the control loop beyond one failed write.
"""

import json
import time
from contextlib import suppress
from pathlib import Path


class RunTrace:
    @classmethod
    def open(cls, directory, *, snapshot_every=20):
        if not directory:
            return None
        try:
            folder = Path(directory) / time.strftime("%Y%m%d-%H%M%S")
            folder.mkdir(parents=True, exist_ok=True)
            return cls(folder, snapshot_every)
        except Exception:  # noqa: BLE001 - diagnostics must not prevent a run
            return None

    def __init__(self, folder, snapshot_every):
        self.folder = folder
        self.snapshot_every = snapshot_every
        self.file = (folder / "trace.jsonl").open("w")
        self.started = time.monotonic()
        self.enabled = True

    def frame(self, index, state, action, inference_s, observation):
        if not self.enabled:
            return
        try:
            self.file.write(
                json.dumps(
                    {
                        "frame": index,
                        "t": round(time.monotonic() - self.started, 4),
                        "state": [float(value) for value in state],
                        "action": [float(value) for value in action],
                        "inference_ms": round(inference_s * 1000),
                    }
                )
                + "\n"
            )
            if index % self.snapshot_every == 0:
                self._snapshot(index, observation)
        except Exception:  # noqa: BLE001 - never raise into the control loop
            self.enabled = False

    def _snapshot(self, index, observation):
        import numpy as np

        for key, value in observation.items():
            if not key.startswith("observation.images."):
                continue
            name = f"{index:06d}-{key.rsplit('.', 1)[-1]}"
            frame = np.asarray(value)
            try:
                import cv2

                # Camera frames are RGB; OpenCV writes BGR.
                cv2.imwrite(str(self.folder / f"{name}.jpg"), frame[..., ::-1])
            except ImportError:
                np.save(self.folder / f"{name}.npy", frame)

    def close(self):
        self.enabled = False
        # Closing diagnostics is best effort.
        with suppress(Exception):
            self.file.close()
