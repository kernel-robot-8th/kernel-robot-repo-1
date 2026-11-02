"""Read ROS trinary PNG/PGM maps without changing their frame or source files."""

import base64
import math
from pathlib import Path

import yaml
from PyQt6.QtCore import QBuffer, QIODevice
from PyQt6.QtGui import QImage, QImageReader


def pixel_to_map(column, row, *, height, resolution, origin):
    x, y = (column + 0.5) * resolution, (height - row - 0.5) * resolution
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return origin[0] + c * x - s * y, origin[1] + s * x + c * y


def load_map(path: str | Path) -> dict:
    path = Path(path).resolve()
    if path.stat().st_size > 64 * 1024:
        raise ValueError("map YAML exceeds 64 KiB")
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict) or config.get("mode", "trinary") != "trinary":
        raise ValueError("only ROS trinary maps are supported")
    resolution, origin = config.get("resolution"), config.get("origin")
    if (
        type(resolution) not in (int, float)
        or not math.isfinite(resolution)
        or not 0 < resolution <= 10
    ):
        raise ValueError("positive finite map resolution required")
    if (
        not isinstance(origin, list)
        or len(origin) != 3
        or not all(type(n) in (int, float) and math.isfinite(n) for n in origin)
    ):
        raise ValueError("map origin requires [x, y, yaw]")
    free, occupied = config.get("free_thresh"), config.get("occupied_thresh")
    if (
        not all(type(n) in (int, float) and math.isfinite(n) for n in [free, occupied])
        or not 0 <= free < occupied <= 1
    ):
        raise ValueError("invalid map occupancy thresholds")
    if type(config.get("negate")) is not int or config["negate"] not in (0, 1):
        raise ValueError("negate must be 0 or 1")
    if not isinstance(config.get("image"), str) or not config["image"]:
        raise ValueError("map image filename required")
    image_path = (path.parent / config["image"]).resolve()
    if (
        image_path.suffix.lower() not in (".png", ".pgm")
        or image_path.stat().st_size > 16 * 1024 * 1024
    ):
        raise ValueError("PNG/PGM image up to 16 MiB required")
    reader = QImageReader(str(image_path))
    size = reader.size()
    if (
        not 0 < size.width() <= 4096
        or not 0 < size.height() <= 4096
        or size.width() * size.height() > 1_000_000
    ):
        raise ValueError("map image must have at most one million pixels")
    image = reader.read().convertToFormat(QImage.Format.Format_RGBA8888)
    if image.isNull():
        raise ValueError("map image cannot be decoded")
    w, h = image.width(), image.height()
    raw = image.constBits().asstring(image.sizeInBytes())
    walls = []
    for row in range(h):
        start = None
        for col in range(w + 1):
            is_wall = False
            if col < w:
                offset = row * image.bytesPerLine() + col * 4
                gray = sum(raw[offset : offset + 3]) / (3 * 255)
                probability = gray if config["negate"] else 1 - gray
                is_wall = raw[offset + 3] == 255 and probability > occupied
            if is_wall and start is None:
                start = col
            if not is_wall and start is not None:
                walls.append([start, row, col - start])
                start = None
        if len(walls) > 20000:
            raise ValueError("map exceeds the 20000 wall-run display limit")
    output = QBuffer()
    output.open(QIODevice.OpenModeFlag.WriteOnly)
    if not image.save(output, "PNG"):
        raise ValueError("map preview encoding failed")
    return {
        "id": path.stem,
        "frame_id": "map",
        "width": w,
        "height": h,
        "resolution": resolution,
        "origin": origin,
        "walls": walls,
        "wall_height": 0.35,
        "height_verified": False,
        "image": "data:image/png;base64,"
        + base64.b64encode(bytes(output.data())).decode(),
        "source": str(path),
    }
