"""Read ROS trinary PNG/PGM maps without changing their frame or source files."""

import base64
import hashlib
import math
from io import BytesIO
from pathlib import Path

import yaml
from PIL import Image

from .map_layout import load_layout


def pixel_to_map(column, row, *, height, resolution, origin):
    x, y = (column + 0.5) * resolution, (height - row - 0.5) * resolution
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return origin[0] + c * x - s * y, origin[1] + s * x + c * y


def load_map(path: str | Path, layout_path: str | Path | None = None) -> dict:
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
    with Image.open(image_path) as source:
        w, h = source.size
        if not 0 < w <= 4096 or not 0 < h <= 4096 or w * h > 1_000_000:
            raise ValueError("map image must have at most one million pixels")
        image = source.convert("RGBA")
    raw = image.tobytes()
    walls = []
    for row in range(h):
        start = None
        for col in range(w + 1):
            is_wall = False
            if col < w:
                offset = row * w * 4 + col * 4
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
    output = BytesIO()
    image.save(output, format="PNG")
    result = {
        "id": path.stem,
        "version": hashlib.sha256(
            path.read_bytes() + image_path.read_bytes()
        ).hexdigest()[:16],
        "frame_id": "map",
        "width": w,
        "height": h,
        "resolution": resolution,
        "origin": origin,
        "walls": walls,
        "wall_height": 0.35,
        "height_verified": False,
        "image": "data:image/png;base64,"
        + base64.b64encode(output.getvalue()).decode(),
    }
    if layout_path is not None:
        layout = load_layout(layout_path, result)
        result.update(
            layout=layout, wall_height=layout["wall_height"], height_verified=True
        )
    return result
