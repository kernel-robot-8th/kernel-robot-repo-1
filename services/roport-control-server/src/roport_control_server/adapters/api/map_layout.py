"""Read measured layout metadata for display, without importing ROS or planning routes."""

import hashlib
import math
import re
from pathlib import Path

import yaml


def load_layout(path, grid):
    path = Path(path)
    if path.stat().st_size > 64 * 1024:
        raise ValueError("layout YAML exceeds 64 KiB")
    raw = path.read_bytes()
    value = yaml.safe_load(raw)

    def number(value, low=-100000, high=100000):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("layout requires finite numbers")
        if not low <= value <= high:
            raise ValueError("layout value out of range")
        return value

    def pair(value):
        if not isinstance(value, list) or len(value) != 2:
            raise ValueError("layout coordinates require [x, y]")
        return [number(v) for v in value]

    def name(value):
        if not isinstance(value, str) or not re.fullmatch(
            r"[A-Za-z0-9_.-]{1,64}", value
        ):
            raise ValueError("invalid layout identifier")
        return value

    try:
        if value["units"] != "cm":
            raise ValueError("layout units must be cm")
        meta = value["map"]
        if (
            any(meta[k] != grid[k] for k in ("width", "height", "resolution"))
            or pair(meta["origin"]) != grid["origin"][:2]
            or grid["origin"][2] != 0
        ):
            raise ValueError("layout and map grids do not match")
        frame = value["frame"]
        tx, ty = number(frame["x_m"]), number(frame["y_m"])
        yaw = math.radians(number(frame["yaw_deg"], -360, 360))
        c, s = math.cos(yaw), math.sin(yaw)

        def position(x, y):
            return [tx + (c * x - s * y) / 100, ty + (s * x + c * y) / 100]

        items = value["nodes"]["items"]
        if not isinstance(items, list) or not 1 <= len(items) <= 256:
            raise ValueError("layout must have 1..256 nodes")
        nodes = []
        for item in items:
            x, y = pair(item["at"])
            x, y = position(x, y)
            if not (
                grid["origin"][0]
                <= x
                <= grid["origin"][0] + grid["width"] * grid["resolution"]
                and grid["origin"][1]
                <= y
                <= grid["origin"][1] + grid["height"] * grid["resolution"]
            ):
                raise ValueError("layout node outside map")
            links = item["links"]
            if not isinstance(links, list) or len(links) > 256:
                raise ValueError("invalid layout links")
            nodes.append(
                {
                    "id": name(item["id"]),
                    "position": [x, y],
                    "place": name(item["place"]) if "place" in item else None,
                    "nav2_switch": item.get("nav2_switch") is True,
                    "links": [name(link) for link in links],
                }
            )
        by_id = {n["id"]: n for n in nodes}
        if len(by_id) != len(nodes):
            raise ValueError("duplicate layout node")
        for node in nodes:
            for target in node["links"]:
                if (
                    target == node["id"]
                    or target not in by_id
                    or node["id"] not in by_id[target]["links"]
                ):
                    raise ValueError(
                        "layout links must be symmetric and reference known nodes"
                    )
        mounts = []
        for item in value.get("objects", []):
            if item["name"] not in ("omx12_left", "omx12_right"):
                continue
            low, high = pair(item["min"]), pair(item["max"])
            if any(a >= b for a, b in zip(low, high, strict=True)):
                raise ValueError("invalid OMX support bounds")
            robot_id = "omx1" if item["name"] == "omx12_left" else "omx2"
            if any(m["robot_id"] == robot_id for m in mounts):
                raise ValueError("duplicate OMX support")
            mounts.append(
                {
                    "robot_id": robot_id,
                    "edge_center": position((low[0] + high[0]) / 2, high[1])
                    + [
                        number(
                            item["height"]
                            if "height" in item
                            else value["defaults"]["object_height"],
                            0.1,
                            1000,
                        )
                        / 100
                    ],
                    "edge_yaw": yaw,
                    "yaw": yaw + (0 if robot_id == "omx1" else math.pi),
                }
            )
        return {
            "version": name(value["version"]),
            "graph_version": name(value["nodes"]["graph_version"]),
            "revision": hashlib.sha256(raw).hexdigest()[:16],
            "wall_height": number(value["defaults"]["wall_height"], 0.1, 1000) / 100,
            "nodes": nodes,
            "mounts": mounts,
        }
    except (KeyError, TypeError) as exc:
        raise ValueError("invalid measured layout") from exc
