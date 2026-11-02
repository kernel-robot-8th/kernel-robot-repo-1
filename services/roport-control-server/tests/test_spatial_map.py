import math
import os
from pathlib import Path

import pytest

if os.environ.get("REQUIRE_UI_TESTS") != "1":
    pytest.importorskip("PyQt6")
    pytest.importorskip("yaml")
import yaml
from PyQt6.QtGui import QColor, QImage

from roport_control_server.spatial_map import load_map, pixel_to_map


def test_pixel_centres_ros_y_flip_origin_and_rotation():
    assert pixel_to_map(0, 0, height=2, resolution=0.5, origin=[-1, 2, 0]) == (
        -0.75,
        2.75,
    )
    assert pixel_to_map(
        0, 0, height=2, resolution=0.5, origin=[-1, 2, math.pi / 2]
    ) == pytest.approx((-1.75, 2.25))


@pytest.fixture
def map_path(tmp_path):
    image = QImage(4, 3, QImage.Format.Format_RGB32)
    image.fill(QColor("white"))
    image.setPixelColor(1, 0, QColor("black"))
    image.setPixelColor(2, 0, QColor("black"))
    image.save(str(tmp_path / "map.png"))
    path = tmp_path / "map.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "image": "map.png",
                "mode": "trinary",
                "origin": [-1, 2, 0.3],
                "resolution": 0.05,
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
        )
    )
    return path


def test_wall_runs_and_original_files_preserved(map_path):
    original = map_path.read_bytes()
    result = load_map(map_path)
    assert result["walls"] == [[1, 0, 2]]
    assert result["origin"] == [-1, 2, 0.3]
    assert result["image"].startswith("data:image/png;base64,")
    assert result["height_verified"] is False
    assert map_path.read_bytes() == original


@pytest.mark.parametrize(
    "changes",
    [
        {"resolution": float("nan")},
        {"origin": [0, 0]},
        {"negate": True},
        {"mode": "raw"},
        {"free_thresh": 0.9},
    ],
)
def test_bad_map_metadata_rejected(map_path, changes):
    data = yaml.safe_load(map_path.read_text())
    data.update(changes)
    map_path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_map(map_path)


def test_asset_bundle_contains_pinned_models():
    import hashlib
    import json

    from roport_control_server import spatial_map

    assets = Path(spatial_map.__file__).with_name("spatial_assets")
    for path, expected in json.loads((assets / "checksums.json").read_text()).items():
        assert hashlib.sha256((assets / path).read_bytes()).hexdigest() == expected
    for kind in ("pinky", "omx"):
        assert (assets / "models" / kind / "robot.glb").read_bytes()[:4] == b"glTF"
        assert (
            len(json.loads((assets / "models" / kind / "joints.json").read_text())) >= 2
        )
        assert (assets / "models" / kind / "LICENSE").is_file()


def test_pinky_visuals_preserve_source_triangles():
    import json
    import struct
    import xml.etree.ElementTree as ET

    from roport_control_server import spatial_map

    model = Path(spatial_map.__file__).with_name("spatial_assets") / "models/pinky"
    source = Path(__file__).parents[2] / "roport-pinky/src/pinky_pro/pinky_description"
    data = (model / "robot.glb").read_bytes()
    length, chunk_type = struct.unpack_from("<II", data, 12)
    assert chunk_type == 0x4E4F534A
    gltf = json.loads(data[20 : 20 + length])
    nodes = {node["name"]: node for node in gltf["nodes"]}
    # Reducing the whole DAE assembly to 8000 faces previously removed body panels.
    for link in ET.parse(model / "robot.urdf").findall("link"):
        for index, visual in enumerate(link.findall("visual")):
            mesh_path = visual.find("geometry/mesh").get("filename").split("/", 3)[3]
            original = ET.parse(source / mesh_path)
            assert not original.findall(".//{*}polylist")
            expected = sum(
                int(t.get("count")) for t in original.findall(".//{*}triangles")
            )
            node = nodes[f"{link.get('name')}_visual_{index}"]
            primitives = gltf["meshes"][node["mesh"]]["primitives"]
            assert all(p.get("mode", 4) == 4 for p in primitives)
            actual = sum(
                gltf["accessors"][p["indices"]]["count"] // 3 for p in primitives
            )
            assert actual == expected, link.get("name")
