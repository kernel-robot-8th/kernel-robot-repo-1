import math

import pytest
import yaml
from PIL import Image

from roport_control_server.adapters.api.maps import load_map, pixel_to_map


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
    image = Image.new("RGB", (4, 3), "white")
    image.putpixel((1, 0), (0, 0, 0))
    image.putpixel((2, 0), (0, 0, 0))
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
