import copy
import math

import pytest
import yaml
from fastapi.testclient import TestClient
from PIL import Image

from roport_control_server.adapters.api.app import create_app_from_env
from roport_control_server.adapters.api.display import Display
from roport_control_server.adapters.api.maps import load_map


@pytest.fixture
def measured_map(tmp_path):
    Image.new("RGB", (8, 4), "white").save(tmp_path / "site.png")
    grid = {"width": 8, "height": 4, "resolution": 1.0, "origin": [-2.0, -1.0]}
    map_path = tmp_path / "site.yaml"
    map_path.write_text(
        yaml.safe_dump(
            {
                "image": "site.png",
                "resolution": 1.0,
                "origin": [-2.0, -1.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
        )
    )
    layout = {
        "units": "cm",
        "version": "fixture-v1",
        "map": grid,
        "frame": {"x_m": -2, "y_m": -1, "yaw_deg": 90},
        "defaults": {"wall_height": 24},
        "nodes": {
            "graph_version": "fixture-lines-v1",
            "items": [
                {"id": "N01", "at": [100, -100], "place": "P4", "links": ["N02"]},
                {"id": "N02", "at": [200, -100], "nav2_switch": True, "links": ["N01"]},
            ],
        },
    }
    layout_path = tmp_path / "layout.yaml"
    layout_path.write_text(yaml.safe_dump(layout))
    return map_path, layout_path, layout


def test_measured_transform_height_and_distinct_layout_revision(measured_map):
    path, layout_path, layout = measured_map
    originals = path.read_bytes(), layout_path.read_bytes()
    plain = load_map(path)
    value = load_map(path, layout_path)
    assert value["version"] == plain["version"]
    assert value["wall_height"] == 0.24 and value["height_verified"]
    first, second = value["layout"]["nodes"]
    assert first["position"] == pytest.approx([-1, 0])
    assert second["position"] == pytest.approx([-1, 1])
    assert first["place"] == "P4" and second["nav2_switch"]
    assert (path.read_bytes(), layout_path.read_bytes()) == originals
    layout["nodes"]["items"][0]["at"][0] = 110
    layout_path.write_text(yaml.safe_dump(layout))
    updated = load_map(path, layout_path)
    assert updated["version"] == value["version"]
    assert updated["layout"]["revision"] != value["layout"]["revision"]


def test_omx_mounts_use_forward_edge_and_face_each_other(measured_map):
    path, layout_path, layout = measured_map
    layout["defaults"]["object_height"] = 24
    layout["objects"] = [
        {"name": "omx12_left", "min": [100, -200], "max": [120, -170]},
        {"name": "omx12_right", "min": [150, -200], "max": [170, -170]},
    ]
    layout_path.write_text(yaml.safe_dump(layout))
    left, right = load_map(path, layout_path)["layout"]["mounts"]
    assert left["robot_id"] == "omx1" and right["robot_id"] == "omx2"
    assert left["edge_center"] == pytest.approx([-0.3, 0.1, 0.24])
    assert right["edge_center"] == pytest.approx([-0.3, 0.6, 0.24])
    assert left["edge_yaw"] == right["edge_yaw"] == math.pi / 2
    assert right["yaw"] - left["yaw"] == pytest.approx(math.pi)
    layout["objects"][0]["max"] = layout["objects"][0]["min"]
    layout_path.write_text(yaml.safe_dump(layout))
    with pytest.raises(ValueError, match="support bounds"):
        load_map(path, layout_path)


@pytest.mark.parametrize(
    "fault",
    [
        "units",
        "grid",
        "origin",
        "yaw",
        "height",
        "duplicate",
        "unknown_link",
        "asymmetric",
        "outside",
    ],
)
def test_invalid_or_mismatched_layout_rejected(measured_map, fault):
    path, layout_path, value = measured_map
    nodes = value["nodes"]["items"]
    if fault == "units":
        value["units"] = "m"
    elif fault == "grid":
        value["map"]["width"] = 9
    elif fault == "origin":
        value["map"]["origin"] = [0, 0]
    elif fault == "yaw":
        value["frame"]["yaw_deg"] = float("nan")
    elif fault == "height":
        value["defaults"]["wall_height"] = -1
    elif fault == "duplicate":
        nodes.append(copy.deepcopy(nodes[0]))
    elif fault == "unknown_link":
        nodes[0]["links"] = ["missing"]
    elif fault == "asymmetric":
        nodes[1]["links"] = []
    elif fault == "outside":
        nodes[0]["at"] = [9999, 9999]
    layout_path.write_text(yaml.safe_dump(value))
    with pytest.raises(ValueError):
        load_map(path, layout_path)


def test_api_uses_operator_map_and_layout_without_db_or_path_disclosure(measured_map):
    path, layout_path, _ = measured_map
    app = create_app_from_env(
        {"ROPORT_MAP": str(path), "ROPORT_MAP_LAYOUT": str(layout_path)}
    )
    with TestClient(app) as client:
        response = client.get("/api/bootstrap")
        assert str(path.parent) not in response.text
        value = response.json()
        assert not value["example_map"]
        assert value["map"]["layout"]["graph_version"] == "fixture-lines-v1"
        assert client.get("/api/robots").status_code == 404
        assert client.get("/api/snapshot").json()["source"] == "OFFLINE · 실행부 미연결"


def test_demo_coordinates_follow_selected_map_without_becoming_real_state(measured_map):
    path, _, _ = measured_map
    grid = load_map(path)
    original = Display(demo=True)
    mapped = Display(demo=True, map_geometry=grid)
    original.playing = mapped.playing = False
    for a, b in zip(
        original.snapshot()["robots"], mapped.snapshot()["robots"], strict=True
    ):
        assert b["pose"][:2] == pytest.approx(
            [
                -2 + a["pose"][0] / 4.6 * 8,
                -1 + a["pose"][1] / 2.65 * 4,
            ]
        )
        assert -math.pi <= b["pose"][3] <= math.pi
    assert all(
        r["pose"] is None for r in Display(map_geometry=grid).snapshot()["robots"]
    )


def test_explicit_mount_height_does_not_require_default(measured_map):
    path, layout_path, layout = measured_map
    layout["objects"] = [
        {"name": "omx12_left", "min": [100, -200], "max": [120, -170], "height": 30},
    ]
    layout_path.write_text(yaml.safe_dump(layout))
    assert load_map(path, layout_path)["layout"]["mounts"][0]["edge_center"][2] == 0.3
