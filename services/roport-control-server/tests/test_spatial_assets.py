import hashlib
import json
import runpy
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[1]
ASSETS = ROOT / "src/roport_control_server/spatial_assets"
TOOLS = runpy.run_path(str(ROOT / "tools/prepare_web_assets.py"))


def test_prepared_assets_and_licenses_match_pinned_hashes():
    TOOLS["verify"](ROOT)
    for kind in ("pinky", "omx"):
        assert (ASSETS / "models" / kind / "robot.glb").read_bytes()[:4] == b"glTF"
        assert (
            len(json.loads((ASSETS / "models" / kind / "joints.json").read_text())) >= 2
        )
        assert (ASSETS / "models" / kind / "LICENSE").is_file()
    assert (ASSETS / "vendor/three/LICENSE").is_file()


@pytest.fixture
def source_bundle(tmp_path):
    assets = tmp_path / "src/roport_control_server/spatial_assets"
    npm = tmp_path / "node_modules/three"
    assets.mkdir(parents=True)
    (npm / "build").mkdir(parents=True)
    (tmp_path / "package.json").write_text(
        json.dumps({"dependencies": {"three": "0.187.0"}})
    )
    (npm / "package.json").write_text(json.dumps({"version": "0.187.0"}))
    (npm / "LICENSE").write_bytes(b"license")
    (npm / "build/three.core.js").write_bytes(b"expected")
    manifest = {
        "vendor/three/LICENSE": hashlib.sha256(b"license").hexdigest(),
        "vendor/three/esm/three.core.js": hashlib.sha256(b"expected").hexdigest(),
    }
    (assets / "checksums.json").write_text(json.dumps(manifest))
    return tmp_path, npm, assets


def test_prepare_copies_build_to_esm_and_checks_final_bundle(source_bundle):
    root, _, assets = source_bundle
    TOOLS["prepare"](root)
    target = assets / "vendor/three/esm/three.core.js"
    assert target.read_bytes() == b"expected"
    target.write_bytes(b"corrupt")
    with pytest.raises(ValueError, match="Missing or changed asset"):
        TOOLS["verify"](root)
    target.unlink()
    with pytest.raises(ValueError, match="Missing or changed asset"):
        TOOLS["verify"](root)


@pytest.mark.parametrize("failure", ["version", "checksum", "missing"])
def test_bad_npm_input_does_not_partially_write_bundle(source_bundle, failure):
    root, npm, assets = source_bundle
    if failure == "version":
        (npm / "package.json").write_text('{"version":"0.186.0"}')
    elif failure == "checksum":
        (npm / "build/three.core.js").write_bytes(b"changed upstream")
    else:
        (npm / "build/three.core.js").unlink()
    with pytest.raises((ValueError, FileNotFoundError)):
        TOOLS["prepare"](root)
    assert not (assets / "vendor").exists()


def test_pinky_visuals_preserve_source_triangles():
    import json
    import struct
    import xml.etree.ElementTree as ET

    model = ASSETS / "models/pinky"
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
