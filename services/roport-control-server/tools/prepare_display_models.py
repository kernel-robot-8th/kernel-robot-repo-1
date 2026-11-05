"""Build display GLBs from pinned URDF sources; never imports robot drivers.

Run with: env -u PYTHONPATH uv run --isolated --no-project --with trimesh==5.2.0 --with pycollada==0.9.3
  --with fast-simplification==0.2.0 --with xacro==2.1.1 --with scipy==1.18.1 python -I tools/prepare_display_models.py
"""

import json
import re
import shutil
import tempfile
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
import xacro

ROOT = Path(__file__).resolve().parents[3]
OUT = (
    Path(__file__).resolve().parents[1]
    / "src/roport_control_server/spatial_assets/models"
)
OMX_SHA = "b83daec7b98c08d00adb856d7e31f2b0065d3be9"
(PINKY_SHA,) = re.findall(
    r"^- Commit: `([0-9a-f]{40})`$",
    (ROOT / "services/roport-pinky/src/UPSTREAM.md").read_text(),
    re.MULTILINE,
)


def origin(element):
    node = element.find("origin")
    xyz = (
        [float(v) for v in node.get("xyz", "0 0 0").split()]
        if node is not None
        else [0, 0, 0]
    )
    rpy = (
        [float(v) for v in node.get("rpy", "0 0 0").split()]
        if node is not None
        else [0, 0, 0]
    )
    matrix = trimesh.transformations.euler_matrix(*rpy)
    matrix[:3, 3] = xyz
    return matrix


def build(name, urdf, package_dir):
    tree = ET.fromstring(urdf)
    if name == "pinky":
        ET.indent(tree)
        urdf = ET.tostring(tree, encoding="unicode")
    source = (
        f"https://github.com/pinklab-art/pinky_pro/tree/{PINKY_SHA}"
        if name == "pinky"
        else f"https://github.com/ROBOTIS-GIT/open_manipulator/tree/{OMX_SHA}"
    )
    notice = (
        "<!-- RO:PORT generated display derivative; xacro expansion / XML formatting.\n"
        f"     Source: {source}\n"
        "     Apache-2.0; see the accompanying LICENSE. Not device calibration. -->\n"
    )
    # XML declarations must remain the first bytes of the document.
    if urdf.startswith("<?xml"):
        declaration, body = urdf.split("?>", 1)
        urdf = declaration + "?>\n" + notice + body.lstrip("\n")
    else:
        urdf = notice + urdf
    target = OUT / name
    target.mkdir(parents=True, exist_ok=True)
    (target / "robot.urdf").write_text(urdf)
    scene = trimesh.Scene(base_frame="world")
    joints = []
    for j in tree.findall("joint"):
        parent, child = j.find("parent").get("link"), j.find("child").get("link")
        scene.graph.update(frame_to=child, frame_from=parent, matrix=origin(j))
        if j.get("type") != "fixed":
            axis = j.find("axis")
            row = {
                "name": j.get("name"),
                "node": child,
                "type": j.get("type"),
                "axis": [float(v) for v in axis.get("xyz").split()]
                if axis is not None
                else [1, 0, 0],
            }
            mimic = j.find("mimic")
            if mimic is not None:
                row["mimic"] = {
                    "joint": mimic.get("joint"),
                    "multiplier": float(mimic.get("multiplier", 1)),
                    "offset": float(mimic.get("offset", 0)),
                }
            joints.append(row)
    child_links = {j.find("child").get("link") for j in tree.findall("joint")}
    for link in tree.findall("link"):
        label = link.get("name")
        if label not in child_links and label != "world":
            scene.graph.update(frame_to=label, frame_from="world", matrix=np.eye(4))
        for i, visual in enumerate(link.findall("visual")):
            mesh_node = visual.find("geometry/mesh")
            if mesh_node is None:
                continue
            rel = mesh_node.get("filename").split("/", 3)[3]
            source = package_dir / rel
            loaded = trimesh.load_scene(source).to_mesh()
            loaded.apply_scale(
                [float(v) for v in mesh_node.get("scale", "1 1 1").split()]
            )
            # Pinky DAE surfaces tear under quadric reduction; retain their source faces.
            if name != "pinky" and len(loaded.faces) > 8000:
                loaded = loaded.simplify_quadric_decimation(face_count=8000)
            color = [66, 75, 81, 255]
            if name == "pinky" and "wheel" in label:
                color = [75, 139, 149, 255]
            elif "screen" in label or "camera" in label:
                color = [32, 41, 46, 255]
            loaded.visual = trimesh.visual.ColorVisuals(loaded, face_colors=color)
            scene.add_geometry(
                loaded,
                node_name=f"{label}_visual_{i}",
                parent_node_name=label,
                transform=origin(visual),
            )
    assert len(scene.geometry) > 5, name
    assert np.isfinite(scene.bounds).all(), name
    scene.export(target / "robot.glb")
    (target / "joints.json").write_text(json.dumps(joints, indent=2) + "\n")
    print(
        name,
        "bounds_m=",
        scene.bounds.tolist(),
        "faces=",
        sum(len(m.faces) for m in scene.geometry.values()),
        "bytes=",
        (target / "robot.glb").stat().st_size,
    )


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    pinky = ROOT / "services/roport-pinky/src/pinky_pro/pinky_description"
    with tempfile.TemporaryDirectory() as tmp:
        temp = Path(tmp)
        for filename in [
            "robot.urdf.xacro",
            "pinky.urdf.xacro",
            "common/insert_inertia.urdf.xacro",
        ]:
            p = temp / filename
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(
                (pinky / "urdf" / filename)
                .read_text()
                .replace("$(find pinky_description)/urdf/", str(temp) + "/")
            )
        urdf = xacro.process_file(str(temp / "robot.urdf.xacro")).toxml()
        build("pinky", urdf, pinky)
        shutil.copyfile(pinky.parent / "LICENSE", OUT / "pinky/LICENSE")
        remote = (
            f"https://raw.githubusercontent.com/ROBOTIS-GIT/open_manipulator/{OMX_SHA}/"
        )
        data = (
            urllib.request.urlopen(
                remote + "open_manipulator_description/urdf/omx_f/omx_f.urdf"
            )
            .read()
            .decode()
        )
        package = temp / "omx"
        for node in ET.fromstring(data).findall(".//visual/geometry/mesh"):
            rel = node.get("filename").split("/", 3)[3]
            dest = package / rel
            if not dest.exists():
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(
                    urllib.request.urlopen(
                        remote + "open_manipulator_description/" + rel
                    ).read()
                )
        build("omx", data, package)
        (OUT / "omx/LICENSE").write_bytes(
            urllib.request.urlopen(remote + "LICENSE").read()
        )
    (OUT / "sources.json").write_text(
        json.dumps(
            {
                "pinky": {
                    "repository": "https://github.com/pinklab-art/pinky_pro",
                    "commit": PINKY_SHA,
                },
                "omx": {
                    "repository": "https://github.com/ROBOTIS-GIT/open_manipulator",
                    "commit": OMX_SHA,
                },
                "calibration": "URDF defaults; installed device calibration not verified",
                "mesh_processing": "Pinky: original visual faces preserved; OMX: target 8000 faces per visual; neutral display colors, metres",
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
