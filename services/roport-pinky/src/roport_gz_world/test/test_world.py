import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

from roport_gz_world.generate_world import build_sdf, main
from roport_gz_world.layout import Layout

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAYOUT = os.path.join(PKG, 'layout', 'roport_airport.yaml')
WORLD = os.path.join(PKG, 'worlds', 'roport_airport.sdf')


@pytest.fixture(scope='module')
def root():
    return ET.parse(WORLD).getroot()


def _model(root, name):
    return root.find(f".//model[@name='{name}']")


def test_committed_world_matches_layout():
    """배치 데이터를 고친 뒤 world 를 다시 생성하지 않으면 실패한다."""
    assert open(WORLD, encoding='utf-8').read() == build_sdf(Layout.load(LAYOUT))
    assert main(['--layout', LAYOUT, '--output', WORLD, '--check']) == 0


def test_check_detects_stale_world(tmp_path):
    stale = tmp_path / 'stale.sdf'
    stale.write_text('<sdf/>')
    assert main(['--layout', LAYOUT, '--output', str(stale), '--check']) == 1


def test_sensors_plugin_for_lidar_and_camera(root):
    sensors = root.find(".//plugin[@name='gz::sim::systems::Sensors']")
    assert sensors is not None and sensors.findtext('render_engine') == 'ogre2'
    assert root.find(".//plugin[@name='gz::sim::systems::Physics']") is not None


def test_collision_only_for_walls_objects_ground(root):
    for name in ('walls', 'objects'):
        m = _model(root, name)
        assert len(m.findall('.//collision')) == len(m.findall('.//visual')) > 0
    for name in ('floor_lines', 'markers'):
        m = _model(root, name)
        assert m.findall('.//collision') == []
        assert len(m.findall('.//visual')) > 0
    assert _model(root, 'ground').find('.//collision/geometry/plane') is not None


def test_all_models_static(root):
    for m in root.findall('.//model'):
        assert m.findtext('static') == 'true', m.get('name')


@pytest.mark.skipif(shutil.which('gz') is None, reason='gz (Gazebo) not installed')
def test_gz_sdf_validates():
    r = subprocess.run(['gz', 'sdf', '-k', WORLD], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0 and 'Valid' in r.stdout, r.stdout + r.stderr
