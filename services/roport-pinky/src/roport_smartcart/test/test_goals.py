from copy import deepcopy

import pytest
import yaml

from roport_smartcart.goals import Destination, load_destination, map_hashes


@pytest.fixture
def configured(tmp_path):
    (tmp_path / 'map.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    places = tmp_path / 'places.yaml'
    data = {'map': {'frame': 'map', **map_hashes(map_yaml)}, 'places': {
        'gate1': {'measured': True, 'pose': [1.0, 2.0, 0.5], 'marker_id': 8}}}
    places.write_text(yaml.safe_dump(data))
    return places, map_yaml, data


def test_measured_destination(configured):
    places, map_yaml, _ = configured
    assert load_destination(places, map_yaml, 'gate1') == Destination(1, 2, .5, 8)
    with pytest.raises(ValueError):
        load_destination(places, map_yaml, 'unknown')


@pytest.mark.parametrize('change', ['yaml', 'image'])
def test_map_changes_invalidate_coordinates(configured, change):
    places, map_yaml, _ = configured
    target = map_yaml if change == 'yaml' else map_yaml.parent / 'map.pgm'
    target.write_bytes(target.read_bytes() + b'\n')
    with pytest.raises(ValueError, match='map changed'):
        load_destination(places, map_yaml, 'gate1')


@pytest.mark.parametrize('entry', [
    {'measured': False, 'pose': [1, 2, 0]},
    {'measured': True, 'pose': None},
    {'measured': True, 'pose': [float('nan'), 2, 0]},
    {'measured': True, 'pose': [True, 2, 0]},
    {'measured': True, 'pose': [1, 2, 180]},
    {'measured': True, 'pose': [1, 2, 0], 'marker_id': 50},
])
def test_invalid_or_unmeasured_pose(configured, entry):
    places, map_yaml, original = configured
    data = deepcopy(original)
    data['places']['gate1'] = entry
    places.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_destination(places, map_yaml, 'gate1')


def test_duplicate_place_is_rejected(configured):
    places, map_yaml, _ = configured
    places.write_text(places.read_text() + '\nplaces: {}\n')
    with pytest.raises(ValueError, match='duplicate YAML key'):
        load_destination(places, map_yaml, 'gate1')


@pytest.mark.parametrize('environment', ['sim', 'real'])
def test_same_map_and_pose_cannot_cross_execution_environment(configured, environment):
    places, map_yaml, data = configured
    data['environment'] = environment
    places.write_text(yaml.safe_dump(data))
    assert load_destination(places, map_yaml, 'gate1', environment=environment) == (
        Destination(1, 2, .5, 8))
    other = 'real' if environment == 'sim' else 'sim'
    with pytest.raises(ValueError, match='environment'):
        load_destination(places, map_yaml, 'gate1', environment=other)


@pytest.mark.parametrize('environment', ['sim', 'real'])
@pytest.mark.parametrize('declared', [None, '', 'gazebo', 'REAL'])
def test_unlabelled_or_invalid_places_do_not_authorize_explicit_mode(
        configured, environment, declared):
    places, map_yaml, data = configured
    if declared is not None:
        data['environment'] = declared
    places.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='environment'):
        load_destination(places, map_yaml, 'gate1', environment=environment)


def test_repository_sim_destinations_keep_padded_footprints_in_free_cells():
    import math
    from pathlib import Path
    from roport_smartcart.live_map import reference_from_file

    package = Path(__file__).parents[1]
    cfg, width, height, cells = reference_from_file(package.parents[1] / 'maps/airport_map.yaml')
    places = yaml.safe_load((package / 'config/places_sim.yaml').read_text())['places']
    # Generated sim profile: 8 cm half-width/length plus 1 cm footprint padding.
    # Conservatively check every map cell in the rotated body's bounding box.
    for name, entry in places.items():
        x, y, yaw = entry['pose']
        corners = [(x + dx * math.cos(yaw) - dy * math.sin(yaw),
                    y + dx * math.sin(yaw) + dy * math.cos(yaw))
                   for dx in (-.09, .09) for dy in (-.09, .09)]
        columns = [math.floor((p[0] - cfg['origin'][0]) / cfg['resolution']) for p in corners]
        rows = [math.floor((p[1] - cfg['origin'][1]) / cfg['resolution']) for p in corners]
        assert 0 <= min(columns) <= max(columns) < width, name
        assert 0 <= min(rows) <= max(rows) < height, name
        assert all(cells[row * width + column] == 0
                   for row in range(min(rows), max(rows) + 1)
                   for column in range(min(columns), max(columns) + 1)), name
