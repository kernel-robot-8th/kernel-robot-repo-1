"""Measured destinations tied to the exact map YAML and image, in metres/radians."""

from dataclasses import dataclass
from hashlib import sha256
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Destination:
    x: float
    y: float
    yaw: float
    marker_id: int | None = None

    def __post_init__(self):
        if not all(type(v) in (int, float) and math.isfinite(v)
                   for v in (self.x, self.y, self.yaw)):
            raise ValueError('pose must contain finite numbers')
        if not -math.pi <= self.yaw <= math.pi:
            raise ValueError('yaw must be in [-pi, pi] radians')
        if self.marker_id is not None and (
                type(self.marker_id) is not int or not 0 <= self.marker_id < 50):
            raise ValueError('marker_id must be a DICT_4X4_50 ID')


class _UniqueLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if key in result:
            raise ValueError(f'duplicate YAML key: {key}')
        result[key] = loader.construct_object(value_node)
    return result


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def _read_yaml(path):
    value = yaml.load(Path(path).read_text(), Loader=_UniqueLoader)
    if not isinstance(value, dict):
        raise ValueError('expected a YAML mapping')
    return value


def map_hashes(map_yaml):
    path = Path(map_yaml)
    data = _read_yaml(path)
    image = data.get('image')
    if not isinstance(image, str) or not image:
        raise ValueError('map YAML must name an image')
    return {
        'yaml_sha256': sha256(path.read_bytes()).hexdigest(),
        'image_sha256': sha256((path.parent / image).read_bytes()).hexdigest(),
    }


def load_destination(places_file, map_yaml, place_id, *, environment=None):
    """Reject unmeasured/stale targets before a Nav2 goal can be submitted.

    This checks configuration provenance, not live localization or clearance.
    """
    data = _read_yaml(places_file)
    if environment is not None:
        if environment not in ('sim', 'real') or data.get('environment') != environment:
            raise ValueError('destination environment must match sim/real execution mode')
    metadata = data.get('map')
    if not isinstance(metadata, dict) or metadata.get('frame') != 'map':
        raise ValueError('destination frame must be map')
    for name, value in map_hashes(map_yaml).items():
        if metadata.get(name) != value:
            raise ValueError('map changed: remeasure and verify destinations')
    places = data.get('places')
    if not isinstance(place_id, str) or not isinstance(places, dict):
        raise ValueError('expected a configured place ID')
    entry = places.get(place_id)
    if not isinstance(entry, dict) or entry.get('measured') is not True:
        raise ValueError(f'unmeasured or unknown destination: {place_id}')
    pose = entry.get('pose')
    if not isinstance(pose, list) or len(pose) != 3:
        raise ValueError('pose must be [x, y, yaw]')
    return Destination(*pose, marker_id=entry.get('marker_id'))
