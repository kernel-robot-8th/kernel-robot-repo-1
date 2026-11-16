"""상차 노드 정밀 도킹 값(FFT1-112). 순수 Python, ROS 없음.

상차(load) 노드마다 세 값: 가로선에서 멈춘 뒤
  advance_cm  ① 접근 방향으로 더 감(음수 = 뒤로)
  rotate_deg  ② 반시계 회전(기본 90 = 지금 동작)
  move_cm     ③ 회전 뒤 새 방향으로 직진(음수 = 뒤로)
허용 범위(2026-11-09 사용자: 실로봇 시험하며 조정): |advance_cm| <= 6, 60 <= rotate_deg <= 120, |move_cm| <= 6.
파일에 없는 노드 = {0, 90, 0}(지금 동작과 같음). 값은 실측 전이라 기본값을 따로 두지 않는다.

파일 형식(dock_file, 기본 ~/roport_dock.yaml):
  version: 1
  nodes:
    N11: {advance_cm: 0.0, rotate_deg: 90.0, move_cm: 0.0}
"""

import math
import os
import tempfile
from dataclasses import dataclass

import yaml

from roport_line_nav.graph import LOAD

ADVANCE_MAX_CM = 6.0
ROTATE_MIN_DEG = 60.0
ROTATE_MAX_DEG = 120.0
MOVE_MAX_CM = 6.0
FILE_VERSION = 1
MAX_FILE_BYTES = 64 * 1024
FIELDS = ('advance_cm', 'rotate_deg', 'move_cm')


def _number(name, v):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError(f'{name} must be a finite number')
    return float(v)


@dataclass(frozen=True)
class DockValue:
    advance_cm: float = 0.0
    rotate_deg: float = 90.0
    move_cm: float = 0.0

    def __post_init__(self):
        a = _number('advance_cm', self.advance_cm)
        r = _number('rotate_deg', self.rotate_deg)
        b = _number('move_cm', self.move_cm)
        if not abs(a) <= ADVANCE_MAX_CM:
            raise ValueError(f'advance_cm must be in -{ADVANCE_MAX_CM}..{ADVANCE_MAX_CM}: {a}')
        if not ROTATE_MIN_DEG <= r <= ROTATE_MAX_DEG:
            raise ValueError(f'rotate_deg must be in {ROTATE_MIN_DEG}..{ROTATE_MAX_DEG}: {r}')
        if not abs(b) <= MOVE_MAX_CM:
            raise ValueError(f'move_cm must be in -{MOVE_MAX_CM}..{MOVE_MAX_CM}: {b}')
        object.__setattr__(self, 'advance_cm', a)
        object.__setattr__(self, 'rotate_deg', r)
        object.__setattr__(self, 'move_cm', b)

    @property
    def is_default(self):
        return self == DEFAULT

    def as_dict(self):
        return {k: getattr(self, k) for k in FIELDS}


DEFAULT = DockValue()


def load_nodes(graph):
    """도킹 값을 둘 수 있는 노드 = 도착 동작이 load 인 가로선 노드(N01·N11·N21·N22)."""
    return frozenset(nid for nid, n in graph.nodes.items()
                     if n.bar and graph.arrival_action(nid) == LOAD)


class DockTable:
    def __init__(self, graph, values=None):
        self.graph = graph
        self.allowed = load_nodes(graph)
        self._values = {}
        for nid, v in (values or {}).items():
            self.set(nid, v.advance_cm, v.rotate_deg, v.move_cm)

    def _check_node(self, nid):
        if nid not in self.allowed:
            raise ValueError(f'{nid} is not a load node (allowed: {", ".join(sorted(self.allowed))})')

    def get(self, nid):
        return self._values.get(nid, DEFAULT)

    def set(self, nid, advance_cm, rotate_deg, move_cm):
        """검사 후 적용. 범위 밖·상차 노드 아님이면 ValueError(그대로 유지)."""
        nid = str(nid)
        self._check_node(nid)
        v = DockValue(advance_cm, rotate_deg, move_cm)
        if v.is_default:
            self._values.pop(nid, None)
        else:
            self._values[nid] = v
        return v

    def reset(self, nid):
        nid = str(nid)
        self._check_node(nid)
        self._values.pop(nid, None)

    def copy(self):
        return DockTable(self.graph, dict(self._values))

    def as_dict(self):
        """기본값이 아닌 노드만."""
        return {nid: self._values[nid].as_dict() for nid in sorted(self._values)}

    def to_data(self):
        return {'version': FILE_VERSION, 'nodes': self.as_dict()}

    @classmethod
    def from_data(cls, graph, data):
        """파일 내용 → DockTable. 형식·범위·노드가 하나라도 틀리면 ValueError(일부만 쓰지 않음)."""
        if not isinstance(data, dict) or data.get('version') != FILE_VERSION:
            raise ValueError(f'dock file needs version: {FILE_VERSION}')
        nodes = data.get('nodes') or {}
        if not isinstance(nodes, dict):
            raise ValueError('dock file "nodes" must be a mapping')
        table = cls(graph)
        for nid, v in nodes.items():
            if not isinstance(v, dict) or set(v) - set(FIELDS):
                raise ValueError(f'{nid}: needs only {", ".join(FIELDS)}')
            d = DEFAULT.as_dict()
            d.update(v)
            table.set(str(nid), d['advance_cm'], d['rotate_deg'], d['move_cm'])
        return table


def load_file(path, graph):
    """→ (DockTable, 경고 문자열 또는 None). 없거나 깨지면 기본값(빈 표) + 경고."""
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return DockTable(graph), f'dock file {path} not found; using defaults (0, 90, 0)'
    try:
        if os.path.getsize(path) > MAX_FILE_BYTES:
            raise ValueError('file too large')
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return DockTable.from_data(graph, data), None
    except (OSError, ValueError, yaml.YAMLError) as e:
        return DockTable(graph), f'dock file {path} unreadable ({e}); using defaults (0, 90, 0)'


def save_file(path, table):
    """원자적 저장(같은 디렉터리 임시 파일 → fsync → rename). 실패하면 OSError."""
    path = os.path.expanduser(path)
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.roport_dock.', suffix='.tmp', dir=d)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            yaml.safe_dump(table.to_data(), f, sort_keys=True, allow_unicode=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


__all__ = ['DockValue', 'DockTable', 'DEFAULT', 'load_nodes', 'load_file', 'save_file',
           'ADVANCE_MAX_CM', 'ROTATE_MIN_DEG', 'ROTATE_MAX_DEG', 'MOVE_MAX_CM']
