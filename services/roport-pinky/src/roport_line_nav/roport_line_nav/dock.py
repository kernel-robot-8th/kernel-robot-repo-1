"""상차 노드 정밀 도킹 값(FFT1-112). 순수 Python, ROS 없음.

상차(load) 노드마다 세 값: 가로선에서 멈춘 뒤
  advance_cm  ① 접근 방향으로 더 감(음수 = 뒤로)
  rotate_deg  ② 반시계 회전(기본 90 = 지금 동작)
  move_cm     ③ 회전 뒤 새 방향으로 직진(음수 = 뒤로)
허용 범위(2026-11-09 사용자: 실로봇 시험하며 조정): |advance_cm| <= 6, |move_cm| <= 6,
rotate_deg 는 노드별(양 끝 포함):
  N01·N11      60..120
  N21·N22      -10..10 · 60..120 (2026-11-16 OMX 팀 요청·사용자 결정: 정면 0° 추가, 180° 는 안 씀)
파일에 없는 노드 = {0, 90, 0}. N21·N22 만 처음 값이 {0, 0, -3}(NODE_INITIAL, 2026-11-25 사용자 결정, 실로봇 미확인).

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
ROTATE_MIN_DEG = 60.0          # N01·N11(기본) 범위
ROTATE_MAX_DEG = 120.0
ROTATE_BANDS_DEFAULT = ((ROTATE_MIN_DEG, ROTATE_MAX_DEG),)
# 노드별 ② 범위(양 끝 포함). 없는 노드 = ROTATE_BANDS_DEFAULT
ROTATE_BANDS = {
    'N21': ((-10.0, 10.0), (ROTATE_MIN_DEG, ROTATE_MAX_DEG)),
    'N22': ((-10.0, 10.0), (ROTATE_MIN_DEG, ROTATE_MAX_DEG)),
}
MOVE_MAX_CM = 6.0
FILE_VERSION = 1
MAX_FILE_BYTES = 64 * 1024
FIELDS = ('advance_cm', 'rotate_deg', 'move_cm')


def _number(name, v):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError(f'{name} must be a finite number')
    return float(v)


def _rotate(v):
    """② 값 검사·정리: 유한 수, −0 → 0."""
    return _number('rotate_deg', v) + 0.0


def rotate_bands(nid):
    """노드 nid 의 ② 허용 범위 ((최소, 최대), ...)."""
    return ROTATE_BANDS.get(str(nid), ROTATE_BANDS_DEFAULT)


def bands_text(bands):
    return ', '.join(f'{lo:g}..{hi:g}' for lo, hi in bands)


def _in_bands(r, bands):
    return any(lo <= r <= hi for lo, hi in bands)


_ALL_BANDS = tuple(sorted({b for bs in (ROTATE_BANDS_DEFAULT, *ROTATE_BANDS.values()) for b in bs}))


@dataclass(frozen=True)
class DockValue:
    advance_cm: float = 0.0
    rotate_deg: float = 90.0
    move_cm: float = 0.0

    def __post_init__(self):
        a = _number('advance_cm', self.advance_cm)
        r = _rotate(self.rotate_deg)
        b = _number('move_cm', self.move_cm)
        if not abs(a) <= ADVANCE_MAX_CM:
            raise ValueError(f'advance_cm must be in -{ADVANCE_MAX_CM}..{ADVANCE_MAX_CM}: {a}')
        # 여기서는 어느 노드에서든 가능한 범위만 본다. 노드별 범위는 DockTable.set 이 본다.
        if not _in_bands(r, _ALL_BANDS):
            raise ValueError(f'rotate_deg must be in {bands_text(_ALL_BANDS)}: {r}')
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

# 도킹 파일에 없는 노드의 처음 값. 없는 노드 = DEFAULT(0, 90, 0).
# N21·N22(2026-11-25 사용자 결정): 도착 회전 없이(0°) 가로선에서 3 cm 후진해 상차 대기 → 상차 뒤 3 cm 전진 →
# 반시계 약 180° 로 진입선을 찾아 나감. 파일이나 dock 명령으로 노드별로 바꿀 수 있다.
NODE_INITIAL = {'N21': (0.0, 0.0, -3.0), 'N22': (0.0, 0.0, -3.0)}


def initial_for(nid):
    """표에 값이 없는 노드 nid 가 쓰는 값."""
    return DockValue(*NODE_INITIAL[nid]) if nid in NODE_INITIAL else DEFAULT


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
        return self._values.get(nid, initial_for(str(nid)))

    def set(self, nid, advance_cm, rotate_deg, move_cm):
        """검사 후 적용. 범위 밖·상차 노드 아님이면 ValueError(그대로 유지)."""
        nid = str(nid)
        self._check_node(nid)
        r = _rotate(rotate_deg)
        bands = rotate_bands(nid)
        if not _in_bands(r, bands):
            raise ValueError(f'{nid} rotate_deg must be in {bands_text(bands)}: {r:g}')
        v = DockValue(advance_cm, r, move_cm)
        if v == initial_for(nid):           # 그 노드의 처음 값이면 표에 두지 않는다
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
        """처음 값(initial_for)과 다른 노드만."""
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
        return DockTable(graph), f'dock file {path} not found; using defaults (0, 90, 0; N21·N22 0, 0, -3)'
    try:
        if os.path.getsize(path) > MAX_FILE_BYTES:
            raise ValueError('file too large')
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return DockTable.from_data(graph, data), None
    except (OSError, ValueError, yaml.YAMLError) as e:
        return DockTable(graph), f'dock file {path} unreadable ({e}); using defaults (0, 90, 0; N21·N22 0, 0, -3)'


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
           'rotate_bands', 'NODE_INITIAL', 'initial_for', 'ADVANCE_MAX_CM', 'ROTATE_MIN_DEG', 'ROTATE_MAX_DEG', 'ROTATE_BANDS',
           'MOVE_MAX_CM']
