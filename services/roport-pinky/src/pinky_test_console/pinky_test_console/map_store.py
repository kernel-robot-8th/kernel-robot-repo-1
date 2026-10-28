"""지도 저장 폴더(~/roport_maps) 관리 (ROS 의존성 없음, 단위 테스트 대상).

보안: slam_toolbox 의 save_map 서비스는 받은 이름을
  system("ros2 run nav2_map_server map_saver_cli -f " + name + " ...")
로 쉘에 그대로 넘긴다(slam_toolbox 2.8.x). 따라서 이름과 폴더 경로에
쉘 메타문자·공백이 들어가지 않도록 허용 문자만 통과시킨다.
"""

import os
import re

DEFAULT_MAP_DIR = '~/roport_maps'
_NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,64}$')
_DIR_RE = re.compile(r'^/[A-Za-z0-9_./-]*$')


class MapNameError(ValueError):
    pass


def validate_name(name):
    if not isinstance(name, str) or not _NAME_RE.fullmatch(name):
        raise MapNameError(
            'map name must be 1-64 chars of A-Z a-z 0-9 _ - (no spaces, dots or slashes)')
    return name


def resolve_map_dir(path=DEFAULT_MAP_DIR):
    """~ 를 펼친 절대경로. 쉘에 안전한 문자만 허용."""
    d = os.path.abspath(os.path.expanduser(str(path)))
    if not _DIR_RE.fullmatch(d) or '/../' in d + '/':
        raise MapNameError(f'map directory has unsafe characters: {d!r}')
    return d


def map_base_path(map_dir, name):
    """SaveMap 에 넘길 확장자 없는 절대경로 (<dir>/<name>)."""
    return os.path.join(map_dir, validate_name(name))


def map_exists(map_dir, name):
    base = map_base_path(map_dir, name)
    return os.path.exists(base + '.yaml') or os.path.exists(base + '.pgm')


def list_maps(map_dir):
    """저장된 지도 목록. 최근 수정 순."""
    if not os.path.isdir(map_dir):
        return []
    out = []
    for fname in os.listdir(map_dir):
        if not fname.endswith('.yaml'):
            continue
        name = fname[:-len('.yaml')]
        if not _NAME_RE.fullmatch(name):
            continue
        yaml_path = os.path.join(map_dir, fname)
        pgm_path = os.path.join(map_dir, name + '.pgm')
        try:
            st = os.stat(yaml_path)
        except OSError:
            continue
        has_pgm = os.path.isfile(pgm_path)
        out.append({
            'name': name,
            'yaml': yaml_path,
            'has_pgm': has_pgm,
            'pgm_bytes': os.path.getsize(pgm_path) if has_pgm else 0,
            'mtime': st.st_mtime,
        })
    out.sort(key=lambda m: m['mtime'], reverse=True)
    return out
