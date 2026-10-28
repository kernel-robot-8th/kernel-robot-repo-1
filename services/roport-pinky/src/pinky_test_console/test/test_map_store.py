import os

import pytest

from pinky_test_console.map_store import (
    MapNameError,
    list_maps,
    map_base_path,
    map_exists,
    resolve_map_dir,
    validate_name,
)


@pytest.mark.parametrize('name', ['classroom', 'room_3F-01', 'A', 'x' * 64])
def test_valid_names(name):
    assert validate_name(name) == name


@pytest.mark.parametrize('name', [
    '', ' ', 'x' * 65, 'a b', 'a.b', '../etc', 'a/b', '~/x', '강의실',
    # slam_toolbox 가 system() 으로 넘기므로 쉘 메타문자는 모두 거부
    'a;rm -rf ~', 'a&&b', 'a|b', '$(id)', '`id`', "a'b", 'a"b', 'a\nb', 'a>b',
    'room\n', 'room\r', '\troom',
    None, 123,
])
def test_invalid_names(name):
    with pytest.raises(MapNameError):
        validate_name(name)


def test_map_dir_tilde(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    assert resolve_map_dir('~/roport_maps') == str(tmp_path / 'roport_maps')


@pytest.mark.parametrize('path', ['/tmp/my maps', '/tmp/a;b', '/tmp/$(x)', '/tmp/강의', '/tmp/x\n'])
def test_map_dir_unsafe(path):
    with pytest.raises(MapNameError):
        resolve_map_dir(path)


def test_map_base_path_is_absolute(tmp_path):
    d = resolve_map_dir(str(tmp_path))
    assert map_base_path(d, 'room') == os.path.join(d, 'room')
    assert os.path.isabs(map_base_path(d, 'room'))


def test_list_and_exists(tmp_path):
    d = str(tmp_path)
    assert list_maps(str(tmp_path / 'missing')) == []
    (tmp_path / 'old.yaml').write_text('image: old.pgm\n')
    (tmp_path / 'old.pgm').write_bytes(b'P5' + b'\0' * 100)
    os.utime(tmp_path / 'old.yaml', (1000, 1000))
    (tmp_path / 'new.yaml').write_text('image: new.pgm\n')  # pgm 없음
    (tmp_path / 'bad name.yaml').write_text('x')             # 허용 안 되는 이름은 숨김
    (tmp_path / 'notes.txt').write_text('x')
    maps = list_maps(d)
    assert [m['name'] for m in maps] == ['new', 'old']
    assert maps[0]['has_pgm'] is False
    assert maps[1]['has_pgm'] is True and maps[1]['pgm_bytes'] == 102
    assert maps[1]['yaml'] == os.path.join(d, 'old.yaml')
    assert map_exists(d, 'old') and map_exists(d, 'new')
    assert not map_exists(d, 'other')
