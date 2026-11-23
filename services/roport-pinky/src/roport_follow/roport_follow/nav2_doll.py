"""roport_smartcart navigation.launch 에 넘길 Nav2 덮어쓰기 파일 만들기(인형 비용 지도 층 추가).

roport_smartcart 패키지의 파일은 고치지 않는다. 그쪽 nav2_overrides.yaml 을 읽어 그대로 두고
지역·전역 비용 지도에 doll_layer(ObstacleLayer, /doll_obstacles PointCloud2)만 더한 새 파일을 쓴다.

  로봇에서(ROS 환경, 패키지 설치 뒤):
  python3 -m roport_follow.nav2_doll --out ~/roport_follow/nav2_overrides_doll.yaml
  ros2 launch roport_smartcart navigation.launch.py mode:=real ... overrides_file:=$HOME/roport_follow/nav2_overrides_doll.yaml

plugins 목록은 기본 nav2_params(pinky_navigation) + 덮어쓰기를 합친 실제 목록에서 inflation_layer 앞에 넣는다.
지역 비용 지도의 expected_update_rate: 노트북이 그 시간 넘게 점을 안 보내면 비용 지도가 「최신 아님」이 되어
controller 가 멈추게 하려는 값이다(Nav2 동작은 시뮬·실로봇에서 확인 필요).
"""
from __future__ import annotations

import argparse
import copy
from pathlib import Path

import yaml

LAYER = 'doll_layer'


def _merge(base, changes):
    for key, value in changes.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value


def layer_params(topic='/doll_obstacles', max_range_m=1.2, clear_range_m=1.25, keep_near_m=0.0,
                 persistence_s=0.0, expected_s=0.0):
    """기본값 = 인형 기억 모드(11-21): 로봇 인형 층은 노트북이 이번에 보낸 것만 비춘다.
    keep_near_m 0(로봇 바로 앞도 찍고 지움) · persistence_s 0(가장 최근 묶음 하나만). 노트북 도구가 --doll-memory
    (기본 켬)로 본 인형을 기억해 다시 보내고 둘레를 지우는 점(clear_ring)을 함께 보내야 한다.
    예전 방식(기억 없음)은 keep_near_m 0.15 · persistence_s 1.0 + 노트북 --no-doll-memory."""
    if not (0 <= keep_near_m < max_range_m < clear_range_m):
        raise ValueError('need 0 <= keep_near_m < max_range_m < clear_range_m')
    return {
        'plugin': 'nav2_costmap_2d::ObstacleLayer',
        'enabled': True,
        'footprint_clearing_enabled': True,
        'max_obstacle_height': 0.5,
        'combination_method': 1,
        'observation_sources': 'doll',
        'doll': {
            'topic': topic, 'data_type': 'PointCloud2',
            'marking': True, 'clearing': True,
            'min_obstacle_height': 0.0, 'max_obstacle_height': 0.5,
            # 표시·지움 모두 keep_near 밖에서만. 예전 방식(0.15): 15 cm 안 표시는 지움 광선이 닿지 않아 영영 남아서
            # (11-20) 표시도 막았더니, 지도를 비우면 15 cm 안 인형이 다시 안 찍혀 로봇이 침(11-21) → 기억 모드는 0
            'obstacle_min_range': float(keep_near_m), 'obstacle_max_range': float(max_range_m),
            'raytrace_min_range': float(keep_near_m), 'raytrace_max_range': float(clear_range_m) + 0.05,
            'observation_persistence': float(persistence_s),
            'expected_update_rate': float(expected_s),
            'inf_is_valid': False,
        },
    }


def _with_layer(plugins):
    plugins = [p for p in plugins if p != LAYER]
    i = plugins.index('inflation_layer') if 'inflation_layer' in plugins else len(plugins)
    return plugins[:i] + [LAYER] + plugins[i:]


def build_overrides(base_params, overrides, **kw):
    """기본 nav2_params·smartcart 덮어쓰기(dict) → doll_layer 를 더한 새 덮어쓰기(dict)."""
    expected_s = float(kw.pop('expected_s', 1.0))
    effective = copy.deepcopy(base_params)
    _merge(effective, copy.deepcopy(overrides))
    out = copy.deepcopy(overrides)
    for name, expect in (('local_costmap', expected_s), ('global_costmap', 0.0)):
        cur = effective[name][name]['ros__parameters']
        dst = out.setdefault(name, {}).setdefault(name, {}).setdefault('ros__parameters', {})
        dst['plugins'] = _with_layer(list(cur['plugins']))
        dst[LAYER] = layer_params(expected_s=expect, **kw)
    return out


def check_overrides(path, topic, max_range_m):
    """로봇에 넘길 덮어쓰기 파일이 웹 도구 설정과 맞는지(시험 화면 표시). → {'status', 'detail'}"""
    p = Path(path).expanduser()
    if not p.is_file():
        return {'status': 'missing', 'detail': f'{p} 없음'}
    try:
        data = yaml.safe_load(p.read_text()) or {}
        src = data['local_costmap']['local_costmap']['ros__parameters'][LAYER]['doll']
    except (yaml.YAMLError, KeyError, TypeError) as e:
        return {'status': 'different', 'detail': f'doll_layer 없음 또는 형식 오류({type(e).__name__})'}
    diff = []
    if src.get('topic') != topic:
        diff.append(f"topic {src.get('topic')} ≠ {topic}")
    if abs(float(src.get('obstacle_max_range', -1)) - max_range_m) > 1e-6:
        diff.append(f"거리 {float(src.get('obstacle_max_range', -1)) * 100:g} cm ≠ {max_range_m * 100:g} cm")
    return {'status': 'different' if diff else 'same', 'detail': ', '.join(diff) or p.name}


def _share(pkg, *parts):
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory(pkg), *parts))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--base-params', help='기본: pinky_navigation/params/nav2_params.yaml')
    ap.add_argument('--overrides', help='기본: roport_smartcart/config/nav2_overrides.yaml')
    ap.add_argument('--out', default='~/roport_follow/nav2_overrides_doll.yaml')
    ap.add_argument('--topic', default='/doll_obstacles')
    ap.add_argument('--max-range-cm', type=float, default=120.0)
    ap.add_argument('--clear-range-cm', type=float, default=125.0)
    ap.add_argument('--keep-near-cm', type=float, default=0.0, help='로봇 축에서 이 거리 안은 찍지도 지우지도 않음(기억 모드 0)')
    ap.add_argument('--persistence-s', type=float, default=0.0, help='최근 이 시간의 점을 함께 씀(기억 모드 0 = 최근 묶음 하나)')
    ap.add_argument('--expected-s', type=float, default=1.0, help='지역 비용 지도: 점이 이 시간 넘게 안 오면 최신 아님(0 = 안 봄)')
    a = ap.parse_args(argv)
    base = a.base_params or _share('pinky_navigation', 'params', 'nav2_params.yaml')
    over = a.overrides or _share('roport_smartcart', 'config', 'nav2_overrides.yaml')
    data = build_overrides(yaml.safe_load(Path(base).expanduser().read_text()),
                           yaml.safe_load(Path(over).expanduser().read_text()) or {},
                           topic=a.topic, max_range_m=a.max_range_cm / 100,
                           clear_range_m=a.clear_range_cm / 100, keep_near_m=a.keep_near_cm / 100,
                           persistence_s=a.persistence_s, expected_s=a.expected_s)
    out = Path(a.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(f'# 생성: python3 -m roport_follow.nav2_doll (원본 {over})\n'
                   + yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    print(f'저장: {out}\n  지역 plugins: {data["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]}'
          f'\n  전역 plugins: {data["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]}')


if __name__ == '__main__':
    main()
