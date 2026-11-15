"""config/robots.yaml에서 기체별 IR 기준값을 읽는다."""

import yaml

from roport_pinky_line.line_sensor import Calibration


def load_calibration(path, robot):
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    robots = data.get('robots') or {}
    if robot not in robots:
        names = ', '.join(sorted(robots)) or '(none)'
        raise KeyError(f'robot "{robot}" not in {path}; choose one of: {names}')
    entry = robots[robot]
    return Calibration(
        carpet=tuple(int(v) for v in entry['carpet']),
        tape=tuple(int(v) for v in entry['tape']),
        threshold=tuple(int(v) for v in entry['threshold']),
        hysteresis=int(entry.get('hysteresis', data.get('hysteresis', 100))),
    )
