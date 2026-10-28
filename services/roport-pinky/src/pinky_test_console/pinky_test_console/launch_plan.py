"""console.launch.py 인자 검사 (ROS 의존성 없음, 단위 테스트 대상)."""

import os
from dataclasses import dataclass

VALID_MODES = ('slam', 'nav', 'ir', 'marker')
NO_INCLUDE_MODES = ('ir', 'marker')   # pinky_navigation 을 include 하지 않는 모드


class LaunchArgError(ValueError):
    pass


@dataclass(frozen=True)
class LaunchPlan:
    mode: str
    map_yaml: str  # nav 모드에서만 사용 (절대경로), slam 모드는 ''
    port: int
    use_sim_time: bool
    ir_node: bool = False  # pinky_sensor_adc main_node 를 함께 실행


def parse_bool(text, name='use_sim_time'):
    t = str(text).strip().lower()
    if t in ('true', '1', 'yes', 'on'):
        return True
    if t in ('false', '0', 'no', 'off', ''):
        return False
    raise LaunchArgError(f"{name} must be true/false, got '{text}'")


def parse_port(text):
    try:
        port = int(str(text).strip())
    except ValueError:
        raise LaunchArgError(f"port must be an integer, got '{text}'") from None
    if not 1 <= port <= 65535:
        raise LaunchArgError(f'port out of range (1-65535): {port}')
    return port


def resolve_plan(mode, map_path='', port='8080', use_sim_time='false', ir_node='auto'):
    """launch 인자를 검사해 LaunchPlan을 만든다. 잘못되면 LaunchArgError.

    ir_node: auto 이면 mode:=ir 일 때만 IR 센서 노드를 함께 띄운다.
    """
    m = str(mode).strip().lower()
    if m not in VALID_MODES:
        raise LaunchArgError(
            f"mode must be one of {'|'.join(VALID_MODES)}, got '{mode}'")

    map_yaml = ''
    if m == 'nav':
        raw = str(map_path).strip()
        if not raw:
            raise LaunchArgError('mode:=nav requires map:=<path to map .yaml>')
        map_yaml = os.path.abspath(os.path.expanduser(raw))
        if not map_yaml.endswith('.yaml'):
            raise LaunchArgError(f'map must be a .yaml file: {map_yaml}')
        if not os.path.isfile(map_yaml):
            raise LaunchArgError(f'map file not found: {map_yaml}')

    ir = str(ir_node).strip().lower()
    return LaunchPlan(
        mode=m,
        map_yaml=map_yaml,
        port=parse_port(port),
        use_sim_time=parse_bool(use_sim_time),
        ir_node=(m == 'ir') if ir == 'auto' else parse_bool(ir, 'ir_node'),
    )


def included_launch(plan):
    """pinky_navigation 에서 include 할 launch 파일 이름과 인자. mode:=ir|marker 는 (None, {}).

    bringup_robot.launch.xml(모터·라이다)은 포함하지 않는다: 사용자가 먼저 따로 실행.
    """
    if plan.mode in NO_INCLUDE_MODES:
        return None, {}
    sim_time = 'True' if plan.use_sim_time else 'False'
    if plan.mode == 'slam':
        # 원본 map_building.launch.xml = slam_toolbox online_sync + mapper_params.yaml
        return 'map_building.launch.xml', {'use_sim_time': sim_time}
    # 원본 bringup_launch.xml = map_server + amcl + Nav2 (지도 지정)
    return 'bringup_launch.xml', {'map': plan.map_yaml, 'use_sim_time': sim_time}


def resolve_rotate_180(camera, text='auto'):
    """마커 확인 화면의 180° 회전. auto 이면 로봇 카메라(picamera2)일 때만 회전.

    이전 Pinky 주행 코드(Picamera2)가 카메라 영상을 180° 돌려 썼다.
    """
    t = str(text).strip().lower()
    if t == 'auto':
        return str(camera).strip() == 'picamera2'
    return parse_bool(t, 'rotate_180')
