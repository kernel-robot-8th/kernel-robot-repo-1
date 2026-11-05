"""console.launch.py 인자 검사 (ROS 의존성 없음, 단위 테스트 대상)."""

import ipaddress
import os
import re
from dataclasses import dataclass

# route = 로봇에서 노드 경로 주행(roport_line_nav) 확인, fleet = 노트북에서 Pinky 설정·테스트 관제
# test = 로봇 통합 시험(경로 주행 + IR 측정 화면, 2026-11-04 결정. 마커는 부착·시험 뒤에 추가)
VALID_MODES = ('slam', 'nav', 'ir', 'marker', 'amcl', 'route', 'test', 'fleet')
MAP_MODES = ('nav', 'amcl')            # map:= 가 필요한 모드
ROUTE_MODES = ('route', 'test')        # roport_line_nav 경로 주행 노드를 함께 띄우는 모드
NO_INCLUDE_MODES = ('ir', 'marker', 'route', 'test', 'fleet')  # pinky_navigation 을 include 하지 않음
IR_NODE_MODES = ('ir', 'route', 'test')  # ir_node:=auto 일 때 IR 센서 노드를 함께 띄우는 모드
ROBOT_NAME = re.compile(r'^pinky_[0-9a-z]{2,8}$')   # roport_pinky_line config/robots.yaml 키


class LaunchArgError(ValueError):
    pass


@dataclass(frozen=True)
class LaunchPlan:
    mode: str
    map_yaml: str  # nav·amcl 모드에서만 사용 (절대경로), 나머지는 ''
    port: int
    use_sim_time: bool
    ir_node: bool = False  # pinky_sensor_adc main_node 를 함께 실행
    ip: str = '0.0.0.0'    # 웹 서버 bind 주소. fleet 기본은 127.0.0.1(SSH 권한이 있으므로)
    robot: str = ''        # mode:=route 의 기체 이름(IR 기준값)


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


def resolve_ip(mode, ip):
    """빈 값이면 fleet 은 127.0.0.1(이 PC 에서만), 나머지는 0.0.0.0. IPv4 만 받는다."""
    raw = str(ip).strip()
    if not raw:
        return '127.0.0.1' if mode == 'fleet' else '0.0.0.0'
    try:
        return str(ipaddress.IPv4Address(raw))
    except ValueError:
        raise LaunchArgError(f"ip must be an IPv4 address, got '{ip}'") from None


def resolve_plan(mode, map_path='', port='8080', use_sim_time='false', ir_node='auto', ip='',
                 robot=''):
    """launch 인자를 검사해 LaunchPlan을 만든다. 잘못되면 LaunchArgError.

    ir_node: auto 이면 mode:=ir|route 일 때만 IR 센서 노드를 함께 띄운다.
    """
    m = str(mode).strip().lower()
    if m not in VALID_MODES:
        raise LaunchArgError(
            f"mode must be one of {'|'.join(VALID_MODES)}, got '{mode}'")

    map_yaml = ''
    if m in MAP_MODES:
        raw = str(map_path).strip()
        if not raw:
            raise LaunchArgError(f'mode:={m} requires map:=<path to map .yaml>')
        map_yaml = os.path.abspath(os.path.expanduser(raw))
        if not map_yaml.endswith('.yaml'):
            raise LaunchArgError(f'map must be a .yaml file: {map_yaml}')
        if not os.path.isfile(map_yaml):
            raise LaunchArgError(f'map file not found: {map_yaml}')

    name = str(robot).strip()
    if m in ROUTE_MODES:
        if not ROBOT_NAME.match(name):
            raise LaunchArgError(
                f"mode:={m} requires robot:=<pinky_xxxx> (IR calibration name), got '{robot}'")
    else:
        name = ''
    ir = str(ir_node).strip().lower()
    return LaunchPlan(
        mode=m,
        map_yaml=map_yaml,
        port=parse_port(port),
        use_sim_time=parse_bool(use_sim_time),
        ir_node=(m in IR_NODE_MODES) if ir == 'auto' else parse_bool(ir, 'ir_node'),
        ip=resolve_ip(m, ip),
        robot=name,
    )


def included_launch(plan):
    """pinky_navigation 에서 include 할 launch 파일 이름과 인자. mode:=ir|marker|route|fleet 는 (None, {}).

    bringup_robot.launch.xml(모터·라이다)은 포함하지 않는다: 사용자가 먼저 따로 실행.
    """
    if plan.mode in NO_INCLUDE_MODES:
        return None, {}
    sim_time = 'True' if plan.use_sim_time else 'False'
    if plan.mode == 'slam':
        # 원본 map_building.launch.xml = slam_toolbox online_sync + mapper_params.yaml
        return 'map_building.launch.xml', {'use_sim_time': sim_time}
    if plan.mode == 'amcl':
        # 원본 localization_launch.xml = map_server + amcl 만 (경로 계획·주행 제어 없음 → 자율 주행 없음)
        return 'localization_launch.xml', {'map': plan.map_yaml, 'use_sim_time': sim_time}
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


# Pinky 이미지에는 libcamera 0.3(/usr/local, rpicam-apps 빌드)과 ROS 의 ros-jazzy-libcamera(0.7)가
# 함께 있다. ROS 환경을 source 하면 libpisp 만 /opt/ros/jazzy/lib 쪽이 잡혀 /usr/local 의
# 카메라 IPA 모듈과 맞지 않아 Picamera2 가 'undefined symbol' 로 죽는다. /usr/local 을 앞에 둔다.
PICAMERA_LIB_DIR = '/usr/local/lib/aarch64-linux-gnu'


def camera_env(mode, camera, environ, exists=os.path.exists):
    """web_server 프로세스에 줄 추가 환경 변수. mode:=marker + picamera2 일 때만."""
    if mode != 'marker' or str(camera).strip() != 'picamera2':
        return {}
    if not exists(os.path.join(PICAMERA_LIB_DIR, 'libpisp.so.1')):
        return {}
    current = environ.get('LD_LIBRARY_PATH', '')
    paths = [p for p in current.split(':') if p and p != PICAMERA_LIB_DIR]
    return {'LD_LIBRARY_PATH': ':'.join([PICAMERA_LIB_DIR] + paths)}
