"""배치 데이터 → Gazebo(gz sim) SDF world 생성.

패키지 폴더에서 실행한다:
  python3 -m roport_gz_world.generate_world            # worlds/roport_airport.sdf 갱신
  python3 -m roport_gz_world.generate_world --check    # 커밋된 world 와 배치 데이터가 같은지 검사

world 플러그인 구성은 원본 pinky_gz_sim/worlds/pinky_map.world 와 같게 둔다
(Sensors + ogre2 가 있어야 Pinky 라이다·카메라가 동작).
"""

import argparse
import os
import sys

from .layout import Layout

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_LAYOUT = os.path.join(PKG_DIR, 'layout', 'roport_airport.yaml')
DEFAULT_WORLD = os.path.join(PKG_DIR, 'worlds', 'roport_airport.sdf')

COLORS = {
    'ground': '0.42 0.42 0.43 1',   # 회색 카펫
    'wall': '0.95 0.95 0.95 1',     # 흰 폼보드
    'object': '0.85 0.84 0.78 1',
    'line': '1 1 1 1',              # 흰 테이프
    'marker': '0.85 0.35 0.10 1',
}

HEADER = """<?xml version="1.0" ?>
<!-- 자동 생성 파일: 직접 고치지 말고 layout/roport_airport.yaml 을 고친 뒤
     python3 -m roport_gz_world.generate_world 로 다시 만든다. -->
<sdf version="1.8">
  <world name="roport_airport">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 2 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>0 0 -1</direction>
    </light>
"""


def _f(v):
    s = f'{v:.4f}'
    return '0.0000' if s == '-0.0000' else s


def _material(color):
    return f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'


def _box_elements(box, color, collision):
    pose = f'{_f(box.cx)} {_f(box.cy)} {_f(box.cz)} 0 0 {_f(box.yaw)}'
    size = f'{_f(box.sx)} {_f(box.sy)} {_f(box.sz)}'
    geom = f'<geometry><box><size>{size}</size></box></geometry>'
    out = [f'        <visual name="{box.name}_visual"><pose>{pose}</pose>{geom}'
           f'{_material(color)}</visual>']
    if collision:
        out.append(f'        <collision name="{box.name}_collision"><pose>{pose}</pose>'
                   f'{geom}</collision>')
    return out


def _model(name, boxes, color, collision, comment):
    lines = [f'    <!-- {comment} -->',
             f'    <model name="{name}">',
             '      <static>true</static>',
             f'      <link name="{name}_link">']
    for b in boxes:
        lines += _box_elements(b, color, collision)
    lines += ['      </link>', '    </model>']
    return lines


def _ground(layout):
    # 실내 영역 중심에 바닥 표시, 충돌 평면은 넓게
    cx, cy = layout.frame.to_world(221.5, 110.0)
    pose = f'{_f(cx)} {_f(cy)} 0 0 0 {_f(layout.frame.yaw_deg * 3.141592653589793 / 180)}'
    return [
        '    <model name="ground">',
        '      <static>true</static>',
        '      <link name="ground_link">',
        '        <collision name="ground_collision"><geometry><plane><normal>0 0 1</normal>'
        '<size>20 20</size></plane></geometry></collision>',
        f'        <visual name="ground_visual"><pose>{pose}</pose><geometry><plane>'
        f'<normal>0 0 1</normal><size>6 4</size></plane></geometry>'
        f'{_material(COLORS["ground"])}</visual>',
        '      </link>',
        '    </model>',
    ]


def build_sdf(layout):
    parts = [HEADER.rstrip('\n')]
    parts += _ground(layout)
    parts += _model('walls', layout.walls, COLORS['wall'], True,
                    f'가벽 두께 {layout.wall_thickness} cm, 충돌 있음')
    parts += _model('objects', layout.objects, COLORS['object'], True, '물체, 충돌 있음')
    parts += _model('floor_lines', layout.lines, COLORS['line'], False,
                    f'바닥 라인 폭 {layout.line_width} cm, 시각용(충돌 없음)')
    parts += _model('markers', layout.markers, COLORS['marker'], False,
                    'ArUco 마커 위치 표시판, 시각용(충돌 없음)')
    parts += ['  </world>', '</sdf>', '']
    return '\n'.join(parts)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--layout', default=DEFAULT_LAYOUT)
    ap.add_argument('--output', default=DEFAULT_WORLD)
    ap.add_argument('--check', action='store_true',
                    help='do not write; exit 1 if the world file is out of date')
    args = ap.parse_args(argv)

    sdf = build_sdf(Layout.load(args.layout))
    if args.check:
        try:
            with open(args.output, 'r', encoding='utf-8') as f:
                current = f.read()
        except OSError:
            current = None
        if current != sdf:
            print(f'{args.output} is out of date; run python3 -m roport_gz_world.generate_world',
                  file=sys.stderr)
            return 1
        print(f'{args.output} is up to date')
        return 0
    with open(args.output, 'w', encoding='utf-8') as f:
        f.write(sdf)
    print(f'wrote {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
