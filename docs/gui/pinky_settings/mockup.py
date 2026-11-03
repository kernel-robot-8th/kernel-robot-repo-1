"""웹 통합관제 「Pinky 탭 > Pinky Settings」 화면 시안 (matplotlib 와이어프레임).

색·메뉴 구조는 웹 관제(PR #25, roport_control_server/spatial_assets/style.css·index.html)에 맞춤.
python3 mockup.py <출력 폴더>
"""

import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Polygon, Rectangle  # noqa: E402

for style in ('Regular', 'Bold'):
    p = f'/usr/share/fonts/opentype/noto/NotoSansCJK-{style}.ttc'
    if os.path.exists(p):
        font_manager.fontManager.addfont(p)
plt.rcParams['font.family'] = font_manager.FontProperties(
    fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()

# 웹 관제 style.css :root
BG, PANEL, INK, MUTED, LINE, SOFT = '#f5f6f7', '#ffffff', '#292f33', '#69757d', '#d7dde0', '#edf0f2'
ACCENT, WARN = '#547e87', '#926b32'
OK, ERR, OFF, NOTE = '#2f855a', '#b23b3b', '#9aa3a9', '#e0457b'
W, H = 1600, 1000
ZOFF = 0   # 대화상자를 그릴 때 100으로 올려 어두운 배경 위에 그림


def zz(z):
    return ZOFF + z


def canvas():
    global ZOFF
    ZOFF = 0
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)
    ax.axis('off')
    ax.add_patch(Rectangle((0, 0), W, H, fc=BG, ec='none'))
    return fig, ax


def box(ax, x, y, w, h, fc=PANEL, ec=LINE, lw=1.2, r=6, z=1, ls='-'):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f'round,pad=0,rounding_size={r}',
                                fc=fc, ec=ec, lw=lw, zorder=zz(z), ls=ls))


def t(ax, x, y, s, size=11, color=INK, weight='normal', ha='left', va='center', z=5):
    ax.text(x, y, s, fontsize=size, color=color, fontweight=weight, ha=ha, va=va, zorder=zz(z))


def button(ax, x, y, w, label, kind='outline', h=34, z=4):
    fc, ec, fg = {'primary': (INK, INK, 'white'), 'outline': (PANEL, '#aab4ba', INK),
                  'danger': (PANEL, ERR, ERR), 'disabled': (SOFT, LINE, OFF),
                  'accent': (ACCENT, ACCENT, 'white')}[kind]
    box(ax, x, y, w, h, fc=fc, ec=ec, r=4, z=z)
    t(ax, x + w / 2, y + h / 2, label, 11, fg, 'bold' if kind in ('primary', 'accent') else 'normal',
      ha='center', z=z + 1)


def arrow(ax, x, y, z=6):
    ax.add_patch(Polygon([(x - 6, y - 3), (x + 6, y - 3), (x, y + 4)], closed=True, fc=MUTED,
                         ec='none', zorder=zz(z)))


def checkbox(ax, x, y, on, z=6):
    ax.add_patch(Rectangle((x - 7, y - 7), 14, 14, fc=INK if on else PANEL, ec=INK, lw=1.3,
                           zorder=zz(z)))
    if on:
        t(ax, x, y, '✓', 9, 'white', 'bold', ha='center', z=z + 1)


def radio(ax, x, y, on, z=6):
    ax.add_patch(Circle((x, y), 8, fc=INK if on else PANEL, ec=INK, lw=1.5, zorder=zz(z)))


def dot(ax, x, y, color, r=5, z=6):
    ax.add_patch(Circle((x, y), r, fc=color, ec='none', zorder=zz(z)))


def field(ax, x, y, w, label, value, dropdown=False, label_w=130, h=32, value_color=INK, z=4,
          hint=None):
    t(ax, x, y + h / 2, label, 10.5, MUTED, z=z)
    box(ax, x + label_w, y, w - label_w, h, fc=PANEL, ec='#b8c2c7', r=4, z=z)
    t(ax, x + label_w + 10, y + h / 2, value, 11, value_color, z=z + 1)
    if dropdown:
        arrow(ax, x + w - 16, y + h / 2, z=z + 1)
    if hint:
        t(ax, x + label_w, y + h + 11, hint, 9.5, MUTED, z=z)


def note(ax, x, y, n, text=None, ha='left'):
    """시안 설명용 분홍 번호 표시(실제 화면에는 없음)."""
    ax.add_patch(Circle((x, y), 13, fc=NOTE, ec='white', lw=1.5, zorder=zz(20)))
    t(ax, x, y, str(n), 10.5, 'white', 'bold', ha='center', z=21)
    if text:
        t(ax, x + (18 if ha == 'left' else -18), y, text, 10, NOTE, 'bold', ha=ha, z=21)


def table(ax, x, y, widths, header, rows, row_h=38, selected=None, colors=None):
    total = sum(widths)
    box(ax, x, y, total, row_h * (len(rows) + 1), fc=PANEL, r=4)
    ax.add_patch(Rectangle((x, y), total, row_h, fc=SOFT, ec='none', zorder=zz(2)))
    cx = x
    for w_, h_ in zip(widths, header):
        t(ax, cx + 10, y + row_h / 2, h_, 10.5, INK, 'bold')
        cx += w_
    for i, row in enumerate(rows):
        ry = y + row_h * (i + 1)
        fc = '#e3ecee' if i == selected else PANEL
        ax.add_patch(Rectangle((x, ry), total, row_h, fc=fc, ec='none', zorder=zz(2)))
        ax.plot([x, x + total], [ry, ry], color=LINE, lw=1, zorder=zz(3))
        cx = x
        for j, (w_, cell) in enumerate(zip(widths, row)):
            col = (colors or {}).get((i, j), INK)
            if cell in ('[ ]', '[v]'):
                checkbox(ax, cx + 18, ry + row_h / 2, cell == '[v]')
            elif isinstance(cell, tuple):        # (점 색, 글자)
                dot(ax, cx + 16, ry + row_h / 2, cell[0])
                t(ax, cx + 28, ry + row_h / 2, cell[1], 10.5, col)
            else:
                t(ax, cx + 10, ry + row_h / 2, cell, 10.5, col)
            cx += w_


def web_frame(ax):
    """웹 관제 상단 메뉴: 로고 칸 + 통합관제 / Pinky / OMX, Pinky 탭 안의 하위 탭."""
    ax.add_patch(Rectangle((0, 0), W, 54, fc=PANEL, ec='none', zorder=zz(3)))
    ax.plot([0, W], [54, 54], color=LINE, lw=1, zorder=zz(3))
    ax.add_patch(Rectangle((0, 0), 140, 54, fc=INK, ec='none', zorder=zz(4)))
    t(ax, 70, 27, 'R O : P O R T', 11.5, 'white', 'bold', ha='center', z=5)
    for i, (name, on) in enumerate((('통합관제', False), ('Pinky', True), ('OMX', False))):
        x = 170 + i * 110
        t(ax, x + 40, 27, name, 12, INK if on else MUTED, 'bold' if on else 'normal', ha='center',
          z=5)
        if on:
            ax.add_patch(Rectangle((x, 50), 80, 4, fc=ACCENT, ec='none', zorder=zz(5)))
    box(ax, W - 220, 14, 200, 26, fc='#fbf3e6', ec='#e2c99d', r=13, z=4)
    t(ax, W - 120, 27, 'OFFLINE · 실행부 미연결', 10, WARN, ha='center', z=5)
    # Pinky 탭 하위 탭
    for i, (name, on) in enumerate((('Pinky 세부 관제', False), ('Pinky Settings', True))):
        x = 24 + i * 170
        box(ax, x, 68, 156, 34, fc=INK if on else PANEL, ec=INK if on else LINE, r=17, z=3)
        t(ax, x + 78, 85, name, 11, 'white' if on else INK, 'bold' if on else 'normal',
          ha='center', z=4)


def legend(ax, text):
    box(ax, 16, H - 42, W - 32, 30, fc='#fdeef4', ec=NOTE, r=4, z=19)
    t(ax, 30, H - 27, text, 10, NOTE, z=21)


def dim_background(ax):
    global ZOFF
    web_frame(ax)
    t(ax, 24, 132, 'Pinky Settings', 20, INK, 'bold')
    ax.add_patch(Rectangle((0, 0), W, H, fc='#1f2427', ec='none', alpha=0.45, zorder=50))
    ZOFF = 100


# ------------------------------------------------------------------ 1. 메인
def screen_main(path):
    fig, ax = canvas()
    web_frame(ax)
    x0 = 24
    t(ax, x0, 132, 'Pinky Settings', 20, INK, 'bold')
    t(ax, x0, 160, '등록 3대 · 관제 서버 저장소 main @ 99ab80f · 최신 릴리스 pinky-v0.3.0 · '
      'MQTT 사이트 demo', 11, MUTED)
    button(ax, 900, 118, 150, '+ Pinky 추가', 'primary')
    button(ax, 1060, 118, 170, '선택한 Pinky 업데이트')
    button(ax, 1240, 118, 150, '모두 연결 확인')
    note(ax, 900, 114, 1)
    note(ax, 1060, 114, 2)

    widths = [36, 96, 100, 116, 120, 118, 128, 122, 128]
    header = ['', '로봇 ID', '역할', 'IP (SSH)', '기체', 'ROS 도메인', '연결', '설치 버전', '상태']
    rows = [
        ['[ ]', 'pinky1', '물류', '192.0.2.11', 'pinky_d660', '14', (OK, 'SSH·MQTT'),
         'v0.3.0', (OK, '최신')],
        ['[v]', 'pinky2', '물류', '192.0.2.12', 'pinky_6422', '15', (OK, 'SSH·MQTT'),
         'v0.2.1', (WARN, '업데이트 있음')],
        ['[ ]', 'pinky3', '스마트카트', '192.0.2.13', 'pinky_764e', '16 (미적용)', (WARN, 'SSH만'),
         '없음', (WARN, '기체 설정 필요')],
    ]
    table(ax, x0, 190, widths, header, rows, selected=1,
          colors={(1, 8): WARN, (2, 5): WARN, (2, 6): WARN, (2, 7): WARN, (2, 8): WARN})
    note(ax, x0 - 4, 208, 3)
    note(ax, x0 + 36 + 96 + 100 + 116 + 120 + 60, 186, 5)
    t(ax, x0, 360, '로봇 ID(pinky1~3)는 MQTT 토픽·관제 배정의 기준이며 한 대에 하나만 씀. '
      'ROS 도메인은 기체마다 달라야 함(같으면 다른 Pinky의 cmd_vel을 받을 수 있음).', 10, MUTED)

    # 설치 기록
    box(ax, x0, 382, 1100, 556, fc=PANEL)
    t(ax, x0 + 16, 406, '설치 · 설정 기록', 13.5, INK, 'bold')
    logs = [
        ('11-03 10:21', 'pinky1', 'v0.2.1 → v0.3.0', (OK, '완료'), '전송 4 s · 빌드 2분 31초 · 전환'),
        ('11-03 10:24', 'pinky2', 'v0.2.1 → v0.3.0', (ERR, '빌드 실패'), 'roport_pinky_line · 로그 보기 · v0.2.1 유지'),
        ('11-03 11:02', 'pinky3', '등록', (OK, '완료'), '새 SD카드 · 기체 설정 없음 → 「기체 설정 적용」 필요'),
    ]
    table(ax, x0 + 16, 426, [120, 90, 160, 130, 560], ['시각', '대상', '내용', '결과', '상세'],
          [list(r) for r in logs], row_h=36, colors={(1, 3): ERR})
    t(ax, x0 + 16, 600, '진행 중: pinky2 · v0.3.0 다시 설치', 12, INK, 'bold')
    steps = [('연결 확인', 'done'), ('소스 전송', 'done'), ('빌드 3/5', 'run'), ('전환', 'wait'),
             ('완료', 'wait')]
    for i, (name, state) in enumerate(steps):
        sx = x0 + 16 + i * 210
        fc = {'done': OK, 'run': ACCENT, 'wait': SOFT}[state]
        box(ax, sx, 620, 196, 34, fc=fc, ec=fc, r=17)
        t(ax, sx + 98, 637, ('✓ ' if state == 'done' else '▶ ' if state == 'run' else '') + name,
          11, 'white' if state != 'wait' else MUTED, 'bold', ha='center')
    box(ax, x0 + 16, 668, 1068, 212, fc='#1f2427', ec='#1f2427', r=4)
    log = ['$ colcon build --packages-select pinky_interfaces roport_pinky_line roport_pinky '
           '--parallel-workers 1',
           'Starting >>> pinky_interfaces', 'Finished <<< pinky_interfaces [41.2s]',
           'Starting >>> roport_pinky_line', 'Finished <<< roport_pinky_line [12.8s]',
           'Starting >>> roport_pinky ...']
    for i, line in enumerate(log):
        t(ax, x0 + 30, 690 + i * 24, line, 10, '#b9f6ca' if 'Finished' in line else '#d6e2e6')
    button(ax, x0 + 16, 892, 120, '설치 중지', 'danger', h=30)
    t(ax, x0 + 150, 907, '빌드가 끝나기 전에 멈추면 이전 버전을 그대로 씀', 10, MUTED)

    # 상세 패널
    px = 1140
    box(ax, px, 190, 436, 748, fc=PANEL)
    t(ax, px + 16, 216, 'pinky2 상세', 14, INK, 'bold')
    dot(ax, px + 136, 216, OK)
    t(ax, px + 148, 216, 'SSH·MQTT 연결', 10.5, OK)
    t(ax, px + 16, 250, '기본', 11.5, ACCENT, 'bold')
    field(ax, px + 16, 264, 404, '로봇 ID', 'pinky2', dropdown=True, label_w=120)
    field(ax, px + 16, 302, 404, '역할', '물류', dropdown=True, label_w=120)
    note(ax, px + 432, 318, 4)
    field(ax, px + 16, 340, 404, 'IP (SSH)', '192.0.2.12', label_w=120)
    t(ax, px + 16, 388, '기체  pinky_6422 · PP_26072 · RAM 2 GB · IR 기준값 pinky_6422', 10, MUTED)

    t(ax, px + 16, 420, '기체 설정 (관제 등록값 = 기체 적용값)', 11.5, ACCENT, 'bold')
    field(ax, px + 16, 434, 404, 'ROS 도메인 ID', '15', label_w=120)
    t(ax, px + 330, 450, '✓ 적용됨', 10.5, OK, 'bold')
    note(ax, px + 432, 450, 5)
    t(ax, px + 136, 478, '사용 중 14(pinky1)·16(pinky3) · 겹치면 저장 안 됨', 9.5, MUTED)
    field(ax, px + 16, 492, 404, 'DDS (RMW)', 'rmw_fastrtps_cpp', dropdown=True, label_w=120)
    field(ax, px + 16, 530, 404, 'MQTT 브로커', '192.0.2.10:1883 · 사이트 demo', label_w=120)
    field(ax, px + 16, 568, 404, 'MQTT 계정', 'pinky2 · 비밀값 등록됨', label_w=120)
    button(ax, px + 16, 612, 196, '기체 설정 적용…', 'accent')
    t(ax, px + 222, 629, '화면 4 · 값이 다르면 노란색', 10, MUTED)

    t(ax, px + 16, 670, '설치', 11.5, ACCENT, 'bold')
    t(ax, px + 16, 694, '현재  v0.2.1 · 9f8e7d6 · 11-02 14:10', 10.5)
    field(ax, px + 16, 708, 404, '설치할 버전', 'pinky-v0.3.0 (릴리스)', dropdown=True, label_w=120)
    button(ax, px + 16, 750, 196, '업데이트', 'primary')
    button(ax, px + 224, 750, 196, '이전 버전으로')
    note(ax, px + 16, 746, 2)
    t(ax, px + 16, 808, '저장공간 3.3 GB · 메모리 1.1/2.0 GB · 배터리 78 % · 주행·임무 없음', 10, MUTED)
    button(ax, px + 16, 832, 196, '연결 확인')
    button(ax, px + 224, 832, 196, '로그 내려받기')
    button(ax, px + 16, 886, 404, '등록 해제', 'danger')

    legend(ax, '분홍 번호 = 요구사항: ① Pinky 추가(IP 등록)  ② 업데이트·릴리스 설치·빌드  ③ 등록된 Pinky 목록  '
               '④ 역할(물류·스마트카트)  ⑤ ROS 도메인 ID 설정(기체별로 다르게)      ※ IP·버전은 예시')
    fig.savefig(path, dpi=100)
    plt.close(fig)


# ------------------------------------------------------------------ 2. 추가
def screen_add(path):
    fig, ax = canvas()
    dim_background(ax)
    dx, dy, dw, dh = 400, 70, 800, 880
    box(ax, dx, dy, dw, dh, fc=PANEL, ec=LINE, r=10, z=1)
    z = 11
    t(ax, dx + 28, dy + 36, 'Pinky 추가', 17, INK, 'bold', z=z)
    t(ax, dx + 28, dy + 64, '같은 네트워크의 Pinky를 IP로 등록함. 비밀번호는 받지 않고 SSH 키로만 접속함.',
      10.5, MUTED, z=z)
    fx, fw, lw = dx + 28, 744, 160
    field(ax, fx, dy + 92, fw, 'IP 주소 *', '192.0.2.13', label_w=lw, z=z,
          hint='SSH 접속용. 공유기에서 IP 고정(DHCP 예약) 권장')
    field(ax, fx, dy + 150, fw, 'SSH 사용자 · 키', 'pinky · ~/.ssh/pinky_ed25519', dropdown=True,
          label_w=lw, z=z)
    field(ax, fx, dy + 196, fw, '로봇 ID *', 'pinky3', dropdown=True, label_w=lw, z=z,
          hint='pinky1~3 중 아직 안 쓴 것만 고를 수 있음 (MQTT 토픽·관제 배정 기준)')
    t(ax, fx, dy + 270, '역할 *', 10.5, MUTED, z=z)
    for i, (name, on) in enumerate((('물류', False), ('스마트카트', True))):
        cx = fx + lw + 12 + i * 150
        radio(ax, cx, dy + 270, on, z=z)
        t(ax, cx + 16, dy + 270, name, 11.5, z=z)
    note(ax, fx + 520, dy + 270, 4)
    field(ax, fx, dy + 300, fw, 'ROS 도메인 ID *', '16', label_w=lw, z=z,
          hint='추천값 16 · 사용 중: 14 pinky1, 15 pinky2 · 0~101 사이, 다른 Pinky와 겹치면 등록 안 됨')
    note(ax, fx + fw + 20, dy + 316, 5)

    button(ax, fx, dy + 372, 160, '연결 확인', 'accent', z=z)
    t(ax, fx + 176, dy + 389, '통과해야 [등록]이 켜짐 · 읽기만 함(bringup·노드 실행·모터 구동 없음)', 10.5, MUTED,
      z=z)
    box(ax, fx, dy + 418, fw, 330, fc='#f9fafb', ec=LINE, r=6, z=z)
    checks = [(OK, 'SSH 접속 (키 인증, 호스트 키 확인)', '통과'),
              (OK, 'Ubuntu 24.04 · ROS 2 Jazzy', '확인'),
              (OK, '저장공간', '12.1 GB 남음'),
              (OK, '기체 식별 (시리얼)', 'PP_26091 → pinky_764e'),
              (OK, 'IR 기준값 프로필 (robots.yaml)', 'pinky_764e 있음'),
              (WARN, '기체 설정 (로봇 ID·도메인·MQTT)', '없음 (새 SD카드) → 등록 후 화면 4'),
              (WARN, '설치된 RO:PORT 버전', '없음 → 등록 후 설치'),
              (OK, '로봇 ID·도메인 중복', 'pinky3 · 16 사용 가능'),
              (ERR, '(예) SSH 키 미등록이면', '아래 안내 표시')]
    for i, (c, k, v) in enumerate(checks):
        yy = dy + 442 + i * 32
        dot(ax, fx + 20, yy, c, z=z)
        t(ax, fx + 36, yy, k, 10.5, z=z + 1)
        t(ax, fx + fw - 20, yy, v, 10.5, c, ha='right', z=z + 1)
    t(ax, fx, dy + 772, 'SSH 키가 없는 새 기체: 관제 서버 터미널에서 한 번만 실행 '
      '(비밀번호는 터미널에만 입력, 화면은 받지 않음)', 10.5, MUTED, z=z)
    box(ax, fx, dy + 786, fw, 30, fc='#1f2427', ec='#1f2427', r=4, z=z)
    t(ax, fx + 12, dy + 801, 'ssh-copy-id -i ~/.ssh/pinky_ed25519.pub pinky@192.0.2.13', 10.5,
      '#d6e2e6', z=z + 1)
    button(ax, dx + dw - 300, dy + dh - 52, 130, '취소', z=z)
    button(ax, dx + dw - 158, dy + dh - 52, 130, '등록', 'primary', z=z)
    note(ax, dx + dw - 158, dy + dh - 56, 1)
    fig.savefig(path, dpi=100)
    plt.close(fig)


# ------------------------------------------------------------------ 3. 업데이트·릴리스
def screen_update(path):
    fig, ax = canvas()
    dim_background(ax)
    dx, dy, dw, dh = 300, 70, 1000, 880
    box(ax, dx, dy, dw, dh, fc=PANEL, ec=LINE, r=10, z=1)
    z = 11
    t(ax, dx + 28, dy + 36, 'Pinky 소스 설치 (업데이트 · 릴리스)', 17, INK, 'bold', z=z)
    t(ax, dx + 28, dy + 64, '관제 서버 저장소에서 고른 버전을 Pinky로 보내고 Pinky에서 빌드함. '
      'Pinky는 인터넷·GitHub 접속이 필요 없음.', 10.5, MUTED, z=z)
    note(ax, dx + dw - 40, dy + 36, 2)

    t(ax, dx + 28, dy + 104, '1. 버전', 12.5, ACCENT, 'bold', z=z)
    for i, (name, on) in enumerate((('릴리스 (태그, 시연용)', True), ('개발 (브랜치 최신, 시험용)', False))):
        cx = dx + 48 + i * 260
        radio(ax, cx, dy + 136, on, z=z)
        t(ax, cx + 16, dy + 136, name, 11, z=z)
    table(ax, dx + 28, dy + 158, [170, 130, 140, 500],
          ['버전', '커밋', '날짜', '내용'],
          [['● pinky-v0.3.0', 'd960c64', '11-05', 'IR 노드 주행·경로 (권장)'],
           ['○ pinky-v0.2.1', '9f8e7d6', '11-02', '라인 추종 v0'],
           ['○ pinky-v0.2.0', '4b116ff', '10-31', 'IR 측정 화면']], row_h=34, selected=0)

    t(ax, dx + 28, dy + 318, '2. 대상', 12.5, ACCENT, 'bold', z=z)
    table(ax, dx + 28, dy + 340, [40, 110, 110, 170, 510],
          ['', '로봇 ID', '역할', '현재 → 설치', '설치할 패키지 (역할에 따라 자동)'],
          [['[v]', 'pinky1', '물류', 'v0.2.1 → v0.3.0', 'pinky_pro(원본) · roport_pinky_line · roport_pinky'],
           ['[v]', 'pinky2', '물류', 'v0.2.1 → v0.3.0', 'pinky_pro(원본) · roport_pinky_line · roport_pinky'],
           ['[ ]', 'pinky3', '스마트카트', '기체 설정 먼저', '+ roport_smartcart (기체 설정 적용 후 선택 가능)']],
          row_h=34, colors={(2, 1): OFF, (2, 2): OFF, (2, 3): WARN, (2, 4): OFF})

    t(ax, dx + 28, dy + 500, '3. 사전 확인 (하나라도 안 되면 그 Pinky는 설치하지 않음)', 12.5, ACCENT, 'bold', z=z)
    checks = [(OK, '주행·임무 중 아님 (라인 추종·Nav2 꺼짐)'), (OK, '배터리 30 % 이상'),
              (WARN, '저장공간 1 GB 이상 — pinky2: 3.3 GB (경고만)'), (OK, '기체 설정(로봇 ID·도메인) 적용됨'),
              (OK, '관제 서버 저장소에 pinky-v0.3.0 있음'), (OK, '로봇 ID·도메인 중복 없음')]
    for i, (c, s) in enumerate(checks):
        cx = dx + 48 + (i % 2) * 460
        cy = dy + 532 + (i // 2) * 28
        dot(ax, cx, cy, c, z=z)
        t(ax, cx + 14, cy, s, 10.5, z=z)

    t(ax, dx + 28, dy + 640, '4. 진행', 12.5, ACCENT, 'bold', z=z)
    for r, (name, states) in enumerate((('pinky1', ['done'] * 5),
                                        ('pinky2', ['done', 'done', 'run', 'wait', 'wait']))):
        yy = dy + 664 + r * 42
        t(ax, dx + 28, yy + 15, name, 11, z=z)
        for i, (step, st) in enumerate(zip(('연결 확인', '소스 전송', '빌드', '새 버전으로 전환', '완료'),
                                           states)):
            sx = dx + 120 + i * 170
            fc = {'done': OK, 'run': ACCENT, 'wait': SOFT}[st]
            box(ax, sx, yy, 160, 30, fc=fc, ec=fc, r=15, z=z)
            t(ax, sx + 80, yy + 15, ('✓ ' if st == 'done' else '▶ ' if st == 'run' else '') + step,
              10.5, 'white' if st != 'wait' else MUTED, 'bold', ha='center', z=z + 1)
    t(ax, dx + 28, dy + 770, '빌드가 성공했을 때만 새 버전으로 전환, 실패하면 이전 버전 유지. 파일 복사·빌드만 하고 '
      'bringup·노드 재시작은 하지 않음. 2 GB 기체는 --parallel-workers 1.', 10.5, MUTED, z=z)
    button(ax, dx + dw - 300, dy + dh - 52, 130, '취소', z=z)
    button(ax, dx + dw - 158, dy + dh - 52, 130, '설치 시작', 'primary', z=z)
    fig.savefig(path, dpi=100)
    plt.close(fig)


# ------------------------------------------------------------------ 4. 기체 설정 적용
def screen_device(path):
    fig, ax = canvas()
    dim_background(ax)
    dx, dy, dw, dh = 300, 70, 1000, 880
    box(ax, dx, dy, dw, dh, fc=PANEL, ec=LINE, r=10, z=1)
    z = 11
    t(ax, dx + 28, dy + 36, '기체 설정 적용 — pinky3 (pinky_764e)', 17, INK, 'bold', z=z)
    t(ax, dx + 28, dy + 64, '관제에 등록한 값을 Pinky에 씀. 새 SD카드처럼 설정이 없는 기체를 처음 준비할 때와 '
      '도메인 ID를 바꿀 때 씀.', 10.5, MUTED, z=z)
    note(ax, dx + dw - 40, dy + 36, 5)

    t(ax, dx + 28, dy + 104, '1. 바꿀 값 (관제 등록값 ↔ 기체 현재값)', 12.5, ACCENT, 'bold', z=z)
    rows = [
        ['로봇 ID', 'pinky3', '없음', (WARN, '적용')],
        ['ROS 도메인 ID', '16', '0 (설정 없음, 기본값)', (WARN, '적용')],
        ['DDS (RMW)', '기본 (rmw_fastrtps_cpp)', '기본', (OK, '같음')],
        ['MQTT 브로커 · 사이트', '192.0.2.10:1883 · demo', '없음', (WARN, '적용')],
        ['MQTT 계정', 'pinky3 · 비밀값(화면에 안 보임)', '없음', (WARN, '적용')],
        ['IR 기준값 프로필', 'pinky_764e', '소스 설치 후 생김', (OFF, '설치 때')],
    ]
    table(ax, dx + 28, dy + 124, [210, 300, 260, 160], ['항목', '관제 등록값', '기체 현재값', '처리'],
          rows, row_h=36, colors={(5, 2): OFF})
    t(ax, dx + 28, dy + 392, '도메인 ID 16: 사용 중인 14(pinky1)·15(pinky2)와 겹치지 않음 ✓  '
      '(0~101, 같은 Wi-Fi에서 같은 값이면 서로 cmd_vel을 받을 수 있음)', 10.5, OK, z=z)

    t(ax, dx + 28, dy + 432, '2. 기체에 쓰는 내용', 12.5, ACCENT, 'bold', z=z)
    box(ax, dx + 28, dy + 448, dw - 56, 150, fc='#1f2427', ec='#1f2427', r=4, z=z)
    lines = ['# ~/.roport/pinky.env  (권한 600, 관제가 씀)',
             'ROBOT_ID=pinky3',
             'ROS_DOMAIN_ID=16',
             'MQTT_HOST=192.0.2.10  MQTT_PORT=1883  MQTT_SITE=demo  MQTT_USERNAME=pinky3',
             'MQTT_PASSWORD_FILE=~/.roport/mqtt_password   # 비밀값은 별도 파일(600), 로그·화면에 안 남김',
             '# ~/.bashrc 에 한 줄:  [ -f ~/.roport/pinky.env ] && set -a && . ~/.roport/pinky.env && set +a']
    for i, line in enumerate(lines):
        t(ax, dx + 42, dy + 468 + i * 22, line, 10, '#9fb3b9' if line.startswith('#') else '#d6e2e6',
          z=z + 1)

    t(ax, dx + 28, dy + 628, '3. 적용 후', 12.5, ACCENT, 'bold', z=z)
    after = [(WARN, '실행 중인 ROS 노드(bringup 등)는 새 도메인을 쓰려면 다시 켜야 함 — 현장에서 손으로 (관제가 자동 재시작 안 함)'),
             (OK, '적용 직후 읽어서 확인: 기체 현재값이 관제 등록값과 같으면 목록의 「16 (미적용)」이 「16」으로 바뀜'),
             (OK, '되돌리기: 이전 pinky.env를 pinky.env.bak으로 남김')]
    for i, (c, s) in enumerate(after):
        yy = dy + 656 + i * 30
        dot(ax, dx + 44, yy, c, z=z)
        t(ax, dx + 58, yy, s, 10.5, z=z)
    t(ax, dx + 28, dy + 760, '주행·임무 중이면 적용 버튼이 꺼짐. 원격 명령은 정해진 스크립트만 실행(셸 문자열 조합 없음).',
      10.5, MUTED, z=z)
    button(ax, dx + dw - 300, dy + dh - 52, 130, '취소', z=z)
    button(ax, dx + dw - 158, dy + dh - 52, 130, '기체에 적용', 'primary', z=z)
    fig.savefig(path, dpi=100)
    plt.close(fig)


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    screen_main(os.path.join(out, 'pinky_settings_1_main.png'))
    screen_add(os.path.join(out, 'pinky_settings_2_add.png'))
    screen_update(os.path.join(out, 'pinky_settings_3_update.png'))
    screen_device(os.path.join(out, 'pinky_settings_4_device.png'))
    print('ok')
