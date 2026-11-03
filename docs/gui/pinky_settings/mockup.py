"""관제 GUI 「설정 > Pinky Settings」 화면 시안 (matplotlib 와이어프레임).

색은 관제 GUI(roport_control_server/adapters/ui/window.py) 스타일시트와 맞춤.
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

BG, PANEL, BORDER, TEXT, MUTED = '#f4f6fa', '#ffffff', '#d8dee8', '#233248', '#526279'
NAVY, HEAD, ALT, TEAL = '#233e60', '#e9eef5', '#f7f9fc', '#79b6bd'
OK, WARN, ERR, OFF, NOTE = '#1f9d55', '#c27c0e', '#c0392b', '#8a94a6', '#e0457b'
W, H = 1600, 1000
ZOFF = 0   # 대화상자를 그릴 때 100으로 올려 어두운 배경 위에 그림


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


def box(ax, x, y, w, h, fc=PANEL, ec=BORDER, lw=1.2, r=6, z=1, ls='-'):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f'round,pad=0,rounding_size={r}',
                                fc=fc, ec=ec, lw=lw, zorder=ZOFF + z, ls=ls))


def t(ax, x, y, s, size=11, color=TEXT, weight='normal', ha='left', va='center', z=5):
    ax.text(x, y, s, fontsize=size, color=color, fontweight=weight, ha=ha, va=va, zorder=ZOFF + z)


def button(ax, x, y, w, label, kind='outline', h=34, z=4):
    fc, ec, fg = {'primary': (NAVY, NAVY, 'white'), 'outline': (PANEL, '#9fb0c6', TEXT),
                  'danger': (PANEL, ERR, ERR), 'disabled': ('#e5e9f0', '#cbd3df', '#747e8d'),
                  'teal': (TEAL, TEAL, 'white')}[kind]
    box(ax, x, y, w, h, fc=fc, ec=ec, r=5, z=z)
    t(ax, x + w / 2, y + h / 2, label, 11, fg, 'bold' if kind == 'primary' else 'normal',
      ha='center', z=z + 1)


def field(ax, x, y, w, label, value, dropdown=False, label_w=130, h=32):
    t(ax, x, y + h / 2, label, 10.5, MUTED)
    box(ax, x + label_w, y, w - label_w, h, fc=PANEL, ec='#b8c4d4', r=4)
    t(ax, x + label_w + 10, y + h / 2, value, 11)
    if dropdown:
        arrow(ax, x + w - 16, y + h / 2)


def arrow(ax, x, y, z=6):
    ax.add_patch(Polygon([(x - 6, y - 3), (x + 6, y - 3), (x, y + 4)], closed=True, fc=MUTED,
                         ec='none', zorder=ZOFF + z))


def checkbox(ax, x, y, on, z=6):
    ax.add_patch(Rectangle((x - 7, y - 7), 14, 14, fc=NAVY if on else PANEL, ec=NAVY, lw=1.3,
                           zorder=ZOFF + z))
    if on:
        t(ax, x, y, '✓', 9, 'white', 'bold', ha='center', z=z + 1)


def dot(ax, x, y, color, r=5):
    ax.add_patch(Circle((x, y), r, fc=color, ec='none', zorder=ZOFF + 6))


def note(ax, x, y, n, text=None, ha='left'):
    """시안 설명용 분홍 번호 표시(실제 화면에는 없음)."""
    ax.add_patch(Circle((x, y), 13, fc=NOTE, ec='white', lw=1.5, zorder=ZOFF + 20))
    t(ax, x, y, str(n), 10.5, 'white', 'bold', ha='center', z=21)
    if text:
        t(ax, x + (18 if ha == 'left' else -18), y, text, 10, NOTE, 'bold', ha=ha, z=21)


def table(ax, x, y, widths, header, rows, row_h=38, selected=None, colors=None):
    total = sum(widths)
    box(ax, x, y, total, row_h * (len(rows) + 1), fc=PANEL, r=4)
    ax.add_patch(Rectangle((x, y), total, row_h, fc=HEAD, ec='none', zorder=ZOFF + 2))
    cx = x
    for w_, h_ in zip(widths, header):
        t(ax, cx + 10, y + row_h / 2, h_, 10.5, TEXT, 'bold')
        cx += w_
    for i, row in enumerate(rows):
        ry = y + row_h * (i + 1)
        fc = '#dde8f6' if i == selected else (ALT if i % 2 else PANEL)
        ax.add_patch(Rectangle((x, ry), total, row_h, fc=fc, ec='none', zorder=ZOFF + 2))
        ax.plot([x, x + total], [ry, ry], color='#e8edf4', lw=1, zorder=ZOFF + 3)
        cx = x
        for j, (w_, cell) in enumerate(zip(widths, row)):
            col = (colors or {}).get((i, j), TEXT)
            if cell in ('[ ]', '[v]'):          # 체크박스
                checkbox(ax, cx + 18, ry + row_h / 2, cell == '[v]')
            elif isinstance(cell, tuple):        # (점 색, 글자)
                dot(ax, cx + 16, ry + row_h / 2, cell[0])
                t(ax, cx + 28, ry + row_h / 2, cell[1], 10.5, col)
            else:
                t(ax, cx + 10, ry + row_h / 2, cell, 10.5, col)
            cx += w_


def window_frame(ax, title):
    ax.add_patch(Rectangle((0, 0), W, 34, fc='#1b2a40', ec='none', zorder=ZOFF + 3))
    t(ax, 16, 17, title, 11, 'white', z=4)
    for i, c in enumerate(('#ef6b6b', '#f5c04f', '#59c16d')):
        dot(ax, W - 70 + i * 22, 17, c, 6)


def settings_nav(ax, active='Pinky Settings'):
    ax.add_patch(Rectangle((0, 34), 220, H - 34, fc='#e4eaf2', ec='none', zorder=ZOFF + 2))
    t(ax, 24, 70, '설정', 15, TEXT, 'bold')
    for i, name in enumerate(('Pinky Settings', 'OMX Settings', '네트워크 · MQTT', '릴리스 관리')):
        y = 104 + i * 46
        if name == active:
            box(ax, 12, y, 196, 38, fc=NAVY, ec=NAVY, r=5, z=3)
        t(ax, 30, y + 19, name, 11.5, 'white' if name == active else TEXT,
          'bold' if name == active else 'normal')
    button(ax, 16, H - 62, 188, '← 관제 화면으로')


def legend(ax, text):
    box(ax, 236, H - 44, W - 252, 32, fc='#fdeef4', ec=NOTE, r=4, z=19)
    t(ax, 250, H - 28, text, 10, NOTE, z=21)


# ------------------------------------------------------------------ 1. 메인
def screen_main(path):
    fig, ax = canvas()
    window_frame(ax, 'RO:PORT · 통합관제 · 설정')
    settings_nav(ax)
    x0 = 248
    t(ax, x0, 72, 'Pinky Settings', 21, TEXT, 'bold')
    t(ax, x0, 102, '등록된 Pinky 3대 · 관제 PC 저장소 main @ d960c64 · 최신 릴리스 v0.3.0', 11, MUTED)
    button(ax, 1040, 56, 150, '+ Pinky 추가', 'primary')
    button(ax, 1200, 56, 170, '선택한 Pinky 업데이트')
    button(ax, 1380, 56, 150, '모두 연결 확인')
    note(ax, 1040, 52, 1)
    note(ax, 1200, 52, 2)

    widths = [36, 96, 104, 150, 140, 150, 150, 156]
    header = ['', '이름', '역할', 'IP 주소', '기체', '연결', '설치 버전', '설치 상태']
    rows = [
        ['[ ]', 'Pinky 1', '물류', '192.0.2.11', 'pinky_d660', (OK, '온라인 12 ms'),
         'v0.3.0', (OK, '최신')],
        ['[v]', 'Pinky 2', '물류', '192.0.2.12', 'pinky_6422', (OK, '온라인 18 ms'),
         'v0.2.1', (WARN, '업데이트 있음')],
        ['[ ]', 'Pinky 3', '스마트카트', '192.0.2.13', 'pinky_764e', (OFF, '오프라인 5분'),
         'v0.3.0', (OFF, '확인 불가')],
    ]
    table(ax, x0, 130, widths, header, rows, selected=1,
          colors={(1, 7): WARN, (2, 5): OFF, (2, 7): OFF})
    note(ax, x0 - 8, 148, 3)
    t(ax, x0, 300, '행을 누르면 오른쪽에 상세가 열림. 오프라인 Pinky는 업데이트 대상에서 빠짐.', 10, MUTED)

    # 설치 기록
    box(ax, x0, 328, 982, 610, fc=PANEL)
    t(ax, x0 + 16, 352, '설치 기록', 13.5, TEXT, 'bold')
    t(ax, x0 + 120, 352, '최근 작업 · 단계별 결과 · 실패 로그', 10, MUTED)
    logs = [
        ('11-03 10:21', 'Pinky 1', 'v0.2.1 → v0.3.0', (OK, '완료'), '전송 4 s · 빌드 2분 31초 · 전환'),
        ('11-03 10:24', 'Pinky 2', 'v0.2.1 → v0.3.0', (ERR, '빌드 실패'), 'roport_pinky_line · 로그 보기 · v0.2.1 유지'),
        ('11-02 16:40', 'Pinky 3', 'v0.2.0 → v0.3.0', (OK, '완료'), '전송 3 s · 빌드 1분 58초 · 전환'),
        ('11-02 16:02', 'Pinky 3', '역할 변경', (OK, '완료'), '물류 → 스마트카트 · 실행부 재시작 필요 안내'),
    ]
    table(ax, x0 + 16, 372, [120, 90, 160, 130, 450], ['시각', '대상', '내용', '결과', '상세'],
          [list(r) for r in logs], row_h=36, colors={(1, 3): ERR})
    # 진행 중 작업
    t(ax, x0 + 16, 580, '진행 중: Pinky 2 · v0.3.0 다시 설치', 12, TEXT, 'bold')
    steps = [('연결 확인', 'done'), ('소스 전송', 'done'), ('빌드 3/5', 'run'), ('전환', 'wait'),
             ('완료', 'wait')]
    for i, (name, state) in enumerate(steps):
        sx = x0 + 16 + i * 190
        fc = {'done': OK, 'run': TEAL, 'wait': '#e6ebf3'}[state]
        box(ax, sx, 600, 176, 34, fc=fc, ec=fc, r=17)
        t(ax, sx + 88, 617, ('✓ ' if state == 'done' else '▶ ' if state == 'run' else '') + name,
          11, 'white' if state != 'wait' else MUTED, 'bold', ha='center')
    box(ax, x0 + 16, 648, 950, 236, fc='#0f1b2b', ec='#0f1b2b', r=4)
    log = ['$ colcon build --packages-select pinky_interfaces roport_pinky_line roport_pinky '
           '--parallel-workers 1',
           'Starting >>> pinky_interfaces', 'Finished <<< pinky_interfaces [41.2s]',
           'Starting >>> roport_pinky_line', 'Finished <<< roport_pinky_line [12.8s]',
           'Starting >>> roport_pinky ...']
    for i, line in enumerate(log):
        t(ax, x0 + 30, 670 + i * 24, line, 10, '#b9f6ca' if 'Finished' in line else '#d6e2f0')
    button(ax, x0 + 16, 896, 120, '설치 중지', 'danger', h=30)
    t(ax, x0 + 150, 911, '빌드가 끝나기 전에 멈추면 이전 버전을 그대로 씀', 10, MUTED)

    # 상세 패널
    px = 1246
    box(ax, px, 130, 330, 808, fc=PANEL)
    t(ax, px + 16, 156, 'Pinky 2 상세', 14, TEXT, 'bold')
    dot(ax, px + 140, 156, OK)
    t(ax, px + 152, 156, '온라인', 10.5, OK)
    t(ax, px + 16, 192, '기본 정보', 11.5, NAVY, 'bold')
    field(ax, px + 16, 206, 298, '이름', 'Pinky 2', label_w=110)
    field(ax, px + 16, 244, 298, 'IP 주소', '192.0.2.12', label_w=110)
    field(ax, px + 16, 282, 298, '역할', '물류', dropdown=True, label_w=110)
    note(ax, px + 330, 298, 4)
    field(ax, px + 16, 320, 298, '도메인 ID', '14', label_w=110)
    t(ax, px + 16, 370, '기체  pinky_6422 · PP_26072 · RAM 2 GB', 10.5, MUTED)
    t(ax, px + 16, 392, 'IR 기준값  robots.yaml: pinky_6422', 10.5, MUTED)
    t(ax, px + 16, 414, 'DDS  rmw_fastrtps_cpp (기체별 설정)', 10.5, MUTED)

    t(ax, px + 16, 452, '설치', 11.5, NAVY, 'bold')
    t(ax, px + 16, 478, '현재  v0.2.1 · 9f8e7d6 · 11-02 14:10', 10.5)
    field(ax, px + 16, 494, 298, '설치할 버전', 'v0.3.0 (릴리스)', dropdown=True, label_w=110)
    button(ax, px + 16, 536, 140, '업데이트', 'primary')
    button(ax, px + 166, 536, 148, '이전 버전으로')
    note(ax, px + 16, 532, 2)

    t(ax, px + 16, 600, '상태 (연결 확인 시 갱신)', 11.5, NAVY, 'bold')
    for i, (k, v, c) in enumerate([('저장공간', '3.3 GB 남음', WARN), ('메모리', '1.1 / 2.0 GB', TEXT),
                                   ('배터리', '78 %', TEXT), ('주행·임무', '없음 (설치 가능)', OK),
                                   ('마지막 확인', '12 s 전', TEXT)]):
        t(ax, px + 16, 628 + i * 26, k, 10.5, MUTED)
        t(ax, px + 130, 628 + i * 26, v, 10.5, c)
    button(ax, px + 16, 770, 140, '연결 확인')
    button(ax, px + 166, 770, 148, '로그 내려받기')
    button(ax, px + 16, 886, 298, '등록 해제', 'danger')

    legend(ax, '분홍 번호 = 요구사항: ① Pinky 추가(IP 등록)  ② 업데이트·릴리스 설치·빌드  ③ 등록된 Pinky 목록  '
               '④ 역할(물류·스마트카트) 설정      ※ IP는 예시(192.0.2.x 문서용 주소)')
    fig.savefig(path, dpi=100)
    plt.close(fig)


# ------------------------------------------------------------------ 2. 추가
def dim_background(ax, title):
    window_frame(ax, title)
    settings_nav(ax)
    t(ax, 248, 72, 'Pinky Settings', 21, TEXT, 'bold')
    global ZOFF
    ax.add_patch(Rectangle((0, 34), W, H - 34, fc='#1b2a40', ec='none', alpha=0.45, zorder=50))
    ZOFF = 100


def screen_add(path):
    fig, ax = canvas()
    dim_background(ax, 'RO:PORT · 통합관제 · 설정')
    dx, dy, dw, dh = 430, 90, 740, 830
    box(ax, dx, dy, dw, dh, fc=PANEL, ec=BORDER, r=10, z=1)
    z = 11
    t(ax, dx + 28, dy + 36, 'Pinky 추가', 17, TEXT, 'bold', z=z)
    t(ax, dx + 28, dy + 64, '같은 네트워크의 Pinky를 IP로 등록함. 비밀번호는 받지 않고 SSH 키로만 접속함.',
      10.5, MUTED, z=z)
    fx = dx + 28
    rows = [('IP 주소 *', '192.0.2.13', False), ('SSH 사용자', 'pinky', False),
            ('SSH 키', '~/.ssh/pinky_ed25519', True), ('표시 이름', 'Pinky 3', False),
            ('ROS_DOMAIN_ID', '14', False)]
    for i, (k, v, dd) in enumerate(rows):
        y = dy + 92 + i * 46
        t(ax, fx, y + 16, k, 11, MUTED, z=z)
        box(ax, fx + 150, y, 520, 32, fc=PANEL, ec='#b8c4d4', r=4, z=z)
        t(ax, fx + 162, y + 16, v, 11, z=z + 1)
        if dd:
            arrow(ax, fx + 654, y + 16, z=z + 1)
    y = dy + 92 + 5 * 46
    t(ax, fx, y + 16, '역할 *', 11, MUTED, z=z)
    for i, (name, on) in enumerate((('물류', False), ('스마트카트', True))):
        cx = fx + 162 + i * 150
        ax.add_patch(Circle((cx, y + 16), 8, fc=NAVY if on else PANEL, ec=NAVY, lw=1.5, zorder=ZOFF + z))
        t(ax, cx + 16, y + 16, name, 11.5, z=z)
    note(ax, fx + 470, y + 16, 4, '등록 후 상세에서도 변경 가능')

    button(ax, fx, dy + 400, 160, '연결 확인', 'teal', z=z)
    t(ax, fx + 176, dy + 417, '통과해야 [등록]이 켜짐', 10.5, MUTED, z=z)
    box(ax, fx, dy + 446, 684, 250, fc='#f7f9fc', ec=BORDER, r=6, z=z)
    checks = [(OK, 'SSH 접속 (키 인증, 호스트 키 확인)', '통과'),
              (OK, '기체 이름 (hostname)', 'pinky_764e'),
              (OK, 'Ubuntu 24.04 · ROS 2 Jazzy', '확인'),
              (OK, '저장공간', '12.1 GB 남음'),
              (OK, 'IR 기준값 프로필 (robots.yaml)', 'pinky_764e 있음'),
              (WARN, '설치된 RO:PORT 버전', '없음 → 등록 후 설치 필요'),
              (ERR, '(예) 같은 IP·기체가 이미 등록됨', '등록 불가')]
    for i, (c, k, v) in enumerate(checks):
        yy = dy + 470 + i * 32
        dot(ax, fx + 20, yy, c)
        t(ax, fx + 36, yy, k, 10.5, z=z + 1)
        t(ax, fx + 660, yy, v, 10.5, c, ha='right', z=z + 1)
    t(ax, fx, dy + 720, '연결 확인은 읽기만 함(hostname·OS·저장공간·버전). bringup·노드 실행·모터 구동 없음.',
      10.5, MUTED, z=z)
    t(ax, fx, dy + 744, 'IP 형식·범위 검사, 같은 IP·기체 중복 등록 막음.', 10.5, MUTED, z=z)
    button(ax, dx + dw - 300, dy + dh - 60, 130, '취소', z=z)
    button(ax, dx + dw - 158, dy + dh - 60, 130, '등록', 'primary', z=z)
    note(ax, dx + dw - 158, dy + dh - 64, 1)
    fig.savefig(path, dpi=100)
    plt.close(fig)


# ------------------------------------------------------------------ 3. 업데이트·릴리스
def screen_update(path):
    fig, ax = canvas()
    dim_background(ax, 'RO:PORT · 통합관제 · 설정')
    dx, dy, dw, dh = 300, 70, 1000, 880
    box(ax, dx, dy, dw, dh, fc=PANEL, ec=BORDER, r=10, z=1)
    z = 11
    t(ax, dx + 28, dy + 36, 'Pinky 소스 설치 (업데이트 · 릴리스)', 17, TEXT, 'bold', z=z)
    t(ax, dx + 28, dy + 64, '관제 PC 저장소에서 고른 버전을 Pinky로 보내고 Pinky에서 빌드함. '
      'Pinky는 인터넷·GitHub 접속이 필요 없음.', 10.5, MUTED, z=z)
    note(ax, dx + dw - 40, dy + 36, 2)

    # 버전
    t(ax, dx + 28, dy + 104, '1. 버전', 12.5, NAVY, 'bold', z=z)
    for i, (name, on) in enumerate((('릴리스 (태그, 시연용)', True), ('개발 (브랜치 최신, 시험용)', False))):
        cx = dx + 48 + i * 260
        ax.add_patch(Circle((cx, dy + 136), 8, fc=NAVY if on else PANEL, ec=NAVY, lw=1.5, zorder=ZOFF + z))
        t(ax, cx + 16, dy + 136, name, 11, z=z)
    table(ax, dx + 28, dy + 158, [150, 150, 140, 500],
          ['버전', '커밋', '날짜', '내용'],
          [['● v0.3.0', 'd960c64', '11-05', 'IR 노드 주행·경로 (권장)'],
           ['○ v0.2.1', '9f8e7d6', '11-02', '라인 추종 v0'],
           ['○ v0.2.0', '4b116ff', '10-31', 'IR 측정 화면']], row_h=34, selected=0)

    # 대상
    t(ax, dx + 28, dy + 318, '2. 대상', 12.5, NAVY, 'bold', z=z)
    table(ax, dx + 28, dy + 340, [40, 120, 120, 160, 500],
          ['', '이름', '역할', '현재 → 설치', '설치할 패키지 (역할에 따라 자동)'],
          [['[v]', 'Pinky 1', '물류', 'v0.2.1 → v0.3.0', 'pinky_pro(원본) · roport_pinky_line · roport_pinky'],
           ['[v]', 'Pinky 2', '물류', 'v0.2.1 → v0.3.0', 'pinky_pro(원본) · roport_pinky_line · roport_pinky'],
           ['[ ]', 'Pinky 3', '스마트카트', '오프라인', '+ roport_smartcart (오프라인이라 선택 불가)']],
          row_h=34, colors={(2, 1): OFF, (2, 2): OFF, (2, 3): OFF, (2, 4): OFF})

    # 사전 확인
    t(ax, dx + 28, dy + 500, '3. 사전 확인 (하나라도 안 되면 그 Pinky는 설치하지 않음)', 12.5, NAVY, 'bold', z=z)
    checks = [(OK, '주행·임무 중 아님 (라인 추종·Nav2 꺼짐)'), (OK, '배터리 30 % 이상'),
              (WARN, '저장공간 1 GB 이상 — Pinky 2: 3.3 GB (경고만)'), (OK, '관제 PC 저장소에 v0.3.0 있음')]
    for i, (c, s) in enumerate(checks):
        cx = dx + 48 + (i % 2) * 460
        cy = dy + 532 + (i // 2) * 30
        dot(ax, cx, cy, c)
        t(ax, cx + 14, cy, s, 10.5, z=z)

    # 진행
    t(ax, dx + 28, dy + 608, '4. 진행', 12.5, NAVY, 'bold', z=z)
    for r, (name, states) in enumerate((('Pinky 1', ['done', 'done', 'done', 'done', 'done']),
                                        ('Pinky 2', ['done', 'done', 'run', 'wait', 'wait']))):
        yy = dy + 634 + r * 44
        t(ax, dx + 28, yy + 15, name, 11, z=z)
        for i, (step, st) in enumerate(zip(('연결 확인', '소스 전송', '빌드', '새 버전으로 전환', '완료'),
                                           states)):
            sx = dx + 120 + i * 170
            fc = {'done': OK, 'run': TEAL, 'wait': '#e6ebf3'}[st]
            box(ax, sx, yy, 160, 30, fc=fc, ec=fc, r=15, z=z)
            t(ax, sx + 80, yy + 15, ('✓ ' if st == 'done' else '▶ ' if st == 'run' else '') + step,
              10.5, 'white' if st != 'wait' else MUTED, 'bold', ha='center', z=z + 1)
    t(ax, dx + 28, dy + 742, '빌드가 성공했을 때만 새 버전으로 전환하고, 실패하면 이전 버전을 그대로 씀(되돌리기 가능). '
      '설치는 파일 복사·빌드만 하고 bringup·노드 재시작은 하지 않음.', 10.5, MUTED, z=z)
    t(ax, dx + 28, dy + 766, '2 GB 기체는 빌드 동시 작업 1개(--parallel-workers 1)로 메모리 부족을 막음.',
      10.5, MUTED, z=z)
    button(ax, dx + dw - 300, dy + dh - 60, 130, '취소', z=z)
    button(ax, dx + dw - 158, dy + dh - 60, 130, '설치 시작', 'primary', z=z)
    fig.savefig(path, dpi=100)
    plt.close(fig)


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    screen_main(os.path.join(out, 'pinky_settings_1_main.png'))
    screen_add(os.path.join(out, 'pinky_settings_2_add.png'))
    screen_update(os.path.join(out, 'pinky_settings_3_update.png'))
    print('ok')
