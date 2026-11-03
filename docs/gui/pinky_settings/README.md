# Pinky Settings 화면 시안

Jira: [FFT1-124](https://fp-team1.atlassian.net/browse/FFT1-124). 관제 GUI 「설정 > Pinky Settings」 레이아웃 시안입니다. 구현 전에 관제 담당과 확정하기 위한 그림이며 동작하는 화면이 아닙니다.
설명과 검토 항목은 Confluence 「GUI > Pinky GUI」에 있습니다.

| 파일 | 내용 |
|---|---|
| `pinky_settings_1_main.png` | 목록·상세·설치 기록 (메인) |
| `pinky_settings_2_add.png` | Pinky 추가 (IP 등록, 연결 확인) |
| `pinky_settings_3_update.png` | 소스 설치 (업데이트·릴리스) |
| `mockup.py` | 시안을 그리는 스크립트. `python3 mockup.py <출력 폴더>` (matplotlib, fonts-noto-cjk) |

색은 `services/roport-control-server/src/roport_control_server/adapters/ui/window.py` 스타일시트와 맞췄습니다. IP는 문서용 예시 주소(192.0.2.x)입니다.
