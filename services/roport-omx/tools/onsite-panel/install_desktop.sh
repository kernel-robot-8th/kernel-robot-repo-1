#!/usr/bin/env bash
# 바탕화면·앱 메뉴에 "OMX On-site Control Panel" 아이콘을 만든다(이 폴더의 open.sh를 실행). 한 번만 실행하면 된다.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
chmod +x "$DIR/open.sh" "$DIR/run.sh"
ENTRY="[Desktop Entry]
Type=Application
Name=OMX On-site Control Panel
Comment=Host PC에서 관제 통신 기반 미션 시험·운영 (127.0.0.1:8870)
Exec=$DIR/open.sh
Icon=$DIR/icon.svg
Terminal=false
Categories=Utility;"
mkdir -p "$HOME/.local/share/applications"
printf '%s\n' "$ENTRY" > "$HOME/.local/share/applications/omx-onsite-panel.desktop"
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
if [ -d "$DESK" ]; then
  printf '%s\n' "$ENTRY" > "$DESK/omx-onsite-panel.desktop"
  chmod +x "$DESK/omx-onsite-panel.desktop"
  gio set "$DESK/omx-onsite-panel.desktop" metadata::trusted true 2>/dev/null || true
fi
echo "아이콘을 만들었습니다: OMX On-site Control Panel → $DIR/open.sh"
