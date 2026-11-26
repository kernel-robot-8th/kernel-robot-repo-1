#!/usr/bin/env bash
# OMX Host 시험 패널 실행: http://127.0.0.1:8870 (이 PC에서만 열림)
# Host 저장소(~/.config/roport/omx-host.json의 repo)의 가상환경으로 실행해 Host 경로표 규칙을 그대로 씁니다.
set -euo pipefail
REPO=$(python3 -c "import json,os;print(os.path.expanduser(json.load(open(os.path.expanduser('~/.config/roport/omx-host.json')))['repo']))")
cd "$(dirname "$0")"
unset VIRTUAL_ENV
exec uv run --no-sync --project "$REPO" python server.py
