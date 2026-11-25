#!/usr/bin/env bash
# 바탕화면 아이콘용: 시험 패널 서버가 꺼져 있으면 켜고, 브라우저로 연다.
URL=http://127.0.0.1:8870
DIR="$(cd "$(dirname "$0")" && pwd)"
if ! curl -s -m 2 -o /dev/null "$URL/"; then
  nohup "$DIR/run.sh" >> "$DIR/server.log" 2>&1 &
  for _ in $(seq 1 40); do curl -s -m 1 -o /dev/null "$URL/" && break; sleep 0.25; done
fi
xdg-open "$URL" >/dev/null 2>&1 &
