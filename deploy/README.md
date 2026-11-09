# 팀 LAN 전용 배포 (Docker Compose)

관제·AI·서비스 서버와 웹(고객용·관리자용)을 한 PC에서 컨테이너로 띄우고, 팀 Wi-Fi(TEAM1, TEAM1_5g)에서만 접속하게 하는 준비물입니다. 최대 3주 운영을 가정하며 클라우드·유료 서비스는 쓰지 않습니다.

**서버에 인증이 아직 없습니다.** 인터넷에 공개하지 않습니다. 보호 수단은 LAN IP에만 포트를 게시하는 것과 허용 대역 방화벽입니다.

## 사용

호스트에 Docker(Compose v2)가 필요합니다.

```bash
cp deploy/.env.example deploy/.env      # LAN_IP, ALLOWED_CIDRS를 이 PC의 실제 값으로 수정
docker compose -f deploy/compose.yaml --env-file deploy/.env up -d --build
docker compose -f deploy/compose.yaml --env-file deploy/.env ps
```

`LAN_IP`가 없으면 compose가 시작하지 않습니다(0.0.0.0에 열리는 사고 방지). 실제 SSID·IP는 저장소에 올리지 않고 각 PC의 `deploy/.env`에만 둡니다.

| 접속 | 주소(`LAN_IP` 기준) |
| --- | --- |
| 관제 서버·3D 웹 UI | `http://LAN_IP:8000/ui/` |
| AI 서버 | `http://LAN_IP:8001/healthz` |
| 서비스 서버 | `http://LAN_IP:8080/actuator/health` |
| 고객용 웹 / 관리자용 웹 | `http://LAN_IP:8090` / `http://LAN_IP:8091` |

## 접속 대역 제한

게시 IP를 `LAN_IP`로 제한하면 다른 네트워크 인터페이스(VPN 등)에서는 열리지 않습니다. 같은 Wi-Fi의 다른 대역(2.4G/5G)까지 좁히려면 허용 대역 밖을 막는 규칙을 씁니다. 스크립트는 규칙을 **출력만** 하고 실행하지 않습니다. 출력을 검토한 뒤 직접 sudo로 실행하며, 복구 방법은 스크립트 머리말에 있습니다.

```bash
set -a; . deploy/.env; set +a; sh deploy/firewall-rules.sh
```

## 웹을 Vercel에 올릴 때

Vercel 프로젝트의 Firewall 사용자 규칙으로 허용 IP 외 요청을 거절할 수 있습니다. 다만 웹이 이 LAN 서버를 호출하게 되면 https 페이지에서 `http://<사설 IP>` 호출이 브라우저에서 차단되므로(혼합 콘텐츠) LAN 서버에 신뢰되는 HTTPS가 필요합니다. 현재 웹은 서버를 호출하지 않는 정적 뼈대라 지금은 문제가 없습니다.

## 검증 상태

- 확인함(로컬, Docker 없이): 웹 `npm ci && npm run build`로 두 앱의 `dist` 생성, AI 서버 `uv sync --locked --no-dev`와 import, 관제 서버 자산 준비 후 기동해 `/healthz`·`/ui/` 응답. 서비스 서버는 `./gradlew build` 통과(FFT1-159).
- 미확인: Docker 이미지 빌드, compose 기동, 컨테이너 헬스체크, 방화벽 규칙의 실제 효과, 허용 대역 밖 접속 차단. Docker가 설치된 PC에서 확인해야 합니다.
- 서비스 서버 Dockerfile은 Gradle wrapper를 사용하므로 Gradle 전환 PR(FFT1-159)이 먼저 병합되어야 합니다.
- 관제 서버의 운영 DB·MQTT 브로커·실행부는 포함하지 않습니다. 로봇 연결은 이 배포로 시작되지 않습니다.
