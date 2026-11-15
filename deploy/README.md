# 팀 LAN 전용 배포 (Docker Compose)

관제·AI·서비스 서버와 웹(고객용·관리자용)을 한 PC에서 컨테이너로 띄우고, 팀 Wi-Fi에서만 접속하게 하는 준비물입니다. 최대 3주 운영을 가정하며 클라우드·유료 서비스는 쓰지 않습니다.

**서버에 인증이 아직 없습니다.** 인터넷에 공개하지 않습니다. 보호 수단은 LAN IP에만 포트를 게시하는 것과 허용 대역 방화벽입니다.

## 사용

호스트에 Docker(Compose v2)가 필요합니다.

1. `cp deploy/.env.example deploy/.env` 후 `LAN_IP`, `ALLOWED_CIDRS`(2.4GHz·5GHz 두 대역)를 이 PC의 실제 값으로 채웁니다.
2. **방화벽 규칙을 먼저 적용합니다(필수).** 아래 "접속 대역 제한"을 따릅니다. 서버에 인증이 없어 이 단계 없이 기동하지 않습니다.
3. 컨테이너를 기동합니다.

```bash
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
| 카트 패드 웹 (데모 데이터) | `http://LAN_IP:8092` |

## 접속 대역 제한 (필수)

게시 IP를 `LAN_IP`로 제한해도 이 PC로 라우팅되는 다른 호스트(공유기 포트 포워딩, 다른 대역 등)의 접속은 막지 못합니다. 그래서 허용 대역 밖을 막는 규칙을 기동 전에 적용해야 합니다. 공유기에 이 PC로 향하는 포트 포워딩이 없는지도 확인합니다. 스크립트는 규칙을 **출력만** 하고 실행하지 않습니다. 출력을 검토한 뒤 직접 sudo로 실행하며, 복구 방법은 스크립트 머리말에 있습니다.

```bash
set -a; . deploy/.env; set +a; sh deploy/firewall-rules.sh
```

## 재부팅 후 절차

컨테이너는 자동으로 다시 뜨지 않습니다(`restart: "no"`). 방화벽 규칙을 영구 저장하지 않아 재부팅하면 사라지는데, 자동으로 올라오면 규칙 없이 인증 없는 API가 열리기 때문입니다. 호스트나 Docker를 재시작한 뒤에는 다음 순서를 지킵니다.

1. 방화벽 규칙을 다시 적용합니다(위 "접속 대역 제한").
2. `sudo iptables -S DOCKER-USER`로 규칙이 들어갔는지 확인합니다.
3. `docker compose -f deploy/compose.yaml --env-file deploy/.env up -d`로 기동합니다.

규칙 영속화(부팅 순서 보장)는 이 호스트에서 검증할 수 없어 하지 않았습니다. 자동 재시작이 필요해지면 영속화와 부팅 순서를 따로 설계하고 검증합니다.

## 수용 기준(실제 호스트에서 확인)

- 호스트 재부팅 뒤 `docker compose ps`에 컨테이너가 떠 있지 않다.
- 방화벽 규칙 적용 전에는 서비스가 게시되지 않고, 적용 후 기동하면 허용 대역의 PC에서 5개 포트에 접속된다.
- 허용 대역 밖의 PC(예: 휴대전화 핫스팟)에서는 같은 포트가 응답하지 않는다.
- `up -d` 후 `docker compose ps`에서 네 컨테이너가 healthy다.

## 웹을 Vercel에 올릴 때

Vercel 프로젝트의 Firewall 사용자 규칙으로 허용 IP 외 요청을 거절할 수 있습니다. 다만 웹이 이 LAN 서버를 호출하게 되면 https 페이지에서 `http://<사설 IP>` 호출이 브라우저에서 차단되므로(혼합 콘텐츠) LAN 서버에 신뢰되는 HTTPS가 필요합니다. 현재 웹은 서버를 호출하지 않는 정적 뼈대라 지금은 문제가 없습니다.

## 이미지 갱신

베이스 이미지는 digest가 아닌 minor 태그까지만 고정합니다(3주 운영에서 보안 패치 수신을 우선). 패치를 반영하려면 `docker compose -f deploy/compose.yaml --env-file deploy/.env build --pull` 후 `up -d`로 컨테이너를 다시 만듭니다.

## 검증 상태

- 확인함(로컬, Docker 없이): 웹 `npm ci && npm run build`로 두 앱의 `dist` 생성, AI 서버 `uv sync --locked --no-dev`와 import, 관제 서버 자산 준비 후 기동해 `/healthz`·`/ui/` 응답. 서비스 서버는 `./gradlew build` 통과(FFT1-159).
- 확인함(추가): 서비스 헬스체크 명령을 실제 JAR에 실행해 UP이면 0, 포트가 닫히거나 서버가 내려가면 1.
- 미확인: Docker 이미지 빌드, compose 기동, 컨테이너 헬스체크, 방화벽 규칙의 실제 효과, 허용 대역 밖 접속 차단. Docker가 설치된 PC에서 확인해야 합니다.
- 서비스 서버 Dockerfile은 Gradle wrapper를 사용하므로 Gradle 전환 PR(FFT1-159)이 먼저 병합되어야 합니다.
- 관제 서버의 운영 DB·MQTT 브로커·실행부는 포함하지 않습니다. 로봇 연결은 이 배포로 시작되지 않습니다.
