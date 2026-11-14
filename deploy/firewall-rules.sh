#!/bin/sh
# 허용 대역(ALLOWED_CIDRS) 밖의 접속을 막는 iptables 규칙을 "출력만" 한다. 실행하지 않는다.
# Docker가 게시한 포트는 ufw를 우회하므로 DOCKER-USER 체인을 쓴다. 이 체인에서는 DNAT가 끝나 목적지가 컨테이너 IP이므로
# 게시 주소·포트는 conntrack의 원래 목적지(--ctorigdst/--ctorigdstport)로 맞추고, 응답 패킷이 막히지 않게 요청 방향(--ctdir ORIGINAL)만 대상으로 한다.
# 허용 대역 밖 호스트는 이미 맺은 연결의 요청 패킷도 막힌다(의도한 동작). 응답 방향 패킷은 영향이 없다.
# 사용: set -a; . deploy/.env; set +a; sh deploy/firewall-rules.sh   (출력 검토 후 직접 sudo로 실행)
# 복구: 출력의 -I를 -D로 바꿔 같은 순서로 실행하거나 재부팅(규칙은 영구 저장하지 않음). 미검증: 실제 호스트에서 확인 필요.
set -eu
: "${LAN_IP:?LAN_IP 필요}" "${ALLOWED_CIDRS:?ALLOWED_CIDRS 필요}"
[ "$LAN_IP" != REPLACE_ME ] && [ "$ALLOWED_CIDRS" != REPLACE_ME ] || { echo "REPLACE_ME를 실제 값으로 바꾸세요" >&2; exit 1; }
for port in "${CONTROL_PORT:-8000}" "${AI_PORT:-8001}" "${SERVICE_PORT:-8080}" "${WEB_SERVICE_PORT:-8090}" "${WEB_ADMIN_PORT:-8091}" "${WEB_PAD_PORT:-8092}"; do
  # DROP을 먼저 넣고, 그 위에 허용 대역 ACCEPT를 넣는다(-I는 맨 앞에 삽입).
  echo "sudo iptables -I DOCKER-USER -p tcp -m conntrack --ctdir ORIGINAL --ctorigdst $LAN_IP --ctorigdstport $port -j DROP"
  echo "$ALLOWED_CIDRS" | tr ',' '\n' | while read -r cidr; do
    echo "sudo iptables -I DOCKER-USER -s $cidr -p tcp -m conntrack --ctdir ORIGINAL --ctorigdst $LAN_IP --ctorigdstport $port -j ACCEPT"
  done
done
