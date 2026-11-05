#!/usr/bin/env bash
# PC simulation only. Never inherits the physical robot's ROS discovery domain.
set -eo pipefail
SMARTCART_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$SMARTCART_WS/install/setup.bash" ]]; then
  echo '먼저 ROS 환경에서 colcon build --packages-up-to roport_smartcart 를 실행하세요.' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$SMARTCART_WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID=198
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# A previous hardware shell may have configured explicit remote discovery.
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER
cd "$SMARTCART_WS"

case "${1:-start}" in
  start|headless)
    SMARTCART_GUI=true
    if [[ "${1:-start}" == headless ]]; then SMARTCART_GUI=false; fi
    SMARTCART_PYTHON="${SMARTCART_PYTHON:-$(dirname "$(dirname "$SMARTCART_WS")")/../.venvs/smartcart-console/bin/python}"
    if [[ ! -x "$SMARTCART_PYTHON" ]]; then SMARTCART_PYTHON="$(command -v python3)"; fi
    if ! "$SMARTCART_PYTHON" -c 'import flask, rclpy' >/dev/null 2>&1; then
      echo 'Flask와 ROS를 함께 읽는 Python이 필요합니다. README의 가상환경 준비 절차를 확인하세요.' >&2
      exit 1
    fi
    exec "$SMARTCART_PYTHON" "$(command -v ros2)" launch roport_smartcart simulation.launch.py \
      gui:="$SMARTCART_GUI" rviz:=false
    ;;
  gate1|gate2|gate3|home)
    SMARTCART_SHARE="$(ros2 pkg prefix --share roport_smartcart)"
    SMARTCART_MAP="$(ros2 pkg prefix --share roport_gz_world)/maps/airport_map.yaml"
    SMARTCART_RETURN=()
    if [[ "$1" != home ]]; then SMARTCART_RETURN=(--return-to home); fi
    exec ros2 run roport_smartcart navigate --mode sim --domain-id 198 \
      --places "$SMARTCART_SHARE/config/places_sim.yaml" --map "$SMARTCART_MAP" \
      --target "$1" "${SMARTCART_RETURN[@]}" --startup-wait 30 --timeout 120
    ;;
  *)
    echo '사용법: smartcart_sim.sh [start|headless|gate1|gate2|gate3|home]' >&2
    exit 2
    ;;
esac
