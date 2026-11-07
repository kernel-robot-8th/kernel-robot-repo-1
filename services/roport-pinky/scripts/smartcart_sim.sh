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
  start|headless|sensors)
    SMARTCART_GUI=true
    if [[ "${1:-start}" == headless ]]; then SMARTCART_GUI=false; fi
    SMARTCART_PYTHON="${SMARTCART_PYTHON:-$(dirname "$(dirname "$SMARTCART_WS")")/../.venvs/smartcart-console/bin/python}"
    if [[ ! -x "$SMARTCART_PYTHON" ]]; then SMARTCART_PYTHON="$(command -v python3)"; fi
    if ! "$SMARTCART_PYTHON" -c 'import flask, rclpy' >/dev/null 2>&1; then
      echo 'Flask와 ROS를 함께 읽는 Python이 필요합니다. README의 가상환경 준비 절차를 확인하세요.' >&2
      exit 1
    fi
    if [[ "${1:-start}" == sensors ]]; then
      exec "$SMARTCART_PYTHON" "$(command -v ros2)" launch roport_smartcart sensor_demo.launch.py height:="${2:-0.05}"
    fi
    exec "$SMARTCART_PYTHON" "$(command -v ros2)" launch roport_smartcart simulation.launch.py \
      gui:="$SMARTCART_GUI" rviz:=false
    ;;
  control)
    shift
    exec ros2 run roport_smartcart sim_control "$@"
    ;;
  delivery1|delivery2|delivery3)
    exec ros2 run roport_smartcart sim_return_h1 --gate "gate${1#delivery}" --wait-for-control
    ;;
  h1|h2)
    exec ros2 run roport_smartcart sim_return_h1 --bay "${1^^}"
    ;;
  gate1|gate2|gate3)
    exec ros2 run roport_smartcart sim_return_h1 --gate "$1"
    ;;
  home|sensor-goal)
    SMARTCART_SHARE="$(ros2 pkg prefix --share roport_smartcart)"
    SMARTCART_MAP="$(ros2 pkg prefix --share roport_gz_world)/maps/airport_map.yaml"
    SMARTCART_RETURN=()
    SMARTCART_TARGET="$1"
    if [[ "$1" == sensor-goal ]]; then
      SMARTCART_TARGET=sensor_goal
    elif [[ "$1" != home ]]; then
      SMARTCART_RETURN=(--return-to home)
    fi
    exec ros2 run roport_smartcart navigate --mode sim --domain-id 198 \
      --places "$SMARTCART_SHARE/config/places_sim.yaml" --map "$SMARTCART_MAP" \
      --target "$SMARTCART_TARGET" "${SMARTCART_RETURN[@]}" --startup-wait 30 --timeout 120
    ;;
  *)
    echo '사용법: smartcart_sim.sh [start|headless|sensors [높이_m]|sensor-goal|gate1|gate2|gate3|delivery1|delivery2|delivery3|control [--occupied H1 H2]|h1|h2|home]' >&2
    exit 2
    ;;
esac
