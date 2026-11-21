#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
runtime="$HOME/.cache/roport-test-runtime/ubuntu-noble/root"
if [[ -d "$runtime/usr/lib/postgresql/16/bin" ]]; then
  export PG_BIN="${PG_BIN:-$runtime/usr/lib/postgresql/16/bin}"
  export MOSQUITTO_BIN="${MOSQUITTO_BIN:-$runtime/usr/sbin/mosquitto}"
  export LD_LIBRARY_PATH="$runtime/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export UV_PROJECT_ENVIRONMENT="$PWD/.venv"
export PYTHONPATH="$PWD/src"
uv sync --locked
.venv/bin/python tools/prepare_web_assets.py --check
if [[ ! -d ../roport-web/node_modules ]]; then
  (cd ../roport-web && npm ci)
fi
exec .venv/bin/python -u tools/run_service_fleet.py "$@"
