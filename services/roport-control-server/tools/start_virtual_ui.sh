#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
export UV_PROJECT_ENVIRONMENT="$PWD/.venv"
export PYTHONPATH="$PWD/src"

# This PC keeps its tested Mosquitto runtime outside the repository.
if [[ -z "${MOSQUITTO_BIN:-}" ]] && ! command -v mosquitto >/dev/null; then
    runtime_root="$HOME/.cache/roport-test-runtime/ubuntu-noble/root"
    if [[ ! -x "$runtime_root/usr/sbin/mosquitto" ]]; then
        echo 'Mosquitto를 설치하거나 MOSQUITTO_BIN을 지정하세요.' >&2
        exit 1
    fi
    export MOSQUITTO_BIN="$runtime_root/usr/sbin/mosquitto"
    export LD_LIBRARY_PATH="$runtime_root/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

uv sync --locked
.venv/bin/python tools/prepare_web_assets.py --check
exec .venv/bin/python -u tools/run_virtual_fleet.py "$@"
