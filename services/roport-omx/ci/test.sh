#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
uv run --locked python -m unittest discover -s ci -p 'test_*.py' -v
