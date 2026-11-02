#!/bin/sh
set -eu

if ! command -v gitleaks >/dev/null 2>&1; then
  echo 'Gitleaks is required: sh scripts/install-gitleaks.sh, then add ~/.local/bin to PATH.' >&2
  exit 1
fi
exec gitleaks git --redact=100 --no-banner --ignore-gitleaks-allow "$@"
