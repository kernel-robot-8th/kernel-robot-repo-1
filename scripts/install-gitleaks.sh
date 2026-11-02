#!/bin/sh
set -eu

# This project's development hosts and CI use Linux x86_64.
if [ "$(uname -s)-$(uname -m)" != Linux-x86_64 ]; then
  echo 'Install Gitleaks 8.30.1 for your platform from https://github.com/gitleaks/gitleaks/releases/tag/v8.30.1' >&2
  exit 1
fi
task_install_dir=${1:-"$HOME/.local/bin"}
task_download_dir=$(mktemp -d)
trap 'rm -rf "$task_download_dir"' EXIT HUP INT TERM
curl --fail --silent --show-error --location \
  https://github.com/gitleaks/gitleaks/releases/download/v8.30.1/gitleaks_8.30.1_linux_x64.tar.gz \
  --output "$task_download_dir/gitleaks.tar.gz"
printf '%s  %s\n' 551f6fc83ea457d62a0d98237cbad105af8d557003051f41f3e7ca7b3f2470eb \
  "$task_download_dir/gitleaks.tar.gz" | sha256sum --check --status
tar -xzf "$task_download_dir/gitleaks.tar.gz" -C "$task_download_dir" gitleaks
mkdir -p "$task_install_dir"
install -m 755 "$task_download_dir/gitleaks" "$task_install_dir/gitleaks"
"$task_install_dir/gitleaks" version
