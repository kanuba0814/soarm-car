#!/usr/bin/env bash
set -euo pipefail

docker stop soarm-astra-camera >/dev/null 2>&1 || true
echo "Astra S 已停止。"
