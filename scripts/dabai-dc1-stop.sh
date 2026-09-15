#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/dabai-dc1-common.sh"
if docker container inspect "$DC1_CONTAINER" >/dev/null 2>&1; then
    [[ $(docker inspect -f '{{index .Config.Labels "soarm.dc1.workspace"}}' "$DC1_CONTAINER") == "$ROOT_DIR" ]] || exit 2
    docker stop -t 15 "$DC1_CONTAINER" >/dev/null
    docker rm "$DC1_CONTAINER" >/dev/null
fi
echo 'DC1 驱动已停止。'
