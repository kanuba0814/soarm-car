#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/dev-env.sh"
output="$(docker compose run --rm --no-deps ros2-dev glxinfo -B)"
printf '%s\n' "$output"
grep -q 'direct rendering: Yes' <<<"$output"
if grep -Eqi 'renderer.*(llvmpipe|softpipe)|Accelerated: no' <<<"$output"; then
    echo 'Container is using software rendering, not the host GPU.' >&2
    exit 1
fi
echo 'Container OpenGL hardware acceleration OK.'
