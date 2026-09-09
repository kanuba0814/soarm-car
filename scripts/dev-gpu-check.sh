#!/usr/bin/env bash
set -euo pipefail
for forbidden_variable in NVIDIA_VISIBLE_DEVICES NVIDIA_DRIVER_CAPABILITIES CUDA_VISIBLE_DEVICES __NV_PRIME_RENDER_OFFLOAD; do
    if [[ -n "${!forbidden_variable:-}" ]]; then
        echo "Intel runtime refuses forced NVIDIA environment: $forbidden_variable" >&2
        exit 2
    fi
done
source "$(dirname "$0")/dev-env.sh"
output="$(docker compose run --rm --no-deps ros2-dev glxinfo -B)"
printf '%s\n' "$output"
grep -q 'direct rendering: Yes' <<<"$output"
if grep -Eqi 'renderer.*(llvmpipe|softpipe)|Accelerated: no|NVIDIA|CUDA' <<<"$output"; then
    echo 'Container is not using the required Intel hardware renderer.' >&2
    exit 1
fi
grep -Eqi 'Vendor: Intel|OpenGL vendor string: Intel|Mesa Intel\(R\) Graphics \(ARL\)' <<<"$output"
echo 'Container Intel OpenGL hardware acceleration OK.'
