#!/usr/bin/env bash
# Intel-only environment wrapper for the existing FAST-LIO/Nav2 house runner.
set -euo pipefail

cd "$(dirname "$0")/.."

for forbidden_variable in NVIDIA_VISIBLE_DEVICES NVIDIA_DRIVER_CAPABILITIES CUDA_VISIBLE_DEVICES __NV_PRIME_RENDER_OFFLOAD; do
    if [[ -n "${!forbidden_variable:-}" ]]; then
        echo "Intel runtime refuses forced NVIDIA environment: $forbidden_variable" >&2
        exit 2
    fi
done

export LIBGL_ALWAYS_SOFTWARE=0
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-2}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-2}"

./scripts/dev-gpu-check.sh
exec ./scripts/dev-fastlio-house-nav2-smoke.sh
