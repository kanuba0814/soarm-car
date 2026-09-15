#!/usr/bin/env bash
# A1 (row-map scan + odometry confirmation) and A3 (depth statistics) on both DC1 bags.
# Usage: bash scripts/data-acceptance/run-a1a3.sh NEW_OUTPUT_NAME
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
NAME=${1:?usage: bash scripts/data-acceptance/run-a1a3.sh NEW_OUTPUT_NAME}
[[ "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || exit 2
OUT="$ROOT/artifacts/perception-dev/dabai-dc1/$NAME"
[[ ! -e "$OUT" ]] || { echo "Refusing to overwrite $OUT" >&2; exit 2; }
INPUT="$ROOT/artifacts/perception-dev/dabai-dc1/orbslam3-validation"
FROZEN="$ROOT/artifacts/perception-dev/dabai-dc1/rgbd-comparison"
IMAGE=${DC1_COMPARISON_IMAGE:-sha256:02ecfd52f40232a19f12449bce086b10969a88c7f068c6ab7feb01713529c8a1}
mkdir -p "$OUT"
docker run --rm --network none --user "$(id -u):$(id -g)" -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 -e MPLCONFIGDIR=/tmp/mpl \
 -v "$ROOT/scripts/data-acceptance:/da:ro" -v "$INPUT:/inputs:ro" -v "$FROZEN:/frozen:ro" -v "$OUT:/results" \
 --entrypoint bash "$IMAGE" -lc '
set -e
for bag in office-20260913-212016 office-20260913-212121; do
  echo "=== A1 scan $bag ==="
  python3 /da/a1_geometry.py /inputs/$bag /results/$bag/a1 --frame-step 2
  winner=$(python3 -c "import json;print(json.load(open(\"/results/$bag/a1/a1_scan.json\"))[\"winner\"])")
  wb=${winner##*:}; ws=${winner%%:*}
  echo "A1 winner: $winner"
  # Odometry confirmation: offset 0 (current assumption), 40 (centre-crop control),
  # and the scan winner when it differs. Full sequences, same frontend parameters.
  for off in 0 40 $([ "$wb" != 0 ] && [ "$wb" != 40 ] && echo "$wb"); do
    echo "=== A1 confirm $bag offset $off ==="
    python3 /da/dense_gen.py /inputs/$bag /results/$bag/a1-confirm-offset$off --width 320 --row-offset $off
  done
  echo "=== A3 $bag ==="
  python3 /da/a3_depth.py /inputs/$bag /results/$bag/a3 --frame-step 2 \
    --frozen-hybrid-frames /frozen/$bag/hybrid/frames.csv
done
echo ALL_DONE'
