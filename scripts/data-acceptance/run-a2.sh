#!/usr/bin/env bash
# A2: timestamp dump (ros2-dev image), then offset re-pairing scan + analysis
# (algorithms image). Usage: bash scripts/data-acceptance/run-a2.sh NEW_OUTPUT_NAME
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
NAME=${1:?usage: bash scripts/data-acceptance/run-a2.sh NEW_OUTPUT_NAME}
[[ "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || exit 2
OUT="$ROOT/artifacts/perception-dev/dabai-dc1/$NAME"
[[ ! -e "$OUT" ]] || { echo "Refusing to overwrite $OUT" >&2; exit 2; }
INPUT="$ROOT/artifacts/perception-dev/dabai-dc1/orbslam3-validation"
FROZEN="$ROOT/artifacts/perception-dev/dabai-dc1/rgbd-comparison"
ROS_IMAGE=${DC1_SLAM_IMAGE:-dabai-dc1-visual-validation-ros2-dev:latest}
ALGO_IMAGE=${DC1_COMPARISON_IMAGE:-sha256:02ecfd52f40232a19f12449bce086b10969a88c7f068c6ab7feb01713529c8a1}
mkdir -p "$OUT"
for bag in office-20260913-212016 office-20260913-212121; do
  echo "=== dump timestamps $bag ==="
  docker run --rm --network none --user "$(id -u):$(id -g)" \
   -v "$ROOT/scripts/data-acceptance:/da:ro" -v "$ROOT/recordings:/bags:ro" -v "$OUT:/results" \
   --entrypoint bash "$ROS_IMAGE" -lc 'source /opt/ros/humble/setup.bash && python3 /da/dump_timestamps.py /bags/'"$bag"' /results/'"$bag"'_timestamps.csv'
  echo "=== scan $bag ==="
  docker run --rm --network none --user "$(id -u):$(id -g)" -e OMP_NUM_THREADS=1 -e MPLCONFIGDIR=/tmp/mpl \
   -v "$ROOT/scripts/data-acceptance:/da:ro" -v "$INPUT:/inputs:ro" -v "$FROZEN:/frozen:ro" -v "$OUT:/results" \
   --entrypoint python3 "$ALGO_IMAGE" /da/a2_timing.py /inputs/$bag /results/${bag}_timestamps.csv \
   /results/$bag --frozen-hybrid-frames /frozen/$bag/hybrid/frames.csv
done
echo ALL_DONE
