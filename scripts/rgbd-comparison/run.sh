#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
NAME=${1:?usage: bash scripts/rgbd-comparison/run.sh NEW_OUTPUT_NAME}
[[ "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || exit 2
OUT="$ROOT/artifacts/perception-dev/dabai-dc1/$NAME"
[[ ! -e "$OUT" ]] || { echo "Refusing to overwrite $OUT" >&2;exit 2; }
INPUT="$ROOT/artifacts/perception-dev/dabai-dc1/orbslam3-validation"
[[ -f "$INPUT/office-20260913-212016/associations.txt" && -f "$INPUT/office-20260913-212121/associations.txt" ]] || { echo 'Extracted inputs from ORB validation are required.' >&2;exit 2; }
IMAGE=${DC1_COMPARISON_IMAGE:-sha256:02ecfd52f40232a19f12449bce086b10969a88c7f068c6ab7feb01713529c8a1}
mkdir -p "$OUT"
docker run --rm --network none --user "$(id -u):$(id -g)" -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
 -v "$ROOT/scripts/rgbd-comparison:/comparison:ro" -v "$INPUT:/inputs:ro" -v "$OUT:/results" \
 --entrypoint bash "$IMAGE" -lc '
set -e
source /opt/ros/humble/setup.bash
bash /comparison/build-rtab.sh > /results/rtab-build.log 2>&1
for bag in office-20260913-212016 office-20260913-212121; do
 for method in rgbd icp hybrid; do
  python3 /comparison/dense.py /inputs/$bag /results/$bag/$method --method $method
 done
 python3 /comparison/dense.py /inputs/$bag /results/$bag/hybrid-stride2 --method hybrid --stride 2
 mkdir /results/$bag/rtabmap
 /results/rtab-runner /inputs/$bag /results/$bag/rtabmap > /results/$bag/rtabmap/run.log 2>&1
 mkdir /results/$bag/rtabmap-reset3s
 /results/rtab-runner /inputs/$bag /results/$bag/rtabmap-reset3s reset3s > /results/$bag/rtabmap-reset3s/run.log 2>&1
done'
MPLCONFIGDIR=/tmp/dc1-mpl python3 "$ROOT/scripts/rgbd-comparison/analyze.py" "$OUT" "$INPUT"
