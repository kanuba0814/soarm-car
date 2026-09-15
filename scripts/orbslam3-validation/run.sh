#!/usr/bin/env bash
# Explicit new output directory; never overwrites a previous experiment or changes the camera.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
NAME=${1:?usage: bash scripts/orbslam3-validation/run.sh NEW_EXPERIMENT_NAME}
[[ "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || exit 2
OUT="$ROOT/artifacts/perception-dev/dabai-dc1/$NAME"
[[ ! -e "$OUT" ]] || { echo "Output already exists: $OUT" >&2; exit 2; }
IMAGE=${DC1_SLAM_IMAGE:-sha256:a9b58f3bd659a89ae6059a1946e5c68d1e14a6f2786fd634c900005e9402592c}
mkdir -p "$OUT"
docker run --rm --network none --user "$(id -u):$(id -g)" \
 -v "$ROOT/scripts/orbslam3-validation:/validation:ro" \
 -v "$ROOT/recordings:/bags:ro" -v "$OUT:/results" \
 --entrypoint bash "$IMAGE" -lc '
set -e
source /opt/ros/humble/setup.bash
bash /validation/build.sh > /results/build.log 2>&1
for bag in office-20260913-212016 office-20260913-212121; do
 python3 /validation/extract.py /bags/$bag /results/$bag
 for run in baseline features2000; do
  config=dc1.yaml; if [ "$run" = features2000 ]; then config=dc1-2000.yaml; fi
  mkdir /results/$bag/$run
  cd /results/$bag/$run
  /results/runner /workspaces/soarm-car/vendor/orb_slam3/ORB_SLAM3/Vocabulary/ORBvoc.txt /validation/$config /results/$bag /results/$bag/$run 0 > run.log 2>&1
 done
done'
MPLCONFIGDIR=/tmp/dc1-validation-mpl python3 "$ROOT/scripts/orbslam3-validation/analyze.py" "$OUT"
python3 "$ROOT/scripts/orbslam3-validation/check.py" "$OUT/summary.json"
