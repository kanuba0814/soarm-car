#!/usr/bin/env bash
# Four additional runs plus the original make five, without recompilation.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
NAME=${1:?usage: bash scripts/rgbd-backend/repeat-long.sh EXISTING_OUTPUT_NAME}
[[ "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || exit 2
OUT="$ROOT/artifacts/perception-dev/dabai-dc1/$NAME"
[[ -f "$OUT/summary.json" && -x "$OUT/runner" && -x "$OUT/export" ]] || exit 2
[[ ! -e "$OUT/reliability" ]] || { echo 'Refusing to overwrite reliability runs' >&2; exit 2; }
POLICY=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("matching_policy", "baseline"))' "$OUT/manifest.json")
[[ "$POLICY" == baseline || "$POLICY" == exact ]] || exit 2
docker run --rm --network none --read-only --tmpfs /tmp:rw,size=512m \
 --user "$(id -u):$(id -g)" -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
 -e MATCH_POLICY="$POLICY" \
 -e LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib \
 -v "$ROOT/datasets/tum-rgbd/rgbd_dataset_freiburg3_long_office_household:/sequence:ro" \
 -v "$OUT:/results:rw" --entrypoint bash \
 sha256:02ecfd52f40232a19f12449bce086b10969a88c7f068c6ab7feb01713529c8a1 -lc '
set -e
EXTRA=()
if [[ "$MATCH_POLICY" == exact ]]; then
 EXTRA=(Vis/CorNNType=3 Vis/CorGuessWinSize=0 Kp/NNStrategy=3)
fi
mkdir /results/reliability
for n in 2 3 4 5; do
 dir=/results/reliability/long-refine-$n
 mkdir "$dir"
 /results/runner /sequence /results/freiburg3_long_office_household/keyframes.txt "$dir" \
  535.4 539.2 320.1 247.6 480 0.0002 8 true "${EXTRA[@]}" > "$dir/run.log" 2>&1
 /results/export "$dir" > "$dir/export.log" 2>&1
 echo "DONE repeat $n"
done'
python3 "$ROOT/scripts/rgbd-backend/stability.py" \
 --groundtruth "$ROOT/datasets/tum-rgbd/rgbd_dataset_freiburg3_long_office_household/groundtruth.txt" \
 --out "$OUT/reliability-summary.json" "$OUT/freiburg3_long_office_household/refine" \
 "$OUT/reliability/long-refine-2" "$OUT/reliability/long-refine-3" \
 "$OUT/reliability/long-refine-4" "$OUT/reliability/long-refine-5"
