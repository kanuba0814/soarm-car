#!/usr/bin/env bash
# C3: deterministic depth-source ablation.
# Phase 1: held-out block fill error (DC1 bags + fr1_desk).
# Phase 2: downstream joint-frontend runs, development set (DC1 + fr1_desk/room).
# Phase 3: frozen test on fr3_long_office_household, run once per arm.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
NAME=${1:?usage: bash scripts/data-acceptance/run-c3.sh NEW_OUTPUT_NAME}
[[ "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || exit 2
OUT="$ROOT/artifacts/perception-dev/dabai-dc1/$NAME"
[[ ! -e "$OUT" ]] || { echo "Refusing to overwrite $OUT" >&2; exit 2; }
INPUT="$ROOT/artifacts/perception-dev/dabai-dc1/orbslam3-validation"
DATASETS="$ROOT/datasets/tum-rgbd"
ALGO_IMAGE=${DC1_COMPARISON_IMAGE:-sha256:02ecfd52f40232a19f12449bce086b10969a88c7f068c6ab7feb01713529c8a1}
mkdir -p "$OUT"
CALIB1='517.306408,516.469215,318.643040,255.313989'
CALIB3='535.4,539.2,320.1,247.6'

docker run --rm --network none --user "$(id -u):$(id -g)" -e OMP_NUM_THREADS=1 -e MPLCONFIGDIR=/tmp/mpl \
 -v "$ROOT/scripts/data-acceptance:/da:ro" -v "$INPUT:/inputs:ro" -v "$DATASETS:/datasets" -v "$OUT:/results" \
 --entrypoint bash "$ALGO_IMAGE" -lc '
set -e
ARMS="none median5 nearestfill edgegate"

echo "=== C3 phase 1: held-out fill error ==="
for bag in office-20260913-212016 office-20260913-212121; do
  python3 /da/c3_heldout.py /inputs/$bag /results/heldout-$bag --frame-step 4
done
python3 /da/c3_heldout.py /datasets/rgbd_dataset_freiburg1_desk /results/heldout-freiburg1_desk \
  --frame-step 4 --depth-scale 0.0002

dense_run () {  # seqdir out calib maxd arm
  python3 /da/dense_gen.py "$1" "$2" --width 320 --rows 480 --row-offset 0 \
    --calib "$3" --depth-scale 0.0002 --max-depth "$4" --depth-preproc "$5"
}
dc1_run () {  # bag arm
  python3 /da/dense_gen.py /inputs/$1 /results/downstream-$1/$2 --width 320 --depth-preproc $2
}

echo "=== C3 phase 2: downstream development set ==="
for arm in $ARMS; do
  for bag in office-20260913-212016 office-20260913-212121; do
    dc1_run $bag $arm
  done
  dense_run /datasets/rgbd_dataset_freiburg1_desk /results/downstream-freiburg1_desk/$arm '"$CALIB1"' 5.0 $arm
  dense_run /datasets/rgbd_dataset_freiburg1_room  /results/downstream-freiburg1_room/$arm  '"$CALIB1"' 5.0 $arm
done
for seq in freiburg1_desk freiburg1_room; do
  for arm in $ARMS; do
    python3 /da/evaluate.py dense /results/downstream-$seq/$arm \
      /datasets/rgbd_dataset_$seq/groundtruth.txt /results/downstream-$seq/$arm-eval
  done
done

echo "=== C3 phase 3: frozen test fr3 long ==="
for arm in $ARMS; do
  dense_run /datasets/rgbd_dataset_freiburg3_long_office_household /results/frozen-freiburg3_long/$arm '"$CALIB3"' 8.0 $arm
  python3 /da/evaluate.py dense /results/frozen-freiburg3_long/$arm \
    /datasets/rgbd_dataset_freiburg3_long_office_household/groundtruth.txt /results/frozen-freiburg3_long/$arm-eval
done
echo ALL_DONE'
