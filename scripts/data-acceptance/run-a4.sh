#!/usr/bin/env bash
# A4: TUM RGB-D ground-truth adaptation, joint-frontend drift measurement and
# ORB-SLAM3 cross-validation of the evaluation toolchain.
# Sequences are downloaded to datasets/tum-rgbd (git-ignored) on the host first.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
NAME=${1:?usage: bash scripts/data-acceptance/run-a4.sh NEW_OUTPUT_NAME}
[[ "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || exit 2
OUT="$ROOT/artifacts/perception-dev/dabai-dc1/$NAME"
[[ ! -e "$OUT" ]] || { echo "Refusing to overwrite $OUT" >&2; exit 2; }
DATASETS="$ROOT/datasets/tum-rgbd"
ROS_IMAGE=${DC1_SLAM_IMAGE:-dabai-dc1-visual-validation-ros2-dev:latest}
ALGO_IMAGE=${DC1_COMPARISON_IMAGE:-sha256:02ecfd52f40232a19f12449bce086b10969a88c7f068c6ab7feb01713529c8a1}
mkdir -p "$OUT"

# --- host: fetch sequences if missing (network only here, per repo convention) ---
for entry in "freiburg1 freiburg1_xyz" "freiburg1 freiburg1_desk" "freiburg1 freiburg1_room" \
             "freiburg3 freiburg3_long_office_household"; do
  set -- $entry; family=$1; seq=$2
  dir="$DATASETS/rgbd_dataset_$seq"
  if [[ ! -d "$dir" ]]; then
    tgz="$DATASETS/rgbd_dataset_$seq.tgz"
    [[ -f "$tgz" ]] || curl -sL --retry 3 -o "$tgz" "https://cvg.cit.tum.de/rgbd/dataset/$family/rgbd_dataset_$seq.tgz"
    tar -xzf "$tgz" -C "$DATASETS" && rm -f "$tgz"
  fi
done

CALIB1='517.306408,516.469215,318.643040,255.313989'
CALIB3='535.4,539.2,320.1,247.6'

# --- phase 1: adapt + joint frontend + dense evaluation (algorithms image) ---
docker run --rm --network none --user "$(id -u):$(id -g)" -e OMP_NUM_THREADS=1 -e MPLCONFIGDIR=/tmp/mpl \
 -v "$ROOT/scripts/data-acceptance:/da:ro" -v "$DATASETS:/datasets" -v "$OUT:/results" \
 --entrypoint bash "$ALGO_IMAGE" -lc '
set -e
run_seq () {
  seq=$1; calib=$2; maxd=$3
  echo "=== adapt $seq ==="
  python3 /da/tum_adapt.py /datasets/rgbd_dataset_$seq
  echo "=== dense $seq ==="
  python3 /da/dense_gen.py /datasets/rgbd_dataset_$seq /results/$seq/dense \
    --width 320 --rows 480 --row-offset 0 --calib "$calib" --depth-scale 0.0002 --max-depth $maxd
  echo "=== evaluate dense $seq ==="
  python3 /da/evaluate.py dense /results/$seq/dense /datasets/rgbd_dataset_$seq/groundtruth.txt /results/$seq/dense-eval
}
run_seq freiburg1_xyz '"$CALIB1"' 5.0
run_seq freiburg1_desk '"$CALIB1"' 5.0
run_seq freiburg1_room '"$CALIB1"' 5.0
run_seq freiburg3_long_office_household '"$CALIB3"' 8.0
echo PHASE1_DONE'

# --- phase 2: ORB-SLAM3 with official TUM configs (ros2-dev image) ---
docker run --rm --network none --user "$(id -u):$(id -g)" \
 -v "$ROOT/scripts/data-acceptance:/da:ro" -v "$DATASETS:/datasets:ro" -v "$OUT:/results" \
 --entrypoint bash "$ROS_IMAGE" -lc '
set -e
source /opt/ros/humble/setup.bash
bash /da/build2.sh > /results/orb-build.log 2>&1
orb=/workspaces/soarm-car/vendor/orb_slam3/ORB_SLAM3
run_orb () {
  seq=$1; yaml=$2
  echo "=== orb $seq ==="
  mkdir -p /results/$seq/orb
  /results/runner2 $orb/Vocabulary/ORBvoc.txt $orb/Examples/RGB-D/$yaml \
    /datasets/rgbd_dataset_$seq /results/$seq/orb 0 480 640 > /results/$seq/orb/run.log 2>&1
}
run_orb freiburg1_xyz TUM1.yaml
run_orb freiburg1_desk TUM1.yaml
run_orb freiburg1_room TUM1.yaml
run_orb freiburg3_long_office_household TUM3.yaml
echo PHASE2_DONE'

# --- phase 3: evaluate ORB runs + aggregate (algorithms image) ---
docker run --rm --network none --user "$(id -u):$(id -g)" -e MPLCONFIGDIR=/tmp/mpl \
 -v "$ROOT/scripts/data-acceptance:/da:ro" -v "$DATASETS:/datasets:ro" -v "$OUT:/results" \
 --entrypoint bash "$ALGO_IMAGE" -lc '
set -e
for seq in freiburg1_xyz freiburg1_desk freiburg1_room freiburg3_long_office_household; do
  python3 /da/evaluate.py orb /results/$seq/orb /datasets/rgbd_dataset_$seq/groundtruth.txt /results/$seq/orb-eval
done
python3 - <<EOF
import json
from pathlib import Path
summary = {}
for seq in ["freiburg1_xyz", "freiburg1_desk", "freiburg1_room", "freiburg3_long_office_household"]:
    d = json.loads(Path(f"/results/{seq}/dense-eval/evaluation.json").read_text())
    o = json.loads(Path(f"/results/{seq}/orb-eval/evaluation.json").read_text())
    summary[seq] = {
        "dense": {k: d[k] for k in ("edges", "accepted_rate", "segments_ge_min",
                                     "longest_segment_s", "ate_rmse_best_segment_m",
                                     "rpe_1s", "false_accept_rate")},
        "orb": {k: o[k] for k in ("frames", "ok_rate", "ate_rmse_m_all_ok",
                                  "rpe_1s", "map_event_count")},
    }
Path("/results/summary.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
EOF
echo ALL_DONE'
