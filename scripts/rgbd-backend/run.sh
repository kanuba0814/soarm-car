#!/usr/bin/env bash
# Frozen CPU-only external-odometry/backend experiment; all inputs read-only.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
NAME=${1:?usage: bash scripts/rgbd-backend/run.sh NEW_OUTPUT_NAME [baseline|exact]}
POLICY=${2:-baseline}
[[ "$POLICY" == baseline || "$POLICY" == exact ]] || exit 2
[[ "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || exit 2
OUT="$ROOT/artifacts/perception-dev/dabai-dc1/$NAME"
[[ ! -e "$OUT" ]] || { echo "Refusing to overwrite $OUT" >&2; exit 2; }
IMAGE=sha256:02ecfd52f40232a19f12449bce086b10969a88c7f068c6ab7feb01713529c8a1
docker image inspect "$IMAGE" >/dev/null
mkdir -p "$OUT"
python3 - "$ROOT" "$OUT" "$IMAGE" "$POLICY" <<'PY'
import hashlib, json, platform, shutil, subprocess, sys
from pathlib import Path
root, out, image, policy = sys.argv[1:]
files = list((Path(root)/'scripts/rgbd-backend').glob('*'))
manifest = dict(image=image, platform=platform.platform(),
    git_head=subprocess.check_output(['git','-C',root,'rev-parse','HEAD'],text=True).strip(),
    script_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()},
    arms=['closures','refine'], matching_policy=policy, backend_period_s=0.5, seed=17,
    notes=['No GT supplied to estimator.', 'Identity graph weights are uncalibrated.',
           'No depth filling, no new sensors, no navigation or camera access.',
           'Cross-segment closures are unverified candidates, not recovery certification.'])
(Path(out)/'manifest.json').write_text(json.dumps(manifest,indent=2))
snapshot = Path(out)/'source'
snapshot.mkdir()
for path in files:
    if path.is_file():
        shutil.copyfile(path, snapshot/path.name)
PY
docker run --rm --network none --read-only --tmpfs /tmp:rw,size=512m \
 --user "$(id -u):$(id -g)" -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
 -e MATCH_POLICY="$POLICY" \
 -e LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib \
 -v "$OUT/source:/experiment:ro" \
 -v "$ROOT/artifacts/perception-dev/dabai-dc1:/inputs:ro" \
 -v "$ROOT/datasets/tum-rgbd:/datasets:ro" -v "$OUT:/results:rw" \
 --entrypoint bash "$IMAGE" -lc '
set -euo pipefail
EXTRA=()
if [[ "$MATCH_POLICY" == exact ]]; then
 EXTRA=(Vis/CorNNType=3 Vis/CorGuessWinSize=0 Kp/NNStrategy=3)
fi
c++ -std=c++17 -O3 /experiment/runner.cc -o /results/runner \
 -I/opt/ros/humble/include/rtabmap-0.23 -I/usr/include/opencv4 -I/usr/include/eigen3 -I/usr/include/pcl-1.12 \
 -L/opt/ros/humble/lib/x86_64-linux-gnu -Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu \
 -lrtabmap_core -lrtabmap_utilite $(pkg-config --libs opencv4) > /results/build.log 2>&1
c++ -std=c++17 -O3 /experiment/export.cc -o /results/export \
 -I/opt/ros/humble/include/rtabmap-0.23 -I/usr/include/opencv4 -I/usr/include/eigen3 -I/usr/include/pcl-1.12 \
 -L/opt/ros/humble/lib/x86_64-linux-gnu -Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu \
 -lrtabmap_core -lrtabmap_utilite $(pkg-config --libs opencv4) > /results/export-build.log 2>&1
run_seq () {
 name=$1; data=$2; dense=$3; fx=$4; fy=$5; cx=$6; cy=$7; rows=$8; scale=$9; maxd=${10}
 python3 /experiment/prepare.py "$data/associations.txt" "$dense/frames.csv" "/results/$name"
 for arm in closures refine; do
  mkdir "/results/$name/$arm"
  flag=false; if [[ "$arm" == refine ]]; then flag=true; fi
  /results/runner "$data" "/results/$name/keyframes.txt" "/results/$name/$arm" \
   "$fx" "$fy" "$cx" "$cy" "$rows" "$scale" "$maxd" "$flag" "${EXTRA[@]}" > "/results/$name/$arm/run.log" 2>&1
  /results/export "/results/$name/$arm" > "/results/$name/$arm/export.log" 2>&1
  echo "DONE $name $arm"
 done
}
# Freeze the exact same settings before seeing any scores. Long sequence last.
for bag in office-20260913-212016 office-20260913-212121; do
 run_seq "$bag" "/inputs/orbslam3-validation/$bag" "/inputs/rgbd-comparison/$bag/hybrid" \
  488.09033203125 488.09033203125 320.88531494140625 211.70437622070312 400 0.001 5
done
for seq in freiburg1_xyz freiburg1_desk freiburg1_room; do
 run_seq "$seq" "/datasets/rgbd_dataset_$seq" "/inputs/a4-tum-20260916/$seq/dense" \
  517.306408 516.469215 318.643040 255.313989 480 0.0002 5
done
run_seq freiburg3_long_office_household /datasets/rgbd_dataset_freiburg3_long_office_household \
 /inputs/a4-tum-20260916/freiburg3_long_office_household/dense 535.4 539.2 320.1 247.6 480 0.0002 8
'
python3 "$OUT/source/evaluate.py" "$OUT" "$ROOT/datasets/tum-rgbd"
