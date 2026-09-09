#!/usr/bin/env bash
# Run only after the Intel owner has started the restored Nav2 runtime.
set -euo pipefail
cd "$(dirname "$0")/.."
: "${EXPLORATION_EVIDENCE_DIR:?set a new evidence directory}"
[[ ! -e "$EXPLORATION_EVIDENCE_DIR" ]] || { echo "refusing existing evidence directory" >&2; exit 2; }
export NAV2_CLEAN_TASK=prior_map_assisted_exploration
exec python3 scripts/nav2-clean-route-consumer.py --ros-args \
  -p use_sim_time:=true \
  -p evidence_dir:="$EXPLORATION_EVIDENCE_DIR" \
  -p expected_health_map_id:=house_widened_reference.pcd \
  -p asset_root:="${NAV2_ASSET_ROOT:-/tmp/soarm-intel-nav-restore-20260909}" \
  "$@"
