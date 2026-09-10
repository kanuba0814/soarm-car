#!/usr/bin/env bash
# Run only after the NVIDIA runtime owner has released its verified Nav2 stack.
set -euo pipefail
cd "$(dirname "$0")/.."
: "${EXPLORATION_EVIDENCE_DIR:?set a new evidence directory}"
[[ ! -e "$EXPLORATION_EVIDENCE_DIR" ]] || { echo "refusing existing evidence directory" >&2; exit 2; }
if ! python3 -c 'from soarm_interfaces.msg import LocalizationHealth' 2>/dev/null; then
  command -v colcon >/dev/null || { echo "soarm_interfaces is absent and colcon is unavailable" >&2; exit 2; }
  colcon build --packages-select soarm_interfaces --base-paths ros2_ws/src
  # Make generated interfaces visible to the adapter in this same runtime prefix.
  source install/setup.bash
fi
export NAV2_CLEAN_TASK=prior_map_assisted_exploration
exec python3 scripts/nav2-clean-route-consumer.py --ros-args \
  -p use_sim_time:=true \
  -p evidence_dir:="$EXPLORATION_EVIDENCE_DIR" \
  -p expected_health_map_id:=house_widened_reference.pcd \
  -p prior_asset_root:="${NAV2_ASSET_ROOT:-/workspaces/soarm-car}" \
  -p asset_root:="${NAV2_ASSET_ROOT:-/workspaces/soarm-car}" \
  "$@"
