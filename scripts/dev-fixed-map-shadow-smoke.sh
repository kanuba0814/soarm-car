#!/usr/bin/env bash
# Build-only/static smoke for the opt-in shadow package; it never starts simulation hardware.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT/ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --packages-up-to soarm_fixed_map_localization soarm_navigation --event-handlers console_direct+
source install/setup.bash
set -u
python3 - <<'PY'
from pathlib import Path
import yaml

launch = next(Path('install').glob('soarm_navigation/**/house_fastlio_nav2.launch.py'))
text = launch.read_text()
assert "DeclareLaunchArgument('fixed_map_shadow', default_value='false')" in text
assert "fixed_map_shadow_node" in text
assert "IfCondition(LaunchConfiguration('fixed_map_shadow'))" in text
node = next(Path('install').glob('soarm_fixed_map_localization/**/fixed_map_shadow_node'))
assert node.exists()
config_path = next(Path('install').glob('soarm_fixed_map_localization/**/config/house_shadow.yaml'))
params = yaml.safe_load(config_path.read_text())['/fixed_map_shadow']['ros__parameters']
assert params['accept_enabled'] is False
assert params['recovery_enabled'] is False
assert params['motion_odom_topic'] == '/wheel/odom_raw'
source = Path('src/soarm_fixed_map_localization/src/fixed_map_shadow_node.cpp').read_text()
assert '"/fixed_map_shadow/candidate"' in source
assert '"/fixed_map_shadow/diagnostics"' in source
for forbidden in ('"/map_to_odom"', '"/ground_truth', 'TransformBroadcaster'):
    assert forbidden not in source
print('installed fail-closed shadow launch assets verified')
PY
