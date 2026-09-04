#!/usr/bin/env bash
set -euo pipefail

# Static and Fortress consumer smoke for the Sweet Home 3D navigation house.
python3 - <<'PY'
from collections import deque
from pathlib import Path
import math
import xml.etree.ElementTree as ET

root = Path('ros2_ws/src/soarm_simulation')
world = ET.parse(root / 'worlds/house_floorplan.sdf').getroot()
assert world.find("./world/include/uri[.='model://house_floorplan']") is not None
assert world.find("./world/include/uri[.='model://house_furniture']") is not None
assert world.find("./world/include/uri[.='model://kuruma']") is not None
model = ET.parse(root / 'models/house_floorplan/model.sdf').getroot()
collisions = model.findall('.//collision')
names = [element.get('name', '') for element in collisions]
assert sum(name.startswith('wall_') for name in names) == 30
assert sum(name.startswith('door_frame_') for name in names) == 20
assert sum(name.startswith('door_header_') for name in names) == 10
assert not any('leaf' in name or 'swing' in name for name in names)
assert len(model.findall('.//visual')) == len(collisions) == 60
door_widths = [
    float(element.findtext('geometry/box/size').split()[0])
    for element in collisions
    if element.get('name', '').startswith('door_header_')
]
assert min(door_widths) == 1.2, door_widths
assert door_widths.count(1.371) == 7, door_widths
furniture = ET.parse(root / 'models/house_furniture/model.sdf').getroot()
assert [link.get('name') for link in furniture.findall('.//link')] == ['furniture']
collision_names = [collision.get('name') for collision in furniture.findall('.//collision')]
expected_collisions = [
    f'{table}_{part}_collision{suffix}'
    for table in ('coffee_table', 'small_table', 'desk')
    for part, suffix in [('top', '')] + [('leg', f'_{index}') for index in range(4)]
]
assert collision_names == expected_collisions
assert len(furniture.findall('.//visual')) == len(collision_names) == 15
for collision in furniture.findall('.//collision'):
    visual_name = collision.get('name').replace('_collision', '_visual')
    visual = furniture.find(f".//visual[@name='{visual_name}']")
    assert visual is not None
    assert collision.findtext('pose') == visual.findtext('pose')
    assert collision.findtext('geometry/box/size') == visual.findtext('geometry/box/size')

pgm = (root / 'maps/house_floorplan.pgm').read_bytes()
header, body = pgm.split(b'\n255\n', 1)
assert header == b'P5\n480 435'
assert len(body) == 480 * 435
assert {0, 205, 254}.issubset(set(body))
yaml = (root / 'maps/house_floorplan.yaml').read_text()
assert 'resolution: 0.05' in yaml
assert 'origin: [-12.0, -10.875, 0.0]' in yaml

width, height = 480, 435
def map_value(world_x, world_y):
    column = int((world_x - -12.0) / 0.05)
    row = height - 1 - int((world_y - -10.875) / 0.05)
    return body[row * width + column]
for centre in ((-8.2457304, 0.948594), (-3.4827291, -0.306303), (8.0408715, -1.152513)):
    assert map_value(*centre) == 0, centre

occupied = {index for index, value in enumerate(body) if value == 0}
free = {index for index, value in enumerate(body) if value == 254}
# Conservatively reserve the 0.31 m footprint half-width plus 0.16 m of
# documented localization error without changing the navigation configuration.
clearance_radius = 0.47
clearance_cells = math.ceil(clearance_radius / 0.05)
inflated = set(occupied)
for index in occupied:
    row, column = divmod(index, width)
    for dr in range(-clearance_cells, clearance_cells + 1):
        for dc in range(-clearance_cells, clearance_cells + 1):
            if (math.hypot(dr, dc) * 0.05 <= clearance_radius
                    and 0 <= row + dr < height and 0 <= column + dc < width):
                inflated.add((row + dr) * width + column + dc)
traversable = free - inflated
spawn_column = int((0.0 - -12.0) / 0.05)
spawn_row = height - 1 - int((0.0 - -10.875) / 0.05)
spawn = spawn_row * width + spawn_column
assert spawn in traversable
spawn_distance = min(math.hypot(spawn_row - row, spawn_column - column) * 0.05
                     for row, column in map(lambda index: divmod(index, width), occupied))
assert spawn_distance >= 1.0, spawn_distance

seen = {spawn}
queue = deque([spawn])
while queue:
    index = queue.popleft()
    row, column = divmod(index, width)
    for next_row, next_column in ((row - 1, column), (row + 1, column),
                                  (row, column - 1), (row, column + 1)):
        candidate = next_row * width + next_column
        if (0 <= next_row < height and 0 <= next_column < width
                and candidate in traversable and candidate not in seen):
            seen.add(candidate)
            queue.append(candidate)
assert len(seen) / len(traversable) > 0.97, (len(seen), len(traversable))
def grid_index(world_x, world_y):
    column = int((world_x - -12.0) / 0.05)
    row = height - 1 - int((world_y - -10.875) / 0.05)
    return row * width + column
# Representative goals cover the living room, central routes, right rooms,
# diagonal corridor, and furniture-adjacent free space.
for goal in ((-7.5, 3.0), (-4.5, 1.2), (-1.5, -1.5), (0.0, 3.0),
             (6.0, 3.0), (6.13227, 0.84215), (7.5, -3.0), (7.5, -7.5),
             (5.5, -6.0)):
    assert grid_index(*goal) in seen, goal
PY
python3 -m py_compile \
    ros2_ws/src/soarm_simulation/tools/generate_house_floorplan.py \
    ros2_ws/src/soarm_simulation/soarm_simulation/launch/simulation.launch.py

container_name="${SOARM_TEST_CONTAINER:-soarm-house-floorplan-test-$$}"
test_image="${SOARM_TEST_IMAGE:-soarm-car-ros2-dev:latest}"
ros_domain_id="${ROS_DOMAIN_ID:-96}"
ign_partition="${IGN_PARTITION:-${container_name}-ign}"
output_dir="$(mktemp -d /tmp/house-floorplan-smoke.XXXXXX)"
cleanup() {
    docker stop "$container_name" >/dev/null 2>&1 || true
    rm -rf "$output_dir"
}
trap cleanup EXIT

docker image inspect "$test_image" >/dev/null
container="$(docker run -d --rm \
    --name "$container_name" \
    --network host \
    --ipc host \
    --device /dev/dri \
    --group-add "${VIDEO_GID:-39}" \
    --group-add "${RENDER_GID:-105}" \
    --security-opt label=disable \
    -e ROS_DOMAIN_ID="$ros_domain_id" \
    -e IGN_PARTITION="$ign_partition" \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" \
    -v "$(pwd -P):/feature:ro" \
    -w /feature \
    --entrypoint bash \
    "$test_image" -lc '
        source /opt/ros/humble/setup.bash
        source /workspaces/soarm-car/install/setup.bash
        colcon --log-base /tmp/house_floorplan/log build \
            --base-paths /feature/ros2_ws/src/soarm_simulation \
            --build-base /tmp/house_floorplan/build \
            --install-base /tmp/house_floorplan/install \
            --symlink-install
        source /tmp/house_floorplan/install/setup.bash
        ros2 launch soarm_simulation simulation.launch.py world:=house_floorplan
    ')"
ros_command='source /opt/ros/humble/setup.bash && source /workspaces/soarm-car/install/setup.bash && source /tmp/house_floorplan/install/setup.bash && ros2'
ready=false
for _ in {1..120}; do
    if docker exec "$container_name" bash -lc \
        "test -f /tmp/house_floorplan/install/setup.bash && $ros_command topic info /mid360/points | grep -q 'Publisher count: 1' && $ros_command topic info /map | grep -Eq 'Publisher count: [1-9]' && $ros_command topic info /gazebo/odom | grep -q 'Publisher count: 1'"; then
        ready=true
        break
    fi
    sleep 1
done
$ready || { docker logs "$container_name" >&2; exit 1; }
docker exec "$container_name" bash -lc "ign model --list | grep -q 'house_furniture'"
inside() { docker exec "$container_name" bash -lc "$ros_command $*"; }
inside_timeout() {
    local seconds=$1 command
    shift
    printf -v command '%q ' "$@"
    docker exec "$container_name" bash -lc \
        "source /opt/ros/humble/setup.bash && source /workspaces/soarm-car/install/setup.bash && source /tmp/house_floorplan/install/setup.bash && timeout $seconds $command"
}
inside_timeout 20 ros2 topic echo /map --once >"$output_dir/map.yaml"
inside_timeout 20 ros2 topic echo /mid360/points --once --qos-reliability best_effort >"$output_dir/points.yaml"
inside_timeout 10 ros2 topic echo /gazebo/odom --once >"$output_dir/odom-before.yaml"
inside "topic pub -r 20 --times 60 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}, angular: {z: 0.0}}'" >/dev/null
inside "topic pub -r 20 --times 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'" >/dev/null
inside_timeout 10 ros2 topic echo /gazebo/odom --once >"$output_dir/odom-after.yaml"
python3 - "$output_dir" <<'PY'
from pathlib import Path
import math
import re
import sys
root = Path(sys.argv[1])
map_text = (root / 'map.yaml').read_text()
assert 'width: 480' in map_text and 'height: 435' in map_text
points = (root / 'points.yaml').read_text()
width = re.search(r'width: (\d+)', points)
assert width and int(width.group(1)) > 100, points[:1000]
def position(path):
    text = path.read_text()
    match = re.search(r'position:\n      x: ([^\n]+)\n      y: ([^\n]+)', text)
    assert match, text
    return float(match.group(1)), float(match.group(2))
before = position(root / 'odom-before.yaml')
after = position(root / 'odom-after.yaml')
distance = math.hypot(after[0] - before[0], after[1] - before[1])
assert distance > 0.1, (before, after)
PY
printf '%s\n' "House floor-plan Fortress consumer smoke passed in isolated container $container_name."
