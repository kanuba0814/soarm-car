#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

output="artifacts/fastlio_house_floorplan.pcd"
python3 scripts/generate_house_reference_pcd.py --output "$output"
python3 - "$output" <<'PY'
import math
import re
import struct
import sys

path = sys.argv[1]
raw = open(path, 'rb').read()
header, data = raw.split(b'DATA binary\n', 1)
points = int(re.search(rb'POINTS (\d+)', header).group(1))
assert points > 10_000, points
assert len(data) == points * 32, (len(data), points)
xyz = [struct.unpack_from('<fff', data, index * 32) for index in range(points)]
assert all(math.isfinite(value) for point in xyz for value in point)
extent = [max(point[axis] for point in xyz) - min(point[axis] for point in xyz) for axis in range(3)]
assert extent[0] > 20.0 and extent[1] > 18.0 and extent[2] > 2.0, extent
print({'points': points, 'extent_xyz': extent})
PY
printf 'Generated complete structural reference %s from the checked-in simulation SDF.\n' "$output"
