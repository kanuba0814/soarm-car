#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
lock="$root/config/aws_small_house_companion.json"
destination="${AWS_SMALL_HOUSE_DIR:-$root/artifacts/baselines/aws_small_house_world}"

[[ -f "$lock" ]] || { printf 'Missing source lock: %s\n' "$lock" >&2; exit 2; }
[[ ! -e "$destination" ]] || { printf 'Refusing to overwrite existing asset: %s\n' "$destination" >&2; exit 2; }

readarray -t locked < <(python3 - "$lock" <<'PY'
import json, sys
asset = json.load(open(sys.argv[1], encoding='utf-8'))['asset']
print(asset['repository'])
print(asset['branch'])
print(asset['commit'])
print(asset['license_sha256'])
print(asset['consumed_files']['worlds/small_house.world'])
PY
)

mkdir -p "$(dirname "$destination")"
git clone --no-checkout --branch "${locked[1]}" "${locked[0]}" "$destination"
git -C "$destination" checkout --detach "${locked[2]}"
[[ "$(git -C "$destination" rev-parse HEAD)" == "${locked[2]}" ]]
[[ "$(git -C "$destination" status --porcelain)" == "" ]]
[[ "$(sha256sum "$destination/LICENSE" | awk '{print $1}')" == "${locked[3]}" ]]
[[ "$(sha256sum "$destination/worlds/small_house.world" | awk '{print $1}')" == "${locked[4]}" ]]
python3 - "$root" "$destination" "$lock" <<'PY'
import hashlib, json, sys
from pathlib import Path
root, destination, lock_path = map(Path, sys.argv[1:])
asset = json.loads(lock_path.read_text(encoding='utf-8'))['asset']
manifest_path = root / asset['content_manifest']
assert hashlib.sha256(manifest_path.read_bytes()).hexdigest() == asset['content_manifest_sha256']
manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
assert manifest['commit'] == asset['commit']
expected = manifest['files']
actual_paths = [Path('LICENSE'), Path('worlds/small_house.world')]
actual_paths.extend(sorted(path.relative_to(destination) for path in (destination / 'models').rglob('*') if path.is_file()))
assert set(expected) == {str(path) for path in actual_paths}, 'asset manifest file set mismatch'
for relative, details in expected.items():
    data = (destination / relative).read_bytes()
    assert len(data) == details['bytes'], relative
    assert hashlib.sha256(data).hexdigest() == details['sha256'], relative
print(f'Verified {len(expected)} pinned source files.')
PY
printf 'Acquired pinned AWS Small House source at %s\n' "$destination"
