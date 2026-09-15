#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/dabai-dc1-common.sh"
NAME=${1:?usage: $0 BAG_NAME}
[[ "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || { echo 'Invalid bag name' >&2; exit 2; }
[[ -f "$ROOT_DIR/recordings/$NAME/metadata.yaml" ]] || { echo "Completed bag not found: $NAME" >&2; exit 2; }
exec docker run --rm --network none \
    -v "$ROOT_DIR/recordings/$NAME:/bag:ro" -v "$ROOT_DIR/scripts:/dc1:ro" \
    --entrypoint bash "$DC1_IMAGE" -lc \
    'source /opt/ros/humble/setup.bash; python3 /dc1/dabai-dc1-check.py --bag /bag'
