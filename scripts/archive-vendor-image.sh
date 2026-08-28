#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $0 <downloaded-image-or-archive> <official-source-url>" >&2
    echo "Copies the file to artifacts/images and writes a non-secret manifest." >&2
}

if [[ $# -ne 2 ]]; then
    usage
    exit 2
fi

source_file=$1
source_url=$2
if [[ ! -f "$source_file" ]]; then
    echo "File not found: $source_file" >&2
    exit 1
fi
if [[ ! "$source_url" =~ ^https:// ]]; then
    echo "Source URL must use HTTPS." >&2
    exit 1
fi

mkdir -p artifacts/images
filename=$(basename "$source_file")
destination="artifacts/images/$filename"
if [[ -e "$destination" && "$(realpath "$source_file")" != "$(realpath "$destination")" ]]; then
    echo "Refusing to overwrite: $destination" >&2
    exit 1
fi

if [[ "$(realpath "$source_file")" != "$(realpath -m "$destination")" ]]; then
    cp --reflink=auto --preserve=timestamps "$source_file" "$destination"
fi

sha256=$(sha256sum "$destination" | awk '{print $1}')
size=$(stat --format='%s' "$destination")
timestamp=$(date --iso-8601=seconds)
manifest="artifacts/images/${filename}.manifest.txt"
cat > "$manifest" <<EOF
filename=$filename
size_bytes=$size
sha256=$sha256
source_url=$source_url
archived_at=$timestamp
hardware=WHEELTEC Raspberry Pi 5 (must be confirmed by vendor)
EOF

printf 'Archived: %s\nManifest: %s\nSHA-256: %s\n' "$destination" "$manifest" "$sha256"
