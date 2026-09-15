#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
OUT_DIR=${1:-"$ROOT_DIR/artifacts/perception-dev/dabai-dc1"}
mkdir -p "$OUT_DIR"
raw="$OUT_DIR/hardware-check.txt"
json="$OUT_DIR/hardware-check.json"

have_lsusb=false
if command -v lsusb >/dev/null 2>&1; then
  have_lsusb=true
  lsusb > "$OUT_DIR/lsusb-nn.txt" || true
else
  : > "$OUT_DIR/lsusb-nn.txt"
fi
{
  printf 'captured_at=%s\n' "$(date --iso-8601=seconds)"
  printf 'hostname=%s\n' "$(hostname)"
  printf 'kernel=%s\n' "$(uname -srmo)"
  printf 'lsusb_available=%s\n' "$have_lsusb"
  printf '\n[usb]\n'
  grep -Ei 'orbbec|astra|dabai|2bc5|camera' "$OUT_DIR/lsusb-nn.txt" || true
  printf '\n[video-devices]\n'
  find /dev -maxdepth 1 -type c -name 'video*' -printf '%f\n' 2>/dev/null | sort || true
  printf '\n[udev-links]\n'
  find /dev -maxdepth 2 -type l \( -name 'astra*' -o -name 'dabai*' \) -printf '%p -> %l\n' 2>/dev/null | sort || true
} | tee "$raw"

python3 - "$raw" "$json" <<'PY'
import json, pathlib, re, sys
raw = pathlib.Path(sys.argv[1]).read_text()
usb = [line.strip() for line in raw.splitlines() if re.search(r'2bc5|orbbec|astra|dabai|camera', line, re.I)]
video = [line.strip() for line in raw.split('[video-devices]\n', 1)[-1].split('[udev-links]', 1)[0].splitlines() if line.strip()]
def bus_for(pid):
    pattern = re.compile(r'^Bus (\d+) Device (\d+):.*2bc5:%s\b' % pid, re.I)
    for line in usb:
        match = pattern.search(line)
        if match:
            return f'{match.group(1)}/{match.group(2)}'
    return None
result = {
    'captured_at': next((x.split('=',1)[1] for x in raw.splitlines() if x.startswith('captured_at=')), None),
    'usb_matches': usb,
    'rgb_bus': bus_for('0557'),
    'depth_bus': bus_for('0657'),
    'video_devices': video,
    'evidence_complete': bool(usb),
    'status': 'EVIDENCE_REQUIRED' if not usb else 'USB_ID_CAPTURED_NEEDS_ROS_CHECK',
    'next_steps': [
        'Run inside the ROS 2 container: ros2 topic list -t',
        'Inspect CameraInfo, image encoding, frame_id, timestamps and depth units',
        'Record serial, supported modes, registration and base_link-to-camera_link calibration',
    ],
}
pathlib.Path(sys.argv[2]).write_text(json.dumps(result, indent=2) + '\n')
PY

if [[ "$have_lsusb" != true || ! -s "$OUT_DIR/lsusb-nn.txt" ]]; then
  echo "No USB camera evidence captured; profile remains unchanged." >&2
  exit 2
fi
