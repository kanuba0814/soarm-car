#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
rule_source="$root/config/udev/99-soarm-astra-s.rules"
rule_target=/etc/udev/rules.d/99-soarm-astra-s.rules

if [[ ${EUID} -ne 0 ]]; then
    exec sudo "$0" "$@"
fi
install -m 0644 "$rule_source" "$rule_target"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --attr-match=idVendor=2bc5 --attr-match=idProduct=0402
printf 'Installed %s and reloaded the Astra S USB rule.\n' "$rule_target"
