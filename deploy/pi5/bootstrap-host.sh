#!/usr/bin/env bash
# Run as root on Raspberry Pi OS 64-bit desktop after cloning this repository.
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
    echo "Run with sudo: sudo $0" >&2
    exit 1
fi

repository_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source /etc/os-release
architecture=$(dpkg --print-architecture)

if [[ ${architecture} != arm64 ]]; then
    echo "This deployment targets arm64; detected ${architecture}." >&2
    exit 1
fi

apt-get update
apt-get install -y --no-install-recommends ca-certificates curl gnupg
install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/${ID}/gpg \
    | gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg
chmod a+r /etc/apt/keyrings/docker.gpg
printf '%s\n' \
    "deb [arch=${architecture} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/${ID} ${VERSION_CODENAME} stable" \
    > /etc/apt/sources.list.d/docker.list
apt-get update
apt-get install -y --no-install-recommends docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Retain vendor-matched controller and Orbbec USB access rules on the host.
# Containers receive only the controller alias and the USB bus mapping.
install -m 0644 "${repository_root}/vendor/wheeltec/udev/wheeltec_controller.rules" \
    /etc/udev/rules.d/wheeltec_controller.rules
install -m 0644 "${repository_root}/vendor/wheeltec/udev/56-orbbec-usb.rules" \
    /etc/udev/rules.d/56-orbbec-usb.rules
install -m 0644 "${repository_root}/vendor/wheeltec/udev/wheeltec_gemini.rules" \
    /etc/udev/rules.d/wheeltec_gemini.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
udevadm trigger --subsystem-match=usb

# Ethernet uses the Fedora host (10.42.0.1) for its default route and DNS.
install -m 0600 "${repository_root}/deploy/pi5/soarm-ethernet.nmconnection" \
    /etc/NetworkManager/system-connections/soarm-ethernet.nmconnection
nmcli connection reload
nmcli connection up soarm-ethernet || true

# Allows the invoking desktop user to run Docker after signing out and back in.
if [[ -n ${SUDO_USER:-} && ${SUDO_USER} != root ]]; then
    usermod -aG docker "${SUDO_USER}"
fi

docker compose -f "${repository_root}/deploy/pi5/compose.yaml" build
