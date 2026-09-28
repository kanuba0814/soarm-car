#!/usr/bin/env bash
set -euo pipefail

log_file=/var/log/soarm-firstboot.log
exec > >(tee -a "${log_file}") 2>&1

echo "[$(date --iso-8601=seconds)] Starting SOARM Pi 5 first-boot setup"

if ! id aiteam >/dev/null 2>&1; then
    echo "Waiting for Raspberry Pi OS to create the aiteam user"
    exit 1
fi

install -d -m 0700 -o aiteam -g aiteam /home/aiteam/.ssh
install -m 0600 -o aiteam -g aiteam \
    /opt/soarm-car/deploy/pi5/firstboot/aiteam-authorized-key.pub \
    /home/aiteam/.ssh/authorized_keys
chown -R aiteam:aiteam /opt/soarm-car

systemctl enable --now ssh

# Bring up the direct cable before apt/Docker access is needed.
install -m 0600 /opt/soarm-car/deploy/pi5/soarm-ethernet.nmconnection \
    /etc/NetworkManager/system-connections/soarm-ethernet.nmconnection
nmcli connection reload
nmcli connection up soarm-ethernet || true

# Desktop autologin keeps the local touchscreen usable without a keyboard.
raspi-config nonint do_boot_behaviour B4 || true

/opt/soarm-car/deploy/pi5/bootstrap-host.sh
usermod -aG docker aiteam

touch /var/lib/soarm-firstboot-complete
systemctl disable soarm-firstboot.service
echo "[$(date --iso-8601=seconds)] SOARM Pi 5 first-boot setup complete"
